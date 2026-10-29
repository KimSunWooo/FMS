"""Pose for one robot id. A sample from another robot is never reused.

Physical pose priority:
1. /amcl_pose Gateway IPC (map frame) when present — valid or last+stale
2. RobotStatus pose (legacy Receiver TF) only if amcl missing
3. Domain-worker TF sample for diagnostic cross-check / last resort
Registry home is never substituted for a physical robot's live pose.
"""
from __future__ import annotations

from app.services.domain_status import interpret_status, read_status, status_path
from app.services.fleet_registry import RegistryError, load_registry
from app.services.receiver_pose import (
    interpret_receiver_battery,
    interpret_receiver_pose,
    read_receiver_status,
)
from app.services.robot_telemetry import read_robot_telemetry
from app.services.route_preview import fresh_tf_pose

START_MATCH_M = 0.05


def robot_state(robot_id: str, settings, snapshot: dict | None = None) -> dict:
    robots = load_registry(settings.physical_fleet_path)
    robot = next((item for item in robots if item["robot_id"] == robot_id), None)
    if robot is None:
        return {
            "robot_id": robot_id,
            "error": "robot is not registered",
            "pose": None,
            "pose_source": None,
            "tf_fresh": False,
            "assignable": False,
            "pose_valid": False,
            "pose_freshness": "missing",
        }
    domain = int(robot["ros_domain_id"])
    worker = _worker(settings, robot)
    pose = None
    source = None
    observed = None
    fresh = False
    pose_valid = False
    pose_freshness = "missing"
    worker_tf_pose = None
    battery_valid = False
    battery_percentage = None
    battery_source = None
    battery_timestamp = None
    battery_freshness = "missing"
    plan = None
    plan_valid = False
    plan_source = None
    plan_timestamp = None
    plan_freshness = "missing"
    physical = robot["kind"] == "physical" and robot["physical_connection_enabled"]
    ipc = None
    if not physical:
        home = robot["home"]
        pose = {"x": home["x"], "y": home["y"], "yaw": home.get("yaw", 0.0)}
        source = "registry_home"
        pose_valid = True
        pose_freshness = "valid"
        fresh = True
        plan_freshness = "not_applicable"
    else:
        # Primary: /amcl_pose on this robot's ROS domain (Gateway IPC).
        telem = read_robot_telemetry(settings.data_dir, robot_id, domain)
        pose_freshness = telem["pose_freshness"]
        plan_valid = bool(telem.get("plan_valid"))
        plan = telem.get("plan")
        plan_source = telem.get("plan_source")
        plan_timestamp = telem.get("plan_timestamp")
        plan_freshness = telem.get("plan_freshness") or "missing"
        if isinstance(telem.get("pose"), dict) and telem.get("pose_freshness") not in {None, "missing"}:
            sample = telem["pose"]
            pose = {"x": float(sample["x"]), "y": float(sample["y"]), "yaw": sample.get("yaw")}
            source = telem.get("pose_source") or "topic:amcl_pose"
            observed = telem.get("pose_timestamp")
            pose_valid = bool(telem.get("pose_valid"))
            fresh = pose_valid
        # Secondary: RobotStatus pose (legacy Receiver TF path) if amcl absent.
        ipc = read_receiver_status(settings.data_dir, robot_id, expected_domain=domain)
        if pose is None:
            reported = interpret_receiver_pose(ipc, robot_id=robot_id, ros_domain_id=domain)
            pose_freshness = reported["pose_freshness"]
            if reported["tf_fresh"] and isinstance(reported.get("pose"), dict):
                sample = reported["pose"]
                pose = {"x": float(sample["x"]), "y": float(sample["y"]), "yaw": sample.get("yaw")}
                source = f"robot_status:{reported.get('pose_source') or 'tf:map->base_footprint'}"
                observed = reported.get("pose_timestamp")
                fresh = True
                pose_valid = True
        # Worker TF remains diagnostic / cross-check.
        diagnostic = None
        if worker is not None and worker.get("robot_id") == robot_id and int(worker.get("ros_domain_id", -1)) == domain:
            raw_diag = worker.get("worker_tf_pose") if isinstance(worker.get("worker_tf_pose"), dict) else None
            if raw_diag is None and worker.get("tf_status") == "fresh" and isinstance(worker.get("pose"), dict):
                raw_diag = worker.get("pose")
            if (
                raw_diag is not None
                and raw_diag.get("x") is not None
                and (worker.get("worker_tf_status") == "fresh" or worker.get("tf_status") == "fresh")
            ):
                diagnostic = {
                    "x": float(raw_diag["x"]),
                    "y": float(raw_diag["y"]),
                    "yaw": raw_diag.get("yaw"),
                    "source": f"domain-{domain}:worker-tf",
                    "observed_at": worker.get("last_seen"),
                }
                worker_tf_pose = diagnostic
        if diagnostic is not None:
            if pose is None:
                pose = {"x": diagnostic["x"], "y": diagnostic["y"], "yaw": diagnostic["yaw"]}
                source = diagnostic["source"]
                observed = diagnostic["observed_at"]
                fresh = True
                pose_valid = True
                pose_freshness = "valid"
            elif pose_valid and _pose_mismatch(pose, diagnostic):
                pose_freshness = "cross_check_mismatch"
        owned_snapshot = snapshot if isinstance(snapshot, dict) and snapshot.get("robot_id") == robot_id else None
        if pose is None and owned_snapshot is not None:
            live = fresh_tf_pose(owned_snapshot)
            if live is not None:
                pose = {"x": live["x"], "y": live["y"]}
                source = live["source"]
                observed = live.get("observed_at")
                fresh = True
                pose_valid = True
                pose_freshness = "valid"
        # Battery: /battery/percent primary (incl. last+stale); RobotStatus only if absent.
        if telem.get("battery_percentage") is not None and telem.get("battery_freshness") not in {None, "missing"}:
            battery_percentage = telem["battery_percentage"]
            battery_source = telem.get("battery_source")
            battery_timestamp = telem.get("battery_timestamp")
            battery_freshness = telem.get("battery_freshness") or "valid"
            battery_valid = bool(telem.get("battery_valid"))
        else:
            batt = interpret_receiver_battery(ipc, robot_id=robot_id, ros_domain_id=domain)
            battery_freshness = batt["battery_freshness"]
            if batt.get("battery_valid"):
                battery_valid = True
                battery_percentage = batt["battery_percentage"]
                battery_source = batt.get("battery_source")
                battery_timestamp = batt.get("battery_timestamp")
    return {
        "robot_id": robot_id,
        "ros_domain_id": domain,
        "kind": robot["kind"],
        "physical_connection_enabled": bool(robot["physical_connection_enabled"]),
        "pose": pose,
        "pose_source": source,
        "pose_timestamp": observed,
        "pose_valid": pose_valid and pose is not None,
        "pose_freshness": pose_freshness,
        "tf_fresh": fresh,
        "worker_tf_pose": worker_tf_pose,
        "plan": plan if physical else None,
        "plan_valid": bool(plan_valid) if physical else False,
        "plan_source": plan_source if physical else None,
        "plan_timestamp": plan_timestamp if physical else None,
        "plan_freshness": plan_freshness if physical else "not_applicable",
        "battery_valid": battery_valid,
        "battery_percentage": battery_percentage,
        "battery_source": battery_source,
        "battery_timestamp": battery_timestamp,
        "battery_freshness": battery_freshness,
        "nav2_status": None if worker is None else worker.get("nav2_ready"),
        "connection_status": None if worker is None else worker.get("connection_state"),
        "receiver_ready": None if worker is None else worker.get("receiver_ready"),
        "map_revision": None,
        "localization_status": "amcl" if (fresh and source and "amcl" in str(source)) else (
            "tf" if fresh else ("registry_home" if source == "registry_home" else "unavailable")
        ),
        "assignable": (not physical) or fresh,
    }


def _pose_mismatch(a: dict, b: dict, *, limit_m: float = 0.25) -> bool:
    try:
        dx = float(a["x"]) - float(b["x"])
        dy = float(a["y"]) - float(b["y"])
    except (KeyError, TypeError, ValueError):
        return False
    return (dx * dx + dy * dy) ** 0.5 > limit_m


def _worker(settings, robot: dict) -> dict | None:
    try:
        raw = read_status(status_path(settings.data_dir, robot["robot_id"]))
    except RegistryError:
        return None
    if raw is None:
        return None
    return interpret_status(raw, robot)
