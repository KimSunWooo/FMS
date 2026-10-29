"""Pose for one robot id. A sample from another robot is never reused."""
from __future__ import annotations

from app.services.domain_status import interpret_status, read_status, status_path
from app.services.fleet_registry import RegistryError, load_registry
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
        }
    domain = int(robot["ros_domain_id"])
    worker = _worker(settings, robot)
    pose = None
    source = None
    observed = None
    fresh = False
    physical = robot["kind"] == "physical" and robot["physical_connection_enabled"]
    if not physical:
        home = robot["home"]
        pose = {"x": home["x"], "y": home["y"], "yaw": home.get("yaw", 0.0)}
        source = "registry_home"
    else:
        if (
            worker is not None
            and worker.get("robot_id") == robot_id
            and int(worker.get("ros_domain_id", -1)) == domain
            and worker.get("tf_status") == "fresh"
            and isinstance(worker.get("pose"), dict)
            and worker["pose"].get("x") is not None
        ):
            sample = worker["pose"]
            pose = {"x": float(sample["x"]), "y": float(sample["y"]), "yaw": sample.get("yaw")}
            source = f"domain-{domain}:worker-tf"
            observed = worker.get("last_seen")
            fresh = True
        owned_snapshot = snapshot if isinstance(snapshot, dict) and snapshot.get("robot_id") == robot_id else None
        if pose is None and owned_snapshot is not None:
            live = fresh_tf_pose(owned_snapshot)
            if live is not None:
                pose = {"x": live["x"], "y": live["y"]}
                source = live["source"]
                observed = live.get("observed_at")
                fresh = True
    return {
        "robot_id": robot_id,
        "ros_domain_id": domain,
        "kind": robot["kind"],
        "physical_connection_enabled": bool(robot["physical_connection_enabled"]),
        "pose": pose,
        "pose_source": source,
        "pose_timestamp": observed,
        "tf_fresh": fresh,
        "nav2_status": None if worker is None else worker.get("nav2_ready"),
        "connection_status": None if worker is None else worker.get("connection_state"),
        "map_revision": None,
        "localization_status": "tf" if fresh else ("registry_home" if source == "registry_home" else "unavailable"),
        "assignable": (not physical) or fresh,
    }


def _worker(settings, robot: dict) -> dict | None:
    try:
        raw = read_status(status_path(settings.data_dir, robot["robot_id"]))
    except RegistryError:
        return None
    if raw is None:
        return None
    return interpret_status(raw, robot)
