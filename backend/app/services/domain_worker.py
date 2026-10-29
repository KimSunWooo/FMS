"""One robot, one ROS domain, one process.

The parent API does not change its own ROS_DOMAIN_ID. This process only
writes the status file for FMS_WORKER_ROBOT_ID. It does not publish cmd_vel
or send a Nav2 goal.
"""
from __future__ import annotations

import json
import os
import shlex
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from app.services.domain_status import build_status, status_path, write_status
from app.services.fleet_registry import RegistryError, load_registry
from app.services.physical_nav2_auto import parse_map_tf_pose

ROOT = Path(__file__).resolve().parents[3]


def observe_physical(env: dict[str, str] | None = None) -> dict:
    """Read-only discovery on the process domain. Empty discovery is not a pose."""
    ok, text = _ros(("ros2", "node", "list"), env=env, timeout=5.0)
    nodes = {line.strip() for line in text.splitlines() if line.strip().startswith("/")}
    if not ok and not nodes:
        return {
            "ros_connected": False,
            "nav2_ready": False,
            "tf_status": "missing",
            "pose": None,
            "connection_state": "error",
            "detail": "ROS discovery failed",
        }
    if not nodes:
        return {
            "ros_connected": False,
            "nav2_ready": False,
            "tf_status": "missing",
            "pose": None,
            "connection_state": "disconnected",
            "detail": "No ROS nodes in this domain",
        }
    life_ok, life = _ros(("ros2", "lifecycle", "get", "/bt_navigator"), env=env, timeout=5.0)
    nav2_active = life_ok and "active" in life
    tf_ok, tf_text = _ros(
        ("timeout", "3", "ros2", "run", "tf2_ros", "tf2_echo", "map", "base_footprint"),
        env=env,
        timeout=4.0,
    )
    pose = parse_map_tf_pose(tf_text) if tf_text else None
    tf_status = "fresh" if pose else ("lookup_failed" if tf_ok or "Invalid" in tf_text else "missing")
    return {
        "ros_connected": True,
        "nav2_ready": bool(nav2_active and pose),
        "tf_status": tf_status,
        "pose": pose,
        "connection_state": "nav2_ready" if nav2_active and pose else "ros_connected",
        "detail": f"nodes={len(nodes)} bt_navigator={'active' if nav2_active else 'inactive'}",
    }


def run_once(robot: dict, observed_domain: int, probe: dict | None = None) -> dict:
    if robot["kind"] == "physical" and robot["physical_connection_enabled"] and int(observed_domain) == int(robot["ros_domain_id"]):
        probe = observe_physical() if probe is None else probe
    return build_status(robot, observed_domain, probe)


def main() -> int:
    robot_id = os.environ.get("FMS_WORKER_ROBOT_ID", "").strip()
    fleet_path = Path(os.environ.get("FMS_PHYSICAL_FLEET_PATH") or (ROOT / "config" / "physical_fleet.json"))
    data_dir = Path(os.environ.get("FMS_DATA_DIR") or (ROOT / "data"))
    observed = int(os.environ["ROS_DOMAIN_ID"])
    interval = max(1.0, float(os.environ.get("FMS_DOMAIN_WORKER_INTERVAL_S", "3")))
    stop = False
    gateway: subprocess.Popen | None = None

    def _stop(*_args) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    while not stop:
        try:
            robots = load_registry(fleet_path)
            robot = next(item for item in robots if item["robot_id"] == robot_id)
        except (RegistryError, StopIteration, KeyError, ValueError) as exc:
            broken = {
                "robot_id": robot_id or "unknown",
                "ros_domain_id": observed,
                "connection_state": "error",
                "detail": str(exc),
                "pose": None,
                "ros_connected": False,
                "nav2_ready": False,
                "tf_status": "missing",
                "last_seen": datetime.now(timezone.utc).isoformat(),
                "worker_pid": os.getpid(),
            }
            if robot_id:
                write_status(status_path(data_dir, robot_id), broken)
            time.sleep(interval)
            continue
        status = run_once(robot, observed)
        status["worker_pid"] = os.getpid()
        status["last_seen"] = datetime.now(timezone.utc).isoformat()
        gateway = _ensure_gateway(gateway, robot, data_dir)
        status["task_transport"] = _transport_label(robot, gateway)
        status["task_execution_enabled"] = False
        status["receiver_ready"] = _receiver_ready(data_dir, robot["robot_id"])
        _merge_receiver_pose(status, data_dir, robot)
        write_status(status_path(data_dir, robot["robot_id"]), status)
        deadline = time.monotonic() + interval
        while not stop and time.monotonic() < deadline:
            time.sleep(0.1)
    if gateway is not None and gateway.poll() is None:
        gateway.terminate()
    return 0


def _ensure_gateway(current: subprocess.Popen | None, robot: dict, data_dir: Path) -> subprocess.Popen | None:
    """Start the topic gateway only for a physical robot. Virtual robots stay silent."""
    mode = os.environ.get("FMS_FLEET_DISPATCH_MODE", "ros_topic")
    physical = robot["kind"] == "physical" and robot["physical_connection_enabled"] and mode == "ros_topic"
    if not physical:
        if current is not None and current.poll() is None:
            current.terminate()
        return None
    if current is not None and current.poll() is None:
        return current
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(robot["ros_domain_id"])
    env["FMS_WORKER_ROBOT_ID"] = robot["robot_id"]
    env["FMS_DATA_DIR"] = str(data_dir)
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    return subprocess.Popen(
        ["/bin/bash", str(ROOT / "scripts" / "run_ros_gateway.sh")],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def _transport_label(robot: dict, gateway: subprocess.Popen | None) -> str:
    if robot["kind"] != "physical" or not robot["physical_connection_enabled"]:
        return "physical_transport_disabled"
    if os.environ.get("FMS_FLEET_DISPATCH_MODE", "ros_topic") != "ros_topic":
        return os.environ.get("FMS_FLEET_DISPATCH_MODE", "ros_topic")
    if gateway is not None and gateway.poll() is None:
        return "ros_topic"
    return "gateway_unavailable"


def _merge_receiver_pose(status: dict, data_dir: Path, robot: dict) -> None:
    """Prefer RobotStatus IPC pose as the primary reported location.

    Worker TF probe remains under worker_tf_* for diagnosis/cross-check.
    Registry home is never copied into pose.
    """
    from app.services.receiver_pose import interpret_receiver_pose, read_receiver_status

    robot_id = robot["robot_id"]
    domain = int(robot["ros_domain_id"])
    status["worker_tf_pose"] = status.get("pose")
    status["worker_tf_status"] = status.get("tf_status")
    ipc = read_receiver_status(data_dir, robot_id, expected_domain=domain)
    reported = interpret_receiver_pose(ipc, robot_id=robot_id, ros_domain_id=domain)
    status["pose_freshness"] = reported["pose_freshness"]
    status["pose_source"] = reported.get("pose_source")
    status["pose_valid"] = bool(reported.get("pose_valid"))
    status["pose_observed_at"] = reported.get("pose_timestamp")
    status["receiver_status_observed_at"] = reported.get("status_observed_at")
    if reported.get("tf_fresh") and isinstance(reported.get("pose"), dict):
        status["pose"] = {
            "x": reported["pose"]["x"],
            "y": reported["pose"]["y"],
            "yaw": reported["pose"].get("yaw"),
        }
        status["tf_status"] = "fresh"
        status["pose_primary"] = "robot_status"
        return
    # Primary contract path unavailable: do not publish worker TF as authoritative pose.
    status["pose"] = None
    status["pose_primary"] = None
    # Preserve worker diagnostic tf_status for Nav2 tooling.
    status["tf_status"] = status.get("worker_tf_status") or status.get("tf_status") or "missing"


def _receiver_ready(data_dir: Path, robot_id: str) -> bool:
    path = data_dir / "ros_ipc" / robot_id / "robot_status.json"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    if not (
        payload.get("receiver_ready")
        and payload.get("robot_id") == robot_id
        and payload.get("task_execution_enabled") is False
    ):
        return False
    raw = payload.get("observed_at")
    try:
        observed = datetime.fromisoformat(str(raw))
    except (TypeError, ValueError):
        return False
    if observed.tzinfo is None:
        observed = observed.replace(tzinfo=timezone.utc)
    age = (datetime.now(timezone.utc) - observed).total_seconds()
    max_age = float(os.environ.get("FMS_ROBOT_STATUS_MAX_AGE_S", "5"))
    return -1.0 <= age <= max_age


def _ros(args: tuple[str, ...], *, env: dict[str, str] | None, timeout: float) -> tuple[bool, str]:
    command = "source /opt/ros/jazzy/setup.bash && " + " ".join(shlex.quote(part) for part in args)
    try:
        result = subprocess.run(
            ["/bin/bash", "-c", command],
            env=env or os.environ.copy(),
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return result.returncode == 0, (result.stdout + "\n" + result.stderr)[:4000]
    except (subprocess.TimeoutExpired, OSError) as exc:
        output = ""
        if isinstance(exc, subprocess.TimeoutExpired):
            output = ((exc.stdout or "") + "\n" + (exc.stderr or ""))[:4000]
        return False, output or str(exc)


if __name__ == "__main__":
    raise SystemExit(main())
