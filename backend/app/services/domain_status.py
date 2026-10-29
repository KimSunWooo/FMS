"""Per-robot domain worker status. No ROS imports."""
from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path

from app.services.fleet_registry import RegistryError, load_registry

STALE_S = 12.0


def status_dir(data_dir: Path) -> Path:
    return Path(data_dir) / "domain_workers"


def status_path(data_dir: Path, robot_id: str) -> Path:
    if not robot_id or any(char not in "abcdefghijklmnopqrstuvwxyz0123456789-" for char in robot_id):
        raise RegistryError(f"Refusing a worker status path for {robot_id!r}")
    return status_dir(data_dir) / f"{robot_id}.json"


def write_status(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def read_status(path: Path) -> dict | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def resolve_robot_domain(robot_id: str, settings) -> int:
    path = getattr(settings, "physical_fleet_path", None)
    if path:
        for robot in load_registry(Path(path)):
            if robot["robot_id"] == robot_id:
                return int(robot["ros_domain_id"])
    domain = getattr(settings, "ros_domain_id", None)
    if domain is None:
        raise RegistryError(f"No ROS domain configured for {robot_id}")
    return int(domain)


def build_status(robot: dict, observed_domain: int, probe: dict | None = None) -> dict:
    """One robot's observation. A probe is ignored unless this robot owns the domain."""
    status = {
        "robot_id": robot["robot_id"],
        "ros_domain_id": robot["ros_domain_id"],
        "kind": robot["kind"],
        "physical_connection_enabled": robot["physical_connection_enabled"],
        "planned_physical": robot["planned_physical"],
        "ssh_connected": False,
        "ros_connected": False,
        "nav2_ready": False,
        "tf_status": "not_connected",
        "pose": None,
        "map_matches_fms": None,
        "connection_state": "physical_connection_not_configured",
        "detail": "Physical connection not configured",
        "worker_pid": None,
        "observed_domain_id": observed_domain,
        "last_seen": None,
    }
    if int(observed_domain) != int(robot["ros_domain_id"]):
        status["connection_state"] = "error"
        status["detail"] = (
            f"worker ROS_DOMAIN_ID {observed_domain} does not match "
            f"{robot['robot_id']} domain {robot['ros_domain_id']}"
        )
        return status
    if robot["kind"] != "physical" or not robot["physical_connection_enabled"]:
        return status
    status["connection_state"] = "disconnected"
    status["detail"] = "Physical connection enabled; ROS sample not available"
    status["tf_status"] = "missing"
    if not probe:
        return status
    status["ros_connected"] = bool(probe.get("ros_connected"))
    status["nav2_ready"] = bool(probe.get("nav2_ready"))
    status["tf_status"] = str(probe.get("tf_status") or "missing")
    status["connection_state"] = str(probe.get("connection_state") or "disconnected")
    status["detail"] = str(probe.get("detail") or "")
    pose = probe.get("pose")
    if status["tf_status"] == "fresh" and isinstance(pose, dict):
        status["pose"] = {
            "x": pose.get("x"),
            "y": pose.get("y"),
            "yaw": pose.get("yaw"),
        }
    return status


def interpret_status(raw: dict | None, robot: dict, *, now: datetime | None = None) -> dict:
    """Drop a stale or foreign sample. Virtual robots never keep a TF pose."""
    now = now or datetime.now(timezone.utc)
    if raw is None or raw.get("robot_id") != robot["robot_id"] or raw.get("ros_domain_id") != robot["ros_domain_id"]:
        return build_status(robot, robot["ros_domain_id"]) | {
            "connection_state": "disconnected",
            "detail": "worker status missing",
            "worker_running": False,
        }
    status = dict(raw)
    status["pose"] = None if not robot["physical_connection_enabled"] else status.get("pose")
    if not robot["physical_connection_enabled"]:
        status["ros_connected"] = False
        status["nav2_ready"] = False
        status["ssh_connected"] = False
        status["tf_status"] = "not_connected"
        if status.get("connection_state") != "error":
            status["connection_state"] = "physical_connection_not_configured"
    seen = status.get("last_seen")
    try:
        age = (now - datetime.fromisoformat(str(seen))).total_seconds()
    except (TypeError, ValueError):
        age = float("inf")
    if not 0 <= age <= STALE_S:
        status["pose"] = None
        status["ros_connected"] = False
        status["nav2_ready"] = False
        status["tf_status"] = "expired"
        status["connection_state"] = "disconnected"
        status["detail"] = "worker heartbeat expired"
    status["worker_running"] = status.get("worker_running", True)
    return status
