"""Read RobotStatus pose from ros_ipc. Never invents a physical location."""
from __future__ import annotations

import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# Pose freshness is tighter than worker heartbeat (STALE_S=12).
POSE_MAX_AGE_S = float(os.environ.get("FMS_ROBOT_POSE_MAX_AGE_S", "5"))
STATUS_MAX_AGE_S = float(os.environ.get("FMS_ROBOT_STATUS_MAX_AGE_S", "5"))
# Battery sensors on Pinky refresh ~5s (bringup) or slower; allow a wider window.
BATTERY_MAX_AGE_S = float(os.environ.get("FMS_ROBOT_BATTERY_MAX_AGE_S", "30"))


def ipc_status_path(data_dir: Path, robot_id: str) -> Path:
    return Path(data_dir) / "ros_ipc" / robot_id / "robot_status.json"


def read_receiver_status(data_dir: Path, robot_id: str, *, expected_domain: int | None = None) -> dict[str, Any] | None:
    path = ipc_status_path(data_dir, robot_id)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    if payload.get("robot_id") != robot_id:
        return None
    if expected_domain is not None and int(payload.get("ros_domain_id", -1)) != int(expected_domain):
        return None
    return payload


def interpret_receiver_pose(
    payload: dict[str, Any] | None,
    *,
    robot_id: str,
    ros_domain_id: int,
    now: datetime | None = None,
    pose_max_age_s: float = POSE_MAX_AGE_S,
    status_max_age_s: float = STATUS_MAX_AGE_S,
) -> dict[str, Any]:
    """Classify RobotStatus pose for assignable/physical use.

    Returns pose=None unless pose_valid, structurally sound, and fresh.
    Reasons: missing, stale, frame_mismatch, lookup_failed, invalid_data, valid.
    """
    now = now or datetime.now(timezone.utc)
    empty = {
        "robot_id": robot_id,
        "ros_domain_id": ros_domain_id,
        "pose": None,
        "pose_valid": False,
        "pose_source": None,
        "pose_timestamp": None,
        "pose_freshness": "missing",
        "status_observed_at": None,
        "tf_fresh": False,
    }
    if payload is None:
        return empty
    if payload.get("robot_id") != robot_id or int(payload.get("ros_domain_id", -1)) != int(ros_domain_id):
        return {**empty, "pose_freshness": "invalid_data"}

    status_observed = _parse_time(payload.get("observed_at") or payload.get("gateway_received_at") or payload.get("stamp"))
    empty["status_observed_at"] = None if status_observed is None else status_observed.isoformat()
    if status_observed is None:
        return {**empty, "pose_freshness": "missing"}
    status_age = (now - status_observed).total_seconds()
    if status_age < -1.0 or status_age > status_max_age_s:
        return {**empty, "pose_freshness": "stale", "status_observed_at": status_observed.isoformat()}

    if not payload.get("pose_valid"):
        reason = str(payload.get("pose_reason") or "missing")
        if reason in {"lookup_failed", "frame_mismatch", "stale", "invalid_data", "out_of_map", "missing_stamp"}:
            freshness = reason if reason != "out_of_map" else "invalid_data"
        else:
            freshness = "missing"
        return {**empty, "pose_freshness": freshness, "status_observed_at": status_observed.isoformat()}

    pose = payload.get("pose")
    if not isinstance(pose, dict):
        return {**empty, "pose_freshness": "invalid_data", "status_observed_at": status_observed.isoformat()}
    if str(pose.get("frame_id") or "map") != "map":
        return {**empty, "pose_freshness": "frame_mismatch", "status_observed_at": status_observed.isoformat()}

    try:
        x = float(pose["x"])
        y = float(pose["y"])
        z = float(pose.get("z") or 0.0)
        yaw = pose.get("yaw")
        yaw_f = None if yaw is None else float(yaw)
    except (TypeError, ValueError, KeyError):
        return {**empty, "pose_freshness": "invalid_data", "status_observed_at": status_observed.isoformat()}
    if not all(math.isfinite(v) for v in (x, y, z)):
        return {**empty, "pose_freshness": "invalid_data", "status_observed_at": status_observed.isoformat()}
    if yaw_f is not None and not math.isfinite(yaw_f):
        return {**empty, "pose_freshness": "invalid_data", "status_observed_at": status_observed.isoformat()}

    pose_observed = _parse_time(payload.get("pose_observed_at"))
    if pose_observed is None:
        return {**empty, "pose_freshness": "missing_stamp", "status_observed_at": status_observed.isoformat()}
    pose_age = (now - pose_observed).total_seconds()
    # A fresh RobotStatus carrying an old pose is still stale for assignment.
    if pose_age < -1.0 or pose_age > pose_max_age_s:
        return {
            **empty,
            "pose_freshness": "stale",
            "status_observed_at": status_observed.isoformat(),
            "pose_timestamp": pose_observed.isoformat(),
        }

    source = str(payload.get("pose_source") or "") or "robot_status"
    return {
        "robot_id": robot_id,
        "ros_domain_id": ros_domain_id,
        "pose": {"x": x, "y": y, "z": z, "yaw": yaw_f},
        "pose_valid": True,
        "pose_source": source,
        "pose_timestamp": pose_observed.isoformat(),
        "pose_freshness": "valid",
        "status_observed_at": status_observed.isoformat(),
        "tf_fresh": True,
        "orientation": pose.get("orientation"),
    }


def interpret_receiver_battery(
    payload: dict[str, Any] | None,
    *,
    robot_id: str,
    ros_domain_id: int,
    now: datetime | None = None,
    battery_max_age_s: float = BATTERY_MAX_AGE_S,
    status_max_age_s: float = STATUS_MAX_AGE_S,
) -> dict[str, Any]:
    """Classify RobotStatus battery independently from pose."""
    now = now or datetime.now(timezone.utc)
    empty = {
        "robot_id": robot_id,
        "ros_domain_id": ros_domain_id,
        "battery_valid": False,
        "battery_percentage": None,
        "battery_source": None,
        "battery_timestamp": None,
        "battery_freshness": "missing",
    }
    if payload is None:
        return empty
    if payload.get("robot_id") != robot_id or int(payload.get("ros_domain_id", -1)) != int(ros_domain_id):
        return {**empty, "battery_freshness": "invalid_data"}

    status_observed = _parse_time(payload.get("observed_at") or payload.get("gateway_received_at") or payload.get("stamp"))
    if status_observed is None:
        return {**empty, "battery_freshness": "missing"}
    status_age = (now - status_observed).total_seconds()
    if status_age < -1.0 or status_age > status_max_age_s:
        return {**empty, "battery_freshness": "stale"}

    if not payload.get("battery_valid"):
        reason = str(payload.get("battery_reason") or "missing")
        freshness = reason if reason in {"stale", "invalid_data", "missing_stamp", "voltage_only_or_missing", "i2c_unavailable"} else "missing"
        return {**empty, "battery_freshness": freshness}

    try:
        percentage = float(payload["battery_percentage"])
    except (TypeError, ValueError, KeyError):
        return {**empty, "battery_freshness": "invalid_data"}
    if not math.isfinite(percentage) or percentage < 0.0 or percentage > 1.0:
        return {**empty, "battery_freshness": "invalid_data"}

    battery_observed = _parse_time(payload.get("battery_observed_at"))
    if battery_observed is None:
        return {**empty, "battery_freshness": "missing_stamp"}
    battery_age = (now - battery_observed).total_seconds()
    if battery_age < -1.0 or battery_age > battery_max_age_s:
        return {
            **empty,
            "battery_freshness": "stale",
            "battery_timestamp": battery_observed.isoformat(),
        }
    return {
        "robot_id": robot_id,
        "ros_domain_id": ros_domain_id,
        "battery_valid": True,
        "battery_percentage": percentage,
        "battery_source": str(payload.get("battery_source") or "") or "robot_status",
        "battery_timestamp": battery_observed.isoformat(),
        "battery_freshness": "valid",
    }


def _parse_time(raw: Any) -> datetime | None:
    if raw is None or raw == "":
        return None
    try:
        value = datetime.fromisoformat(str(raw).replace("Z", "+00:00"))
    except (TypeError, ValueError):
        return None
    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    return value
