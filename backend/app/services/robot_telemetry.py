"""Read per-robot amcl_pose / plan / battery/percent IPC written by the Gateway."""
from __future__ import annotations

import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

POSE_MAX_AGE_S = float(os.environ.get("FMS_AMCL_POSE_MAX_AGE_S", "5"))
PLAN_MAX_AGE_S = float(os.environ.get("FMS_PLAN_MAX_AGE_S", "30"))
BATTERY_MAX_AGE_S = float(os.environ.get("FMS_BATTERY_PERCENT_MAX_AGE_S", "30"))


def telemetry_dir(data_dir: Path, robot_id: str) -> Path:
    return Path(data_dir) / "ros_ipc" / robot_id


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


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


def _owned(payload: dict[str, Any] | None, *, robot_id: str, ros_domain_id: int) -> dict[str, Any] | None:
    if payload is None:
        return None
    if payload.get("robot_id") != robot_id:
        return None
    if int(payload.get("ros_domain_id", -1)) != int(ros_domain_id):
        return None
    return payload


def interpret_amcl_pose(
    payload: dict[str, Any] | None,
    *,
    robot_id: str,
    ros_domain_id: int,
    now: datetime | None = None,
    max_age_s: float = POSE_MAX_AGE_S,
) -> dict[str, Any]:
    now = now or datetime.now(timezone.utc)
    empty = {
        "pose": None,
        "pose_valid": False,
        "pose_source": None,
        "pose_timestamp": None,
        "pose_freshness": "missing",
    }
    payload = _owned(payload, robot_id=robot_id, ros_domain_id=ros_domain_id)
    if payload is None:
        return empty
    received = _parse_time(payload.get("gateway_received_at"))
    observed = _parse_time(payload.get("pose_observed_at")) or received
    if not payload.get("pose_valid") or not isinstance(payload.get("pose"), dict):
        reason = str(payload.get("pose_reason") or "invalid_data")
        return {**empty, "pose_freshness": reason if reason != "ok" else "invalid_data"}
    if observed is None:
        return {**empty, "pose_freshness": "missing_stamp"}
    pose = payload["pose"]
    try:
        x, y = float(pose["x"]), float(pose["y"])
        yaw = pose.get("yaw")
        yaw_f = None if yaw is None else float(yaw)
    except (TypeError, ValueError, KeyError):
        return {**empty, "pose_freshness": "invalid_data"}
    if not all(math.isfinite(v) for v in (x, y)):
        return {**empty, "pose_freshness": "invalid_data"}
    last = {
        "pose": {"x": x, "y": y, "z": float(pose.get("z") or 0.0), "yaw": yaw_f},
        "pose_source": payload.get("pose_source") or "topic:amcl_pose",
        "pose_timestamp": observed.isoformat(),
    }
    age = (now - observed).total_seconds()
    # Prefer pose stamp; if ROS Time≠wall, fall back to gateway receive age.
    if age < -1.0 or age > max_age_s:
        if received is not None:
            recv_age = (now - received).total_seconds()
            if 0 <= recv_age <= max_age_s:
                observed = received
                age = recv_age
                last["pose_timestamp"] = observed.isoformat()
            else:
                # Keep last coordinates; mark stale so UI can show aged state.
                return {**last, "pose_valid": False, "pose_freshness": "stale"}
        else:
            return {**last, "pose_valid": False, "pose_freshness": "stale"}
    return {**last, "pose_valid": True, "pose_freshness": "valid"}


def interpret_plan(
    payload: dict[str, Any] | None,
    *,
    robot_id: str,
    ros_domain_id: int,
    now: datetime | None = None,
    max_age_s: float = PLAN_MAX_AGE_S,
) -> dict[str, Any]:
    now = now or datetime.now(timezone.utc)
    empty = {
        "plan": None,
        "plan_valid": False,
        "plan_source": None,
        "plan_timestamp": None,
        "plan_freshness": "missing",
        "pose_count": 0,
    }
    payload = _owned(payload, robot_id=robot_id, ros_domain_id=ros_domain_id)
    if payload is None:
        return empty
    received = _parse_time(payload.get("gateway_received_at"))
    observed = _parse_time(payload.get("plan_observed_at")) or received
    if not payload.get("plan_valid"):
        return {**empty, "plan_freshness": str(payload.get("plan_reason") or "invalid_data")}
    if observed is None:
        return {**empty, "plan_freshness": "missing_stamp"}
    poses = payload.get("poses") if isinstance(payload.get("poses"), list) else []
    last = {
        "plan": {
            "frame_id": payload.get("frame_id") or "map",
            "poses": poses,
            "pose_count": len(poses),
        },
        "plan_source": payload.get("plan_source") or "topic:plan",
        "plan_timestamp": observed.isoformat(),
        "pose_count": len(poses),
    }
    age = (now - observed).total_seconds()
    if age < -1.0 or age > max_age_s:
        if received is None or not (0 <= (now - received).total_seconds() <= max_age_s):
            return {**last, "plan_valid": False, "plan_freshness": "stale"}
        observed = received
        last["plan_timestamp"] = observed.isoformat()
    return {**last, "plan_valid": True, "plan_freshness": "valid"}


def interpret_battery_percent(
    payload: dict[str, Any] | None,
    *,
    robot_id: str,
    ros_domain_id: int,
    now: datetime | None = None,
    max_age_s: float = BATTERY_MAX_AGE_S,
) -> dict[str, Any]:
    now = now or datetime.now(timezone.utc)
    empty = {
        "battery_valid": False,
        "battery_percentage": None,
        "battery_source": None,
        "battery_timestamp": None,
        "battery_freshness": "missing",
    }
    payload = _owned(payload, robot_id=robot_id, ros_domain_id=ros_domain_id)
    if payload is None:
        return empty
    observed = _parse_time(payload.get("battery_observed_at") or payload.get("gateway_received_at"))
    if not payload.get("battery_valid"):
        return {**empty, "battery_freshness": str(payload.get("battery_reason") or "invalid_data")}
    if observed is None:
        return {**empty, "battery_freshness": "missing_stamp"}
    try:
        percentage = float(payload["battery_percentage"])
    except (TypeError, ValueError, KeyError):
        return {**empty, "battery_freshness": "invalid_data"}
    if not math.isfinite(percentage) or percentage < 0.0 or percentage > 1.0:
        return {**empty, "battery_freshness": "invalid_data"}
    last = {
        "battery_percentage": percentage,
        "battery_source": payload.get("battery_source") or "topic:battery/percent",
        "battery_timestamp": observed.isoformat(),
    }
    age = (now - observed).total_seconds()
    if age < -1.0 or age > max_age_s:
        return {**last, "battery_valid": False, "battery_freshness": "stale"}
    return {**last, "battery_valid": True, "battery_freshness": "valid"}


def read_robot_telemetry(data_dir: Path, robot_id: str, ros_domain_id: int, *, now: datetime | None = None) -> dict[str, Any]:
    root = telemetry_dir(data_dir, robot_id)
    pose = interpret_amcl_pose(_read_json(root / "amcl_pose.json"), robot_id=robot_id, ros_domain_id=ros_domain_id, now=now)
    plan = interpret_plan(_read_json(root / "plan.json"), robot_id=robot_id, ros_domain_id=ros_domain_id, now=now)
    battery = interpret_battery_percent(
        _read_json(root / "battery_percent.json"), robot_id=robot_id, ros_domain_id=ros_domain_id, now=now,
    )
    return {**pose, **plan, **battery, "robot_id": robot_id, "ros_domain_id": ros_domain_id}
