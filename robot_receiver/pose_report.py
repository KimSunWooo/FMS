"""Validate and serialize RobotStatus pose fields without inventing a location.

TF lookup lives in ros_task_node. This module is safe to unit-test without rclpy.
"""
from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Any


POSE_SOURCE_TF = "tf:map->base_footprint"
MAP_FRAME = "map"
DEFAULT_POSE_MAX_AGE_S = 2.0


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_is_valid(x: float, y: float, z: float, w: float) -> bool:
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        return False
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return 0.5 <= norm <= 1.5


def coordinates_are_finite(x: float, y: float, z: float = 0.0) -> bool:
    return all(math.isfinite(value) for value in (x, y, z))


def pose_in_map_bounds(x: float, y: float, *, width_m: float = 1.86, height_m: float = 2.36) -> bool:
    # Official 1f_map_f is about 186×236 cm with origin near (0, 0).
    margin = 0.5
    return (-margin) <= x <= (width_m + margin) and (-margin) <= y <= (height_m + margin)


def evaluate_pose_sample(
    *,
    frame_id: str,
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    observed_at: datetime | None,
    now: datetime | None = None,
    max_age_s: float = DEFAULT_POSE_MAX_AGE_S,
    require_map_bounds: bool = True,
    age_basis: str = "wall",
) -> dict[str, Any]:
    """Return a validated pose report. Never treats (0,0,0) as success by itself.

    age_basis:
      - wall: compare observed_at to UTC wall clock (Gateway / FMS freshness)
      - ros: age already checked against ROS Time; do not re-compare to wall clock
      - skip: structural checks only
    """
    now = now or datetime.now(timezone.utc)
    if observed_at is not None and observed_at.tzinfo is None:
        observed_at = observed_at.replace(tzinfo=timezone.utc)
    if str(frame_id or "") != MAP_FRAME:
        return _invalid("frame_mismatch")
    if not coordinates_are_finite(x, y, z):
        return _invalid("invalid_data")
    if not quaternion_is_valid(qx, qy, qz, qw):
        return _invalid("invalid_data")
    if require_map_bounds and not pose_in_map_bounds(x, y):
        return _invalid("out_of_map")
    if observed_at is None:
        return _invalid("missing_stamp")
    age = None
    if age_basis == "wall":
        age = (now - observed_at).total_seconds()
        # Small negative skew only; ROS Time must not be treated as UTC here.
        if age < -1.0 or age > max_age_s:
            return _invalid("stale", age_s=age)
    elif age_basis not in {"ros", "skip"}:
        return _invalid("invalid_data")
    # For ros-basis samples, record wall-clock receive time for FMS IPC freshness.
    wall_observed = observed_at if age_basis == "wall" else now
    return {
        "pose_valid": True,
        "pose_source": POSE_SOURCE_TF,
        "reason": "ok",
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "yaw": yaw_from_quaternion(qx, qy, qz, qw),
        "orientation": {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
        "pose_observed_at": wall_observed.isoformat(),
        "age_s": age,
        "age_basis": age_basis,
    }


def status_pose_payload(report: dict[str, Any] | None) -> dict[str, Any]:
    """IPC / API-facing pose fields. Invalid reports expose null coordinates."""
    if not report or not report.get("pose_valid"):
        return {
            "pose_valid": False,
            "pose_source": "",
            "pose_reason": None if report is None else report.get("reason"),
            "pose": None,
            "pose_observed_at": None,
        }
    return {
        "pose_valid": True,
        "pose_source": report["pose_source"],
        "pose_reason": "ok",
        "pose": {
            "x": report["x"],
            "y": report["y"],
            "z": report["z"],
            "yaw": report["yaw"],
            "orientation": report["orientation"],
            "frame_id": MAP_FRAME,
        },
        "pose_observed_at": report["pose_observed_at"],
    }


def _invalid(reason: str, **extra: Any) -> dict[str, Any]:
    body = {
        "pose_valid": False,
        "pose_source": "",
        "reason": reason,
        "x": None,
        "y": None,
        "z": None,
        "yaw": None,
        "orientation": None,
        "pose_observed_at": None,
    }
    body.update(extra)
    return body
