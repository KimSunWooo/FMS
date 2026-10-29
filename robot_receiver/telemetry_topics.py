"""Pinky → FMS status telemetry topic contract.

Canonical topics (per-robot ROS domain; same names on every robot):
  /amcl_pose        geometry_msgs/PoseWithCovarianceStamped
  /plan             nav_msgs/Path
  /battery/percent  std_msgs/Float32   (Pinky bringup: 0..100)

QoS for all three: RELIABLE / VOLATILE / KEEP_LAST / depth 10.

These are manufacturer/Nav2 topics. FMS must not republish the same data on a
new FMS topic. TaskCommand / ACK contracts are unchanged.
"""
from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Any

TOPIC_AMCL_POSE = "/amcl_pose"
TOPIC_PLAN = "/plan"
TOPIC_BATTERY_PERCENT = "/battery/percent"

TELEMETRY_QOS = {
    "reliability": "reliable",
    "durability": "volatile",
    "history": "keep_last",
    "depth": 10,
}

MSG_AMCL_POSE = "geometry_msgs/msg/PoseWithCovarianceStamped"
MSG_PLAN = "nav_msgs/msg/Path"
MSG_BATTERY_PERCENT = "std_msgs/msg/Float32"

# Pinky bringup Float32 is 0..100. Normalize once to 0..1 for FMS storage.
BATTERY_INPUT_SCALE = "0_100"


def amcl_pose_to_dict(msg, *, robot_id: str, ros_domain_id: int, received_at: datetime | None = None) -> dict[str, Any]:
    """Extract map pose. Does not treat this as a relative start offset."""
    received_at = received_at or datetime.now(timezone.utc)
    header = msg.header
    frame_id = str(header.frame_id or "")
    # PoseWithCovarianceStamped: msg.pose is PoseWithCovariance; .pose is Pose.
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    x = float(position.x)
    y = float(position.y)
    z = float(position.z)
    qx = float(orientation.x)
    qy = float(orientation.y)
    qz = float(orientation.z)
    qw = float(orientation.w)
    stamp = _stamp_iso(header.stamp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    finite = all(math.isfinite(v) for v in (x, y, z, qx, qy, qz, qw))
    quat_ok = 0.5 <= math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) <= 1.5
    if frame_id and frame_id != "map":
        valid = False
        reason = "frame_mismatch"
    elif not finite or not quat_ok:
        valid = False
        reason = "invalid_data"
    else:
        valid = True
        reason = "ok"
        if not frame_id:
            frame_id = "map"
    return {
        "robot_id": robot_id,
        "ros_domain_id": int(ros_domain_id),
        "topic": TOPIC_AMCL_POSE,
        "message_type": MSG_AMCL_POSE,
        "pose_valid": valid,
        "pose_reason": reason,
        "pose_source": "topic:amcl_pose",
        "frame_id": frame_id,
        "pose": None if not valid else {
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            "frame_id": "map",
        },
        "pose_observed_at": stamp,
        "gateway_received_at": received_at.isoformat(),
    }

def plan_to_dict(msg, *, robot_id: str, ros_domain_id: int, received_at: datetime | None = None) -> dict[str, Any]:
    """Nav2 local/global plan snapshot. Not a drive command and not travel history."""
    received_at = received_at or datetime.now(timezone.utc)
    frame_id = str(msg.header.frame_id or "")
    stamp = _stamp_iso(msg.header.stamp)
    poses = []
    for item in list(msg.poses or []):
        try:
            p = item.pose.position
            o = item.pose.orientation
            x, y, z = float(p.x), float(p.y), float(p.z)
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            poses.append({
                "x": x,
                "y": y,
                "z": z,
                "orientation": {
                    "x": float(o.x), "y": float(o.y), "z": float(o.z), "w": float(o.w),
                },
                "stamp": _stamp_iso(item.header.stamp) if hasattr(item, "header") else None,
                "frame_id": str(getattr(item.header, "frame_id", "") or frame_id or ""),
            })
        except (TypeError, ValueError, AttributeError):
            continue
    valid = frame_id in {"map", ""} and len(poses) > 0
    if frame_id and frame_id != "map":
        valid = False
        reason = "frame_mismatch"
    elif not poses:
        reason = "empty"
        valid = False
    else:
        reason = "ok"
        if not frame_id:
            frame_id = "map"
    return {
        "robot_id": robot_id,
        "ros_domain_id": int(ros_domain_id),
        "topic": TOPIC_PLAN,
        "message_type": MSG_PLAN,
        "plan_valid": valid,
        "plan_reason": reason,
        "plan_source": "topic:plan",
        "frame_id": frame_id,
        "pose_count": len(poses),
        "poses": poses if valid else [],
        "plan_observed_at": stamp,
        "gateway_received_at": received_at.isoformat(),
    }


def battery_percent_to_dict(
    msg,
    *,
    robot_id: str,
    ros_domain_id: int,
    received_at: datetime | None = None,
) -> dict[str, Any]:
    """Float32 has no stamp; gateway_received_at is the observation wall time."""
    received_at = received_at or datetime.now(timezone.utc)
    try:
        raw = float(msg.data)
    except (TypeError, ValueError, AttributeError):
        return {
            "robot_id": robot_id,
            "ros_domain_id": int(ros_domain_id),
            "topic": TOPIC_BATTERY_PERCENT,
            "message_type": MSG_BATTERY_PERCENT,
            "battery_valid": False,
            "battery_percentage": None,
            "battery_source": "",
            "battery_reason": "invalid_data",
            "gateway_received_at": received_at.isoformat(),
            "battery_observed_at": None,
        }
    if not math.isfinite(raw):
        reason = "invalid_data"
        soc = None
    elif 0.0 <= raw <= 1.0:
        # Already normalized; do not divide again.
        soc = raw
        reason = "ok"
    elif 0.0 <= raw <= 100.0:
        soc = raw / 100.0
        reason = "ok"
    else:
        soc = None
        reason = "out_of_range"
    valid = reason == "ok" and soc is not None
    return {
        "robot_id": robot_id,
        "ros_domain_id": int(ros_domain_id),
        "topic": TOPIC_BATTERY_PERCENT,
        "message_type": MSG_BATTERY_PERCENT,
        "battery_valid": valid,
        "battery_percentage": soc if valid else None,
        "battery_raw": raw,
        "battery_input_scale": BATTERY_INPUT_SCALE,
        "battery_source": "topic:battery/percent" if valid else "",
        "battery_reason": reason,
        "gateway_received_at": received_at.isoformat(),
        "battery_observed_at": received_at.isoformat() if valid else None,
    }


def _stamp_iso(stamp) -> str | None:
    try:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if sec == 0 and nanosec == 0:
        return None
    return datetime.fromtimestamp(sec + nanosec * 1e-9, tz=timezone.utc).isoformat()
