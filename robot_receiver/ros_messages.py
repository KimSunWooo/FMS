"""Fill fms_msgs from plain dicts. Import only after ROS and fms_msgs are on the path."""
from __future__ import annotations

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header

from fms_msgs.msg import RobotStatus, TaskAck, TaskCancel, TaskCancelAck, TaskCommand, Waypoint


def _time_from_iso(value: str) -> Time:
    stamp = Time()
    text = str(value or "")
    if not text:
        return stamp
    from datetime import datetime
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError:
        return stamp
    whole = int(parsed.timestamp())
    stamp.sec = whole
    stamp.nanosec = int(round((parsed.timestamp() - whole) * 1_000_000_000))
    return stamp


def _pose(x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header = Header(frame_id="map")
    pose.pose = Pose(
        position=Point(x=float(x), y=float(y), z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    return pose


def command_from_dict(message: dict) -> TaskCommand:
    msg = TaskCommand()
    msg.task_id = message["task_id"]
    msg.robot_id = message["robot_id"]
    msg.task_type = message["task_type"]
    msg.map_id = message["map_id"]
    msg.map_revision = message["map_revision"]
    msg.goal_label = message.get("goal_label") or ""
    msg.start = Point(x=float(message["start"]["x"]), y=float(message["start"]["y"]), z=0.0)
    msg.goal = Point(x=float(message["goal"]["x"]), y=float(message["goal"]["y"]), z=0.0)
    for item in message.get("waypoints") or []:
        waypoint = Waypoint()
        waypoint.waypoint_id = str(item.get("waypoint_id") or "")
        waypoint.pose = _pose(item["x"], item["y"])
        waypoint.orientation_is_planned = False
        msg.waypoints.append(waypoint)
    msg.payload_hash = message["payload_hash"]
    msg.command_id = message["command_id"]
    msg.canonical_payload = message["canonical_payload"]
    msg.execute = bool(message.get("execute"))
    return msg


def command_to_dict(msg: TaskCommand) -> dict:
    return {
        "task_id": msg.task_id,
        "robot_id": msg.robot_id,
        "task_type": msg.task_type,
        "map_id": msg.map_id,
        "map_revision": msg.map_revision,
        "goal_label": msg.goal_label,
        "start": {"x": msg.start.x, "y": msg.start.y},
        "goal": {"x": msg.goal.x, "y": msg.goal.y},
        "waypoints": [
            {
                "waypoint_id": item.waypoint_id,
                "x": item.pose.pose.position.x,
                "y": item.pose.pose.position.y,
                "orientation_is_planned": bool(item.orientation_is_planned),
            }
            for item in msg.waypoints
        ],
        "payload_hash": msg.payload_hash,
        "command_id": msg.command_id,
        "canonical_payload": msg.canonical_payload,
        "execute": bool(msg.execute),
    }


def ack_from_dict(body: dict) -> TaskAck:
    msg = TaskAck()
    msg.command_id = str(body.get("command_id") or "")
    msg.task_id = str(body.get("task_id") or "")
    msg.robot_id = str(body.get("robot_id") or "")
    msg.accepted = bool(body.get("accepted"))
    msg.status = str(body.get("status") or "")
    msg.reason = str(body.get("reason") or "")
    msg.payload_hash = str(body.get("payload_hash") or "")
    msg.received_at = _time_from_iso(str(body.get("received_at") or ""))
    msg.receiver_id = str(body.get("receiver_id") or "")
    return msg


def ack_to_dict(msg: TaskAck) -> dict:
    return {
        "command_id": msg.command_id,
        "task_id": msg.task_id,
        "robot_id": msg.robot_id,
        "accepted": bool(msg.accepted),
        "status": msg.status,
        "reason": msg.reason,
        "payload_hash": msg.payload_hash,
        "received_at": _iso_from_time(msg.received_at),
        "receiver_id": msg.receiver_id,
        "receiver": msg.receiver_id,
    }


def cancel_from_dict(message: dict) -> TaskCancel:
    msg = TaskCancel()
    msg.task_id = message["task_id"]
    msg.robot_id = message["robot_id"]
    msg.cancellation_request_id = message["cancellation_request_id"]
    msg.requested_at = _time_from_iso("")
    msg.reason = str(message.get("reason") or "")
    msg.execute = False
    return msg


def cancel_to_dict(msg: TaskCancel) -> dict:
    return {
        "task_id": msg.task_id,
        "robot_id": msg.robot_id,
        "cancellation_request_id": msg.cancellation_request_id,
        "reason": msg.reason,
        "execute": bool(msg.execute),
    }


def cancel_ack_from_dict(body: dict) -> TaskCancelAck:
    msg = TaskCancelAck()
    msg.task_id = str(body.get("task_id") or "")
    msg.robot_id = str(body.get("robot_id") or "")
    msg.cancellation_request_id = str(body.get("cancellation_request_id") or "")
    msg.accepted = bool(body.get("accepted"))
    msg.outcome = str(body.get("outcome") or "")
    msg.reason = str(body.get("reason") or "")
    msg.cancelled_at = _time_from_iso(str(body.get("cancelled_at") or ""))
    msg.receiver_id = str(body.get("receiver_id") or "")
    return msg


def cancel_ack_to_dict(msg: TaskCancelAck) -> dict:
    return {
        "kind": "cancel_ack",
        "task_id": msg.task_id,
        "robot_id": msg.robot_id,
        "cancellation_request_id": msg.cancellation_request_id,
        "accepted": bool(msg.accepted),
        "outcome": msg.outcome,
        "reason": msg.reason,
        "cancelled_at": _iso_from_time(msg.cancelled_at),
        "receiver_id": msg.receiver_id,
        "receiver": msg.receiver_id,
        "execute": False,
    }


def _iso_from_time(stamp) -> str:
    from datetime import datetime, timezone
    seconds = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000
    if seconds <= 0:
        return ""
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat()


def status_message(
    robot_id: str,
    domain_id: int,
    *,
    ready: bool,
    last_task_id: str,
    active_task_id: str = "",
    pose_report: dict | None = None,
    battery_report: dict | None = None,
    task_execution_enabled: bool | None = None,
) -> RobotStatus:
    """Build RobotStatus. Invalid reports keep *_valid=false (never invents values)."""
    import os

    msg = RobotStatus()
    msg.robot_id = robot_id
    msg.ros_domain_id = int(domain_id)
    msg.receiver_ready = ready
    if task_execution_enabled is None:
        task_execution_enabled = os.environ.get("FMS_TASK_EXECUTION_ENABLED", "true").lower() in {
            "1", "true", "yes", "on",
        }
    msg.task_execution_enabled = bool(task_execution_enabled)
    msg.active_task_id = str(active_task_id or "")
    msg.last_received_task_id = last_task_id
    apply_pose_report(msg, pose_report)
    apply_battery_report(msg, battery_report)
    return msg


def apply_battery_report(msg: RobotStatus, battery_report: dict | None) -> None:
    """Attach validated SOC (0..1) or clear battery_valid without inventing a level."""
    from builtin_interfaces.msg import Time

    msg.battery_valid = False
    msg.battery_percentage = 0.0
    msg.battery_stamp = Time()
    msg.battery_source = ""
    if not battery_report or not battery_report.get("battery_valid"):
        return
    msg.battery_valid = True
    msg.battery_percentage = float(battery_report["battery_percentage"])
    msg.battery_source = str(battery_report.get("battery_source") or "")
    ros_stamp = battery_report.get("battery_ros_stamp")
    if ros_stamp is not None:
        msg.battery_stamp = ros_stamp
    else:
        observed = str(battery_report.get("battery_observed_at") or "")
        if observed:
            msg.battery_stamp = _time_from_iso(observed)


def apply_pose_report(msg: RobotStatus, pose_report: dict | None) -> None:
    """Attach a validated TF pose, or clear pose_valid without treating zeros as location."""
    from robot_receiver.pose_report import POSE_SOURCE_TF

    msg.pose = PoseStamped()
    msg.pose.header = Header(frame_id="map")
    msg.pose.pose = Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    msg.pose_valid = False
    msg.pose_source = ""
    if not pose_report or not pose_report.get("pose_valid"):
        return
    orientation = pose_report["orientation"]
    msg.pose_valid = True
    msg.pose_source = str(pose_report.get("pose_source") or POSE_SOURCE_TF)
    msg.pose.header.frame_id = "map"
    tf_stamp = pose_report.get("tf_stamp")
    if tf_stamp is not None:
        msg.pose.header.stamp = tf_stamp
    else:
        observed = str(pose_report.get("pose_observed_at") or "")
        if observed:
            msg.pose.header.stamp = _time_from_iso(observed)
    msg.pose.pose.position.x = float(pose_report["x"])
    msg.pose.pose.position.y = float(pose_report["y"])
    msg.pose.pose.position.z = float(pose_report["z"])
    msg.pose.pose.orientation.x = float(orientation["x"])
    msg.pose.pose.orientation.y = float(orientation["y"])
    msg.pose.pose.orientation.z = float(orientation["z"])
    msg.pose.pose.orientation.w = float(orientation["w"])


def status_to_dict(msg: RobotStatus) -> dict:
    """Deserialize RobotStatus for IPC. pose_valid=false ⇒ pose is null."""
    from robot_receiver.pose_report import quaternion_is_valid, yaw_from_quaternion
    import math

    stamp = _iso_from_time(msg.stamp) if hasattr(msg, "stamp") else ""
    claimed = bool(getattr(msg, "pose_valid", False))
    pose_source = str(getattr(msg, "pose_source", "") or "")
    pose_block = None
    pose_observed_at = None
    pose_valid = False
    if claimed and hasattr(msg, "pose"):
        pose = msg.pose
        frame_id = str(pose.header.frame_id or "")
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        z = float(pose.pose.position.z)
        qx = float(pose.pose.orientation.x)
        qy = float(pose.pose.orientation.y)
        qz = float(pose.pose.orientation.z)
        qw = float(pose.pose.orientation.w)
        if (
            frame_id == "map"
            and all(math.isfinite(v) for v in (x, y, z))
            and quaternion_is_valid(qx, qy, qz, qw)
        ):
            pose_valid = True
            # ROS stamp is kept separately; Gateway fills wall-clock pose_observed_at.
            pose_observed_at = _iso_from_time(pose.header.stamp) or None
            pose_block = {
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw_from_quaternion(qx, qy, qz, qw),
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                "frame_id": "map",
            }
        else:
            pose_source = ""
    else:
        pose_source = ""
    battery_valid = bool(getattr(msg, "battery_valid", False))
    battery_source = str(getattr(msg, "battery_source", "") or "")
    battery_percentage = None
    battery_observed_at = None
    if battery_valid:
        try:
            raw = float(msg.battery_percentage)
        except (TypeError, ValueError, AttributeError):
            battery_valid = False
            battery_source = ""
        else:
            from robot_receiver.battery_report import normalize_soc

            soc = normalize_soc(raw, assume_percent_if_gt_one=False)
            if soc is None:
                battery_valid = False
                battery_source = ""
            else:
                battery_percentage = soc
                battery_observed_at = _iso_from_time(getattr(msg, "battery_stamp", None)) or None
    else:
        battery_source = ""

    return {
        "robot_id": msg.robot_id,
        "ros_domain_id": int(msg.ros_domain_id),
        "receiver_ready": bool(msg.receiver_ready),
        "task_execution_enabled": bool(getattr(msg, "task_execution_enabled", False)),
        "active_task_id": str(getattr(msg, "active_task_id", "") or ""),
        "last_received_task_id": msg.last_received_task_id,
        "stamp": stamp or None,
        "pose_valid": pose_valid,
        "pose_source": pose_source if pose_valid else "",
        "pose": pose_block if pose_valid else None,
        "pose_ros_stamp": pose_observed_at if pose_valid else None,
        "pose_observed_at": None,
        "battery_valid": battery_valid,
        "battery_percentage": battery_percentage if battery_valid else None,
        "battery_source": battery_source if battery_valid else "",
        "battery_ros_stamp": battery_observed_at if battery_valid else None,
        "battery_observed_at": None,
    }