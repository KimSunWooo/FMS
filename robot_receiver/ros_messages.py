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
    msg.execute = False
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
        "execute": False,
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


def status_message(robot_id: str, domain_id: int, *, ready: bool, last_task_id: str) -> RobotStatus:
    msg = RobotStatus()
    msg.robot_id = robot_id
    msg.ros_domain_id = int(domain_id)
    msg.receiver_ready = ready
    msg.task_execution_enabled = False
    msg.active_task_id = ""
    msg.last_received_task_id = last_task_id
    return msg
