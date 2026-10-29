"""Canonical task payload and ROS field checks. No rclpy import."""
from __future__ import annotations

import json
from typing import Any

from robot_receiver.pinky_task_receiver import RECEIVER_NAME, accept, cancel, payload_hash

TOPIC_COMMAND = "/fms/task/command"
TOPIC_ACK = "/fms/task/ack"
TOPIC_CANCEL = "/fms/task/cancel"
TOPIC_CANCEL_ACK = "/fms/task/cancel_ack"
TOPIC_STATUS = "/fms/robot/status"

QOS = {
    "reliability": "reliable",
    "durability": "volatile",
    "history": "keep_last",
    "depth": 10,
}


def canonical_body(payload: dict) -> dict:
    return {key: value for key, value in payload.items() if key != "payload_hash"}


def fields_match_payload(message: dict, body: dict) -> str | None:
    """Return a reason when structured fields disagree with the canonical body."""
    if str(message.get("task_id") or "") != str(body.get("task_id") or ""):
        return "task_id does not match canonical payload"
    if str(message.get("robot_id") or "") != str(body.get("robot_id") or ""):
        return "robot_id does not match canonical payload"
    if str(message.get("task_type") or "") != str(body.get("task_type") or ""):
        return "task_type does not match canonical payload"
    if str(message.get("map_id") or "") != str(body.get("map_id") or ""):
        return "map_id does not match canonical payload"
    if str(message.get("map_revision") or "") != str(body.get("map_revision") or ""):
        return "map_revision does not match canonical payload"
    goal = body.get("goal") or {}
    if str(message.get("goal_label") or "") != str(goal.get("label") or ""):
        return "goal label does not match canonical payload"
    if _point_differs(message.get("start"), body.get("start")):
        return "start does not match canonical payload"
    if _point_differs(message.get("goal"), goal):
        return "goal does not match canonical payload"
    if not isinstance(body.get("waypoints"), list):
        return "waypoints are missing from canonical payload"
    encoded = message.get("waypoints") or []
    if len(encoded) != len(body["waypoints"]):
        return "waypoint count does not match canonical payload"
    for left, right in zip(encoded, body["waypoints"]):
        if str(left.get("waypoint_id") or "") != str(right.get("waypoint_id") or ""):
            return "waypoint id does not match canonical payload"
        if left.get("orientation_is_planned") is True:
            return "waypoint orientation was not planned"
        if _point_differs(left, right):
            return "waypoint coordinate does not match canonical payload"
    return None


def command_message(payload: dict, command_id: str) -> dict:
    if payload.get("execute") is True:
        raise ValueError("execute=true is not allowed")
    body = canonical_body(payload)
    body["execute"] = False
    digest = payload.get("payload_hash") or payload_hash(body)
    start = body.get("start") or {}
    goal = body.get("goal") or {}
    waypoints = []
    for item in body.get("waypoints") or []:
        waypoints.append({
            "waypoint_id": str(item.get("waypoint_id") or ""),
            "x": float(item["x"]),
            "y": float(item["y"]),
            "orientation_is_planned": False,
        })
    return {
        "task_id": str(body.get("task_id") or ""),
        "robot_id": str(body.get("robot_id") or ""),
        "task_type": str(body.get("task_type") or ""),
        "map_id": str(body.get("map_id") or ""),
        "map_revision": str(body.get("map_revision") or ""),
        "goal_label": str(goal.get("label") or ""),
        "start": {"x": float(start.get("x", 0.0)), "y": float(start.get("y", 0.0))},
        "goal": {"x": float(goal.get("x", 0.0)), "y": float(goal.get("y", 0.0))},
        "waypoints": waypoints,
        "payload_hash": digest,
        "command_id": command_id,
        "canonical_payload": json.dumps(body, sort_keys=True, separators=(",", ":")),
        "execute": False,
    }


def apply_command(store, message: dict, *, expected_robot_id: str) -> dict:
    """Validate and store. The returned dict is an application ACK, not a publish receipt."""
    command_id = str(message.get("command_id") or "")
    task_id = str(message.get("task_id") or "")
    robot_id = str(message.get("robot_id") or "")
    if robot_id != expected_robot_id:
        return _ack(command_id, task_id, robot_id, False, "robot_mismatch", "robot_id does not match this receiver")
    if message.get("execute") is not False:
        return _ack(command_id, task_id, robot_id, False, "execute_refused", "execute must be false")
    try:
        body = json.loads(message.get("canonical_payload") or "")
    except json.JSONDecodeError:
        return _ack(command_id, task_id, robot_id, False, "rejected", "canonical payload is not JSON")
    if not isinstance(body, dict):
        return _ack(command_id, task_id, robot_id, False, "rejected", "canonical payload must be an object")
    if body.get("execute") is True:
        return _ack(command_id, task_id, robot_id, False, "execute_refused", "execute must be false")
    mismatch = fields_match_payload(message, body)
    if mismatch:
        return _ack(command_id, task_id, robot_id, False, "rejected", mismatch)
    if payload_hash(body) != str(message.get("payload_hash") or ""):
        return _ack(command_id, task_id, robot_id, False, "hash_mismatch", "payload hash does not match")
    try:
        stored = accept(store, body)
    except ValueError as exc:
        return _ack(command_id, task_id, robot_id, False, "rejected", str(exc))
    return _ack(
        command_id, task_id, robot_id, True, "stored", "",
        payload_hash_value=stored.get("payload_hash") or "",
        received_at=stored.get("received_at") or "",
    )


def apply_cancel(store, message: dict, *, expected_robot_id: str) -> dict:
    task_id = str(message.get("task_id") or "")
    robot_id = str(message.get("robot_id") or "")
    request_id = str(message.get("cancellation_request_id") or "")
    if robot_id != expected_robot_id:
        return _cancel_ack(task_id, robot_id, request_id, False, "robot_mismatch", "robot_id does not match this receiver")
    if message.get("execute") is True:
        return _cancel_ack(task_id, robot_id, request_id, False, "execute_refused", "execute must be false")
    try:
        stored = cancel(store, {
            "action": "cancel",
            "task_id": task_id,
            "robot_id": robot_id,
            "cancellation_request_id": request_id,
            "reason": str(message.get("reason") or ""),
            "execute": False,
        })
    except ValueError as exc:
        outcome = "not_stored" if "not stored" in str(exc).lower() else "rejected"
        return _cancel_ack(task_id, robot_id, request_id, False, outcome, str(exc))
    return _cancel_ack(
        task_id,
        robot_id,
        str(stored.get("cancellation_request_id") or request_id),
        True,
        "cancel_ack",
        str(stored.get("reason") or ""),
        cancelled_at=stored.get("cancelled_at") or "",
    )


def _ack(command_id, task_id, robot_id, accepted, status, reason, payload_hash_value="", received_at="") -> dict:
    return {
        "command_id": command_id,
        "task_id": task_id,
        "robot_id": robot_id,
        "accepted": accepted,
        "status": status,
        "reason": reason,
        "payload_hash": payload_hash_value,
        "received_at": received_at,
        "receiver_id": RECEIVER_NAME,
        "receiver": RECEIVER_NAME,
        "execute": False,
        "channel": "ros_topic",
        "transport": "ros_topic",
    }


def _cancel_ack(task_id, robot_id, request_id, accepted, outcome, reason, cancelled_at="") -> dict:
    return {
        "kind": "cancel_ack",
        "task_id": task_id,
        "robot_id": robot_id,
        "cancellation_request_id": request_id,
        "accepted": accepted,
        "outcome": outcome,
        "reason": reason,
        "cancelled_at": cancelled_at,
        "receiver_id": RECEIVER_NAME,
        "receiver": RECEIVER_NAME,
        "execute": False,
        "channel": "ros_topic",
        "transport": "ros_topic",
    }


def _point_differs(message_point: Any, payload_point: Any) -> bool:
    if not isinstance(payload_point, dict):
        return True
    try:
        mx = float((message_point or {}).get("x"))
        my = float((message_point or {}).get("y"))
        px = float(payload_point.get("x"))
        py = float(payload_point.get("y"))
    except (TypeError, ValueError):
        return True
    return abs(mx - px) > 1e-6 or abs(my - py) > 1e-6
