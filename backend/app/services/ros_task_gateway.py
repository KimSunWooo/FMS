"""Hand a task to the robot's domain gateway and wait for a receiver ACK.

Publish success is not an ACK. SSH is not used when this exchange fails.
The FastAPI process does not create a ROS context.
"""
from __future__ import annotations

import json
import time
import uuid
from pathlib import Path

from app.services.fleet_dispatcher import DispatchError
from app.services.fleet_registry import RegistryError, load_registry
from robot_receiver.task_codec import (
    TOPIC_ACK,
    TOPIC_CANCEL,
    TOPIC_CANCEL_ACK,
    TOPIC_COMMAND,
    command_message,
)


def ipc_dir(data_dir: Path, robot_id: str) -> Path:
    return Path(data_dir) / "ros_ipc" / robot_id


def deliver_ros_topic(settings, payload: dict) -> dict:
    if payload.get("execute") is True:
        raise DispatchError("execute_refused", "execute=true is not allowed")
    robot = _physical_robot(settings, str(payload.get("robot_id") or ""))
    command_id = str(uuid.uuid4())
    message = command_message(payload, command_id)
    result = _exchange(settings, robot, "task_command", message, command_id)
    _require_task_ack(result, payload, command_id)
    return _decorate(settings, robot, result)


def cancel_ros_topic(settings, request: dict) -> dict:
    if request.get("execute") is True:
        raise DispatchError("execute_refused", "execute=true is not allowed")
    robot = _physical_robot(settings, str(request.get("robot_id") or ""))
    command_id = str(request.get("cancellation_request_id") or "")
    if not command_id:
        raise DispatchError("cancel_rejected", "cancellation_request_id is required")
    message = {
        "task_id": str(request.get("task_id") or ""),
        "robot_id": robot["robot_id"],
        "cancellation_request_id": command_id,
        "reason": str(request.get("reason") or ""),
        "execute": False,
    }
    result = _exchange(settings, robot, "task_cancel", message, command_id)
    if not result.get("accepted") or result.get("kind") != "cancel_ack":
        detail = str(result.get("reason") or "Receiver rejected the cancellation")
        code = "not_stored" if result.get("outcome") == "not_stored" or "not stored" in detail.lower() else "cancel_rejected"
        raise DispatchError(code, detail)
    if result.get("task_id") != request.get("task_id") or result.get("robot_id") != request.get("robot_id"):
        raise DispatchError("ack_mismatch", "Cancel ACK does not match the task")
    return _decorate(settings, robot, result)


def _physical_robot(settings, robot_id: str) -> dict:
    try:
        robots = load_registry(settings.physical_fleet_path)
    except RegistryError as exc:
        raise DispatchError("robot_unknown", str(exc)) from exc
    robot = next((item for item in robots if item["robot_id"] == robot_id), None)
    if robot is None:
        raise DispatchError("robot_unknown", f"{robot_id} is not registered")
    if robot["kind"] != "physical" or not robot["physical_connection_enabled"]:
        raise DispatchError("physical_transport_disabled", f"{robot_id} has no physical ROS transport")
    return robot


def _exchange(settings, robot: dict, kind: str, message: dict, command_id: str) -> dict:
    root = ipc_dir(Path(settings.data_dir), robot["robot_id"])
    commands = root / "commands"
    results = root / "results"
    commands.mkdir(parents=True, exist_ok=True)
    results.mkdir(parents=True, exist_ok=True)
    attempts = max(1, int(getattr(settings, "ros_task_max_attempts", 2)))
    timeout = float(getattr(settings, "ros_task_ack_timeout_s", 8.0))
    result_path = results / f"{command_id}.json"
    command_path = commands / f"{command_id}.json"
    last = {"accepted": False, "reason": "ACK not received", "status": "timeout"}
    for _attempt in range(attempts):
        if result_path.exists():
            result_path.unlink()
        command_path.write_text(json.dumps({
            "kind": kind,
            "robot_id": robot["robot_id"],
            "ros_domain_id": int(robot["ros_domain_id"]),
            "command_id": command_id,
            "message": message,
        }, sort_keys=True), encoding="utf-8")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if result_path.is_file():
                try:
                    last = json.loads(result_path.read_text(encoding="utf-8"))
                except json.JSONDecodeError:
                    last = {"accepted": False, "reason": "ACK file is not JSON", "status": "rejected"}
                break
            time.sleep(0.05)
        else:
            last = {"accepted": False, "reason": "Receiver ACK timed out", "status": "timeout", "command_id": command_id}
        if last.get("accepted"):
            return last
        if last.get("status") in {"rejected", "hash_mismatch", "robot_mismatch", "execute_refused"}:
            raise DispatchError(str(last.get("status") or "ack_rejected"), str(last.get("reason") or "Receiver rejected the task"))
    raise DispatchError("ack_timeout", str(last.get("reason") or "Receiver ACK timed out"))


def _require_task_ack(result: dict, payload: dict, command_id: str) -> None:
    if not result.get("accepted"):
        raise DispatchError("ack_rejected", str(result.get("reason") or "Receiver rejected the task"))
    if result.get("command_id") != command_id or result.get("task_id") != payload.get("task_id"):
        raise DispatchError("ack_mismatch", "ACK command id or task id does not match")
    if result.get("payload_hash") != payload.get("payload_hash"):
        raise DispatchError("ack_mismatch", "ACK payload hash does not match")


def _decorate(settings, robot: dict, result: dict) -> dict:
    ack = dict(result)
    ack["channel"] = "ros_topic"
    ack["transport"] = "ros_topic"
    ack["ros_domain_id"] = int(robot["ros_domain_id"])
    ack["command_topic"] = getattr(settings, "ros_task_command_topic", TOPIC_COMMAND)
    ack["ack_topic"] = getattr(settings, "ros_task_ack_topic", TOPIC_ACK)
    ack["cancel_topic"] = getattr(settings, "ros_task_cancel_topic", TOPIC_CANCEL)
    ack["cancel_ack_topic"] = getattr(settings, "ros_task_cancel_ack_topic", TOPIC_CANCEL_ACK)
    ack["publish_state"] = "ack_received"
    ack["executed"] = False
    ack["completed"] = False
    ack["receiver_ready"] = True
    return ack
