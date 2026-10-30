"""Pinky task inbox.

Stores a validated payload and returns a receive ACK. Motion is performed by
fms_nav2_executor when execute=true. Run from the robot or swPC home directory,
not as a ROS package entry point alone.
"""
from __future__ import annotations

import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

RECEIVER_NAME = "pinky-fms-receiver"


def payload_hash(payload: dict) -> str:
    body = {key: value for key, value in payload.items() if key != "payload_hash"}
    encoded = json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def accept(store: Path, payload: dict, *, command_id: str = "") -> dict:
    task_id = str(payload.get("task_id") or "")
    robot_id = str(payload.get("robot_id") or "")
    if not task_id or not robot_id:
        raise ValueError("task_id and robot_id are required")
    execute = bool(payload.get("execute"))
    digest = payload_hash(payload)
    store.mkdir(parents=True, exist_ok=True)
    path = store / f"{task_id}.json"
    if path.exists():
        previous = json.loads(path.read_text(encoding="utf-8"))
        if previous.get("payload_hash") != digest:
            raise ValueError(f"Task {task_id} already stored with a different payload")
        ack = dict(previous["ack"])
        if command_id:
            ack["command_id"] = command_id
            return ack
        cid = str(payload.get("command_id") or "")
        if cid:
            ack["command_id"] = cid
        return ack
    ack = {
        "task_id": task_id,
        "robot_id": robot_id,
        "receiver": RECEIVER_NAME,
        "accepted": True,
        "received_at": datetime.now(timezone.utc).isoformat(),
        "payload_hash": digest,
        "execute": execute,
        "status": "stored",
    }
    if command_id:
        ack["command_id"] = command_id
    record = {
        "payload": payload,
        "payload_hash": digest,
        "ack": ack,
        "executable": execute,
        "execution": None,
    }
    path.write_text(json.dumps(record, indent=2), encoding="utf-8")
    return ack


def cancel(store: Path, request: dict) -> dict:
    """Mark a stored task non-executable and return a cancel ACK.

    Distinct from the receive ACK. Does not publish a motion command.
    Repeating the request returns the stored cancel ACK.
    """
    if request.get("execute") is True:
        raise ValueError("Cancel request must not set execute=true")
    task_id = str(request.get("task_id") or "")
    robot_id = str(request.get("robot_id") or "")
    request_id = str(request.get("cancellation_request_id") or "")
    if not task_id or not robot_id or not request_id:
        raise ValueError("task_id, robot_id, and cancellation_request_id are required")
    path = store / f"{task_id}.json"
    if not path.exists():
        raise ValueError(f"Task {task_id} is not stored")
    stored = json.loads(path.read_text(encoding="utf-8"))
    payload = stored.get("payload") or {}
    if str(payload.get("robot_id") or "") != robot_id:
        raise ValueError("robot_id does not match the stored task")
    existing = stored.get("cancellation")
    if isinstance(existing, dict) and existing.get("accepted") and existing.get("kind") == "cancel_ack":
        return existing
    ack = {
        "kind": "cancel_ack",
        "task_id": task_id,
        "robot_id": robot_id,
        "receiver": RECEIVER_NAME,
        "accepted": True,
        "cancelled": True,
        "cancellation_request_id": request_id,
        "cancelled_at": datetime.now(timezone.utc).isoformat(),
        "reason": str(request.get("reason") or ""),
        "execute": False,
    }
    stored["cancellation"] = ack
    stored["executable"] = False
    path.write_text(json.dumps(stored, indent=2), encoding="utf-8")
    return ack


def main() -> int:
    store = Path(sys.argv[1] if len(sys.argv) > 1 else "inbox")
    try:
        payload = json.load(sys.stdin)
        if payload.get("action") == "cancel":
            ack = cancel(store, payload)
        else:
            ack = accept(store, payload, command_id=str(payload.get("command_id") or ""))
    except Exception as exc:
        print(json.dumps({"accepted": False, "receiver": RECEIVER_NAME, "detail": str(exc)}))
        return 2
    print(json.dumps(ack))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
