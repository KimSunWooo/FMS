"""Receive-only Pinky task inbox.

Stores a validated payload and prints an ACK. It does not publish cmd_vel,
call Nav2, or move the robot. Run from the robot home directory, not from a
ROS package.
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


def accept(store: Path, payload: dict) -> dict:
    if payload.get("execute") is True:
        raise ValueError("Receiver refuses payloads marked execute=true")
    task_id = str(payload.get("task_id") or "")
    robot_id = str(payload.get("robot_id") or "")
    if not task_id or not robot_id:
        raise ValueError("task_id and robot_id are required")
    digest = payload_hash(payload)
    store.mkdir(parents=True, exist_ok=True)
    path = store / f"{task_id}.json"
    if path.exists():
        previous = json.loads(path.read_text(encoding="utf-8"))
        if previous.get("payload_hash") != digest:
            raise ValueError(f"Task {task_id} already stored with a different payload")
        return previous["ack"]
    ack = {
        "task_id": task_id,
        "robot_id": robot_id,
        "receiver": RECEIVER_NAME,
        "accepted": True,
        "received_at": datetime.now(timezone.utc).isoformat(),
        "payload_hash": digest,
        "execute": False,
    }
    path.write_text(json.dumps({"payload": payload, "payload_hash": digest, "ack": ack}, indent=2), encoding="utf-8")
    return ack


def cancel(store: Path, request: dict) -> dict:
    """Mark a stored task non-executable and return a cancel ACK.

    This is distinct from the receive ACK. It does not publish a motion command.
    Repeating the request returns the stored cancel ACK.
    """
    if request.get("execute") is True:
        raise ValueError("Receiver refuses payloads marked execute=true")
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
            ack = accept(store, payload)
    except Exception as exc:
        print(json.dumps({"accepted": False, "receiver": RECEIVER_NAME, "detail": str(exc)}))
        return 2
    print(json.dumps(ack))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
