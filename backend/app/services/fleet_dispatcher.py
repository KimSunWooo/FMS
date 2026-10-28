"""Deliver a stored task payload. Physical delivery never sends a motion command."""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RECEIVER = ROOT / "robot_receiver" / "pinky_task_receiver.py"


class DispatchError(Exception):
    def __init__(self, code: str, detail: str) -> None:
        super().__init__(detail)
        self.code = code
        self.detail = detail


def _run_local(store: Path, payload: dict) -> dict:
    try:
        completed = subprocess.run(
            [sys.executable, str(RECEIVER), str(store)],
            input=json.dumps(payload),
            text=True,
            capture_output=True,
            timeout=10,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise DispatchError("ack_timeout", "Receiver ACK timed out") from exc
    line = (completed.stdout or "").strip().splitlines()
    try:
        ack = json.loads(line[-1]) if line else {}
    except json.JSONDecodeError as exc:
        raise DispatchError("ack_missing", "Receiver did not return JSON") from exc
    if completed.returncode != 0 or not ack.get("accepted"):
        raise DispatchError("ack_rejected", ack.get("detail", "Receiver rejected the payload"))
    if ack.get("task_id") != payload.get("task_id") or ack.get("payload_hash") != payload.get("payload_hash"):
        raise DispatchError("ack_mismatch", "ACK task id or payload hash does not match")
    return ack


def deliver_simulated(store: Path, payload: dict) -> dict:
    """Record a simulated receipt. This is not a Pinky receiver ACK."""
    from datetime import datetime, timezone

    store.mkdir(parents=True, exist_ok=True)
    task_id = str(payload.get("task_id") or "")
    path = store / f"{task_id}.simulated.json"
    if path.exists():
        previous = json.loads(path.read_text(encoding="utf-8"))
        if previous.get("payload_hash") != payload.get("payload_hash"):
            raise DispatchError("payload_conflict", "Simulated task id was stored with a different payload")
        return previous["ack"]
    ack = {
        "task_id": task_id,
        "robot_id": payload.get("robot_id"),
        "receiver": "fms-virtual-receiver",
        "channel": "simulated",
        "accepted": True,
        "received_at": datetime.now(timezone.utc).isoformat(),
        "payload_hash": payload.get("payload_hash"),
        "execute": False,
    }
    path.write_text(json.dumps({"payload_hash": payload.get("payload_hash"), "ack": ack}, indent=2), encoding="utf-8")
    return ack


def deliver_local_receiver(store: Path, payload: dict) -> dict:
    ack = dict(_run_local(store, payload))
    ack["channel"] = "physical"
    return ack


def _parse_process_output(completed) -> tuple[int, dict]:
    line = (completed.stdout or "").strip().splitlines()
    try:
        body = json.loads(line[-1]) if line else {}
    except json.JSONDecodeError as exc:
        raise DispatchError("ack_missing", "Receiver did not return JSON") from exc
    return completed.returncode, body


def cancel_local_receiver(store: Path, request: dict) -> dict:
    try:
        completed = subprocess.run(
            [sys.executable, str(RECEIVER), str(store)],
            input=json.dumps(request),
            text=True,
            capture_output=True,
            timeout=10,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise DispatchError("ack_timeout", "Receiver cancel ACK timed out") from exc
    code, ack = _parse_process_output(completed)
    return _require_cancel_ack(code, ack, request, channel="physical")


def cancel_simulated(store: Path, request: dict) -> dict:
    """Record cancellation of a simulated receipt. This is not a Pinky ACK."""
    from datetime import datetime, timezone

    task_id = str(request.get("task_id") or "")
    path = store / f"{task_id}.simulated.json"
    if not path.exists():
        raise DispatchError("not_stored", f"Task {task_id} is not stored")
    stored = json.loads(path.read_text(encoding="utf-8"))
    existing = stored.get("cancellation")
    if isinstance(existing, dict) and existing.get("accepted") and existing.get("kind") == "cancel_ack":
        return existing
    ack = {
        "kind": "cancel_ack",
        "task_id": task_id,
        "robot_id": request.get("robot_id"),
        "receiver": "fms-virtual-receiver",
        "channel": "simulated",
        "accepted": True,
        "cancelled": True,
        "cancellation_request_id": request.get("cancellation_request_id"),
        "cancelled_at": datetime.now(timezone.utc).isoformat(),
        "reason": str(request.get("reason") or ""),
        "execute": False,
    }
    stored["cancellation"] = ack
    stored["executable"] = False
    path.write_text(json.dumps(stored, indent=2), encoding="utf-8")
    return ack


def _require_cancel_ack(code: int, ack: dict, request: dict, *, channel: str) -> dict:
    if code != 0 or not ack.get("accepted") or ack.get("kind") != "cancel_ack":
        detail = str(ack.get("detail") or "Receiver rejected the cancellation")
        error_code = "not_stored" if "not stored" in detail.lower() else "cancel_rejected"
        raise DispatchError(error_code, detail)
    if ack.get("task_id") != request.get("task_id") or ack.get("robot_id") != request.get("robot_id"):
        raise DispatchError("ack_mismatch", "Cancel ACK does not match the task")
    ack = dict(ack)
    ack["channel"] = channel
    return ack


def _ssh_exchange(settings, target: str, message: dict) -> tuple[int, dict]:
    """Upload the receive-only script and exchange one JSON message. No ros2."""
    import paramiko

    if not settings.physical_ssh_password:
        raise DispatchError("ssh_unconfigured", "SSH password is not configured")
    username, hostname = target.rsplit("@", 1)
    client = paramiko.SSHClient()
    try:
        client.load_system_host_keys()
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
        client.connect(
            hostname=hostname,
            username=username,
            password=settings.physical_ssh_password,
            look_for_keys=False,
            allow_agent=False,
            timeout=settings.physical_ssh_timeout_seconds,
            auth_timeout=settings.physical_ssh_timeout_seconds,
            banner_timeout=settings.physical_ssh_timeout_seconds,
        )
        remote_dir = "fms_task_receiver"
        with client.open_sftp() as sftp:
            try:
                sftp.stat(remote_dir)
            except OSError:
                sftp.mkdir(remote_dir)
            sftp.put(str(RECEIVER), f"{remote_dir}/pinky_task_receiver.py")
        command = "python3 ~/fms_task_receiver/pinky_task_receiver.py ~/fms_task_receiver/inbox"
        if "ros2" in command or "cmd_vel" in command:
            raise DispatchError("unsafe_command", "Refusing a dispatch command that can move the robot")
        stdin, stdout, _stderr = client.exec_command(command, timeout=20)
        stdin.write(json.dumps(message))
        stdin.channel.shutdown_write()
        raw = stdout.read().decode("utf-8", errors="replace").strip()
        code = stdout.channel.recv_exit_status()
    except DispatchError:
        raise
    except (TimeoutError, subprocess.TimeoutExpired) as exc:
        raise DispatchError("ack_timeout", "Receiver ACK timed out") from exc
    except Exception as exc:
        raise DispatchError("ssh_failed", f"SSH delivery failed: {type(exc).__name__}") from exc
    finally:
        client.close()
    try:
        ack = json.loads(raw.splitlines()[-1]) if raw else {}
    except json.JSONDecodeError as exc:
        raise DispatchError("ack_missing", "Receiver did not return JSON") from exc
    return code, ack


def deliver_ssh(settings, target: str, payload: dict) -> dict:
    """Copy the receive-only script to ~/fms_task_receiver and read its ACK.

    The remote command is python3 on that script. It does not call ros2.
    """
    code, ack = _ssh_exchange(settings, target, payload)
    if code != 0 or not ack.get("accepted"):
        raise DispatchError(ack.get("detail", "ack_rejected"), ack.get("detail", "Receiver rejected the payload"))
    if ack.get("task_id") != payload.get("task_id") or ack.get("payload_hash") != payload.get("payload_hash"):
        raise DispatchError("ack_mismatch", "ACK task id or payload hash does not match")
    ack["channel"] = "physical"
    return ack


def cancel_ssh(settings, target: str, request: dict) -> dict:
    code, ack = _ssh_exchange(settings, target, request)
    return _require_cancel_ack(code, ack, request, channel="physical")
