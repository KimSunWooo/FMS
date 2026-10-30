"""ROS topic task contract. IPC tests do not claim a live Pinky ACK."""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path

from robot_receiver.task_codec import apply_cancel, apply_command, command_message

from test_fleet_tasks import GOAL, START, _create


def _sample_payload() -> dict:
    body = {
        "task_id": "task-1",
        "robot_id": "pinky-1",
        "task_type": "navigate",
        "map_id": "1f_map_f",
        "map_revision": "abc",
        "start": {"x": 0.1, "y": 0.2},
        "goal": {"x": 0.3, "y": 0.4, "label": "desk"},
        "waypoints": [{"x": 0.1, "y": 0.2, "waypoint_id": "w0"}, {"x": 0.3, "y": 0.4, "waypoint_id": "w1"}],
        "execute": False,
    }
    from robot_receiver.pinky_task_receiver import payload_hash
    body["payload_hash"] = payload_hash(body)
    return body


def test_receiver_rejects_wrong_robot_hash_and_accepts_execute(tmp_path):
    payload = _sample_payload()
    message = command_message(payload, "cmd-1")
    foreign = dict(message)
    foreign["robot_id"] = "pinky-2"
    rejected = apply_command(tmp_path, foreign, expected_robot_id="pinky-1")
    assert rejected["accepted"] is False
    assert rejected["status"] == "robot_mismatch"
    assert list(tmp_path.glob("*.json")) == []

    tampered = dict(message)
    tampered["payload_hash"] = "0" * 64
    hashed = apply_command(tmp_path, tampered, expected_robot_id="pinky-1")
    assert hashed["status"] == "hash_mismatch"

    # execute mismatch between message field and canonical body is rejected.
    forced = dict(message)
    forced["execute"] = True
    refused = apply_command(tmp_path, forced, expected_robot_id="pinky-1")
    assert refused["status"] == "rejected"
    assert list(tmp_path.glob("*.json")) == []

    # Consistent execute=true is accepted and stored (Executor runs Nav2 separately).
    from robot_receiver.pinky_task_receiver import payload_hash

    drive = dict(payload)
    drive["execute"] = True
    drive["payload_hash"] = payload_hash(drive)
    stored = apply_command(tmp_path, command_message(drive, "cmd-drive"), expected_robot_id="pinky-1")
    assert stored["accepted"] is True
    assert stored["status"] == "stored"
    assert stored["execute"] is True
    body = json.loads((tmp_path / "task-1.json").read_text(encoding="utf-8"))
    assert body["payload"]["execute"] is True
    assert body["executable"] is True


def test_duplicate_command_reuses_the_stored_task(tmp_path):
    payload = _sample_payload()
    first = apply_command(tmp_path, command_message(payload, "cmd-1"), expected_robot_id="pinky-1")
    second = apply_command(tmp_path, command_message(payload, "cmd-2"), expected_robot_id="pinky-1")
    assert first["accepted"] is True
    assert second["accepted"] is True
    assert second["command_id"] == "cmd-2"
    assert second["payload_hash"] == first["payload_hash"]
    assert len(list(tmp_path.glob("*.json"))) == 1
    changed = dict(payload)
    changed["goal"] = {"x": 9.0, "y": 9.0, "label": "desk"}
    from robot_receiver.pinky_task_receiver import payload_hash
    changed["payload_hash"] = payload_hash(changed)
    # Same task id, different body: the stored file rejects it.
    again = command_message(changed, "cmd-3")
    # command_message hashes the changed body, but the store already has the first hash.
    rejected = apply_command(tmp_path, again, expected_robot_id="pinky-1")
    assert rejected["accepted"] is False


def test_cancel_ack_keeps_the_receive_hash(tmp_path):
    payload = _sample_payload()
    stored = apply_command(tmp_path, command_message(payload, "cmd-1"), expected_robot_id="pinky-1")
    ack = apply_cancel(tmp_path, {
        "task_id": payload["task_id"],
        "robot_id": "pinky-1",
        "cancellation_request_id": "cancel-1",
        "reason": "operator",
        "execute": False,
    }, expected_robot_id="pinky-1")
    assert ack["accepted"] is True
    assert ack["outcome"] == "cancel_ack"
    body = json.loads((tmp_path / "task-1.json").read_text(encoding="utf-8"))
    assert body["executable"] is False
    assert body["payload_hash"] == stored["payload_hash"]
    repeat = apply_cancel(tmp_path, {
        "task_id": payload["task_id"],
        "robot_id": "pinky-1",
        "cancellation_request_id": "cancel-2",
        "reason": "again",
        "execute": False,
    }, expected_robot_id="pinky-1")
    assert repeat["accepted"] is True
    assert repeat["cancellation_request_id"] == "cancel-1"


def test_ros_topic_dispatch_waits_for_ack_and_does_not_use_ssh(env):
    client, _runtime = env
    settings = client.app.state.settings
    settings.fleet_dispatch_mode = "ros_topic"
    settings.ros_task_ack_timeout_s = 2.0
    settings.ros_task_max_attempts = 1
    settings.physical_ssh_targets = "pinky-1=pinky@192.168.0.83"
    settings.physical_ssh_password = "unused"
    store = Path(settings.data_dir) / "topic_inbox"
    stop = threading.Event()

    def pump() -> None:
        root = Path(settings.data_dir) / "ros_ipc" / "pinky-1"
        while not stop.is_set():
            folder = root / "commands"
            if folder.is_dir():
                for path in list(folder.glob("*.json")):
                    request = json.loads(path.read_text(encoding="utf-8"))
                    message = request["message"]
                    if request["kind"] == "task_command":
                        result = apply_command(store, message, expected_robot_id="pinky-1")
                    else:
                        result = apply_cancel(store, message, expected_robot_id="pinky-1")
                    (root / "results").mkdir(parents=True, exist_ok=True)
                    (root / "results" / f"{request['command_id']}.json").write_text(
                        json.dumps(result), encoding="utf-8",
                    )
                    path.unlink(missing_ok=True)
            time.sleep(0.02)

    thread = threading.Thread(target=pump, daemon=True)
    thread.start()
    try:
        task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
        delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch")
        body = delivered.json()
        assert body["status"] == "ACKNOWLEDGED", body
        assert body["ack_channel"] == "ros_topic"
        assert body["ack"]["transport"] == "ros_topic"
        assert body["ack"]["publish_state"] == "ack_received"
        assert body["ack"]["executed"] is False
        assert body["transport"]["received"] is True
        assert body["transport"]["completed"] is False
        assert body["motion_verified"] is False
        cancelled = client.post(f"/api/fleet-tasks/{task['task_id']}/cancel", json={"reason": "operator"})
        assert cancelled.status_code == 200, cancelled.text
        assert cancelled.json()["status"] == "CANCELLED"
        assert cancelled.json()["ack_status"] == "CANCEL_ACKNOWLEDGED"
    finally:
        stop.set()
        thread.join(timeout=2)


def test_ack_timeout_rewrites_the_same_command_before_accepting_it(env):
    client, _runtime = env
    settings = client.app.state.settings
    settings.fleet_dispatch_mode = "ros_topic"
    settings.ros_task_ack_timeout_s = 0.4
    settings.ros_task_max_attempts = 2
    store = Path(settings.data_dir) / "topic_inbox"
    stamps: list[int] = []
    stop = threading.Event()

    def pump() -> None:
        root = Path(settings.data_dir) / "ros_ipc" / "pinky-1"
        while not stop.is_set():
            folder = root / "commands"
            if folder.is_dir():
                for path in list(folder.glob("*.json")):
                    stamp = path.stat().st_mtime_ns
                    if stamp not in stamps:
                        stamps.append(stamp)
                    if len(stamps) < 2:
                        continue
                    request = json.loads(path.read_text(encoding="utf-8"))
                    message = request["message"]
                    result = apply_command(store, message, expected_robot_id="pinky-1")
                    assert result["accepted"] is True
                    assert result["command_id"] == request["command_id"]
                    assert result["task_id"] == message["task_id"]
                    assert result["robot_id"] == "pinky-1"
                    assert result["payload_hash"] == message["payload_hash"]
                    (root / "results").mkdir(parents=True, exist_ok=True)
                    (root / "results" / f"{request['command_id']}.json").write_text(
                        json.dumps(result), encoding="utf-8",
                    )
                    path.unlink(missing_ok=True)
            time.sleep(0.02)

    thread = threading.Thread(target=pump, daemon=True)
    thread.start()
    try:
        task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
        delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch")
        body = delivered.json()
        assert body["status"] == "ACKNOWLEDGED", body
        assert body["ack_channel"] == "ros_topic"
        assert body["motion_verified"] is False
        assert len(stamps) >= 2
    finally:
        stop.set()
        thread.join(timeout=2)


def test_ros_topic_timeout_is_not_success_and_virtual_stays_simulated(env):
    client, _runtime = env
    settings = client.app.state.settings
    settings.fleet_dispatch_mode = "ros_topic"
    settings.ros_task_ack_timeout_s = 0.3
    settings.ros_task_max_attempts = 1
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1', start=START, goal=GOAL)['task_id']}/plan").json()
    failed = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch").json()
    assert failed["status"] == "FAILED"
    assert failed["ack_status"] == "NOT_RECEIVED"
    assert "SSH" not in (failed["failure_reason"] or "")
    virtual = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-2')['task_id']}/plan").json()
    delivered = client.post(f"/api/fleet-tasks/{virtual['task_id']}/dispatch").json()
    assert delivered["ack_channel"] == "simulated"
    assert delivered["ack"]["receiver"] == "fms-virtual-receiver"
