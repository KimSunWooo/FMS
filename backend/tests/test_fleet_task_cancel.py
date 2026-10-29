"""Cancellation releases only the selected task and waits for a receiver cancel ACK."""
from __future__ import annotations

import json
import subprocess
import threading
from pathlib import Path

from app.models.tables import FleetTaskRecord
from app.services.fleet_task_service import FleetTaskService
from app.services.velocity_chain import classify_velocity, goal_status_name, stop_zone_block_reason

START = {"x": 0.625, "y": 0.075}
GOAL = {"x": 0.625, "y": 0.475}
OTHER_START = {"x": 0.625, "y": 1.025}
OTHER_GOAL = {"x": 0.625, "y": 1.575}


def _create(client, **extra):
    body = {
        "assignment_mode": "manual",
        "robot_id": "pinky-2",
        "start": START,
        "goal": GOAL,
        "priority": 5,
        "task_type": "navigate",
        "label": "cancel-test",
    }
    body.update(extra)
    response = client.post("/api/fleet-tasks", json=body)
    assert response.status_code == 200, response.text
    return response.json()


def _ids(client, task_id: str) -> list[str]:
    rows = client.get("/api/planning/reservations").json()["reservations"]
    return [row["reservation_id"] for row in rows if row["task_id"] == task_id]


def _force(runtime, task_id: str, status: str, reason: str = "") -> None:
    with runtime.db.write() as session:
        task = session.get(FleetTaskRecord, task_id)
        task.status = status
        if reason:
            task.failure_reason = reason


def test_created_planned_and_waiting_cancel(env):
    client, runtime = env
    created = _create(client)
    cancelled = client.post(f"/api/fleet-tasks/{created['task_id']}/cancel")
    assert cancelled.status_code == 200
    assert cancelled.json()["status"] == "CANCELLED"
    assert _ids(client, created["task_id"]) == []

    planned = client.post(f"/api/fleet-tasks/{_create(client)['task_id']}/plan").json()
    _force(runtime, planned["task_id"], "PLANNED")
    assert client.post(f"/api/fleet-tasks/{planned['task_id']}/cancel").json()["status"] == "CANCELLED"

    waiting = client.post(f"/api/fleet-tasks/{_create(client)['task_id']}/plan").json()
    assert waiting["status"] == "RESERVED"
    owned = _ids(client, waiting["task_id"])
    assert owned
    _force(runtime, waiting["task_id"], "WAITING")
    body = client.post(f"/api/fleet-tasks/{waiting['task_id']}/cancel").json()
    assert body["status"] == "CANCELLED"
    assert body["cancellation"]["reservations_released"] == len(owned)
    assert _ids(client, waiting["task_id"]) == []


def test_reserved_cancel_keeps_the_other_robot(env):
    client, _runtime = env
    first = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-2')['task_id']}/plan").json()
    second = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-3', start=OTHER_START, goal=OTHER_GOAL)['task_id']}/plan").json()
    assert first["status"] == "RESERVED"
    assert second["status"] == "RESERVED"
    before = _ids(client, second["task_id"])
    cancelled = client.post(f"/api/fleet-tasks/{first['task_id']}/cancel", json={"reason": "operator"})
    assert cancelled.json()["status"] == "CANCELLED"
    assert _ids(client, first["task_id"]) == []
    assert _ids(client, second["task_id"]) == before
    assert client.get(f"/api/fleet-tasks/{second['task_id']}").json()["planned_path"]


def test_duplicate_cancel_is_idempotent_and_blocks_dispatch(env):
    client, _runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client)['task_id']}/plan").json()
    first = client.post(f"/api/fleet-tasks/{task['task_id']}/cancel").json()
    second = client.post(f"/api/fleet-tasks/{task['task_id']}/cancel").json()
    assert first["status"] == second["status"] == "CANCELLED"
    assert first["cancellation"]["cancellation_request_id"] == second["cancellation"]["cancellation_request_id"]
    rejected = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch")
    assert rejected.status_code == 409


def test_failure_history_is_not_cancelled(env):
    client, runtime = env
    task = _create(client)
    _force(runtime, task["task_id"], "FAILED", "goal occupied")
    rejected = client.post(f"/api/fleet-tasks/{task['task_id']}/cancel")
    assert rejected.status_code == 409
    kept = client.get(f"/api/fleet-tasks/{task['task_id']}").json()
    assert kept["status"] == "FAILED"
    assert kept["failure_reason"] == "goal occupied"


def test_cancel_during_dispatch_does_not_acknowledge(env, monkeypatch):
    client, runtime = env
    service = FleetTaskService(client.app.state.settings, runtime.db, lambda: None)
    created = service.create(
        mode="manual", goal_x=GOAL["x"], goal_y=GOAL["y"], robot_id="pinky-1",
        start=START, priority=5, task_type="navigate", label="race", idempotency_key=None,
    )
    planned = service.plan(created["task_id"])
    started = threading.Event()
    release = threading.Event()

    def slow(*_args, **kwargs):
        payload = json.loads(kwargs["input"])

        class Result:
            returncode = 0
            stderr = ""

        result = Result()
        if payload.get("action") == "cancel":
            result.stdout = json.dumps({
                "kind": "cancel_ack",
                "accepted": True,
                "cancelled": True,
                "task_id": payload["task_id"],
                "robot_id": payload["robot_id"],
                "cancellation_request_id": payload["cancellation_request_id"],
                "execute": False,
            })
            return result
        started.set()
        assert release.wait(5)
        result.stdout = json.dumps({
            "accepted": True,
            "task_id": payload["task_id"],
            "robot_id": payload["robot_id"],
            "payload_hash": payload["payload_hash"],
            "execute": False,
        })
        return result

    monkeypatch.setattr("app.services.fleet_dispatcher.subprocess.run", slow)
    box: dict = {}

    def run() -> None:
        box["body"] = service.dispatch(planned["task_id"])

    thread = threading.Thread(target=run)
    thread.start()
    assert started.wait(5)
    service.cancel(planned["task_id"], "race")
    release.set()
    thread.join(5)
    final = service.get(planned["task_id"])
    assert final["status"] == "CANCELLED"
    assert final["ack_status"] == "CANCEL_ACKNOWLEDGED"
    assert final["cancellation"]["receiver"]["kind"] == "cancel_ack"
    assert _ids(client, planned["task_id"]) == []


def test_physical_cancel_ack_and_communication_failure(env, monkeypatch):
    client, runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
    delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch")
    assert delivered.status_code == 200
    assert delivered.json()["status"] == "ACKNOWLEDGED"
    store = Path(client.app.state.settings.data_dir) / "physical_inbox" / f"{task['task_id']}.json"
    assert store.exists()

    def boom(*_args, **_kwargs):
        raise subprocess.TimeoutExpired(cmd="receiver", timeout=0.01)

    monkeypatch.setattr("app.services.fleet_dispatcher.subprocess.run", boom)
    pending = client.post(f"/api/fleet-tasks/{task['task_id']}/cancel", json={"reason": "link down"})
    assert pending.status_code == 200
    assert pending.json()["status"] == "CANCEL_PENDING"
    assert pending.json()["ack_status"] != "CANCEL_ACKNOWLEDGED"
    stored = json.loads(store.read_text(encoding="utf-8"))
    assert "cancellation" not in stored

    monkeypatch.undo()
    finished = client.post(f"/api/fleet-tasks/{task['task_id']}/cancel")
    body = finished.json()
    assert body["status"] == "CANCELLED"
    assert body["ack_status"] == "CANCEL_ACKNOWLEDGED"
    assert body["cancellation"]["receiver"]["kind"] == "cancel_ack"
    assert body["ack"]["payload_hash"] == delivered.json()["ack"]["payload_hash"]
    stored = json.loads(store.read_text(encoding="utf-8"))
    assert stored["executable"] is False
    assert stored["cancellation"]["kind"] == "cancel_ack"
    assert stored["ack"]["payload_hash"] == stored["payload_hash"]
    again = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch")
    assert again.status_code == 409

    fresh = FleetTaskService(client.app.state.settings, runtime.db, lambda: None)
    recovered = fresh.get(task["task_id"])
    assert recovered["status"] == "CANCELLED"
    assert recovered["cancellation"]["receiver"]["kind"] == "cancel_ack"


def test_virtual_cancel_is_not_a_physical_ack(env):
    client, _runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-2')['task_id']}/plan").json()
    delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch").json()
    assert delivered["ack_channel"] == "simulated"
    cancelled = client.post(f"/api/fleet-tasks/{task['task_id']}/cancel").json()
    assert cancelled["status"] == "CANCELLED"
    assert cancelled["cancellation"]["receiver"]["channel"] == "simulated"
    assert cancelled["ack_status"] != "CANCEL_ACKNOWLEDGED"


def test_velocity_cases_are_distinct_from_motion():
    assert classify_velocity(nav_linear_x=0.04, smoothed_linear_x=0.04, cmd_linear_x=0.0, cmd_seen=True) == "A"
    assert classify_velocity(nav_linear_x=0.04, smoothed_linear_x=0.04, cmd_linear_x=None, cmd_seen=False) == "B"
    assert classify_velocity(nav_linear_x=0.04, smoothed_linear_x=0.04, cmd_linear_x=0.04, cmd_seen=True) == "C"
    assert classify_velocity(nav_linear_x=None, smoothed_linear_x=None, cmd_linear_x=None, cmd_seen=False) is None
    assert goal_status_name(6) == "ABORTED"
    assert goal_status_name(4) == "SUCCEEDED"
    blocked = stop_zone_block_reason({
        "stop_zone_would_stop": True,
        "scan": {"received": True, "points_within_stop_radius": 46, "stop_radius_m": 0.18, "min_range_m": 0.102},
    })
    assert blocked and "not sent" in blocked
    assert stop_zone_block_reason({"stop_zone_would_stop": False, "scan": {"received": True}}) is None
    assert stop_zone_block_reason({"scan": {"received": False}})
    failed = stop_zone_block_reason({
        "stop_zone_would_stop": False,
        "scan": {"received": True, "transform_status": "failed", "transform_error": "lookup failed"},
    })
    assert failed and "not sent" in failed
