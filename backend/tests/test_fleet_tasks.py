"""Fleet task planning, reservations, and receive-only delivery."""
from __future__ import annotations

import json
import threading
from pathlib import Path

import pytest

from app.services.reservation_manager import Interval, find_conflicts
from robot_receiver.pinky_task_receiver import accept

ROOT = Path(__file__).resolve().parents[2]
START = {"x": 0.9, "y": 1.2}
GOAL = {"x": 1.2, "y": 0.8}


def _create(client, **extra):
    body = {
        "assignment_mode": "manual",
        "robot_id": "pinky-2",
        "start": START,
        "goal": GOAL,
        "priority": 5,
        "task_type": "navigate",
        "label": "test",
    }
    body.update(extra)
    response = client.post("/api/fleet-tasks", json=body)
    assert response.status_code == 200, response.text
    return response.json()


def test_preview_does_not_create_tasks_or_reservations(env):
    client, _runtime = env
    preview = client.post("/api/planning/routes/preview", json={
        "robot_id": "pinky-2", "start": START, "goal": GOAL,
    })
    assert preview.status_code == 200
    assert preview.json()["paths"]
    assert client.get("/api/fleet-tasks").json()["tasks"] == []
    assert client.get("/api/planning/reservations").json()["reservations"] == []


def test_manual_assignment_reserves_requested_robot(env):
    client, _runtime = env
    created = _create(client, robot_id="pinky-1")
    assert created["status"] == "CREATED"
    planned = client.post(f"/api/fleet-tasks/{created['task_id']}/plan")
    assert planned.status_code == 200, planned.text
    body = planned.json()
    assert body["assigned_robot_id"] == "pinky-1"
    assert body["status"] == "RESERVED"
    assert body["start"]["source"] == "request"
    assert body["planned_path"]
    assert body["candidate_paths"]
    assert body["actual_motion_verified"] is False
    assert body["schedule"]["collision_safety_verified"] is False
    reservations = client.get("/api/planning/reservations").json()["reservations"]
    assert reservations
    assert {item["occupancy_class"] for item in reservations} == {"planned"}


def test_invalid_registry_home_is_not_rewritten(env, tmp_path):
    client, _runtime = env
    live = ROOT / "config" / "physical_fleet.json"
    live_bytes = live.read_bytes()
    payload = json.loads(live_bytes)
    for item in payload["robots"]:
        if item["robot_id"] == "pinky-2":
            item["home"] = {"x": 0.4, "y": 1.9, "yaw": 0.0}
        if item["robot_id"] == "pinky-3":
            item["home"] = {"x": 0.4, "y": 0.5, "yaw": 1.5707963267948966}
    copy = tmp_path / "physical_fleet.json"
    copy.write_text(json.dumps(payload), encoding="utf-8")
    client.app.state.settings.physical_fleet_path = copy
    created = client.post("/api/fleet-tasks", json={
        "assignment_mode": "automatic",
        "goal": GOAL,
        "priority": 3,
        "task_type": "navigate",
    })
    assert created.status_code == 200
    planned = client.post(f"/api/fleet-tasks/{created.json()['task_id']}/plan")
    body = planned.json()
    assert body["status"] == "FAILED"
    assert body["assigned_robot_id"] is None
    assert json.loads(copy.read_text(encoding="utf-8")) == payload
    assert live.read_bytes() == live_bytes
    availability = client.get("/api/fleet-tasks/availability").json()["robots"]
    virtual = {item["robot_id"]: item for item in availability}
    assert virtual["pinky-2"]["assignable"] is False
    assert virtual["pinky-3"]["assignable"] is False
    assert virtual["pinky-2"]["ssh_online"] is False
    for robot_id in ("pinky-2", "pinky-3"):
        suggestions = virtual[robot_id]["inspection"]["suggestions"]
        assert suggestions
        assert all(item["replaces_registered_pose"] is False for item in suggestions)
        assert all(item["suggestion_only"] is True for item in suggestions)


def test_automatic_assignment_reports_cost(env):
    client, _runtime = env
    registry_before = (ROOT / "config" / "physical_fleet.json").read_bytes()
    first = _create(client, assignment_mode="automatic", robot_id=None, priority=5)
    second = _create(client, assignment_mode="automatic", robot_id=None, priority=9)
    planned_first = client.post(f"/api/fleet-tasks/{first['task_id']}/plan").json()
    planned_second = client.post(f"/api/fleet-tasks/{second['task_id']}/plan").json()
    assert planned_first["status"] == "RESERVED"
    assert "score" in (planned_first["assignment_reason"] or "")
    assert planned_second["assigned_robot_id"] in {"pinky-1", "pinky-2", "pinky-3"}
    assert planned_second["assignment_reason"]
    assert (ROOT / "config" / "physical_fleet.json").read_bytes() == registry_before


def test_vertex_conflict():
    proposed = Interval("pinky-1", "a", "vertex", "v-1-1", 1.0, 1.0, 0.0, 1.0, (1, 1))
    existing = Interval("pinky-2", "b", "vertex", "v-1-1", 1.0, 1.0, 0.0, 1.0, (1, 1))
    kinds = {item["conflict_type"] for item in find_conflicts([proposed], [existing], robot_radius_m=0)}
    assert "vertex" in kinds
    touching = Interval("pinky-2", "b", "vertex", "v-1-1", 1.0, 1.0, 1.0, 2.0, (1, 1))
    assert find_conflicts([proposed], [touching], robot_radius_m=0) == []


def test_edge_conflict():
    forward = Interval("pinky-1", "a", "edge", "e-0,0>1,0", 0.5, 0.0, 0.0, 1.0)
    backward = Interval("pinky-2", "b", "edge", "e-1,0>0,0", 0.5, 0.0, 0.0, 1.0)
    same_direction = Interval("pinky-2", "b", "edge", "e-0,0>1,0", 0.5, 0.0, 0.0, 1.0)
    assert any(item["conflict_type"] == "edge" for item in find_conflicts([forward], [backward], robot_radius_m=0))
    assert find_conflicts([forward], [same_direction], robot_radius_m=0) == []


def test_crossing_conflict():
    diagonal = Interval("pinky-1", "a", "edge", "e-0,0>1,1", 0.5, 0.5, 0.0, 1.0)
    other = Interval("pinky-2", "b", "edge", "e-1,0>0,1", 0.5, 0.5, 0.0, 1.0)
    assert any(item["conflict_type"] == "crossing" for item in find_conflicts([diagonal], [other], robot_radius_m=0))


def test_footprint_conflict():
    left = Interval("pinky-1", "a", "vertex", "v-1-1", 1.0, 1.0, 0.0, 1.0, (1, 1))
    right = Interval("pinky-2", "b", "vertex", "v-2-1", 1.05, 1.0, 0.0, 1.0, (2, 1))
    kinds = {item["conflict_type"] for item in find_conflicts([left], [right], robot_radius_m=0.06)}
    assert "footprint" in kinds
    assert find_conflicts([left], [right], robot_radius_m=0) == []


def test_waiting_conflict():
    passer = Interval("pinky-1", "a", "vertex", "v-1-1", 1.0, 1.0, 0.0, 1.0, (1, 1))
    dweller = Interval("pinky-2", "b", "vertex", "v-1-1", 1.0, 1.0, 0.0, 2.0, (1, 1))
    kinds = {item["conflict_type"] for item in find_conflicts([passer], [dweller], robot_radius_m=0)}
    assert {"vertex", "waiting"} <= kinds


def test_second_robot_does_not_share_the_same_reservation(env):
    client, _runtime = env
    first = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
    second = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-2')['task_id']}/plan").json()
    assert first["status"] == "RESERVED"
    assert second["status"] in {"RESERVED", "WAITING"}
    if second["status"] == "RESERVED":
        assert second["schedule"]["depart_s"] > 0 or second["selected_path_id"] != first["selected_path_id"]
    report = client.get("/api/planning/conflicts").json()
    assert report["conflicts"] == []
    assert report["collision_safety_verified"] is False


def test_cancel_releases_reservations(env):
    client, _runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client)['task_id']}/plan").json()
    assert client.get("/api/planning/reservations").json()["reservations"]
    cancelled = client.post(f"/api/fleet-tasks/{task['task_id']}/cancel")
    assert cancelled.json()["status"] == "CANCELLED"
    assert client.get("/api/planning/reservations").json()["reservations"] == []


def test_select_conflict_keeps_existing_reservation(env, monkeypatch):
    client, _runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client)['task_id']}/plan").json()
    before = client.get("/api/planning/reservations").json()["reservations"]
    monkeypatch.setattr("app.services.fleet_task_service.schedule_candidates", lambda *args, **kwargs: None)
    rejected = client.post(f"/api/fleet-tasks/{task['task_id']}/select", json={"path_id": task["selected_path_id"]})
    assert rejected.status_code == 409
    after = client.get("/api/planning/reservations").json()["reservations"]
    assert [item["reservation_id"] for item in after] == [item["reservation_id"] for item in before]


def test_concurrent_plans_do_not_duplicate_a_reservation(env):
    client, runtime = env
    from app.services.fleet_task_service import FleetTaskService

    service = FleetTaskService(client.app.state.settings, runtime.db, lambda: None)
    ids = [
        service.create(mode="manual", goal_x=GOAL["x"], goal_y=GOAL["y"], robot_id=robot,
                       start=START, priority=5, task_type="navigate", label="race", idempotency_key=None)["task_id"]
        for robot in ("pinky-1", "pinky-2")
    ]
    results = []
    errors = []

    def run(task_id: str) -> None:
        try:
            results.append(service.plan(task_id))
        except Exception as exc:  # noqa: BLE001 - the assertion reports the real failure
            errors.append(exc)

    threads = [threading.Thread(target=run, args=(task_id,)) for task_id in ids]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=30)
    assert errors == []
    assert len(results) == 2
    report = service.conflict_report()
    assert report["conflicts"] == []


def test_virtual_dispatch_is_not_a_physical_ack(env):
    client, _runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-2')['task_id']}/plan").json()
    delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch")
    assert delivered.status_code == 200, delivered.text
    body = delivered.json()
    assert body["status"] == "DELIVERED"
    assert body["ack_status"] == "SIMULATED_RECEIPT"
    assert body["ack_channel"] == "simulated"
    assert body["ack"]["receiver"] == "fms-virtual-receiver"
    assert body["status"] != "ACKNOWLEDGED"


def test_local_physical_receiver_ack_matches_hash_and_does_not_move(env):
    client, _runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
    delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch")
    assert delivered.status_code == 200, delivered.text
    body = delivered.json()
    assert body["status"] == "ACKNOWLEDGED"
    assert body["ack_channel"] == "physical"
    assert body["ack"]["receiver"] == "pinky-fms-receiver"
    assert body["ack"]["task_id"] == task["task_id"]
    assert body["ack"]["payload_hash"] == body["payload_hash"]
    assert body["ack"]["execute"] is False
    stored = json.loads((client.app.state.settings.data_dir / "physical_inbox" / f"{task['task_id']}.json").read_text())
    assert stored["payload_hash"] == body["payload_hash"]
    assert stored["payload"]["execute"] is False
    again = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch")
    assert again.json()["ack"]["received_at"] == body["ack"]["received_at"]


def test_receiver_is_idempotent_and_rejects_a_different_payload(tmp_path):
    payload = {"task_id": "t-1", "robot_id": "pinky-1", "execute": False, "goal": {"x": 1}}
    first = accept(tmp_path, payload)
    second = accept(tmp_path, payload)
    assert second == first
    changed = {"task_id": "t-1", "robot_id": "pinky-1", "execute": False, "goal": {"x": 2}}
    with pytest.raises(ValueError):
        accept(tmp_path, changed)
    drive = accept(tmp_path, {"task_id": "t-2", "robot_id": "pinky-1", "execute": True})
    assert drive["execute"] is True
    assert drive["accepted"] is True


def test_dispatch_without_reservation_is_rejected(env):
    client, _runtime = env
    created = _create(client, robot_id="pinky-1")
    rejected = client.post(f"/api/fleet-tasks/{created['task_id']}/dispatch")
    assert rejected.status_code == 409
    assert client.get(f"/api/fleet-tasks/{created['task_id']}").json()["status"] == "CREATED"


def test_ssh_without_password_fails_closed(env):
    client, _runtime = env
    client.app.state.settings.fleet_dispatch_mode = "ssh"
    client.app.state.settings.physical_ssh_targets = "pinky-1=pinky@192.168.0.83"
    client.app.state.settings.physical_ssh_password = ""
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
    delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch")
    body = delivered.json()
    assert body["status"] == "FAILED"
    assert body["ack_status"] == "NOT_RECEIVED"
    assert "password" not in body["failure_reason"].lower() or "not configured" in body["failure_reason"].lower()
    assert client.get("/api/planning/reservations").json()["reservations"] == []


def test_restart_does_not_treat_dispatching_as_delivered(env):
    client, runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
    from app.models.tables import FleetTaskRecord

    with runtime.db.write() as session:
        row = session.get(FleetTaskRecord, task["task_id"])
        row.status = "DISPATCHING"
    client.app.state.fleet_tasks._recovered = False
    recovered = client.get(f"/api/fleet-tasks/{task['task_id']}").json()
    assert recovered["status"] == "FAILED"
    assert recovered["failure_reason"] == "dispatch interrupted by process restart"
    assert recovered["status"] != "ACKNOWLEDGED"


def test_stale_map_revision_requires_replan(env):
    client, runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client)['task_id']}/plan").json()
    from app.models.tables import FleetTaskRecord

    with runtime.db.write() as session:
        session.get(FleetTaskRecord, task["task_id"]).map_revision = "stale-revision"
    client.app.state.fleet_tasks._recovered = False
    recovered = client.get(f"/api/fleet-tasks/{task['task_id']}").json()
    assert recovered["status"] == "REPLANNING"
    assert client.get("/api/planning/reservations").json()["reservations"] == []


def test_facilities_without_coordinates_are_not_selectable(env):
    client, _runtime = env
    payload = client.get("/api/fleet-tasks/facilities").json()
    assert payload["facilities"]
    assert all(item["selectable"] is False and item["x"] is None for item in payload["facilities"])


def test_home_approval_writes_only_after_validation(env, tmp_path):
    client, _runtime = env
    source = ROOT / "config" / "physical_fleet.json"
    copy = tmp_path / "physical_fleet.json"
    before_source = source.read_bytes()
    copy.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
    before_copy = copy.read_text(encoding="utf-8")
    client.app.state.settings.physical_fleet_path = copy
    refused = client.post("/api/fleet-tasks/robots/pinky-2/home", json={"x": 0.375, "y": 1.925, "approve": False})
    assert refused.status_code == 422
    assert copy.read_text(encoding="utf-8") == before_copy
    approved = client.post("/api/fleet-tasks/robots/pinky-2/home", json={"x": 0.375, "y": 1.925, "yaw": 0, "approve": True})
    assert approved.status_code == 200, approved.text
    assert json.loads(copy.read_text())["robots"][1]["home"]["x"] == 0.375
    assert source.read_bytes() == before_source
    blocked = client.post("/api/fleet-tasks/robots/pinky-1/home", json={"x": 0.425, "y": 1.925, "approve": True})
    assert blocked.status_code == 404


def test_goal_outside_map_is_rejected_by_inspect(env):
    client, _runtime = env
    report = client.post("/api/planning/points/inspect", json={"x": 9, "y": 9})
    assert report.status_code == 200
    assert report.json()["accepted_as_goal"] is False


def test_ack_timeout_does_not_acknowledge(env, monkeypatch):
    import subprocess

    client, _runtime = env

    def hang(*args, **kwargs):
        raise subprocess.TimeoutExpired(cmd="receiver", timeout=0.01)

    monkeypatch.setattr("app.services.fleet_dispatcher.subprocess.run", hang)
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
    delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch")
    body = delivered.json()
    assert delivered.status_code == 200
    assert body["status"] == "FAILED"
    assert body["ack_status"] == "NOT_RECEIVED"
    assert body["status"] != "ACKNOWLEDGED"
    assert "timed out" in body["failure_reason"].lower()
    assert client.get("/api/planning/reservations").json()["reservations"] == []


def test_wrong_ack_hash_is_rejected(env, monkeypatch):
    client, _runtime = env

    def bad_stdout(*args, **kwargs):
        payload = json.loads(kwargs["input"])
        class _Result:
            returncode = 0
            stdout = json.dumps({
                "accepted": True,
                "task_id": payload["task_id"],
                "payload_hash": "0" * 64,
                "receiver": "pinky-fms-receiver",
                "execute": False,
            })
            stderr = ""
        return _Result()

    monkeypatch.setattr("app.services.fleet_dispatcher.subprocess.run", bad_stdout)
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
    delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch")
    body = delivered.json()
    assert body["status"] == "FAILED"
    assert body["ack_status"] == "NOT_RECEIVED"
    assert "hash" in body["failure_reason"].lower()


def test_repeated_virtual_dispatch_does_not_send_twice(env):
    client, _runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-2')['task_id']}/plan").json()
    first = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch").json()
    second = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch").json()
    assert first["ack_channel"] == "simulated"
    assert second["ack"]["received_at"] == first["ack"]["received_at"]
    assert second["status"] == "DELIVERED"


def test_concurrent_idempotency_key_keeps_one_task(env):
    client, runtime = env
    from app.services.fleet_task_service import FleetTaskService

    service = FleetTaskService(client.app.state.settings, runtime.db, lambda: None)
    results = []
    errors = []

    def run() -> None:
        try:
            results.append(service.create(
                mode="manual", goal_x=GOAL["x"], goal_y=GOAL["y"], robot_id="pinky-2",
                start=START, priority=5, task_type="navigate", label="once",
                idempotency_key="same-create",
            ))
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)

    threads = [threading.Thread(target=run) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=30)
    assert errors == []
    assert len({item["task_id"] for item in results}) == 1


def test_acknowledged_task_survives_recovery_without_completion(env):
    client, _runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
    delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch").json()
    assert delivered["status"] == "ACKNOWLEDGED"
    client.app.state.fleet_tasks._recovered = False
    recovered = client.get(f"/api/fleet-tasks/{task['task_id']}").json()
    assert recovered["status"] == "ACKNOWLEDGED"
    assert recovered["actual_motion_verified"] is False
    assert recovered.get("status") != "COMPLETED"


def test_five_cm_status_get_does_not_start_motion(env):
    client, _runtime = env
    response = client.get("/api/physical/tests/pinky-1/forward-5cm")
    assert response.status_code == 200
    assert response.json()["state"] == "disabled"


def test_five_cm_service_no_longer_imports_removed_map_helper():
    source = (ROOT / "backend" / "app" / "services" / "forward_5cm_service.py").read_text(encoding="utf-8")
    dispatcher = (ROOT / "backend" / "app" / "services" / "fleet_dispatcher.py").read_text(encoding="utf-8")
    assert "_map_files" not in source
    assert "load_geometry" in source
    assert "forward_5cm" not in dispatcher
    assert 'command = "python3 ~/fms_task_receiver/pinky_task_receiver.py ~/fms_task_receiver/inbox"' in dispatcher
