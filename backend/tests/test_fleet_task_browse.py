"""Paged task lists and archive flags. Archive never cancels or deletes a task."""
from __future__ import annotations

import json
from pathlib import Path

from fastapi.testclient import TestClient

from app.main import create_app

START = {"x": 0.625, "y": 0.075}
GOAL = {"x": 0.625, "y": 0.475}
OTHER_START = {"x": 0.625, "y": 1.025}
OTHER_GOAL = {"x": 0.625, "y": 1.575}


def _create(client, **extra):
    body = {
        "assignment_mode": "manual",
        "robot_id": "pinky-2",
        "goal": {"x": 1.2, "y": 1.2},
        "priority": 5,
        "task_type": "navigate",
        "label": extra.pop("label", "browse"),
    }
    body.update(extra)
    response = client.post("/api/fleet-tasks", json=body)
    assert response.status_code == 200, response.text
    return response.json()


def _reservations(client, task_id: str) -> list[str]:
    rows = client.get("/api/planning/reservations").json()["reservations"]
    return [row["reservation_id"] for row in rows if row["task_id"] == task_id]


def test_active_and_history_are_separate(env):
    client, _runtime = env
    created = _create(client, robot_id="pinky-2", label="still-open")
    planned = client.post(
        f"/api/fleet-tasks/{_create(client, robot_id='pinky-3', start=OTHER_START, goal=OTHER_GOAL)['task_id']}/plan"
    ).json()
    cancelled = client.post(f"/api/fleet-tasks/{created['task_id']}/cancel").json()
    active = client.get("/api/fleet-tasks/browse", params={"scope": "active"}).json()
    history = client.get("/api/fleet-tasks/browse", params={"scope": "history"}).json()
    assert planned["task_id"] in {row["task_id"] for row in active["tasks"]}
    assert cancelled["task_id"] not in {row["task_id"] for row in active["tasks"]}
    assert history["tasks"][0]["task_id"] == cancelled["task_id"]
    assert history["tasks"][0]["status"] == "CANCELLED"
    assert client.get("/api/fleet-tasks").json()["tasks"]


def test_filters_search_and_pages(env):
    client, _runtime = env
    created = []
    for index in range(12):
        robot = "pinky-3" if index == 0 else "pinky-2"
        created.append(_create(client, robot_id=robot, label=f"page-{index}"))
    newest = created[-1]["task_id"]
    first_page = client.get("/api/fleet-tasks/browse", params={"scope": "active", "page_size": 10}).json()
    second_page = client.get("/api/fleet-tasks/browse", params={"scope": "active", "page": 2, "page_size": 10}).json()
    assert first_page["total"] == 12
    assert len(first_page["tasks"]) == 10
    assert first_page["tasks"][0]["task_id"] == newest
    assert len(second_page["tasks"]) == 2
    assert client.get("/api/fleet-tasks/browse", params={"page_size": 15}).status_code == 400
    pinky3 = client.get("/api/fleet-tasks/browse", params={"scope": "active", "robot_id": "pinky-3"}).json()
    assert pinky3["total"] == 1
    pinky3_row = pinky3["tasks"][0]
    assert (pinky3_row["assigned_robot_id"] or pinky3_row["requested_robot_id"]) == "pinky-3"
    found = client.get("/api/fleet-tasks/browse", params={"scope": "active", "q": newest}).json()
    assert [row["task_id"] for row in found["tasks"]] == [newest]
    reserved = client.post(f"/api/fleet-tasks/{created[1]['task_id']}/plan").json()
    only_reserved = client.get("/api/fleet-tasks/browse", params={"scope": "active", "status": "RESERVED"}).json()
    assert [row["task_id"] for row in only_reserved["tasks"]] == [reserved["task_id"]]
    assert only_reserved["tasks"][0]["active_reservation_count"] > 0


def test_cancel_releases_only_that_reservation(env):
    client, _runtime = env
    first = client.post(
        f"/api/fleet-tasks/{_create(client, robot_id='pinky-2', start=START, goal=GOAL)['task_id']}/plan"
    ).json()
    second = client.post(
        f"/api/fleet-tasks/{_create(client, robot_id='pinky-3', start=OTHER_START, goal=OTHER_GOAL)['task_id']}/plan"
    ).json()
    kept = _reservations(client, second["task_id"])
    cancelled = client.post(f"/api/fleet-tasks/{first['task_id']}/cancel", json={"reason": "operator"}).json()
    assert cancelled["status"] == "CANCELLED"
    assert _reservations(client, first["task_id"]) == []
    assert _reservations(client, second["task_id"]) == kept
    assert client.get(f"/api/fleet-tasks/{second['task_id']}").json()["status"] == "RESERVED"


def test_archive_hides_without_deleting_or_cancelling(env):
    client, runtime = env
    task = client.post(
        f"/api/fleet-tasks/{_create(client, robot_id='pinky-1', start=START, goal=GOAL)['task_id']}/plan"
    ).json()
    delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch").json()
    assert delivered["status"] == "ACKNOWLEDGED"
    store = Path(client.app.state.settings.data_dir) / "physical_inbox" / f"{task['task_id']}.json"
    before = store.read_text(encoding="utf-8")
    refused = client.post(f"/api/fleet-tasks/{task['task_id']}/archive")
    assert refused.status_code == 409
    assert store.read_text(encoding="utf-8") == before
    assert "cancellation" not in json.loads(before)
    active = client.get("/api/fleet-tasks/browse", params={"scope": "active", "status": "ACKNOWLEDGED"}).json()
    assert task["task_id"] in {row["task_id"] for row in active["tasks"]}

    reserved = client.post(
        f"/api/fleet-tasks/{_create(client, robot_id='pinky-2', start=OTHER_START, goal=OTHER_GOAL)['task_id']}/plan"
    ).json()
    kept = _reservations(client, reserved["task_id"])
    assert client.post(f"/api/fleet-tasks/{reserved['task_id']}/archive").status_code == 409
    assert _reservations(client, reserved["task_id"]) == kept

    cancelled = client.post(f"/api/fleet-tasks/{reserved['task_id']}/cancel").json()
    hidden = client.post(
        f"/api/fleet-tasks/{reserved['task_id']}/archive",
        json={"archived_by": "operator"},
    ).json()
    assert hidden["status"] == "CANCELLED"
    assert hidden["is_archived"] is True
    assert hidden["cancellation"]["outcome"] == cancelled["cancellation"]["outcome"]
    assert client.get("/api/fleet-tasks/browse", params={"scope": "history"}).json()["total"] == 0
    archived = client.get("/api/fleet-tasks/browse", params={"scope": "archived"}).json()
    assert archived["tasks"][0]["task_id"] == reserved["task_id"]
    listed = client.get("/api/fleet-tasks").json()["tasks"]
    assert any(row["task_id"] == reserved["task_id"] and row["is_archived"] for row in listed)

    restored = client.post(f"/api/fleet-tasks/{reserved['task_id']}/restore").json()
    assert restored["is_archived"] is False
    assert restored["status"] == "CANCELLED"
    assert client.get("/api/fleet-tasks/browse", params={"scope": "history"}).json()["total"] == 1

    failed = _create(client, robot_id="pinky-3")
    with runtime.db.write() as session:
        from app.models.tables import FleetTaskRecord
        row = session.get(FleetTaskRecord, failed["task_id"])
        row.status = "FAILED"
        row.failure_reason = "goal occupied"
    assert client.post(f"/api/fleet-tasks/{failed['task_id']}/cancel").status_code == 409
    failed_hidden = client.post(f"/api/fleet-tasks/{failed['task_id']}/archive").json()
    assert failed_hidden["status"] == "FAILED"
    assert failed_hidden["failure_reason"] == "goal occupied"

    pending = _create(client, robot_id="pinky-2")
    with runtime.db.write() as session:
        from app.models.tables import FleetTaskRecord
        row = session.get(FleetTaskRecord, pending["task_id"])
        row.status = "CANCEL_PENDING"
    assert client.get("/api/fleet-tasks/browse", params={"scope": "active", "status": "CANCEL_PENDING"}).json()["total"] == 1
    assert client.post(f"/api/fleet-tasks/{pending['task_id']}/archive").status_code == 409


def test_archive_survives_restart(env):
    client, runtime = env
    created = _create(client)
    client.post(f"/api/fleet-tasks/{created['task_id']}/cancel")
    client.post(f"/api/fleet-tasks/{created['task_id']}/archive", json={"archived_by": "operator"})
    runtime.shutdown()
    with TestClient(create_app(client.app.state.settings)) as restarted:
        row = restarted.get(f"/api/fleet-tasks/{created['task_id']}").json()
        assert row["status"] == "CANCELLED"
        assert row["is_archived"] is True
        assert row["archived_by"] == "operator"
        archived = restarted.get("/api/fleet-tasks/browse", params={"scope": "archived"}).json()
        assert archived["total"] == 1
        assert archived["tasks"][0]["task_id"] == created["task_id"]
