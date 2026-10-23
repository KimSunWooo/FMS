from concurrent.futures import ThreadPoolExecutor

import pytest
from sqlalchemy.exc import IntegrityError

from app.core.state_machine import TaskStatus
from app.models.tables import Task


def _running(client, runtime, *, robot_id="pinky-3"):
    runtime.settings.mock_goal_duration_seconds = 30
    created = client.post(
        "/api/tasks",
        json={
            "destination": {"x": 4.0, "y": 1.5, "label": "west-bay"},
            "robot_id": robot_id,
        },
    )
    task_id = created.json()["task"]["task_id"]
    runtime.tick()
    task = client.get(f"/api/tasks/{task_id}").json()
    assert task["status"] == "RUNNING"
    assert task["navigation_goal_count"] == 1
    return task


def test_one_robot_cannot_take_two_active_tasks(env):
    client, runtime = env
    runtime.settings.mock_goal_duration_seconds = 30
    first = client.post(
        "/api/tasks",
        json={"destination": {"x": 4.0, "y": 1.5, "label": "west-bay"}, "robot_id": "pinky-3"},
    )
    second = client.post(
        "/api/tasks",
        json={"destination": {"x": 6.0, "y": 2.0, "label": "south-bay"}, "robot_id": "pinky-3"},
    )
    runtime.tick()
    statuses = {
        item["task_id"]: client.get(f"/api/tasks/{item['task_id']}").json()["status"]
        for item in (first.json()["task"], second.json()["task"])
    }
    assert list(statuses.values()).count("RUNNING") == 1
    assert "PENDING" in statuses.values()


def test_parallel_schedule_keeps_a_single_owner(env):
    client, runtime = env
    runtime.settings.mock_goal_duration_seconds = 30
    ids = []
    for label, y in (("west-bay", 1.5), ("south-bay", 2.0)):
        response = client.post(
            "/api/tasks",
            json={"destination": {"x": 4.0, "y": y, "label": label}, "robot_id": "pinky-3"},
        )
        ids.append(response.json()["task"]["task_id"])

    def once():
        with runtime.db.write() as session:
            runtime.scheduler.schedule_pending(session, runtime.clock.now())

    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(lambda _: once(), range(2)))

    bodies = [client.get(f"/api/tasks/{task_id}").json() for task_id in ids]
    active = [body for body in bodies if body["status"] in {"ASSIGNED", "DISPATCHING", "RUNNING"}]
    assert len(active) == 1


def test_database_constraint_rejects_a_second_active_row(env):
    client, runtime = env
    running = _running(client, runtime)
    with pytest.raises(IntegrityError):
        with runtime.db.write() as session:
            session.add(
                Task(
                    id="manual-second",
                    idempotency_key=None,
                    robot_id=running["robot_id"],
                    requested_robot_id=running["robot_id"],
                    task_type="navigate",
                    status=TaskStatus.ASSIGNED.value,
                    destination_x=6,
                    destination_y=2,
                    destination_yaw=0,
                    destination_label="illegal",
                    priority=0,
                    owner_id="intruder",
                    owner_epoch=1,
                    lease_id=None,
                    lease_expires_at=None,
                    execution_id=None,
                    goal_id=None,
                    session_id=None,
                    state_version=1,
                    failure_reason=None,
                    recovery_reason=None,
                    goal_may_be_in_flight=False,
                    dispatch_attempts=0,
                    not_before=None,
                    last_decision_key=None,
                    created_at=runtime.clock.now(),
                    updated_at=runtime.clock.now(),
                    last_heartbeat_at=None,
                    cancel_requested_at=None,
                    completed_at=None,
                )
            )
            session.flush()
    stored = client.get(f"/api/tasks/{running['task_id']}").json()
    assert stored["status"] == "RUNNING"


def test_stale_epoch_cannot_complete(env):
    client, runtime = env
    running = _running(client, runtime)
    stale = runtime.apply_external_result(
        task_id=running["task_id"],
        goal_id=running["goal_id"],
        execution_id=running["execution_id"],
        owner_epoch=running["owner_epoch"] - 1,
        session_id=running["session_id"],
        outcome="succeeded",
        pose_x=4.0,
        pose_y=1.5,
        pose_yaw=0,
        pose_trustworthy=True,
        source="delayed-agent",
    )
    assert stale["status"] == "RUNNING"
    assert stale["state_version"] == running["state_version"]
    assert stale["goal"]["last_result_disposition"] == "rejected_stale_owner"


def test_assign_transaction_rolls_back(env):
    client, runtime = env
    created = client.post(
        "/api/tasks",
        json={"destination": {"x": 4.0, "y": 1.5, "label": "west-bay"}, "robot_id": "pinky-3"},
    )
    task_id = created.json()["task"]["task_id"]
    try:
        with runtime.db.write() as session:
            runtime.scheduler.schedule_pending(session, runtime.clock.now())
            raise RuntimeError("disk failed")
    except RuntimeError:
        pass
    task = client.get(f"/api/tasks/{task_id}").json()
    assert task["status"] == "PENDING"
    assert task["robot_id"] is None
    assert task["navigation_goal_count"] == 0


def test_pose_mismatch_does_not_complete(env):
    client, runtime = env
    running = _running(client, runtime)
    first = runtime.apply_external_result(
        task_id=running["task_id"],
        goal_id=running["goal_id"],
        execution_id=running["execution_id"],
        owner_epoch=running["owner_epoch"],
        session_id=running["session_id"],
        outcome="succeeded",
        pose_x=0.0,
        pose_y=0.0,
        pose_yaw=0,
        pose_trustworthy=True,
        source="bad-pose",
    )
    assert first["status"] == "RUNNING"
    assert first["goal"]["observation_failures"] == 1
    second = runtime.apply_external_result(
        task_id=running["task_id"],
        goal_id=running["goal_id"],
        execution_id=running["execution_id"],
        owner_epoch=running["owner_epoch"],
        session_id=running["session_id"],
        outcome="succeeded",
        pose_x=0.1,
        pose_y=0.1,
        pose_yaw=0,
        pose_trustworthy=True,
        source="bad-pose",
    )
    assert second["status"] == "RECOVERY_REQUIRED"
    assert second["completion_evidence"] is None
