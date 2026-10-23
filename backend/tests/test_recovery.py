from fastapi.testclient import TestClient

from app.main import create_app
from app.models.tables import Robot
from conftest import FIXED, make_settings


def _pin_running(client, runtime):
    runtime.settings.mock_goal_duration_seconds = 30
    created = client.post(
        "/api/tasks",
        json={"destination": {"x": 4.0, "y": 1.5, "label": "west-bay"}, "robot_id": "pinky-3"},
    )
    task_id = created.json()["task"]["task_id"]
    runtime.tick()
    task = client.get(f"/api/tasks/{task_id}").json()
    assert task["status"] == "RUNNING"
    return task


def test_restart_quarantines_inflight_and_does_not_replay(tmp_path):
    settings = make_settings(tmp_path)
    app = create_app(settings)
    app.state.runtime.clock.freeze(FIXED)
    with TestClient(app) as client:
        task = _pin_running(client, app.state.runtime)
        task_id = task["task_id"]
        assert task["navigation_goal_count"] == 1
    restarted = create_app(settings)
    restarted.state.runtime.clock.freeze(FIXED)
    with TestClient(restarted) as client:
        task = client.get(f"/api/tasks/{task_id}").json()
        assert task["status"] == "RECOVERY_REQUIRED"
        assert task["navigation_goal_count"] == 1
        restarted.state.runtime.tick()
        again = client.get(f"/api/tasks/{task_id}").json()
        assert again["status"] == "RECOVERY_REQUIRED"
        assert again["navigation_goal_count"] == 1
        assert again["completion_evidence"] is None


def test_agent_crash_then_late_success_stays_in_recovery(env):
    client, runtime = env
    running = _pin_running(client, runtime)
    with runtime.db.write() as session:
        robot = session.get(Robot, running["robot_id"])
        runtime.agent.stop(session, robot, runtime.clock.now())
    runtime.tick()
    held = client.get(f"/api/tasks/{running['task_id']}").json()
    assert held["status"] == "RECOVERY_REQUIRED"
    assert held["navigation_goal_count"] == 1
    late = runtime.apply_external_result(
        task_id=running["task_id"],
        goal_id=running["goal_id"],
        execution_id=running["execution_id"],
        owner_epoch=running["owner_epoch"],
        session_id=running["session_id"],
        outcome="succeeded",
        pose_x=4.0,
        pose_y=1.5,
        pose_yaw=0,
        pose_trustworthy=True,
        source="late-result",
    )
    assert late["status"] == "RECOVERY_REQUIRED"
    assert late["goal"]["status"] != "succeeded"
    runtime.tick()
    final = client.get(f"/api/tasks/{running['task_id']}").json()
    assert final["navigation_goal_count"] == 1


def test_cancel_ack_is_not_completion_and_timeout_is_not_cancel(env):
    client, runtime = env
    running = _pin_running(client, runtime)
    canceled = client.post(f"/api/tasks/{running['task_id']}/cancel")
    assert canceled.json()["task"]["status"] == "CANCEL_REQUESTED"
    runtime.tick()
    done = client.get(f"/api/tasks/{running['task_id']}").json()
    assert done["status"] == "CANCELED"
    assert done["goal"]["status"] == "cancel_acked"
    assert done["completion_evidence"] is None

    runtime.settings.mock_goal_duration_seconds = 30
    other = client.post(
        "/api/tasks",
        json={"destination": {"x": 6.0, "y": 2.0, "label": "south-bay"}, "robot_id": "pinky-3"},
    )
    other_id = other.json()["task"]["task_id"]
    runtime.tick()
    assert client.get(f"/api/tasks/{other_id}").json()["status"] == "RUNNING"
    runtime.agent.suppress_cancel_ack.add("pinky-3")
    runtime.settings.cancel_timeout_seconds = 5
    client.post(f"/api/tasks/{other_id}/cancel")
    runtime.clock.advance(6)
    runtime.tick()
    hung = client.get(f"/api/tasks/{other_id}").json()
    assert hung["status"] == "RECOVERY_REQUIRED"
    assert hung["status"] != "CANCELED"
    assert hung["completion_evidence"] is None


def test_resolve_recovery_does_not_replay_a_goal(env):
    client, runtime = env
    running = _pin_running(client, runtime)
    with runtime.db.write() as session:
        robot = session.get(Robot, "pinky-3")
        runtime.agent.stop(session, robot, runtime.clock.now())
    runtime.tick()
    denied = client.post(
        f"/api/tasks/{running['task_id']}/resolve",
        json={"outcome": "COMPLETED", "reason": "wishful", "acknowledge_position_unknown": True},
    )
    assert denied.status_code == 409
    closed = client.post(
        f"/api/tasks/{running['task_id']}/resolve",
        json={
            "outcome": "CANCELED",
            "reason": "operator inspected the bay",
            "acknowledge_position_unknown": True,
        },
    )
    assert closed.status_code == 200
    assert closed.json()["goal_replayed"] is False
    assert closed.json()["task"]["status"] == "CANCELED"
    assert closed.json()["task"]["navigation_goal_count"] == 1
    follow = client.post(
        "/api/tasks",
        json={"destination": {"x": 6.0, "y": 2.0, "label": "south-bay"}, "robot_id": "pinky-3"},
    )
    follow_id = follow.json()["task"]["task_id"]
    runtime.tick()
    assert client.get(f"/api/tasks/{follow_id}").json()["status"] == "PENDING"
    observed = client.post("/api/robots/pinky-3/mock-observe")
    assert observed.status_code == 200
    assert observed.json()["pose_trustworthy"] is True
    assert runtime.goals_for_robot("pinky-3") == 1
    runtime.tick()
    moved = client.get(f"/api/tasks/{follow_id}").json()
    assert moved["status"] in {"RUNNING", "COMPLETED"}
    assert runtime.goals_for_robot("pinky-3") == 2


def test_heartbeat_loss_is_recovery_not_success(env):
    client, runtime = env
    running = _pin_running(client, runtime)
    runtime.agent.suppress_cancel_ack.add("pinky-3")
    runtime.settings.heartbeat_timeout_seconds = 5
    runtime.clock.advance(6)
    runtime.tick()
    task = client.get(f"/api/tasks/{running['task_id']}").json()
    assert task["status"] == "RECOVERY_REQUIRED"
    assert task["completion_evidence"] is None
    assert task["navigation_goal_count"] == 1
