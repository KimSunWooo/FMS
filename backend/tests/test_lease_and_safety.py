import pytest

from app.core.config import Settings
from app.robot.safety import SafetyGuard
from robot_agent.nav2_bridge import Nav2BridgeDisabled, send_navigate_to_pose


def test_gateway_rejects_stale_and_foreign_leases(env):
    _client, runtime = env
    now = runtime.clock.now()
    with runtime.db.write() as session:
        session_id = runtime.agent.session_id("pinky-3")
        first = runtime.gateway.issue(
            session, robot_id="pinky-3", session_id=session_id, owner_id="fms", now=now
        )
        assert runtime.gateway.verify(session, first, now=now, robot_id="pinky-3", session_id=session_id)
        second = runtime.gateway.issue(
            session, robot_id="pinky-3", session_id=session_id, owner_id="fms", now=now
        )
        assert runtime.gateway.verify(session, second, now=now, robot_id="pinky-3", session_id=session_id)
        assert not runtime.gateway.verify(session, first, now=now, robot_id="pinky-3", session_id=session_id)
        assert not runtime.gateway.verify(session, second, now=now, robot_id="pinky-1", session_id=session_id)
        second.environment_id = "care-move-legacy"
        assert not runtime.gateway.verify(session, second, now=now, robot_id="pinky-3", session_id=session_id)
        second.environment_id = runtime.gateway.environment_id
        second.signature = runtime.gateway._sign(second)
        assert runtime.gateway.verify(session, second, now=now, robot_id="pinky-3", session_id=session_id)
        runtime.gateway.revoke(session, second.lease_id, now)
        assert not runtime.gateway.verify(session, second, now=now, robot_id="pinky-3", session_id=session_id)
        third = runtime.gateway.issue(
            session, robot_id="pinky-3", session_id=session_id, owner_id="fms", now=now
        )
        runtime.gateway.boot_id = "previous-boot"
        assert not runtime.gateway.verify(session, third, now=now, robot_id="pinky-3", session_id=session_id)


def test_expired_lease_is_rejected(env):
    _client, runtime = env
    now = runtime.clock.now()
    with runtime.db.write() as session:
        session_id = runtime.agent.session_id("pinky-1")
        lease = runtime.gateway.issue(
            session, robot_id="pinky-1", session_id=session_id, owner_id="fms", now=now
        )
        runtime.clock.advance(runtime.settings.lease_ttl_seconds + 1)
        assert not runtime.gateway.verify(
            session, lease, now=runtime.clock.now(), robot_id="pinky-1", session_id=session_id
        )


def test_dispatch_without_a_lease_sends_no_goal(env):
    client, runtime = env
    runtime.gateway.force_reject = True
    runtime.settings.max_dispatch_attempts = 1
    created = client.post(
        "/api/tasks",
        json={"destination": {"x": 4.0, "y": 1.5, "label": "west-bay"}, "robot_id": "pinky-3"},
    )
    task_id = created.json()["task"]["task_id"]
    runtime.tick()
    task = client.get(f"/api/tasks/{task_id}").json()
    assert task["status"] == "FAILED"
    assert task["navigation_goal_count"] == 0
    assert task["goal_may_be_in_flight"] is False


def test_estop_does_not_publish_velocity(env):
    client, runtime = env
    runtime.settings.mock_goal_duration_seconds = 30
    created = client.post(
        "/api/tasks",
        json={"destination": {"x": 4.0, "y": 1.5, "label": "west-bay"}, "robot_id": "pinky-3"},
    )
    task_id = created.json()["task"]["task_id"]
    runtime.tick()
    stopped = client.post(
        "/api/robots/pinky-3/emergency-stop",
        json={"reason": "person in the corridor"},
    )
    assert stopped.status_code == 202
    assert stopped.json()["estop_latched"] is True
    runtime.tick()
    task = client.get(f"/api/tasks/{task_id}").json()
    assert task["status"] == "CANCELED"
    assert task["goal"]["status"] == "cancel_acked"
    assert runtime.safety.velocity_publish_count == 0
    with pytest.raises(Exception) as caught:
        runtime.safety.publish_cmd_vel("pinky-3", 0.2, 0.0)
    assert "cmd_vel" in str(caught.value)
    assert runtime.safety.velocity_publish_count == 0
    assert runtime.safety.rejected_velocity_count == 1


def test_physical_and_ros_cannot_be_enabled(tmp_path):
    with pytest.raises(ValueError):
        Settings(
            physical_control_enabled=True,
            database_url=f"sqlite:///{tmp_path / 'a.sqlite3'}",
            data_dir=tmp_path,
        )
    with pytest.raises(ValueError):
        Settings(
            ros_bridge_enabled=True,
            database_url=f"sqlite:///{tmp_path / 'b.sqlite3'}",
            data_dir=tmp_path,
        )
    with pytest.raises(ValueError):
        Settings(
            database_url="postgresql://fms:fms@localhost/db_care_rmf",
            data_dir=tmp_path,
        )
    guard = SafetyGuard()
    with pytest.raises(Exception):
        guard.publish_cmd_vel("pinky-1", 1, 0)
    with pytest.raises(Nav2BridgeDisabled):
        send_navigate_to_pose("pinky-1", 1, 2, 0)


def test_physical_registration_is_rejected(env):
    client, _runtime = env
    response = client.post(
        "/api/robots",
        json={"robot_id": "pinky-9", "name": "Pinky #9", "mode": "physical", "battery_percent": 80},
    )
    assert response.status_code == 403
    assert response.json()["error"]["code"] == "PHYSICAL_CONTROL_BLOCKED"
