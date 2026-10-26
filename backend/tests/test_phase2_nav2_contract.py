"""Phase 2 contract tests.

Labels:
- UNIT: preflight predicate, domain gate, physical refusal. No simulator.
- MOCK: FMS task state driven by the agent HTTP API. A succeeded outcome in
  these tests is a fixture posted by the test, not a Gazebo or Nav2 result.
- GAZEBO_E2E: not collected. The host has no Jazzy, Gazebo, or Pinky package,
  so nothing here is allowed to report that tier as passed.
"""

import shutil
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from app.core.config import Settings
from app.main import create_app
from conftest import FIXED, make_settings, post_task
from robot_agent.agent_main import main as agent_main
from robot_agent.agent_main import note_fms_failure
from robot_agent.gazebo_nav2_client import (
    GazeboNav2Client,
    GazeboNav2Unavailable,
    format_action_goal_id,
    goal_statuses_block_send,
    yaw_from_quaternion,
)
from robot_agent.nav2_bridge import Nav2BridgeDisabled, build_bridge, send_navigate_to_pose
from robot_agent.preflight import evaluate, live_report


READY_PROBE = {
    "ros_distro": "jazzy",
    "ros_domain_id": 73,
    "rmw_implementation": "rmw_cyclonedds_cpp",
    "ros2_binary": True,
    "gazebo_binary": True,
    "nav2_interface": True,
    "pinky_packages": ["pinky_gazebo"],
}


@pytest.fixture
def gazebo(tmp_path):
    settings = make_settings(tmp_path, seed_fleet=False, execution_mode="gazebo")
    app = create_app(settings)
    app.state.runtime.clock.freeze(FIXED)
    with TestClient(app) as client:
        yield client, app.state.runtime


def _register(client, robot_id, x, y):
    response = client.post(
        "/api/robots",
        json={
            "robot_id": robot_id,
            "name": robot_id,
            "mode": "gazebo",
            "battery_percent": 90,
            "x": x,
            "y": y,
            "yaw": 0,
            "controller_ready": True,
        },
    )
    assert response.status_code == 201, response.text
    assert response.json()["controller_kind"] == "nav2"
    return response.json()


def _arm(client, robot_id):
    response = client.post("/api/agent/session", json={"robot_id": robot_id})
    assert response.status_code == 201, response.text
    return response.json()["session_id"]


def _dispatch(client, runtime, robot_id, *, x=5.0, y=6.5, label="open-bay"):
    created = post_task(client, x=x, y=y, label=label, robot_id=robot_id)
    assert created.status_code == 202, created.text
    task_id = created.json()["task"]["task_id"]
    runtime.tick()
    task = client.get(f"/api/tasks/{task_id}").json()
    assert task["status"] == "DISPATCHING"
    assert task["navigation_goal_count"] == 1
    assert task["goal_may_be_in_flight"] is True
    return task


def _claim(client, task):
    response = client.post(
        "/api/agent/claim",
        json={
            "robot_id": task["robot_id"],
            "session_id": task["session_id"],
            "owner_epoch": task["owner_epoch"],
            "lease_id": task["lease_id"],
            "goal_id": task["goal_id"],
        },
    )
    assert response.status_code == 200, response.text
    return response.json()


def _event(client, task, *, kind, outcome=None, pose=None, trustworthy=False, epoch=None, nav2_id=None):
    body = {
        "robot_id": task["robot_id"],
        "session_id": task["session_id"],
        "task_id": task["task_id"],
        "goal_id": task["goal_id"],
        "execution_id": task["execution_id"],
        "owner_epoch": task["owner_epoch"] if epoch is None else epoch,
        "lease_id": task["lease_id"],
        "kind": kind,
        "outcome": outcome,
        "pose_trustworthy": trustworthy,
        "nav2_action_goal_id": nav2_id,
    }
    if pose is not None:
        body["pose_x"] = pose[0]
        body["pose_y"] = pose[1]
        body["pose_yaw"] = pose[2]
    response = client.post("/api/agent/goal-event", json=body)
    assert response.status_code == 200, response.text
    return response.json()


def _running(client, runtime, robot_id="pinky-sim", **dest):
    _register(client, robot_id, 4.0, 1.5)
    _arm(client, robot_id)
    task = _dispatch(client, runtime, robot_id, **dest) if dest else _dispatch(client, runtime, robot_id)
    claim = _claim(client, task)
    assert claim["send_navigate_to_pose"] is True
    accepted = _event(client, task, kind="accepted", nav2_id="fixture-action-id")
    assert accepted["status"] == "RUNNING"
    assert accepted["goal"]["nav2_action_goal_id"] == "fixture-action-id"
    assert accepted["goal"]["status"] == "accepted"
    return accepted


def test_unit_live_preflight_is_not_ready_and_client_fails_closed():
    report = live_report(expected_domain=73)
    assert report.ready is False
    # This process is not a sourced ROS shell. Domain, RMW, and Pinky overlay stay closed.
    assert "ros_distro_not_jazzy" in report.reasons
    assert "ros_domain_mismatch" in report.reasons
    assert "rmw_implementation_unset" in report.reasons
    assert "pinky_simulation_package_missing" in report.reasons
    assert report.pinky_packages == ()
    nav2_share = Path("/opt/ros/jazzy/share")
    nav2_installed = (nav2_share / "nav2_msgs").is_dir() or (nav2_share / "nav2_bringup").is_dir()
    if nav2_installed:
        assert report.nav2_interface is True
        assert "nav2_interface_missing" not in report.reasons
    else:
        assert "nav2_interface_missing" in report.reasons
    if shutil.which("ros2") is None:
        assert "ros2_binary_missing" in report.reasons
    if not any(shutil.which(name) for name in ("gz", "gazebo", "ign")):
        assert "gazebo_binary_missing" in report.reasons
    with pytest.raises(GazeboNav2Unavailable, match="not ready"):
        build_bridge("gazebo", report, expected_domain=73, observed_domain=report.ros_domain_id).send_goal(
            robot_id="pinky-sim", goal_id="goal", x=1, y=2, yaw=0
        )
    with pytest.raises(Nav2BridgeDisabled):
        send_navigate_to_pose("pinky-sim", 1, 2, 0)


def test_unit_action_uuid_and_yaw_are_not_invented():
    assert format_action_goal_id(None) is None
    assert format_action_goal_id(object()) is None
    blank = type("UUID", (), {"uuid": bytes(16)})()
    assert format_action_goal_id(blank) is None
    sample = bytes.fromhex("00112233445566778899aabbccddeeff")
    assert format_action_goal_id(type("UUID", (), {"uuid": sample})()) == "00112233-4455-6677-8899-aabbccddeeff"
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0
    assert goal_statuses_block_send(None) is True
    assert goal_statuses_block_send([4, 6]) is False
    assert goal_statuses_block_send([4, 2]) is True
    summary = {"nav2_goal_sent": True, "task_completed": True}
    note_fms_failure(summary, RuntimeError("database write failed"))
    assert summary["nav2_goal_sent"] is True
    assert summary["task_completed"] is False


def test_unit_preflight_predicate_requires_every_gate():
    ready = evaluate(READY_PROBE, expected_domain=73)
    assert ready.ready is True
    missing = dict(READY_PROBE)
    missing["pinky_packages"] = []
    assert evaluate(missing, expected_domain=73).ready is False
    wrong_domain = dict(READY_PROBE)
    wrong_domain["ros_domain_id"] = 0
    assert "ros_domain_mismatch" in evaluate(wrong_domain, expected_domain=73).reasons


def test_unit_wrong_domain_and_physical_mode_are_refused(tmp_path):
    ready = evaluate(READY_PROBE, expected_domain=73)
    client = GazeboNav2Client(mode="gazebo", expected_domain=73, observed_domain=0, preflight=ready)
    with pytest.raises(GazeboNav2Unavailable, match="domain"):
        client.send_goal(robot_id="pinky-sim", goal_id="goal", x=1, y=2, yaw=0)
    with pytest.raises(Nav2BridgeDisabled):
        build_bridge("physical", ready, expected_domain=73, observed_domain=73)
    with pytest.raises(GazeboNav2Unavailable, match="physical"):
        GazeboNav2Client(mode="physical", expected_domain=73, observed_domain=73, preflight=ready).assert_can_send()
    with pytest.raises(ValueError):
        Settings(
            execution_mode="physical",
            database_url=f"sqlite:///{tmp_path / 'physical.sqlite3'}",
            data_dir=tmp_path,
        )
    with pytest.raises(GazeboNav2Unavailable, match="SUCCEEDED"):
        GazeboNav2Client(
            mode="gazebo", expected_domain=73, observed_domain=73, preflight=ready
        ).send_goal(robot_id="pinky-sim", goal_id="goal", x=1, y=2, yaw=0)


def test_unit_agent_process_exits_without_a_goal(capsys):
    code = agent_main(["--robot-id", "pinky-sim", "--expected-domain", "73", "--fms-url", "http://127.0.0.1:9"])
    assert code == 2
    printed = capsys.readouterr().out
    assert '"nav2_goal_sent": false' in printed
    assert '"task_completed": false' in printed
    assert "SUCCEEDED" not in printed


def test_mock_gazebo_registration_is_closed_until_execution_mode_is_gazebo(env):
    client, _runtime = env
    response = client.post(
        "/api/robots",
        json={"robot_id": "pinky-sim", "name": "Pinky sim", "mode": "gazebo", "battery_percent": 80},
    )
    assert response.status_code == 403
    assert response.json()["error"]["code"] == "PHYSICAL_CONTROL_BLOCKED"


def test_mock_nav2_success_requires_the_full_fence(gazebo):
    client, runtime = gazebo
    task = _running(client, runtime)
    incomplete = client.get(f"/api/tasks/{task['task_id']}").json()
    assert incomplete["status"] == "RUNNING"
    assert incomplete["completion_evidence"] is None
    runtime.tick()
    still = client.get(f"/api/tasks/{task['task_id']}").json()
    assert still["status"] == "RUNNING"
    assert still["navigation_goal_count"] == 1
    done = _event(
        client,
        task,
        kind="result",
        outcome="succeeded",
        pose=(5.0, 6.5, 0.0),
        trustworthy=True,
    )
    assert done["status"] == "COMPLETED"
    assert done["goal"]["goal_id"] == task["goal_id"]
    assert done["execution_id"] == task["execution_id"]
    assert done["goal"]["nav2_action_goal_id"] == "fixture-action-id"
    assert done["completion_evidence"]["goal_status"] == "succeeded"
    assert done["navigation_goal_count"] == 1
    runtime.tick()
    assert client.get(f"/api/tasks/{task['task_id']}").json()["navigation_goal_count"] == 1


def test_mock_nav2_reject_abort_cancel_and_disconnect(gazebo):
    client, runtime = gazebo
    _register(client, "pinky-sim", 4.0, 1.5)
    _arm(client, "pinky-sim")
    rejected = _dispatch(client, runtime, "pinky-sim")
    assert _claim(client, rejected)["send_navigate_to_pose"] is True
    failed = _event(client, rejected, kind="result", outcome="rejected")
    assert failed["status"] == "FAILED"
    assert failed["failure_reason"] == "nav2_goal_rejected"
    assert failed["goal"]["nav2_action_goal_id"] is None
    assert failed["goal"]["nav2_action_goal_id_absent_reason"] == "action_server_rejected_before_goal_id"
    assert failed["navigation_goal_count"] == 1

    abort_task = _dispatch(client, runtime, "pinky-sim", x=6.0, y=7.0, label="north-bay")
    assert _claim(client, abort_task)["send_navigate_to_pose"] is True
    _event(client, abort_task, kind="accepted", nav2_id="fixture-abort")
    abort_result = _event(client, abort_task, kind="result", outcome="aborted")
    assert abort_result["status"] == "FAILED"
    assert abort_result["navigation_goal_count"] == 1

    # An abort can arrive without a prior accept event. The action UUID is still stored.
    direct = _dispatch(client, runtime, "pinky-sim", x=8.0, y=1.0, label="outside")
    assert _claim(client, direct)["send_navigate_to_pose"] is True
    direct_abort = _event(client, direct, kind="result", outcome="aborted", nav2_id="11111111-2222-3333-4444-555555555555")
    assert direct_abort["status"] == "FAILED"
    assert direct_abort["goal"]["nav2_action_goal_id"] == "11111111-2222-3333-4444-555555555555"
    assert direct_abort["navigation_goal_count"] == 1

    cancel_task = _dispatch(client, runtime, "pinky-sim", x=5.5, y=6.8, label="east-open")
    assert _claim(client, cancel_task)["send_navigate_to_pose"] is True
    assert _event(client, cancel_task, kind="accepted", nav2_id="fixture-cancel")["status"] == "RUNNING"
    requested = client.post(f"/api/tasks/{cancel_task['task_id']}/cancel")
    assert requested.json()["task"]["status"] == "CANCEL_REQUESTED"
    canceled = _event(client, cancel_task, kind="result", outcome="canceled", trustworthy=True)
    assert canceled["status"] == "CANCELED"
    assert canceled["completion_evidence"] is None

    lost = _dispatch(client, runtime, "pinky-sim", x=4.4, y=7.1, label="west-open")
    assert _claim(client, lost)["send_navigate_to_pose"] is True
    assert _event(client, lost, kind="accepted", nav2_id="fixture-lost")["status"] == "RUNNING"
    disconnected = _event(client, lost, kind="disconnected", outcome="disconnected")
    assert disconnected["status"] == "RECOVERY_REQUIRED"
    assert disconnected["navigation_goal_count"] == 1
    runtime.tick()
    again = client.get(f"/api/tasks/{lost['task_id']}").json()
    assert again["status"] == "RECOVERY_REQUIRED"
    assert again["navigation_goal_count"] == 1


def test_mock_duplicate_claim_does_not_create_a_second_goal(gazebo):
    client, runtime = gazebo
    _register(client, "pinky-sim", 4.0, 1.5)
    _arm(client, "pinky-sim")
    task = _dispatch(client, runtime, "pinky-sim")
    first = _claim(client, task)
    second = _claim(client, task)
    assert first["send_navigate_to_pose"] is True
    assert second["send_navigate_to_pose"] is False
    assert second["reason"] == "duplicate_send"
    assert second["goal_id"] == first["goal_id"]
    assignment = client.get(
        "/api/agent/assignment",
        params={"robot_id": "pinky-sim", "session_id": task["session_id"]},
    )
    assert assignment.json()["send_navigate_to_pose"] is False
    assert client.get(f"/api/tasks/{task['task_id']}").json()["navigation_goal_count"] == 1


def test_mock_stale_epoch_and_cancel_race_do_not_complete(gazebo):
    client, runtime = gazebo
    task = _running(client, runtime)
    stale = _event(
        client,
        task,
        kind="result",
        outcome="succeeded",
        pose=(5.0, 6.5, 0.0),
        trustworthy=True,
        epoch=task["owner_epoch"] - 1,
    )
    assert stale["status"] == "RUNNING"
    assert stale["goal"]["status"] == "accepted"
    assert stale["goal"]["last_result_disposition"] == "rejected_stale_owner"
    raced = client.post(f"/api/tasks/{task['task_id']}/cancel")
    assert raced.json()["task"]["status"] == "CANCEL_REQUESTED"
    late = _event(
        client,
        task,
        kind="result",
        outcome="succeeded",
        pose=(5.0, 6.5, 0.0),
        trustworthy=True,
    )
    assert late["status"] == "RECOVERY_REQUIRED"
    assert late["status"] != "COMPLETED"
    assert late["navigation_goal_count"] == 1
    assert late["completion_evidence"] is None


def test_mock_lease_expiry_rejects_a_late_success(gazebo):
    client, runtime = gazebo
    task = _running(client, runtime)
    runtime.clock.advance(runtime.settings.lease_ttl_seconds + 1)
    late = _event(
        client,
        task,
        kind="result",
        outcome="succeeded",
        pose=(5.0, 6.5, 0.0),
        trustworthy=True,
    )
    assert late["status"] == "RECOVERY_REQUIRED"
    assert late["navigation_goal_count"] == 1
    assert late["completion_evidence"] is None


def test_mock_fms_restart_does_not_assume_the_goal_finished(tmp_path):
    settings = make_settings(tmp_path, seed_fleet=False, execution_mode="gazebo")
    app = create_app(settings)
    app.state.runtime.clock.freeze(FIXED)
    with TestClient(app) as client:
        task = _running(client, app.state.runtime)
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


def test_mock_agent_restart_does_not_resume_the_old_goal(gazebo):
    client, runtime = gazebo
    task = _running(client, runtime)
    replaced = client.post("/api/agent/session/replace", json={"robot_id": "pinky-sim"})
    assert replaced.status_code == 200, replaced.text
    assert replaced.json()["resumed_previous_goal"] is False
    assert task["task_id"] in replaced.json()["recovered_task_ids"]
    current = client.get(f"/api/tasks/{task['task_id']}").json()
    assert current["status"] == "RECOVERY_REQUIRED"
    claim = client.post(
        "/api/agent/claim",
        json={
            "robot_id": "pinky-sim",
            "session_id": replaced.json()["session_id"],
            "owner_epoch": current["owner_epoch"],
            "lease_id": current["lease_id"] or task["lease_id"],
            "goal_id": task["goal_id"],
        },
    )
    assert claim.status_code == 200
    assert claim.json()["send_navigate_to_pose"] is False
    runtime.tick()
    final = client.get(f"/api/tasks/{task['task_id']}").json()
    assert final["status"] == "RECOVERY_REQUIRED"
    assert final["navigation_goal_count"] == 1


def test_mock_stopping_one_agent_leaves_the_other_task_untouched(gazebo):
    client, runtime = gazebo
    first = _running(client, runtime, robot_id="pinky-a")
    _register(client, "pinky-b", 10.2, 6.4)
    _arm(client, "pinky-b")
    second = _dispatch(client, runtime, "pinky-b", x=6.2, y=7.2, label="far-open")
    assert _claim(client, second)["send_navigate_to_pose"] is True
    second = _event(client, second, kind="accepted", nav2_id="fixture-b")
    assert second["status"] == "RUNNING"
    stopped = client.post(
        "/api/agent/stop",
        json={"robot_id": "pinky-a", "session_id": first["session_id"]},
    )
    assert stopped.status_code == 200
    assert first["task_id"] in stopped.json()["recovered_task_ids"]
    runtime.tick()
    left = client.get(f"/api/tasks/{first['task_id']}").json()
    right = client.get(f"/api/tasks/{second['task_id']}").json()
    assert left["status"] == "RECOVERY_REQUIRED"
    assert left["navigation_goal_count"] == 1
    assert right["status"] == "RUNNING"
    assert right["navigation_goal_count"] == 1
    assert right["goal"]["status"] == "accepted"


def test_mock_agent_death_right_after_claim_is_recovery(gazebo):
    client, runtime = gazebo
    _register(client, "pinky-sim", 4.0, 1.5)
    session_id = _arm(client, "pinky-sim")
    task = _dispatch(client, runtime, "pinky-sim")
    assert _claim(client, task)["send_navigate_to_pose"] is True
    stopped = client.post("/api/agent/stop", json={"robot_id": "pinky-sim", "session_id": session_id})
    assert stopped.status_code == 200
    current = client.get(f"/api/tasks/{task['task_id']}").json()
    assert current["status"] == "RECOVERY_REQUIRED"
    assert current["navigation_goal_count"] == 1
    runtime.tick()
    assert client.get(f"/api/tasks/{task['task_id']}").json()["navigation_goal_count"] == 1


def test_mock_unsolicited_cancel_is_recovery(gazebo):
    client, runtime = gazebo
    task = _running(client, runtime)
    result = _event(client, task, kind="result", outcome="canceled", trustworthy=True)
    assert result["status"] == "RECOVERY_REQUIRED"
    assert result["navigation_goal_count"] == 1


def test_mock_uuid_is_kept_on_timeout_conflict_and_expired_lease(gazebo):
    client, runtime = gazebo
    _register(client, "pinky-lease", 4.0, 1.5)
    _arm(client, "pinky-lease")
    late = _dispatch(client, runtime, "pinky-lease", x=6.0, y=7.0, label="lease-bay")
    assert _claim(client, late)["send_navigate_to_pose"] is True
    runtime.clock.advance(runtime.settings.lease_ttl_seconds + 1)
    expired = _event(
        client,
        late,
        kind="result",
        outcome="succeeded",
        pose=(6.0, 7.0, 0.0),
        trustworthy=True,
        nav2_id="12121212-3434-5656-7878-909090909090",
    )
    assert expired["status"] == "RECOVERY_REQUIRED"
    assert expired["goal"]["nav2_action_goal_id"] == "12121212-3434-5656-7878-909090909090"
    assert expired["completion_evidence"] is None
    assert expired["navigation_goal_count"] == 1

    _register(client, "pinky-sim", 4.0, 1.5)
    session_id = _arm(client, "pinky-sim")
    timed = _dispatch(client, runtime, "pinky-sim")
    assert _claim(client, timed)["send_navigate_to_pose"] is True
    held = _event(
        client,
        timed,
        kind="result",
        outcome="timed_out",
        nav2_id="aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee",
    )
    assert held["status"] == "RECOVERY_REQUIRED"
    assert held["goal"]["nav2_action_goal_id"] == "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"
    assert held["goal"]["nav2_action_goal_id_absent_reason"] is None
    assert held["navigation_goal_count"] == 1
    closed = client.post(
        f"/api/tasks/{timed['task_id']}/resolve",
        json={"outcome": "FAILED", "reason": "timeout hold closed", "acknowledge_position_unknown": True},
    )
    assert closed.status_code == 200, closed.text
    posed = client.post(
        "/api/agent/pose",
        json={
            "robot_id": "pinky-sim",
            "session_id": session_id,
            "pose_x": 4.0,
            "pose_y": 1.5,
            "pose_yaw": 0,
            "frame_id": "map",
        },
    )
    assert posed.status_code == 200, posed.text
    task = _dispatch(client, runtime, "pinky-sim", x=5.0, y=6.5, label="open-bay")
    assert _claim(client, task)["send_navigate_to_pose"] is True
    accepted = _event(client, task, kind="accepted", nav2_id="fixture-action-id")
    assert accepted["status"] == "RUNNING"
    conflict = _event(
        client,
        task,
        kind="result",
        outcome="succeeded",
        pose=(5.0, 6.5, 0.0),
        trustworthy=True,
        nav2_id="ffffffff-ffff-ffff-ffff-ffffffffffff",
    )
    assert conflict["status"] == "RUNNING"
    assert conflict["goal"]["nav2_action_goal_id"] == "fixture-action-id"
    assert conflict["goal"]["last_result_disposition"] == "rejected_nav2_uuid_conflict"
    stale = _event(
        client,
        task,
        kind="result",
        outcome="succeeded",
        pose=(5.0, 6.5, 0.0),
        trustworthy=True,
        epoch=0,
        nav2_id="99999999-9999-9999-9999-999999999999",
    )
    assert stale["status"] == "RUNNING"
    assert stale["goal"]["nav2_action_goal_id"] == "fixture-action-id"


def test_mock_dead_session_is_not_given_another_goal(gazebo):
    client, runtime = gazebo
    _register(client, "pinky-sim", 4.0, 1.5)
    session_id = _arm(client, "pinky-sim")
    task = _dispatch(client, runtime, "pinky-sim")
    assert _claim(client, task)["send_navigate_to_pose"] is True
    runtime.settings.heartbeat_timeout_seconds = 5
    runtime.clock.advance(6)
    runtime.tick()
    current = client.get(f"/api/tasks/{task['task_id']}").json()
    assert current["status"] == "RECOVERY_REQUIRED"
    assert current["navigation_goal_count"] == 1
    assert runtime.agent.is_alive("pinky-sim") is False
    late = client.post(
        "/api/agent/goal-event",
        json={
            "robot_id": "pinky-sim",
            "session_id": session_id,
            "task_id": task["task_id"],
            "goal_id": task["goal_id"],
            "execution_id": task["execution_id"],
            "owner_epoch": task["owner_epoch"],
            "lease_id": task["lease_id"],
            "kind": "result",
            "outcome": "succeeded",
            "pose_trustworthy": True,
            "pose_x": 5.0,
            "pose_y": 6.5,
            "nav2_action_goal_id": "abababab-abab-abab-abab-abababababab",
        },
    )
    assert late.status_code == 409
    assert client.get(f"/api/tasks/{task['task_id']}").json()["status"] == "RECOVERY_REQUIRED"
    opened = client.post("/api/agent/session", json={"robot_id": "pinky-sim"})
    assert opened.status_code == 201
    claim = client.post(
        "/api/agent/claim",
        json={
            "robot_id": "pinky-sim",
            "session_id": opened.json()["session_id"],
            "owner_epoch": current["owner_epoch"],
            "lease_id": task["lease_id"],
            "goal_id": task["goal_id"],
        },
    )
    assert claim.status_code == 200
    assert claim.json()["send_navigate_to_pose"] is False
    posed = client.post(
        "/api/agent/pose",
        json={
            "robot_id": "pinky-sim",
            "session_id": opened.json()["session_id"],
            "pose_x": 1.0,
            "pose_y": 1.0,
            "pose_yaw": 0,
            "frame_id": "map",
        },
    )
    assert posed.status_code == 400
    blocked = client.post("/api/robots/pinky-sim/mock-observe")
    assert blocked.status_code == 403


def test_mock_uuid_is_stored_after_recovery_without_completing(gazebo):
    client, runtime = gazebo
    _register(client, "pinky-sim", 4.0, 1.5)
    _arm(client, "pinky-sim")
    task = _dispatch(client, runtime, "pinky-sim")
    assert _claim(client, task)["send_navigate_to_pose"] is True
    runtime.settings.heartbeat_timeout_seconds = 100000
    runtime.clock.advance(runtime.settings.lease_ttl_seconds + 1)
    runtime.tick()
    held = client.get(f"/api/tasks/{task['task_id']}").json()
    assert held["status"] == "RECOVERY_REQUIRED"
    assert held["goal"]["nav2_action_goal_id"] is None
    late = _event(
        client,
        task,
        kind="result",
        outcome="succeeded",
        pose=(5.0, 6.5, 0.0),
        trustworthy=True,
        epoch=held["owner_epoch"],
        nav2_id="abababab-cdcd-efef-1212-343434343434",
    )
    assert late["status"] == "RECOVERY_REQUIRED"
    assert late["goal"]["nav2_action_goal_id"] == "abababab-cdcd-efef-1212-343434343434"
    assert late["goal"]["last_result_disposition"] == "rejected_task_not_running"
    assert late["completion_evidence"] is None
    assert late["navigation_goal_count"] == 1


def test_mock_pose_miss_then_session_stop_stays_a_pose_hold(gazebo):
    client, runtime = gazebo
    task = _running(client, runtime)
    missed = _event(
        client,
        task,
        kind="result",
        outcome="succeeded",
        pose=(0.0, 0.0, 0.0),
        trustworthy=True,
    )
    assert missed["status"] == "RUNNING"
    assert missed["goal"]["last_result_disposition"] == "observation_mismatch"
    assert missed["goal"]["nav2_action_goal_id"] == "fixture-action-id"
    stopped = client.post(
        "/api/agent/stop",
        json={"robot_id": "pinky-sim", "session_id": task["session_id"]},
    )
    assert stopped.status_code == 200
    current = client.get(f"/api/tasks/{task['task_id']}").json()
    assert current["status"] == "RECOVERY_REQUIRED"
    assert current["recovery_reason"] == "pose_mismatch"
    assert current["navigation_goal_count"] == 1
    assert current["completion_evidence"] is None


def test_mock_gazebo_pose_observation_allows_a_new_assignment(gazebo):
    client, runtime = gazebo
    _register(client, "pinky-sim", 4.0, 1.5)
    session_id = _arm(client, "pinky-sim")
    stopped = client.post("/api/agent/stop", json={"robot_id": "pinky-sim", "session_id": session_id})
    assert stopped.status_code == 200
    created = post_task(client, x=5.0, y=6.5, label="after-stop", robot_id="pinky-sim")
    runtime.tick()
    waiting = client.get(f"/api/tasks/{created.json()['task']['task_id']}").json()
    assert waiting["status"] == "PENDING"
    session_id = _arm(client, "pinky-sim")
    posed = client.post(
        "/api/agent/pose",
        json={
            "robot_id": "pinky-sim",
            "session_id": session_id,
            "pose_x": 0.2,
            "pose_y": -0.1,
            "pose_yaw": 0.3,
            "frame_id": "map",
        },
    )
    assert posed.status_code == 200, posed.text
    assert posed.json()["pose_source"] == "gazebo-tf"
    assert posed.json()["pose_trustworthy"] is True
    runtime.tick()
    assigned = client.get(f"/api/tasks/{waiting['task_id']}").json()
    assert assigned["status"] == "DISPATCHING"
    assert assigned["navigation_goal_count"] == 1
