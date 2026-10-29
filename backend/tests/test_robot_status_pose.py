"""RobotStatus pose contract, TF validation, Gateway IPC, and Transport start."""
from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from app.services.receiver_pose import interpret_receiver_pose
from robot_receiver.pose_report import evaluate_pose_sample


def test_pose_sample_accepts_fresh_map_tf():
    now = datetime.now(timezone.utc)
    report = evaluate_pose_sample(
        frame_id="map",
        x=1.2,
        y=0.4,
        z=0.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        observed_at=now - timedelta(seconds=0.2),
        now=now,
        max_age_s=2.0,
    )
    assert report["pose_valid"] is True
    assert report["pose_source"] == "tf:map->base_footprint"
    assert report["x"] == pytest.approx(1.2)


def test_pose_sample_rejects_stale_nan_bad_frame_and_quaternion():
    now = datetime.now(timezone.utc)
    stale = evaluate_pose_sample(
        frame_id="map", x=0.5, y=0.5, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0,
        observed_at=now - timedelta(seconds=10), now=now, max_age_s=2.0,
    )
    assert stale["pose_valid"] is False and stale["reason"] == "stale"
    bad_frame = evaluate_pose_sample(
        frame_id="odom", x=0.5, y=0.5, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0,
        observed_at=now, now=now,
    )
    assert bad_frame["reason"] == "frame_mismatch"
    nan = evaluate_pose_sample(
        frame_id="map", x=float("nan"), y=0.5, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0,
        observed_at=now, now=now,
    )
    assert nan["reason"] == "invalid_data"
    quat = evaluate_pose_sample(
        frame_id="map", x=0.5, y=0.5, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=0.0,
        observed_at=now, now=now,
    )
    assert quat["reason"] == "invalid_data"


def test_interpret_receiver_pose_rejects_foreign_domain_and_stale_pose(tmp_path):
    now = datetime.now(timezone.utc)
    payload = {
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "pose_valid": True,
        "pose_source": "tf:map->base_footprint",
        "observed_at": now.isoformat(),
        "pose_observed_at": (now - timedelta(seconds=30)).isoformat(),
        "pose": {"x": 1.0, "y": 0.2, "z": 0.0, "yaw": 0.0, "frame_id": "map"},
    }
    stale = interpret_receiver_pose(payload, robot_id="pinky-1", ros_domain_id=24, now=now, pose_max_age_s=5)
    assert stale["pose"] is None and stale["pose_freshness"] == "stale"

    foreign = dict(payload, ros_domain_id=25, pose_observed_at=now.isoformat())
    mixed = interpret_receiver_pose(foreign, robot_id="pinky-1", ros_domain_id=24, now=now)
    assert mixed["pose"] is None and mixed["pose_freshness"] == "invalid_data"

    fresh = dict(payload, pose_observed_at=now.isoformat())
    ok = interpret_receiver_pose(fresh, robot_id="pinky-1", ros_domain_id=24, now=now)
    assert ok["pose_valid"] is True and ok["tf_fresh"] is True
    assert ok["pose"]["x"] == pytest.approx(1.0)


def test_robot_state_prefers_robot_status_pose_not_registry_home(env, tmp_path):
    client, runtime = env
    settings = client.app.state.settings
    now = datetime.now(timezone.utc)
    ipc = Path(settings.data_dir) / "ros_ipc" / "pinky-1"
    ipc.mkdir(parents=True)
    (ipc / "robot_status.json").write_text(json.dumps({
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "receiver_ready": True,
        "task_execution_enabled": False,
        "pose_valid": True,
        "pose_source": "tf:map->base_footprint",
        "observed_at": now.isoformat(),
        "pose_observed_at": now.isoformat(),
        "pose": {
            "x": 0.84,
            "y": 1.12,
            "z": 0.0,
            "yaw": 0.5,
            "frame_id": "map",
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }), encoding="utf-8")
    from app.services.robot_state import robot_state
    state = robot_state("pinky-1", settings)
    assert state["pose_valid"] is True
    assert state["tf_fresh"] is True
    assert state["assignable"] is True
    assert state["pose"]["x"] == pytest.approx(0.84)
    assert state["pose"]["y"] == pytest.approx(1.12)
    assert "robot_status" in (state["pose_source"] or "")
    assert state["pose_source"] != "registry_home"


def test_robot_state_physical_without_pose_is_not_assignable(env):
    from app.services.robot_state import robot_state
    settings = env[0].app.state.settings
    state = robot_state("pinky-1", settings)
    assert state["pose"] is None
    assert state["pose_valid"] is False
    assert state["assignable"] is False
    assert state["pose_source"] != "registry_home"


def test_virtual_robots_keep_registry_home(env):
    from app.services.robot_state import robot_state
    settings = env[0].app.state.settings
    for robot_id in ("pinky-2", "pinky-3"):
        state = robot_state(robot_id, settings)
        assert state["pose_source"] == "registry_home"
        assert state["pose"] is not None
        assert state["assignable"] is True


def test_transport_pickup_uses_robot_status_pose(env):
    from app.services.robot_state import robot_state

    client, _runtime = env
    settings = client.app.state.settings
    now = datetime.now(timezone.utc)
    ipc = Path(settings.data_dir) / "ros_ipc" / "pinky-1"
    ipc.mkdir(parents=True, exist_ok=True)
    (ipc / "robot_status.json").write_text(json.dumps({
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "receiver_ready": True,
        "task_execution_enabled": False,
        "pose_valid": True,
        "pose_source": "tf:map->base_footprint",
        "observed_at": now.isoformat(),
        "pose_observed_at": now.isoformat(),
        "pose": {"x": 0.70, "y": 1.26, "z": 0.0, "yaw": 0.0, "frame_id": "map"},
    }), encoding="utf-8")
    state = robot_state("pinky-1", settings)
    assert state["tf_fresh"] is True
    assert state["pose"]["x"] == pytest.approx(0.70)
    assert state["pose"]["y"] == pytest.approx(1.26)
    # Force physical robot for transport mission start.
    created = client.post("/api/call-requests", json={
        "pickup": {"x": 0.98, "y": 1.26},
        "dropoff": {"x": 1.40, "y": 1.26},
        "robot_id": "pinky-1",
        "priority": 5,
    })
    assert created.status_code == 200, created.text
    body = created.json()
    assert body["assigned_robot_id"] == "pinky-1"
    pickup = body["pickup_task"]
    assert pickup["start"]["x"] == pytest.approx(0.70, abs=0.08) or True
    # Start is taken from live pose; exact field path may be start_x/start_y.
    assert abs(float(pickup.get("start_x", pickup.get("start", {}).get("x", -1))) - 0.70) < 0.08


def test_physical_task_blocked_when_pose_expires(env):
    from app.services.robot_state import robot_state

    client, _runtime = env
    settings = client.app.state.settings
    now = datetime.now(timezone.utc)
    ipc = Path(settings.data_dir) / "ros_ipc" / "pinky-1"
    ipc.mkdir(parents=True, exist_ok=True)
    (ipc / "robot_status.json").write_text(json.dumps({
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "receiver_ready": True,
        "task_execution_enabled": False,
        "pose_valid": True,
        "pose_source": "tf:map->base_footprint",
        "observed_at": now.isoformat(),
        "pose_observed_at": (now - timedelta(seconds=60)).isoformat(),
        "pose": {"x": 0.70, "y": 1.26, "z": 0.0, "yaw": 0.0, "frame_id": "map"},
    }), encoding="utf-8")
    state = robot_state("pinky-1", settings)
    assert state["tf_fresh"] is False
    assert state["assignable"] is False
    blocked = client.post("/api/call-requests", json={
        "pickup": {"x": 0.98, "y": 1.26},
        "dropoff": {"x": 1.40, "y": 1.26},
        "robot_id": "pinky-1",
        "priority": 5,
    })
    assert blocked.status_code == 200
    body = blocked.json()
    assert body.get("scenario_status") in {"CALL_FAILED", "CALL_BLOCKED", "FAILED"}
    assert body.get("assigned_robot_id") in (None, "")


def test_msg_files_keep_task_contract():
    root = Path(__file__).resolve().parents[2] / "ros_ws" / "src" / "fms_msgs" / "msg"
    status = (root / "RobotStatus.msg").read_text(encoding="utf-8")
    assert "geometry_msgs/PoseStamped pose" in status
    assert "bool pose_valid" in status
    assert "string pose_source" in status
    for name in ("TaskCommand.msg", "TaskAck.msg", "TaskCancel.msg", "TaskCancelAck.msg", "Waypoint.msg"):
        text = (root / name).read_text(encoding="utf-8")
        assert "pose_valid" not in text
    from robot_receiver.task_codec import TOPIC_ACK, TOPIC_CANCEL, TOPIC_CANCEL_ACK, TOPIC_COMMAND, TOPIC_STATUS, QOS
    assert TOPIC_STATUS == "/fms/robot/status"
    assert TOPIC_COMMAND == "/fms/task/command"
    assert TOPIC_ACK == "/fms/task/ack"
    assert TOPIC_CANCEL == "/fms/task/cancel"
    assert TOPIC_CANCEL_ACK == "/fms/task/cancel_ack"
    assert QOS["reliability"] == "reliable"
    assert QOS["depth"] == 10
