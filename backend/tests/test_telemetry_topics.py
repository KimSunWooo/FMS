"""Canonical /amcl_pose, /plan, /battery/percent telemetry contract."""
from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace

import pytest

from app.services.robot_telemetry import read_robot_telemetry
from robot_receiver.telemetry_topics import (
    TELEMETRY_QOS,
    TOPIC_AMCL_POSE,
    TOPIC_BATTERY_PERCENT,
    TOPIC_PLAN,
    amcl_pose_to_dict,
    battery_percent_to_dict,
    plan_to_dict,
)


def _stamp(sec: int = 100, nanosec: int = 0):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def test_telemetry_topic_names_and_qos():
    assert TOPIC_AMCL_POSE == "/amcl_pose"
    assert TOPIC_PLAN == "/plan"
    assert TOPIC_BATTERY_PERCENT == "/battery/percent"
    assert TELEMETRY_QOS == {
        "reliability": "reliable",
        "durability": "volatile",
        "history": "keep_last",
        "depth": 10,
    }


def test_amcl_pose_accepts_map_rejects_odom_frame():
    now = datetime.now(timezone.utc)
    msg = SimpleNamespace(
        header=SimpleNamespace(frame_id="map", stamp=_stamp()),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.7, y=1.1, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
    )
    body = amcl_pose_to_dict(msg, robot_id="pinky-1", ros_domain_id=24, received_at=now)
    assert body["pose_valid"] is True
    assert body["pose"]["x"] == pytest.approx(0.7)
    assert body["pose_source"] == "topic:amcl_pose"

    bad = SimpleNamespace(
        header=SimpleNamespace(frame_id="odom", stamp=_stamp()),
        pose=msg.pose,
    )
    rejected = amcl_pose_to_dict(bad, robot_id="pinky-1", ros_domain_id=24, received_at=now)
    assert rejected["pose_valid"] is False
    assert rejected["pose_reason"] == "frame_mismatch"


def test_plan_and_battery_conversion():
    now = datetime.now(timezone.utc)
    pose = SimpleNamespace(
        header=SimpleNamespace(frame_id="map", stamp=_stamp()),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    path = SimpleNamespace(
        header=SimpleNamespace(frame_id="map", stamp=_stamp()),
        poses=[pose, pose],
    )
    plan = plan_to_dict(path, robot_id="pinky-1", ros_domain_id=24, received_at=now)
    assert plan["plan_valid"] is True
    assert plan["pose_count"] == 2

    batt100 = battery_percent_to_dict(SimpleNamespace(data=85.0), robot_id="pinky-1", ros_domain_id=24, received_at=now)
    assert batt100["battery_valid"] is True
    assert batt100["battery_percentage"] == pytest.approx(0.85)
    # Already normalized: do not divide again.
    batt1 = battery_percent_to_dict(SimpleNamespace(data=0.4), robot_id="pinky-1", ros_domain_id=24, received_at=now)
    assert batt1["battery_percentage"] == pytest.approx(0.4)
    bad = battery_percent_to_dict(SimpleNamespace(data=float("nan")), robot_id="pinky-1", ros_domain_id=24, received_at=now)
    assert bad["battery_valid"] is False


def test_robot_telemetry_isolates_domain_and_stale(tmp_path):
    now = datetime.now(timezone.utc)
    root = tmp_path / "ros_ipc" / "pinky-1"
    root.mkdir(parents=True)
    (root / "amcl_pose.json").write_text(json.dumps({
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "pose_valid": True,
        "pose_source": "topic:amcl_pose",
        "pose": {"x": 0.5, "y": 0.6, "z": 0.0, "yaw": 0.1, "frame_id": "map"},
        "pose_observed_at": now.isoformat(),
        "gateway_received_at": now.isoformat(),
    }), encoding="utf-8")
    (root / "battery_percent.json").write_text(json.dumps({
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "battery_valid": True,
        "battery_percentage": 0.9,
        "battery_source": "topic:battery/percent",
        "battery_observed_at": now.isoformat(),
        "gateway_received_at": now.isoformat(),
    }), encoding="utf-8")
    (root / "plan.json").write_text(json.dumps({
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "plan_valid": True,
        "plan_source": "topic:plan",
        "frame_id": "map",
        "poses": [{"x": 0.5, "y": 0.6, "z": 0.0}],
        "plan_observed_at": now.isoformat(),
        "gateway_received_at": now.isoformat(),
    }), encoding="utf-8")

    ok = read_robot_telemetry(tmp_path, "pinky-1", 24, now=now)
    assert ok["pose_valid"] and ok["battery_valid"] and ok["plan_valid"]

    foreign = read_robot_telemetry(tmp_path, "pinky-1", 25, now=now)
    assert foreign["pose_valid"] is False and foreign["battery_valid"] is False

    stale_time = now - timedelta(seconds=120)
    payload = json.loads((root / "amcl_pose.json").read_text(encoding="utf-8"))
    payload["pose_observed_at"] = stale_time.isoformat()
    payload["gateway_received_at"] = stale_time.isoformat()
    (root / "amcl_pose.json").write_text(json.dumps(payload), encoding="utf-8")
    stale = read_robot_telemetry(tmp_path, "pinky-1", 24, now=now)
    assert stale["pose_valid"] is False and stale["pose_freshness"] == "stale"
    assert stale["pose"]["x"] == pytest.approx(0.5)  # last value retained


def test_robot_state_prefers_amcl_over_registry(env):
    from app.services.robot_state import robot_state

    client, _runtime = env
    settings = client.app.state.settings
    now = datetime.now(timezone.utc)
    root = Path(settings.data_dir) / "ros_ipc" / "pinky-1"
    root.mkdir(parents=True)
    (root / "amcl_pose.json").write_text(json.dumps({
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "pose_valid": True,
        "pose_source": "topic:amcl_pose",
        "pose": {"x": 0.91, "y": 1.05, "z": 0.0, "yaw": 0.2, "frame_id": "map"},
        "pose_observed_at": now.isoformat(),
        "gateway_received_at": now.isoformat(),
    }), encoding="utf-8")
    (root / "battery_percent.json").write_text(json.dumps({
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "battery_valid": True,
        "battery_percentage": 0.77,
        "battery_source": "topic:battery/percent",
        "battery_observed_at": now.isoformat(),
        "gateway_received_at": now.isoformat(),
    }), encoding="utf-8")
    state = robot_state("pinky-1", settings)
    assert state["pose_valid"] is True
    assert state["pose"]["x"] == pytest.approx(0.91)
    assert "amcl" in (state["pose_source"] or "")
    assert state["pose_source"] != "registry_home"
    assert state["battery_percentage"] == pytest.approx(0.77)
