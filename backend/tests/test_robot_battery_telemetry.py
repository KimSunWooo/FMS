"""RobotStatus battery contract and IPC freshness. No invented SOC."""
from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from app.services.receiver_pose import interpret_receiver_battery
from robot_receiver.battery_adapter import BatteryTelemetryCache
from robot_receiver.battery_report import evaluate_battery_sample, normalize_soc


def test_normalize_soc_accepts_fraction_and_percent():
    assert normalize_soc(0.42) == pytest.approx(0.42)
    assert normalize_soc(85.0) == pytest.approx(0.85)
    assert normalize_soc(1.0) == pytest.approx(1.0)
    assert normalize_soc(0.0) == pytest.approx(0.0)
    assert normalize_soc(float("nan")) is None
    assert normalize_soc(-0.1) is None
    assert normalize_soc(150.0) is None


def test_evaluate_battery_rejects_stale_and_missing():
    now = datetime.now(timezone.utc)
    ok = evaluate_battery_sample(
        percentage=0.55, observed_at=now, source="topic:battery/percent", now=now, max_age_s=30,
    )
    assert ok["battery_valid"] is True
    stale = evaluate_battery_sample(
        percentage=0.55,
        observed_at=now - timedelta(seconds=120),
        source="topic:battery/percent",
        now=now,
        max_age_s=30,
    )
    assert stale["battery_valid"] is False and stale["reason"] == "stale"
    missing = evaluate_battery_sample(percentage=None, observed_at=now, source="x", now=now)
    assert missing["reason"] == "missing"


def test_battery_cache_float32_and_voltage_only_state():
    cache = BatteryTelemetryCache(i2c_fallback=False, max_age_s=30)
    cache.update_percent_float32(73.5)
    snap = cache.snapshot()
    assert snap["battery_valid"] is True
    assert snap["battery_percentage"] == pytest.approx(0.735)

    class _Msg:
        percentage = float("nan")
        class header:
            stamp = None

    cache2 = BatteryTelemetryCache(i2c_fallback=False)
    cache2.update_battery_state(_Msg())
    assert cache2.snapshot()["battery_valid"] is False
    assert cache2.last_reason == "voltage_only_or_missing"


def test_interpret_receiver_battery_independent_of_pose(tmp_path):
    now = datetime.now(timezone.utc)
    payload = {
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "pose_valid": False,
        "battery_valid": True,
        "battery_percentage": 0.81,
        "battery_source": "pinkylib:i2c",
        "observed_at": now.isoformat(),
        "battery_observed_at": now.isoformat(),
    }
    ok = interpret_receiver_battery(payload, robot_id="pinky-1", ros_domain_id=24, now=now)
    assert ok["battery_valid"] is True
    assert ok["battery_percentage"] == pytest.approx(0.81)

    foreign = dict(payload, ros_domain_id=25)
    bad = interpret_receiver_battery(foreign, robot_id="pinky-1", ros_domain_id=24, now=now)
    assert bad["battery_valid"] is False


def test_robot_state_exposes_battery_without_requiring_pose(env):
    from app.services.robot_state import robot_state

    client, _runtime = env
    settings = client.app.state.settings
    now = datetime.now(timezone.utc)
    ipc = Path(settings.data_dir) / "ros_ipc" / "pinky-1"
    ipc.mkdir(parents=True)
    (ipc / "robot_status.json").write_text(json.dumps({
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "receiver_ready": True,
        "task_execution_enabled": False,
        "pose_valid": False,
        "battery_valid": True,
        "battery_percentage": 0.66,
        "battery_source": "topic:battery/percent",
        "observed_at": now.isoformat(),
        "battery_observed_at": now.isoformat(),
    }), encoding="utf-8")
    state = robot_state("pinky-1", settings)
    assert state["pose"] is None
    assert state["assignable"] is False
    assert state["battery_valid"] is True
    assert state["battery_percentage"] == pytest.approx(0.66)


def test_msg_includes_battery_fields():
    root = Path(__file__).resolve().parents[2] / "ros_ws" / "src" / "fms_msgs" / "msg"
    text = (root / "RobotStatus.msg").read_text(encoding="utf-8")
    assert "bool battery_valid" in text
    assert "float32 battery_percentage" in text
    assert "builtin_interfaces/Time battery_stamp" in text
    assert "string battery_source" in text
    for name in ("TaskCommand.msg", "TaskAck.msg", "TaskCancel.msg", "TaskCancelAck.msg", "Waypoint.msg"):
        assert "battery_valid" not in (root / name).read_text(encoding="utf-8")
