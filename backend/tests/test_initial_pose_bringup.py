"""Initial-pose /initialpose path and Nav2 bringup ordering (no motion)."""
from __future__ import annotations

import asyncio
import math
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import pytest

from app.services.initial_pose_registry import resolve_registered_initial_pose
from app.services.physical_nav2_auto import PhysicalNav2Auto


def _settings(tmp_path: Path, **extra):
    root = Path(__file__).resolve().parents[2]
    base = dict(
        physical_nav2_robot_id="pinky-1",
        physical_initial_pose_auto=True,
        physical_initial_pose_max_attempts=3,
        physical_initial_pose_retry_s=0.01,
        physical_initial_pose_topic="/initialpose",
        physical_initial_pose_yaw_deg=0.0,
        physical_map_yaml=root / "config/maps/physical_site/1f_map_f.yaml",
        physical_nav2_params_yaml=root / "config/maps/physical_site/nav2_params.yaml",
        physical_ssh_targets="",
        physical_ssh_password="",
        physical_ssh_timeout_seconds=1.0,
        physical_nav2_poll_seconds=3.0,
        physical_nav2_activate_max_attempts=5,
        data_dir=tmp_path,
        ros_amcl_pose_topic="/amcl_pose",
    )
    base.update(extra)
    return SimpleNamespace(**base)


@pytest.fixture
def monitor(tmp_path):
    m = PhysicalNav2Auto(_settings(tmp_path))
    m.state["map_matches_fms"] = True
    m.state["expected_map_sha256"] = "abc"
    return m


def test_registered_pose_comes_from_nav2_params_not_legacy_point():
    from app.core.config import Settings

    seed = resolve_registered_initial_pose(Settings(), robot_id="pinky-1")
    assert seed["valid"] is True
    assert seed["x"] == pytest.approx(0.11)
    assert seed["y"] == pytest.approx(2.527)
    assert seed["yaw_rad"] == pytest.approx(-3.052)
    assert not (math.isclose(seed["x"], 1.696) and math.isclose(seed["y"], 0.103))


def test_active_lifecycle_is_not_reconfigured(monitor, monkeypatch):
    calls: list[tuple] = []

    async def fake_ros(*args, **kwargs):
        calls.append(args)
        return True, "ok"

    async def fake_life(name: str) -> str:
        return "active"

    monkeypatch.setattr(monitor, "_ros", fake_ros)
    monkeypatch.setattr(monitor, "_lifecycle", fake_life)
    ok = asyncio.run(monitor._ensure_lifecycle("amcl", {"/amcl"}, ["ros2", "run", "nav2_amcl", "amcl"]))
    assert ok is True
    assert calls == []


def test_existing_localization_skips_auto_seed(monitor, monkeypatch):
    async def observed() -> bool:
        monitor.state["pose"] = {"x": 0.5, "y": 0.6, "yaw": 0.0}
        monitor.state["pose_checked_at"] = datetime.now(timezone.utc).isoformat()
        return True

    pubs: list = []

    async def fake_ros(*args, **kwargs):
        pubs.append(args)
        return True, "ok"

    monkeypatch.setattr(monitor, "_localization_observed", observed)
    monkeypatch.setattr(monitor, "_ros", fake_ros)
    assert asyncio.run(monitor._ensure_localization()) is True
    assert monitor.state["initial_pose_state"] == "existing_localization"
    assert monitor.state["initial_pose_confirmed"] is True
    assert not any("topic" in args and "pub" in args for args in pubs)


def test_map_mismatch_rejects_seed(monitor, monkeypatch):
    monitor.state["map_matches_fms"] = False
    monitor.state["map_mismatch_detail"] = "hash differs"

    async def no_loc() -> bool:
        return False

    monkeypatch.setattr(monitor, "_localization_observed", no_loc)
    result = asyncio.run(monitor.send_initial_pose(x=0.11, y=2.527, yaw_rad=-3.052))
    assert result["accepted"] is False
    assert result["reason"] == "map_mismatch"


def test_send_publishes_initialpose_topic_not_service(monitor, monkeypatch):
    async def no_loc() -> bool:
        return False

    calls: list[tuple] = []

    async def fake_ros(*args, **kwargs):
        calls.append(args)
        return True, "Publishing"

    monkeypatch.setattr(monitor, "_localization_observed", no_loc)
    monkeypatch.setattr(monitor, "_ros", fake_ros)
    result = asyncio.run(monitor.send_initial_pose(x=0.11, y=2.527, yaw_rad=-3.052, force=True))
    assert result["accepted"] is True
    assert result["topic"] == "/initialpose"
    assert any(args[:3] == ("ros2", "topic", "pub") for args in calls)
    assert not any("set_initial_pose" in args for args in calls)
    assert monitor.state["initial_pose_sent"] is True
    assert monitor.state["initial_pose_confirmed"] is False
    assert monitor.state["initial_pose_state"] == "sent_waiting_for_observation"


def test_duplicate_poll_does_not_resend_while_waiting(monitor, monkeypatch):
    async def no_loc() -> bool:
        return False

    pubs = {"n": 0}

    async def fake_ros(*args, **kwargs):
        if args[:3] == ("ros2", "topic", "pub"):
            pubs["n"] += 1
        return True, "ok"

    monkeypatch.setattr(monitor, "_localization_observed", no_loc)
    monkeypatch.setattr(monitor, "_ros", fake_ros)
    asyncio.run(monitor.send_initial_pose(x=0.11, y=2.527, yaw_rad=-3.052, force=True))
    assert pubs["n"] == 1
    assert asyncio.run(monitor._ensure_localization()) is False
    assert pubs["n"] == 1
    assert monitor.state["initial_pose_state"] == "sent_waiting_for_observation"


def test_max_attempts_stops_republish(monitor, monkeypatch):
    async def no_loc() -> bool:
        return False

    monkeypatch.setattr(monitor, "_localization_observed", no_loc)

    async def fake_ros(*args, **kwargs):
        return True, "ok"

    monkeypatch.setattr(monitor, "_ros", fake_ros)
    monitor._initial_pose_attempts = 3
    result = asyncio.run(monitor.send_initial_pose(x=0.11, y=2.527, yaw_rad=-3.052))
    assert result["reason"] == "failed_after_max_attempts"


def test_nav2_not_started_without_localization(monitor, monkeypatch):
    monitor.state["localization_ready"] = False
    started = {"n": 0}

    def fake_spawn(key, cmd):
        started["n"] += 1
        return True

    monkeypatch.setattr(monitor, "_spawn", fake_spawn)
    assert asyncio.run(monitor._ensure_nav2_after_localization(set())) is False
    assert started["n"] == 0


def test_unregistered_pose_blocks_auto(tmp_path, monkeypatch):
    missing = tmp_path / "empty_nav2.yaml"
    missing.write_text("amcl:\n  ros__parameters: {}\n", encoding="utf-8")
    m = PhysicalNav2Auto(_settings(tmp_path, physical_nav2_params_yaml=missing, physical_initial_pose_auto=True))
    m.state["map_matches_fms"] = True

    async def no_loc() -> bool:
        return False

    monkeypatch.setattr(m, "_localization_observed", no_loc)
    assert asyncio.run(m._ensure_localization()) is False
    assert m.state["initial_pose_state"] == "initial_pose_unregistered"


def test_ssh_offline_does_not_block_ensure_lifecycle(monitor, monkeypatch):
    monitor.state["ssh_connected"] = False

    async def fake_life(name: str) -> str:
        return "active"

    monkeypatch.setattr(monitor, "_lifecycle", fake_life)
    assert asyncio.run(monitor._ensure_lifecycle("amcl", {"/amcl"}, ["x"])) is True
