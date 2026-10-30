from __future__ import annotations

import sys
from datetime import datetime, timezone
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

WORKSPACE = Path(__file__).resolve().parents[2]
if str(WORKSPACE) not in sys.path:
    sys.path.insert(0, str(WORKSPACE))

from app.core.config import ROOT, Settings
from app.main import create_app

FIXED = datetime(2026, 10, 23, 10, 0, tzinfo=timezone.utc)


def make_settings(
    tmp_path: Path,
    *,
    seed_fleet: bool = True,
    execution_mode: str = "mock",
    planning_resolution_m: float = 0.05,
    planning_robot_radius_m: float = 0.06,
) -> Settings:
    """The shared fixture stays on the previous 0.05 m lattice.

    Production defaults are R_effective (~0.09485 m). Tests of that lattice must pass
    those values explicitly so this pin cannot hide them.
    """
    return Settings(
        env="development",
        environment_id="care-move-fms-test",
        lease_secret="test-secret-not-from-care-move",
        database_url=f"sqlite:///{tmp_path / 'fms.sqlite3'}",
        data_dir=tmp_path / "data",
        log_dir=tmp_path / "logs",
        zones_path=ROOT / "config" / "zones.json",
        fleet_path=ROOT / "config" / "fleet.json",
        auto_tick=False,
        seed_fleet=seed_fleet,
        process_lock=True,
        mock_goal_duration_seconds=0,
        heartbeat_timeout_seconds=3600,
        cancel_timeout_seconds=3600,
        lease_ttl_seconds=3600,
        physical_control_enabled=False,
        ros_bridge_enabled=False,
        execution_mode=execution_mode,
        ros_domain_id=73,
        physical_nav2_auto_start=False,
        domain_workers_enabled=False,
        physical_5cm_test_enabled=False,
        physical_ssh_targets="",
        physical_ssh_password="",
        physical_map_yaml=ROOT / "config/maps/physical_site/1f_map_f.yaml",
        physical_fleet_path=ROOT / "config" / "physical_fleet.json",
        planning_resolution_m=planning_resolution_m,
        planning_robot_radius_m=planning_robot_radius_m,
        planning_max_candidates=3,
        planning_allow_diagonal=True,
        planning_speed_m_s=0.15,
        planning_max_depart_slots=8,
        planning_goal_dwell_s=0.5,
        fleet_dispatch_mode="local",
        fleet_replan_enabled=False,
    )


@pytest.fixture
def env(tmp_path: Path):
    settings = make_settings(tmp_path)
    app = create_app(settings)
    app.state.runtime.clock.freeze(FIXED)
    with TestClient(app) as client:
        yield client, app.state.runtime


@pytest.fixture
def bare(tmp_path: Path):
    settings = make_settings(tmp_path, seed_fleet=False)
    app = create_app(settings)
    app.state.runtime.clock.freeze(FIXED)
    with TestClient(app) as client:
        yield client, app.state.runtime


def post_task(client, *, x, y, label, robot_id=None, key=None, priority=0, yaw=0.0):
    headers = {}
    if key is not None:
        headers["Idempotency-Key"] = key
    return client.post(
        "/api/tasks",
        headers=headers,
        json={
            "destination": {"x": x, "y": y, "yaw": yaw, "label": label},
            "priority": priority,
            "robot_id": robot_id,
        },
    )
