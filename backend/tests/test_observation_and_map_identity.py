import asyncio
from datetime import datetime, timedelta, timezone
from types import SimpleNamespace

import pytest
from fastapi import HTTPException

from app.core.config import ROOT
from app.services.forward_5cm_service import PinkyFiveCmTest
from app.services.map_identity import assess_map_identity, occupancy_sha256
from app.services.observation_gate import observation_blockers
from app.services.occupancy_grid import OccupancyGrid
from app.services.physical_nav2_auto import PhysicalNav2Auto

MAP = ROOT / "config" / "maps" / "physical_site" / "1f_map_f.yaml"


def _ready(**extra):
    now = datetime.now(timezone.utc).isoformat()
    body = {
        "checked_at": now,
        "live_tf_status": "fresh",
        "live_pose_checked_at": now,
        "pose": {"x": 1.0, "y": 0.1, "yaw": 0.0},
        "map_matches_fms": True,
        "map_mismatch_detail": "",
    }
    body.update(extra)
    return body


def test_missing_and_invalid_stamps_name_the_field():
    missing = observation_blockers(_ready(checked_at=None, live_pose_checked_at=None, live_tf_status="missing"))
    assert "checked_at missing" in missing
    assert "live_pose_checked_at missing" in missing
    assert "Recent Nav2/TF observations unavailable" not in "; ".join(missing)
    invalid = observation_blockers(_ready(checked_at="yesterday", live_pose_checked_at="2026-10-28T00:00:00"))
    assert "checked_at invalid" in invalid
    assert "live_pose_checked_at invalid" in invalid


def test_expired_live_tf_is_not_described_as_missing():
    old = (datetime.now(timezone.utc) - timedelta(seconds=9)).isoformat()
    reasons = observation_blockers(_ready(live_pose_checked_at=old))
    assert any(item.startswith("live_pose_checked_at expired:") for item in reasons)
    assert "live_pose_checked_at missing" not in reasons


def test_fresh_live_tf_is_not_withheld_for_ssh_or_amcl_flags():
    """ros_topic mode: a fresh TF sample is shown even if SSH/AMCL flags are false."""
    monitor = PhysicalNav2Auto(SimpleNamespace(
        physical_nav2_robot_id="pinky-1",
        physical_initial_pose_yaw_deg=0.0,
        physical_initial_pose_auto=False,
        data_dir="/tmp/fms-test-data",
    ))
    fresh = datetime.now(timezone.utc).isoformat()
    monitor._published_state.update(
        checked_at=fresh,
        ssh_connected=False,
        amcl_active=False,
        initial_pose_confirmed=False,
        map_matches_fms=True,
        map_mismatch_detail="",
        pose={"x": 1.0, "y": 2.0, "yaw": 0.0},
        pose_checked_at=fresh,
    )
    monitor._live_pose_seen = True
    monitor._live_pose = {"x": 3.0, "y": 4.0, "yaw": 0.1}
    monitor._live_pose_checked_at = fresh
    snap = monitor.snapshot()
    assert snap["live_tf_status"] == "fresh"
    assert snap["pose"]["x"] == 3.0
    assert snap.get("live_tf_withheld_reason") == ""
    assert "live TF pose withheld" not in snap["five_cm_blockers"]


def test_lookup_failure_stays_distinct_from_expiry():
    reasons = observation_blockers(_ready(live_tf_status="lookup_failed", live_pose_checked_at=None, pose=None))
    assert reasons == ["live TF lookup failed"]


def test_snapshot_drops_expired_live_tf_without_reusing_poll_pose():
    monitor = PhysicalNav2Auto(SimpleNamespace(
        physical_nav2_robot_id="pinky-1",
        physical_initial_pose_yaw_deg=0.0,
        data_dir="/tmp/fms-test-data",
    ))
    fresh = datetime.now(timezone.utc).isoformat()
    expired = (datetime.now(timezone.utc) - timedelta(seconds=10)).isoformat()
    monitor._published_state.update(
        checked_at=fresh,
        pose={"x": 1.0, "y": 2.0, "yaw": 0.0},
        pose_checked_at=fresh,
        ssh_connected=True,
        amcl_active=True,
        initial_pose_confirmed=True,
        map_matches_fms=True,
        map_mismatch_detail="",
    )
    monitor._live_pose_seen = True
    monitor._live_pose = {"x": 9.0, "y": 9.0, "yaw": 0.0}
    monitor._live_pose_checked_at = expired
    snap = monitor.snapshot()
    assert snap["pose"] is None
    assert snap["pose_checked_at"] is None
    assert snap["poll_pose"]["x"] == 1.0
    assert snap["checked_at"] == fresh
    assert snap["live_tf_status"] == "expired"
    assert any(item.startswith("live_pose_checked_at expired:") for item in snap["five_cm_blockers"])


def test_start_rejects_with_the_failing_field():
    class Monitor:
        primary_id = "pinky-1"

        def snapshot(self):
            return {
                "ssh_connected": True,
                "ros_online": True,
                "map_server_active": True,
                "amcl_active": True,
                "nav2_ready": True,
                "tf_ready": True,
                "cmd_vel_chain_ready": True,
                "initial_pose_confirmed": True,
                **_ready(checked_at=None, map_matches_fms=False, map_mismatch_detail="map yaml mismatch"),
            }

    controller = PinkyFiveCmTest(SimpleNamespace(), Monitor())

    async def exercise():
        with pytest.raises(HTTPException) as caught:
            await controller.start()
        return caught.value

    error = asyncio.run(exercise())
    assert error.status_code == 409
    assert "checked_at missing" in error.detail
    assert "map yaml mismatch" in error.detail
    assert "unavailable" not in error.detail


def test_map_match_requires_yaml_geometry_and_cells():
    grid = OccupancyGrid.load(MAP)
    probe = {
        "width": grid.width,
        "height": grid.height,
        "resolution": grid.resolution,
        "origin_x": grid.geometry.origin_x,
        "origin_y": grid.geometry.origin_y,
        "origin_yaw": grid.geometry.origin_yaw,
        "content_sha256": occupancy_sha256(grid.cells),
    }
    matched = assess_map_identity(expected_yaml=MAP, server_yaml=str(MAP), probe=probe)
    assert matched["map_matches_fms"] is True
    # Absolute path may differ; geometry+occupancy content is enough.
    path_only = assess_map_identity(
        expected_yaml=MAP,
        server_yaml="/tmp/other_host/1f_map_f.yaml",
        probe=probe,
    )
    assert path_only["map_matches_fms"] is True
    changed = dict(probe, content_sha256="0" * 64)
    mismatched = assess_map_identity(expected_yaml=MAP, server_yaml=str(MAP), probe=changed)
    assert mismatched["map_matches_fms"] is False
    assert "occupancy sha256" in mismatched["map_mismatch_detail"]
    missing = assess_map_identity(
        expected_yaml=MAP,
        server_yaml=str(MAP.with_name("1f_map.yaml")),
        probe=changed,
    )
    assert missing["map_matches_fms"] is False
    assert "yaml file is missing" in missing["map_mismatch_detail"]
    assert "occupancy sha256" in missing["map_mismatch_detail"]
    assert assess_map_identity(expected_yaml=MAP, server_yaml=str(MAP), probe=None)["map_matches_fms"] is False
