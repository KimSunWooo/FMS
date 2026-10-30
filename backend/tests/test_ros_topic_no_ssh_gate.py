"""ros_topic mode must not require SSH for pose observation or readiness display."""
from __future__ import annotations

from datetime import datetime, timezone

from app.services.fleet_registry import present_robot
from app.services.route_preview import fresh_tf_pose
from app.services.physical_nav2_auto import PhysicalNav2Auto


def test_fresh_tf_pose_without_ssh():
    now = datetime.now(timezone.utc)
    pose = fresh_tf_pose({
        "ssh_connected": False,
        "pose": {"x": 1.2, "y": 0.4, "yaw": 0.1},
        "pose_checked_at": now.isoformat(),
        "pose_source": "tf:map->base_footprint",
    }, now=now)
    assert pose is not None
    assert pose["x"] == 1.2
    assert pose["source"] == "tf:map->base_footprint"


def test_present_robot_exposes_nav2_without_ssh():
    record = {
        "robot_id": "pinky-1",
        "name": "Pinky 1",
        "kind": "physical",
        "home": None,
        "ros_domain_id": 24,
        "physical_connection_enabled": True,
        "planned_physical": True,
    }
    now = datetime.now(timezone.utc).isoformat()
    robot = present_robot(
        record,
        ssh_robot=None,
        nav2={
            "robot_id": "pinky-1",
            "ssh_connected": False,
            "ros_online": True,
            "nav2_ready": True,
            "cmd_vel_chain_ready": True,
            "state": "ready",
            "detail": "ROS only",
            "map_matches_fms": True,
            "live_tf_status": "fresh",
            "pose": {"x": 1.1, "y": 0.2, "yaw": 0.0},
            "pose_checked_at": now,
            "pose_source": "tf:map->base_footprint",
        },
        geometry=None,
        worker={
            "robot_id": "pinky-1",
            "ros_domain_id": 24,
            "connection_state": "nav2_ready",
            "tf_status": "fresh",
            "receiver_ready": True,
        },
    )
    assert robot["ssh_online"] is False
    assert robot["ros_online"] is True
    assert robot["nav2_ready"] is True
    assert robot["pose"]["x"] == 1.1
    assert robot["connection_phase"] == "nav2_ready"


def test_snapshot_does_not_withhold_fresh_tf_for_ssh(monkeypatch):
    class _Settings:
        physical_nav2_robot_id = "pinky-1"
        physical_initial_pose_yaw_deg = 0.0
        physical_initial_pose_auto = False
        physical_ssh_targets = ""
        physical_ssh_password = ""
        physical_ssh_timeout_seconds = 1.0
        physical_map_yaml = "/tmp/none"
        data_dir = "/tmp/fms-test-data"

    monitor = PhysicalNav2Auto(_Settings())
    stamp = datetime.now(timezone.utc).isoformat()
    monitor._published_state = {
        "robot_id": "pinky-1",
        "ssh_connected": False,
        "amcl_active": True,
        "initial_pose_confirmed": False,
        "pose": None,
        "pose_checked_at": None,
        "map_matches_fms": True,
        "checked_at": stamp,
    }
    monitor._live_pose_seen = True
    monitor._live_observer_down = False
    monitor._live_lookup_failed = False
    monitor._live_pose = {"x": 0.5, "y": 0.6, "yaw": 0.0}
    monitor._live_pose_checked_at = stamp
    snap = monitor.snapshot()
    assert snap["live_tf_status"] == "fresh"
    assert snap["pose"]["x"] == 0.5
    assert snap.get("live_tf_withheld_reason") == ""
