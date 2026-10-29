"""Robot domains stay on the registry. Workers do not share TF samples."""
from __future__ import annotations

import json
import os
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace

from app.core.config import ROOT
from app.services.domain_status import build_status, interpret_status, resolve_robot_domain, write_status, status_path
from app.services.domain_supervisor import DomainSupervisor
from app.services.domain_worker import run_once
from app.services.fleet_registry import RegistryError, load_registry, present_robot
from app.services.forward_5cm_service import PinkyFiveCmTest
from app.services.physical_nav2_auto import PhysicalNav2Auto
import pytest


def test_registry_assigns_distinct_domains_without_moving_homes():
    robots = load_registry(ROOT / "config" / "physical_fleet.json")
    assert [(robot["robot_id"], robot["ros_domain_id"], robot["kind"]) for robot in robots] == [
        ("pinky-1", 24, "physical"),
        ("pinky-2", 25, "virtual"),
        ("pinky-3", 26, "virtual"),
    ]
    assert robots[1]["physical_connection_enabled"] is False
    assert robots[1]["planned_physical"] is True
    assert robots[1]["home"] == {"x": 0.425, "y": 1.925, "yaw": 0.0}
    assert robots[2]["home"] == {"x": 0.425, "y": 0.575, "yaw": 0.0}
    shown = present_robot(
        robots[1],
        ssh_robot={"host": "should-not-apply"},
        nav2={"robot_id": "pinky-1", "pose": {"x": 1.7, "y": 0.1, "yaw": 0}, "pose_checked_at": "2026-10-28T00:00:00+00:00", "ssh_connected": True},
        geometry=None,
        worker={
            "robot_id": "pinky-1",
            "ros_domain_id": 24,
            "pose": {"x": 1.7, "y": 0.1, "yaw": 0},
            "tf_status": "fresh",
            "connection_state": "nav2_ready",
        },
    )
    assert shown["pose"] is None
    assert shown["ros_domain_id"] == 25
    assert shown["connection_phase"] == "virtual"
    assert shown["worker"] is None
    assert shown["motion_allowed"] is False
    assert shown["task_assignable"] is True


def test_virtual_connection_cannot_be_enabled_without_a_kind_change(tmp_path: Path):
    payload = json.loads((ROOT / "config" / "physical_fleet.json").read_text(encoding="utf-8"))
    payload["robots"][1]["physical_connection_enabled"] = True
    path = tmp_path / "fleet.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(RegistryError, match="stays virtual"):
        load_registry(path)


def test_unconfigured_domain_does_not_keep_a_pose():
    robot = load_registry()[1]
    status = run_once(
        robot,
        25,
        {"pose": {"x": 1, "y": 2, "yaw": 0}, "ros_connected": True, "tf_status": "fresh", "nav2_ready": True},
    )
    assert status["pose"] is None
    assert status["ros_connected"] is False
    assert status["connection_state"] == "physical_connection_not_configured"
    mismatch = build_status(load_registry()[0], 25, {"pose": {"x": 1, "y": 1, "yaw": 0}, "tf_status": "fresh"})
    assert mismatch["connection_state"] == "error"
    assert mismatch["pose"] is None


def test_stale_or_foreign_sample_is_dropped():
    pinky1, pinky2 = load_registry()[0], load_registry()[1]
    fresh = build_status(pinky1, 24, {
        "ros_connected": True,
        "nav2_ready": True,
        "tf_status": "fresh",
        "pose": {"x": 1.7, "y": 0.1, "yaw": 0},
        "connection_state": "nav2_ready",
    })
    fresh["last_seen"] = datetime.now(timezone.utc).isoformat()
    reused = interpret_status(fresh, pinky2)
    assert reused["robot_id"] == "pinky-2"
    assert reused["pose"] is None
    fresh["last_seen"] = (datetime.now(timezone.utc) - timedelta(seconds=30)).isoformat()
    expired = interpret_status(fresh, pinky1)
    assert expired["pose"] is None
    assert expired["tf_status"] == "expired"
    assert expired["connection_state"] == "disconnected"


def test_supervisor_restarts_a_dead_worker_without_changing_the_parent_domain(tmp_path: Path):
    class Proc:
        def __init__(self):
            self.dead = False

        def poll(self):
            return 0 if self.dead else None

        def terminate(self):
            self.dead = True

        def wait(self, timeout=0):
            self.dead = True

        def kill(self):
            self.dead = True

    spawned: list[dict] = []

    def popen(cmd, **kwargs):
        spawned.append(kwargs["env"])
        return Proc()

    before = os.environ.get("ROS_DOMAIN_ID")
    settings = SimpleNamespace(
        physical_fleet_path=ROOT / "config" / "physical_fleet.json",
        data_dir=tmp_path,
    )
    supervisor = DomainSupervisor(settings, popen=popen)
    robots = load_registry()
    first = supervisor.ensure(robots[0])
    again = supervisor.ensure(robots[0])
    assert again is first
    assert len(spawned) == 1
    first.dead = True
    supervisor.ensure(robots[0])
    supervisor.ensure(robots[1])
    supervisor.ensure(robots[2])
    assert [env["ROS_DOMAIN_ID"] for env in spawned] == ["24", "24", "25", "26"]
    assert [env["FMS_WORKER_ROBOT_ID"] for env in spawned] == ["pinky-1", "pinky-1", "pinky-2", "pinky-3"]
    assert os.environ.get("ROS_DOMAIN_ID") == before
    assert "FMS_PHYSICAL_SSH_PASSWORD" not in spawned[0]


def test_worker_files_stay_on_their_own_robot(tmp_path: Path):
    now = datetime.now(timezone.utc).isoformat()
    for robot, pose in ((load_registry()[0], {"x": 1.1, "y": 0.2, "yaw": 0}), (load_registry()[1], {"x": 9, "y": 9, "yaw": 1})):
        payload = build_status(robot, robot["ros_domain_id"])
        payload["last_seen"] = now
        if robot["robot_id"] == "pinky-1":
            payload["pose"] = pose
            payload["tf_status"] = "fresh"
            payload["connection_state"] = "ros_connected"
        write_status(status_path(tmp_path, robot["robot_id"]), payload)
    settings = SimpleNamespace(physical_fleet_path=ROOT / "config" / "physical_fleet.json", data_dir=tmp_path)
    supervisor = DomainSupervisor(settings, popen=lambda *args, **kwargs: None)
    class Alive:
        def poll(self):
            return None
    for robot in load_registry():
        supervisor.children[robot["robot_id"]] = Alive()
    rows = {row["robot_id"]: row for row in supervisor.statuses()}
    assert rows["pinky-1"]["pose"]["x"] == 1.1
    assert rows["pinky-2"]["pose"] is None
    assert rows["pinky-2"]["ros_domain_id"] == 25
    assert rows["pinky-2"]["connection_state"] == "physical_connection_not_configured"
    assert rows["pinky-3"]["ros_domain_id"] == 26
    assert rows["pinky-1"]["ros_domain_id"] == 24


def test_pinky1_commands_follow_the_registry_domain():
    settings = SimpleNamespace(
        physical_fleet_path=ROOT / "config" / "physical_fleet.json",
        ros_domain_id=73,
        physical_nav2_robot_id="pinky-1",
        physical_initial_pose_yaw_deg=0.0,
    )
    assert resolve_robot_domain("pinky-2", settings) == 25
    monitor = PhysicalNav2Auto(settings)
    assert monitor._env()["ROS_DOMAIN_ID"] == "24"
    service = PinkyFiveCmTest(settings, monitor)
    assert service._env()["ROS_DOMAIN_ID"] == "24"
    assert service._env()["ROS_DOMAIN_ID"] != "25"
    assert service._env()["ROS_DOMAIN_ID"] != "26"
