"""Absolute reservations, per-robot pose, queue order, and plan-time conflicts."""
from __future__ import annotations

from datetime import datetime, timezone

from app.models.tables import FleetReservationRecord
from app.services.domain_status import status_path, write_status
from app.services.reservation_manager import (
    EPOCH_FLOOR,
    Interval,
    classify_time_basis,
    find_conflicts,
    schedule_candidates,
)
from sqlalchemy import select

from test_fleet_tasks import GOAL, START, _create


def test_relative_timestamps_stay_legacy_and_block_absolute_plans():
    assert classify_time_basis(13.5, "absolute") == "legacy"
    assert classify_time_basis(EPOCH_FLOOR + 10, "absolute") == "absolute"
    assert classify_time_basis(EPOCH_FLOOR + 10, "legacy") == "legacy"
    legacy = Interval("pinky-1", "old", "vertex", "v-1-1", 1.0, 1.0, 0.0, 2.0, (1, 1), "legacy")
    future = Interval("pinky-2", "new", "vertex", "v-1-1", 1.0, 1.0, 1_700_000_000, 1_700_000_002, (1, 1))
    kinds = {item["conflict_type"] for item in find_conflicts([future], [legacy], robot_radius_m=0)}
    assert "legacy_time_basis" in kinds
    assert "vertex" in kinds


def test_past_due_plan_still_occupies_until_motion_is_verified():
    stale = Interval("pinky-1", "old", "vertex", "v-1-1", 1.0, 1.0, 1_700_000_000, 1_700_000_010, (1, 1))
    later = Interval("pinky-2", "new", "vertex", "v-1-1", 1.0, 1.0, 1_700_000_100, 1_700_000_110, (1, 1))
    kinds = {
        item["conflict_type"]
        for item in find_conflicts([later], [stale], robot_radius_m=0, now_epoch=1_700_000_050)
    }
    assert "delayed_motion" in kinds


def test_edge_vertex_following_and_proximity_use_square_corner():
    occupied = Interval("pinky-1", "a", "vertex", "v-2-0", 1.0, 0.0, 0.0, 1.0, (2, 0))
    passing = Interval(
        "pinky-2", "b", "edge", "e-0,0>2,0", 1.0, 0.0, 0.0, 1.0,
        ax=0.0, ay=0.0, bx=2.0, by=0.0,
    )
    assert "edge_vertex" in {
        item["conflict_type"] for item in find_conflicts([passing], [occupied], robot_radius_m=0.06)
    }
    follower = Interval(
        "pinky-2", "b", "edge", "e-0,0>1,0", 0.5, 0.0, 0.0, 1.0,
        ax=0.0, ay=0.0, bx=1.0, by=0.0,
    )
    leader = Interval(
        "pinky-1", "a", "edge", "e-0,0>1,0", 0.5, 0.0, 0.0, 1.0,
        ax=0.0, ay=0.0, bx=1.0, by=0.0,
    )
    assert "following" in {
        item["conflict_type"] for item in find_conflicts([follower], [leader], robot_radius_m=0.06)
    }
    near = Interval(
        "pinky-2", "b", "edge", "e-0,0>0,1", 0.5, 0.02, 0.0, 1.0,
        ax=0.5, ay=-0.02, bx=0.5, by=0.02,
    )
    lateral = Interval(
        "pinky-1", "a", "edge", "e-0,0>1,0", 0.5, 0.0, 0.0, 1.0,
        ax=0.0, ay=0.0, bx=1.0, by=0.0,
    )
    assert "edge_proximity" in {
        item["conflict_type"] for item in find_conflicts([near], [lateral], robot_radius_m=0.06)
    }
    far = Interval(
        "pinky-2", "b", "edge", "e-0,2>1,2", 0.5, 2.0, 0.0, 1.0,
        ax=0.0, ay=2.0, bx=1.0, by=2.0,
    )
    assert find_conflicts([far], [lateral], robot_radius_m=0.06) == []


def test_earliest_arrival_beats_the_first_feasible_path(monkeypatch):
    def time_path(graph, nodes, *, depart_s, speed_m_s, dwell_s, robot_id, task_id):
        travel = 4.0 if nodes[0] == (1, 0) else 5.0
        return [Interval(robot_id, task_id, "vertex", "v", 0.0, 0.0, depart_s, depart_s + travel, nodes[0])]

    def conflicts(proposed, existing, **kwargs):
        if proposed and proposed[0].node == (1, 0) and proposed[0].t_start < 1_700_000_003:
            return [{"conflict_type": "vertex"}]
        return []

    monkeypatch.setattr("app.services.reservation_manager.time_path", time_path)
    monkeypatch.setattr("app.services.reservation_manager.find_conflicts", conflicts)

    class Graph:
        planning_resolution_m = 1.0

        def world(self, node):
            return (0.0, 0.0)

    chosen = schedule_candidates(
        Graph(), [[(1, 0)], [(2, 0)]], [],
        robot_id="pinky-2", task_id="task", speed_m_s=1.0, dwell_s=0.5,
        robot_radius_m=0.0, max_depart_slots=8, origin_epoch=1_700_000_000,
    )
    assert chosen is not None
    assert chosen["nodes"] == [(2, 0)]
    assert chosen["depart_s"] == 0
    assert chosen["arrival_epoch"] == 1_700_000_005


def test_distinct_origins_compare_on_one_clock(monkeypatch):
    def time_path(graph, nodes, *, depart_s, speed_m_s, dwell_s, robot_id, task_id):
        return [Interval(robot_id, task_id, "vertex", "v-1-1", 1.0, 1.0, depart_s, depart_s + 1, (1, 1))]

    monkeypatch.setattr("app.services.reservation_manager.time_path", time_path)

    class Graph:
        planning_resolution_m = 1.0

        def world(self, node):
            return (1.0, 1.0)

    existing = [Interval("pinky-1", "a", "vertex", "v-1-1", 1.0, 1.0, 1_700_000_000, 1_700_000_001, (1, 1))]
    same = schedule_candidates(
        Graph(), [[(0, 0)]], existing,
        robot_id="pinky-2", task_id="b", speed_m_s=1, dwell_s=0.5,
        robot_radius_m=0, max_depart_slots=4, origin_epoch=1_700_000_000,
    )
    later = schedule_candidates(
        Graph(), [[(0, 0)]], existing,
        robot_id="pinky-2", task_id="c", speed_m_s=1, dwell_s=0.5,
        robot_radius_m=0, max_depart_slots=4, origin_epoch=1_700_000_001,
    )
    assert same is not None and same["depart_s"] == 1
    assert later is not None and later["depart_s"] == 0
    assert later["intervals"][0].t_start == 1_700_000_001


def test_reserved_times_are_utc_epochs(env):
    client, runtime = env
    planned = client.post(f"/api/fleet-tasks/{_create(client)['task_id']}/plan").json()
    assert planned["estimated_start_time"].endswith("+00:00")
    assert planned["schedule"]["time_basis"] == "absolute"
    assert planned["schedule"]["origin_epoch"] > EPOCH_FLOOR
    with runtime.db.read() as session:
        row = session.scalars(select(FleetReservationRecord)).first()
        assert row is not None
        assert row.t_start > EPOCH_FLOOR
        assert row.time_basis == "absolute"
    restarted = client.app.state.settings
    from app.services.fleet_task_service import FleetTaskService
    service = FleetTaskService(restarted, runtime.db, lambda: None)
    again = service.get(planned["task_id"])
    assert again["status"] == "RESERVED"
    assert again["schedule"]["time_basis"] == "absolute"


def test_pinky_tf_is_not_shared_across_domains(env):
    client, runtime = env
    now = datetime.now(timezone.utc).isoformat()
    write_status(status_path(runtime.settings.data_dir, "pinky-2"), {
        "robot_id": "pinky-2",
        "ros_domain_id": 25,
        "tf_status": "fresh",
        "pose": {"x": 0.2, "y": 0.2, "yaw": 0},
        "last_seen": now,
        "connection_state": "connected",
    })
    write_status(status_path(runtime.settings.data_dir, "pinky-1"), {
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "kind": "physical",
        "physical_connection_enabled": True,
        "tf_status": "fresh",
        "pose": {"x": START["x"], "y": START["y"], "yaw": 0},
        "last_seen": now,
        "connection_state": "connected",
        "ssh_connected": True,
    })
    mismatched = client.post(
        f"/api/fleet-tasks/{_create(client, robot_id='pinky-1', start={'x': 0.2, 'y': 0.2})['task_id']}/plan"
    ).json()
    assert mismatched["status"] == "FAILED"
    assert mismatched["failure_reason"] == "start_mismatch"
    matched = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
    assert matched["status"] == "RESERVED"
    assert matched["start"]["source"] == "domain-24:worker-tf"
    assert abs(matched["start"]["x"] - START["x"]) < 1e-6


def test_same_robot_queue_waits_for_verified_motion(env):
    client, _runtime = env
    first = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-2')['task_id']}/plan").json()
    second = client.post(
        f"/api/fleet-tasks/{_create(client, robot_id='pinky-2', goal={'x': 0.7, 'y': 1.5})['task_id']}/plan"
    ).json()
    assert first["status"] == "RESERVED"
    assert second["status"] == "RESERVED"
    assert second["predecessor_task_id"] == first["task_id"]
    assert second["start"]["source"] == "predecessor_goal"
    assert abs(second["start"]["x"] - first["goal"]["x"]) < 1e-6
    assert abs(second["start"]["y"] - first["goal"]["y"]) < 1e-6
    blocked = client.post(f"/api/fleet-tasks/{second['task_id']}/dispatch")
    assert blocked.status_code == 409
    delivered = client.post(f"/api/fleet-tasks/{first['task_id']}/dispatch").json()
    assert delivered["status"] == "DELIVERED"
    assert delivered["motion_verified"] is False
    still = client.post(f"/api/fleet-tasks/{second['task_id']}/dispatch")
    assert still.status_code == 409
    verified = client.post(f"/api/fleet-tasks/{first['task_id']}/simulated-motion").json()
    assert verified["motion_verified"] is True
    assert verified["status"] == "DELIVERED"
    followed = client.post(f"/api/fleet-tasks/{second['task_id']}/dispatch")
    assert followed.status_code == 200, followed.text


def test_physical_ack_is_not_simulated_completion(env):
    client, _runtime = env
    task = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-1')['task_id']}/plan").json()
    delivered = client.post(f"/api/fleet-tasks/{task['task_id']}/dispatch").json()
    assert delivered["status"] == "ACKNOWLEDGED"
    rejected = client.post(f"/api/fleet-tasks/{task['task_id']}/simulated-motion")
    assert rejected.status_code == 409
    assert client.get(f"/api/fleet-tasks/{task['task_id']}").json()["motion_verified"] is False


def test_priority_plans_before_an_older_lower_task_without_preemption(env):
    client, _runtime = env
    low = _create(client, assignment_mode="automatic", robot_id=None, priority=1)
    high = _create(client, assignment_mode="automatic", robot_id=None, priority=10)
    ordered = client.post("/api/fleet-tasks/replan").json()["results"]
    assert ordered[0]["task_id"] == high["task_id"]
    assert ordered[0]["status"] == "RESERVED"
    assert "priority 10" in (ordered[0]["assignment_reason"] or "")
    held = client.get(f"/api/fleet-tasks/{low['task_id']}").json()
    assert held["status"] == "RESERVED"
    urgent = _create(client, assignment_mode="automatic", robot_id=None, priority=10)
    client.post("/api/fleet-tasks/replan")
    kept = client.get(f"/api/fleet-tasks/{held['task_id']}").json()
    assert kept["status"] == "RESERVED"
    assert kept["assigned_robot_id"] == held["assigned_robot_id"]
    assert client.get(f"/api/fleet-tasks/{urgent['task_id']}").json()["status"] in {"RESERVED", "WAITING"}


def test_cancelling_a_legacy_blocker_replans_the_waiting_task(env):
    client, runtime = env
    blocker = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-2')['task_id']}/plan").json()
    with runtime.db.write() as session:
        rows = session.scalars(select(FleetReservationRecord).where(
            FleetReservationRecord.task_id == blocker["task_id"],
        )).all()
        for row in rows:
            row.time_basis = "legacy"
            row.t_start = 1.0
            row.t_end = 2.0
    waiting = client.post(f"/api/fleet-tasks/{_create(client, robot_id='pinky-3')['task_id']}/plan").json()
    assert waiting["status"] == "WAITING"
    cancelled = client.post(f"/api/fleet-tasks/{blocker['task_id']}/cancel").json()
    assert cancelled["status"] == "CANCELLED"
    released = client.get("/api/planning/reservations").json()["reservations"]
    assert all(item["task_id"] != blocker["task_id"] or item["time_basis"] == "legacy" for item in released)
    again = client.get(f"/api/fleet-tasks/{waiting['task_id']}").json()
    assert again["status"] == "RESERVED"
    assert again["schedule"]["time_basis"] == "absolute"
    assert again["schedule"]["origin_epoch"] > EPOCH_FLOOR
