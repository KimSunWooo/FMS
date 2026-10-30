"""R_effective planning lattice. Shared fixtures stay on 0.05 m so they cannot hide this."""
from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from app.core.config import ROOT, Settings
from app.main import create_app
from app.services.domain_status import status_path, write_status
from app.services.occupancy_grid import OccupancyGrid
from app.services.reservation_manager import (
    Interval,
    find_conflicts,
    footprint_corner_m,
    planning_profile,
    schedule_candidates,
)
from app.services.robot_geometry import (
    PINKY_BODY_LENGTH_M,
    PINKY_BODY_WIDTH_M,
    PINKY_CLEARANCE_M,
    PINKY_R_BODY_M,
    PINKY_R_EFFECTIVE_M,
)
from app.services.route_planner import k_shortest_paths, shortest_path
from app.services.route_preview import build_graph, inspect_point
from app.services.waypoint_graph import WaypointGraph
from conftest import FIXED, make_settings

MAP = ROOT / "config/maps/physical_site/1f_map_f.yaml"
R = PINKY_R_EFFECTIVE_M
PROFILE = planning_profile(R, R)


@pytest.fixture
def lattice_env(tmp_path: Path):
    settings = make_settings(
        tmp_path,
        planning_resolution_m=R,
        planning_robot_radius_m=R,
    )
    # Pinky-2 registry home is too close to occupied for R_effective clearance.
    fleet = json.loads(Path(settings.physical_fleet_path).read_text(encoding="utf-8"))
    for robot in fleet["robots"]:
        if robot["robot_id"] == "pinky-2":
            robot["home"] = {"x": 0.8, "y": 1.8, "yaw": 0.0}
    copy = tmp_path / "physical_fleet_reffective.json"
    copy.write_text(json.dumps(fleet), encoding="utf-8")
    settings.physical_fleet_path = copy
    app = create_app(settings)
    app.state.runtime.clock.freeze(FIXED)
    with TestClient(app) as client:
        yield client, app.state.runtime


def _graph_r() -> WaypointGraph:
    return build_graph(MAP, planning_resolution_m=R, robot_radius_m=R, allow_diagonal=True)


def _graph_05() -> WaypointGraph:
    return build_graph(MAP, planning_resolution_m=0.05, robot_radius_m=0.06, allow_diagonal=True)


def _seed_pinky1(runtime, x: float = 0.9, y: float = 1.2) -> None:
    write_status(status_path(runtime.settings.data_dir, "pinky-1"), {
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "kind": "physical",
        "physical_connection_enabled": True,
        "tf_status": "fresh",
        "pose": {"x": x, "y": y, "yaw": 0},
        "last_seen": datetime.now(timezone.utc).isoformat(),
        "connection_state": "connected",
    })


def test_settings_defaults_match_nav2_footprint_circumradius():
    assert math.isclose(PINKY_BODY_LENGTH_M, 0.12)
    assert math.isclose(PINKY_BODY_WIDTH_M, 0.12)
    assert math.isclose(PINKY_R_BODY_M, math.sqrt(0.12**2 + 0.12**2) / 2)
    assert math.isclose(PINKY_CLEARANCE_M, 0.01)
    assert math.isclose(PINKY_R_EFFECTIVE_M, PINKY_R_BODY_M + PINKY_CLEARANCE_M)
    assert math.isclose(Settings.model_fields["planning_resolution_m"].default, R)
    assert math.isclose(Settings.model_fields["planning_robot_radius_m"].default, R)
    assert math.isclose(Settings.model_fields["planning_depart_slot_s"].default, 0.05 / 0.15)
    assert PROFILE == planning_profile(R, R)
    # Radius is already R_effective — corner helper must not multiply sqrt(2) again.
    assert math.isclose(footprint_corner_m(R), R)


def test_effective_lattice_keeps_requested_spacing_on_unchanged_map():
    grid = OccupancyGrid.load(MAP)
    assert (grid.width, grid.height, grid.resolution) == (186, 236, 0.01)
    graph = _graph_r()
    g05 = _graph_05()
    assert graph.planning_resolution_m == R
    assert graph.requested_planning_resolution_m == R
    assert graph.resolution_applied_equals_requested is True
    assert graph.robot_radius_m == R
    assert (graph.width, graph.height) == (19, 24)
    a = graph.world((0, 0))
    b = graph.world((1, 0))
    d = graph.world((1, 1))
    assert math.isclose(math.hypot(b[0] - a[0], b[1] - a[1]), R)
    assert math.isclose(math.hypot(d[0] - a[0], d[1] - a[1]), R * math.sqrt(2))
    trav = sum(1 for c in range(graph.width) for r in range(graph.height) if graph.traversable((c, r)))
    trav05 = sum(1 for c in range(g05.width) for r in range(g05.height) if g05.traversable((c, r)))
    assert trav == 321
    assert trav05 == 1331
    assert trav < trav05


def test_api_exposes_geometry_and_applied_lattice(lattice_env):
    client, runtime = lattice_env
    assert math.isclose(runtime.settings.planning_resolution_m, R)
    assert math.isclose(runtime.settings.planning_robot_radius_m, R)
    starts = client.get("/api/planning/starts")
    assert starts.status_code == 200
    body = starts.json()["map"]
    assert math.isclose(body["planning_resolution_m"], R)
    assert math.isclose(body["robot_radius_m"], R)
    assert body["planning_resolution_applied_equals_requested"] is True
    assert body["planning_profile"] == PROFILE
    geom = body["robot_geometry"]
    assert math.isclose(geom["r_effective_m"], R)
    assert math.isclose(geom["r_body_m"], PINKY_R_BODY_M)
    assert math.isclose(geom["clearance_m"], PINKY_CLEARANCE_M)
    assert math.isclose(body["planning_depart_slot_s"], 0.05 / 0.15)


def test_real_map_paths_exist_on_effective_lattice():
    graph = _graph_r()
    start = graph.locate(0.9, 1.2)
    goal = graph.locate(1.2, 0.8)
    assert start[0] == "ok" and goal[0] == "ok"
    path = shortest_path(graph, start[1], goal[1])
    assert path is not None
    yen = k_shortest_paths(graph, start[1], goal[1], 3)
    assert 1 <= len(yen) <= 3


def test_start_goal_segment_and_reservation_share_radius(lattice_env):
    client, runtime = lattice_env
    graph = _graph_r()
    start = inspect_point(graph, 0.9, 1.2)
    goal = inspect_point(graph, 1.2, 0.8)
    assert start["reason"] == "ok" and goal["reason"] == "ok"
    assert start["traversable"] and goal["traversable"]
    # Same radius on clearance: a point closer than R to occupied is blocked.
    blocked = inspect_point(graph, 0.425, 1.925)
    assert blocked["reason"] == "clearance"

    _seed_pinky1(runtime, 0.9, 1.2)
    created = client.post("/api/fleet-tasks", json={
        "robot_id": "pinky-1",
        "start": {"x": 0.9, "y": 1.2},
        "goal": {"x": 1.2, "y": 0.8},
        "assignment_mode": "manual",
    })
    assert created.status_code in {200, 201}
    planned = client.post(f"/api/fleet-tasks/{created.json()['task_id']}/plan")
    assert planned.status_code == 200
    body = planned.json()
    assert math.isclose(body["schedule"]["planning_resolution_m"], R)
    assert math.isclose(body["schedule"]["robot_radius_m"], R)
    assert body["schedule"]["planning_profile"] == PROFILE


def test_legacy_profile_requires_replan_before_dispatch(lattice_env):
    client, runtime = lattice_env
    _seed_pinky1(runtime, 0.9, 1.2)
    created = client.post("/api/fleet-tasks", json={
        "robot_id": "pinky-1",
        "start": {"x": 0.9, "y": 1.2},
        "goal": {"x": 1.2, "y": 0.8},
        "assignment_mode": "manual",
    }).json()
    planned = client.post(f"/api/fleet-tasks/{created['task_id']}/plan").json()
    # Force a stale profile on the reserved schedule (previous 0.14 lattice).
    from app.models.tables import FleetTaskRecord
    from sqlalchemy import select

    with runtime.db.write() as session:
        task = session.scalars(select(FleetTaskRecord).where(FleetTaskRecord.id == planned["task_id"])).one()
        schedule = json.loads(task.schedule_json or "{}")
        schedule["planning_profile"] = "lattice-0.140-r0.096"
        task.schedule_json = json.dumps(schedule)
        session.flush()
    shown = client.get(f"/api/fleet-tasks/{planned['task_id']}").json()
    assert shown["schedule"]["needs_replan"] is True
    dispatched = client.post(f"/api/fleet-tasks/{planned['task_id']}/dispatch")
    assert dispatched.status_code == 409


def test_inter_robot_conflict_uses_sum_of_radii():
    plan_now = 1_700_000_000.0
    left = Interval(
        robot_id="pinky-1", task_id="t1", kind="vertex", resource_id="a",
        x=0.0, y=0.0, t_start=plan_now, t_end=plan_now + 1.0,
        planning_profile=PROFILE,
    )
    # Centers closer than 2R must conflict; farther must not.
    near = Interval(
        robot_id="pinky-2", task_id="t2", kind="vertex", resource_id="b",
        x=2 * R - 0.001, y=0.0, t_start=plan_now, t_end=plan_now + 1.0,
        planning_profile=PROFILE,
    )
    far = Interval(
        robot_id="pinky-2", task_id="t2", kind="vertex", resource_id="c",
        x=2 * R + 0.01, y=0.0, t_start=plan_now, t_end=plan_now + 1.0,
        planning_profile=PROFILE,
    )
    assert find_conflicts([left], [near], robot_radius_m=R, now_epoch=plan_now)
    assert not find_conflicts([left], [far], robot_radius_m=R, now_epoch=plan_now)


def test_schedule_candidates_use_same_radius():
    graph = _graph_r()
    start = graph.locate(0.9, 1.2)
    goal = graph.locate(1.2, 0.8)
    assert start[0] == "ok" and goal[0] == "ok"
    path = shortest_path(graph, start[1], goal[1])
    assert path is not None
    scheduled = schedule_candidates(
        graph, [path[0]], [],
        robot_id="pinky-1", task_id="t-sched",
        speed_m_s=0.15, dwell_s=0.5, robot_radius_m=R, max_depart_slots=8,
        depart_slot_s=0.05 / 0.15, current_profile=PROFILE,
    )
    assert scheduled is not None
    assert scheduled["planning_profile"] == PROFILE


def test_mid_edge_obstacle_blocks_neighbor_link():
    grid = OccupancyGrid.from_rows([
        ".....",
        "..#..",
        ".....",
    ], resolution=1.0)
    graph = WaypointGraph(grid, planning_resolution_m=1.0, robot_radius_m=0.0, allow_diagonal=False)
    assert graph.traversable((0, 1))
    assert graph.traversable((4, 1))
    assert not graph.traversable((2, 1))
    assert graph.segment_clear(0.5, 1.5, 4.5, 1.5) is False
    neighbor_ids = {node for node, _cost in graph.neighbors((0, 1))}
    assert (4, 1) not in neighbor_ids
    assert (1, 1) in neighbor_ids


def test_diagonal_corner_cutting_still_blocked():
    grid = OccupancyGrid.from_rows([
        ".#",
        "#.",
    ], resolution=1.0)
    graph = WaypointGraph(grid, planning_resolution_m=1.0, robot_radius_m=0.0, allow_diagonal=True)
    assert graph.traversable((0, 1))
    assert graph.traversable((1, 0))
    assert (1, 0) not in {node for node, _ in graph.neighbors((0, 1))}


def test_start_and_goal_snap_preserves_pose_and_checks_segment():
    graph = _graph_r()
    cases = {
        "pinky-2-registry-home": (0.425, 1.925),
        "pinky-3-registry-home": (0.425, 0.575),
        "pinky-1-observed": (1.696, 0.103),
        "test-start": (0.9, 1.2),
        "test-goal": (1.2, 0.8),
    }
    results = {}
    for name, (x, y) in cases.items():
        reason, node = graph.locate(x, y)
        inspection = inspect_point(graph, x, y)
        assert inspection["x"] == x and inspection["y"] == y
        results[name] = (reason, node, inspection)
        if reason == "ok":
            assert node is not None
            sx, sy = graph.world(node)
            assert graph.segment_clear(x, y, sx, sy)
            assert inspection["lattice_connection"]["is_robot_pose"] is False
            assert inspection["lattice_connection"]["segment_checked"] is True
        else:
            assert inspection["traversable"] is False
            assert inspection["lattice_connection"] is None
    assert results["pinky-2-registry-home"][0] == "clearance"
    assert results["pinky-3-registry-home"][0] == "ok"
    assert results["pinky-1-observed"][0] == "ok"
    assert results["test-start"][0] == "ok"
    assert results["test-goal"][0] == "ok"


def test_map_boundary_adjacent_points_fail_explicitly():
    graph = _graph_r()
    outside = inspect_point(graph, -0.1, 1.0)
    assert outside["reason"] == "outside"
    assert outside["traversable"] is False
    edge = inspect_point(graph, 0.05, 0.05)
    assert edge["traversable"] is False
    assert edge["reason"] in {"outside", "occupied", "unknown", "clearance", "snap_blocked"}


def test_depart_slot_stays_near_one_third_second(lattice_env):
    client, runtime = lattice_env
    _seed_pinky1(runtime)
    created = client.post("/api/fleet-tasks", json={
        "assignment_mode": "manual",
        "robot_id": "pinky-2",
        "start": {"x": 0.8, "y": 1.8},
        "goal": {"x": 1.2, "y": 0.8},
        "priority": 5,
        "task_type": "navigate",
        "label": "slot",
    })
    planned = client.post(f"/api/fleet-tasks/{created.json()['task_id']}/plan")
    assert planned.status_code == 200, planned.text
    body = planned.json()
    assert body["status"] == "RESERVED"
    assert body["schedule"]["planning_profile"] == PROFILE
    assert math.isclose(body["schedule"]["depart_slot_s"], 0.05 / 0.15)
    assert math.isclose(body["schedule"]["planning_resolution_m"], R)
    assert body["start"]["x"] == 0.8
    assert body["start"]["y"] == 1.8


def test_three_robots_assign_with_zero_plan_clock_conflicts(lattice_env):
    client, runtime = lattice_env
    _seed_pinky1(runtime)
    goals = [
        {"x": 0.8, "y": 0.6},
        {"x": 1.0, "y": 1.0},
        {"x": 1.2, "y": 0.8},
        {"x": 1.1, "y": 1.6},
        {"x": 1.5, "y": 1.5},
    ]
    manual = client.post("/api/fleet-tasks", json={
        "assignment_mode": "manual",
        "robot_id": "pinky-2",
        "start": {"x": 0.8, "y": 1.8},
        "goal": {"x": 0.7, "y": 1.5},
        "priority": 5,
        "task_type": "navigate",
        "label": "manual",
    })
    planned_manual = client.post(f"/api/fleet-tasks/{manual.json()['task_id']}/plan")
    assert planned_manual.json()["status"] == "RESERVED"
    reserved = [planned_manual.json()]
    for index, goal in enumerate(goals):
        created = client.post("/api/fleet-tasks", json={
            "assignment_mode": "automatic",
            "goal": goal,
            "priority": 2 + index,
            "task_type": "navigate",
            "label": f"auto-{index}",
        })
        planned = client.post(f"/api/fleet-tasks/{created.json()['task_id']}/plan")
        assert planned.status_code == 200, planned.text
        body = planned.json()
        assert body["status"] in {"RESERVED", "WAITING"}, body
        if body["status"] == "RESERVED":
            reserved.append(body)
            assert body["schedule"]["planning_profile"] == PROFILE
    assigned = {item["assigned_robot_id"] for item in reserved}
    assert assigned == {"pinky-1", "pinky-2", "pinky-3"}, reserved
    stored = client.get("/api/planning/reservations").json()["reservations"]
    graph = _graph_r()
    from app.services.reservation_manager import prepare_interval
    intervals = [
        prepare_interval(graph, Interval(
            item["robot_id"], item["task_id"], item["resource_kind"], item["resource_id"],
            item["x"], item["y"], item["t_start"], item["t_end"], None, item["time_basis"],
            planning_profile=item.get("planning_profile") or "",
        ), PROFILE)
        for item in stored
    ]
    plan_now = min(item.t_start for item in intervals) - 1.0
    hits = find_conflicts(intervals, intervals, robot_radius_m=R, now_epoch=plan_now)
    assert hits == []


def test_same_path_allowed_at_different_times():
    graph = _graph_r()
    start = graph.locate(0.9, 1.2)[1]
    goal = graph.locate(1.2, 0.8)[1]
    path = shortest_path(graph, start, goal)[0]
    first = schedule_candidates(
        graph, [path], [],
        robot_id="pinky-1", task_id="a",
        speed_m_s=0.15, dwell_s=0.5, robot_radius_m=R, max_depart_slots=8,
        origin_epoch=1_700_000_000.0, now_epoch=1_700_000_000.0,
        depart_slot_s=0.05 / 0.15, current_profile=PROFILE,
    )
    assert first is not None
    second = schedule_candidates(
        graph, [path], first["intervals"],
        robot_id="pinky-2", task_id="b",
        speed_m_s=0.15, dwell_s=0.5, robot_radius_m=R, max_depart_slots=40,
        origin_epoch=1_700_000_000.0, now_epoch=1_700_000_000.0,
        depart_slot_s=0.05 / 0.15, current_profile=PROFILE,
    )
    assert second is not None
    assert second["depart_s"] > first["depart_s"]
    assert second["arrival_epoch"] > first["arrival_epoch"]


def test_old_profile_reservation_is_not_reinterpreted_or_dispatched(lattice_env):
    client, runtime = lattice_env
    _seed_pinky1(runtime)
    from app.models.tables import FleetReservationRecord, FleetTaskRecord
    from app.services.fleet_task_service import map_revision
    revision = map_revision(runtime.settings.physical_map_yaml)
    now = datetime.now(timezone.utc)
    with runtime.db.write() as session:
        session.add(FleetTaskRecord(
            id="legacy-profile-task",
            task_type="navigate",
            priority=5,
            assignment_mode="manual",
            requested_robot_id="pinky-2",
            assigned_robot_id="pinky-2",
            start_x=0.9,
            start_y=1.2,
            start_source="request",
            goal_x=1.2,
            goal_y=0.8,
            goal_label="legacy",
            map_id="1f_map_f",
            map_revision=revision,
            selected_path_id="path-1",
            planned_path_json="[]",
            candidate_paths_json="[]",
            schedule_json=json.dumps({
                "planning_profile": "lattice-0.140-r0.096",
                "payload": {"execute": False, "task_id": "legacy-profile-task", "robot_id": "pinky-2"},
                "needs_replan": False,
            }),
            payload_hash="legacy",
            status="RESERVED",
            delivery_status="NOT_SENT",
            ack_status="NOT_RECEIVED",
            ack_channel="none",
            ack_json="",
            failure_reason="",
            assignment_reason="legacy lattice",
            cancellation_json="",
            motion_verified=False,
            is_archived=False,
            archived_by="",
            created_at=now,
            updated_at=now,
        ))
    with runtime.db.write() as session:
        session.add(FleetReservationRecord(
            id="legacy-profile-res",
            task_id="legacy-profile-task",
            robot_id="pinky-2",
            resource_kind="vertex",
            resource_id="v-18-24",
            x=0.925,
            y=1.225,
            t_start=1_700_000_100.0,
            t_end=1_700_000_110.0,
            status="confirmed",
            time_basis="absolute",
            occupancy_class="planned",
            map_revision=revision,
            planning_profile="lattice-0.140-r0.096",
            created_at=now,
        ))
    viewed = client.get("/api/fleet-tasks/legacy-profile-task").json()
    assert viewed["status"] == "RESERVED"
    assert viewed["schedule"]["needs_replan"] is True
    assert viewed["schedule"]["planning_profile_current"] == PROFILE
    refused = client.post("/api/fleet-tasks/legacy-profile-task/dispatch")
    assert refused.status_code == 409
    still = client.get("/api/fleet-tasks/legacy-profile-task").json()
    assert still["status"] == "RESERVED"
    assert still["ack_status"] == "NOT_RECEIVED"


def test_path_comparison_fine_vs_effective():
    g05 = _graph_05()
    g_r = _graph_r()
    s05, g05n = g05.locate(0.9, 1.2)[1], g05.locate(1.2, 0.8)[1]
    sr, grn = g_r.locate(0.9, 1.2)[1], g_r.locate(1.2, 0.8)[1]
    p05 = shortest_path(g05, s05, g05n)
    pr = shortest_path(g_r, sr, grn)
    assert p05 is not None and pr is not None
    assert len(pr[0]) < len(p05[0])
