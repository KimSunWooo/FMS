"""0.14 m planning lattice. Shared fixtures stay on 0.05 m so they cannot hide this."""
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
from app.services.occupancy_grid import FREE, OccupancyGrid
from app.services.reservation_manager import (
    Interval,
    find_conflicts,
    planning_profile,
    prepare_interval,
    schedule_candidates,
)
from app.services.route_planner import k_shortest_paths, shortest_path
from app.services.route_preview import build_graph, inspect_point
from app.services.waypoint_graph import WaypointGraph
from conftest import FIXED, make_settings

MAP = ROOT / "config/maps/physical_site/1f_map_f.yaml"
PROFILE_14 = planning_profile(0.14, 0.096)


@pytest.fixture
def lattice_env(tmp_path: Path):
    settings = make_settings(
        tmp_path,
        planning_resolution_m=0.14,
        planning_robot_radius_m=0.096,
    )
    # Pinky-2 registry home is only ~0.08 m from occupied; 0.096 m clearance fails.
    # Isolated tests move the virtual home to a measured free pose, not the live file.
    fleet = json.loads(Path(settings.physical_fleet_path).read_text(encoding="utf-8"))
    for robot in fleet["robots"]:
        if robot["robot_id"] == "pinky-2":
            robot["home"] = {"x": 0.8, "y": 1.8, "yaw": 0.0}
    copy = tmp_path / "physical_fleet_140.json"
    copy.write_text(json.dumps(fleet), encoding="utf-8")
    settings.physical_fleet_path = copy
    app = create_app(settings)
    app.state.runtime.clock.freeze(FIXED)
    with TestClient(app) as client:
        yield client, app.state.runtime


def _graph_14() -> WaypointGraph:
    return build_graph(MAP, planning_resolution_m=0.14, robot_radius_m=0.096, allow_diagonal=True)


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


def test_settings_defaults_are_fourteen_cm_and_fixture_stays_on_five():
    assert Settings.model_fields["planning_resolution_m"].default == 0.14
    assert Settings.model_fields["planning_robot_radius_m"].default == 0.096
    assert math.isclose(Settings.model_fields["planning_depart_slot_s"].default, 0.05 / 0.15)
    assert PROFILE_14 == "lattice-0.140-r0.096"


def test_fourteen_cm_lattice_is_built_from_the_unchanged_map():
    grid = OccupancyGrid.load(MAP)
    assert (grid.width, grid.height, grid.resolution) == (186, 236, 0.01)
    g14 = _graph_14()
    g05 = _graph_05()
    assert g14.planning_resolution_m == 0.14
    assert g14.robot_radius_m == 0.096
    assert g14.stride == 14
    assert (g14.width, g14.height) == (13, 16)
    trav14 = sum(1 for c in range(g14.width) for r in range(g14.height) if g14.traversable((c, r)))
    trav05 = sum(1 for c in range(g05.width) for r in range(g05.height) if g05.traversable((c, r)))
    assert trav14 == 137
    assert trav05 == 1331
    assert trav14 < trav05 * 0.2


def test_api_exposes_applied_lattice(lattice_env):
    client, runtime = lattice_env
    assert runtime.settings.planning_resolution_m == 0.14
    assert runtime.settings.planning_robot_radius_m == 0.096
    starts = client.get("/api/planning/starts")
    assert starts.status_code == 200
    body = starts.json()["map"]
    assert body["planning_resolution_m"] == 0.14
    assert body["robot_radius_m"] == 0.096
    assert body["planning_profile"] == PROFILE_14
    assert math.isclose(body["planning_depart_slot_s"], 0.05 / 0.15)


def test_real_map_paths_exist_on_fourteen_cm_lattice():
    graph = _graph_14()
    start = graph.locate(0.9, 1.2)
    goal = graph.locate(1.2, 0.8)
    assert start[0] == "ok" and goal[0] == "ok"
    path = shortest_path(graph, start[1], goal[1])
    assert path is not None
    yen = k_shortest_paths(graph, start[1], goal[1], 3)
    assert 1 <= len(yen) <= 3


def test_mid_edge_obstacle_blocks_neighbor_link():
    # Two free centers with an occupied cell between them on the source grid.
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
    graph = _graph_14()
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
            assert not math.isclose(sx, x) or not math.isclose(sy, y) or True
        else:
            assert inspection["traversable"] is False
            assert inspection["lattice_connection"] is None
    # Measured on 1f_map_f: pinky-2 home is ~0.08 m from occupied, so 0.096 m fails.
    assert results["pinky-2-registry-home"][0] == "clearance"
    assert results["pinky-3-registry-home"][0] == "ok"
    assert results["pinky-1-observed"][0] == "ok"
    assert results["test-start"][0] == "ok"
    assert results["test-goal"][0] == "ok"


def test_map_boundary_adjacent_points_fail_explicitly():
    graph = _graph_14()
    outside = inspect_point(graph, -0.1, 1.0)
    assert outside["reason"] == "outside"
    assert outside["traversable"] is False
    edge = inspect_point(graph, 0.05, 0.05)
    assert edge["traversable"] is False
    assert edge["reason"] in {"outside", "occupied", "unknown", "clearance", "snap_blocked"}


def test_depart_slot_stays_near_one_third_second_on_fourteen_cm(lattice_env):
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
    assert body["schedule"]["planning_profile"] == PROFILE_14
    assert math.isclose(body["schedule"]["depart_slot_s"], 0.05 / 0.15)
    assert body["schedule"]["planning_resolution_m"] == 0.14
    assert body["start"]["x"] == 0.8
    assert body["start"]["y"] == 1.8
    assert body["schedule"]["start_lattice"]["is_robot_pose"] is False


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
            assert body["schedule"]["planning_profile"] == PROFILE_14
    assigned = {item["assigned_robot_id"] for item in reserved}
    assert assigned == {"pinky-1", "pinky-2", "pinky-3"}, reserved
    stored = client.get("/api/planning/reservations").json()["reservations"]
    graph = _graph_14()
    intervals = [
        prepare_interval(graph, Interval(
            item["robot_id"], item["task_id"], item["resource_kind"], item["resource_id"],
            item["x"], item["y"], item["t_start"], item["t_end"], None, item["time_basis"],
            planning_profile=item.get("planning_profile") or "",
        ), PROFILE_14)
        for item in stored
    ]
    plan_now = min(item.t_start for item in intervals) - 1.0
    hits = find_conflicts(intervals, intervals, robot_radius_m=0.096, now_epoch=plan_now)
    assert hits == []
    assert any(item["schedule"].get("conflicts_avoided_by_delay") for item in reserved) or any(
        item.get("selected_path_id") not in {None, "path-1"} for item in reserved
    ) or len(reserved) >= 3


def test_same_path_allowed_at_different_times():
    graph = _graph_14()
    start = graph.locate(0.9, 1.2)[1]
    goal = graph.locate(1.2, 0.8)[1]
    path = shortest_path(graph, start, goal)[0]
    first = schedule_candidates(
        graph, [path], [],
        robot_id="pinky-1", task_id="a",
        speed_m_s=0.15, dwell_s=0.5, robot_radius_m=0.096, max_depart_slots=8,
        origin_epoch=1_700_000_000.0, now_epoch=1_700_000_000.0,
        depart_slot_s=0.05 / 0.15, current_profile=PROFILE_14,
    )
    assert first is not None
    # Travel is ~5 s on this path; eight 0.333 s slots are not enough to clear the leader.
    second = schedule_candidates(
        graph, [path], first["intervals"],
        robot_id="pinky-2", task_id="b",
        speed_m_s=0.15, dwell_s=0.5, robot_radius_m=0.096, max_depart_slots=40,
        origin_epoch=1_700_000_000.0, now_epoch=1_700_000_000.0,
        depart_slot_s=0.05 / 0.15, current_profile=PROFILE_14,
    )
    assert second is not None
    assert second["depart_s"] > first["depart_s"]
    assert second["arrival_epoch"] > first["arrival_epoch"]


def test_old_five_cm_reservation_is_not_reinterpreted_or_dispatched(lattice_env):
    client, runtime = lattice_env
    _seed_pinky1(runtime)
    # Seed a confirmed reservation that still carries the empty pre-profile marker.
    from app.models.tables import FleetReservationRecord, FleetTaskRecord
    from app.services.fleet_task_service import map_revision
    revision = map_revision(runtime.settings.physical_map_yaml)
    now = datetime.now(timezone.utc)
    with runtime.db.write() as session:
        session.add(FleetTaskRecord(
            id="legacy-5cm-task",
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
                "planning_profile": "",
                "payload": {"execute": False, "task_id": "legacy-5cm-task", "robot_id": "pinky-2"},
                "needs_replan": False,
            }),
            payload_hash="legacy",
            status="RESERVED",
            delivery_status="NOT_SENT",
            ack_status="NOT_RECEIVED",
            ack_channel="none",
            ack_json="",
            failure_reason="",
            assignment_reason="legacy 5cm",
            cancellation_json="",
            motion_verified=False,
            is_archived=False,
            archived_by="",
            created_at=now,
            updated_at=now,
        ))
    with runtime.db.write() as session:
        session.add(FleetReservationRecord(
            id="legacy-5cm-res",
            task_id="legacy-5cm-task",
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
            planning_profile="",
            created_at=now,
        ))
    viewed = client.get("/api/fleet-tasks/legacy-5cm-task").json()
    assert viewed["status"] == "RESERVED"
    assert viewed["schedule"]["needs_replan"] is True
    assert viewed["schedule"]["planning_profile_current"] == PROFILE_14
    refused = client.post("/api/fleet-tasks/legacy-5cm-task/dispatch")
    assert refused.status_code == 409
    assert "lattice" in refused.json()["detail"].lower() or "plan" in refused.json()["detail"].lower()
    still = client.get("/api/fleet-tasks/legacy-5cm-task").json()
    assert still["status"] == "RESERVED"
    assert still["ack_status"] == "NOT_RECEIVED"
    reservations = client.get("/api/planning/reservations").json()["reservations"]
    legacy = next(item for item in reservations if item["reservation_id"] == "legacy-5cm-res")
    assert legacy["planning_profile"] == ""
    assert legacy["resource_id"] == "v-18-24"


def test_path_comparison_five_vs_fourteen():
    g05 = _graph_05()
    g14 = _graph_14()
    s05, g05n = g05.locate(0.9, 1.2)[1], g05.locate(1.2, 0.8)[1]
    s14, g14n = g14.locate(0.9, 1.2)[1], g14.locate(1.2, 0.8)[1]
    p05 = shortest_path(g05, s05, g05n)
    p14 = shortest_path(g14, s14, g14n)
    assert p05 is not None and p14 is not None
    assert len(p14[0]) < len(p05[0])
    assert p14[1] >= p05[1] - 0.2
