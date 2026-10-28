import math
from pathlib import Path

from app.core.config import ROOT
from app.services.map_frame import grid_to_pixel, grid_to_world, world_to_grid
from app.services.occupancy_grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid, classify_pixel
from app.services.route_planner import k_shortest_paths, shortest_path
from app.services.route_preview import inspect_point, preview_route, validate_robot_starts
from app.services.fleet_registry import load_registry
from app.services.waypoint_graph import WaypointGraph


def _graph(rows: list[str], *, diagonal: bool = True, radius: float = 0.0, resolution: float = 1.0) -> WaypointGraph:
    grid = OccupancyGrid.from_rows(rows, resolution=resolution)
    return WaypointGraph(
        grid,
        planning_resolution_m=resolution,
        robot_radius_m=radius,
        allow_diagonal=diagonal,
    )


def _ids(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    return path


def test_site_map_loads_free_and_occupied_cells():
    grid = OccupancyGrid.load(ROOT / "config/maps/physical_site/1f_map_f.yaml")
    assert grid.geometry.yaml_path.name == "1f_map_f.yaml"
    assert grid.geometry.image_path.name == "1f_map_f.pgm"
    assert (grid.width, grid.height, grid.resolution) == (186, 236, 0.01)
    counts = {FREE: 0, OCCUPIED: 0, UNKNOWN: 0}
    for row in range(grid.height):
        for col in range(grid.width):
            counts[grid.state(col, row)] += 1
    assert counts[FREE] > 0
    assert counts[OCCUPIED] > 0
    assert classify_pixel(0, negate=False, free_thresh=0.196, occupied_thresh=0.65) == OCCUPIED
    assert classify_pixel(255, negate=False, free_thresh=0.196, occupied_thresh=0.65) == FREE
    assert classify_pixel(128, negate=False, free_thresh=0.196, occupied_thresh=0.65) == UNKNOWN


def test_map_grid_pixel_round_trip_uses_map_frame():
    grid = OccupancyGrid.load(ROOT / "config/maps/physical_site/1f_map_f.yaml")
    geometry = grid.geometry
    col, row = world_to_grid(geometry, 1.696, 0.103)
    x, y = grid_to_world(geometry, col, row)
    pixel_x, pixel_y = grid_to_pixel(geometry, col, row)
    assert math.isclose(x, 1.696, abs_tol=1e-9)
    assert math.isclose(y, 0.103, abs_tol=1e-9)
    assert grid.cell_at(1.696, 0.103) == (math.floor(col), math.floor(row))
    assert math.isclose(pixel_y, geometry.height_px - row, abs_tol=1e-9)
    assert math.isclose(pixel_x, col, abs_tol=1e-9)


def test_points_outside_the_map_are_rejected():
    graph = _graph(["..."] * 3)
    assert graph.locate(-0.1, 0.5)[0] == "outside"
    assert graph.locate(3.0, 0.5)[0] == "outside"
    assert graph.grid.cell_at(0.5, 3.0) is None


def test_occupied_cells_block_entry():
    graph = _graph([".#.", "..."])
    assert graph.grid.state_at(1.5, 1.5) == OCCUPIED
    assert graph.locate(1.5, 1.5)[0] == "occupied"
    assert graph.locate(0.5, 0.5)[0] == "ok"


def test_unknown_cells_are_not_traversable():
    graph = _graph([".?.", "..."])
    assert graph.grid.state_at(1.5, 1.5) == UNKNOWN
    assert graph.locate(1.5, 1.5)[0] == "unknown"
    reason, _node = graph.locate(1.5, 0.5)
    assert reason == "ok"


def test_shortest_path_goes_around_an_obstacle():
    graph = _graph([
        ".....",
        ".###.",
        ".....",
    ], diagonal=False)
    found = shortest_path(graph, (0, 0), (4, 0))
    assert found is not None
    nodes, cost = found
    assert nodes[0] == (0, 0) and nodes[-1] == (4, 0)
    assert all(node[1] == 0 for node in nodes)
    assert math.isclose(cost, 4.0)
    assert all(graph.grid.state(*node) == FREE for node in nodes)


def test_diagonal_move_does_not_cut_an_obstacle_corner():
    graph = _graph([
        "...",
        ".#.",
        "...",
    ])
    found = shortest_path(graph, (0, 0), (2, 2))
    assert found is not None
    nodes, cost = found
    edges = list(zip(nodes, nodes[1:]))
    assert ((1, 0), (2, 1)) not in edges
    assert ((0, 1), (1, 2)) not in edges
    assert math.isclose(cost, 4.0)
    assert not any(abs(a[0] - b[0]) == 1 and abs(a[1] - b[1]) == 1 and (1, 1) in (a, b) for a, b in edges)


def test_unreachable_goal_returns_no_path():
    graph = _graph([
        "#.",
        "##",
        "..",
    ])
    assert shortest_path(graph, (0, 0), (1, 2)) is None
    try:
        preview_route(
            graph,
            robot_id="pinky-2",
            start_x=0.5,
            start_y=0.5,
            start_source="request",
            goal_x=1.5,
            goal_y=2.5,
            max_candidates=3,
        )
    except Exception as exc:
        assert exc.code == "unreachable"
        assert exc.payload["paths"] == []
    else:
        raise AssertionError("unreachable goal was reported as success")


def test_k_shortest_paths_are_distinct_and_ranked():
    graph = _graph(["...."] * 4, diagonal=False)
    found = k_shortest_paths(graph, (0, 0), (3, 3), 3)
    assert len(found) == 3
    identities = [tuple(nodes) for nodes, _cost in found]
    assert len(set(identities)) == 3
    costs = [cost for _nodes, cost in found]
    assert costs == sorted(costs)
    assert math.isclose(costs[0], 6.0)


def test_fewer_routes_than_requested_are_returned_without_duplicates():
    graph = _graph([
        "...",
        "###",
        "...",
    ], diagonal=False)
    found = k_shortest_paths(graph, (0, 0), (2, 0), 3)
    assert len(found) == 1
    assert len({tuple(nodes) for nodes, _cost in found}) == 1


def test_registered_robot_starts_are_checked_against_the_real_map():
    grid = OccupancyGrid.load(ROOT / "config/maps/physical_site/1f_map_f.yaml")
    graph = WaypointGraph(grid, planning_resolution_m=0.05, robot_radius_m=0.06, allow_diagonal=True)
    registry = load_registry(ROOT / "config" / "physical_fleet.json")
    reports = {item["robot_id"]: item for item in validate_robot_starts(graph, registry, None)}
    assert reports["pinky-1"]["inspection"]["reason"] == "tf_unavailable"
    for robot_id in ("pinky-2", "pinky-3"):
        inspection = reports[robot_id]["inspection"]
        assert inspection["occupancy"] == FREE
        assert inspection["traversable"] is True
        assert inspection["reason"] == "ok"
    for x, y in ((0.4, 1.9), (0.4, 0.5)):
        inspection = inspect_point(graph, x, y)
        assert inspection["occupancy"] == FREE
        assert inspection["traversable"] is False
        assert inspection["reason"] == "clearance"
        assert inspection["suggestions"]
        assert all(item["replaces_registered_pose"] is False for item in inspection["suggestions"])
        for suggestion in inspection["suggestions"]:
            suggested = inspect_point(graph, suggestion["x"], suggestion["y"])
            assert suggested["traversable"] is True
            assert suggested["occupancy"] == FREE
    injected = inspect_point(graph, 1.696, 0.103)
    assert injected["occupancy"] == FREE
    assert injected["traversable"] is True


def test_preview_api_and_existing_physical_endpoints(env):
    client, _runtime = env
    site = client.get("/api/physical/map")
    assert site.status_code == 200
    assert site.json()["source"] == "1f_map_f.yaml"
    robots = client.get("/api/physical/robots")
    assert robots.status_code == 200
    assert [item["kind"] for item in robots.json()["robots"]] == ["physical", "virtual", "virtual"]
    converted = client.post("/api/physical/map/coordinates", json={"grid": {"col": 40, "row": 190}})
    assert converted.status_code == 200

    starts = client.get("/api/planning/starts")
    assert starts.status_code == 200
    body = starts.json()
    assert body["map"]["collision_safety_verified"] is False
    assert [item["robot_id"] for item in body["robots"]] == ["pinky-1", "pinky-2", "pinky-3"]

    missing = client.post("/api/planning/routes/preview", json={
        "robot_id": "pinky-9",
        "goal": {"x": 1.0, "y": 1.0},
    })
    assert missing.status_code == 404

    no_tf = client.post("/api/planning/routes/preview", json={
        "robot_id": "pinky-1",
        "goal": {"x": 1.0, "y": 1.0},
    })
    assert no_tf.status_code == 422
    assert no_tf.json()["code"] == "tf_unavailable"
    assert no_tf.json()["paths"] == []

    blocked_home = client.post("/api/planning/routes/preview", json={
        "robot_id": "pinky-2",
        "start": {"x": 0.4, "y": 1.9},
        "goal": {"x": 1.0, "y": 1.0},
    })
    assert blocked_home.status_code == 422
    assert blocked_home.json()["code"] == "start_clearance"
    assert blocked_home.json()["reachable"] is False
    assert blocked_home.json()["paths"] == []
    assert blocked_home.json()["start"]["suggestions"]

    occupied = client.post("/api/planning/routes/preview", json={
        "robot_id": "pinky-2",
        "start": {"x": 0.9, "y": 1.2},
        "goal": {"x": 0.0, "y": 0.0},
    })
    assert occupied.status_code == 422
    assert occupied.json()["code"].startswith("goal_")
    assert occupied.json()["paths"] == []

    planned = client.post("/api/planning/routes/preview", json={
        "robot_id": "pinky-2",
        "start": {"x": 0.9, "y": 1.2},
        "goal": {"x": 1.2, "y": 0.8},
        "max_candidates": 3,
    })
    assert planned.status_code == 200, planned.text
    payload = planned.json()
    assert payload["reachable"] is True
    assert payload["map"]["source"] == "1f_map_f.yaml"
    assert payload["map"]["collision_safety_verified"] is False
    assert payload["start"]["source"] == "request"
    assert 1 <= len(payload["paths"]) <= 3
    assert payload["shortest"]["path_id"] == payload["paths"][0]["path_id"]
    assert payload["shortest"]["rank"] == 1
    assert payload["shortest"]["success"] is True
    assert payload["shortest"]["length_m"] > 0
    identities = [tuple(point["waypoint_id"] for point in path["waypoints"]) for path in payload["paths"]]
    assert len(identities) == len(set(identities))
    costs = [path["cost"] for path in payload["paths"]]
    assert costs == sorted(costs)
    planner_source = Path(ROOT / "backend/app/services/route_planner.py").read_text(encoding="utf-8")
    preview_source = Path(ROOT / "backend/app/services/route_preview.py").read_text(encoding="utf-8")
    for source in (planner_source, preview_source):
        assert "navigate_to_pose" not in source
        assert "cmd_vel" not in source


def test_static_isolation_failure_is_a_dangling_symlink():
    link = ROOT / "simulation/ros_ws/install/aws_robomaker_small_house_world/share/aws_robomaker_small_house_world/launch/view_small_house.launch.py"
    assert link.is_symlink()
    assert not link.exists()
