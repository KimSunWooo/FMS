"""Read-only route preview for registered robots.

Resolves a start pose, checks the occupancy grid, and returns candidate paths.
It does not create a task, assign a robot, or send a Nav2 goal.
"""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from app.services.fleet_registry import RegistryError, load_registry
from app.services.map_frame import MapFileError
from app.services.occupancy_grid import OccupancyGrid
from app.services.route_planner import PlanningFailure, k_shortest_paths, path_record
from app.services.reservation_manager import planning_profile
from app.services.waypoint_graph import WaypointGraph

TF_MAX_AGE_SECONDS = 60.0


def build_graph(
    yaml_path: Path,
    *,
    planning_resolution_m: float,
    robot_radius_m: float,
    allow_diagonal: bool,
) -> WaypointGraph:
    grid = OccupancyGrid.load(yaml_path)
    return WaypointGraph(
        grid,
        planning_resolution_m=planning_resolution_m,
        robot_radius_m=robot_radius_m,
        allow_diagonal=allow_diagonal,
    )


def map_summary(graph: WaypointGraph) -> dict:
    from app.services.robot_geometry import planning_geometry_public

    geometry = graph.grid.geometry
    body = planning_geometry_public()
    return {
        "frame_id": "map",
        "source": geometry.yaml_path.name,
        "image": geometry.image_path.name,
        "width_px": geometry.width_px,
        "height_px": geometry.height_px,
        "resolution_m": geometry.resolution,
        "origin": {"x": geometry.origin_x, "y": geometry.origin_y, "yaw": geometry.origin_yaw},
        "planning_resolution_m": graph.planning_resolution_m,
        "planning_resolution_requested_m": getattr(
            graph, "requested_planning_resolution_m", graph.planning_resolution_m
        ),
        "planning_resolution_applied_equals_requested": getattr(
            graph, "resolution_applied_equals_requested", True
        ),
        "planning_profile": planning_profile(graph.planning_resolution_m, graph.robot_radius_m),
        "occupancy_resolution_m": graph.grid.resolution,
        "robot_radius_m": graph.robot_radius_m,
        "robot_radius_source": body["source"],
        "robot_geometry": body,
        "collision_safety_verified": False,
        "unknown_policy": "blocked",
        "allow_diagonal": graph.allow_diagonal,
        "corner_cutting": False,
    }


def inspect_point(graph: WaypointGraph, x: float, y: float) -> dict:
    index = graph.grid.cell_at(x, y)
    reason, node = graph.locate(x, y)
    distance = graph.grid.nearest_occupied_distance(x, y)
    connection = None
    if reason == "ok" and node is not None:
        sx, sy = graph.world(node)
        connection = {
            "x": round(sx, 6),
            "y": round(sy, 6),
            "waypoint_id": graph.node_id(node),
            "is_robot_pose": False,
            "segment_checked": True,
        }
    return {
        "x": x,
        "y": y,
        "inside_map": index is not None,
        "occupancy": None if index is None else graph.grid.state(*index),
        "traversable": reason == "ok",
        "reason": reason,
        "nearest_occupied_m": None if distance is None else round(distance, 6),
        "lattice_connection": connection,
        "suggestions": [] if reason == "ok" else graph.suggestions(x, y),
    }


def fresh_tf_pose(snapshot: dict | None, *, now: datetime | None = None) -> dict | None:
    """Return a recent TF pose from the Nav2 monitor, or None. Never invents one.

    SSH is diagnostic only. A fresh map→base_footprint sample is accepted without
    an SSH session.
    """
    if not snapshot:
        return None
    pose = snapshot.get("pose")
    observed_at = snapshot.get("pose_checked_at")
    if not isinstance(pose, dict) or not observed_at:
        return None
    try:
        age = ((now or datetime.now(timezone.utc)) - datetime.fromisoformat(observed_at)).total_seconds()
        coordinates = (float(pose["x"]), float(pose["y"]))
    except (TypeError, ValueError, KeyError):
        return None
    if not 0 <= age <= TF_MAX_AGE_SECONDS:
        return None
    source = str(snapshot.get("pose_source") or "tf:map->base_footprint")
    return {"x": coordinates[0], "y": coordinates[1], "source": source, "observed_at": observed_at}


def _physical_start_pose(
    record: dict,
    snapshot: dict | None,
    live_state: dict | None,
) -> dict | None:
    """Prefer Gateway/amcl robot_state; fall back to monitor TF. Never invent."""
    if (
        isinstance(live_state, dict)
        and live_state.get("robot_id") == record["robot_id"]
        and live_state.get("pose_valid")
        and isinstance(live_state.get("pose"), dict)
        and live_state.get("pose_source") != "registry_home"
    ):
        sample = live_state["pose"]
        try:
            return {
                "x": float(sample["x"]),
                "y": float(sample["y"]),
                "source": str(live_state.get("pose_source") or "topic:amcl_pose"),
                "observed_at": live_state.get("pose_timestamp"),
                "pose_freshness": live_state.get("pose_freshness"),
            }
        except (TypeError, ValueError, KeyError):
            pass
    if snapshot is not None and snapshot.get("robot_id") not in (None, record["robot_id"]):
        return None
    return fresh_tf_pose(snapshot)


def validate_robot_starts(
    graph: WaypointGraph,
    registry: list[dict],
    snapshot: dict | None,
    states: dict[str, dict] | None = None,
) -> list[dict]:
    reports = []
    for record in registry:
        if record["kind"] == "virtual":
            home = record["home"]
            inspection = inspect_point(graph, home["x"], home["y"])
            reports.append({
                "robot_id": record["robot_id"],
                "kind": "virtual",
                "pose_source": "registry_home",
                "registered_pose": {"x": home["x"], "y": home["y"], "yaw": home["yaw"]},
                "inspection": inspection,
            })
            continue
        live = None if states is None else states.get(record["robot_id"])
        pose = _physical_start_pose(record, snapshot, live)
        if pose is None:
            reports.append({
                "robot_id": record["robot_id"],
                "kind": "physical",
                "pose_source": None,
                "registered_pose": None,
                "inspection": {
                    "inside_map": None,
                    "occupancy": None,
                    "traversable": None,
                    "reason": "tf_unavailable",
                    "suggestions": [],
                },
            })
            continue
        inspection = inspect_point(graph, pose["x"], pose["y"])
        inspection["observed_at"] = pose.get("observed_at")
        inspection["pose_freshness"] = pose.get("pose_freshness")
        reports.append({
            "robot_id": record["robot_id"],
            "kind": "physical",
            "pose_source": pose["source"],
            "registered_pose": None,
            "inspection": inspection,
        })
    return reports


def preview_route(
    graph: WaypointGraph,
    *,
    robot_id: str,
    goal_x: float,
    goal_y: float,
    start_x: float,
    start_y: float,
    start_source: str,
    max_candidates: int,
) -> dict:
    summary = map_summary(graph)
    start_inspection = inspect_point(graph, start_x, start_y)
    goal_inspection = inspect_point(graph, goal_x, goal_y)
    base = {
        "robot_id": robot_id,
        "start": {**start_inspection, "source": start_source},
        "goal": goal_inspection,
        "map": summary,
        "paths": [],
        "shortest": None,
        "reachable": False,
        "collision_safety_verified": False,
    }
    if start_inspection["reason"] != "ok":
        raise PlanningFailure(
            f"start_{start_inspection['reason']}",
            f"Start is not a traversable map cell ({start_inspection['reason']})",
            base,
        )
    if goal_inspection["reason"] != "ok":
        raise PlanningFailure(
            f"goal_{goal_inspection['reason']}",
            f"Goal is not a traversable map cell ({goal_inspection['reason']})",
            base,
        )
    _reason, start_node = graph.locate(start_x, start_y)
    _reason, goal_node = graph.locate(goal_x, goal_y)
    assert start_node is not None and goal_node is not None
    found = k_shortest_paths(graph, start_node, goal_node, max_candidates)
    if not found:
        raise PlanningFailure("unreachable", "No traversable path exists between the start and the goal", base)
    paths = [path_record(graph, nodes, cost, rank) for rank, (nodes, cost) in enumerate(found, start=1)]
    identities = [tuple(item["waypoint_id"] for item in path["waypoints"]) for path in paths]
    if len(identities) != len(set(identities)):
        raise PlanningFailure("duplicate_path", "Path search returned a duplicate candidate", base)
    base["paths"] = paths
    base["shortest"] = paths[0]
    base["reachable"] = True
    base["candidate_count"] = len(paths)
    return base


def resolve_start(
    record: dict,
    requested: dict | None,
    snapshot: dict | None,
    live_state: dict | None = None,
) -> tuple[float, float, str]:
    if requested is not None:
        return float(requested["x"]), float(requested["y"]), "request"
    if record["kind"] == "virtual":
        home = record["home"]
        return float(home["x"]), float(home["y"]), "registry_home"
    pose = _physical_start_pose(record, snapshot, live_state)
    if pose is None:
        raise PlanningFailure(
            "tf_unavailable",
            "Physical robot has no fresh map pose (amcl_pose / TF). "
            "Pass an explicit start or wait for a live observation. SSH is not required.",
            {"robot_id": record["robot_id"], "paths": [], "shortest": None, "reachable": False},
        )
    return pose["x"], pose["y"], pose["source"]


def load_registered(path: Path) -> list[dict]:
    try:
        return load_registry(path)
    except RegistryError as exc:
        raise PlanningFailure("registry", str(exc), {"paths": [], "shortest": None, "reachable": False}) from exc


def load_graph_or_fail(yaml_path: Path, **kwargs) -> WaypointGraph:
    try:
        return build_graph(yaml_path, **kwargs)
    except MapFileError as exc:
        raise PlanningFailure("map", str(exc), {"paths": [], "shortest": None, "reachable": False}) from exc
