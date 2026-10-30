"""Planned spatiotemporal reservations on top of an existing waypoint graph.

Times are planning estimates from a configured speed. They are not measured
motion and never mark a cell as actually traversed.

New intervals use absolute UTC epoch seconds. ``depart_s`` remains a delay
from the planning origin. Rows whose ``t_start`` is below ``EPOCH_FLOOR``
cannot be placed on that clock and stay ``legacy``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from datetime import datetime, timezone

from app.services.waypoint_graph import WaypointGraph

Node = tuple[int, int]
EPOCH_FLOOR = 1_000_000_000.0


@dataclass(frozen=True)
class Interval:
    robot_id: str
    task_id: str
    kind: str
    resource_id: str
    x: float
    y: float
    t_start: float
    t_end: float
    node: Node | None = None
    time_basis: str = "absolute"
    planning_profile: str = ""
    ax: float | None = None
    ay: float | None = None
    bx: float | None = None
    by: float | None = None


def classify_time_basis(t_start: float, stored: str | None = None) -> str:
    """Keep an unrestorable relative timestamp out of the absolute clock."""
    if stored == "legacy" or float(t_start) < EPOCH_FLOOR:
        return "legacy"
    return "absolute"


def footprint_corner_m(radius_m: float) -> float:
    """Planning disk radius used for edge/vertex separation.

    ``planning_robot_radius_m`` is already R_effective (Nav2 square circumradius
    + 0.01 m clearance). Do not multiply by sqrt(2) again — that would double
    the diagonal inflate. Inter-robot center distance uses 2 * R_effective.
    """
    return max(0.0, float(radius_m))


def overlaps(a0: float, a1: float, b0: float, b1: float) -> bool:
    return a0 < b1 and b0 < a1


def crossing_keys(a: Node, b: Node) -> set[str]:
    if abs(a[0] - b[0]) != 1 or abs(a[1] - b[1]) != 1:
        return set()
    i, j = min(a[0], b[0]), min(a[1], b[1])
    pair = {a, b}
    keys = set()
    if pair == {(i, j), (i + 1, j + 1)}:
        keys.add(f"cross-{i}-{j}")
    if pair == {(i + 1, j), (i, j + 1)}:
        keys.add(f"cross-{i}-{j}")
    return keys


def time_path(
    graph: WaypointGraph,
    nodes: list[Node],
    *,
    depart_s: float,
    speed_m_s: float,
    dwell_s: float,
    robot_id: str,
    task_id: str,
) -> list[Interval]:
    if speed_m_s <= 0:
        raise ValueError("speed_m_s must be positive")
    if not nodes:
        return []
    times = [depart_s]
    distance = 0.0
    for left, right in zip(nodes, nodes[1:]):
        lx, ly = graph.world(left)
        rx, ry = graph.world(right)
        distance += math.hypot(rx - lx, ry - ly)
        times.append(depart_s + distance / speed_m_s)
    intervals: list[Interval] = []
    for index, node in enumerate(nodes):
        x, y = graph.world(node)
        t_start = times[index]
        t_end = times[index + 1] if index + 1 < len(times) else t_start + dwell_s
        if t_end <= t_start:
            t_end = t_start + dwell_s
        intervals.append(Interval(
            robot_id, task_id, "vertex", f"v-{node[0]}-{node[1]}", x, y, t_start, t_end, node,
            ax=x, ay=y, bx=x, by=y,
        ))
        if index + 1 < len(nodes):
            nxt = nodes[index + 1]
            nx, ny = graph.world(nxt)
            intervals.append(Interval(
                robot_id, task_id, "edge",
                f"e-{node[0]},{node[1]}>{nxt[0]},{nxt[1]}",
                (x + nx) / 2,
                (y + ny) / 2,
                times[index], times[index + 1], None,
                ax=x, ay=y, bx=nx, by=ny,
            ))
    return intervals


def shift_intervals(intervals: list[Interval], origin_epoch: float) -> list[Interval]:
    """Move relative planning times onto the shared UTC epoch clock."""
    shifted = []
    for item in intervals:
        shifted.append(replace(
            item,
            t_start=item.t_start + origin_epoch,
            t_end=item.t_end + origin_epoch,
            time_basis="absolute",
        ))
    return shifted


def hydrate_interval(graph: WaypointGraph, interval: Interval) -> Interval:
    """Fill edge endpoints from the graph so stored rows can be rechecked."""
    if interval.kind != "edge" or interval.ax is not None:
        if interval.kind == "vertex" and interval.ax is None:
            return replace(interval, ax=interval.x, ay=interval.y, bx=interval.x, by=interval.y)
        return interval
    nodes = _edge_nodes(interval.resource_id)
    if nodes is None:
        return interval
    try:
        ax, ay = graph.world(nodes[0])
        bx, by = graph.world(nodes[1])
    except (TypeError, ValueError, IndexError, KeyError):
        return interval
    return replace(interval, ax=ax, ay=ay, bx=bx, by=by)


def find_conflicts(
    proposed: list[Interval],
    existing: list[Interval],
    *,
    robot_radius_m: float,
    now_epoch: float | None = None,
) -> list[dict]:
    """Plan-time conflicts. A past-due absolute reservation still occupies space.

    ``now_epoch`` does not mean the robot finished moving. A legacy interval
    has no UTC meaning, so it conflicts on geometry with every other task.
    """
    found: list[dict] = []
    for left in proposed:
        for right in existing:
            if left.task_id == right.task_id:
                continue
            legacy = left.time_basis == "legacy" or right.time_basis == "legacy"
            stale = (
                now_epoch is not None
                and right.time_basis == "absolute"
                and right.t_end < now_epoch
            )
            timed = overlaps(left.t_start, left.t_end, right.t_start, right.t_end)
            if not legacy and not stale and not timed:
                continue
            kinds = _spatial_kinds(left, right, robot_radius_m)
            if not kinds:
                continue
            if legacy:
                found.append(_hit("legacy_time_basis", left, right))
            if stale:
                found.append(_hit("delayed_motion", left, right))
            if timed or legacy or stale:
                for kind in kinds:
                    found.append(_hit(kind, left, right))
    unique: list[dict] = []
    seen: set[tuple] = set()
    for item in found:
        key = (item["conflict_type"], item["resource_id"], item["other_task_id"], round(item["t_start"], 6))
        if key in seen:
            continue
        seen.add(key)
        unique.append(item)
    return unique


def schedule_candidates(
    graph: WaypointGraph,
    candidates: list[list[Node]],
    existing: list[Interval],
    *,
    robot_id: str,
    task_id: str,
    speed_m_s: float,
    dwell_s: float,
    robot_radius_m: float,
    max_depart_slots: int,
    origin_epoch: float | None = None,
    now_epoch: float | None = None,
    depart_slot_s: float | None = None,
    current_profile: str = "",
) -> dict | None:
    """Pick the conflict-free candidate with the earliest estimated arrival.

    The first feasible departure of each path is enough: a later slot only
    increases arrival when travel time is fixed. The search stops at
    ``max_depart_slots * len(candidates)``.
    """
    origin = datetime.now(timezone.utc).timestamp() if origin_epoch is None else float(origin_epoch)
    # A future queue origin is not "now". Past-due occupancy uses the wall clock.
    wall = origin if now_epoch is None else float(now_epoch)
    # Spatial lattice spacing and departure slots are separate. Callers that
    # omit the slot keep the old resolution/speed step for existing tests.
    if depart_slot_s is None:
        slot = graph.planning_resolution_m / speed_m_s
    else:
        slot = float(depart_slot_s)
        if slot <= 0:
            raise ValueError("depart_slot_s must be positive")
    prepared = [prepare_interval(graph, item, current_profile) for item in existing]
    attempts = 0
    limit = max(1, max_depart_slots) * max(1, len(candidates))
    best: dict | None = None
    options: list[dict] = []
    for nodes in candidates:
        for slot_index in range(max(1, max_depart_slots)):
            attempts += 1
            if attempts > limit:
                break
            depart = slot_index * slot
            relative = time_path(
                graph, nodes, depart_s=depart, speed_m_s=speed_m_s, dwell_s=dwell_s,
                robot_id=robot_id, task_id=task_id,
            )
            proposed = shift_intervals(relative, origin)
            if current_profile:
                proposed = [replace(item, planning_profile=current_profile) for item in proposed]
            hits = find_conflicts(
                proposed, prepared, robot_radius_m=robot_radius_m, now_epoch=wall,
            )
            if hits:
                continue
            arrival = max((item.t_end for item in proposed), default=origin + depart)
            option = {
                "nodes": nodes,
                "depart_s": depart,
                "arrival_epoch": arrival,
                "travel_s": max(0.0, arrival - (origin + depart)),
            }
            options.append(option)
            if best is None or arrival < best["arrival_epoch"] - 1e-9:
                best = {
                    "nodes": nodes,
                    "depart_s": depart,
                    "intervals": proposed,
                    "conflicts_avoided_by_delay": slot_index > 0,
                    "attempts": attempts,
                    "origin_epoch": origin,
                    "arrival_epoch": arrival,
                }
            break
        if attempts > limit:
            break
    if best is None:
        return None
    best["options"] = options
    best["attempts"] = attempts
    best["planning_profile"] = current_profile
    best["depart_slot_s"] = slot
    return best


def planning_profile(resolution_m: float, radius_m: float) -> str:
    """Identity of the lattice used to name vertices and edges. Not a map hash."""
    return f"lattice-{float(resolution_m):.3f}-r{float(radius_m):.3f}"


def profiles_share_lattice(left: Interval, right: Interval) -> bool:
    """Empty profiles are the pre-profile tests. A blank stored row is not the new lattice."""
    if not left.planning_profile and not right.planning_profile:
        return True
    return left.planning_profile == right.planning_profile


def prepare_interval(graph: WaypointGraph, item: Interval, current_profile: str) -> Interval:
    """Do not project an older vertex id through the current lattice."""
    if item.planning_profile != current_profile:
        return item
    return hydrate_interval(graph, item)


def _spatial_kinds(left: Interval, right: Interval, robot_radius_m: float) -> list[str]:
    kinds: list[str] = []
    if not profiles_share_lattice(left, right):
        if robot_radius_m > 0 and math.hypot(left.x - right.x, left.y - right.y) < 2 * robot_radius_m:
            kinds.append("profile_mismatch")
        return kinds
    corner = footprint_corner_m(robot_radius_m)
    if left.kind == "vertex" and right.kind == "vertex" and left.resource_id == right.resource_id:
        kinds.append("vertex")
        if (right.t_end - right.t_start) > (left.t_end - left.t_start) + 1e-9:
            kinds.append("waiting")
    if left.kind == "edge" and right.kind == "edge" and _opposite_edge(left.resource_id, right.resource_id):
        kinds.append("edge")
    if left.kind == "edge" and right.kind == "edge" and _crosses(left.resource_id, right.resource_id):
        kinds.append("crossing")
    if (
        robot_radius_m > 0
        and left.kind == "vertex"
        and right.kind == "vertex"
        and left.resource_id != right.resource_id
        and math.hypot(left.x - right.x, left.y - right.y) < 2 * robot_radius_m
    ):
        kinds.append("footprint")
    if robot_radius_m > 0 and _edge_vertex(left, right, 2 * corner):
        kinds.append("edge_vertex")
    if robot_radius_m > 0 and left.kind == "edge" and right.kind == "edge":
        gap = _edge_gap(left, right)
        same_direction = _same_direction(left.resource_id, right.resource_id)
        if left.resource_id == right.resource_id or (same_direction and gap is not None and gap < 2 * corner):
            kinds.append("following")
        elif gap is not None and gap < 2 * corner and left.resource_id != right.resource_id:
            kinds.append("edge_proximity")
    return kinds


def _edge_vertex(left: Interval, right: Interval, limit: float) -> bool:
    edge, point = (left, right) if left.kind == "edge" and right.kind == "vertex" else (
        (right, left) if right.kind == "edge" and left.kind == "vertex" else (None, None)
    )
    if edge is None or point is None or edge.ax is None or edge.bx is None:
        return False
    return _point_segment((point.x, point.y), (edge.ax, edge.ay or 0.0), (edge.bx, edge.by or 0.0)) < limit


def _edge_gap(left: Interval, right: Interval) -> float | None:
    if None in (left.ax, left.ay, left.bx, left.by, right.ax, right.ay, right.bx, right.by):
        return None
    return _segment_distance(
        (left.ax, left.ay), (left.bx, left.by), (right.ax, right.ay), (right.bx, right.by),
    )


def _same_direction(left: str, right: str) -> bool:
    a = _edge_vector(left)
    b = _edge_vector(right)
    if a is None or b is None:
        return False
    return a[0] * b[0] + a[1] * b[1] > 0


def _edge_vector(resource_id: str) -> tuple[float, float] | None:
    nodes = _edge_nodes(resource_id)
    if nodes is None:
        return None
    return (nodes[1][0] - nodes[0][0], nodes[1][1] - nodes[0][1])


def _point_segment(point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    scale = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq))
    return math.hypot(point[0] - (start[0] + scale * dx), point[1] - (start[1] + scale * dy))


def _segment_distance(a, b, c, d) -> float:
    """Closest distance between two finite segments. Exact for the clamped parameters."""
    def dot(u, v):
        return u[0] * v[0] + u[1] * v[1]

    def sub(u, v):
        return (u[0] - v[0], u[1] - v[1])

    ab, cd = sub(b, a), sub(d, c)
    uu, vv, uv = dot(ab, ab), dot(cd, cd), dot(ab, cd)
    width = sub(a, c)
    along = dot(ab, width)
    across = dot(cd, width)
    denom = uu * vv - uv * uv
    if denom > 1e-12 and uu > 1e-12:
        s = max(0.0, min(1.0, (uv * across - vv * along) / denom))
    else:
        s = 0.0
    if vv > 1e-12:
        t = max(0.0, min(1.0, (uv * s + across) / vv))
    else:
        t = 0.0
    if uu > 1e-12:
        s = max(0.0, min(1.0, (uv * t - along) / uu))
    if vv > 1e-12:
        t = max(0.0, min(1.0, (uv * s + across) / vv))
    closest_ab = (a[0] + s * ab[0], a[1] + s * ab[1])
    closest_cd = (c[0] + t * cd[0], c[1] + t * cd[1])
    return math.hypot(closest_ab[0] - closest_cd[0], closest_ab[1] - closest_cd[1])


def _hit(kind: str, left: Interval, right: Interval) -> dict:
    return {
        "conflict_type": kind,
        "resource_id": left.resource_id,
        "other_robot_id": right.robot_id,
        "other_task_id": right.task_id,
        "t_start": left.t_start,
        "t_end": left.t_end,
        "occupancy_class": "planned",
        "actual_motion_verified": False,
    }


def _opposite_edge(left: str, right: str) -> bool:
    if not (left.startswith("e-") and right.startswith("e-") and ">" in left and ">" in right):
        return False
    a, b = left[2:].split(">")
    c, d = right[2:].split(">")
    return a == d and b == c


def _crosses(left: str, right: str) -> bool:
    pair_l = _edge_nodes(left)
    pair_r = _edge_nodes(right)
    if pair_l is None or pair_r is None:
        return False
    return bool(crossing_keys(*pair_l) & crossing_keys(*pair_r)) and pair_l != pair_r and tuple(reversed(pair_l)) != pair_r


def _edge_nodes(resource_id: str) -> tuple[Node, Node] | None:
    if not resource_id.startswith("e-") or ">" not in resource_id:
        return None
    left, right = resource_id[2:].split(">")
    return _parse_node(left), _parse_node(right)


def _parse_node(text: str) -> Node:
    col, row = text.split(",")
    return int(col), int(row)
