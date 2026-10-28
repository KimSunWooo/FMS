"""Planned spatiotemporal reservations on top of an existing waypoint graph.

Times are planning estimates from a configured speed. They are not measured
motion and never mark a cell as actually traversed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from app.services.waypoint_graph import WaypointGraph

Node = tuple[int, int]


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
        ))
        if index + 1 < len(nodes):
            nxt = nodes[index + 1]
            intervals.append(Interval(
                robot_id, task_id, "edge",
                f"e-{node[0]},{node[1]}>{nxt[0]},{nxt[1]}",
                (x + graph.world(nxt)[0]) / 2,
                (y + graph.world(nxt)[1]) / 2,
                times[index], times[index + 1], None,
            ))
    return intervals


def find_conflicts(
    proposed: list[Interval],
    existing: list[Interval],
    *,
    robot_radius_m: float,
) -> list[dict]:
    found: list[dict] = []
    for left in proposed:
        for right in existing:
            if left.task_id == right.task_id:
                continue
            if not overlaps(left.t_start, left.t_end, right.t_start, right.t_end):
                continue
            if left.kind == "vertex" and right.kind == "vertex" and left.resource_id == right.resource_id:
                found.append(_hit("vertex", left, right))
                if (right.t_end - right.t_start) > (left.t_end - left.t_start) + 1e-9:
                    found.append(_hit("waiting", left, right))
            if left.kind == "edge" and right.kind == "edge" and _opposite_edge(left.resource_id, right.resource_id):
                found.append(_hit("edge", left, right))
            if left.kind == "edge" and right.kind == "edge" and _crosses(left.resource_id, right.resource_id):
                found.append(_hit("crossing", left, right))
            if (
                robot_radius_m > 0
                and left.kind == "vertex"
                and right.kind == "vertex"
                and left.resource_id != right.resource_id
                and math.hypot(left.x - right.x, left.y - right.y) < 2 * robot_radius_m
            ):
                found.append(_hit("footprint", left, right))
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
) -> dict | None:
    """Try candidate geometry, then a later departure. Stops after the slot cap."""
    slot = graph.planning_resolution_m / speed_m_s
    attempts = 0
    limit = max(1, max_depart_slots) * max(1, len(candidates))
    for nodes in candidates:
        for slot_index in range(max(1, max_depart_slots)):
            attempts += 1
            if attempts > limit:
                return None
            depart = slot_index * slot
            proposed = time_path(
                graph, nodes, depart_s=depart, speed_m_s=speed_m_s, dwell_s=dwell_s,
                robot_id=robot_id, task_id=task_id,
            )
            hits = find_conflicts(proposed, existing, robot_radius_m=robot_radius_m)
            if not hits:
                return {
                    "nodes": nodes,
                    "depart_s": depart,
                    "intervals": proposed,
                    "conflicts_avoided_by_delay": slot_index > 0,
                    "attempts": attempts,
                }
    return None


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
