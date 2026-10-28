"""A* and Yen's K-shortest paths on a waypoint graph.

Search results are previews only. Nothing here creates a task or sends a
Nav2 goal.
"""
from __future__ import annotations

import heapq
import math

from app.services.waypoint_graph import WaypointGraph

Node = tuple[int, int]


class PlanningFailure(Exception):
    def __init__(self, code: str, detail: str, payload: dict) -> None:
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.payload = payload


def shortest_path(
    graph: WaypointGraph,
    start: Node,
    goal: Node,
    *,
    banned_edges: set[tuple[Node, Node]] | None = None,
    banned_nodes: set[Node] | None = None,
) -> tuple[list[Node], float] | None:
    if start == goal and start not in (banned_nodes or set()):
        return [start], 0.0
    banned_edges = banned_edges or set()
    banned_nodes = banned_nodes or set()
    heuristic = lambda node: _euclidean(graph, node, goal)
    queue: list[tuple[float, float, Node]] = [(heuristic(start), 0.0, start)]
    best: dict[Node, float] = {start: 0.0}
    parent: dict[Node, Node] = {}
    while queue:
        _, cost, node = heapq.heappop(queue)
        if cost != best.get(node):
            continue
        if node == goal:
            return _reconstruct(parent, start, goal), cost
        for other, step in graph.neighbors(node):
            if other in banned_nodes or (node, other) in banned_edges:
                continue
            proposed = cost + step
            if proposed + 1e-9 < best.get(other, math.inf):
                best[other] = proposed
                parent[other] = node
                heapq.heappush(queue, (proposed + heuristic(other), proposed, other))
    return None


def k_shortest_paths(graph: WaypointGraph, start: Node, goal: Node, count: int) -> list[tuple[list[Node], float]]:
    """Yen's algorithm. Returns only distinct paths that were actually found."""
    if count < 1:
        raise ValueError("count must be >= 1")
    first = shortest_path(graph, start, goal)
    if first is None:
        return []
    accepted = [first]
    pool: list[tuple[float, tuple[Node, ...]]] = []
    seen = {tuple(first[0])}
    for _ in range(1, count):
        previous, _ = accepted[-1]
        for index in range(len(previous) - 1):
            spur = previous[index]
            root = previous[:index + 1]
            banned_edges: set[tuple[Node, Node]] = set()
            for path, _ in accepted:
                if len(path) > index and path[:index + 1] == root:
                    banned_edges.add((path[index], path[index + 1]))
            spur_result = shortest_path(
                graph,
                spur,
                goal,
                banned_edges=banned_edges,
                banned_nodes=set(root[:-1]),
            )
            if spur_result is None:
                continue
            spur_nodes, _ = spur_result
            combined = root[:-1] + spur_nodes
            identity = tuple(combined)
            if identity in seen:
                continue
            seen.add(identity)
            pool.append((_path_cost(graph, combined), identity))
        if not pool:
            break
        pool.sort(key=lambda item: (item[0], item[1]))
        cost, identity = pool.pop(0)
        accepted.append((list(identity), cost))
    return accepted


def _euclidean(graph: WaypointGraph, left: Node, right: Node) -> float:
    lx, ly = graph.world(left)
    rx, ry = graph.world(right)
    return math.hypot(lx - rx, ly - ry)


def _path_cost(graph: WaypointGraph, nodes: list[Node]) -> float:
    total = 0.0
    for left, right in zip(nodes, nodes[1:]):
        total += _step_cost(graph, left, right)
    return total


def _step_cost(graph: WaypointGraph, left: Node, right: Node) -> float:
    for other, cost in graph.neighbors(left):
        if other == right:
            return cost
    raise PlanningFailure("path_edge", "Reconstructed path left the waypoint graph", {})


def _reconstruct(parent: dict[Node, Node], start: Node, goal: Node) -> list[Node]:
    nodes = [goal]
    while nodes[-1] != start:
        nodes.append(parent[nodes[-1]])
    nodes.reverse()
    return nodes


def path_record(graph: WaypointGraph, nodes: list[Node], cost: float, rank: int) -> dict:
    waypoints = []
    previous: Node | None = None
    for node in nodes:
        step = 0.0 if previous is None else _step_cost(graph, previous, node)
        waypoints.append(graph.describe(node, step_cost=step))
        previous = node
    length = 0.0
    for left, right in zip(nodes, nodes[1:]):
        lx, ly = graph.world(left)
        rx, ry = graph.world(right)
        length += math.hypot(lx - rx, ly - ry)
    return {
        "path_id": f"path-{rank}",
        "rank": rank,
        "success": True,
        "waypoint_count": len(waypoints),
        "waypoints": waypoints,
        "length_m": round(length, 6),
        "cost": round(cost, 6),
    }
