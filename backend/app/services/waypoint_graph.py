"""Planning lattice over an occupancy grid.

The occupancy resolution and the planning spacing are independent. A planning
cell is traversable only when a disk of the configured robot radius is inside
the map and entirely free. That radius comes from configuration. It is not a
verified collision-safety certificate.

Diagonal steps are rejected when either orthogonal side cell is blocked, so a
path cannot cut an obstacle corner.
"""
from __future__ import annotations

import math

from app.services.map_frame import grid_to_pixel, grid_to_world
from app.services.occupancy_grid import FREE, OccupancyGrid

ORTHOGONAL = ((1, 0), (-1, 0), (0, 1), (0, -1))
DIAGONAL = ((1, 1), (1, -1), (-1, 1), (-1, -1))


class WaypointGraph:
    def __init__(
        self,
        grid: OccupancyGrid,
        *,
        planning_resolution_m: float,
        robot_radius_m: float,
        allow_diagonal: bool = True,
    ) -> None:
        if planning_resolution_m <= 0:
            raise ValueError("planning_resolution_m must be positive")
        if robot_radius_m < 0:
            raise ValueError("robot_radius_m must be >= 0")
        self.grid = grid
        self.stride = max(1, int(round(planning_resolution_m / grid.resolution)))
        self.planning_resolution_m = self.stride * grid.resolution
        self.robot_radius_m = robot_radius_m
        self.allow_diagonal = allow_diagonal
        self.width = grid.width // self.stride
        self.height = grid.height // self.stride
        self._traversable: dict[tuple[int, int], bool] = {}
        self._segment_clear: dict[tuple[tuple[float, float], tuple[float, float]], bool] = {}

    def in_lattice(self, node: tuple[int, int]) -> bool:
        col, row = node
        return 0 <= col < self.width and 0 <= row < self.height

    def traversable(self, node: tuple[int, int]) -> bool:
        cached = self._traversable.get(node)
        if cached is not None:
            return cached
        if not self.in_lattice(node):
            self._traversable[node] = False
            return False
        x, y = self.world(node)
        allowed = self.grid.point_is_blocked(x, y, self.robot_radius_m) is None
        self._traversable[node] = allowed
        return allowed

    def world(self, node: tuple[int, int]) -> tuple[float, float]:
        col, row = self._center_index(node)
        return grid_to_world(self.grid.geometry, col, row)

    def center_index(self, node: tuple[int, int]) -> tuple[float, float]:
        return self._center_index(node)

    def locate(self, x: float, y: float) -> tuple[str, tuple[int, int] | None]:
        """Return (reason, node). The node is a lattice index, not the queried pose.

        A free point may sit in a cell whose center does not clear the robot.
        The connection is accepted only when the segment from the original
        point to that center is free on the source occupancy grid.
        """
        index = self.grid.cell_at(x, y)
        if index is None:
            return "outside", None
        state = self.grid.state(*index)
        if state != FREE:
            return state, None
        blocked = self.grid.point_is_blocked(x, y, self.robot_radius_m)
        if blocked is not None:
            return blocked, None
        base = (index[0] // self.stride, index[1] // self.stride)
        candidates: list[tuple[float, int, tuple[int, int]]] = []
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                node = (base[0] + dc, base[1] + dr)
                if not self.traversable(node):
                    continue
                sx, sy = self.world(node)
                if not self.segment_clear(x, y, sx, sy):
                    continue
                candidates.append((math.hypot(sx - x, sy - y), 0 if node == base else 1, node))
        if not candidates:
            return "snap_blocked", None
        candidates.sort()
        return "ok", candidates[0][2]

    def segment_clear(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """Sample the source grid along the segment, including the robot disk.

        Two lattice centers being free is not enough. The step is the occupancy
        resolution, so a 1 cm map is not crossed between samples.
        """
        key = ((round(x0, 4), round(y0, 4)), (round(x1, 4), round(y1, 4)))
        cached = self._segment_clear.get(key)
        if cached is not None:
            return cached
        reverse = (key[1], key[0])
        cached = self._segment_clear.get(reverse)
        if cached is not None:
            return cached
        allowed = self._segment_clear_uncached(x0, y0, x1, y1)
        self._segment_clear[key] = allowed
        return allowed

    def _segment_clear_uncached(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        length = math.hypot(x1 - x0, y1 - y0)
        step = max(float(self.grid.resolution), 1e-6)
        count = max(1, math.ceil(length / step))
        for index in range(count + 1):
            scale = index / count
            if self.grid.point_is_blocked(
                x0 + (x1 - x0) * scale,
                y0 + (y1 - y0) * scale,
                self.robot_radius_m,
            ) is not None:
                return False
        return True

    def neighbors(self, node: tuple[int, int]) -> list[tuple[tuple[int, int], float]]:
        if not self.traversable(node):
            return []
        found: list[tuple[tuple[int, int], float]] = []
        origin = self.world(node)
        for step in ORTHOGONAL:
            other = (node[0] + step[0], node[1] + step[1])
            if self.traversable(other) and self.segment_clear(*origin, *self.world(other)):
                found.append((other, self.planning_resolution_m))
        if self.allow_diagonal:
            for step in DIAGONAL:
                side_a = (node[0] + step[0], node[1])
                side_b = (node[0], node[1] + step[1])
                other = (node[0] + step[0], node[1] + step[1])
                if not (self.traversable(side_a) and self.traversable(side_b) and self.traversable(other)):
                    continue
                if not self.segment_clear(*origin, *self.world(other)):
                    continue
                found.append((other, self.planning_resolution_m * math.sqrt(2.0)))
        return found

    def describe(self, node: tuple[int, int], *, step_cost: float | None = None) -> dict:
        col, row = self._center_index(node)
        x, y = grid_to_world(self.grid.geometry, col, row)
        pixel_x, pixel_y = grid_to_pixel(self.grid.geometry, col, row)
        adjacent = [
            {"waypoint_id": self.node_id(other), "cost": round(cost, 6)}
            for other, cost in self.neighbors(node)
        ]
        return {
            "waypoint_id": self.node_id(node),
            "planning_grid": {"col": node[0], "row": node[1]},
            "grid": {"col": col, "row": row},
            "pixel": {"x": pixel_x, "y": pixel_y},
            "x": round(x, 6),
            "y": round(y, 6),
            "traversable": self.traversable(node),
            "neighbors": adjacent,
            "step_cost": None if step_cost is None else round(step_cost, 6),
        }

    def node_id(self, node: tuple[int, int]) -> str:
        return f"w-{node[0]}-{node[1]}"

    def _center_index(self, node: tuple[int, int]) -> tuple[float, float]:
        return (node[0] + 0.5) * self.stride, (node[1] + 0.5) * self.stride

    def suggestions(self, x: float, y: float, limit: int = 3) -> list[dict]:
        """Nearest traversable planning centers. Suggestions are not registered poses."""
        ranked: list[tuple[float, tuple[int, int]]] = []
        for col in range(self.width):
            for row in range(self.height):
                node = (col, row)
                if not self.traversable(node):
                    continue
                sx, sy = self.world(node)
                ranked.append((math.hypot(sx - x, sy - y), node))
        ranked.sort(key=lambda item: (item[0], item[1]))
        chosen = []
        for distance, node in ranked[:limit]:
            sx, sy = self.world(node)
            chosen.append({
                "x": round(sx, 6),
                "y": round(sy, 6),
                "waypoint_id": self.node_id(node),
                "distance_m": round(distance, 6),
                "suggestion_only": True,
                "replaces_registered_pose": False,
                "segment_clear": self.segment_clear(x, y, sx, sy),
            })
        return chosen
