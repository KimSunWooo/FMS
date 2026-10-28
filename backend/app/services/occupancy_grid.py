"""Occupancy grid read from the physical-site PGM.

Cell states follow the Nav2 trinary map rules. Unknown cells are not
traversable. This module only classifies pixels. It does not command a robot.
"""
from __future__ import annotations

import math
from pathlib import Path

from app.services.map_frame import (
    MapFileError,
    MapGeometry,
    grid_to_world,
    load_geometry,
    world_to_grid,
)

FREE = "free"
OCCUPIED = "occupied"
UNKNOWN = "unknown"


def classify_pixel(
    value: int,
    *,
    negate: bool,
    free_thresh: float,
    occupied_thresh: float,
) -> str:
    """Nav2 map_server occupancy probability, then trinary thresholds."""
    probability = value / 255.0
    if not negate:
        probability = 1.0 - probability
    if probability > occupied_thresh:
        return OCCUPIED
    if probability < free_thresh:
        return FREE
    return UNKNOWN


class OccupancyGrid:
    def __init__(
        self,
        geometry: MapGeometry,
        cells: bytes,
        *,
        negate: bool,
        free_thresh: float,
        occupied_thresh: float,
    ) -> None:
        expected = geometry.width_px * geometry.height_px
        if len(cells) != expected:
            raise MapFileError(f"Occupancy cell count {len(cells)} != {expected}")
        self.geometry = geometry
        self.cells = cells
        self.negate = negate
        self.free_thresh = free_thresh
        self.occupied_thresh = occupied_thresh
        self.width = geometry.width_px
        self.height = geometry.height_px
        self.resolution = geometry.resolution

    @classmethod
    def load(cls, yaml_path: Path) -> "OccupancyGrid":
        geometry = load_geometry(yaml_path)
        meta = _yaml_meta(geometry.yaml_path)
        negate = _bool_meta(meta.get("negate", "0"))
        try:
            free_thresh = float(meta.get("free_thresh", "0.196"))
            occupied_thresh = float(meta.get("occupied_thresh", "0.65"))
        except ValueError as exc:
            raise MapFileError("Invalid occupancy thresholds") from exc
        pixels = _read_pgm_bytes(geometry.image_path, geometry.width_px, geometry.height_px)
        cells = bytearray(geometry.width_px * geometry.height_px)
        for pixel_y in range(geometry.height_px):
            grid_row = geometry.height_px - 1 - pixel_y
            for col in range(geometry.width_px):
                state = classify_pixel(
                    pixels[pixel_y * geometry.width_px + col],
                    negate=negate,
                    free_thresh=free_thresh,
                    occupied_thresh=occupied_thresh,
                )
                cells[grid_row * geometry.width_px + col] = _code(state)
        return cls(
            geometry,
            bytes(cells),
            negate=negate,
            free_thresh=free_thresh,
            occupied_thresh=occupied_thresh,
        )

    @classmethod
    def from_rows(
        cls,
        rows_top_to_bottom: list[str],
        *,
        resolution: float = 1.0,
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> "OccupancyGrid":
        """Build a grid from text. The first row is the top of the image.

        `.` free, `#` occupied, `?` unknown. Grid row 0 is the last string.
        """
        if not rows_top_to_bottom or any(len(row) != len(rows_top_to_bottom[0]) for row in rows_top_to_bottom):
            raise ValueError("Occupancy rows must be a non-empty rectangle")
        height = len(rows_top_to_bottom)
        width = len(rows_top_to_bottom[0])
        geometry = MapGeometry(
            yaml_path=Path("synthetic.yaml"),
            image_path=Path("synthetic.pgm"),
            width_px=width,
            height_px=height,
            resolution=resolution,
            origin_x=origin[0],
            origin_y=origin[1],
            origin_yaw=origin[2],
        )
        cells = bytearray(width * height)
        for pixel_y, text in enumerate(rows_top_to_bottom):
            grid_row = height - 1 - pixel_y
            for col, mark in enumerate(text):
                if mark == ".":
                    state = FREE
                elif mark == "#":
                    state = OCCUPIED
                elif mark == "?":
                    state = UNKNOWN
                else:
                    raise ValueError(f"Unknown map mark {mark!r}")
                cells[grid_row * width + col] = _code(state)
        return cls(geometry, bytes(cells), negate=False, free_thresh=0.196, occupied_thresh=0.65)

    def state(self, col: int, row: int) -> str:
        return _name(self.cells[row * self.width + col])

    def cell_at(self, x: float, y: float) -> tuple[int, int] | None:
        """Occupancy index containing the map point. None when it is outside."""
        col, row = world_to_grid(self.geometry, x, y)
        if not (0.0 <= col < self.width and 0.0 <= row < self.height):
            return None
        return math.floor(col), math.floor(row)

    def state_at(self, x: float, y: float) -> str | None:
        index = self.cell_at(x, y)
        if index is None:
            return None
        return self.state(*index)

    def point_is_blocked(self, x: float, y: float, radius_m: float) -> str | None:
        """None when the disk is inside the map and entirely free.

        Otherwise a reason: outside, occupied, unknown, or clearance.
        Unknown is never treated as free.
        """
        index = self.cell_at(x, y)
        if index is None:
            return "outside"
        state = self.state(*index)
        if state == OCCUPIED:
            return "occupied"
        if state == UNKNOWN:
            return "unknown"
        if radius_m < 0:
            raise ValueError("radius_m must be >= 0")
        if radius_m == 0:
            return None
        reach = math.ceil(radius_m / self.resolution) + 1
        center_col, center_row = index
        for col in range(center_col - reach, center_col + reach + 1):
            for row in range(center_row - reach, center_row + reach + 1):
                sample_x, sample_y = grid_to_world(self.geometry, col + 0.5, row + 0.5)
                if math.hypot(sample_x - x, sample_y - y) > radius_m + 1e-9:
                    continue
                if col < 0 or row < 0 or col >= self.width or row >= self.height:
                    return "clearance"
                sample = self.state(col, row)
                if sample != FREE:
                    return "clearance"
        return None

    def nearest_occupied_distance(self, x: float, y: float) -> float | None:
        best: float | None = None
        for row in range(self.height):
            base = row * self.width
            for col in range(self.width):
                if _name(self.cells[base + col]) != OCCUPIED:
                    continue
                sample_x, sample_y = grid_to_world(self.geometry, col + 0.5, row + 0.5)
                distance = math.hypot(sample_x - x, sample_y - y)
                if best is None or distance < best:
                    best = distance
        return best


def _code(state: str) -> int:
    if state == FREE:
        return 0
    if state == OCCUPIED:
        return 1
    if state == UNKNOWN:
        return 2
    raise ValueError(state)


def _name(code: int) -> str:
    return (FREE, OCCUPIED, UNKNOWN)[code]


def _yaml_meta(path: Path) -> dict[str, str]:
    meta: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        meta[key.strip()] = value.strip()
    return meta


def _bool_meta(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes"}


def _read_pgm_bytes(path: Path, width: int, height: int) -> bytes:
    try:
        content = path.read_bytes()
    except OSError as exc:
        raise MapFileError(f"Actual map PGM unavailable: {path}: {exc}") from exc
    pos = 0

    def token() -> bytes:
        nonlocal pos
        while pos < len(content):
            if content[pos] in b" \r\n\t":
                pos += 1
            elif content[pos] == ord("#"):
                while pos < len(content) and content[pos] not in b"\r\n":
                    pos += 1
            else:
                break
        start = pos
        while pos < len(content) and content[pos] not in b" \r\n\t":
            pos += 1
        if start == pos:
            raise MapFileError(f"Truncated PGM header: {path}")
        return content[start:pos]

    fmt = token()
    image_width, image_height, max_value = int(token()), int(token()), int(token())
    if fmt != b"P5" or image_width != width or image_height != height or not 1 <= max_value <= 255:
        raise MapFileError(f"Expected an 8-bit P5 PGM of {width}x{height}: {path}")
    if content[pos:pos + 2] == b"\r\n":
        pos += 2
    else:
        pos += 1
    raw = content[pos:pos + width * height]
    if len(raw) != width * height:
        raise MapFileError(f"Incomplete PGM pixels: {path}")
    if max_value != 255:
        raw = bytes(round(value * 255 / max_value) for value in raw)
    return raw
