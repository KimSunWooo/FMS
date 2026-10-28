"""Map-frame meters and Nav2 occupancy-grid indices for the physical site map.

Grid column 0 is the left edge of the PGM. Grid row 0 is the bottom edge, the
same cell as the YAML origin. Pixel row 0 is the top of the image, matching
the dashboard SVG. This module does not plan paths or command a robot.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MAP = ROOT / "config/maps/physical_site/1f_map_f.yaml"


class MapFileError(Exception):
    pass


@dataclass(frozen=True)
class MapGeometry:
    yaml_path: Path
    image_path: Path
    width_px: int
    height_px: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float

    def public_dict(self) -> dict:
        return {
            "frame_id": "map",
            "width_px": self.width_px,
            "height_px": self.height_px,
            "width_m": round(self.width_px * self.resolution, 6),
            "height_m": round(self.height_px * self.resolution, 6),
            "resolution": self.resolution,
            "origin": {"x": self.origin_x, "y": self.origin_y, "yaw": self.origin_yaw},
            "image_url": "/fms-api/physical/map/image",
            "source": self.yaml_path.name,
            "data_source": "physical_site_map_files",
            "grid": {
                "width": self.width_px,
                "height": self.height_px,
                "resolution": self.resolution,
                "origin_cell": {"col": 0, "row": 0},
                "convention": "nav2_occupancy",
                "row_origin": "image_bottom",
            },
        }


def _pgm_size(path: Path) -> tuple[int, int]:
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

    try:
        fmt = token()
        width, height = int(token()), int(token())
    except (ValueError, IndexError) as exc:
        raise MapFileError(f"Could not read PGM size: {path}") from exc
    if fmt not in (b"P5", b"P2") or width <= 0 or height <= 0:
        raise MapFileError(f"Invalid PGM header: {path}")
    return width, height


def load_geometry(yaml_path: Path) -> MapGeometry:
    yaml_path = Path(yaml_path).expanduser()
    try:
        lines = yaml_path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise MapFileError(f"Actual map YAML unavailable: {yaml_path}: {exc}") from exc
    meta: dict[str, str] = {}
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        meta[key.strip()] = value.strip()
    if not meta.get("image"):
        raise MapFileError("Map YAML has no 'image' field")
    image = Path(meta["image"].strip("\"'"))
    if not image.is_absolute():
        image = yaml_path.parent / image
    if image.suffix.lower() != ".pgm":
        raise MapFileError("Expected the downloaded Nav2 .pgm map image")
    try:
        resolution = float(meta["resolution"])
        origin = [float(part) for part in meta["origin"].strip("[]").split(",")]
    except (KeyError, ValueError) as exc:
        raise MapFileError("Invalid map YAML origin/resolution") from exc
    if resolution <= 0 or len(origin) != 3:
        raise MapFileError("Invalid map geometry")
    width, height = _pgm_size(image)
    return MapGeometry(
        yaml_path=yaml_path,
        image_path=image,
        width_px=width,
        height_px=height,
        resolution=resolution,
        origin_x=origin[0],
        origin_y=origin[1],
        origin_yaw=origin[2],
    )


def world_to_grid(geom: MapGeometry, x: float, y: float) -> tuple[float, float]:
    """Continuous occupancy-grid indices. Row 0 lies on the YAML origin."""
    theta = geom.origin_yaw
    dx = x - geom.origin_x
    dy = y - geom.origin_y
    local_x = dx * math.cos(theta) + dy * math.sin(theta)
    local_y = -dx * math.sin(theta) + dy * math.cos(theta)
    return local_x / geom.resolution, local_y / geom.resolution


def grid_to_world(geom: MapGeometry, col: float, row: float) -> tuple[float, float]:
    theta = geom.origin_yaw
    local_x = col * geom.resolution
    local_y = row * geom.resolution
    x = geom.origin_x + local_x * math.cos(theta) - local_y * math.sin(theta)
    y = geom.origin_y + local_x * math.sin(theta) + local_y * math.cos(theta)
    return x, y


def grid_to_pixel(geom: MapGeometry, col: float, row: float) -> tuple[float, float]:
    return col, geom.height_px - row


def pixel_to_grid(geom: MapGeometry, px: float, py: float) -> tuple[float, float]:
    return px, geom.height_px - py


def inside_map(geom: MapGeometry, col: float, row: float) -> bool:
    return 0 <= col <= geom.width_px and 0 <= row <= geom.height_px


def describe_map_point(geom: MapGeometry, x: float, y: float, yaw: float = 0.0) -> dict:
    col, row = world_to_grid(geom, x, y)
    px, py = grid_to_pixel(geom, col, row)
    return {
        "frame_id": "map",
        "x": x,
        "y": y,
        "yaw": yaw,
        "grid": {"col": col, "row": row},
        "pixel": {"x": px, "y": py},
        "inside_map": inside_map(geom, col, row),
    }


def describe_grid_point(geom: MapGeometry, col: float, row: float) -> dict:
    x, y = grid_to_world(geom, col, row)
    px, py = grid_to_pixel(geom, col, row)
    return {
        "frame_id": "map",
        "x": x,
        "y": y,
        "yaw": 0.0,
        "grid": {"col": col, "row": row},
        "pixel": {"x": px, "y": py},
        "inside_map": inside_map(geom, col, row),
    }
