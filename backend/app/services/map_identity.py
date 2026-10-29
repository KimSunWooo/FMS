"""Compare the FMS map file with a read-only /map_server observation.

Lifecycle active is not a match. A missing YAML file is not rewritten here,
and this module does not restart Nav2.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

from app.services.occupancy_grid import OccupancyGrid

_GEOMETRY_TOLERANCE = 1e-4


def occupancy_sha256(cells: bytes) -> str:
    return hashlib.sha256(cells).hexdigest()


def assess_map_identity(*, expected_yaml: Path, server_yaml: str | None, probe: dict | None) -> dict:
    """Return map match fields. `map_matches_fms` is true only when YAML, geometry, and cells agree."""
    report = {
        "map_matches_fms": False,
        "map_server_yaml": server_yaml,
        "expected_map_yaml": str(expected_yaml),
        "map_content_sha256": None,
        "expected_map_sha256": None,
        "map_mismatch_detail": "map identity not verified",
    }
    try:
        grid = OccupancyGrid.load(expected_yaml)
    except Exception as exc:
        report["map_mismatch_detail"] = f"FMS map unreadable: {type(exc).__name__}"
        return report
    expected_sha = occupancy_sha256(grid.cells)
    report["expected_map_sha256"] = expected_sha
    if not server_yaml:
        report["map_mismatch_detail"] = "map_server yaml_filename unavailable"
        return report
    server_path = Path(server_yaml)
    if not server_path.is_file():
        yaml_same = False
        yaml_detail = f"map_server yaml file is missing: {server_yaml}; FMS map is {expected_yaml}"
    else:
        try:
            yaml_same = server_path.resolve() == expected_yaml.resolve()
        except OSError:
            yaml_same = False
        yaml_detail = f"map_server yaml {server_yaml} does not match FMS map {expected_yaml}"
    if probe is None or probe.get("error"):
        error = (probe or {}).get("error") or "no map observation"
        report["map_mismatch_detail"] = f"{yaml_detail}; map identity observation failed: {error}"
        return report
    try:
        info_same = (
            int(probe["width"]) == grid.width
            and int(probe["height"]) == grid.height
            and abs(float(probe["resolution"]) - grid.resolution) <= _GEOMETRY_TOLERANCE
            and abs(float(probe["origin_x"]) - grid.geometry.origin_x) <= _GEOMETRY_TOLERANCE
            and abs(float(probe["origin_y"]) - grid.geometry.origin_y) <= _GEOMETRY_TOLERANCE
            and abs(float(probe["origin_yaw"]) - grid.geometry.origin_yaw) <= _GEOMETRY_TOLERANCE
        )
    except (KeyError, TypeError, ValueError):
        report["map_mismatch_detail"] = f"map metadata incomplete from /map; server yaml {server_yaml}"
        return report
    loaded_sha = probe.get("content_sha256")
    report["map_content_sha256"] = loaded_sha if isinstance(loaded_sha, str) else None
    content_same = isinstance(loaded_sha, str) and loaded_sha == expected_sha
    if yaml_same and info_same and content_same:
        report["map_matches_fms"] = True
        report["map_mismatch_detail"] = ""
        return report
    parts: list[str] = []
    if not yaml_same:
        parts.append(yaml_detail)
    if not info_same:
        parts.append(
            f"/map geometry {probe.get('width')}x{probe.get('height')} "
            f"res {probe.get('resolution')} differs from {grid.width}x{grid.height} res {grid.resolution}"
        )
    if not content_same:
        parts.append(
            f"loaded occupancy sha256 {str(loaded_sha)[:12]} differs from FMS map sha256 {expected_sha[:12]}"
        )
    report["map_mismatch_detail"] = "; ".join(parts)
    return report
