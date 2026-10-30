"""Registered AMCL initial-pose seeds per physical robot.

Source of truth (when Settings overrides are unset):
  config/maps/physical_site/nav2_params.yaml → amcl.ros__parameters.initial_pose

The historical UI coordinate (1.696, 0.103) is NOT treated as the registered
start unless an operator explicitly configures it. Yaw in nav2_params is radians.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_NAV2_PARAMS = ROOT / "config/maps/physical_site/nav2_params.yaml"

# Default covariance used for /initialpose (map frame).
DEFAULT_COVARIANCE = [0.0] * 36
DEFAULT_COVARIANCE[0] = 0.25 ** 2
DEFAULT_COVARIANCE[7] = 0.25 ** 2
DEFAULT_COVARIANCE[35] = (math.pi / 12.0) ** 2


def _parse_nav2_initial_pose(path: Path) -> dict[str, float] | None:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return None
    # Avoid requiring PyYAML in the API venv; the block is small and stable.
    in_pose = False
    values: dict[str, float] = {}
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if line.strip() == "initial_pose:":
            in_pose = True
            continue
        if in_pose:
            if not raw.startswith(" ") and not raw.startswith("\t"):
                break
            if ":" not in line:
                continue
            key, value = line.strip().split(":", 1)
            key = key.strip()
            if key in {"x", "y", "z", "yaw"}:
                try:
                    values[key] = float(value.strip())
                except ValueError:
                    return None
    if not {"x", "y", "yaw"} <= set(values):
        return None
    return values


def resolve_registered_initial_pose(settings, *, robot_id: str) -> dict[str, Any]:
    """Return a registered seed or an explicit rejection reason."""
    import os

    from app.services.fleet_registry import CANONICAL_DOMAINS
    from app.services.map_identity import occupancy_sha256
    from app.services.occupancy_grid import OccupancyGrid

    domain = CANONICAL_DOMAINS.get(robot_id)
    if domain is None:
        domain = int(getattr(settings, "ros_domain_id", 24))
    map_yaml = Path(settings.physical_map_yaml)
    nav2_params = Path(
        getattr(settings, "physical_nav2_params_yaml", None) or DEFAULT_NAV2_PARAMS
    )
    override_x = getattr(settings, "physical_initial_pose_x", None)
    override_y = getattr(settings, "physical_initial_pose_y", None)
    override_yaw = getattr(settings, "physical_initial_pose_yaw_rad", None)
    if override_yaw is None and "FMS_PHYSICAL_INITIAL_POSE_YAW_DEG" in os.environ:
        override_yaw = math.radians(float(settings.physical_initial_pose_yaw_deg))

    parsed = _parse_nav2_initial_pose(nav2_params)
    source = f"nav2_params:{nav2_params.name}"
    if parsed is None and (override_x is None or override_y is None or override_yaw is None):
        return {
            "valid": False,
            "reason": "initial_pose_unregistered",
            "detail": f"No usable initial_pose in {nav2_params} and Settings overrides incomplete",
            "robot_id": robot_id,
            "ros_domain_id": domain,
        }
    x = float(override_x if override_x is not None else parsed["x"])
    y = float(override_y if override_y is not None else parsed["y"])
    yaw = float(override_yaw if override_yaw is not None else parsed["yaw"])
    if override_x is not None or override_y is not None or override_yaw is not None:
        source = "settings_override" if parsed is None else "settings_partial_override"

    if not all(math.isfinite(v) for v in (x, y, yaw)):
        return {
            "valid": False,
            "reason": "initial_pose_invalid",
            "detail": "Registered initial pose contains non-finite values",
            "robot_id": robot_id,
            "ros_domain_id": domain,
        }

    map_sha = None
    try:
        if map_yaml.is_file():
            map_sha = occupancy_sha256(OccupancyGrid.load(map_yaml).cells)
    except Exception:  # noqa: BLE001
        map_sha = None

    return {
        "valid": True,
        "reason": "ok",
        "robot_id": robot_id,
        "ros_domain_id": int(domain),
        "map_yaml": str(map_yaml),
        "map_sha256": map_sha,
        "frame_id": "map",
        "x": x,
        "y": y,
        "z": 0.0,
        "yaw_rad": yaw,
        "yaw_deg": math.degrees(yaw),
        "covariance": list(DEFAULT_COVARIANCE),
        "source": source,
        "topic": str(getattr(settings, "physical_initial_pose_topic", "/initialpose")),
        "message_type": "geometry_msgs/msg/PoseWithCovarianceStamped",
        "auto_enabled": bool(getattr(settings, "physical_initial_pose_auto", False)),
    }


def pose_with_covariance_dict(seed: dict[str, Any]) -> dict[str, Any]:
    yaw = float(seed["yaw_rad"])
    return {
        "header": {"frame_id": "map"},
        "pose": {
            "pose": {
                "position": {"x": float(seed["x"]), "y": float(seed["y"]), "z": float(seed.get("z") or 0.0)},
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": math.sin(yaw / 2.0),
                    "w": math.cos(yaw / 2.0),
                },
            },
            "covariance": list(seed.get("covariance") or DEFAULT_COVARIANCE),
        },
    }
