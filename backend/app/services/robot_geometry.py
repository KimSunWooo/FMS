"""Pinky circular planning model derived from Nav2 / URDF footprint.

Source of truth for the body outline used by FMS planning:
  simulation/.../pinky_navigation/params/nav2_params.yaml
  local_costmap / global_costmap footprint:
    [[0.06, 0.06], [0.06, -0.06], [-0.06, -0.06], [-0.06, 0.06]]
  frame: base_footprint (square, symmetric about the origin)

R_body is the circumradius of that rectangle:
  length = width = 0.12 m
  R_body = sqrt(length^2 + width^2) / 2 = 0.06 * sqrt(2)

Clearance is applied once: R_effective = R_body + 0.01 m.
Planning resolution equals R_effective. Inter-robot separation uses the sum of
radii (2 * R_effective), never diameter confusion or a second margin.

The base_link collision mesh farthest XY point is ~0.0826 m (inside the Nav2
square circumradius). Nav2 inflation / Collision Monitor StopZone are separate
and are not altered here.
"""
from __future__ import annotations

import math
from typing import Any

# Nav2 footprint half-extents in base_footprint (meters).
PINKY_FOOTPRINT_HALF_X_M = 0.06
PINKY_FOOTPRINT_HALF_Y_M = 0.06
PINKY_BODY_LENGTH_M = 2.0 * PINKY_FOOTPRINT_HALF_X_M  # 0.12
PINKY_BODY_WIDTH_M = 2.0 * PINKY_FOOTPRINT_HALF_Y_M   # 0.12

PINKY_R_BODY_M = math.sqrt(PINKY_BODY_LENGTH_M**2 + PINKY_BODY_WIDTH_M**2) / 2.0
PINKY_CLEARANCE_M = 0.01
PINKY_R_EFFECTIVE_M = PINKY_R_BODY_M + PINKY_CLEARANCE_M

# Documented comparisons (not applied as planning radius).
NAV2_COMMENTED_ROBOT_RADIUS_M = 0.22  # commented-out robot_radius in nav2_params
NAV2_INFLATION_RADIUS_M = 0.15
NAV2_FOOTPRINT_PADDING_M = 0.03
MESH_MAX_XY_RADIUS_M = 0.082570  # base_link.stl + drive wheels, base_footprint XY


def planning_geometry_public() -> dict[str, Any]:
    """Values exposed on planning APIs and the fleet console."""
    return {
        "model": "pinky_nav2_square_footprint",
        "frame_id": "base_footprint",
        "source": "nav2_params.yaml footprint [[±0.06, ±0.06]]",
        "body": {
            "shape": "rectangle",
            "length_m": PINKY_BODY_LENGTH_M,
            "width_m": PINKY_BODY_WIDTH_M,
            "half_x_m": PINKY_FOOTPRINT_HALF_X_M,
            "half_y_m": PINKY_FOOTPRINT_HALF_Y_M,
            "asymmetric": False,
            "formula": "R_body = sqrt(length^2 + width^2) / 2",
        },
        "r_body_m": PINKY_R_BODY_M,
        "clearance_m": PINKY_CLEARANCE_M,
        "r_effective_m": PINKY_R_EFFECTIVE_M,
        "planning_resolution_m": PINKY_R_EFFECTIVE_M,
        "planning_robot_radius_m": PINKY_R_EFFECTIVE_M,
        "inter_robot_min_center_distance_m": 2.0 * PINKY_R_EFFECTIVE_M,
        "comparisons": {
            "mesh_max_xy_radius_m": MESH_MAX_XY_RADIUS_M,
            "nav2_commented_robot_radius_m": NAV2_COMMENTED_ROBOT_RADIUS_M,
            "nav2_inflation_radius_m": NAV2_INFLATION_RADIUS_M,
            "nav2_footprint_padding_m": NAV2_FOOTPRINT_PADDING_M,
            "note": "Inflation and Collision Monitor are not reduced or disabled by FMS.",
        },
        "collision_safety_verified": False,
    }
