"""Pure helpers for the Pinky velocity chain. No ROS imports."""
from __future__ import annotations


def classify_velocity(
    *,
    nav_linear_x: float | None,
    smoothed_linear_x: float | None,
    cmd_linear_x: float | None,
    cmd_seen: bool,
) -> str | None:
    """Classify one simultaneous sample.

    A: nav and smoothed are forward, but /cmd_vel is zero.
    B: nav and smoothed are forward, and /cmd_vel produced no message.
    C: /cmd_vel itself is forward. Motion still has to be measured separately.
    """
    if nav_linear_x is None or nav_linear_x <= 1e-4:
        return None
    if smoothed_linear_x is None or smoothed_linear_x <= 1e-4:
        return None
    if not cmd_seen:
        return "B"
    if cmd_linear_x is None or cmd_linear_x <= 1e-4:
        return "A"
    return "C"


def stop_zone_block_reason(diagnosis: dict) -> str | None:
    """Return why a drive must not start. None means this sample does not block it."""
    scan = diagnosis.get("scan") or {}
    if not scan.get("received"):
        return "LaserScan was not observed, so the StopZone cannot be cleared"
    if diagnosis.get("stop_zone_would_stop"):
        count = scan.get("points_within_stop_radius")
        radius = scan.get("stop_radius_m")
        nearest = scan.get("min_range_m")
        return (
            f"Collision Monitor StopZone would stop: {count} returns inside {radius} m"
            f" (nearest {nearest} m). The drive goal was not sent."
        )
    return None


def goal_status_name(status: int | None) -> str:
    names = {
        0: "UNKNOWN",
        1: "ACCEPTED",
        2: "EXECUTING",
        3: "CANCELING",
        4: "SUCCEEDED",
        5: "CANCELED",
        6: "ABORTED",
    }
    if status is None:
        return "UNAVAILABLE"
    return names.get(int(status), f"STATUS_{status}")
