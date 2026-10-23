from __future__ import annotations

import math


class SafetyGuard:
    """Motion gate for this build.

    Destination travel is a future Nav2 action, not a velocity command.
    `/cmd_vel` is rejected here and is not published by any other module.
    Physical and Gazebo modes cannot be armed in Phase 1.
    """

    def __init__(self) -> None:
        self.velocity_publish_count = 0
        self.rejected_velocity_count = 0

    def publish_cmd_vel(self, robot_id: str, linear: float, angular: float) -> None:
        del robot_id, linear, angular
        self.rejected_velocity_count += 1
        from app.core.errors import PhysicalControlBlocked

        raise PhysicalControlBlocked(
            "Care-Move-FMS does not publish /cmd_vel. "
            "Destination motion belongs to a future Nav2 action client, "
            "and physical motion is disabled in this build."
        )

    def assert_backend_allowed(self, mode: str) -> None:
        from app.core.errors import PhysicalControlBlocked

        if mode != "mock":
            raise PhysicalControlBlocked(
                f"Robot mode '{mode}' cannot be commanded. Phase 1 allows mock robots only."
            )

    def assert_coordinates(self, x: float, y: float, yaw: float) -> None:
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("coordinates must be finite")
