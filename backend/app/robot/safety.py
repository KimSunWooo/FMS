from __future__ import annotations

import math


class SafetyGuard:
    """Motion gate for this build.

    `/cmd_vel` is rejected here and is not published by any other module.
    Physical mode is always blocked. Gazebo registration is allowed only
    when the process execution mode is gazebo. The API process still does
    not open a Nav2 client.
    """

    def __init__(self, execution_mode: str = "mock") -> None:
        self.execution_mode = execution_mode
        self.velocity_publish_count = 0
        self.rejected_velocity_count = 0

    def publish_cmd_vel(self, robot_id: str, linear: float, angular: float) -> None:
        del robot_id, linear, angular
        self.rejected_velocity_count += 1
        from app.core.errors import PhysicalControlBlocked

        raise PhysicalControlBlocked(
            "Care-Move-FMS does not publish /cmd_vel. "
            "Destination motion belongs to a Nav2 action client in the robot agent, "
            "and physical motion is disabled in this build."
        )

    def assert_backend_allowed(self, mode: str) -> None:
        from app.core.errors import PhysicalControlBlocked

        if mode == "physical" or self.execution_mode == "physical":
            raise PhysicalControlBlocked(
                "Physical motor control is blocked. Elevators, doors, and real Pinky drives stay closed."
            )
        if mode == "mock":
            return
        if mode == "gazebo" and self.execution_mode == "gazebo":
            return
        if mode == "gazebo":
            raise PhysicalControlBlocked(
                "Gazebo mode is not armed. The process stays fail-closed until FMS_EXECUTION_MODE=gazebo."
            )
        raise PhysicalControlBlocked(f"Robot mode '{mode}' cannot be commanded.")

    def assert_coordinates(self, x: float, y: float, yaw: float) -> None:
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("coordinates must be finite")
