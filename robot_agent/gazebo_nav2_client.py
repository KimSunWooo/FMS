"""Nav2 action client for a confirmed Gazebo session.

Imports of the ROS client library happen inside the send path, after the
preflight and domain checks. This module never invents a SUCCEEDED result.
"""

from __future__ import annotations

import importlib
import math

from robot_agent.preflight import PreflightReport


class GazeboNav2Unavailable(RuntimeError):
    pass


_STATUS_SUCCEEDED = 4
_STATUS_CANCELED = 5
_STATUS_ABORTED = 6


class GazeboNav2Client:
    def __init__(
        self,
        *,
        mode: str,
        expected_domain: int,
        observed_domain: int | None,
        preflight: PreflightReport,
    ) -> None:
        self.mode = mode
        self.expected_domain = expected_domain
        self.observed_domain = observed_domain
        self.preflight = preflight

    def assert_can_send(self) -> None:
        if self.mode == "physical":
            raise GazeboNav2Unavailable("physical mode is blocked; no Nav2 goal is sent")
        if self.mode != "gazebo":
            raise GazeboNav2Unavailable(f"mode {self.mode} is not a Gazebo Nav2 session")
        if not self.preflight.ready:
            reasons = ",".join(self.preflight.reasons)
            raise GazeboNav2Unavailable(f"gazebo preflight is not ready: {reasons}")
        if self.observed_domain != self.expected_domain:
            raise GazeboNav2Unavailable(
                f"ROS domain {self.observed_domain!r} does not match expected {self.expected_domain}"
            )

    def send_goal(self, *, robot_id: str, goal_id: str, x: float, y: float, yaw: float, frame_id: str = "map") -> dict:
        self.assert_can_send()
        try:
            rclpy = importlib.import_module("rclpy")
            nav2_action = importlib.import_module("nav2_msgs.action")
            action_client_mod = importlib.import_module("rclpy.action")
        except ImportError as exc:
            raise GazeboNav2Unavailable(
                "Nav2 action libraries are not importable. Refusing to fabricate SUCCEEDED."
            ) from exc
        return _observe_navigate_to_pose(
            rclpy,
            nav2_action,
            action_client_mod,
            robot_id=robot_id,
            goal_id=goal_id,
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
        )


def _observe_navigate_to_pose(rclpy, nav2_action, action_client_mod, *, robot_id, goal_id, x, y, yaw, frame_id) -> dict:
    """Return only a status the action server actually reported."""
    started = False
    if not rclpy.ok():
        rclpy.init()
        started = True
    node = rclpy.create_node("care_move_fms_nav2_client", namespace=f"/{robot_id}")
    try:
        client = action_client_mod.ActionClient(node, nav2_action.NavigateToPose, "navigate_to_pose")
        if not client.wait_for_server(timeout_sec=3.0):
            raise GazeboNav2Unavailable("navigate_to_pose action server was not observed")
        goal = nav2_action.NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
        if not send_future.done():
            raise GazeboNav2Unavailable("Nav2 did not answer the goal request. Not marking success.")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return {
                "outcome": "rejected",
                "goal_id": goal_id,
                "nav2_action_goal_id": None,
                "nav2_status": None,
            }
        nav2_action_goal_id = str(getattr(handle, "goal_id", "")) or None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=180.0)
        if not result_future.done() or result_future.result() is None:
            raise GazeboNav2Unavailable("Nav2 result was not observed. Refusing to fabricate SUCCEEDED.")
        status = int(result_future.result().status)
        if status == _STATUS_SUCCEEDED:
            outcome = "succeeded"
        elif status == _STATUS_ABORTED:
            outcome = "aborted"
        elif status == _STATUS_CANCELED:
            outcome = "canceled"
        else:
            raise GazeboNav2Unavailable(
                f"Nav2 status {status} is not an explicit terminal result. Not marking SUCCEEDED."
            )
        return {
            "outcome": outcome,
            "goal_id": goal_id,
            "nav2_action_goal_id": nav2_action_goal_id,
            "nav2_status": status,
        }
    finally:
        node.destroy_node()
        if started and rclpy.ok():
            rclpy.shutdown()
