"""Nav2 action client for a confirmed Gazebo session.

Imports of the ROS client library happen inside the send path, after the
preflight and domain checks. This module never invents a SUCCEEDED result
or a goal UUID. A pose is reported only after a TF lookup in the goal frame.
"""

from __future__ import annotations

import importlib
import math
import time
import uuid
from datetime import datetime, timezone

from robot_agent.preflight import PreflightReport


class GazeboNav2Unavailable(RuntimeError):
    pass


_STATUS_ACCEPTED = 1
_STATUS_EXECUTING = 2
_STATUS_CANCELING = 3
_STATUS_SUCCEEDED = 4
_STATUS_CANCELED = 5
_STATUS_ABORTED = 6
_ACTIVE_GOAL_STATUSES = {_STATUS_ACCEPTED, _STATUS_EXECUTING, _STATUS_CANCELING}


def goal_statuses_block_send(statuses: list[int] | None) -> bool:
    """A missing status list is uncertain, so it blocks a new goal."""
    if statuses is None:
        return True
    return any(int(status) in _ACTIVE_GOAL_STATUSES for status in statuses)


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

    def send_goal(
        self,
        *,
        robot_id: str,
        goal_id: str,
        x: float,
        y: float,
        yaw: float,
        frame_id: str = "map",
        action_name: str = "/navigate_to_pose",
        use_sim_time: bool = True,
    ) -> dict:
        self.assert_can_send()
        del robot_id  # FMS id. The Pinky launch is not namespaced by that id.
        try:
            rclpy = importlib.import_module("rclpy")
            nav2_action = importlib.import_module("nav2_msgs.action")
            action_client_mod = importlib.import_module("rclpy.action")
            parameter_mod = importlib.import_module("rclpy.parameter")
            tf2_ros = importlib.import_module("tf2_ros")
        except ImportError as exc:
            raise GazeboNav2Unavailable(
                "Nav2 action libraries are not importable. Refusing to fabricate SUCCEEDED."
            ) from exc
        return _observe_navigate_to_pose(
            rclpy,
            nav2_action,
            action_client_mod,
            parameter_mod,
            tf2_ros,
            goal_id=goal_id,
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            action_name=action_name,
            use_sim_time=use_sim_time,
        )


def format_action_goal_id(goal_id) -> str | None:
    """Format a Nav2 action UUID. A missing or short id stays None."""
    raw = getattr(goal_id, "uuid", None)
    if raw is None:
        return None
    blob = bytes(raw)
    if len(blob) != 16 or blob == bytes(16):
        return None
    return str(uuid.UUID(bytes=blob))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _observe_navigate_to_pose(
    rclpy,
    nav2_action,
    action_client_mod,
    parameter_mod,
    tf2_ros,
    *,
    goal_id,
    x,
    y,
    yaw,
    frame_id,
    action_name,
    use_sim_time,
    cancel_event=None,
    node=None,
    tf_buffer=None,
) -> dict:
    """Return only a status the action server actually reported."""
    started = False
    own_node = node is None
    if own_node and not rclpy.ok():
        rclpy.init()
        started = True
    absolute_action = action_name if str(action_name).startswith("/") else f"/{action_name}"
    if own_node:
        node = rclpy.create_node(
            "care_move_fms_nav2_client",
            parameter_overrides=[
                parameter_mod.Parameter("use_sim_time", parameter_mod.Parameter.Type.BOOL, bool(use_sim_time))
            ],
        )
        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer, node, spin_thread=False)
    feedback_count = {"n": 0}
    try:
        client = action_client_mod.ActionClient(node, nav2_action.NavigateToPose, absolute_action)
        if not client.wait_for_server(timeout_sec=15.0):
            raise GazeboNav2Unavailable(f"{absolute_action} action server was not observed")
        goal = nav2_action.NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        def _feedback(_msg) -> None:
            feedback_count["n"] += 1

        goal_sent_at = _utc_now()
        send_future = client.send_goal_async(goal, feedback_callback=_feedback)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=15.0)
        if not send_future.done():
            return _unfinished(
                goal_id,
                absolute_action,
                goal_sent_at,
                feedback_count["n"],
                "goal_request_unanswered_uuid_unknown",
            )
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return {
                "outcome": "rejected",
                "goal_id": goal_id,
                "nav2_action_goal_id": None,
                "nav2_action_goal_id_absent_reason": "action_server_rejected_before_goal_id",
                "nav2_status": None,
                "nav2_action": absolute_action,
                "goal_sent_at": goal_sent_at,
                "goal_accepted_at": None,
                "goal_result_at": _utc_now(),
                "feedback_count": feedback_count["n"],
                "pose": None,
            }
        goal_accepted_at = _utc_now()
        nav2_action_goal_id = format_action_goal_id(handle.goal_id)
        result_future = handle.get_result_async()
        deadline = time.monotonic() + 180.0
        cancel_sent = False
        while not result_future.done():
            if cancel_event is not None and cancel_event.is_set() and not cancel_sent:
                handle.cancel_goal_async()
                cancel_sent = True
            if time.monotonic() >= deadline:
                return _unfinished(
                    goal_id,
                    absolute_action,
                    goal_sent_at,
                    feedback_count["n"],
                    None if nav2_action_goal_id else "result_timeout_without_goal_id",
                    nav2_action_goal_id=nav2_action_goal_id,
                    goal_accepted_at=goal_accepted_at,
                )
            rclpy.spin_once(node, timeout_sec=0.1)
        if result_future.result() is None:
            return _unfinished(
                goal_id,
                absolute_action,
                goal_sent_at,
                feedback_count["n"],
                None if nav2_action_goal_id else "result_timeout_without_goal_id",
                nav2_action_goal_id=nav2_action_goal_id,
                goal_accepted_at=goal_accepted_at,
            )
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
        pose = _lookup_pose(rclpy, node, tf_buffer, frame_id)
        return {
            "outcome": outcome,
            "goal_id": goal_id,
            "nav2_action_goal_id": nav2_action_goal_id,
            "nav2_action_goal_id_absent_reason": None if nav2_action_goal_id else "terminal_result_without_goal_id",
            "nav2_status": status,
            "nav2_action": absolute_action,
            "goal_sent_at": goal_sent_at,
            "goal_accepted_at": goal_accepted_at,
            "goal_result_at": _utc_now(),
            "feedback_count": feedback_count["n"],
            "pose": pose,
        }
    finally:
        if own_node:
            node.destroy_node()
            if started and rclpy.ok():
                rclpy.shutdown()


def _unfinished(
    goal_id,
    action_name,
    goal_sent_at,
    feedback_count,
    absent_reason,
    *,
    nav2_action_goal_id=None,
    goal_accepted_at=None,
) -> dict:
    """A goal whose terminal status was not observed. This is not success."""
    return {
        "outcome": "timed_out",
        "goal_id": goal_id,
        "nav2_action_goal_id": nav2_action_goal_id,
        "nav2_action_goal_id_absent_reason": absent_reason,
        "nav2_status": None,
        "nav2_action": action_name,
        "goal_sent_at": goal_sent_at,
        "goal_accepted_at": goal_accepted_at,
        "goal_result_at": _utc_now(),
        "feedback_count": feedback_count,
        "pose": None,
    }


def _lookup_pose(rclpy, node, tf_buffer, frame_id: str, settle_seconds: float = 2.0) -> dict | None:
    """Latest body pose in the goal frame after the result.

    The first sample can still be the in-motion pose from the instant Nav2
    reports success. Keep the newest matching transform for a short settle
    window. A missing or mismatched frame stays None.
    """
    deadline = time.monotonic() + settle_seconds
    latest = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        for child in ("base_footprint", "base_link"):
            try:
                transform = tf_buffer.lookup_transform(frame_id, child, rclpy.time.Time())
            except Exception:
                continue
            observed_frame = transform.header.frame_id or frame_id
            if observed_frame != frame_id:
                return None
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            stamp = transform.header.stamp
            latest = {
                "x": float(translation.x),
                "y": float(translation.y),
                "z": float(translation.z),
                "yaw": yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
                "frame_id": observed_frame,
                "child_frame_id": child,
                "stamp_sec": int(stamp.sec),
                "stamp_nanosec": int(stamp.nanosec),
            }
            break
    return latest


class SimNavigation:
    """One ROS node for pose, goal-status, and a single NavigateToPose send."""

    def __init__(self, *, use_sim_time: bool = True) -> None:
        self.use_sim_time = use_sim_time
        self._started = False
        self._rclpy = None
        self._nav2_action = None
        self._action_client_mod = None
        self._parameter_mod = None
        self._tf2_ros = None
        self.node = None
        self.tf_buffer = None

    def __enter__(self) -> "SimNavigation":
        try:
            self._rclpy = importlib.import_module("rclpy")
            self._nav2_action = importlib.import_module("nav2_msgs.action")
            self._action_client_mod = importlib.import_module("rclpy.action")
            self._parameter_mod = importlib.import_module("rclpy.parameter")
            self._tf2_ros = importlib.import_module("tf2_ros")
        except ImportError as exc:
            raise GazeboNav2Unavailable(
                "Nav2 action libraries are not importable. Refusing to fabricate SUCCEEDED."
            ) from exc
        if not self._rclpy.ok():
            self._rclpy.init()
            self._started = True
        self.node = self._rclpy.create_node(
            "care_move_fms_nav2_client",
            parameter_overrides=[
                self._parameter_mod.Parameter(
                    "use_sim_time", self._parameter_mod.Parameter.Type.BOOL, bool(self.use_sim_time)
                )
            ],
        )
        self.tf_buffer = self._tf2_ros.Buffer()
        self._tf2_ros.TransformListener(self.tf_buffer, self.node, spin_thread=False)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self._started and self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()

    def observe_pose(self, frame_id: str, settle_seconds: float = 5.0) -> dict | None:
        return _lookup_pose(self._rclpy, self.node, self.tf_buffer, frame_id, settle_seconds)

    def goal_statuses(self, action_name: str = "/navigate_to_pose", timeout_sec: float = 2.0) -> list[int] | None:
        qos_mod = importlib.import_module("rclpy.qos")
        status_mod = importlib.import_module("action_msgs.msg")
        absolute = action_name if str(action_name).startswith("/") else f"/{action_name}"
        holder: dict = {"msg": None}

        def _on_status(msg) -> None:
            holder["msg"] = msg

        subscription = self.node.create_subscription(
            status_mod.GoalStatusArray,
            f"{absolute}/_action/status",
            _on_status,
            qos_mod.qos_profile_action_status_default,
        )
        deadline = time.monotonic() + timeout_sec
        try:
            while time.monotonic() < deadline and holder["msg"] is None:
                self._rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(subscription)
        if holder["msg"] is None:
            return None
        return [int(item.status) for item in holder["msg"].status_list]

    def send_goal(self, **kwargs) -> dict:
        return _observe_navigate_to_pose(
            self._rclpy,
            self._nav2_action,
            self._action_client_mod,
            self._parameter_mod,
            self._tf2_ros,
            use_sim_time=self.use_sim_time,
            node=self.node,
            tf_buffer=self.tf_buffer,
            **kwargs,
        )
