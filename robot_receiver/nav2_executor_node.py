"""Nav2 executor for stored FMS tasks with execute=true.

Role split:
- fms_task_receiver: validate, store, publish receive TaskAck (status=stored)
- fms_nav2_executor: read inbox, send NavigateThroughPoses for all waypoints,
  write execution IPC (does NOT publish TaskAck)

Duplicate command ids do not start a second Nav2 goal while one is active or
already terminal for the same task_id+payload_hash.
"""
from __future__ import annotations

import json
import math
import os
import sys
import threading
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from robot_receiver.ipc_atomic import write_json_atomic
from robot_receiver.ros_messages import cancel_to_dict
from robot_receiver.task_codec import TOPIC_CANCEL

TERMINAL = {"succeeded", "failed", "cancelled"}


def reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class Nav2ExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("fms_nav2_executor")
        self.robot_id = os.environ["FMS_RECEIVER_ROBOT_ID"]
        self.domain_id = int(os.environ["ROS_DOMAIN_ID"])
        data_dir = Path(os.environ.get("FMS_DATA_DIR") or (ROOT / "data"))
        self.store = Path(
            os.environ.get("FMS_RECEIVER_INBOX")
            or (data_dir / "ros_ipc" / self.robot_id / "inbox")
        )
        self.store.mkdir(parents=True, exist_ok=True)
        self.execution_dir = data_dir / "ros_ipc" / self.robot_id / "execution"
        self.execution_dir.mkdir(parents=True, exist_ok=True)
        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._active_task_id: str | None = None
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._action = ActionClient(
            self,
            NavigateThroughPoses,
            "navigate_through_poses",
            callback_group=self._cb_group,
        )
        self.create_subscription(
            self._cancel_type(),
            os.environ.get("FMS_TOPIC_TASK_CANCEL", TOPIC_CANCEL),
            self._on_cancel_topic,
            reliable_qos(),
            callback_group=self._cb_group,
        )
        self.create_timer(0.5, self._poll_inbox, callback_group=self._cb_group)
        self.get_logger().info(
            f"Nav2 executor ready: robot_id={self.robot_id} domain={self.domain_id} "
            f"inbox={self.store} action=navigate_through_poses"
        )

    def _cancel_type(self):
        from fms_msgs.msg import TaskCancel

        return TaskCancel

    def _on_cancel_topic(self, msg) -> None:
        request = cancel_to_dict(msg)
        if request.get("robot_id") != self.robot_id:
            return
        task_id = str(request.get("task_id") or "")
        path = self.store / f"{task_id}.json"
        if path.is_file():
            try:
                stored = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                stored = {}
            stored["executable"] = False
            stored["cancellation"] = {
                "kind": "cancel_ack",
                "task_id": task_id,
                "robot_id": self.robot_id,
                "cancellation_request_id": request.get("cancellation_request_id"),
                "accepted": True,
                "cancelled": True,
                "cancelled_at": datetime.now(timezone.utc).isoformat(),
                "reason": str(request.get("reason") or ""),
                "execute": False,
            }
            path.write_text(json.dumps(stored, indent=2), encoding="utf-8")
        with self._lock:
            if self._active_task_id == task_id and self._goal_handle is not None:
                self.get_logger().info(f"Cancelling Nav2 goal for task_id={task_id}")
                self._goal_handle.cancel_goal_async()

    def _poll_inbox(self) -> None:
        with self._lock:
            if self._active_task_id is not None:
                return
        for path in sorted(self.store.glob("*.json")):
            try:
                stored = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            payload = stored.get("payload") or {}
            if str(payload.get("robot_id") or "") != self.robot_id:
                continue
            if not bool(payload.get("execute")):
                continue
            if stored.get("executable") is False:
                continue
            if stored.get("cancellation"):
                continue
            execution = stored.get("execution") or {}
            if execution.get("status") in TERMINAL:
                continue
            if execution.get("status") in {"accepted", "executing"}:
                continue
            waypoints = payload.get("waypoints") or []
            if not waypoints:
                self._record(path, stored, {
                    "status": "failed",
                    "reason": "no waypoints in payload",
                    "nav2_action": "navigate_through_poses",
                })
                continue
            if not self._action.wait_for_server(timeout_sec=0.1):
                self.get_logger().warning("navigate_through_poses action server not ready")
                return
            self._start_goal(path, stored, payload, waypoints)
            return

    def _start_goal(self, path: Path, stored: dict, payload: dict, waypoints: list) -> None:
        task_id = str(payload.get("task_id") or "")
        poses = [_pose(float(item["x"]), float(item["y"])) for item in waypoints]
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        # Behavior tree may still replan; FMS planned path is the waypoint sequence.
        info = {
            "status": "accepted",
            "reason": "",
            "task_id": task_id,
            "robot_id": self.robot_id,
            "payload_hash": stored.get("payload_hash") or payload.get("payload_hash"),
            "nav2_action": "navigate_through_poses",
            "waypoint_count": len(poses),
            "waypoints": [
                {"waypoint_id": str(item.get("waypoint_id") or ""), "x": float(item["x"]), "y": float(item["y"])}
                for item in waypoints
            ],
            "note": (
                "FMS lattice waypoints are sent in order via NavigateThroughPoses. "
                "Nav2 may smooth/replan between poses; final pose is the last waypoint."
            ),
            "started_at": datetime.now(timezone.utc).isoformat(),
        }
        with self._lock:
            self._active_task_id = task_id
        self._record(path, stored, info)
        self.get_logger().info(
            f"Sending NavigateThroughPoses: task_id={task_id} waypoints={len(poses)}"
        )
        self._send_future = self._action.send_goal_async(goal)
        self._send_future.add_done_callback(
            lambda fut, p=path, s=stored, t=task_id: self._on_goal_response(fut, p, s, t)
        )

    def _on_goal_response(self, future, path: Path, stored: dict, task_id: str) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._record(path, stored, {
                "status": "failed",
                "reason": "Nav2 rejected NavigateThroughPoses goal",
                "nav2_action": "navigate_through_poses",
                "task_id": task_id,
                "finished_at": datetime.now(timezone.utc).isoformat(),
            })
            with self._lock:
                self._active_task_id = None
                self._goal_handle = None
            return
        with self._lock:
            self._goal_handle = handle
        self._record(path, stored, {
            "status": "executing",
            "reason": "Nav2 accepted NavigateThroughPoses",
            "nav2_action": "navigate_through_poses",
            "task_id": task_id,
            "goal_accepted": True,
            "accepted_at": datetime.now(timezone.utc).isoformat(),
        })
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(
            lambda fut, p=path, s=stored, t=task_id: self._on_result(fut, p, s, t)
        )

    def _on_result(self, future, path: Path, stored: dict, task_id: str) -> None:
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:  # noqa: BLE001
            self._record(path, stored, {
                "status": "failed",
                "reason": f"Nav2 result error: {exc}",
                "nav2_action": "navigate_through_poses",
                "task_id": task_id,
                "finished_at": datetime.now(timezone.utc).isoformat(),
            })
            with self._lock:
                self._active_task_id = None
                self._goal_handle = None
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            outcome = "succeeded"
            reason = "NavigateThroughPoses succeeded"
        elif status in {GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_CANCELING}:
            outcome = "cancelled"
            reason = "NavigateThroughPoses cancelled"
        else:
            outcome = "failed"
            reason = f"NavigateThroughPoses finished with status={status}"
        self._record(path, stored, {
            "status": outcome,
            "reason": reason,
            "nav2_action": "navigate_through_poses",
            "nav2_status_code": status,
            "task_id": task_id,
            "goal_accepted": True,
            "finished_at": datetime.now(timezone.utc).isoformat(),
        })
        with self._lock:
            self._active_task_id = None
            self._goal_handle = None

    def _record(self, path: Path, stored: dict, execution: dict) -> None:
        previous = stored.get("execution") if isinstance(stored.get("execution"), dict) else {}
        merged = dict(previous)
        merged.update(execution)
        if previous.get("started_at") and "started_at" not in execution:
            merged["started_at"] = previous["started_at"]
        if previous.get("waypoints") and "waypoints" not in execution:
            merged["waypoints"] = previous["waypoints"]
            merged["waypoint_count"] = previous.get("waypoint_count")
        if previous.get("note") and "note" not in execution:
            merged["note"] = previous["note"]
        stored["execution"] = merged
        try:
            path.write_text(json.dumps(stored, indent=2), encoding="utf-8")
        except OSError as exc:
            self.get_logger().error(f"Failed to update inbox record: {exc}")
        task_id = str(merged.get("task_id") or path.stem)
        write_json_atomic(self.execution_dir / f"{task_id}.json", merged)


def _pose(x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    # Unplanned yaw: identity quaternion; Nav2 may choose final heading.
    pose.pose.orientation.w = 1.0
    if not all(math.isfinite(v) for v in (x, y)):
        raise ValueError("waypoint coordinate is not finite")
    return pose


def main() -> int:
    rclpy.init()
    node = Nav2ExecutorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
