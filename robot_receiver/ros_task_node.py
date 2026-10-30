"""Pinky task receiver on one ROS domain.

Stores a validated task and publishes an ACK. It does not call Nav2 or publish
cmd_vel. A node list entry is not receiver_ready; this node publishes that only
while it is the subscriber for its own robot id.

RobotStatus includes map→base_footprint TF pose when available. TF failure does
not break command/ACK handling and never invents a location.
"""
from __future__ import annotations

import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from robot_receiver.battery_adapter import BatteryTelemetryCache
from robot_receiver.battery_report import DEFAULT_BATTERY_MAX_AGE_S
from robot_receiver.pose_report import DEFAULT_POSE_MAX_AGE_S, POSE_SOURCE_TF, evaluate_pose_sample
from robot_receiver.ros_messages import (
    ack_from_dict,
    cancel_ack_from_dict,
    cancel_to_dict,
    command_to_dict,
    status_message,
)
from robot_receiver.task_codec import (
    TOPIC_ACK,
    TOPIC_CANCEL,
    TOPIC_CANCEL_ACK,
    TOPIC_COMMAND,
    TOPIC_STATUS,
    apply_cancel,
    apply_command,
)


def reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class TaskReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("fms_task_receiver")
        self.robot_id = os.environ["FMS_RECEIVER_ROBOT_ID"]
        self.domain_id = int(os.environ["ROS_DOMAIN_ID"])
        data_dir = Path(os.environ.get("FMS_DATA_DIR") or (ROOT / "data"))
        self.store = Path(
            os.environ.get("FMS_RECEIVER_INBOX")
            or (data_dir / "ros_ipc" / self.robot_id / "inbox")
        )
        self.audit_log = Path(
            os.environ.get("FMS_RECEIVER_AUDIT_LOG")
            or (self.store.parent / "events.jsonl")
        )
        self.audit_log.parent.mkdir(parents=True, exist_ok=True)
        self._audit_write_failed = False
        self._last_audited_status: tuple | None = None
        self.last_task_id = ""
        self._pose_max_age_s = float(os.environ.get("FMS_POSE_MAX_AGE_S", str(DEFAULT_POSE_MAX_AGE_S)))
        self._last_pose_reason = "missing"
        self._battery = BatteryTelemetryCache(
            max_age_s=float(os.environ.get("FMS_BATTERY_MAX_AGE_S", str(DEFAULT_BATTERY_MAX_AGE_S))),
        )
        qos = reliable_qos()
        self.create_subscription(
            self._command_type(),
            os.environ.get("FMS_TOPIC_TASK_COMMAND", TOPIC_COMMAND),
            self._on_command,
            qos,
        )
        self.create_subscription(
            self._cancel_type(),
            os.environ.get("FMS_TOPIC_TASK_CANCEL", TOPIC_CANCEL),
            self._on_cancel,
            qos,
        )
        self.ack_pub = self.create_publisher(
            self._ack_type(), os.environ.get("FMS_TOPIC_TASK_ACK", TOPIC_ACK), qos
        )
        self.cancel_ack_pub = self.create_publisher(
            self._cancel_ack_type(),
            os.environ.get("FMS_TOPIC_TASK_CANCEL_ACK", TOPIC_CANCEL_ACK),
            qos,
        )
        self.status_pub = self.create_publisher(
            self._status_type(),
            os.environ.get("FMS_TOPIC_ROBOT_STATUS", TOPIC_STATUS),
            qos,
        )
        self._init_battery_subscriptions(qos)
        self._tf_buffer = None
        self._tf_listener = None
        self._init_tf()
        self.create_timer(1.0, self._publish_status)
        self._audit(
            event="receiver_started",
            direction="internal",
            topic="",
            message_type="",
            data={"inbox": str(self.store), "audit_log": str(self.audit_log)},
        )
        self.get_logger().info(
            f"FMS task receiver started: robot_id={self.robot_id}, "
            f"domain_id={self.domain_id}, audit_log={self.audit_log}"
        )

    def _init_tf(self) -> None:
        try:
            from tf2_ros import Buffer, TransformListener

            self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
            self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        except Exception as exc:  # noqa: BLE001 — keep receiver alive without TF
            self.get_logger().warning(f"TF listener unavailable: {exc}")
            self._tf_buffer = None
            self._tf_listener = None

    def _init_battery_subscriptions(self, qos) -> None:
        """Subscribe to existing robot battery topics. Failure must not block tasks."""
        # Canonical battery topic only. Do not republish as a new FMS topic.
        percent_topic = os.environ.get("FMS_TOPIC_BATTERY_PERCENT", "/battery/percent")
        try:
            from std_msgs.msg import Float32

            self.create_subscription(Float32, percent_topic, self._on_battery_percent, qos)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"{percent_topic} subscription unavailable: {exc}")

    def _on_battery_percent(self, msg) -> None:
        try:
            self._battery.update_percent_float32(float(msg.data), source="topic:battery/percent")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"battery percent ignored: {exc}")

    def _command_type(self):
        from fms_msgs.msg import TaskCommand

        return TaskCommand

    def _ack_type(self):
        from fms_msgs.msg import TaskAck

        return TaskAck

    def _cancel_type(self):
        from fms_msgs.msg import TaskCancel

        return TaskCancel

    def _cancel_ack_type(self):
        from fms_msgs.msg import TaskCancelAck

        return TaskCancelAck

    def _status_type(self):
        from fms_msgs.msg import RobotStatus

        return RobotStatus

    def _on_command(self, msg) -> None:
        command = command_to_dict(msg)
        command_topic = os.environ.get("FMS_TOPIC_TASK_COMMAND", TOPIC_COMMAND)
        ack_topic = os.environ.get("FMS_TOPIC_TASK_ACK", TOPIC_ACK)
        self._audit(
            event="task_command_received",
            direction="received",
            topic=command_topic,
            message_type="fms_msgs/msg/TaskCommand",
            data=command,
        )
        self.get_logger().info(
            f"Received TaskCommand: task_id={command['task_id']}, "
            f"robot_id={command['robot_id']}, waypoints={len(command['waypoints'])}"
        )
        body = apply_command(self.store, command, expected_robot_id=self.robot_id)
        if body.get("accepted"):
            self.last_task_id = body["task_id"]
        self.ack_pub.publish(ack_from_dict(body))
        self._audit(
            event="task_ack_published",
            direction="sent",
            topic=ack_topic,
            message_type="fms_msgs/msg/TaskAck",
            data=body,
        )
        self.get_logger().info(
            f"Published TaskAck: task_id={body.get('task_id', '')}, "
            f"accepted={bool(body.get('accepted'))}, status={body.get('status', '')}, "
            f"reason={body.get('reason', '')}"
        )

    def _on_cancel(self, msg) -> None:
        cancel_request = cancel_to_dict(msg)
        cancel_topic = os.environ.get("FMS_TOPIC_TASK_CANCEL", TOPIC_CANCEL)
        cancel_ack_topic = os.environ.get("FMS_TOPIC_TASK_CANCEL_ACK", TOPIC_CANCEL_ACK)
        self._audit(
            event="task_cancel_received",
            direction="received",
            topic=cancel_topic,
            message_type="fms_msgs/msg/TaskCancel",
            data=cancel_request,
        )
        self.get_logger().info(
            f"Received TaskCancel: task_id={cancel_request['task_id']}, "
            f"robot_id={cancel_request['robot_id']}, "
            f"request_id={cancel_request['cancellation_request_id']}"
        )
        body = apply_cancel(self.store, cancel_request, expected_robot_id=self.robot_id)
        self.cancel_ack_pub.publish(cancel_ack_from_dict(body))
        self._audit(
            event="task_cancel_ack_published",
            direction="sent",
            topic=cancel_ack_topic,
            message_type="fms_msgs/msg/TaskCancelAck",
            data=body,
        )
        self.get_logger().info(
            f"Published TaskCancelAck: task_id={body.get('task_id', '')}, "
            f"accepted={bool(body.get('accepted'))}, outcome={body.get('outcome', '')}, "
            f"reason={body.get('reason', '')}"
        )

    def _lookup_pose_report(self) -> dict | None:
        """Non-blocking TF lookup. Never fabricates map coordinates."""
        if self._tf_buffer is None:
            self._last_pose_reason = "lookup_failed"
            return {"pose_valid": False, "reason": "lookup_failed"}
        try:
            if not self._tf_buffer.can_transform(
                "map",
                "base_footprint",
                Time(),
                timeout=Duration(seconds=0.0),
            ):
                self._last_pose_reason = "lookup_failed"
                return {"pose_valid": False, "reason": "lookup_failed"}
            transform = self._tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except Exception as exc:  # noqa: BLE001
            self._last_pose_reason = "lookup_failed"
            self.get_logger().debug(f"TF map->base_footprint unavailable: {exc}")
            return {"pose_valid": False, "reason": "lookup_failed"}

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp
        try:
            ros_now = self.get_clock().now()
            tf_time = Time.from_msg(stamp)
            age_ros = (ros_now - tf_time).nanoseconds / 1e9
            if age_ros < -1.0 or age_ros > self._pose_max_age_s:
                self._last_pose_reason = "stale"
                return {"pose_valid": False, "reason": "stale", "age_s": age_ros}
            observed_at = datetime.fromtimestamp(
                stamp.sec + stamp.nanosec * 1e-9,
                tz=timezone.utc,
            )
        except Exception:  # noqa: BLE001
            observed_at = datetime.now(timezone.utc)

        report = evaluate_pose_sample(
            frame_id=str(transform.header.frame_id or "map"),
            x=float(translation.x),
            y=float(translation.y),
            z=float(translation.z),
            qx=float(rotation.x),
            qy=float(rotation.y),
            qz=float(rotation.z),
            qw=float(rotation.w),
            observed_at=observed_at,
            require_map_bounds=True,
            age_basis="ros",
        )
        if report.get("pose_valid"):
            report["pose_source"] = POSE_SOURCE_TF
            report["tf_stamp"] = stamp
            self._last_pose_reason = "ok"
        else:
            self._last_pose_reason = str(report.get("reason") or "invalid_data")
        return report

    def _publish_status(self) -> None:
        pose_report = self._lookup_pose_report()
        valid = bool(pose_report and pose_report.get("pose_valid"))
        battery_report = self._battery.snapshot()
        battery_valid = bool(battery_report and battery_report.get("battery_valid"))
        msg = status_message(
            self.robot_id,
            self.domain_id,
            ready=True,
            last_task_id=self.last_task_id,
            pose_report=pose_report if valid else None,
            battery_report=battery_report if battery_valid else None,
        )
        msg.stamp = self.get_clock().now().to_msg()
        if not valid:
            msg.pose_valid = False
            msg.pose_source = ""
        if not battery_valid:
            msg.battery_valid = False
            msg.battery_source = ""
            msg.battery_percentage = 0.0
        self.status_pub.publish(msg)

        status_data = {
            "robot_id": msg.robot_id,
            "ros_domain_id": int(msg.ros_domain_id),
            "receiver_ready": bool(msg.receiver_ready),
            "task_execution_enabled": bool(msg.task_execution_enabled),
            "active_task_id": msg.active_task_id,
            "last_received_task_id": msg.last_received_task_id,
            "stamp": {"sec": int(msg.stamp.sec), "nanosec": int(msg.stamp.nanosec)},
            "pose_valid": bool(msg.pose_valid),
            "pose_source": msg.pose_source,
            "pose_reason": self._last_pose_reason,
            "battery_valid": bool(msg.battery_valid),
            "battery_percentage": float(msg.battery_percentage) if msg.battery_valid else None,
            "battery_source": msg.battery_source,
            "battery_reason": self._battery.last_reason,
        }
        if valid and pose_report:
            status_data["pose"] = {
                "x": pose_report["x"],
                "y": pose_report["y"],
                "z": pose_report["z"],
                "yaw": pose_report["yaw"],
                "frame_id": "map",
            }
        status_signature = (
            bool(msg.receiver_ready),
            msg.last_received_task_id,
            bool(msg.pose_valid),
            self._last_pose_reason,
            bool(msg.battery_valid),
            self._battery.last_reason,
        )
        if status_signature != self._last_audited_status:
            self._audit(
                event="robot_status_published",
                direction="sent",
                topic=os.environ.get("FMS_TOPIC_ROBOT_STATUS", TOPIC_STATUS),
                message_type="fms_msgs/msg/RobotStatus",
                data=status_data,
            )
            self._last_audited_status = status_signature

        path = self.store.parent / "robot_status.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "robot_id": self.robot_id,
            "ros_domain_id": self.domain_id,
            "receiver_ready": True,
            "task_execution_enabled": bool(msg.task_execution_enabled),
            "last_received_task_id": self.last_task_id,
            "stamp": datetime.now(timezone.utc).isoformat(),
            "pose_valid": valid,
            "pose_source": (pose_report or {}).get("pose_source") or "",
            "pose_reason": self._last_pose_reason,
            "pose": None,
            "pose_observed_at": None,
            "battery_valid": battery_valid,
            "battery_percentage": battery_report.get("battery_percentage") if battery_valid else None,
            "battery_source": battery_report.get("battery_source") if battery_valid else "",
            "battery_reason": self._battery.last_reason,
            "battery_observed_at": battery_report.get("battery_observed_at") if battery_valid else None,
        }
        if valid and pose_report:
            payload["pose"] = {
                "x": pose_report["x"],
                "y": pose_report["y"],
                "z": pose_report["z"],
                "yaw": pose_report["yaw"],
                "orientation": pose_report["orientation"],
                "frame_id": "map",
            }
            payload["pose_observed_at"] = pose_report.get("pose_observed_at")
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(payload), encoding="utf-8")
        temporary.replace(path)

    def _audit(
        self,
        *,
        event: str,
        direction: str,
        topic: str,
        message_type: str,
        data: dict,
    ) -> None:
        """Append one complete receive/send event as a single JSONL record."""
        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "event": event,
            "direction": direction,
            "node": self.get_name(),
            "robot_id": self.robot_id,
            "ros_domain_id": self.domain_id,
            "topic": topic,
            "message_type": message_type,
            "data": data,
        }
        try:
            with self.audit_log.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
            self._audit_write_failed = False
        except OSError as exc:
            if not self._audit_write_failed:
                self.get_logger().error(f"Failed to write FMS audit log {self.audit_log}: {exc}")
                self._audit_write_failed = True


def main() -> int:
    rclpy.init()
    node = TaskReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
