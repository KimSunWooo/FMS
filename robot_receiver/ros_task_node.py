"""Pinky task receiver on one ROS domain.

Stores a validated task and publishes an ACK. It does not call Nav2 or publish
cmd_vel. A node list entry is not receiver_ready; this node publishes that only
while it is the subscriber for its own robot id.
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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from robot_receiver.ros_messages import (
    ack_from_dict,
    cancel_ack_from_dict,
    cancel_to_dict,
    command_to_dict,
    status_message,
)
from robot_receiver.task_codec import TOPIC_ACK, TOPIC_CANCEL, TOPIC_CANCEL_ACK, TOPIC_COMMAND, TOPIC_STATUS, apply_cancel, apply_command


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
        self.store = Path(os.environ.get("FMS_RECEIVER_INBOX") or (Path.home() / "fms_task_receiver" / "inbox"))
        self.last_task_id = ""
        qos = reliable_qos()
        self.create_subscription(self._command_type(), os.environ.get("FMS_TOPIC_TASK_COMMAND", TOPIC_COMMAND), self._on_command, qos)
        self.create_subscription(self._cancel_type(), os.environ.get("FMS_TOPIC_TASK_CANCEL", TOPIC_CANCEL), self._on_cancel, qos)
        self.ack_pub = self.create_publisher(self._ack_type(), os.environ.get("FMS_TOPIC_TASK_ACK", TOPIC_ACK), qos)
        self.cancel_ack_pub = self.create_publisher(self._cancel_ack_type(), os.environ.get("FMS_TOPIC_TASK_CANCEL_ACK", TOPIC_CANCEL_ACK), qos)
        self.status_pub = self.create_publisher(self._status_type(), os.environ.get("FMS_TOPIC_ROBOT_STATUS", TOPIC_STATUS), qos)
        self.create_timer(1.0, self._publish_status)

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
        body = apply_command(self.store, command_to_dict(msg), expected_robot_id=self.robot_id)
        if body.get("accepted"):
            self.last_task_id = body["task_id"]
        self.ack_pub.publish(ack_from_dict(body))

    def _on_cancel(self, msg) -> None:
        body = apply_cancel(self.store, cancel_to_dict(msg), expected_robot_id=self.robot_id)
        self.cancel_ack_pub.publish(cancel_ack_from_dict(body))

    def _publish_status(self) -> None:
        msg = status_message(self.robot_id, self.domain_id, ready=True, last_task_id=self.last_task_id)
        now = self.get_clock().now().to_msg()
        msg.stamp = now
        self.status_pub.publish(msg)
        path = self.store.parent / "robot_status.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({
            "robot_id": self.robot_id,
            "ros_domain_id": self.domain_id,
            "receiver_ready": True,
            "task_execution_enabled": False,
            "last_received_task_id": self.last_task_id,
            "stamp": datetime.now(timezone.utc).isoformat(),
        }), encoding="utf-8")


def main() -> int:
    rclpy.init()
    node = TaskReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
