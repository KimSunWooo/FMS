"""Domain gateway. Publishes task commands and writes the receiver ACK to disk.

This process owns one ROS_DOMAIN_ID. It does not treat a successful publish as
delivery, and it does not publish cmd_vel or a Nav2 goal.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from robot_receiver.ros_messages import (
    ack_to_dict,
    cancel_ack_to_dict,
    cancel_from_dict,
    command_from_dict,
)
from robot_receiver.task_codec import TOPIC_ACK, TOPIC_CANCEL, TOPIC_CANCEL_ACK, TOPIC_COMMAND, TOPIC_STATUS


def reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class GatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("fms_task_gateway")
        self.robot_id = os.environ["FMS_WORKER_ROBOT_ID"]
        self.domain_id = int(os.environ["ROS_DOMAIN_ID"])
        self.root = Path(os.environ.get("FMS_DATA_DIR") or (ROOT / "data")) / "ros_ipc" / self.robot_id
        (self.root / "commands").mkdir(parents=True, exist_ok=True)
        (self.root / "results").mkdir(parents=True, exist_ok=True)
        self.discovery_s = float(os.environ.get("FMS_ROS_DISCOVERY_TIMEOUT_S", "3"))
        self.pending: dict[str, dict] = {}
        qos = reliable_qos()
        self.command_pub = self.create_publisher(self._type("TaskCommand"), os.environ.get("FMS_TOPIC_TASK_COMMAND", TOPIC_COMMAND), qos)
        self.cancel_pub = self.create_publisher(self._type("TaskCancel"), os.environ.get("FMS_TOPIC_TASK_CANCEL", TOPIC_CANCEL), qos)
        self.create_subscription(self._type("TaskAck"), os.environ.get("FMS_TOPIC_TASK_ACK", TOPIC_ACK), self._on_ack, qos)
        self.create_subscription(self._type("TaskCancelAck"), os.environ.get("FMS_TOPIC_TASK_CANCEL_ACK", TOPIC_CANCEL_ACK), self._on_cancel_ack, qos)
        self.create_subscription(self._type("RobotStatus"), os.environ.get("FMS_TOPIC_ROBOT_STATUS", TOPIC_STATUS), self._on_status, qos)
        self.create_timer(0.2, self._pump)

    def _type(self, name: str):
        from fms_msgs import msg
        return getattr(msg, name)

    def _pump(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        for path in sorted((self.root / "commands").glob("*.json")):
            try:
                request = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            command_id = str(request.get("command_id") or path.stem)
            if int(request.get("ros_domain_id", -1)) != self.domain_id or request.get("robot_id") != self.robot_id:
                self._finish(command_id, {
                    "accepted": False,
                    "status": "robot_mismatch",
                    "reason": "command domain does not match this gateway",
                    "command_id": command_id,
                })
                path.unlink(missing_ok=True)
                continue
            slot = self.pending.get(command_id)
            if slot is None:
                self.pending[command_id] = {"request": request, "since": now, "published": False, "path": path}
                slot = self.pending[command_id]
            publisher = self.command_pub if request.get("kind") == "task_command" else self.cancel_pub
            if publisher.get_subscription_count() < 1:
                if now - slot["since"] > self.discovery_s:
                    self._finish(command_id, {
                        "accepted": False,
                        "status": "timeout",
                        "reason": "Receiver subscriber is not visible on this domain",
                        "command_id": command_id,
                    })
                    path.unlink(missing_ok=True)
                    self.pending.pop(command_id, None)
                continue
            if not slot["published"]:
                message = request["message"]
                if request.get("kind") == "task_command":
                    message = dict(message)
                    message["execute"] = False
                    publisher.publish(command_from_dict(message))
                else:
                    message = dict(message)
                    message["execute"] = False
                    publisher.publish(cancel_from_dict(message))
                slot["published"] = True

    def _on_ack(self, msg) -> None:
        if msg.robot_id != self.robot_id:
            return
        body = ack_to_dict(msg)
        self._finish(msg.command_id, body)

    def _on_cancel_ack(self, msg) -> None:
        if msg.robot_id != self.robot_id:
            return
        body = cancel_ack_to_dict(msg)
        self._finish(msg.cancellation_request_id, body)

    def _on_status(self, msg) -> None:
        if msg.robot_id != self.robot_id or int(msg.ros_domain_id) != self.domain_id:
            return
        (self.root / "robot_status.json").write_text(json.dumps({
            "robot_id": msg.robot_id,
            "ros_domain_id": int(msg.ros_domain_id),
            "receiver_ready": bool(msg.receiver_ready),
            "task_execution_enabled": False,
            "last_received_task_id": msg.last_received_task_id,
        }), encoding="utf-8")

    def _finish(self, command_id: str, body: dict) -> None:
        path = self.root / "results" / f"{command_id}.json"
        path.write_text(json.dumps(body), encoding="utf-8")
        slot = self.pending.pop(command_id, None)
        if slot is not None:
            Path(slot["path"]).unlink(missing_ok=True)

def main() -> int:
    rclpy.init()
    node = GatewayNode()
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
