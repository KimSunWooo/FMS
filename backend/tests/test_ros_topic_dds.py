"""Local DDS exchange on an isolated domain. This is not a Pinky-1 ACK."""
from __future__ import annotations

import json
import os
import subprocess
import uuid
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
INSTALL = ROOT / "ros_ws" / "install" / "setup.bash"


pytestmark = pytest.mark.skipif(not INSTALL.is_file(), reason="fms_msgs is not built")


def _exchange(domain: int, inbox: Path, payload: dict, *, gateway_domain: int | None = None, cancel: bool = False) -> dict:
    robot_id = "pinky-test"
    command_id = str(uuid.uuid4())
    data = inbox.parent / f"ipc-{domain}-{gateway_domain or domain}"
    env = os.environ.copy()
    env.update({
        "ROS_DOMAIN_ID": str(domain),
        "FMS_RECEIVER_ROBOT_ID": robot_id,
        "FMS_RECEIVER_INBOX": str(inbox),
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    })
    env.pop("ROS_LOCALHOST_ONLY", None)
    receiver = subprocess.Popen(
        ["/bin/bash", str(ROOT / "scripts" / "run_task_receiver.sh")],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    gateway_env = env.copy()
    gateway_env["ROS_DOMAIN_ID"] = str(gateway_domain or domain)
    gateway_env["FMS_WORKER_ROBOT_ID"] = robot_id
    gateway_env["FMS_DATA_DIR"] = str(data)
    gateway_env["FMS_ROS_DISCOVERY_TIMEOUT_S"] = "8"
    gateway = subprocess.Popen(
        ["/bin/bash", str(ROOT / "scripts" / "run_ros_gateway.sh")],
        env=gateway_env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    from robot_receiver.task_codec import command_message
    message = command_message(payload, command_id)
    command_dir = data / "ros_ipc" / robot_id / "commands"
    command_dir.mkdir(parents=True, exist_ok=True)
    (command_dir / f"{command_id}.json").write_text(json.dumps({
        "kind": "task_command",
        "robot_id": robot_id,
        "ros_domain_id": int(gateway_domain or domain),
        "command_id": command_id,
        "message": message,
    }), encoding="utf-8")
    result_path = data / "ros_ipc" / robot_id / "results" / f"{command_id}.json"
    try:
        deadline = time_after(25)
        result = {"accepted": False, "status": "timeout", "reason": "no DDS ack"}
        while time_after(0) < deadline:
            if result_path.is_file():
                result = json.loads(result_path.read_text(encoding="utf-8"))
                break
            if receiver.poll() not in (None, 0) or gateway.poll() not in (None, 0):
                break
            time_sleep(0.1)
        if not cancel or not result.get("accepted"):
            return result
        cancel_id = "cancel-dds-1"
        (command_dir / f"{cancel_id}.json").write_text(json.dumps({
            "kind": "task_cancel",
            "robot_id": robot_id,
            "ros_domain_id": int(gateway_domain or domain),
            "command_id": cancel_id,
            "message": {
                "task_id": payload["task_id"],
                "robot_id": robot_id,
                "cancellation_request_id": cancel_id,
                "reason": "operator",
                "execute": False,
            },
        }), encoding="utf-8")
        cancel_path = data / "ros_ipc" / robot_id / "results" / f"{cancel_id}.json"
        deadline = time_after(15)
        while time_after(0) < deadline:
            if cancel_path.is_file():
                result["cancel"] = json.loads(cancel_path.read_text(encoding="utf-8"))
                return result
            time_sleep(0.1)
        result["cancel"] = {"accepted": False, "status": "timeout", "reason": "no cancel ack"}
        return result
    finally:
        for proc in (gateway, receiver):
            if proc.poll() is None:
                proc.terminate()
        for proc in (gateway, receiver):
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()


def time_after(seconds: float) -> float:
    import time
    return time.monotonic() + seconds


def time_sleep(seconds: float) -> None:
    import time
    time.sleep(seconds)


def _payload() -> dict:
    from robot_receiver.pinky_task_receiver import payload_hash
    body = {
        "task_id": "dds-task",
        "robot_id": "pinky-test",
        "task_type": "navigate",
        "map_id": "1f_map_f",
        "map_revision": "rev",
        "start": {"x": 0.2, "y": 0.2},
        "goal": {"x": 0.4, "y": 0.4, "label": ""},
        "waypoints": [{"x": 0.2, "y": 0.2, "waypoint_id": "a"}],
        "execute": False,
    }
    body["payload_hash"] = payload_hash(body)
    return body


def test_domain_80_command_receives_an_ack(tmp_path):
    result = _exchange(80, tmp_path / "inbox", _payload())
    assert result.get("accepted") is True, result
    assert result.get("receiver_id") == "pinky-fms-receiver"
    stored = json.loads((tmp_path / "inbox" / "dds-task.json").read_text(encoding="utf-8"))
    assert stored["payload"]["execute"] is False


def test_domain_81_does_not_ack_a_domain_80_receiver(tmp_path):
    result = _exchange(80, tmp_path / "inbox", _payload(), gateway_domain=81)
    assert result.get("accepted") is False
    assert not (tmp_path / "inbox" / "dds-task.json").exists()


def test_domain_82_cancel_ack(tmp_path):
    inbox = tmp_path / "inbox"
    stored = _exchange(82, inbox, _payload(), cancel=True)
    assert stored.get("accepted") is True, stored
    assert stored.get("cancel", {}).get("accepted") is True, stored
    assert stored["cancel"].get("outcome") == "cancel_ack"
    body = json.loads((inbox / "dds-task.json").read_text(encoding="utf-8"))
    assert body["executable"] is False
    assert body["payload_hash"] == stored["payload_hash"]


def test_gateway_times_out_when_no_receiver_is_subscribed(tmp_path):
    data = tmp_path / "ipc-offline"
    env = os.environ.copy()
    env.update({
        "ROS_DOMAIN_ID": "83",
        "FMS_WORKER_ROBOT_ID": "pinky-test",
        "FMS_DATA_DIR": str(data),
        "FMS_ROS_DISCOVERY_TIMEOUT_S": "3",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    })
    gateway = subprocess.Popen(
        ["/bin/bash", str(ROOT / "scripts" / "run_ros_gateway.sh")],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, start_new_session=True,
    )
    missing = str(uuid.uuid4())
    folder = data / "ros_ipc" / "pinky-test" / "commands"
    folder.mkdir(parents=True, exist_ok=True)
    (folder / f"{missing}.json").write_text(json.dumps({
        "kind": "task_command",
        "robot_id": "pinky-test",
        "ros_domain_id": 83,
        "command_id": missing,
        "message": command_message_for(missing),
    }), encoding="utf-8")
    missing_result = data / "ros_ipc" / "pinky-test" / "results" / f"{missing}.json"
    try:
        deadline = time_after(20)
        result = {"status": "missing"}
        while time_after(0) < deadline:
            if missing_result.is_file():
                result = json.loads(missing_result.read_text(encoding="utf-8"))
                break
            if gateway.poll() not in (None, 0):
                break
            time_sleep(0.1)
        assert result.get("status") == "timeout", result
        assert "not visible" in result.get("reason", "")
    finally:
        if gateway.poll() is None:
            gateway.terminate()
            try:
                gateway.wait(timeout=3)
            except subprocess.TimeoutExpired:
                gateway.kill()


def test_rewritten_command_is_published_again_on_an_isolated_domain(tmp_path):
    """Lost ACK: the same command file is published twice. Not domain 24."""
    data = tmp_path / "ipc-republish"
    count_path = tmp_path / "count.txt"
    count_path.write_text("0", encoding="utf-8")
    script = tmp_path / "count_commands.py"
    script.write_text(
        "import os\n"
        "from pathlib import Path\n"
        "import rclpy\n"
        "from rclpy.node import Node\n"
        "from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy\n"
        "from fms_msgs.msg import TaskCommand\n"
        "class Counter(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('fms_command_counter')\n"
        "        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)\n"
        "        self.n = 0\n"
        "        self.out = Path(os.environ['FMS_COUNT_FILE'])\n"
        "        self.create_subscription(TaskCommand, '/fms/task/command', self._on, qos)\n"
        "        self.create_timer(0.2, self._flush)\n"
        "    def _on(self, msg):\n"
        "        if msg.robot_id == 'pinky-test' and msg.execute is False:\n"
        "            self.n += 1\n"
        "    def _flush(self):\n"
        "        self.out.write_text(str(self.n), encoding='utf-8')\n"
        "rclpy.init()\n"
        "node = Counter()\n"
        "try:\n"
        "    rclpy.spin(node)\n"
        "finally:\n"
        "    node.destroy_node()\n"
        "    rclpy.shutdown()\n",
        encoding="utf-8",
    )
    env = os.environ.copy()
    env.update({
        "ROS_DOMAIN_ID": "84",
        "FMS_WORKER_ROBOT_ID": "pinky-test",
        "FMS_DATA_DIR": str(data),
        "FMS_ROS_DISCOVERY_TIMEOUT_S": "8",
        "FMS_COUNT_FILE": str(count_path),
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    })
    counter = subprocess.Popen(
        ["/bin/bash", "-lc", f"source /opt/ros/jazzy/setup.bash && source {ROOT}/ros_ws/install/setup.bash && exec python3 {script}"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, start_new_session=True,
    )
    gateway = subprocess.Popen(
        ["/bin/bash", str(ROOT / "scripts" / "run_ros_gateway.sh")],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, start_new_session=True,
    )
    command_id = "republish-1"
    folder = data / "ros_ipc" / "pinky-test" / "commands"
    folder.mkdir(parents=True, exist_ok=True)
    payload = command_message_for(command_id)
    body = json.dumps({
        "kind": "task_command",
        "robot_id": "pinky-test",
        "ros_domain_id": 84,
        "command_id": command_id,
        "message": payload,
    })
    target = folder / f"{command_id}.json"
    try:
        time_sleep(1.5)
        target.write_text(body, encoding="utf-8")
        deadline = time_after(15)
        while time_after(0) < deadline and count_path.read_text(encoding="utf-8").strip() == "0":
            time_sleep(0.1)
        assert count_path.read_text(encoding="utf-8").strip() != "0", "first publish was not observed"
        time_sleep(0.3)
        target.write_text(body, encoding="utf-8")
        deadline = time_after(10)
        while time_after(0) < deadline and int(count_path.read_text(encoding="utf-8") or "0") < 2:
            time_sleep(0.1)
        assert int(count_path.read_text(encoding="utf-8") or "0") >= 2
    finally:
        for proc in (gateway, counter):
            if proc.poll() is None:
                proc.terminate()
        for proc in (gateway, counter):
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()


def command_message_for(command_id: str) -> dict:
    from robot_receiver.task_codec import command_message
    payload = _payload()
    payload["task_id"] = "offline-task"
    from robot_receiver.pinky_task_receiver import payload_hash
    payload.pop("payload_hash")
    payload["payload_hash"] = payload_hash(payload)
    return command_message(payload, command_id)
