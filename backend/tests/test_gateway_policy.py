"""Gateway republish rules and stale RobotStatus. Not a Pinky-1 ACK."""
from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone

from app.core.config import Settings
from app.services.domain_worker import _receiver_ready
from robot_receiver.gateway_policy import publish_decision
from robot_receiver.task_codec import QOS, TOPIC_ACK, TOPIC_CANCEL, TOPIC_CANCEL_ACK, TOPIC_COMMAND, TOPIC_STATUS


def test_topic_defaults_match_the_five_message_contract():
    fields = Settings.model_fields
    assert fields["ros_task_command_topic"].default == TOPIC_COMMAND == "/fms/task/command"
    assert fields["ros_task_ack_topic"].default == TOPIC_ACK == "/fms/task/ack"
    assert fields["ros_task_cancel_topic"].default == TOPIC_CANCEL == "/fms/task/cancel"
    assert fields["ros_task_cancel_ack_topic"].default == TOPIC_CANCEL_ACK == "/fms/task/cancel_ack"
    assert fields["ros_robot_status_topic"].default == TOPIC_STATUS == "/fms/robot/status"
    assert fields["fleet_dispatch_mode"].default == "ros_topic"
    assert QOS == {"reliability": "reliable", "durability": "volatile", "history": "keep_last", "depth": 10}


def test_republish_after_a_rewritten_command_or_a_returned_subscriber():
    waiting = dict(published=True, published_mtime=1, file_mtime=1, subscribers=1, age_s=1.0, discovery_s=3.0)
    assert publish_decision(**waiting) == "wait"
    rewritten = dict(waiting, file_mtime=2)
    assert publish_decision(**rewritten) == "publish"
    lost = dict(published=False, published_mtime=1, file_mtime=1, subscribers=1, age_s=2.0, discovery_s=3.0)
    assert publish_decision(**lost) == "publish"
    quiet = dict(published=False, published_mtime=None, file_mtime=1, subscribers=0, age_s=1.0, discovery_s=3.0)
    assert publish_decision(**quiet) == "wait"
    absent = dict(quiet, age_s=3.1)
    assert publish_decision(**absent) == "timeout"


def test_stale_or_unsigned_robot_status_is_not_ready(tmp_path):
    path = tmp_path / "ros_ipc" / "pinky-1" / "robot_status.json"
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({
        "robot_id": "pinky-1",
        "receiver_ready": True,
        "task_execution_enabled": False,
    }), encoding="utf-8")
    assert _receiver_ready(tmp_path, "pinky-1") is False
    old = (datetime.now(timezone.utc) - timedelta(seconds=30)).isoformat()
    path.write_text(json.dumps({
        "robot_id": "pinky-1",
        "receiver_ready": True,
        "task_execution_enabled": False,
        "observed_at": old,
    }), encoding="utf-8")
    assert _receiver_ready(tmp_path, "pinky-1") is False
    path.write_text(json.dumps({
        "robot_id": "pinky-1",
        "receiver_ready": True,
        "task_execution_enabled": False,
        "observed_at": datetime.now(timezone.utc).isoformat(),
    }), encoding="utf-8")
    assert _receiver_ready(tmp_path, "pinky-1") is True
    enabled = json.loads(path.read_text(encoding="utf-8"))
    enabled["task_execution_enabled"] = True
    path.write_text(json.dumps(enabled), encoding="utf-8")
    assert _receiver_ready(tmp_path, "pinky-1") is False
