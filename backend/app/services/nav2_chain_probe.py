#!/usr/bin/env python3
"""Read-only sample of the Pinky velocity chain and StopZone scan.

Subscribes only. Does not publish Twist, call an action, or change a parameter.
"""
from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

STOP_RADIUS_M = 0.18
MIN_POINTS = 4
SAMPLE_SECONDS = 2.0


def main() -> int:
    rclpy.init()
    node = Node("fms_velocity_chain_probe")
    observed = {
        "/cmd_vel_nav": {"seen": False, "max_linear_x": None, "last_stamp": None},
        "/cmd_vel_smoothed": {"seen": False, "max_linear_x": None, "last_stamp": None},
        "/cmd_vel": {"seen": False, "max_linear_x": None, "last_stamp": None},
    }
    scan = {
        "received": False,
        "frame_id": None,
        "stamp": None,
        "min_range_m": None,
        "points_within_stop_radius": 0,
        "stop_radius_m": STOP_RADIUS_M,
        "min_points": MIN_POINTS,
    }

    def on_twist(topic):
        def callback(message: Twist):
            row = observed[topic]
            value = float(message.linear.x)
            row["seen"] = True
            row["max_linear_x"] = value if row["max_linear_x"] is None else max(row["max_linear_x"], value)
            row["last_stamp"] = node.get_clock().now().nanoseconds / 1e9
        return callback

    def on_scan(message: LaserScan):
        finite = []
        sectors = {"front": 0, "left": 0, "rear": 0, "right": 0}
        for index, item in enumerate(message.ranges):
            if not math.isfinite(item) or item <= 0.0:
                continue
            value = float(item)
            finite.append(value)
            if value >= STOP_RADIUS_M:
                continue
            angle = message.angle_min + index * message.angle_increment
            wrapped = math.atan2(math.sin(angle), math.cos(angle))
            if abs(wrapped) <= math.pi / 4:
                sectors["front"] += 1
            elif wrapped > math.pi / 4 and wrapped < 3 * math.pi / 4:
                sectors["left"] += 1
            elif abs(wrapped) >= 3 * math.pi / 4:
                sectors["rear"] += 1
            else:
                sectors["right"] += 1
        scan["received"] = True
        scan["frame_id"] = message.header.frame_id
        scan["stamp"] = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        scan["min_range_m"] = min(finite) if finite else None
        scan["points_within_stop_radius"] = sum(sectors.values())
        scan["inside_sectors_laser_frame"] = sectors

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    for topic in observed:
        node.create_subscription(Twist, topic, on_twist(topic), qos)
    node.create_subscription(LaserScan, "/scan", on_scan, scan_qos)
    end = time.monotonic() + SAMPLE_SECONDS
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    nav = observed["/cmd_vel_nav"]["max_linear_x"]
    smoothed = observed["/cmd_vel_smoothed"]["max_linear_x"]
    cmd = observed["/cmd_vel"]["max_linear_x"]
    cmd_seen = observed["/cmd_vel"]["seen"]
    case = None
    if nav is not None and nav > 1e-4 and smoothed is not None and smoothed > 1e-4:
        if not cmd_seen:
            case = "B"
        elif cmd is None or cmd <= 1e-4:
            case = "A"
        else:
            case = "C"
    blocked = bool(scan["received"] and scan["points_within_stop_radius"] >= MIN_POINTS)
    print(json.dumps({
        "motion_command_sent": False,
        "sample_seconds": SAMPLE_SECONDS,
        "topics": observed,
        "scan": scan,
        "velocity_case": case,
        "stop_zone_would_stop": blocked,
        "detail": (
            "StopZone has enough returns inside 0.18m to command a stop"
            if blocked else
            "No forward command was sent. This sample does not prove a past drive failure."
        ),
    }, allow_nan=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
