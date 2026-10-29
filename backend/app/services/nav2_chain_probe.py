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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

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
        "laser_frame_points_within_radius": 0,
        "stop_radius_m": STOP_RADIUS_M,
        "min_points": MIN_POINTS,
        "transform_status": "not_attempted",
        "base_frame": "base_footprint",
    }
    latest = {"message": None}

    def on_twist(topic):
        def callback(message: Twist):
            row = observed[topic]
            value = float(message.linear.x)
            row["seen"] = True
            row["max_linear_x"] = value if row["max_linear_x"] is None else max(row["max_linear_x"], value)
            row["last_stamp"] = node.get_clock().now().nanoseconds / 1e9
        return callback

    def on_scan(message: LaserScan):
        latest["message"] = message
        finite = [float(item) for item in message.ranges if math.isfinite(item) and item > 0.0]
        scan["received"] = True
        scan["frame_id"] = message.header.frame_id
        scan["stamp"] = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        scan["age_s"] = max(0.0, node.get_clock().now().nanoseconds / 1e9 - scan["stamp"])
        scan["angle_min"] = message.angle_min
        scan["angle_increment"] = message.angle_increment
        scan["range_min_m"] = message.range_min
        scan["range_max_m"] = message.range_max
        scan["min_range_m"] = min(finite) if finite else None
        scan["laser_frame_points_within_radius"] = sum(1 for item in finite if item < STOP_RADIUS_M)

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    buffer = Buffer()
    TransformListener(buffer, node)
    for topic in observed:
        node.create_subscription(Twist, topic, on_twist(topic), qos)
    node.create_subscription(LaserScan, "/scan", on_scan, scan_qos)
    end = time.monotonic() + SAMPLE_SECONDS
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    _project_stop_zone(node, buffer, latest["message"], scan)
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
    blocked = bool(
        scan.get("transform_status") == "ok"
        and scan["points_within_stop_radius"] >= MIN_POINTS
    )
    if scan.get("transform_status") == "no_scan":
        detail = (
            "StopZone diagnosis failed because no LaserScan was received. "
            "This is not a clear zone."
        )
    elif scan.get("transform_status") not in {None, "ok"}:
        detail = (
            "StopZone diagnosis failed because the scan could not be transformed "
            "into base_footprint. This is not a clear zone."
        )
    elif blocked:
        detail = (
            "base_footprint StopZone has enough transformed returns inside 0.18m to command a stop. "
            f"Laser-frame range count was {scan.get('laser_frame_points_within_radius')}."
        )
    else:
        detail = "No forward command was sent. This sample does not prove a past drive failure."
    print(json.dumps({
        "motion_command_sent": False,
        "sample_seconds": SAMPLE_SECONDS,
        "topics": observed,
        "scan": scan,
        "velocity_case": case,
        "stop_zone_would_stop": blocked,
        "detail": detail,
    }, allow_nan=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


def _project_stop_zone(node: Node, buffer: Buffer, message: LaserScan | None, scan: dict) -> None:
    """Count returns inside the circle Collision Monitor evaluates on base_footprint."""
    if message is None:
        scan["transform_status"] = "no_scan"
        return
    try:
        transform = buffer.lookup_transform(
            "base_footprint",
            message.header.frame_id,
            Time.from_msg(message.header.stamp),
            timeout=Duration(seconds=0.5),
        )
        scan["transform_time"] = "scan_stamp"
    except TransformException:
        try:
            transform = buffer.lookup_transform(
                "base_footprint",
                message.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.5),
            )
            scan["transform_time"] = "latest"
        except TransformException as exc:
            scan["transform_status"] = "failed"
            scan["transform_error"] = str(exc)[:180]
            return
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    scan["lidar_in_base_m"] = {
        "x": translation.x, "y": translation.y, "z": translation.z, "yaw_rad": yaw,
    }
    sectors = {"front": 0, "left": 0, "rear": 0, "right": 0}
    right_samples: list[tuple[float, float, float, float]] = []
    inside = 0
    valid_inside = 0
    nearest = None
    below_range_min = 0
    for index, item in enumerate(message.ranges):
        if not math.isfinite(item) or item <= 0.0:
            continue
        distance = float(item)
        if distance < float(message.range_min):
            below_range_min += 1
        angle = message.angle_min + index * message.angle_increment
        local_x = distance * math.cos(angle)
        local_y = distance * math.sin(angle)
        base_x = translation.x + math.cos(yaw) * local_x - math.sin(yaw) * local_y
        base_y = translation.y + math.sin(yaw) * local_x + math.cos(yaw) * local_y
        radius = math.hypot(base_x, base_y)
        if radius >= STOP_RADIUS_M:
            continue
        inside += 1
        nearest = radius if nearest is None else min(nearest, radius)
        heading = math.atan2(base_y, base_x)
        if abs(heading) <= math.pi / 4:
            sectors["front"] += 1
        elif heading > math.pi / 4 and heading < 3 * math.pi / 4:
            sectors["left"] += 1
        elif abs(heading) >= 3 * math.pi / 4:
            sectors["rear"] += 1
        else:
            sectors["right"] += 1
            right_samples.append((radius, base_x, base_y, distance))
        if float(message.range_min) <= distance <= float(message.range_max):
            valid_inside += 1
    right_samples.sort(key=lambda item: item[0])
    scan["transform_status"] = "ok"
    scan["points_within_stop_radius"] = inside
    scan["valid_range_points_within_stop_radius"] = valid_inside
    scan["returns_below_range_min"] = below_range_min
    scan["nearest_base_distance_m"] = nearest
    scan["inside_sectors_base_frame"] = sectors
    scan["right_returns_base_m"] = [
        {
            "x": round(x, 4),
            "y": round(y, 4),
            "range_m": round(item_range, 4),
            "radius_m": round(radius, 4),
        }
        for radius, x, y, item_range in right_samples[:12]
    ]
    scan["right_return_identity"] = "unresolved"
    scan["self_reflection_proven"] = False


if __name__ == "__main__":
    sys.exit(main())
