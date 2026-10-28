#!/usr/bin/env python3
"""Read-only ROS 2 TF watcher. Emits finite map-frame poses; never sends goals."""
from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def publish(value):
    print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)


def main():
    rclpy.init(args=None)
    node = Node('fms_pinky1_live_pose_observer')
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    next_sample = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.10)
            if time.monotonic() < next_sample:
                continue
            next_sample = time.monotonic() + 0.5
            try:
                tf = buffer.lookup_transform('map', 'base_footprint', Time())
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
                age = time.time() - stamp
                # Do not treat an old transform or static frame as current pose.
                if not (0 <= age <= 3.0):
                    publish({'pose': None})
                    continue
                p, q = tf.transform.translation, tf.transform.rotation
                values = (p.x, p.y, q.x, q.y, q.z, q.w)
                if not all(math.isfinite(v) for v in values):
                    publish({'pose': None})
                    continue
                norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
                if not (0.5 < norm < 1.5):
                    publish({'pose': None})
                    continue
                x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
                yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                publish({'pose': {'x': p.x, 'y': p.y, 'yaw': yaw},
                         'pose_checked_at': datetime.now(timezone.utc).isoformat(),
                         'tf_stamp': stamp})
            except Exception:
                publish({'pose': None})
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
