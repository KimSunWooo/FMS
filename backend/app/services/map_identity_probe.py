#!/usr/bin/env python3
"""Read /map once and print identity JSON. Publishes nothing and does not change lifecycle."""
from __future__ import annotations

import hashlib
import json
import math


def _yaw(orientation) -> float:
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _code(value: int) -> int:
    # Same trinary codes as OccupancyGrid: 0 free, 1 occupied, 2 unknown.
    if value == 0:
        return 0
    if value == 100:
        return 1
    if value == -1:
        return 2
    return 9


def main() -> int:
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(args=None)
    node = Node("fms_map_identity_probe")
    result: dict = {"error": "map observation timed out"}
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )

    def on_map(message: OccupancyGrid) -> None:
        info = message.info
        cells = bytes(_code(int(value)) for value in message.data)
        result.clear()
        result.update(
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            origin_yaw=_yaw(info.origin.orientation),
            frame_id=str(message.header.frame_id),
            content_sha256=hashlib.sha256(cells).hexdigest(),
        )
        raise SystemExit

    node.create_subscription(OccupancyGrid, "/map", on_map, qos)
    try:
        rclpy.spin_once(node, timeout_sec=4.0)
    except SystemExit:
        pass
    finally:
        print(json.dumps(result), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if "error" not in result else 2


if __name__ == "__main__":
    raise SystemExit(main())
