"""Disabled Nav2 bridge.

Importing this module does not import rclpy and does not create a node.
Calling the entry point always fails closed.
"""

from __future__ import annotations


class Nav2BridgeDisabled(RuntimeError):
    pass


def send_navigate_to_pose(*_args, **_kwargs) -> None:
    raise Nav2BridgeDisabled(
        "Phase 1 refuses to open a Nav2 action client. "
        "No /navigate_to_pose goal and no /cmd_vel message is sent."
    )
