"""Nav2 entry points.

`send_navigate_to_pose` stays fail-closed so a direct call cannot move a robot.
A Gazebo client is constructed only through `build_bridge`, and that client
still refuses to send unless preflight, domain, and mode all match.
Importing this module does not import a ROS client library.
"""

from __future__ import annotations

from robot_agent.gazebo_nav2_client import GazeboNav2Client
from robot_agent.preflight import PreflightReport


class Nav2BridgeDisabled(RuntimeError):
    pass


def send_navigate_to_pose(*_args, **_kwargs) -> None:
    raise Nav2BridgeDisabled(
        "This entry point refuses to open a Nav2 action client. "
        "No /navigate_to_pose goal and no /cmd_vel message is sent."
    )


def build_bridge(
    mode: str,
    preflight: PreflightReport,
    *,
    expected_domain: int,
    observed_domain: int | None,
) -> GazeboNav2Client:
    if mode != "gazebo":
        raise Nav2BridgeDisabled(
            f"Nav2 bridge is closed for mode {mode!r}. Physical motion stays blocked."
        )
    return GazeboNav2Client(
        mode=mode,
        expected_domain=expected_domain,
        observed_domain=observed_domain,
        preflight=preflight,
    )
