"""Gazebo / Nav2 preflight.

This module does not import a ROS client library and does not create a node.
A missing simulator, a missing Pinky package, or a domain mismatch is not ready.
`/usr/bin/pinky` is the Unix finger utility and is not a robot package.
"""

from __future__ import annotations

import os
import shutil
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class PreflightReport:
    ready: bool
    reasons: tuple[str, ...]
    ros_distro: str | None
    ros_domain_id: int | None
    rmw_implementation: str | None
    ros2_binary: bool
    gazebo_binary: bool
    nav2_interface: bool
    pinky_packages: tuple[str, ...]

    def as_dict(self) -> dict:
        return {
            "ready": self.ready,
            "reasons": list(self.reasons),
            "ros_distro": self.ros_distro,
            "ros_domain_id": self.ros_domain_id,
            "rmw_implementation": self.rmw_implementation,
            "ros2_binary": self.ros2_binary,
            "gazebo_binary": self.gazebo_binary,
            "nav2_interface": self.nav2_interface,
            "pinky_packages": list(self.pinky_packages),
        }


def share_directories() -> tuple[Path, ...]:
    """Jazzy install plus any sourced overlay. Order is not a search path rank."""
    found: list[Path] = []
    seen: set[Path] = set()

    def add(path: Path) -> None:
        resolved = path.resolve() if path.exists() else path
        if resolved in seen or not path.is_dir():
            return
        seen.add(resolved)
        found.append(path)

    add(Path("/opt/ros/jazzy/share"))
    for key in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        raw = os.environ.get(key) or ""
        for prefix in raw.split(":"):
            if prefix:
                add(Path(prefix) / "share")
    return tuple(found)


def packages_named(fragment: str, shares: tuple[Path, ...] | None = None) -> tuple[str, ...]:
    names: set[str] = set()
    needle = fragment.lower()
    for share in shares if shares is not None else share_directories():
        for path in share.iterdir():
            if path.is_dir() and needle in path.name.lower():
                names.add(path.name)
    return tuple(sorted(names))


def live_probe() -> dict:
    domain_raw = os.environ.get("ROS_DOMAIN_ID")
    domain = int(domain_raw) if domain_raw is not None and domain_raw.isdigit() else None
    shares = share_directories()
    nav2 = any((share / "nav2_msgs").is_dir() or (share / "nav2_bringup").is_dir() for share in shares)
    pinky = packages_named("pinky", shares)
    return {
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_domain_id": domain,
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "ros2_binary": shutil.which("ros2") is not None,
        "gazebo_binary": any(shutil.which(name) for name in ("gz", "gazebo", "ign")),
        "nav2_interface": nav2,
        "pinky_packages": list(pinky),
        "opt_ros_jazzy": Path("/opt/ros/jazzy").is_dir(),
    }


def evaluate(probe: dict, *, expected_domain: int = 73) -> PreflightReport:
    reasons: list[str] = []
    if probe.get("ros_distro") != "jazzy":
        reasons.append("ros_distro_not_jazzy")
    if not probe.get("ros2_binary"):
        reasons.append("ros2_binary_missing")
    if not probe.get("gazebo_binary"):
        reasons.append("gazebo_binary_missing")
    if not probe.get("nav2_interface"):
        reasons.append("nav2_interface_missing")
    packages = tuple(probe.get("pinky_packages") or ())
    if not packages:
        reasons.append("pinky_simulation_package_missing")
    if probe.get("ros_domain_id") != expected_domain:
        reasons.append("ros_domain_mismatch")
    if not probe.get("rmw_implementation"):
        reasons.append("rmw_implementation_unset")
    return PreflightReport(
        ready=not reasons,
        reasons=tuple(reasons),
        ros_distro=probe.get("ros_distro"),
        ros_domain_id=probe.get("ros_domain_id"),
        rmw_implementation=probe.get("rmw_implementation"),
        ros2_binary=bool(probe.get("ros2_binary")),
        gazebo_binary=bool(probe.get("gazebo_binary")),
        nav2_interface=bool(probe.get("nav2_interface")),
        pinky_packages=packages,
    )


def live_report(*, expected_domain: int = 73) -> PreflightReport:
    return evaluate(live_probe(), expected_domain=expected_domain)
