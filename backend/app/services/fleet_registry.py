"""Registered fleet for the control screen.

pinky-1 is the real SSH/Nav2 robot. pinky-2 and pinky-3 are virtual registry
entries. A virtual robot never receives an SSH result, a TF pose, or a Nav2
ready flag from this module.
"""
from __future__ import annotations

import json
from pathlib import Path

from app.services.map_frame import MapGeometry, describe_map_point

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_REGISTRY = ROOT / "config" / "physical_fleet.json"


class RegistryError(Exception):
    pass


def update_virtual_home(path: Path, robot_id: str, x: float, y: float, yaw: float) -> dict:
    """Persist an operator-approved virtual home. Callers must validate it first."""
    path = Path(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    found = False
    for item in payload.get("robots", []):
        if item.get("robot_id") != robot_id:
            continue
        if item.get("kind") != "virtual":
            raise RegistryError(f"{robot_id} is not a virtual robot")
        item["home"] = {"x": float(x), "y": float(y), "yaw": float(yaw)}
        found = True
    if not found:
        raise RegistryError(f"Robot is not registered: {robot_id}")
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return next(robot for robot in load_registry(path) if robot["robot_id"] == robot_id)


def load_registry(path: Path | None = None) -> list[dict]:
    path = Path(path or DEFAULT_REGISTRY)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RegistryError(f"Fleet registry unreadable: {path}: {exc}") from exc
    robots = payload.get("robots") if isinstance(payload, dict) else None
    if not isinstance(robots, list) or not robots:
        raise RegistryError("Fleet registry must contain a non-empty robots list")
    seen: set[str] = set()
    cleaned: list[dict] = []
    for item in robots:
        if not isinstance(item, dict):
            raise RegistryError("Fleet registry robot entry must be an object")
        robot_id = str(item.get("robot_id", "")).strip()
        name = str(item.get("name", "")).strip()
        kind = str(item.get("kind", "")).strip()
        if not robot_id or robot_id in seen:
            raise RegistryError(f"Invalid or duplicate robot_id: {robot_id}")
        if kind not in {"physical", "virtual"}:
            raise RegistryError(f"{robot_id} kind must be physical or virtual")
        if not name:
            raise RegistryError(f"{robot_id} is missing a name")
        home = item.get("home")
        if kind == "virtual":
            if not isinstance(home, dict):
                raise RegistryError(f"{robot_id} virtual robot requires a home pose")
            try:
                home = {
                    "x": float(home["x"]),
                    "y": float(home["y"]),
                    "yaw": float(home.get("yaw", 0.0)),
                }
            except (KeyError, TypeError, ValueError) as exc:
                raise RegistryError(f"{robot_id} home pose is invalid") from exc
        elif home is not None:
            raise RegistryError(f"{robot_id} physical robot must not carry a registry home")
        seen.add(robot_id)
        cleaned.append({"robot_id": robot_id, "name": name, "kind": kind, "home": home})
    if sum(1 for robot in cleaned if robot["kind"] == "physical") < 1:
        raise RegistryError("Fleet registry needs at least one physical robot")
    return cleaned


def _home_view(record: dict, geometry: MapGeometry | None) -> dict | None:
    home = record.get("home")
    if home is None:
        return None
    if geometry is None:
        return {
            "frame_id": "map",
            "x": home["x"],
            "y": home["y"],
            "yaw": home["yaw"],
            "grid": None,
            "pixel": None,
            "inside_map": None,
            "source": "registry_home",
        }
    described = describe_map_point(geometry, home["x"], home["y"], home["yaw"])
    described["source"] = "registry_home"
    return described


def present_robot(
    record: dict,
    *,
    ssh_robot: dict | None,
    nav2: dict | None,
    geometry: MapGeometry | None,
) -> dict:
    """One dashboard record. Virtual robots stay free of live TF and SSH."""
    if record["kind"] == "virtual":
        return {
            "robot_id": record["robot_id"],
            "name": record["name"],
            "kind": "virtual",
            "host": None,
            "connection_status": "virtual",
            "connection_kind": "registry",
            "ssh_online": False,
            "pose": None,
            "pose_checked_at": None,
            "pose_source": None,
            "nav2_ready": None,
            "ros_online": False,
            "nav2_state": "not_applicable",
            "nav2_detail": "가상 로봇 · SSH/Nav2/TF 미연결",
            "controller_ready": None,
            "home": _home_view(record, geometry),
        }

    if ssh_robot is None:
        return {
            "robot_id": record["robot_id"],
            "name": record["name"],
            "kind": "physical",
            "host": None,
            "connection_status": "ssh_offline",
            "connection_kind": "ssh_authenticated",
            "ssh_online": False,
            "pose": None,
            "pose_checked_at": None,
            "pose_source": None,
            "nav2_ready": False,
            "ros_online": False,
            "nav2_state": "waiting_for_ssh",
            "nav2_detail": "SSH 인증 전 · 실시간 TF 없음",
            "controller_ready": False,
            "home": None,
        }

    robot = {
        "robot_id": record["robot_id"],
        "name": record["name"],
        "kind": "physical",
        "host": ssh_robot.get("host"),
        "connection_status": "ssh_connected",
        "connection_kind": "ssh_authenticated",
        "ssh_online": True,
        "pose": None,
        "pose_checked_at": None,
        "pose_source": None,
        "nav2_ready": False,
        "ros_online": False,
        "nav2_state": None,
        "nav2_detail": None,
        "controller_ready": False,
        "home": None,
    }
    if nav2 is None or nav2.get("robot_id") not in (None, record["robot_id"]):
        return robot
    robot["ros_online"] = bool(nav2.get("ros_online"))
    robot["nav2_ready"] = bool(nav2.get("nav2_ready"))
    robot["controller_ready"] = bool(nav2.get("cmd_vel_chain_ready")) and bool(nav2.get("nav2_ready"))
    robot["nav2_state"] = nav2.get("state")
    robot["nav2_detail"] = nav2.get("detail")
    pose = nav2.get("pose")
    observed_at = nav2.get("pose_checked_at")
    if nav2.get("ssh_connected") and isinstance(pose, dict) and observed_at:
        robot["pose"] = {
            "x": pose.get("x"),
            "y": pose.get("y"),
            "yaw": pose.get("yaw"),
        }
        robot["pose_checked_at"] = observed_at
        robot["pose_source"] = "tf:map->base_footprint"
        if geometry is not None and _finite_pose(robot["pose"]):
            located = describe_map_point(geometry, float(pose["x"]), float(pose["y"]), float(pose.get("yaw") or 0.0))
            robot["pose"]["grid"] = located["grid"]
            robot["pose"]["inside_map"] = located["inside_map"]
    return robot


def _finite_pose(pose: dict) -> bool:
    try:
        return all(isinstance(pose[key], (int, float)) for key in ("x", "y", "yaw"))
    except KeyError:
        return False
