"""SSH presence for the real Pinky, virtual fleet registration, and the site map.

Virtual robots are registry entries only. This module does not invent a TF
pose, Nav2 ready flag, or motor command for them. SSH authentication is not
ROS / navigation readiness.
"""
from __future__ import annotations

import asyncio
import re
import struct
import zlib
from datetime import datetime, timezone
from pathlib import Path

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import Response
from pydantic import BaseModel, Field

from app.services.fleet_registry import RegistryError, load_registry, present_robot
from app.services.map_frame import (
    MapFileError,
    describe_grid_point,
    describe_map_point,
    load_geometry,
)

router = APIRouter(prefix="/api/physical", tags=["physical"])

_ID = re.compile(r"^[a-z0-9][a-z0-9-]{0,31}$")
_TARGET = re.compile(r"^(?P<user>[A-Za-z_][A-Za-z0-9_-]*)@(?P<host>[A-Za-z0-9][A-Za-z0-9.\-]*)$")


def parse_targets(config: str) -> list[tuple[str, str]]:
    targets: list[tuple[str, str]] = []
    seen: set[str] = set()
    for item in config.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise ValueError("FMS_PHYSICAL_SSH_TARGETS must be id=username@host")
        robot_id, target = (part.strip() for part in item.split("=", 1))
        if not _ID.fullmatch(robot_id) or not _TARGET.fullmatch(target) or robot_id in seen:
            raise ValueError("Invalid or duplicate FMS_PHYSICAL_SSH_TARGETS entry")
        seen.add(robot_id)
        targets.append((robot_id, target))
    return targets


def _ssh_password_probe(target: str, password: str, timeout: float) -> bool:
    """Read-only SSH command. Requires an already trusted host key."""
    import logging
    import paramiko

    username, hostname = target.rsplit("@", 1)
    client = paramiko.SSHClient()
    try:
        client.load_system_host_keys()  # ~/.ssh/known_hosts
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
        client.connect(
            hostname=hostname,
            username=username,
            password=password,
            look_for_keys=False,
            allow_agent=False,
            timeout=timeout,
            auth_timeout=timeout,
            banner_timeout=timeout,
        )
        _, stdout, _ = client.exec_command("true", timeout=timeout)
        return stdout.channel.recv_exit_status() == 0
    except (paramiko.SSHException, OSError, EOFError, TimeoutError) as exc:
        logging.getLogger(__name__).warning("SSH authentication check for %s failed: %s", target, exc)
        return False
    finally:
        client.close()


async def check_ssh(robot_id: str, target: str, timeout: float, password: str = "") -> dict | None:
    timeout = max(1.0, min(float(timeout), 8.0))
    if password:
        # Only perform SSH authentication if the backend has an explicit password.
        try:
            connected = await asyncio.wait_for(
                asyncio.to_thread(_ssh_password_probe, target, password, timeout),
                timeout=timeout * 4 + 2.0,
            )
        except asyncio.TimeoutError:
            return None
        if not connected:
            return None
    else:
        # Keep existing public-key operation for robots with no password set.
        try:
            proc = await asyncio.create_subprocess_exec(
                "ssh", "-T", "-n",
                "-o", "BatchMode=yes",
                "-o", "NumberOfPasswordPrompts=0",
                "-o", "StrictHostKeyChecking=yes",
                "-o", "ConnectionAttempts=1",
                "-o", f"ConnectTimeout={int(timeout)}",
                "-o", "LogLevel=ERROR",
                target, "true",
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
            )
            try:
                await asyncio.wait_for(proc.wait(), timeout=timeout + 1.0)
            except asyncio.TimeoutError:
                proc.kill()
                await proc.wait()
                return None
            if proc.returncode != 0:
                return None
        except (OSError, ValueError):
            return None
    return {
        "robot_id": robot_id,
        "name": robot_id,
        "host": target.rsplit("@", 1)[-1],
        "connection_status": "ssh_connected",
        "connection_kind": "ssh_authenticated",
        "ssh_online": True,
        "pose": None,
        "nav2_ready": None,
        "controller_ready": None,
    }


def _fresh_nav2(snapshot: dict | None) -> dict | None:
    if snapshot is None:
        return None
    published = dict(snapshot)
    pose = published.get("pose")
    observed_at = published.get("pose_checked_at")
    if not (published.get("ssh_connected") and isinstance(pose, dict) and observed_at):
        published["pose"] = None
        published["pose_checked_at"] = None
        return published
    try:
        age_s = (datetime.now(timezone.utc) - datetime.fromisoformat(observed_at)).total_seconds()
    except (ValueError, TypeError):
        age_s = float("inf")
    if not 0 <= age_s <= 60:
        published["pose"] = None
        published["pose_checked_at"] = None
    return published


@router.get("/robots")
async def connected_robots(request: Request) -> dict:
    settings = request.app.state.settings
    try:
        registry = load_registry(Path(settings.physical_fleet_path))
    except RegistryError as exc:
        raise HTTPException(503, str(exc)) from exc
    targets = dict(parse_targets(settings.physical_ssh_targets))
    physical = [robot for robot in registry if robot["kind"] == "physical" and robot["robot_id"] in targets]
    probed = await asyncio.gather(*(
        check_ssh(robot["robot_id"], targets[robot["robot_id"]], settings.physical_ssh_timeout_seconds, settings.physical_ssh_password)
        for robot in physical
    ))
    ssh_by_id = {
        robot["robot_id"]: result
        for robot, result in zip(physical, probed, strict=True)
        if result is not None
    }
    monitor = getattr(request.app.state, "physical_nav2", None)
    snapshot = _fresh_nav2(monitor.snapshot()) if monitor is not None else None
    supervisor = getattr(request.app.state, "domain_supervisor", None)
    workers = {}
    if supervisor is not None:
        try:
            workers = {item["robot_id"]: item for item in supervisor.statuses()}
        except RegistryError:
            workers = {}
    try:
        geometry = load_geometry(Path(settings.physical_map_yaml))
    except MapFileError:
        geometry = None
    from app.services.robot_state import robot_state

    robots = []
    for record in registry:
        ssh_robot = ssh_by_id.get(record["robot_id"]) if record["kind"] == "physical" else None
        nav2 = snapshot if ssh_robot is not None and monitor is not None and record["robot_id"] == monitor.primary_id else None
        item = present_robot(
            record, ssh_robot=ssh_robot, nav2=nav2, geometry=geometry,
            worker=workers.get(record["robot_id"]),
        )
        state = robot_state(record["robot_id"], settings, snapshot if record["robot_id"] == "pinky-1" else None)
        # Physical-only RobotStatus telemetry. Virtual robots keep registry home separate.
        if record["kind"] == "physical":
            if state.get("pose_valid") and isinstance(state.get("pose"), dict) and state.get("pose_source") != "registry_home":
                item["pose"] = {
                    "x": state["pose"].get("x"),
                    "y": state["pose"].get("y"),
                    "yaw": state["pose"].get("yaw"),
                }
                item["pose_checked_at"] = state.get("pose_timestamp")
                item["pose_source"] = state.get("pose_source")
                item["pose_valid"] = True
                item["pose_freshness"] = state.get("pose_freshness")
                if geometry is not None and all(isinstance(item["pose"].get(k), (int, float)) for k in ("x", "y")):
                    located = describe_map_point(
                        geometry,
                        float(item["pose"]["x"]),
                        float(item["pose"]["y"]),
                        float(item["pose"].get("yaw") or 0.0),
                    )
                    item["pose"]["grid"] = located["grid"]
                    item["pose"]["inside_map"] = located["inside_map"]
            else:
                item["pose_valid"] = False
                item["pose_freshness"] = state.get("pose_freshness") or "missing"
            item["battery_valid"] = bool(state.get("battery_valid"))
            item["battery_percentage"] = state.get("battery_percentage") if state.get("battery_valid") else None
            item["battery_source"] = state.get("battery_source") if state.get("battery_valid") else None
            item["battery_timestamp"] = state.get("battery_timestamp") if state.get("battery_valid") else None
            item["battery_freshness"] = state.get("battery_freshness") or "missing"
            item["plan_valid"] = bool(state.get("plan_valid"))
            item["plan"] = state.get("plan") if state.get("plan_valid") else None
            item["plan_source"] = state.get("plan_source") if state.get("plan_valid") else None
            item["plan_timestamp"] = state.get("plan_timestamp") if state.get("plan_valid") else None
            item["plan_freshness"] = state.get("plan_freshness") or "missing"
            item["receiver_ready"] = state.get("receiver_ready")
            item["task_assignable"] = bool(state.get("assignable"))
        else:
            item["pose_valid"] = False
            item["battery_valid"] = False
            item["battery_percentage"] = None
            item["battery_freshness"] = "not_applicable"
            item["plan_valid"] = False
            item["plan"] = None
            item["plan_freshness"] = "not_applicable"
        robots.append(item)
    return {
        "robots": robots,
        "configured_count": len(registry),
        "connected_count": sum(1 for robot in robots if robot["ssh_online"]),
        "physical_count": sum(1 for robot in robots if robot["kind"] == "physical"),
        "virtual_count": sum(1 for robot in robots if robot["kind"] == "virtual"),
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "data_source": "fleet_registry",
    }


@router.get("/domain-workers")
async def domain_workers(request: Request) -> dict:
    supervisor = getattr(request.app.state, "domain_supervisor", None)
    if supervisor is None:
        return {"enabled": False, "workers": [], "detail": "FMS_DOMAIN_WORKERS_ENABLED=false"}
    try:
        workers = supervisor.statuses()
    except RegistryError as exc:
        raise HTTPException(503, str(exc)) from exc
    return {"enabled": True, "workers": workers, "checked_at": datetime.now(timezone.utc).isoformat()}


@router.get("/nav2/status")
async def physical_nav2_status(request: Request) -> dict:
    monitor = getattr(request.app.state, "physical_nav2", None)
    if monitor is None:
        return {"state": "disabled", "detail": "FMS_PHYSICAL_NAV2_AUTO_START=false", "nav2_ready": False}
    return monitor.snapshot()


# FMS_PINKY1_5CM_TEST_V1: dedicated real Nav2 test, NOT the mock task API.
@router.get("/tests/pinky-1/forward-5cm/diagnosis")
async def physical_5cm_diagnosis(request: Request) -> dict:
    controller = getattr(request.app.state, "forward_5cm", None)
    if controller is None:
        return {"robot_id": "pinky-1", "motion_command_sent": False, "detail": "5cm test not enabled"}
    return await controller.diagnose()


@router.get("/tests/pinky-1/forward-5cm")
async def physical_5cm_test_status(request: Request) -> dict:
    controller = getattr(request.app.state, "forward_5cm", None)
    if controller is None:
        return {"robot_id": "pinky-1", "state": "disabled", "detail": "5cm test not enabled"}
    return controller.snapshot()


@router.post("/tests/pinky-1/forward-5cm", status_code=202)
async def physical_5cm_test_start(request: Request) -> dict:
    controller = getattr(request.app.state, "forward_5cm", None)
    if controller is None:
        raise HTTPException(503, "Physical Nav2 test monitor unavailable")
    controller.authorize(request)
    return await controller.start()


@router.post("/tests/pinky-1/forward-5cm/cancel", status_code=202)
async def physical_5cm_test_cancel(request: Request) -> dict:
    controller = getattr(request.app.state, "forward_5cm", None)
    if controller is None:
        raise HTTPException(503, "Physical Nav2 test monitor unavailable")
    controller.authorize(request)
    return await controller.cancel()


# FMS_DIRECT_5CM_LIVE_POSE_V1: fast read-only pose endpoint; no SSH probe on every UI refresh.
@router.get("/nav2/pose")
async def physical_live_pose(request: Request) -> dict:
    settings = request.app.state.settings
    from app.services.robot_state import robot_state

    state = robot_state("pinky-1", settings)
    # Prefer RobotStatus-reported pose; never invent or substitute registry home.
    pose = state.get("pose") if state.get("pose_valid") and state.get("tf_fresh") else None
    pose_checked_at = state.get("pose_timestamp")
    pose_source = state.get("pose_source")
    monitor = getattr(request.app.state, "physical_nav2", None)
    latest = {} if monitor is None else monitor.snapshot()
    if pose is None and monitor is not None:
        # Diagnostic Nav2 monitor path only when RobotStatus pose is unavailable.
        mon_pose = latest.get("pose")
        if isinstance(mon_pose, dict) and mon_pose.get("x") is not None:
            pose = dict(mon_pose)
            pose_checked_at = latest.get("pose_checked_at")
            pose_source = "tf:map->base_footprint"
    if isinstance(pose, dict):
        pose = dict(pose)
        try:
            geometry = _geometry(request)
            if all(isinstance(pose.get(key), (int, float)) for key in ("x", "y")):
                yaw = float(pose["yaw"]) if isinstance(pose.get("yaw"), (int, float)) else 0.0
                located = describe_map_point(geometry, float(pose["x"]), float(pose["y"]), yaw)
                pose["grid"] = located["grid"]
                pose["inside_map"] = located["inside_map"]
        except HTTPException:
            pass
    return {
        "robot_id": "pinky-1",
        "pose": pose,
        "pose_checked_at": pose_checked_at,
        "pose_source": pose_source,
        "pose_valid": bool(pose is not None and state.get("pose_valid")),
        "pose_freshness": state.get("pose_freshness"),
        "yaw": None if not isinstance(pose, dict) else pose.get("yaw"),
        "ros_domain_id": state.get("ros_domain_id"),
        "battery_valid": bool(state.get("battery_valid")),
        "battery_percentage": state.get("battery_percentage") if state.get("battery_valid") else None,
        "battery_source": state.get("battery_source") if state.get("battery_valid") else None,
        "battery_timestamp": state.get("battery_timestamp") if state.get("battery_valid") else None,
        "battery_freshness": state.get("battery_freshness"),
        "live_tf_status": latest.get("live_tf_status"),
        "live_tf_age_s": latest.get("live_tf_age_s"),
        "live_tf_withheld_reason": latest.get("live_tf_withheld_reason"),
        "collision_monitor_active": latest.get("collision_monitor_active"),
        "nav2_ready": latest.get("nav2_ready"),
        "map_matches_fms": latest.get("map_matches_fms"),
        "map_server_yaml": latest.get("map_server_yaml"),
        "expected_map_yaml": latest.get("expected_map_yaml"),
        "map_mismatch_detail": latest.get("map_mismatch_detail"),
        "five_cm_blockers": latest.get("five_cm_blockers") or [],
        "receiver_ready": state.get("receiver_ready"),
    }


def _geometry(request: Request):
    settings = request.app.state.settings
    try:
        return load_geometry(Path(settings.physical_map_yaml))
    except MapFileError as exc:
        raise HTTPException(503, str(exc)) from exc


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    """Decode the standard P5/P2 PGM used by Nav2 without Pillow/ROS imports."""
    try:
        content = path.read_bytes()
    except OSError as exc:
        raise HTTPException(503, f"Actual map PGM unavailable: {path}: {exc}") from exc
    pos = 0

    def token() -> bytes:
        nonlocal pos
        while pos < len(content):
            if content[pos] in b" \r\n\t":
                pos += 1
            elif content[pos] == ord("#"):
                while pos < len(content) and content[pos] not in b"\r\n":
                    pos += 1
            else:
                break
        start = pos
        while pos < len(content) and content[pos] not in b" \r\n\t":
            pos += 1
        if start == pos:
            raise ValueError("Truncated PGM header")
        return content[start:pos]

    try:
        fmt = token()
        width, height, max_value = int(token()), int(token()), int(token())
        if fmt not in (b"P5", b"P2") or width <= 0 or height <= 0 or not 1 <= max_value <= 65535:
            raise ValueError("Invalid PGM header")
        pixels = width * height
        if pixels > 50_000_000:
            raise ValueError("Map image exceeds size limit")
        if fmt == b"P5":
            if pos >= len(content) or content[pos] not in b" \r\n\t":
                raise ValueError("Invalid PGM payload separator")
            if content[pos:pos + 2] == b"\r\n":
                pos += 2
            else:
                pos += 1
            count = pixels * (2 if max_value > 255 else 1)
            raw = content[pos:pos + count]
            if len(raw) != count:
                raise ValueError("Incomplete PGM pixels")
            if max_value > 255:
                values = struct.unpack(f">{pixels}H", raw)
                gray = bytes(round(value * 255 / max_value) for value in values)
            elif max_value != 255:
                gray = bytes(round(value * 255 / max_value) for value in raw)
            else:
                gray = raw
        else:
            values = [int(token()) for _ in range(pixels)]
            gray = bytes(round(value * 255 / max_value) for value in values)
        return width, height, gray
    except (ValueError, IndexError, struct.error) as exc:
        raise HTTPException(503, f"Could not decode actual PGM map: {exc}") from exc


def _png_gray(width: int, height: int, pixels: bytes) -> bytes:
    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    rows = b"".join(b"\x00" + pixels[y * width:(y + 1) * width] for y in range(height))
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">2I5B", width, height, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(rows, level=6))
        + chunk(b"IEND", b"")
    )


@router.get("/map")
def physical_map(request: Request) -> dict:
    return _geometry(request).public_dict()


class _MapPointIn(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class _GridPointIn(BaseModel):
    col: float
    row: float


class _CoordinateIn(BaseModel):
    map: _MapPointIn | None = None
    grid: _GridPointIn | None = Field(default=None)


@router.post("/map/coordinates")
def physical_map_coordinates(body: _CoordinateIn, request: Request) -> dict:
    """Convert one map-frame point or one occupancy-grid cell. No navigation."""
    if (body.map is None) == (body.grid is None):
        raise HTTPException(422, "Provide exactly one of map or grid")
    geometry = _geometry(request)
    if body.map is not None:
        return describe_map_point(geometry, body.map.x, body.map.y, body.map.yaw)
    assert body.grid is not None
    return describe_grid_point(geometry, body.grid.col, body.grid.row)


@router.get("/map/image")
def physical_map_image(request: Request) -> Response:
    geometry = _geometry(request)
    width, height, pixels = _read_pgm(geometry.image_path)
    return Response(_png_gray(width, height, pixels), media_type="image/png", headers={"Cache-Control": "no-store"})

# FMS_LIVE_TF_POSE_OVERLAY_V1
