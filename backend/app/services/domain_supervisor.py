"""Start one domain worker process per registered robot.

Each child receives its own ROS_DOMAIN_ID. This module never assigns
os.environ['ROS_DOMAIN_ID'] on the API process.
"""
from __future__ import annotations

import asyncio
import logging
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from app.services.domain_status import interpret_status, read_status, status_path
from app.services.fleet_registry import RegistryError, load_registry

log = logging.getLogger(__name__)
BACKEND = Path(__file__).resolve().parents[2]


class DomainSupervisor:
    def __init__(self, settings, popen=subprocess.Popen):
        self.settings = settings
        self.popen = popen
        self.children: dict[str, subprocess.Popen] = {}
        self._stop = False

    def spawn_env(self, robot: dict) -> dict[str, str]:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(robot["ros_domain_id"])
        env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SUBNET"
        env.pop("ROS_LOCALHOST_ONLY", None)
        env["FMS_WORKER_ROBOT_ID"] = robot["robot_id"]
        env["FMS_PHYSICAL_FLEET_PATH"] = str(self.settings.physical_fleet_path)
        env["FMS_DATA_DIR"] = str(self.settings.data_dir)
        env["FMS_DOMAIN_WORKER_INTERVAL_S"] = "3"
        for key in list(env):
            if any(token in key.upper() for token in ("PASSWORD", "TOKEN", "SECRET")):
                env.pop(key, None)
        return env

    def ensure(self, robot: dict) -> subprocess.Popen:
        robot_id = robot["robot_id"]
        current = self.children.get(robot_id)
        if current is not None and current.poll() is None:
            return current
        env = self.spawn_env(robot)
        proc = self.popen(
            [sys.executable, "-m", "app.services.domain_worker"],
            cwd=str(BACKEND),
            env=env,
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.children[robot_id] = proc
        return proc

    def statuses(self, *, now: datetime | None = None) -> list[dict]:
        robots = load_registry(self.settings.physical_fleet_path)
        rows = []
        for robot in robots:
            raw = read_status(status_path(self.settings.data_dir, robot["robot_id"]))
            viewed = interpret_status(raw, robot, now=now)
            proc = self.children.get(robot["robot_id"])
            running = bool(proc is not None and proc.poll() is None)
            viewed["worker_running"] = running
            if running and not robot["physical_connection_enabled"] and viewed.get("detail") == "worker status missing":
                viewed["connection_state"] = "physical_connection_not_configured"
                viewed["detail"] = "Physical connection not configured"
            if not running:
                viewed["pose"] = None
                viewed["ros_connected"] = False
                viewed["nav2_ready"] = False
                if viewed["connection_state"] != "physical_connection_not_configured":
                    viewed["connection_state"] = "disconnected"
                    viewed["tf_status"] = "missing"
                    viewed["detail"] = "worker process is not running"
            if viewed.get("robot_id") != robot["robot_id"]:
                continue
            rows.append(viewed)
        return rows

    async def run(self) -> None:
        while not self._stop:
            try:
                for robot in load_registry(self.settings.physical_fleet_path):
                    self.ensure(robot)
            except (RegistryError, OSError):
                log.exception("domain worker supervisor could not refresh workers")
            await asyncio.sleep(2.0)

    async def shutdown(self) -> None:
        self._stop = True
        for proc in self.children.values():
            if proc.poll() is None:
                proc.terminate()
        for proc in self.children.values():
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
