"""Limited, authenticated 5cm Pinky-1 Nav2 demonstration task for FMS.

An individual test does not accept arbitrary distance/pose, and uses a dedicated
ROS Jazzy /usr/bin/python3 worker. Existing simulated FMS tasks are untouched.
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import shutil
import time
from datetime import datetime, timezone
from pathlib import Path
from secrets import compare_digest

from fastapi import HTTPException, Request

ROOT = Path(__file__).resolve().parents[3]
ROS_SETUP = '/opt/ros/jazzy/setup.bash'
WORKER = Path(__file__).with_name('nav2_forward_5cm_worker.py')
ACTIVE = {'checking', 'pose_verified', 'accepted', 'running', 'cancelling'}


def now():
    return datetime.now(timezone.utc).isoformat()


class PinkyFiveCmTest:
    def __init__(self, settings, monitor):
        self.settings = settings
        self.monitor = monitor
        self.lock = asyncio.Lock()
        self.proc = None
        self.job = None
        self.current = {'state': 'idle', 'robot_id': 'pinky-1', 'requested_distance_m': 0.05}

    def snapshot(self):
        return dict(self.current)

    def authorize(self, request: Request):
        if not self.settings.physical_5cm_test_enabled:
            raise HTTPException(403, '5cm physical test disabled')
        if request.client is None or request.client.host not in ('127.0.0.1', '::1'):
            raise HTTPException(403, 'Physical test requires local Next.js proxy or localhost')
        provided = request.headers.get('X-FMS-Operator-Token', '')
        expected = self.settings.physical_5cm_test_token
        if not expected or not compare_digest(provided, expected):
            raise HTTPException(403, 'Invalid operator test token')

    def _env(self):
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(self.settings.ros_domain_id)
        env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'SUBNET'
        env.pop('ROS_LOCALHOST_ONLY', None)
        for key in list(env):
            if any(x in key.upper() for x in ('PASSWORD', 'TOKEN', 'SECRET')):
                env.pop(key, None)
        return env

    async def _manager_active(self):
        # Require Lifecycle Manager itself, not only individual active nodes.
        ok, output = await self.monitor._ros(
            'ros2', 'service', 'call', '/lifecycle_manager_navigation/is_active',
            'std_srvs/srv/Trigger', '{}', timeout=10.0,
        )
        return ok and ('success=True' in output or 'success: true' in output)

    async def start(self):
        async with self.lock:
            if self.job and not self.job.done():
                raise HTTPException(409, '5cm physical test already running')
            if self.monitor is None or self.monitor.primary_id != 'pinky-1':
                raise HTTPException(409, 'Pinky-1 Nav2 monitor not configured')
            s = self.monitor.snapshot()
            if not all(s.get(key) is True for key in (
                'ssh_connected', 'ros_online', 'map_server_active', 'amcl_active',
                'nav2_ready', 'tf_ready', 'cmd_vel_chain_ready', 'initial_pose_confirmed',
            )):
                raise HTTPException(409, 'Pinky-1 Nav2/TF/initial pose not ready')
            try:
                age = (datetime.now(timezone.utc) - datetime.fromisoformat(s['checked_at'])).total_seconds()
                pose_age = (datetime.now(timezone.utc) - datetime.fromisoformat(s['pose_checked_at'])).total_seconds()
            except (TypeError, ValueError, KeyError):
                raise HTTPException(409, 'Recent Nav2/TF observations unavailable')
            if not (0 <= age <= 45 and 0 <= pose_age <= 45):
                raise HTTPException(409, 'Nav2/TF observations stale')
            if not await self._manager_active():
                raise HTTPException(409, 'Nav2 Lifecycle Manager is not active')
            # Check physical start+target map bounds; never forward a goal beyond
            # the actual physical map extent. Nav2 costmap still vets obstacles.
            pose = s.get('pose')
            if not pose or not all(math.isfinite(pose[k]) for k in ('x','y','yaw')):
                raise HTTPException(409, 'No finite real TF position')
            from app.services.map_frame import MapFileError, load_geometry, world_to_grid
            try:
                geometry = load_geometry(Path(self.settings.physical_map_yaml))
            except MapFileError as exc:
                raise HTTPException(503, str(exc)) from exc
            yaw = pose['yaw']
            gx = pose['x'] + 0.05 * math.cos(yaw)
            gy = pose['y'] + 0.05 * math.sin(yaw)
            col, row = world_to_grid(geometry, gx, gy)
            if not (0 <= col < geometry.width_px and 0 <= row < geometry.height_px):
                raise HTTPException(409, '5cm target outside physical map')
            diagnosis = await self.diagnose()
            from app.services.velocity_chain import stop_zone_block_reason
            blocked = stop_zone_block_reason(diagnosis)
            if blocked:
                self.current = {
                    'state': 'blocked', 'robot_id': 'pinky-1',
                    'requested_distance_m': 0.05, 'finished_at': now(),
                    'detail': blocked, 'goal_sent': False,
                    'collision_monitor_bypassed': False,
                }
                raise HTTPException(409, blocked)
            self.current = {
                'state': 'checking', 'robot_id': 'pinky-1',
                'requested_distance_m': 0.05, 'started_at': now(),
                'detail': 'Nav2 DriveOnHeading 5cm task is being started',
            }
            self.job = asyncio.create_task(self._run())
            return self.snapshot()

    async def _run(self):
        import shlex
        cmd = 'source /opt/ros/jazzy/setup.bash && exec /usr/bin/python3 -u ' + shlex.quote(str(WORKER))
        terminal_seen = False
        try:
            self.proc = await asyncio.create_subprocess_exec(
                '/bin/bash', '-c', cmd, env=self._env(),
                stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT, limit=65536,
            )
            # Worker bounds the action itself. Enforce an additional watchdog.
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                try:
                    line = await asyncio.wait_for(self.proc.stdout.readline(), timeout=1.5)
                except asyncio.TimeoutError:
                    if self.proc.returncode is not None:
                        break
                    continue
                if not line:
                    break
                try:
                    event = json.loads(line.decode('utf8', errors='replace'))
                    if not isinstance(event, dict) or 'state' not in event:
                        continue
                except (ValueError, UnicodeDecodeError):
                    continue
                self.current.update(event)
                self.current['updated_at'] = now()
                if event['state'] not in ACTIVE:
                    terminal_seen = True
            if self.proc.returncode is None:
                self.proc.terminate()
                try:
                    await asyncio.wait_for(self.proc.wait(), timeout=3)
                except asyncio.TimeoutError:
                    self.proc.kill()
                    await self.proc.wait()
            else:
                await self.proc.wait()
            if not terminal_seen:
                self.current.update(state='failed', detail='ROS 5cm worker ended without final result', updated_at=now())
        except Exception as exc:
            self.current.update(state='failed', detail=f'{type(exc).__name__}: {str(exc)[:160]}', updated_at=now())
        finally:
            self.proc = None
            self.current['finished_at'] = now()

    async def diagnose(self) -> dict:
        """Sample the velocity chain and scan. Does not send a DriveOnHeading goal."""
        import shlex
        cmd = (
            'source /opt/ros/jazzy/setup.bash && exec /usr/bin/python3 -u '
            + shlex.quote(str(Path(__file__).with_name('nav2_chain_probe.py')))
        )
        try:
            proc = await asyncio.create_subprocess_exec(
                '/bin/bash', '-c', cmd, env=self._env(),
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
            )
            stdout, _stderr = await asyncio.wait_for(proc.communicate(), timeout=8)
        except asyncio.TimeoutError:
            return {'motion_command_sent': False, 'detail': 'Velocity-chain probe timed out'}
        except OSError as exc:
            return {'motion_command_sent': False, 'detail': f'{type(exc).__name__}: {exc}'}
        text = stdout.decode('utf-8', errors='replace').strip().splitlines()
        for line in reversed(text):
            try:
                body = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(body, dict) and 'scan' in body:
                body['collision_monitor_bypassed'] = False
                return body
        return {'motion_command_sent': False, 'detail': 'Velocity-chain probe returned no JSON'}

    async def cancel(self):
        async with self.lock:
            if not self.job or self.job.done() or self.proc is None or self.proc.stdin is None:
                raise HTTPException(409, 'No active 5cm Nav2 task to cancel')
            if self.proc.returncode is not None:
                raise HTTPException(409, 'Nav2 task has already exited')
            self.proc.stdin.write(b'cancel\n')
            await self.proc.stdin.drain()
            return {'cancel_requested': True}

    async def shutdown(self):
        if self.job and not self.job.done():
            if self.proc and self.proc.returncode is None:
                try:
                    self.proc.stdin.write(b'cancel\n')
                    await self.proc.stdin.drain()
                    await asyncio.wait_for(self.job, timeout=7.0)
                except (BrokenPipeError, ConnectionResetError, asyncio.TimeoutError):
                    # During process teardown, leave ROS worker running up to
                    # its bounded action deadline rather than orphaning a goal
                    # by terminating only the worker abruptly.
                    pass
