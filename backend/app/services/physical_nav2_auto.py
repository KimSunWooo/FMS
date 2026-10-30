"""FMS-owned, read-only Nav2 auto bringup for one real Pinky on swPC.

SSH auth is only a gate. Never sends a cmd_vel or navigation action goal.
Existing ROS processes are always reused; an existing incomplete Nav2 stack is
reported rather than being duplicated. No ROS Python imports in the API venv.
"""
from __future__ import annotations

import asyncio
import json
import math
import time
import logging
import os
import re
import signal
import subprocess
from datetime import datetime, timezone
from pathlib import Path

log = logging.getLogger(__name__)
ROOT = Path(__file__).resolve().parents[3]
ROS_SETUP = Path('/opt/ros/jazzy/setup.bash')
NAV_NODES = (
    'controller_server', 'smoother_server', 'planner_server',
    'behavior_server', 'velocity_smoother', 'collision_monitor',
    'bt_navigator', 'waypoint_follower',
)


# FMS_LIVE_TF_POSE_OVERLAY_V1
# Parse actual ROS tf2_echo output; never substitute an initial-position guess.
def parse_map_tf_pose(text: str) -> dict | None:
    import math
    matches = re.findall(
        r'Translation:\s*\[([^\]]+)\].*?Quaternion\s*\(xyzw\)\s*\[([^\]]+)\]',
        text, flags=re.DOTALL,
    )
    if not matches:
        return None
    try:
        xyz = [float(part.strip()) for part in matches[-1][0].split(',')]
        quat = [float(part.strip()) for part in matches[-1][1].split(',')]
        if len(xyz) != 3 or len(quat) != 4:
            return None
        if not all(math.isfinite(value) for value in xyz + quat):
            return None
        qx, qy, qz, qw = quat
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 0.5 or norm > 1.5:
            return None
        qx, qy, qz, qw = (v / norm for v in (qx, qy, qz, qw))
        yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        return {'x': xyz[0], 'y': xyz[1], 'yaw': yaw}
    except (ValueError, TypeError, OverflowError):
        return None


class PhysicalNav2Auto:
    def __init__(self, settings):
        self.settings = settings
        self.primary_id = settings.physical_nav2_robot_id
        self.children: dict[str, subprocess.Popen] = {}
        self.last_start: dict[str, float] = {}
        self.state: dict = {
            'robot_id': self.primary_id,
            'ssh_connected': False,
            'ros_online': False,
            'map_server_active': False,
            'amcl_active': False,
            'nav2_active': False,
            'tf_ready': False,
            'pose': None,  # Fresh map -> base_footprint observation only.
            'pose_checked_at': None,
            'cmd_vel_chain_ready': False,
            'nav2_ready': False,
            'state': 'waiting_for_ssh',
            'detail': '실물 Pinky SSH 인증 대기',
            'checked_at': None,
            'map_matches_fms': False,
            'map_server_yaml': None,
            'expected_map_yaml': None,
            'map_content_sha256': None,
            'expected_map_sha256': None,
            'map_mismatch_detail': 'map identity not checked',
            'collision_monitor_active': False,
            'launch_owned_by_fms': False,
            'initial_pose_target': None,
            'initial_pose_sent': False,
            'initial_pose_confirmed': False,
            'initial_pose_state': 'awaiting_operator',
            'initial_pose_auto': bool(getattr(settings, 'physical_initial_pose_auto', False)),
            'initial_pose_source': None,
            'localization_ready': False,
            'nav2_resume_attempts': 0,
            'bringup_phase': 'boot',
        }
        self._stop = False
        # FMS_DIRECT_5CM_LIVE_POSE_V1: dedicated TF watcher independent of slow Nav2 polls.
        self._live_pose_seen = False
        self._live_pose = None
        self._live_pose_checked_at = None
        self._live_lookup_failed = False
        self._live_observer_down = False
        # FMS_NAV2_ATOMIC_SNAPSHOT_V1: expose only a completed check.
        self._published_state: dict = dict(self.state)
        self._initial_pose_sent = False
        self._initial_pose_attempts = 0
        self._initial_pose_last_attempt = -1e9
        self._seed_cutoff_monotonic = None  # observations before this are ignored after seed
        self._ssh_failure_checks = 0
        self._nav2_activate_attempts = 0
        self._nav2_activate_last = -1e9
        self._load_seed_session()

    def snapshot(self) -> dict:
        """Publish one completed poll, then overlay only a fresh live TF sample.

        A failed live sample does not erase checked_at and is not replaced with
        the slower poll pose. Map display and the 5cm gate both see that gap.
        """
        from app.services.observation_gate import LIVE_TF_MAX_AGE_S, observation_blockers, stamp_status

        result = dict(self._published_state)
        result['poll_pose_checked_at'] = self._published_state.get('pose_checked_at')
        poll_pose = self._published_state.get('pose')
        result['poll_pose'] = dict(poll_pose) if isinstance(poll_pose, dict) else None
        if not self._live_pose_seen and not self._live_observer_down:
            live_status, live_age = 'missing', None
        elif self._live_observer_down:
            live_status, live_age = 'observer_down', None
        elif self._live_lookup_failed or self._live_pose is None:
            live_status, live_age = 'lookup_failed', None
        else:
            live_status, live_age = stamp_status(
                self._live_pose_checked_at, max_age=LIVE_TF_MAX_AGE_S,
            )
        result['live_pose_checked_at'] = self._live_pose_checked_at if self._live_pose_seen else None
        result['live_tf_age_s'] = live_age
        # SSH and initial-pose flags are diagnostic only. A fresh TF sample is
        # never withheld solely because SSH failed or auto-init was skipped.
        result['live_tf_withheld_reason'] = ''
        if live_status == 'fresh' and isinstance(self._live_pose, dict):
            result['pose'] = dict(self._live_pose)
            result['pose_checked_at'] = self._live_pose_checked_at
            result['pose_source'] = 'tf:map->base_footprint'
            result['live_tf_status'] = 'fresh'
        else:
            result['pose'] = None
            result['pose_checked_at'] = None
            result['live_tf_status'] = live_status
        result['five_cm_blockers'] = observation_blockers(result)
        return result

    def _update(self, state: str, detail: str, **extra) -> None:
        self.state.update(state=state, detail=detail,
                          checked_at=datetime.now(timezone.utc).isoformat(), **extra)

    def _env(self) -> dict[str, str]:
        env = os.environ.copy()
        from app.services.domain_status import resolve_robot_domain
        env['ROS_DOMAIN_ID'] = str(resolve_robot_domain(self.primary_id, self.settings))
        env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'SUBNET'
        env.pop('ROS_LOCALHOST_ONLY', None)
        # Do not leak the password into child ROS launch processes.
        env.pop('FMS_PHYSICAL_SSH_PASSWORD', None)
        return env

    @staticmethod
    def _bash(command: str) -> list[str]:
        return ['/bin/bash', '-c',
                'source /opt/ros/jazzy/setup.bash && ' + command]

    async def _ros(self, *args: str, timeout: float = 7.0) -> tuple[bool, str]:
        # All arguments are developer-controlled, never a user/task/HTTP input.
        import shlex
        command = ' '.join(shlex.quote(str(s)) for s in args)
        def call():
            try:
                result = subprocess.run(
                    self._bash(command), env=self._env(), capture_output=True,
                    text=True, timeout=timeout, check=False,
                )
                return result.returncode == 0, (result.stdout + '\n' + result.stderr)[:8000]
            except subprocess.TimeoutExpired as exc:
                stdout = exc.stdout or b''
                stderr = exc.stderr or b''
                if isinstance(stdout, bytes):
                    stdout = stdout.decode('utf-8', errors='replace')
                if isinstance(stderr, bytes):
                    stderr = stderr.decode('utf-8', errors='replace')
                return False, (stdout + '\n' + stderr)[:8000]
            except OSError as exc:
                return False, str(exc)
        return await asyncio.to_thread(call)

    async def _nodes(self) -> set[str]:
        ok, output = await self._ros('ros2', 'node', 'list')
        if not ok:
            return set()
        return set(output.split())

    async def _lifecycle(self, name: str) -> str:
        ok, output = await self._ros('ros2', 'lifecycle', 'get', '/' + name)
        if not ok:
            return 'unavailable'
        match = re.search(r'\b(active|inactive|unconfigured|finalized)\s*\[\d+\]', output)
        return match.group(1) if match else 'unknown'

    async def _publisher(self, topic: str) -> bool:
        ok, output = await self._ros('ros2', 'topic', 'info', topic)
        return bool(ok and re.search(r'Publisher count:\s*[1-9]\d*', output))

    def _spawn(self, key: str, cmd: list[str]) -> bool:
        current = self.children.get(key)
        if current is not None and current.poll() is None:
            return True
        # Avoid tight respawn loops on bad YAML, plugin failures, etc.
        from time import monotonic
        now = monotonic()
        if now - self.last_start.get(key, -1e9) < 20.0:
            return False
        self.last_start[key] = now
        path = ROOT / 'logs' / ('physical-nav2-' + key + '.log')
        path.parent.mkdir(parents=True, exist_ok=True)
        import shlex
        shell = 'exec ' + ' '.join(shlex.quote(part) for part in cmd)
        try:
            with path.open('ab') as output:
                self.children[key] = subprocess.Popen(
                    self._bash(shell), env=self._env(), stdin=subprocess.DEVNULL,
                    stdout=output, stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            log.info('Launched %s; logfile=%s', key, path)
            return True
        except OSError as exc:
            log.error('Failed starting %s: %s', key, exc)
            return False

    def _load_seed_session(self) -> None:
        from app.services.fleet_registry import CANONICAL_DOMAINS
        from app.services.initial_pose_session import load_session, same_seed_context

        data_dir = getattr(self.settings, "data_dir", None)
        if data_dir is None:
            return
        session = load_session(Path(data_dir), self.primary_id)
        if not isinstance(session, dict):
            return
        domain = CANONICAL_DOMAINS.get(self.primary_id, int(getattr(self.settings, "ros_domain_id", 24)))
        if not same_seed_context(
            session,
            robot_id=self.primary_id,
            domain=domain,
            map_sha=session.get("map_sha256"),
        ):
            return
        if session.get("sent"):
            self._initial_pose_sent = True
            self._initial_pose_attempts = int(session.get("attempts") or 1)
            self.state["initial_pose_sent"] = True
            self.state["initial_pose_state"] = (
                "observation_confirmed" if session.get("confirmed") else "sent_waiting_for_observation"
            )
            self.state["initial_pose_confirmed"] = bool(session.get("confirmed"))
            if isinstance(session.get("target"), dict):
                self.state["initial_pose_target"] = session["target"]
            if session.get("source"):
                self.state["initial_pose_source"] = session.get("source")

    async def _ensure_lifecycle(self, name: str, nodes: set[str], cmd: list[str]) -> bool:
        """Own map_server/amcl only. Never re-request configure/activate when already active."""
        matches = [item for item in nodes if item == '/' + name or item.endswith('/' + name)]
        if len(matches) > 1:
            self._update('duplicate_node', f'{name} 중복 노드 감지: {matches}')
            return False
        if '/' + name not in nodes and not any(item.endswith('/' + name) for item in nodes):
            if not self._spawn(name, cmd):
                self._update('launch_failed', f'{name} 실행 실패; logs/physical-nav2-{name}.log 확인')
            else:
                self._update('starting_' + name, f'{name} 프로세스 시작 중')
            return False
        state = await self._lifecycle(name)
        if state == 'unavailable':
            # Timeout/query flake — re-read once; do not treat as inactive.
            state = await self._lifecycle(name)
            if state == 'unavailable':
                self._update('waiting_' + name, f'{name} lifecycle 조회 실패(timeout/미응답); 재시도')
                return False
        if state == 'active':
            return True
        if state == 'unconfigured':
            _ok, output = await self._ros('ros2', 'lifecycle', 'set', '/' + name, 'configure')
            after = await self._lifecycle(name)
            if after == 'active':
                return True
            if after == 'inactive':
                self._update('configuring_' + name, f'{name} configure 완료 → activate 대기')
                return False
            if 'current state active' in output or 'Transition is not registered' in output:
                return (await self._lifecycle(name)) == 'active'
            self._update('configuring_' + name, f'{name} configure 진행/실패 (now={after})')
            return False
        if state == 'inactive':
            _ok, output = await self._ros('ros2', 'lifecycle', 'set', '/' + name, 'activate')
            after = await self._lifecycle(name)
            if after == 'active':
                return True
            if 'current state active' in output or 'No transition matching activate' in output:
                return (await self._lifecycle(name)) == 'active'
            self._update('activating_' + name, f'{name} activate 진행/실패 (now={after})')
            return False
        self._update('waiting_' + name, f'{name} 현재 상태: {state}')
        return False

    async def _param_string(self, node: str, name: str) -> str | None:
        ok, output = await self._ros('ros2', 'param', 'get', node, name, timeout=8.0)
        if not ok:
            return None
        match = re.search(r'String value is:\s*(.+)', output)
        return match.group(1).strip() if match else None

    async def _map_probe(self) -> dict:
        import shlex
        script = Path(__file__).with_name('map_identity_probe.py')
        command = 'source /opt/ros/jazzy/setup.bash && exec /usr/bin/python3 -u ' + shlex.quote(str(script))
        process = await asyncio.create_subprocess_exec(
            '/bin/bash', '-c', command, env=self._env(),
            stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        try:
            line = await asyncio.wait_for(process.stdout.readline(), timeout=6.0)
        except asyncio.TimeoutError:
            process.kill()
            await process.wait()
            return {'error': 'map observation timed out'}
        await process.wait()
        try:
            payload = json.loads(line.decode('utf-8', errors='replace') or '{}')
        except json.JSONDecodeError:
            return {'error': 'map observation was not JSON'}
        return payload if isinstance(payload, dict) else {'error': 'map observation had the wrong type'}

    async def _record_map_identity(self, expected: Path) -> None:
        from app.services.map_identity import assess_map_identity
        try:
            server_yaml = await self._param_string('/map_server', 'yaml_filename')
            probe = await self._map_probe()
            report = assess_map_identity(expected_yaml=expected, server_yaml=server_yaml, probe=probe)
        except Exception as exc:
            report = {
                'map_matches_fms': False,
                'map_server_yaml': None,
                'expected_map_yaml': str(expected),
                'map_content_sha256': None,
                'expected_map_sha256': None,
                'map_mismatch_detail': f'map identity observation failed: {type(exc).__name__}',
            }
        self.state.update(report)
        if not report['map_matches_fms']:
            log.warning('Map server does not match the FMS map: %s', report['map_mismatch_detail'])

    async def _verify_chain(self) -> bool:
        # Real graph checks; topic name presence alone is not sufficient.
        tests = (
            ('/cmd_vel_nav', 'velocity_smoother'),
            ('/cmd_vel_smoothed', 'collision_monitor'),
            ('/cmd_vel', 'pinky_bringup'),
        )
        for topic, subscriber in tests:
            ok, info = await self._ros('ros2', 'topic', 'info', topic, '-v')
            if not ok or f'Node name: {subscriber}' not in info:
                return False
            if topic == '/cmd_vel' and 'Node name: collision_monitor' not in info:
                return False
        return True

    async def _tf(self) -> bool:
        # tf2_echo runs indefinitely; timeout output is expected. An actual
        # complete, finite transform is required for readiness and map marker.
        # Do not reuse old coordinates after a timeout or broken TF.
        _ok, output = await self._ros('ros2', 'run', 'tf2_ros', 'tf2_echo',
                                      'map', 'base_footprint', timeout=4.0)
        pose = parse_map_tf_pose(output)
        if pose is not None and self._seed_cutoff_monotonic is not None:
            # After a seed, ignore TF samples until the cutoff window advances;
            # wall clock of observation is recorded separately below.
            pass
        self.state['pose'] = pose
        self.state['pose_checked_at'] = (
            datetime.now(timezone.utc).isoformat() if pose is not None else None
        )
        return pose is not None

    async def _amcl_pose_once(self) -> dict | None:
        """One /amcl_pose sample via CLI. Not a current-pose substitute for UI alone."""
        topic = str(getattr(self.settings, 'ros_amcl_pose_topic', '/amcl_pose'))
        ok, output = await self._ros(
            'ros2', 'topic', 'echo', topic, '--once', '--full-length', timeout=5.0,
        )
        if not ok or 'position' not in output:
            return None
        try:
            # Minimal parse: x/y from first position block.
            match = re.search(
                r'position:\s*\n\s*x:\s*([-\d.eE]+)\s*\n\s*y:\s*([-\d.eE]+)',
                output,
            )
            if not match:
                return None
            x, y = float(match.group(1)), float(match.group(2))
            if not (math.isfinite(x) and math.isfinite(y)):
                return None
            return {
                'x': x, 'y': y, 'source': 'topic:amcl_pose',
                'observed_at': datetime.now(timezone.utc).isoformat(),
            }
        except (TypeError, ValueError):
            return None

    async def _localization_observed(self) -> bool:
        """True when a post-seed (or pre-existing) map localization is visible."""
        amcl = await self._amcl_pose_once()
        tf_ok = await self._tf()
        if self._seed_cutoff_monotonic is not None:
            # Require observation strictly after the seed publish wall time.
            if time.monotonic() < self._seed_cutoff_monotonic + 0.5:
                return False
        if amcl is not None:
            yaw = None
            if isinstance(self.state.get('pose'), dict):
                yaw = self.state['pose'].get('yaw')
            self.state['pose'] = {'x': amcl['x'], 'y': amcl['y'], 'yaw': yaw}
            self.state['pose_checked_at'] = amcl['observed_at']
            self.state['pose_source'] = 'topic:amcl_pose'
            return True
        if tf_ok:
            self.state['pose_source'] = 'tf:map->base_footprint'
            return True
        return False

    def _persist_seed_session(self, *, confirmed: bool, seed: dict | None = None) -> None:
        from app.services.fleet_registry import CANONICAL_DOMAINS
        from app.services.initial_pose_session import save_session, utc_now

        data_dir = getattr(self.settings, 'data_dir', None)
        if data_dir is None:
            return
        target = self.state.get('initial_pose_target')
        domain = CANONICAL_DOMAINS.get(self.primary_id, 24)
        if seed and seed.get('ros_domain_id') is not None:
            domain = int(seed['ros_domain_id'])
        payload = {
            'robot_id': self.primary_id,
            'ros_domain_id': domain,
            'map_sha256': None if seed is None else seed.get('map_sha256'),
            'sent': True,
            'confirmed': confirmed,
            'attempts': self._initial_pose_attempts,
            'sent_at': utc_now(),
            'target': target,
            'source': self.state.get('initial_pose_source'),
            'topic': str(getattr(self.settings, 'physical_initial_pose_topic', '/initialpose')),
        }
        if seed is not None and seed.get('map_sha256'):
            payload['map_sha256'] = seed.get('map_sha256')
        elif self.state.get('expected_map_sha256'):
            payload['map_sha256'] = self.state.get('expected_map_sha256')
        save_session(Path(data_dir), self.primary_id, payload)

    async def _ensure_localization(self) -> bool:
        """AMCL active + map match assumed. Seed via /initialpose only; never require Nav2 ready."""
        if self.primary_id != 'pinky-1':
            return True
        from app.services.initial_pose_registry import resolve_registered_initial_pose

        seed = resolve_registered_initial_pose(self.settings, robot_id=self.primary_id)
        self.state['initial_pose_auto'] = bool(seed.get('auto_enabled'))
        if seed.get('valid'):
            self.state['initial_pose_target'] = {
                'frame_id': 'map',
                'x': seed['x'], 'y': seed['y'],
                'yaw_rad': seed['yaw_rad'], 'yaw_deg': seed['yaw_deg'],
            }
            self.state['initial_pose_source'] = seed.get('source')
        else:
            self.state['initial_pose_source'] = seed.get('reason')

        # Existing live localization — never overwrite.
        if await self._localization_observed():
            self.state['initial_pose_confirmed'] = True
            self.state['localization_ready'] = True
            self.state['tf_ready'] = True
            if self._initial_pose_sent:
                self.state['initial_pose_state'] = 'observation_confirmed'
                self._persist_seed_session(confirmed=True, seed=seed if seed.get('valid') else None)
            else:
                self.state['initial_pose_state'] = 'existing_localization'
            return True

        self.state['localization_ready'] = False
        self.state['tf_ready'] = False
        self.state['initial_pose_confirmed'] = False

        if self._initial_pose_sent:
            self.state['initial_pose_state'] = 'sent_waiting_for_observation'
            self._update(
                'waiting_for_initial_pose_observation',
                '초기 위치 Topic 전송됨 · /amcl_pose 또는 map→base_footprint 신규 관측 대기',
            )
            return False

        if not seed.get('valid'):
            self.state['initial_pose_state'] = 'initial_pose_unregistered'
            self._update(
                'waiting_for_initial_pose',
                seed.get('detail') or '등록된 초기 위치 없음 · 수동 API 또는 RViz 필요',
            )
            return False

        if not bool(seed.get('auto_enabled')):
            self.state['initial_pose_state'] = 'awaiting_operator'
            self._update(
                'waiting_for_initial_pose',
                f'자동 초기화 꺼짐 · 등록값 map ({seed["x"]:.3f},{seed["y"]:.3f}) '
                f'yaw={seed["yaw_deg"]:.1f}° · 수동 API 대기',
            )
            return False

        if not self.state.get('map_matches_fms'):
            self.state['initial_pose_state'] = 'map_mismatch_blocks_seed'
            self._update(
                'waiting_for_initial_pose',
                '지도 불일치로 자동 초기 위치 전송 거절: ' + str(self.state.get('map_mismatch_detail') or ''),
            )
            return False

        result = await self.send_initial_pose(
            x=float(seed['x']), y=float(seed['y']),
            yaw_rad=float(seed['yaw_rad']), force=False, source=str(seed.get('source')),
        )
        return bool(result.get('accepted')) and False  # sent ≠ confirmed

    async def send_initial_pose(
        self,
        *,
        x: float | None = None,
        y: float | None = None,
        yaw_deg: float | None = None,
        yaw_rad: float | None = None,
        force: bool = False,
        source: str = 'operator_api',
    ) -> dict:
        """Publish /initialpose once. Does not call /set_initial_pose (single path)."""
        from app.services.initial_pose_registry import (
            pose_with_covariance_dict,
            resolve_registered_initial_pose,
        )

        if self.primary_id != 'pinky-1':
            return {'accepted': False, 'reason': 'robot_mismatch'}
        if await self._localization_observed() and not force:
            self.state['initial_pose_confirmed'] = True
            self.state['localization_ready'] = True
            self.state['initial_pose_state'] = 'existing_localization'
            return {
                'accepted': False,
                'reason': 'existing_localization',
                'detail': 'Valid map localization already observed; pass force=true to overwrite',
                'pose': self.state.get('pose'),
            }

        seed = resolve_registered_initial_pose(self.settings, robot_id=self.primary_id)
        if x is None or y is None:
            if not seed.get('valid'):
                return {'accepted': False, 'reason': seed.get('reason') or 'initial_pose_unregistered',
                        'detail': seed.get('detail')}
            x = float(seed['x'])
            y = float(seed['y'])
            if yaw_rad is None and yaw_deg is None:
                yaw_rad = float(seed['yaw_rad'])
            source = source if source != 'operator_api' else str(seed.get('source'))
        if yaw_rad is None:
            yaw_rad = math.radians(float(yaw_deg if yaw_deg is not None else 0.0))

        max_attempts = int(getattr(self.settings, 'physical_initial_pose_max_attempts', 3) or 3)
        retry_s = float(getattr(self.settings, 'physical_initial_pose_retry_s', 12.0) or 12.0)
        if self._initial_pose_attempts >= max_attempts and not force:
            self.state['initial_pose_state'] = 'failed_after_max_attempts'
            return {'accepted': False, 'reason': 'failed_after_max_attempts'}
        if (time.monotonic() - self._initial_pose_last_attempt) < retry_s and self._initial_pose_attempts > 0 and not force:
            self.state['initial_pose_state'] = 'retry_wait'
            return {'accepted': False, 'reason': 'retry_wait'}

        if not self.state.get('map_matches_fms') and not force:
            return {
                'accepted': False,
                'reason': 'map_mismatch',
                'detail': self.state.get('map_mismatch_detail') or 'map identity not verified',
            }

        message = pose_with_covariance_dict({
            'x': float(x), 'y': float(y), 'z': 0.0, 'yaw_rad': float(yaw_rad),
            'covariance': seed.get('covariance') if seed.get('valid') else None,
        })
        topic = str(getattr(self.settings, 'physical_initial_pose_topic', '/initialpose'))
        self.state['initial_pose_target'] = {
            'frame_id': 'map', 'x': float(x), 'y': float(y),
            'yaw_rad': float(yaw_rad), 'yaw_deg': math.degrees(float(yaw_rad)),
        }
        self.state['initial_pose_source'] = source
        self._initial_pose_last_attempt = time.monotonic()
        self._initial_pose_attempts += 1
        # Discard any pre-seed samples as confirmation evidence.
        self._seed_cutoff_monotonic = time.monotonic()
        self.state['pose'] = None
        self.state['pose_checked_at'] = None
        self.state['tf_ready'] = False
        self.state['localization_ready'] = False
        self.state['initial_pose_confirmed'] = False

        ok, output = await self._ros(
            'ros2', 'topic', 'pub', '--once', topic,
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            json.dumps(message),
            timeout=10.0,
        )
        if not ok:
            self.state['initial_pose_state'] = 'request_failed'
            log.warning('Pinky-1 /initialpose publish failed: %s', output[-800:])
            return {'accepted': False, 'reason': 'request_failed', 'detail': output[-400:]}

        self._initial_pose_sent = True
        self.state['initial_pose_sent'] = True
        self.state['initial_pose_state'] = 'sent_waiting_for_observation'
        self._persist_seed_session(confirmed=False, seed=seed if seed.get('valid') else {
            'ros_domain_id': 24, 'map_sha256': self.state.get('expected_map_sha256'),
        })
        log.info(
            'Pinky-1 /initialpose published at X=%.3f Y=%.3f yaw_rad=%.3f source=%s',
            x, y, yaw_rad, source,
        )
        self._update(
            'initial_pose_sent',
            f'/initialpose 전송 ({x:.3f},{y:.3f}) · 관측 확인 대기 (전송≠추정 확인)',
        )
        return {
            'accepted': True,
            'reason': 'sent',
            'topic': topic,
            'target': self.state['initial_pose_target'],
            'source': source,
            'note': 'Publish success is not proven localization',
        }

    async def _ensure_nav2_after_localization(self, nodes: set[str]) -> bool:
        """Start or resume Nav2 only after map localization is observed."""
        if not self.state.get('localization_ready'):
            return False
        existing = [n for n in NAV_NODES if '/' + n in nodes]
        if not existing:
            nav_launch = ROOT / 'launch/pinky1_navigation.launch.py'
            if not nav_launch.is_file():
                self._update('nav2_launch_missing', f'Nav2 launch 파일 없음: {nav_launch}')
                return False
            if not self._spawn('navigation', ['ros2', 'launch', str(nav_launch)]):
                self._update('nav2_launch_failed', 'Nav2 실행 실패; logs/physical-nav2-navigation.log 확인')
            else:
                self._update('starting_nav2', '위치 확인 후 Nav2 launch 시작 · lifecycle_manager 소유')
            return False

        # lifecycle_manager owns transitions; FMS only resumes inactive nodes after Aborting.
        inactive: list[tuple[str, str]] = []
        for name in NAV_NODES:
            state = await self._lifecycle(name)
            if state == 'active':
                continue
            if state in {'inactive', 'unconfigured'}:
                inactive.append((name, state))
            elif state == 'unavailable' and '/' + name not in nodes:
                self._update('existing_nav2_partial', f'Nav2 노드 일부 부재: {name}')
                return False

        if not inactive and len(existing) >= len(NAV_NODES):
            return True

        max_attempts = int(getattr(self.settings, 'physical_nav2_activate_max_attempts', 5) or 5)
        if self._nav2_activate_attempts >= max_attempts:
            self._update(
                'nav2_bringup_failed',
                f'Nav2 재활성화 상한({max_attempts}) 초과 · inactive={ [n for n,_ in inactive] }',
            )
            return False
        if time.monotonic() - self._nav2_activate_last < 8.0:
            self._update('nav2_resume_wait', 'Nav2 재활성화 재시도 대기')
            return False

        self._nav2_activate_last = time.monotonic()
        self._nav2_activate_attempts += 1
        self.state['nav2_resume_attempts'] = self._nav2_activate_attempts
        for name, state in inactive:
            if state == 'unconfigured':
                await self._ros('ros2', 'lifecycle', 'set', '/' + name, 'configure')
                after = await self._lifecycle(name)
                if after == 'active':
                    continue
                if after != 'inactive':
                    continue
            await self._ros('ros2', 'lifecycle', 'set', '/' + name, 'activate')
        self._update(
            'resuming_nav2',
            f'TF 복구 후 Nav2 inactive 재활성화 시도 #{self._nav2_activate_attempts}: '
            f'{[n for n, _ in inactive]}',
        )
        return False

    async def poll(self) -> None:
        from app.api.physical import check_ssh, parse_targets
        targets = dict(parse_targets(self.settings.physical_ssh_targets))
        target = targets.get(self.primary_id)
        # Preserve seed/confirmation flags across polls; refresh observation flags.
        preserved = {
            'initial_pose_sent': self.state.get('initial_pose_sent'),
            'initial_pose_confirmed': self.state.get('initial_pose_confirmed'),
            'initial_pose_state': self.state.get('initial_pose_state'),
            'initial_pose_target': self.state.get('initial_pose_target'),
            'initial_pose_source': self.state.get('initial_pose_source'),
            'nav2_resume_attempts': self.state.get('nav2_resume_attempts'),
        }
        self.state.update(ssh_connected=False, ros_online=False, map_server_active=False,
                          amcl_active=False, nav2_active=False, tf_ready=False,
                          localization_ready=False,
                          pose=None, pose_checked_at=None, pose_source=None,
                          cmd_vel_chain_ready=False, nav2_ready=False,
                          map_matches_fms=False, map_server_yaml=None,
                          collision_monitor_active=False,
                          map_content_sha256=None, expected_map_sha256=None,
                          map_mismatch_detail='map identity not checked',
                          launch_owned_by_fms='navigation' in self.children and
                          self.children['navigation'].poll() is None,
                          initial_pose_auto=bool(getattr(self.settings, 'physical_initial_pose_auto', False)),
                          bringup_phase='poll',
                          **preserved)
        # SSH is optional diagnostics for ros_topic mode. Never gate ROS observation.
        if target:
            authenticated = await check_ssh(
                self.primary_id, target,
                self.settings.physical_ssh_timeout_seconds,
                self.settings.physical_ssh_password,
            )
            self.state['ssh_connected'] = authenticated is not None
            if authenticated is None:
                self._ssh_failure_checks += 1
            else:
                self._ssh_failure_checks = 0
        else:
            self.state['ssh_connected'] = False
        if not ROS_SETUP.is_file():
            self._update('ros_missing', '/opt/ros/jazzy/setup.bash 없음')
            return
        if not await self._publisher('/odom') or not await self._publisher('/scan'):
            detail = 'Pinky Bringup /odom, /scan 발행 대기'
            if not self.state['ssh_connected']:
                detail += ' (SSH 미연결 · ROS Domain 관측만 시도)'
            self._update('waiting_for_sensors', detail)
            return
        self.state['ros_online'] = True
        self.state['bringup_phase'] = 'sensors_ok'
        nodes = await self._nodes()
        map_yaml = Path(self.settings.physical_map_yaml)
        if not map_yaml.is_file():
            self._update('map_missing', f'지도 파일 없음: {map_yaml}')
            return
        if not await self._ensure_lifecycle('map_server', nodes,
                ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
                 '-p', 'use_sim_time:=false', '-p', 'yaml_filename:=' + str(map_yaml)]):
            return
        self.state['map_server_active'] = True
        self.state['bringup_phase'] = 'map_server_active'
        await self._record_map_identity(map_yaml)
        if not self.state.get('map_matches_fms'):
            self._update('map_mismatch', str(self.state.get('map_mismatch_detail') or 'map mismatch'))
            return
        if not await self._ensure_lifecycle('amcl', nodes,
                ['ros2', 'run', 'nav2_amcl', 'amcl', '--ros-args',
                 '-p', 'use_sim_time:=false',
                 '-p', 'base_frame_id:=base_footprint',
                 '-p', 'odom_frame_id:=odom', '-p', 'global_frame_id:=map',
                 '-p', 'scan_topic:=scan', '-p', 'set_initial_pose:=false',
                 '-p', 'tf_broadcast:=true']):
            return
        self.state['amcl_active'] = True
        self.state['bringup_phase'] = 'amcl_active'

        # Initial pose BEFORE Nav2 bringup — breaks the map-TF wait cycle.
        if not await self._ensure_localization():
            return
        self.state['bringup_phase'] = 'localization_ready'

        if not await self._ensure_nav2_after_localization(nodes):
            return

        for name in ('controller_server', 'planner_server', 'bt_navigator',
                     'velocity_smoother', 'collision_monitor'):
            if await self._lifecycle(name) != 'active':
                self._update('activating_nav2', f'{name} 아직 active 아님 · lifecycle_manager/재개 대기')
                return
        self.state['collision_monitor_active'] = True
        self.state['nav2_active'] = True
        self._nav2_activate_attempts = 0
        action_ok, action_list = await self._ros('ros2', 'action', 'list', '-t')
        if not action_ok or '/navigate_to_pose [nav2_msgs/action/NavigateToPose]' not in action_list:
            self._update('waiting_for_nav2_action', 'NavigateToPose Action Server 확인 대기')
            return
        self.state['cmd_vel_chain_ready'] = await self._verify_chain()
        self.state['nav2_ready'] = self.state['cmd_vel_chain_ready']
        self.state['bringup_phase'] = 'nav2_ready' if self.state['nav2_ready'] else 'velocity_chain_missing'
        ssh_note = 'SSH 연결' if self.state['ssh_connected'] else 'SSH 미연결(선택)'
        if self.state['nav2_ready']:
            self._update('ready', f'{ssh_note} · ROS / 지도 / 위치관측 / Nav2 / cmd_vel 확인')
        else:
            self._update('velocity_chain_missing', 'Nav2 활성화됐지만 /cmd_vel 연결 확인 실패')

    # FMS_DIRECT_5CM_LIVE_POSE_V1: persistent read-only ROS 2 TF observer.
    async def run_live_pose(self) -> None:
        import json
        import shlex
        script = Path(__file__).with_name('live_tf_worker.py')
        while True:
            process = None
            try:
                cmd = ('source /opt/ros/jazzy/setup.bash && exec /usr/bin/python3 -u '
                       + shlex.quote(str(script)))
                process = await asyncio.create_subprocess_exec(
                    '/bin/bash', '-c', cmd, env=self._env(),
                    stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.DEVNULL,
                )
                while True:
                    data = await asyncio.wait_for(process.stdout.readline(), timeout=6.0)
                    if not data:
                        break
                    try:
                        message = json.loads(data)
                    except (ValueError, UnicodeDecodeError):
                        continue
                    if not isinstance(message, dict):
                        continue
                    pose = message.get('pose')
                    self._live_pose_seen = True
                    self._live_observer_down = False
                    if isinstance(pose, dict) and all(
                        isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k])
                        for k in ('x', 'y', 'yaw')
                    ):
                        self._live_lookup_failed = False
                        self._live_pose = {k: float(pose[k]) for k in ('x', 'y', 'yaw')}
                        self._live_pose_checked_at = message.get('pose_checked_at')
                    else:
                        self._live_lookup_failed = True
                        self._live_pose = None
                        self._live_pose_checked_at = None
            except asyncio.CancelledError:
                raise
            except (OSError, asyncio.TimeoutError) as exc:
                log.warning('Pinky TF pose observer restart: %s', type(exc).__name__)
            finally:
                self._live_pose = None
                self._live_pose_checked_at = None
                self._live_observer_down = True
                if process is not None and process.returncode is None:
                    process.terminate()
                    try:
                        await asyncio.wait_for(process.wait(), timeout=2.0)
                    except asyncio.TimeoutError:
                        process.kill()
                        await process.wait()
            await asyncio.sleep(3.0)

    async def run(self) -> None:
        log.info('FMS physical Nav2 auto-connect enabled for %s', self.primary_id)
        while True:
            try:
                await self.poll()
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.exception('Physical Nav2 monitor error')
                self.state['nav2_ready'] = False
                self._update('error', type(exc).__name__ + ': ' + str(exc)[:120])
            finally:
                # Publish all flags + state + checked_at from the SAME poll.
                self._published_state = dict(self.state)
            await asyncio.sleep(max(3.0, self.settings.physical_nav2_poll_seconds))

    async def shutdown(self) -> None:
        # Stop only children started by this FMS process, never user-owned ROS
        # sessions. A read-only monitor shutdown never publishes stop/motion.
        for name, child in list(self.children.items()):
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for child in self.children.values():
            try:
                await asyncio.to_thread(child.wait, timeout=4)
            except subprocess.TimeoutExpired:
                # Leave shutdown to OS/process supervisor rather than killing
                # orphans aggressively while another ROS component uses them.
                log.warning('ROS child did not exit promptly: pid=%d', child.pid)
        self.children.clear()
