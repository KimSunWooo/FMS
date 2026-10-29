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
            'initial_pose_target': {'frame_id': 'map', 'x': 1.696, 'y': 0.103,
                                    'yaw_deg': float(settings.physical_initial_pose_yaw_deg)},
            'initial_pose_sent': False,
            'initial_pose_confirmed': False,
            'initial_pose_state': 'waiting_for_ssh',
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
        self._ssh_failure_checks = 0

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
        closed_gates = [
            name for name in ('ssh_connected', 'amcl_active', 'initial_pose_confirmed')
            if not result.get(name)
        ]
        gates_open = not closed_gates
        result['live_tf_withheld_reason'] = ''
        if live_status == 'fresh' and gates_open and isinstance(self._live_pose, dict):
            result['pose'] = dict(self._live_pose)
            result['pose_checked_at'] = self._live_pose_checked_at
            result['live_tf_status'] = 'fresh'
        elif live_status == 'fresh' and not gates_open:
            result['pose'] = None
            result['pose_checked_at'] = None
            result['live_tf_status'] = 'withheld'
            result['live_tf_withheld_reason'] = (
                'fresh TF withheld because ' + ', '.join(closed_gates) + ' is false'
            )
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

    async def _ensure_lifecycle(self, name: str, nodes: set[str], cmd: list[str]) -> bool:
        if '/' + name not in nodes:
            if not self._spawn(name, cmd):
                self._update('launch_failed', f'{name} 실행 실패; logs/physical-nav2-{name}.log 확인')
            else:
                self._update('starting_' + name, f'{name} 프로세스 시작 중')
            return False
        state = await self._lifecycle(name)
        if state == 'active':
            return True
        if state == 'unconfigured':
            await self._ros('ros2', 'lifecycle', 'set', '/' + name, 'configure')
            self._update('configuring_' + name, f'{name} 구성 중')
            return False
        if state == 'inactive':
            await self._ros('ros2', 'lifecycle', 'set', '/' + name, 'activate')
            self._update('activating_' + name, f'{name} 활성화 중')
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
        self.state['pose'] = pose
        self.state['pose_checked_at'] = (
            datetime.now(timezone.utc).isoformat() if pose is not None else None
        )
        return pose is not None

    # FMS_PINKY1_INITIAL_POSE_V1
    async def _ensure_pinky1_initial_pose(self) -> bool:
        """Apply fixed map pose once per SSH session, after AMCL is active.

        Service response means delivery to AMCL's callback, NOT proven localization.
        The latter also needs a live map->base_footprint transform.
        """
        if self.primary_id != 'pinky-1':
            return True
        if self._initial_pose_sent:
            verified = await self._tf()
            self.state['initial_pose_confirmed'] = verified
            self.state['initial_pose_state'] = ('tf_confirmed' if verified else 'sent_waiting_for_tf')
            return True
        if self._initial_pose_attempts >= 3:
            self.state['initial_pose_state'] = 'failed_after_three_attempts'
            self._update('initial_pose_failed',
                         'pinky-1 초기 위치 설정 3회 실패; AMCL 서비스/로그 확인')
            return False
        if time.monotonic() - self._initial_pose_last_attempt < 12.0:
            self.state['initial_pose_state'] = 'retry_wait'
            self._update('initial_pose_retry_wait', 'AMCL 초기 위치 서비스 재시도 대기')
            return False

        # User-provided fixed start XY, ROS map frame. Heading is not in the
        # screenshot; configured default is 0 degrees (+X). Robot remains still.
        yaw = math.radians(self.settings.physical_initial_pose_yaw_deg)
        covariance = [0.0] * 36
        covariance[0] = 0.01   # 0.10m stddev X
        covariance[7] = 0.01   # 0.10m stddev Y
        covariance[35] = (math.pi / 12.0) ** 2  # 15 degrees stddev heading
        message = {
            'pose': {
                'header': {'frame_id': 'map'},
                'pose': {
                    'pose': {
                        'position': {'x': 1.696, 'y': 0.103, 'z': 0.0},
                        'orientation': {
                            'x': 0.0, 'y': 0.0,
                            'z': math.sin(yaw / 2.0),
                            'w': math.cos(yaw / 2.0),
                        },
                    },
                    'covariance': covariance,
                },
            },
        }
        self._initial_pose_last_attempt = time.monotonic()
        self._initial_pose_attempts += 1
        ok, output = await self._ros(
            'ros2', 'service', 'call', '/set_initial_pose',
            'nav2_msgs/srv/SetInitialPose', json.dumps(message), timeout=12.0,
        )
        if not ok or 'response:' not in output:
            self.state['initial_pose_state'] = 'request_failed'
            log.warning('Pinky-1 AMCL initial pose service failed: %s', output[-800:])
            self._update('initial_pose_request_failed',
                         'pinky-1 AMCL /set_initial_pose 요청 실패 (최대 3회 제한)')
            return False
        self._initial_pose_sent = True
        self.state['initial_pose_sent'] = True
        self.state['initial_pose_state'] = 'sent_waiting_for_tf'
        self._update('initial_pose_sent',
                     'pinky-1 AMCL 초기 위치 X=1.696, Y=0.103 전송; TF 확인 대기')
        log.info('Pinky-1 initial pose delivered to AMCL at X=1.696 Y=0.103')
        return True

    async def poll(self) -> None:
        from app.api.physical import check_ssh, parse_targets
        targets = dict(parse_targets(self.settings.physical_ssh_targets))
        target = targets.get(self.primary_id)
        self.state.update(ssh_connected=False, ros_online=False, map_server_active=False,
                          amcl_active=False, nav2_active=False, tf_ready=False,
                          pose=None, pose_checked_at=None,
                          cmd_vel_chain_ready=False, nav2_ready=False,
                          map_matches_fms=False, map_server_yaml=None,
                          collision_monitor_active=False,
                          map_content_sha256=None, expected_map_sha256=None,
                          map_mismatch_detail='map identity not checked',
                          launch_owned_by_fms='navigation' in self.children and
                          self.children['navigation'].poll() is None)
        if not target:
            self._update('unconfigured', 'FMS_PHYSICAL_SSH_TARGETS에 대상 Pinky 없음')
            return
        authenticated = await check_ssh(self.primary_id, target,
                                        self.settings.physical_ssh_timeout_seconds,
                                        self.settings.physical_ssh_password)
        if authenticated is None:
            self._ssh_failure_checks += 1
            if self._ssh_failure_checks >= 2:
                # Re-arm only after two failed probes; avoid reinitializing
                # localization on a single transient SSH timeout.
                self._initial_pose_sent = False
                self._initial_pose_attempts = 0
                self._initial_pose_last_attempt = -1e9
                self.state['initial_pose_sent'] = False
                self.state['initial_pose_confirmed'] = False
                self.state['initial_pose_state'] = 'waiting_for_ssh'
            self._update('waiting_for_ssh', 'SSH 인증 실패 또는 연결 끊김')
            return
        self._ssh_failure_checks = 0
        self.state['ssh_connected'] = True
        if not ROS_SETUP.is_file():
            self._update('ros_missing', '/opt/ros/jazzy/setup.bash 없음')
            return
        # The physical robot's bringup must be running; never remote-start the
        # motor interface just because SSH authentication succeeded.
        if not await self._publisher('/odom') or not await self._publisher('/scan'):
            self._update('waiting_for_robot_ros', 'SSH 연결됨; Pinky Bringup /odom, /scan 발행 대기')
            return
        self.state['ros_online'] = True
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
        # Active only means the lifecycle state. An already-running server may
        # still be serving a different YAML. Do not restart a user-owned process.
        await self._record_map_identity(map_yaml)
        if not await self._ensure_lifecycle('amcl', nodes,
                ['ros2', 'run', 'nav2_amcl', 'amcl', '--ros-args',
                 '-p', 'use_sim_time:=false',
                 '-p', 'base_frame_id:=base_footprint',
                 '-p', 'odom_frame_id:=odom', '-p', 'global_frame_id:=map',
                 '-p', 'scan_topic:=scan', '-p', 'set_initial_pose:=false',
                 '-p', 'tf_broadcast:=true']):
            return
        self.state['amcl_active'] = True
        if not await self._ensure_pinky1_initial_pose():
            return
        # Never spawn duplicate Nav2 nodes over an existing manual launch.
        existing_nav = [n for n in NAV_NODES if '/' + n in nodes]
        if existing_nav:
            if len(existing_nav) < len(NAV_NODES):
                self._update('existing_nav2_partial',
                             '기존 Nav2 일부 노드가 실행 중; 중복 실행 방지 (로그 확인)')
                return
        else:
            nav_launch = ROOT / 'launch/pinky1_navigation.launch.py'
            if not nav_launch.is_file():
                self._update('nav2_launch_missing', f'Nav2 launch 파일 없음: {nav_launch}')
                return
            if not self._spawn('navigation', ['ros2', 'launch', str(nav_launch)]):
                self._update('nav2_launch_failed', 'Nav2 실행 실패; logs/physical-nav2-navigation.log 확인')
            else:
                self._update('starting_nav2', 'Nav2 자동 실행 시작; Lifecycle 활성화 대기')
            return
        for name in ('controller_server', 'planner_server', 'bt_navigator',
                     'velocity_smoother', 'collision_monitor'):
            if await self._lifecycle(name) != 'active':
                self._update('activating_nav2', f'{name} 활성화 대기')
                return
        self.state['collision_monitor_active'] = True
        self.state['nav2_active'] = True
        action_ok, action_list = await self._ros('ros2', 'action', 'list', '-t')
        if not action_ok or '/navigate_to_pose [nav2_msgs/action/NavigateToPose]' not in action_list:
            self._update('waiting_for_nav2_action', 'NavigateToPose Action Server 확인 대기')
            return
        # Even if Nav2 has activated, AMCL must establish map -> base_footprint.
        self.state['tf_ready'] = await self._tf()
        if self._initial_pose_sent:
            self.state['initial_pose_confirmed'] = self.state['tf_ready']
            self.state['initial_pose_state'] = ('tf_confirmed' if self.state['tf_ready']
                                               else 'sent_waiting_for_tf')
        if not self.state['tf_ready']:
            self._update('waiting_for_initial_pose',
                         'Nav2 활성화됨; RViz 2D Pose Estimate로 AMCL 초기 위치 설정 필요')
            return
        self.state['cmd_vel_chain_ready'] = await self._verify_chain()
        self.state['nav2_ready'] = self.state['cmd_vel_chain_ready']
        if self.state['nav2_ready']:
            self._update('ready', 'SSH / ROS / 지도 / AMCL TF / Nav2 / cmd_vel 연결 확인')
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
