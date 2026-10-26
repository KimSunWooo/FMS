from __future__ import annotations

import json
import sys
import threading
import uuid
from datetime import timedelta

from sqlalchemy import func, select, text
from sqlalchemy.exc import IntegrityError

from app.core.clock import Clock
from app.core.config import ROOT, Settings
from app.core.database import Database
from app.core.errors import (
    FmsError,
    IdempotencyMismatchError,
    LeaseRejectedError,
    NotFoundError,
    PhysicalControlBlocked,
    StaleOwnerError,
    VersionConflictError,
    ZoneConflictError,
)
from app.core.logging import configure_logging, get_logger
from app.core.process_lock import ProcessLock
from app.core.state_machine import IN_FLIGHT, OCCUPYING, TaskStatus
from app.models.tables import NavigationGoal, Robot, RobotTelemetry, SystemEvent, Task, TaskEvent, TrafficZone, ZoneReservation
from app.robot.mock_agent import MockFleet
from app.robot.mock_gateway import MockGateway
from app.robot.safety import SafetyGuard
from app.scheduler.scheduler import Scheduler
from app.services.read_model import event_feed, reservation_out, robot_out, task_out, zone_out
from app.services.reconciliation import ReconciliationWorker
from app.services.task_manager import TaskManager, _same_request, refresh_robot
from app.traffic import coordinator as traffic

log = get_logger()


class Runtime:
    def __init__(self, settings: Settings):
        self.settings = settings
        self.clock = Clock()
        self.db = Database(settings.database_url)
        self.safety = SafetyGuard(execution_mode=settings.execution_mode)
        self.gateway = MockGateway(
            environment_id=settings.environment_id,
            secret=settings.lease_secret,
            ttl_seconds=settings.lease_ttl_seconds,
        )
        self.agent = MockFleet()
        self.tasks = TaskManager(settings, self.gateway, self.safety)
        self.scheduler = Scheduler(settings, self.gateway, self.agent)
        self.reconciliation = ReconciliationWorker(settings, self.tasks, self.gateway, self.agent)
        self.lock = ProcessLock(settings.data_dir / "fms.lock")
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.started = False

    def startup(self) -> None:
        self.settings.data_dir.mkdir(parents=True, exist_ok=True)
        self.settings.log_dir.mkdir(parents=True, exist_ok=True)
        configure_logging(self.settings.log_dir)
        if self.settings.using_placeholder_secret:
            log.warning("using a development placeholder lease secret; it is not a Care-Move secret")
        if self.settings.process_lock:
            self.lock.acquire()
        self.db.migrate()
        now = self.clock.now()
        with self.db.write() as session:
            traffic.seed_zones(session, self.settings.zones_path, now)
            if self.settings.seed_fleet:
                self._seed_fleet(session, now)
            robots = list(session.scalars(select(Robot)).all())
            for robot in robots:
                if robot.mode == "mock" and robot.connection_status == "online" and not robot.estop_latched:
                    self.agent.start(session, robot, now)
            self.reconciliation.run(session, now, process_restart=True)
            robots = list(session.scalars(select(Robot)).all())
            for robot in robots:
                self._restore_idle_pose(session, robot, now)
            session.add(
                SystemEvent(
                    level="info",
                    code="fms_started",
                    message="Care-Move-FMS process started",
                    payload_json=json.dumps(
                        {
                            "environment_id": self.settings.environment_id,
                            "gateway_boot_id": self.gateway.boot_id,
                            "physical_control": "blocked",
                            "ros_bridge": "disabled",
                            "execution_mode": self.settings.execution_mode,
                            "rmf": "not_used",
                            "ros_domain_id": self.settings.ros_domain_id,
                        }
                    ),
                    created_at=now,
                )
            )
        self.started = True
        if self.settings.auto_tick:
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, name="fms-tick", daemon=True)
            self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        if self.settings.process_lock:
            self.lock.release()
        self.started = False

    def tick(self) -> None:
        now = self.clock.now()
        with self.db.write() as session:
            self.agent.expire_stale(session, now, self.settings.heartbeat_timeout_seconds)
            self.reconciliation.run(session, now, process_restart=False)
            self.scheduler.schedule_pending(session, now)
        self._dispatch(now)
        self._pump(now)
        with self.db.write() as session:
            for robot in session.scalars(select(Robot)).all():
                if robot.pose_trustworthy:
                    traffic.sync_pose_occupancy(
                        session,
                        robot,
                        now,
                        reservation_ttl=max(self.settings.lease_ttl_seconds * 4, 120),
                    )

    def create_task(self, **kwargs) -> dict:
        key = kwargs.get("idempotency_key")
        try:
            with self.db.write() as session:
                task = self.tasks.create_task(session, now=self.clock.now(), **kwargs)
                return task_out(session, task)
        except IntegrityError:
            if not key:
                raise
            with self.db.read() as session:
                existing = session.scalar(select(Task).where(Task.idempotency_key == key))
                if existing is not None and _same_request(
                    existing,
                    x=kwargs["x"],
                    y=kwargs["y"],
                    yaw=kwargs["yaw"],
                    label=kwargs["label"],
                    priority=kwargs["priority"],
                    robot_id=kwargs.get("robot_id"),
                ):
                    return task_out(session, existing)
            raise IdempotencyMismatchError("idempotency key already exists for another payload")

    def cancel_task(self, task_id: str) -> dict:
        with self.db.write() as session:
            task = self.tasks.cancel(session, task_id, self.clock.now())
            return task_out(session, task)

    def resolve_task(self, task_id: str, **kwargs) -> dict:
        with self.db.write() as session:
            task = self.tasks.resolve_recovery(session, task_id, now=self.clock.now(), **kwargs)
            return task_out(session, task)

    def apply_external_result(self, **kwargs) -> dict:
        with self.db.write() as session:
            task = self.tasks.apply_result(session, now=self.clock.now(), **kwargs)
            return task_out(session, task)

    def register_robot(self, **kwargs) -> dict:
        now = self.clock.now()
        self.safety.assert_backend_allowed(kwargs["mode"])
        self.safety.assert_coordinates(kwargs["x"], kwargs["y"], kwargs["yaw"])
        with self.db.write() as session:
            if session.get(Robot, kwargs["robot_id"]) is not None:
                raise FmsError(f"robot {kwargs['robot_id']} is already registered")
            robot = self._new_robot(kwargs, now, source="registry")
            session.add(robot)
            session.flush()
            if robot.mode == "mock":
                self.agent.start(session, robot, now)
            robot.pose_trustworthy = True
            traffic.sync_pose_occupancy(
                session, robot, now, reservation_ttl=max(self.settings.lease_ttl_seconds * 4, 120)
            )
            return robot_out(session, robot)

    def set_offline(self, robot_id: str) -> dict:
        now = self.clock.now()
        with self.db.write() as session:
            robot = self._require_robot(session, robot_id)
            robot.connection_status = "offline"
            robot.controller_ready = False
            self.agent.stop(session, robot, now)
            refresh_robot(session, robot)
            traffic.block_robot_zones(session, robot.id, now, "fail_closed_robot_offline")
            self.reconciliation.run(session, now, process_restart=False)
            return robot_out(session, robot)

    def set_online(self, robot_id: str) -> dict:
        now = self.clock.now()
        with self.db.write() as session:
            robot = self._require_robot(session, robot_id)
            self.safety.assert_backend_allowed(robot.mode)
            robot.connection_status = "online"
            robot.controller_ready = True
            robot.updated_at = now
            if robot.mode == "mock":
                self.agent.start(session, robot, now)
            self._restore_idle_pose(session, robot, now)
            refresh_robot(session, robot)
            return robot_out(session, robot)

    def emergency_stop(self, robot_id: str, reason: str) -> dict:
        now = self.clock.now()
        with self.db.write() as session:
            robot = self._require_robot(session, robot_id)
            robot.estop_latched = True
            robot.fault_code = "estop"
            robot.updated_at = now
            tasks = list(
                session.scalars(
                    select(Task).where(
                        Task.robot_id == robot.id,
                        Task.status.in_([status.value for status in OCCUPYING]),
                    )
                ).all()
            )
            for task in tasks:
                if task.status == TaskStatus.RECOVERY_REQUIRED.value:
                    continue
                self.tasks.cancel(session, task.id, now)
            refresh_robot(session, robot)
            session.add(
                SystemEvent(
                    level="warning",
                    code="emergency_stop",
                    message=reason,
                    payload_json=json.dumps(
                        {
                            "robot_id": robot.id,
                            "cmd_vel_published": False,
                            "velocity_publish_count": self.safety.velocity_publish_count,
                        }
                    ),
                    created_at=now,
                )
            )
            return robot_out(session, robot)

    def clear_estop(self, robot_id: str, acknowledge: bool) -> dict:
        if not acknowledge:
            raise FmsError("clearing estop requires an explicit acknowledgement")
        now = self.clock.now()
        with self.db.write() as session:
            robot = self._require_robot(session, robot_id)
            robot.estop_latched = False
            if robot.fault_code == "estop":
                robot.fault_code = None
            robot.updated_at = now
            refresh_robot(session, robot)
            session.add(
                SystemEvent(
                    level="info",
                    code="estop_cleared",
                    message="estop latch cleared without a motion command",
                    payload_json=json.dumps({"robot_id": robot.id, "cmd_vel_published": False}),
                    created_at=now,
                )
            )
            return robot_out(session, robot)

    def mock_observe(self, robot_id: str) -> dict:
        now = self.clock.now()
        with self.db.write() as session:
            robot = self._require_robot(session, robot_id)
            if robot.mode != "mock":
                raise PhysicalControlBlocked(
                    "mock-observe is a mock snapshot. It does not unlock a Gazebo or physical robot."
                )
            self.safety.assert_backend_allowed(robot.mode)
            blocking = session.scalar(
                select(func.count())
                .select_from(Task)
                .where(
                    Task.robot_id == robot.id,
                    Task.status.in_(
                        [TaskStatus.RECOVERY_REQUIRED.value]
                        + [status.value for status in IN_FLIGHT]
                    ),
                )
            )
            if blocking:
                raise FmsError("pose stays untrusted while a goal may still be in flight")
            if not self.agent.is_alive(robot.id):
                self.agent.start(session, robot, now)
            robot.pose_trustworthy = True
            robot.pose_source = "mock-agent-snapshot"
            robot.updated_at = now
            session.add(
                RobotTelemetry(
                    robot_id=robot.id,
                    pose_x=robot.pose_x,
                    pose_y=robot.pose_y,
                    pose_yaw=robot.pose_yaw,
                    battery_percent=robot.battery_percent,
                    source="mock-agent-snapshot",
                    trustworthy=True,
                    observed_at=now,
                )
            )
            traffic.sync_pose_occupancy(
                session, robot, now, reservation_ttl=max(self.settings.lease_ttl_seconds * 4, 120)
            )
            goals = int(
                session.scalar(select(func.count()).select_from(NavigationGoal).where(NavigationGoal.robot_id == robot.id))
                or 0
            )
            session.add(
                SystemEvent(
                    level="info",
                    code="mock_pose_snapshot",
                    message="mock agent restated the stored pose; no goal was sent",
                    payload_json=json.dumps({"robot_id": robot.id, "navigation_goal_count": goals}),
                    created_at=now,
                )
            )
            return robot_out(session, robot)

    def release_reservation(self, reservation_id: str, *, reason: str, position_observed_clear: bool) -> dict:
        if not position_observed_clear:
            raise FmsError("a zone is not released from expiry or from an unconfirmed pose")
        now = self.clock.now()
        with self.db.write() as session:
            row = session.get(ZoneReservation, reservation_id)
            if row is None:
                raise NotFoundError("reservation not found")
            if row.reservation_status == "released":
                return reservation_out(row)
            traffic.release_reservation(session, row, now, "operator_confirmed_clear:" + reason)
            session.add(
                SystemEvent(
                    level="warning",
                    code="zone_released_by_operator",
                    message=reason,
                    payload_json=json.dumps({"reservation_id": reservation_id, "zone_id": row.zone_id}),
                    created_at=now,
                )
            )
            return reservation_out(row)

    def list_robots(self) -> list[dict]:
        with self.db.read() as session:
            robots = session.scalars(select(Robot).order_by(Robot.id)).all()
            return [robot_out(session, robot) for robot in robots]

    def get_robot(self, robot_id: str) -> dict:
        with self.db.read() as session:
            return robot_out(session, self._require_robot(session, robot_id))

    def list_tasks(self, status: str | None) -> list[dict]:
        with self.db.read() as session:
            query = select(Task).order_by(Task.created_at.desc())
            if status:
                query = query.where(Task.status == status)
            return [task_out(session, task) for task in session.scalars(query.limit(200)).all()]

    def get_task(self, task_id: str) -> dict:
        with self.db.read() as session:
            task = session.get(Task, task_id)
            if task is None:
                raise NotFoundError(f"task {task_id} not found")
            return task_out(session, task)

    def fleet_status(self) -> dict:
        with self.db.read() as session:
            robots = [robot_out(session, robot) for robot in session.scalars(select(Robot).order_by(Robot.id)).all()]
            counts = {
                status.value: int(
                    session.scalar(
                        select(func.count()).select_from(Task).where(Task.status == status.value)
                    )
                    or 0
                )
                for status in TaskStatus
            }
            return {
                "environment_id": self.settings.environment_id,
                "physical_control": "blocked",
                "rmf": "not_used",
                "ros_bridge": "disabled",
                "execution_mode": self.settings.execution_mode,
                "gazebo_preflight": self._preflight_summary(),
                "ros_domain_id": self.settings.ros_domain_id,
                "distance_model": self.settings.distance_model,
                "gateway": {
                    "implementation": "fms-mock-gateway",
                    "online": self.gateway.online,
                    "boot_id": self.gateway.boot_id,
                    "environment_id": self.gateway.environment_id,
                },
                "robots": robots,
                "task_counts": counts,
                "generated_at": self.clock.now().isoformat(),
            }

    def zones(self) -> dict:
        with self.db.read() as session:
            zones = [zone_out(session, zone) for zone in session.scalars(select(TrafficZone).order_by(TrafficZone.zone_id)).all()]
            return {
                "floor": {
                    "name": "care-ward-schematic",
                    "width_m": self.settings.floor_width_m,
                    "height_m": self.settings.floor_height_m,
                    "note": "schematic zones, not a Nav2 costmap",
                },
                "zones": zones,
            }

    def reservations(self) -> list[dict]:
        with self.db.read() as session:
            rows = session.scalars(select(ZoneReservation).order_by(ZoneReservation.acquired_at.desc()).limit(200)).all()
            return [reservation_out(row) for row in rows]

    def events(self, limit: int) -> list[dict]:
        with self.db.read() as session:
            return event_feed(session, limit)

    def health(self) -> dict:
        db_ok = False
        try:
            with self.db.read() as session:
                session.execute(text("SELECT 1"))
                db_ok = True
        except Exception:
            db_ok = False
        return {
            "service": "care-move-fms",
            "status": "ok" if db_ok and self.started else "starting",
            "phase": 2,
            "database": "ok" if db_ok else "error",
            "physical_control": "blocked",
            "ros_bridge": "disabled",
            "execution_mode": self.settings.execution_mode,
            "gazebo_e2e": "NOT_RUN",
            "gazebo_preflight": self._preflight_summary(),
            "rmf_runtime": "not_used",
            "environment_id": self.settings.environment_id,
            "ros_domain_id": self.settings.ros_domain_id,
            "gateway_boot_id": self.gateway.boot_id,
            "auto_tick": self.settings.auto_tick,
            "motion_backend": "mock-only" if self.settings.execution_mode == "mock" else "gazebo-armed-awaiting-nav2-result",
        }

    def readiness(self) -> dict:
        body = self.health()
        ready = body["database"] == "ok" and self.started
        body["ready"] = ready
        if not ready:
            raise FmsError("not ready")
        return body

    def open_agent_session(self, robot_id: str) -> dict:
        now = self.clock.now()
        with self.db.write() as session:
            robot = self._require_robot(session, robot_id)
            self.tasks._require_external_agent(robot)
            self.agent.expire_stale(session, now, self.settings.heartbeat_timeout_seconds)
            if self.agent.is_alive(robot.id):
                raise VersionConflictError("an agent session is already attached to this robot")
            session_id = self.agent.start(session, robot, now, external=True)
            robot.last_heartbeat_at = now
            return {"robot_id": robot.id, "session_id": session_id, "attached": True}

    def replace_agent_session(self, robot_id: str) -> dict:
        now = self.clock.now()
        with self.db.write() as session:
            robot = self._require_robot(session, robot_id)
            self.tasks._require_external_agent(robot)
            self.agent.stop(session, robot, now)
            session_id = self.agent.start(session, robot, now, external=True)
            recovered = self.reconciliation.run(session, now, process_restart=False)
            return {
                "robot_id": robot.id,
                "session_id": session_id,
                "recovered_task_ids": recovered,
                "resumed_previous_goal": False,
            }

    def agent_stop(self, robot_id: str, session_id: str) -> dict:
        now = self.clock.now()
        with self.db.write() as session:
            robot = self._require_robot(session, robot_id)
            self.tasks._require_external_agent(robot)
            if self.agent.session_id(robot.id) != session_id:
                raise StaleOwnerError("agent session does not match this robot")
            self.agent.stop(session, robot, now)
            recovered = self.reconciliation.run(session, now, process_restart=False)
            return {"robot_id": robot.id, "stopped": True, "recovered_task_ids": recovered}

    def agent_heartbeat(self, robot_id: str, session_id: str) -> dict:
        now = self.clock.now()
        with self.db.write() as session:
            robot = self._require_robot(session, robot_id)
            self.tasks._require_external_agent(robot)
            if self.agent.session_id(robot.id) != session_id:
                raise StaleOwnerError("agent session does not match this robot")
            if not self.agent.touch(robot.id, session_id, now):
                raise StaleOwnerError("agent session does not match this robot")
            robot.last_heartbeat_at = now
            robot.agent_online = True
            task = session.scalar(
                select(Task).where(
                    Task.robot_id == robot.id,
                    Task.session_id == session_id,
                    Task.status.in_([status.value for status in IN_FLIGHT]),
                )
            )
            if task is not None:
                self.tasks.heartbeat(session, task, now)
            return {
                "robot_id": robot.id,
                "session_id": session_id,
                "task_id": None if task is None else task.id,
                "status": None if task is None else task.status,
            }

    def agent_claim(self, **kwargs) -> dict:
        with self.db.write() as session:
            return self.tasks.claim_send(session, now=self.clock.now(), **kwargs)

    def agent_goal_event(self, **kwargs) -> dict:
        kind = kwargs.pop("kind")
        nav2_action_goal_id = kwargs.pop("nav2_action_goal_id", None)
        robot_id = kwargs.pop("robot_id")
        now = self.clock.now()
        with self.db.write() as session:
            robot = self._require_robot(session, robot_id)
            self.tasks._require_external_agent(robot)
            if self.agent.session_id(robot.id) != kwargs["session_id"]:
                raise StaleOwnerError("agent session does not match this robot")
            if kind == "accepted":
                task = self.tasks.note_nav2_accept(
                    session,
                    now=now,
                    nav2_action_goal_id=nav2_action_goal_id,
                    task_id=kwargs["task_id"],
                    goal_id=kwargs["goal_id"],
                    execution_id=kwargs["execution_id"],
                    owner_epoch=kwargs["owner_epoch"],
                    session_id=kwargs["session_id"],
                )
            elif kind in {"result", "disconnected"}:
                outcome = "disconnected" if kind == "disconnected" else kwargs["outcome"]
                task = self.tasks.apply_result(
                    session,
                    now=now,
                    task_id=kwargs["task_id"],
                    goal_id=kwargs["goal_id"],
                    execution_id=kwargs["execution_id"],
                    owner_epoch=kwargs["owner_epoch"],
                    session_id=kwargs["session_id"],
                    outcome=outcome,
                    pose_x=kwargs.get("pose_x") or 0.0,
                    pose_y=kwargs.get("pose_y") or 0.0,
                    pose_yaw=kwargs.get("pose_yaw") or 0.0,
                    pose_trustworthy=bool(kwargs.get("pose_trustworthy")),
                    source="gazebo-agent",
                    nav2_action_goal_id=nav2_action_goal_id,
                    nav2_action_goal_id_absent_reason=kwargs.get("nav2_action_goal_id_absent_reason"),
                )
            else:
                raise FmsError("goal event kind must be accepted, result, or disconnected")
            return task_out(session, task)

    def agent_pose(
        self,
        robot_id: str,
        session_id: str,
        pose_x: float,
        pose_y: float,
        pose_yaw: float,
        frame_id: str,
    ) -> dict:
        if frame_id != "map":
            raise FmsError("pose frame must be map")
        if any(value != value or value in (float("inf"), float("-inf")) for value in (pose_x, pose_y, pose_yaw)):
            raise FmsError("pose values must be finite")
        now = self.clock.now()
        with self.db.write() as session:
            robot = self._require_robot(session, robot_id)
            self.tasks._require_external_agent(robot)
            if self.agent.session_id(robot.id) != session_id:
                raise StaleOwnerError("agent session does not match this robot")
            blocking = session.scalar(
                select(func.count())
                .select_from(Task)
                .where(
                    Task.robot_id == robot.id,
                    Task.status.in_(
                        [TaskStatus.RECOVERY_REQUIRED.value] + [status.value for status in IN_FLIGHT]
                    ),
                )
            )
            if blocking:
                raise FmsError("pose stays untrusted while a goal may still be in flight")
            robot.pose_x = pose_x
            robot.pose_y = pose_y
            robot.pose_yaw = pose_yaw
            robot.pose_trustworthy = True
            robot.pose_source = "gazebo-tf"
            robot.agent_online = True
            robot.last_heartbeat_at = now
            robot.updated_at = now
            self.agent.touch(robot.id, session_id, now)
            session.add(
                RobotTelemetry(
                    robot_id=robot.id,
                    pose_x=pose_x,
                    pose_y=pose_y,
                    pose_yaw=pose_yaw,
                    battery_percent=robot.battery_percent,
                    source="gazebo-tf",
                    trustworthy=True,
                    observed_at=now,
                )
            )
            traffic.sync_pose_occupancy(
                session, robot, now, reservation_ttl=max(self.settings.lease_ttl_seconds * 4, 120)
            )
            return robot_out(session, robot)

    def agent_assignment(self, robot_id: str, session_id: str) -> dict:
        with self.db.read() as session:
            robot = self._require_robot(session, robot_id)
            self.tasks._require_external_agent(robot)
            if self.agent.session_id(robot.id) != session_id:
                raise StaleOwnerError("agent session does not match this robot")
            task = session.scalar(
                select(Task).where(
                    Task.robot_id == robot.id,
                    Task.session_id == session_id,
                    Task.status.in_([TaskStatus.DISPATCHING.value, TaskStatus.RUNNING.value]),
                )
            )
            if task is None:
                return {"assignment": None, "send_navigate_to_pose": False}
            goal = session.get(NavigationGoal, task.goal_id) if task.goal_id else None
            send = (
                task.status == TaskStatus.DISPATCHING.value
                and goal is not None
                and goal.status == "pending"
            )
            return {"assignment": task_out(session, task), "send_navigate_to_pose": send}

    def _preflight_summary(self) -> dict:
        root = str(ROOT)
        if root not in sys.path:
            sys.path.insert(0, root)
        from robot_agent.preflight import live_report

        report = live_report(expected_domain=self.settings.ros_domain_id)
        return {
            "status": "ready" if report.ready else "not_ready",
            "reasons": report.reasons,
            "ros_distro": report.ros_distro,
            "ros_domain_id": report.ros_domain_id,
            "rmw_implementation": report.rmw_implementation,
            "pinky_packages": report.pinky_packages,
        }

    def _dispatch(self, now) -> None:
        with self.db.read() as session:
            ids = list(
                session.scalars(
                    select(Task.id)
                    .where(Task.status == TaskStatus.ASSIGNED.value)
                    .order_by(Task.queue_seq, Task.id)
                ).all()
            )
        for task_id in ids:
            try:
                with self.db.write() as session:
                    task = session.get(Task, task_id)
                    if task is None or task.status != TaskStatus.ASSIGNED.value:
                        continue
                    robot = session.get(Robot, task.robot_id) if task.robot_id else None
                    session_id = self.agent.session_id(robot.id) if robot is not None else None
                    if robot is None or session_id is None:
                        self.tasks.abort_dispatch_before_goal(session, task, "agent_unavailable_before_goal", now)
                        continue
                    self.tasks.dispatch_assigned(session, task, session_id, now)
            except (ZoneConflictError, LeaseRejectedError, IntegrityError) as exc:
                reason = getattr(exc, "code", "dispatch_conflict")
                if not isinstance(reason, str):
                    reason = "dispatch_conflict"
                with self.db.write() as session:
                    task = session.get(Task, task_id)
                    if task is not None and task.status == TaskStatus.ASSIGNED.value:
                        self.tasks.abort_dispatch_before_goal(session, task, reason.lower(), now)

    def _pump(self, now) -> None:
        with self.db.read() as session:
            ids = list(
                session.scalars(
                    select(Task.id).where(Task.status.in_([status.value for status in IN_FLIGHT]))
                ).all()
            )
        for task_id in ids:
            with self.db.write() as session:
                task = session.get(Task, task_id)
                if task is None or task.robot_id is None:
                    continue
                robot = session.get(Robot, task.robot_id)
                if robot is None or robot.mode != "mock":
                    continue
                if not self.agent.is_alive(task.robot_id):
                    continue
                if self.agent.session_id(task.robot_id) != task.session_id:
                    continue
                if task.status == TaskStatus.DISPATCHING.value:
                    self.tasks.accept_goal(session, task, now)
                    session.flush()
                if task.status == TaskStatus.RUNNING.value:
                    self.tasks.heartbeat(session, task, now)
                    if task.status == TaskStatus.RUNNING.value:
                        self.tasks.complete_mock(session, task, now)
                elif task.status == TaskStatus.CANCEL_REQUESTED.value:
                    if task.robot_id in self.agent.suppress_cancel_ack:
                        self.tasks.heartbeat(session, task, now)
                    else:
                        self.tasks.ack_cancel(session, task, now)

    def _loop(self) -> None:
        while not self._stop.wait(self.settings.tick_interval_seconds):
            try:
                self.tick()
            except Exception:
                log.exception("tick failed")

    def _seed_fleet(self, session, now) -> None:
        if not self.settings.fleet_path.exists():
            return
        payload = json.loads(self.settings.fleet_path.read_text(encoding="utf-8"))
        for item in payload["robots"]:
            if session.get(Robot, item["robot_id"]) is not None:
                continue
            pose = item["pose"]
            session.add(
                self._new_robot(
                    {
                        "robot_id": item["robot_id"],
                        "name": item["name"],
                        "mode": "mock",
                        "battery_percent": item["battery_percent"],
                        "x": pose["x"],
                        "y": pose["y"],
                        "yaw": pose["yaw"],
                        "controller_ready": item.get("controller_ready", True),
                    },
                    now,
                    source="registry_seed",
                )
            )
        session.flush()

    def _restore_idle_pose(self, session, robot: Robot, now) -> None:
        blocking = session.scalar(
            select(func.count())
            .select_from(Task)
            .where(
                Task.robot_id == robot.id,
                Task.status.in_([TaskStatus.RECOVERY_REQUIRED.value] + [status.value for status in IN_FLIGHT]),
            )
        )
        if robot.mode != "mock":
            if blocking or robot.connection_status != "online" or robot.estop_latched:
                robot.pose_trustworthy = False
                if blocking:
                    traffic.block_robot_zones(session, robot.id, now, "fail_closed_uncertain_task")
            return
        if blocking or robot.connection_status != "online" or robot.estop_latched:
            robot.pose_trustworthy = False
            if blocking:
                traffic.block_robot_zones(session, robot.id, now, "fail_closed_uncertain_task")
            return
        if not self.agent.is_alive(robot.id):
            robot.pose_trustworthy = False
            return
        if robot.pose_source in {"untrusted_recovery", "unconfirmed_after_recovery", "agent_lost"}:
            robot.pose_trustworthy = False
            return
        robot.pose_trustworthy = True
        if robot.pose_source == "registry_seed":
            robot.pose_source = "mock-agent-snapshot"
        session.add(
            RobotTelemetry(
                robot_id=robot.id,
                pose_x=robot.pose_x,
                pose_y=robot.pose_y,
                pose_yaw=robot.pose_yaw,
                battery_percent=robot.battery_percent,
                source=robot.pose_source,
                trustworthy=True,
                observed_at=now,
            )
        )
        traffic.sync_pose_occupancy(
            session, robot, now, reservation_ttl=max(self.settings.lease_ttl_seconds * 4, 120)
        )

    def _new_robot(self, spec: dict, now, *, source: str) -> Robot:
        return Robot(
            id=spec["robot_id"],
            name=spec["name"],
            mode=spec["mode"],
            connection_status="online",
            operational_state="idle",
            battery_percent=spec["battery_percent"],
            controller_ready=spec.get("controller_ready", True),
            controller_kind="nav2" if spec["mode"] == "gazebo" else "mock",
            pose_x=spec["x"],
            pose_y=spec["y"],
            pose_yaw=spec["yaw"],
            pose_trustworthy=True,
            pose_source=source,
            fault_code=None,
            estop_latched=False,
            agent_online=False,
            last_heartbeat_at=None,
            created_at=now,
            updated_at=now,
        )

    def _require_robot(self, session, robot_id: str) -> Robot:
        robot = session.get(Robot, robot_id)
        if robot is None:
            raise NotFoundError(f"robot {robot_id} is not registered")
        return robot

    def count_task_events(self) -> int:
        with self.db.read() as session:
            return int(session.scalar(select(func.count()).select_from(TaskEvent)) or 0)

    def force_reservation_expiry(self, reservation_id: str) -> None:
        with self.db.write() as session:
            row = session.get(ZoneReservation, reservation_id)
            if row is None:
                raise NotFoundError("reservation not found")
            row.expires_at = self.clock.now() - timedelta(seconds=5)

    def goals_for_robot(self, robot_id: str) -> int:
        with self.db.read() as session:
            return int(
                session.scalar(
                    select(func.count()).select_from(NavigationGoal).where(NavigationGoal.robot_id == robot_id)
                )
                or 0
            )
