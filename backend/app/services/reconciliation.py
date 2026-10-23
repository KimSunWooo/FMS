from __future__ import annotations

from datetime import timedelta

from sqlalchemy import select
from sqlalchemy.orm import Session

from app.core.clock import as_utc
from app.core.config import Settings
from app.core.state_machine import IN_FLIGHT, TaskStatus
from app.models.tables import ControlLease, Robot, Task
from app.robot.mock_agent import MockFleet
from app.robot.mock_gateway import MockGateway
from app.services.task_manager import TaskManager


class ReconciliationWorker:
    """Moves uncertain in-flight work to RECOVERY_REQUIRED.

    It does not create navigation goals and it does not mark a timeout as
    success or as a completed cancel.
    """

    def __init__(
        self,
        settings: Settings,
        tasks: TaskManager,
        gateway: MockGateway,
        agent: MockFleet,
    ):
        self.settings = settings
        self.tasks = tasks
        self.gateway = gateway
        self.agent = agent

    def run(self, session: Session, now, *, process_restart: bool = False) -> list[str]:
        recovered: list[str] = []
        rows = list(
            session.scalars(
                select(Task).where(
                    Task.status.in_(
                        [status.value for status in IN_FLIGHT]
                        + [TaskStatus.ASSIGNED.value, TaskStatus.RECOVERY_REQUIRED.value]
                    )
                )
            ).all()
        )
        for task in rows:
            reason = self._reason(session, task, now, process_restart=process_restart)
            if reason is None:
                continue
            if task.status == TaskStatus.ASSIGNED.value and not task.goal_may_be_in_flight:
                self.tasks.abort_dispatch_before_goal(session, task, reason, now)
                continue
            if task.status == TaskStatus.RECOVERY_REQUIRED.value:
                robot = session.get(Robot, task.robot_id) if task.robot_id else None
                if robot is not None:
                    robot.pose_trustworthy = False
                    robot.operational_state = "recovery_hold"
                from app.traffic import coordinator as traffic

                if robot is not None:
                    traffic.block_robot_zones(session, robot.id, now, "fail_closed_" + reason)
                continue
            self.tasks.enter_recovery(session, task, reason, now)
            recovered.append(task.id)
        return recovered

    def _reason(self, session: Session, task: Task, now, *, process_restart: bool) -> str | None:
        if task.status == TaskStatus.RECOVERY_REQUIRED.value:
            return "hold_maintained"
        if process_restart and task.goal_may_be_in_flight:
            return "process_restart"
        if process_restart and task.status == TaskStatus.ASSIGNED.value and not task.goal_may_be_in_flight:
            return "restart_before_goal"
        robot = session.get(Robot, task.robot_id) if task.robot_id else None
        agent_alive = robot is not None and self.agent.is_alive(robot.id)
        same_session = (
            robot is not None
            and task.session_id is not None
            and self.agent.session_id(robot.id) == task.session_id
        )
        if task.goal_may_be_in_flight and (not agent_alive or not same_session):
            return "agent_session_lost"
        lease = session.get(ControlLease, task.lease_id) if task.lease_id else None
        if task.goal_may_be_in_flight and (
            lease is None
            or not self.gateway.verify(
                session,
                lease,
                now=now,
                robot_id=task.robot_id,
                session_id=task.session_id,
            )
        ):
            return "lease_invalid"
        heartbeat = as_utc(task.last_heartbeat_at)
        if (
            task.status in {TaskStatus.RUNNING.value, TaskStatus.DISPATCHING.value}
            and heartbeat is not None
            and now - heartbeat > timedelta(seconds=self.settings.heartbeat_timeout_seconds)
        ):
            return "heartbeat_timeout"
        cancel_at = as_utc(task.cancel_requested_at)
        if task.status == TaskStatus.CANCEL_REQUESTED.value and cancel_at is not None:
            hung = robot is not None and robot.id in self.agent.suppress_cancel_ack
            if hung and now - cancel_at > timedelta(seconds=self.settings.cancel_timeout_seconds):
                return "cancel_timeout"
            if not agent_alive and task.goal_may_be_in_flight:
                return "agent_lost_during_cancel"
        if robot is not None and robot.connection_status != "online" and task.goal_may_be_in_flight:
            return "robot_offline"
        return None
