from __future__ import annotations

import json
import math
import uuid
from datetime import timedelta

from sqlalchemy import func, select
from sqlalchemy.orm import Session

from app.core.clock import as_utc
from app.core.config import Settings
from app.core.errors import (
    FmsError,
    IdempotencyMismatchError,
    IllegalTransitionError,
    LeaseRejectedError,
    NotFoundError,
    ZoneConflictError,
)
from app.core.state_machine import IN_FLIGHT, OCCUPYING, TaskStatus
from app.models.tables import ControlLease, NavigationGoal, Robot, RobotTelemetry, SystemEvent, Task, TaskEvent
from app.ownership.service import record_ownership, transition
from app.robot.mock_gateway import MockGateway
from app.robot.safety import SafetyGuard
from app.traffic import coordinator as traffic


def _same_request(task: Task, *, x: float, y: float, yaw: float, label: str, priority: int, robot_id: str | None) -> bool:
    return (
        math.isclose(task.destination_x, x)
        and math.isclose(task.destination_y, y)
        and math.isclose(task.destination_yaw, yaw)
        and task.destination_label == label
        and task.priority == priority
        and task.requested_robot_id == robot_id
    )


def refresh_robot(session: Session, robot: Robot) -> None:
    rows = list(
        session.scalars(
            select(Task).where(
                Task.robot_id == robot.id,
                Task.status.in_([status.value for status in OCCUPYING]),
            )
        ).all()
    )
    if any(row.status == TaskStatus.RECOVERY_REQUIRED.value for row in rows):
        robot.operational_state = "recovery_hold"
        return
    if robot.estop_latched:
        robot.operational_state = "fault"
        robot.fault_code = robot.fault_code or "estop"
        return
    if robot.fault_code:
        robot.operational_state = "fault"
        return
    if rows:
        robot.operational_state = "busy"
        return
    robot.operational_state = "idle"


class TaskManager:
    def __init__(self, settings: Settings, gateway: MockGateway, safety: SafetyGuard):
        self.settings = settings
        self.gateway = gateway
        self.safety = safety

    def create_task(
        self,
        session: Session,
        *,
        x: float,
        y: float,
        yaw: float,
        label: str,
        priority: int,
        robot_id: str | None,
        idempotency_key: str | None,
        now,
    ) -> Task:
        self.safety.assert_coordinates(x, y, yaw)
        if robot_id is not None:
            robot = session.get(Robot, robot_id)
            if robot is None:
                raise NotFoundError(f"robot {robot_id} is not registered")
            self.safety.assert_backend_allowed(robot.mode)
        if idempotency_key:
            existing = session.scalar(select(Task).where(Task.idempotency_key == idempotency_key))
            if existing is not None:
                if not _same_request(
                    existing, x=x, y=y, yaw=yaw, label=label, priority=priority, robot_id=robot_id
                ):
                    raise IdempotencyMismatchError(
                        "idempotency key was already used for a different task payload"
                    )
                return existing
        task_id = str(uuid.uuid4())
        queue_seq = int(session.scalar(select(func.max(Task.queue_seq))) or 0) + 1
        task = Task(
            id=task_id,
            idempotency_key=idempotency_key,
            robot_id=None,
            requested_robot_id=robot_id,
            task_type="navigate",
            status=TaskStatus.CREATED.value,
            destination_x=x,
            destination_y=y,
            destination_yaw=yaw,
            destination_label=label,
            priority=priority,
            queue_seq=queue_seq,
            owner_id=None,
            owner_epoch=0,
            lease_id=None,
            lease_expires_at=None,
            execution_id=None,
            goal_id=None,
            session_id=None,
            state_version=0,
            failure_reason=None,
            recovery_reason=None,
            goal_may_be_in_flight=False,
            dispatch_attempts=0,
            not_before=None,
            last_decision_key=None,
            created_at=now,
            updated_at=now,
            last_heartbeat_at=None,
            cancel_requested_at=None,
            completed_at=None,
        )
        session.add(task)
        session.add(
            TaskEvent(
                task_id=task.id,
                from_status=None,
                to_status=TaskStatus.CREATED.value,
                actor="api",
                owner_epoch=0,
                execution_id=None,
                goal_id=None,
                reason="accepted",
                detail_json="{}",
                created_at=now,
            )
        )
        session.flush()
        transition(
            session,
            task,
            TaskStatus.PENDING,
            actor="api",
            now=now,
            reason="queued",
        )
        session.add(
            SystemEvent(
                level="info",
                code="task_accepted",
                message="task stored; robot motion has not started",
                payload_json=json.dumps({"task_id": task.id, "status": task.status}),
                created_at=now,
            )
        )
        return task

    def cancel(self, session: Session, task_id: str, now) -> Task:
        task = self._require(session, task_id)
        if task.status in {status.value for status in {TaskStatus.COMPLETED, TaskStatus.CANCELED, TaskStatus.FAILED}}:
            raise IllegalTransitionError(f"task is already {task.status}")
        if task.status == TaskStatus.RECOVERY_REQUIRED.value:
            raise IllegalTransitionError("recovery holds must be resolved explicitly")
        certain = task.status == TaskStatus.PENDING.value or (
            task.status == TaskStatus.ASSIGNED.value and not task.goal_may_be_in_flight and task.goal_id is None
        )
        if certain:
            robot = session.get(Robot, task.robot_id) if task.robot_id else None

            def mutate(row: Task) -> None:
                row.robot_id = None
                row.completed_at = now
                row.failure_reason = None

            transition(
                session,
                task,
                TaskStatus.CANCELED,
                actor="api",
                now=now,
                reason="canceled_before_goal",
                bump_epoch=True,
                mutate=mutate,
            )
            record_ownership(session, task, "release", now, detail="canceled_before_goal")
            if robot is not None:
                refresh_robot(session, robot)
            return task
        transition(
            session,
            task,
            TaskStatus.CANCEL_REQUESTED,
            actor="api",
            now=now,
            reason="cancel_requested",
            mutate=lambda row: setattr(row, "cancel_requested_at", now),
        )
        return task

    def resolve_recovery(
        self,
        session: Session,
        task_id: str,
        *,
        outcome: str,
        reason: str,
        acknowledge_position_unknown: bool,
        now,
    ) -> Task:
        task = self._require(session, task_id)
        if task.status != TaskStatus.RECOVERY_REQUIRED.value:
            raise IllegalTransitionError("only RECOVERY_REQUIRED tasks can be resolved")
        if outcome not in {TaskStatus.CANCELED.value, TaskStatus.FAILED.value}:
            raise IllegalTransitionError("recovery can only be closed as CANCELED or FAILED")
        if not acknowledge_position_unknown:
            raise FmsError("position is unconfirmed; acknowledge that before closing the hold")
        if outcome == TaskStatus.COMPLETED.value:
            raise IllegalTransitionError("recovery cannot be completed without a fresh goal result")
        robot = session.get(Robot, task.robot_id) if task.robot_id else None

        def mutate(row: Task) -> None:
            row.completed_at = now
            if outcome == TaskStatus.FAILED.value:
                row.failure_reason = reason
            row.recovery_reason = reason

        transition(
            session,
            task,
            outcome,
            actor=f"operator:{self.gateway.boot_id}",
            now=now,
            reason=reason,
            bump_epoch=True,
            mutate=mutate,
        )
        record_ownership(session, task, "release", now, detail="recovery_resolved")
        if robot is not None:
            robot.pose_trustworthy = False
            robot.pose_source = "unconfirmed_after_recovery"
            traffic.block_robot_zones(session, robot.id, now, "fail_closed_after_recovery")
            refresh_robot(session, robot)
        session.add(
            SystemEvent(
                level="warning",
                code="recovery_resolved",
                message="operator closed a recovery hold without issuing a goal",
                payload_json=json.dumps({"task_id": task.id, "outcome": outcome, "goal_replayed": False}),
                created_at=now,
            )
        )
        return task

    def dispatch_assigned(self, session: Session, task: Task, session_id: str, now) -> Task:
        if task.status != TaskStatus.ASSIGNED.value:
            return task
        if task.goal_may_be_in_flight or task.goal_id is not None:
            self.enter_recovery(session, task, "assigned_with_existing_goal", now)
            return task
        robot = session.get(Robot, task.robot_id) if task.robot_id else None
        if robot is None:
            raise NotFoundError("assigned task has no robot")
        self.safety.assert_backend_allowed(robot.mode)
        try:
            traffic.acquire_destination(
                session,
                robot=robot,
                task=task,
                now=now,
                ttl_seconds=max(self.settings.lease_ttl_seconds * 4, 120),
            )
            lease = self.gateway.issue(
                session,
                robot_id=robot.id,
                session_id=session_id,
                owner_id=task.owner_id or f"fms:{self.gateway.boot_id}",
                now=now,
            )
        except (ZoneConflictError, LeaseRejectedError):
            raise
        goal_id = str(uuid.uuid4())
        execution_id = str(uuid.uuid4())
        session.add(
            NavigationGoal(
                goal_id=goal_id,
                task_id=task.id,
                execution_id=execution_id,
                robot_id=robot.id,
                session_id=session_id,
                lease_id=lease.lease_id,
                owner_epoch=task.owner_epoch,
                status="pending",
                destination_x=task.destination_x,
                destination_y=task.destination_y,
                destination_yaw=task.destination_yaw,
                observed_x=None,
                observed_y=None,
                observed_yaw=None,
                observation_failures=0,
                last_result_outcome=None,
                last_result_disposition=None,
                auto_complete_at=None,
                created_at=now,
                updated_at=now,
            )
        )

        def mutate(row: Task) -> None:
            row.goal_id = goal_id
            row.execution_id = execution_id
            row.session_id = session_id
            row.lease_id = lease.lease_id
            row.lease_expires_at = lease.expires_at
            row.goal_may_be_in_flight = True
            row.last_heartbeat_at = now

        robot.last_heartbeat_at = now
        handed = "goal_handed_to_mock_agent" if robot.mode == "mock" else "goal_handed_to_robot_agent"
        transition(
            session,
            task,
            TaskStatus.DISPATCHING,
            actor=task.owner_id or "fms",
            now=now,
            reason=handed,
            mutate=mutate,
        )
        record_ownership(session, task, "dispatch", now, detail=lease.lease_id)
        session.flush()
        return task

    def abort_dispatch_before_goal(self, session: Session, task: Task, reason: str, now) -> Task:
        if task.goal_may_be_in_flight or task.goal_id is not None:
            self.enter_recovery(session, task, reason, now)
            return task
        robot = session.get(Robot, task.robot_id) if task.robot_id else None
        task.dispatch_attempts += 1
        exhausted = task.dispatch_attempts >= self.settings.max_dispatch_attempts
        target = TaskStatus.FAILED if exhausted else TaskStatus.PENDING

        def mutate(row: Task) -> None:
            row.robot_id = None
            row.lease_id = None
            row.session_id = None
            if exhausted:
                row.failure_reason = reason
                row.completed_at = now
            else:
                row.not_before = now + timedelta(seconds=self.settings.dispatch_backoff_seconds)

        transition(
            session,
            task,
            target,
            actor=f"fms:{self.gateway.boot_id}",
            now=now,
            reason=reason,
            bump_epoch=True,
            detail={"goal_sent": False, "dispatch_attempts": task.dispatch_attempts},
            mutate=mutate,
        )
        record_ownership(session, task, "release", now, detail=reason)
        if robot is not None:
            refresh_robot(session, robot)
        return task

    def accept_goal(self, session: Session, task: Task, now) -> Task:
        if task.status != TaskStatus.DISPATCHING.value:
            return task
        robot = session.get(Robot, task.robot_id) if task.robot_id else None
        goal = session.get(NavigationGoal, task.goal_id) if task.goal_id else None
        lease = session.get(ControlLease, task.lease_id) if task.lease_id else None
        if robot is None or goal is None or lease is None:
            self.enter_recovery(session, task, "dispatch_record_missing", now)
            return task
        self.safety.assert_backend_allowed(robot.mode)
        if not self.gateway.verify(
            session, lease, now=now, robot_id=robot.id, session_id=task.session_id
        ):
            self.enter_recovery(session, task, "lease_rejected_before_accept", now)
            return task
        goal.status = "accepted"
        goal.updated_at = now
        if robot.mode == "mock":
            goal.auto_complete_at = now + timedelta(seconds=self.settings.mock_goal_duration_seconds)
            reason = "mock_goal_accepted_not_completed"
        else:
            goal.auto_complete_at = None
            reason = "nav2_goal_accepted_not_completed"
        task.last_heartbeat_at = now
        robot.last_heartbeat_at = now
        robot.agent_online = True
        transition(
            session,
            task,
            TaskStatus.RUNNING,
            actor=task.owner_id or "fms",
            now=now,
            expected_epoch=task.owner_epoch,
            reason=reason,
            detail={"goal_id": goal.goal_id, "nav2_action_goal_id": goal.nav2_action_goal_id},
        )
        return task

    def heartbeat(self, session: Session, task: Task, now) -> None:
        robot = session.get(Robot, task.robot_id) if task.robot_id else None
        lease = session.get(ControlLease, task.lease_id) if task.lease_id else None
        task.last_heartbeat_at = now
        if robot is not None:
            robot.last_heartbeat_at = now
            robot.agent_online = True
        if lease is not None and not self.gateway.renew(session, lease.lease_id, now):
            self.enter_recovery(session, task, "lease_renew_failed", now)
            return
        if lease is not None:
            refreshed = session.get(ControlLease, lease.lease_id)
            if refreshed is not None:
                task.lease_expires_at = refreshed.expires_at

    def complete_mock(self, session: Session, task: Task, now) -> Task:
        goal = session.get(NavigationGoal, task.goal_id) if task.goal_id else None
        if goal is None or goal.auto_complete_at is None:
            return task
        ready = as_utc(goal.auto_complete_at)
        if ready is None or now < ready:
            return task
        return self.apply_result(
            session,
            task_id=task.id,
            goal_id=task.goal_id or "",
            execution_id=task.execution_id or "",
            owner_epoch=task.owner_epoch,
            session_id=task.session_id or "",
            outcome="succeeded",
            pose_x=task.destination_x,
            pose_y=task.destination_y,
            pose_yaw=task.destination_yaw,
            pose_trustworthy=True,
            now=now,
            source="mock-agent",
        )

    def apply_result(
        self,
        session: Session,
        *,
        task_id: str,
        goal_id: str,
        execution_id: str,
        owner_epoch: int,
        session_id: str,
        outcome: str,
        pose_x: float,
        pose_y: float,
        pose_yaw: float,
        pose_trustworthy: bool,
        now,
        source: str,
    ) -> Task:
        task = self._require(session, task_id)
        goal = session.get(NavigationGoal, task.goal_id) if task.goal_id else None
        if task.owner_epoch != owner_epoch:
            self._reject_result(session, task, goal, outcome, "rejected_stale_owner", now)
            return task
        if (
            goal is None
            or task.goal_id != goal_id
            or task.execution_id != execution_id
            or task.session_id != session_id
        ):
            self._reject_result(session, task, goal, outcome, "rejected_goal_mismatch", now)
            return task
        if task.status in {
            TaskStatus.RECOVERY_REQUIRED.value,
            TaskStatus.CANCELED.value,
            TaskStatus.FAILED.value,
            TaskStatus.COMPLETED.value,
        }:
            self._reject_result(session, task, goal, outcome, "rejected_task_not_running", now)
            return task
        lease = session.get(ControlLease, task.lease_id) if task.lease_id else None
        robot = session.get(Robot, task.robot_id) if task.robot_id else None
        if lease is None or robot is None or not self.gateway.verify(
            session, lease, now=now, robot_id=robot.id, session_id=session_id
        ):
            self.enter_recovery(session, task, "result_without_valid_lease", now)
            return task
        if task.status == TaskStatus.CANCEL_REQUESTED.value and outcome != "canceled":
            goal.last_result_outcome = outcome
            goal.last_result_disposition = "cancel_race"
            goal.updated_at = now
            reason = "cancel_success_race" if outcome == "succeeded" else f"cancel_conflict_{outcome}"
            self.enter_recovery(session, task, reason, now)
            return task
        if outcome == "rejected":
            return self._apply_rejection(session, task, goal, robot, now)
        if outcome == "aborted":
            return self._apply_abort(session, task, goal, robot, now, source)
        if outcome == "canceled":
            return self._apply_cancel_result(session, task, goal, now, pose_trustworthy)
        if outcome == "disconnected":
            goal.last_result_outcome = outcome
            goal.last_result_disposition = "action_server_disconnected"
            goal.updated_at = now
            self.enter_recovery(session, task, "nav2_action_server_disconnected", now)
            return task
        if outcome != "succeeded":
            self._reject_result(session, task, goal, outcome, "rejected_outcome", now)
            return task
        if task.status != TaskStatus.RUNNING.value:
            goal.last_result_outcome = outcome
            goal.last_result_disposition = "success_before_running"
            goal.updated_at = now
            self.enter_recovery(session, task, "success_before_running", now)
            return task
        if not pose_trustworthy:
            self.enter_recovery(session, task, "success_report_without_trustworthy_pose", now)
            return task
        distance = math.hypot(pose_x - task.destination_x, pose_y - task.destination_y)
        if distance > self.settings.pose_tolerance_m:
            goal.observation_failures += 1
            goal.last_result_outcome = outcome
            goal.last_result_disposition = "observation_mismatch"
            goal.observed_x = pose_x
            goal.observed_y = pose_y
            goal.observed_yaw = pose_yaw
            goal.updated_at = now
            if goal.observation_failures >= 2:
                self.enter_recovery(session, task, "pose_mismatch", now)
            return task
        goal.status = "succeeded"
        goal.last_result_outcome = "succeeded"
        goal.last_result_disposition = "matched_pose_lease_and_owner"
        goal.observed_x = pose_x
        goal.observed_y = pose_y
        goal.observed_yaw = pose_yaw
        goal.updated_at = now
        robot.pose_x = pose_x
        robot.pose_y = pose_y
        robot.pose_yaw = pose_yaw
        robot.pose_trustworthy = True
        robot.pose_source = source
        robot.last_heartbeat_at = now
        session.add(
            RobotTelemetry(
                robot_id=robot.id,
                pose_x=pose_x,
                pose_y=pose_y,
                pose_yaw=pose_yaw,
                battery_percent=robot.battery_percent,
                source=source,
                trustworthy=True,
                observed_at=now,
            )
        )

        def mutate(row: Task) -> None:
            row.completed_at = now
            row.last_heartbeat_at = now

        transition(
            session,
            task,
            TaskStatus.COMPLETED,
            actor=task.owner_id or "fms",
            now=now,
            expected_epoch=owner_epoch,
            reason="nav_result_succeeded_pose_matched",
            detail={
                "distance_m": distance,
                "source": source,
                "distance_model_note": "completion uses pose tolerance, not the scheduler heuristic",
            },
            mutate=mutate,
        )
        record_ownership(session, task, "complete", now, detail=source)
        traffic.sync_pose_occupancy(
            session, robot, now, reservation_ttl=max(self.settings.lease_ttl_seconds * 4, 120)
        )
        refresh_robot(session, robot)
        return task

    def ack_cancel(self, session: Session, task: Task, now) -> Task:
        if task.status != TaskStatus.CANCEL_REQUESTED.value:
            return task
        robot = session.get(Robot, task.robot_id) if task.robot_id else None
        goal = session.get(NavigationGoal, task.goal_id) if task.goal_id else None
        if robot is None or not robot.pose_trustworthy:
            self.enter_recovery(session, task, "cancel_ack_without_trusted_pose", now)
            return task
        if goal is not None:
            goal.status = "cancel_acked"
            goal.last_result_outcome = "canceled"
            goal.last_result_disposition = "agent_ack"
            goal.updated_at = now

        def mutate(row: Task) -> None:
            row.completed_at = now

        transition(
            session,
            task,
            TaskStatus.CANCELED,
            actor=task.owner_id or "fms",
            now=now,
            expected_epoch=task.owner_epoch,
            reason="cancel_acknowledged",
            mutate=mutate,
        )
        record_ownership(session, task, "release", now, detail="cancel_acknowledged")
        traffic.sync_pose_occupancy(
            session, robot, now, reservation_ttl=max(self.settings.lease_ttl_seconds * 4, 120)
        )
        refresh_robot(session, robot)
        return task

    def enter_recovery(self, session: Session, task: Task, reason: str, now) -> Task:
        if task.status == TaskStatus.RECOVERY_REQUIRED.value:
            robot = session.get(Robot, task.robot_id) if task.robot_id else None
            if robot is not None:
                robot.pose_trustworthy = False
                robot.operational_state = "recovery_hold"
                traffic.block_robot_zones(session, robot.id, now, "fail_closed_" + reason)
            return task
        if task.status not in {status.value for status in IN_FLIGHT | {TaskStatus.ASSIGNED}}:
            return task
        robot = session.get(Robot, task.robot_id) if task.robot_id else None

        def mutate(row: Task) -> None:
            row.recovery_reason = reason

        transition(
            session,
            task,
            TaskStatus.RECOVERY_REQUIRED,
            actor=f"fms:{self.gateway.boot_id}",
            now=now,
            reason=reason,
            bump_epoch=True,
            detail={"automatic_redispatch": False},
            mutate=mutate,
        )
        record_ownership(session, task, "fence", now, detail=reason)
        if robot is not None:
            robot.pose_trustworthy = False
            robot.pose_source = "untrusted_recovery"
            robot.operational_state = "recovery_hold"
            traffic.block_robot_zones(session, robot.id, now, "fail_closed_" + reason)
        session.add(
            SystemEvent(
                level="warning",
                code="recovery_required",
                message=reason,
                payload_json=json.dumps(
                    {
                        "task_id": task.id,
                        "robot_id": task.robot_id,
                        "goal_id": task.goal_id,
                        "execution_id": task.execution_id,
                        "automatic_redispatch": False,
                    }
                ),
                created_at=now,
            )
        )
        return task

    def claim_send(
        self,
        session: Session,
        *,
        robot_id: str,
        session_id: str,
        owner_epoch: int,
        lease_id: str,
        goal_id: str,
        now,
    ) -> dict:
        """Allow one Nav2 send for this goal. A second claim does not insert a goal."""
        robot = session.get(Robot, robot_id)
        goal = session.get(NavigationGoal, goal_id)
        if robot is None or goal is None or goal.robot_id != robot_id:
            raise NotFoundError("goal is not owned by this robot")
        self._require_external_agent(robot)
        task = self._require(session, goal.task_id)
        goals_before = 1
        fence_ok = (
            task.owner_epoch == owner_epoch
            and task.session_id == session_id
            and task.lease_id == lease_id
            and task.goal_id == goal_id
            and task.status == TaskStatus.DISPATCHING.value
        )
        lease = session.get(ControlLease, lease_id)
        lease_ok = lease is not None and self.gateway.verify(
            session, lease, now=now, robot_id=robot_id, session_id=session_id
        )
        if not fence_ok or not lease_ok:
            reason = "fence_mismatch"
        elif goal.status != "pending":
            reason = "duplicate_send"
        else:
            reason = ""
        if reason:
            self._reject_result(session, task, goal, "claim", reason, now)
            return {
                "send_navigate_to_pose": False,
                "reason": reason,
                "goal_id": goal.goal_id,
                "task_id": task.id,
                "execution_id": task.execution_id,
                "owner_epoch": task.owner_epoch,
                "navigation_goal_count": goals_before,
            }
        goal.status = "send_claimed"
        goal.updated_at = now
        task.last_heartbeat_at = now
        robot.last_heartbeat_at = now
        session.add(
            SystemEvent(
                level="info",
                code="goal_send_claimed",
                message="one Nav2 send was claimed; the task is not complete",
                payload_json=json.dumps(
                    {
                        "task_id": task.id,
                        "goal_id": goal.goal_id,
                        "execution_id": task.execution_id,
                        "owner_epoch": task.owner_epoch,
                        "session_id": session_id,
                        "send_navigate_to_pose": True,
                        "completed": False,
                    }
                ),
                created_at=now,
            )
        )
        return {
            "send_navigate_to_pose": True,
            "reason": "claimed_once",
            "goal_id": goal.goal_id,
            "task_id": task.id,
            "execution_id": task.execution_id,
            "owner_epoch": task.owner_epoch,
            "lease_id": task.lease_id,
            "session_id": task.session_id,
            "destination": {
                "x": task.destination_x,
                "y": task.destination_y,
                "yaw": task.destination_yaw,
                "frame_id": "map",
            },
            "nav2_action": "navigate_to_pose",
            "navigation_goal_count": goals_before,
        }

    def note_nav2_accept(
        self,
        session: Session,
        *,
        task_id: str,
        goal_id: str,
        execution_id: str,
        owner_epoch: int,
        session_id: str,
        nav2_action_goal_id: str | None,
        now,
    ) -> Task:
        task = self._require(session, task_id)
        goal = session.get(NavigationGoal, task.goal_id) if task.goal_id else None
        robot = session.get(Robot, task.robot_id) if task.robot_id else None
        if robot is not None:
            self._require_external_agent(robot)
        if (
            goal is None
            or robot is None
            or task.owner_epoch != owner_epoch
            or task.goal_id != goal_id
            or task.execution_id != execution_id
            or task.session_id != session_id
            or goal.status != "send_claimed"
        ):
            self._reject_result(session, task, goal, "accepted", "rejected_accept_fence", now)
            return task
        if nav2_action_goal_id:
            goal.nav2_action_goal_id = nav2_action_goal_id
        return self.accept_goal(session, task, now)

    def _require_external_agent(self, robot: Robot) -> None:
        from app.core.errors import PhysicalControlBlocked

        if robot.mode == "physical" or self.settings.execution_mode == "physical":
            raise PhysicalControlBlocked("Physical motor control is blocked.")
        if self.settings.execution_mode != "gazebo" or robot.mode != "gazebo":
            raise PhysicalControlBlocked(
                "The external agent API is closed unless execution mode and the robot are both gazebo."
            )

    def _apply_rejection(self, session, task, goal, robot, now) -> Task:
        never_accepted = goal.status in {"pending", "send_claimed"} and task.status == TaskStatus.DISPATCHING.value
        goal.last_result_outcome = "rejected"
        goal.updated_at = now
        if not never_accepted:
            goal.last_result_disposition = "reject_after_accept"
            self.enter_recovery(session, task, "nav2_reject_after_accept", now)
            return task
        goal.status = "rejected"
        goal.last_result_disposition = "nav2_rejected_before_accept"

        def mutate(row: Task) -> None:
            row.failure_reason = "nav2_goal_rejected"
            row.completed_at = now

        transition(
            session,
            task,
            TaskStatus.FAILED,
            actor=task.owner_id or "fms",
            now=now,
            expected_epoch=task.owner_epoch,
            reason="nav2_goal_rejected",
            detail={"goal_sent_result": "rejected", "completed": False},
            mutate=mutate,
        )
        refresh_robot(session, robot)
        return task

    def _apply_abort(self, session, task, goal, robot, now, source: str) -> Task:
        reason = "mock_goal_aborted" if source.startswith("mock") else "nav2_goal_aborted"
        goal.status = "aborted"
        goal.last_result_outcome = "aborted"
        goal.last_result_disposition = "accepted_abort"
        goal.updated_at = now

        def mutate(row: Task) -> None:
            row.failure_reason = reason
            row.completed_at = now

        transition(
            session,
            task,
            TaskStatus.FAILED,
            actor=task.owner_id or "fms",
            now=now,
            expected_epoch=task.owner_epoch,
            reason=reason,
            mutate=mutate,
        )
        refresh_robot(session, robot)
        return task

    def _apply_cancel_result(self, session, task, goal, now, pose_trustworthy: bool) -> Task:
        if task.status == TaskStatus.CANCEL_REQUESTED.value:
            if not pose_trustworthy:
                goal.last_result_outcome = "canceled"
                goal.last_result_disposition = "cancel_without_trusted_pose"
                goal.updated_at = now
                self.enter_recovery(session, task, "cancel_ack_without_trusted_pose", now)
                return task
            return self.ack_cancel(session, task, now)
        goal.status = "canceled_unsolicited"
        goal.last_result_outcome = "canceled"
        goal.last_result_disposition = "unsolicited_cancel"
        goal.updated_at = now
        self.enter_recovery(session, task, "nav2_cancel_without_request", now)
        return task

    def _reject_result(self, session, task, goal, outcome: str, disposition: str, now) -> None:
        if goal is not None:
            goal.last_result_outcome = outcome
            goal.last_result_disposition = disposition
            goal.updated_at = now
        record_ownership(
            session,
            task,
            "fence_reject",
            now,
            detail=disposition,
            epoch=task.owner_epoch,
        )
        session.add(
            SystemEvent(
                level="warning",
                code="stale_result_rejected",
                message=disposition,
                payload_json=json.dumps(
                    {"task_id": task.id, "goal_id": task.goal_id, "status_unchanged": task.status}
                ),
                created_at=now,
            )
        )

    def _require(self, session: Session, task_id: str) -> Task:
        task = session.get(Task, task_id)
        if task is None:
            raise NotFoundError(f"task {task_id} not found")
        return task
