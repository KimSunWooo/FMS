from __future__ import annotations

import json
import math

from sqlalchemy import func, select
from sqlalchemy.exc import IntegrityError
from sqlalchemy.orm import Session

from app.core.clock import as_utc
from app.core.config import Settings
from app.core.errors import NotEligibleError, RobotBusyError
from app.core.state_machine import OCCUPYING, TaskStatus
from app.models.tables import Robot, SystemEvent, Task
from app.ownership.service import record_ownership, transition
from app.robot.mock_agent import MockFleet
from app.robot.mock_gateway import MockGateway
from app.traffic import coordinator as traffic


def _occupying_count(session: Session, robot_id: str) -> int:
    value = session.scalar(
        select(func.count())
        .select_from(Task)
        .where(Task.robot_id == robot_id, Task.status.in_([status.value for status in OCCUPYING]))
    )
    return int(value or 0)


class Scheduler:
    def __init__(self, settings: Settings, gateway: MockGateway, agent: MockFleet):
        self.settings = settings
        self.gateway = gateway
        self.agent = agent

    def schedule_pending(self, session: Session, now) -> list[str]:
        pending = list(
            session.scalars(
                select(Task)
                .where(Task.status == TaskStatus.PENDING.value)
                .order_by(Task.priority.desc(), Task.queue_seq, Task.id)
            ).all()
        )
        assigned: list[str] = []
        for task in pending:
            not_before = as_utc(task.not_before)
            if not_before is not None and now < not_before:
                continue
            choice = self._choose(session, task, now)
            self._remember_decision(session, task, choice, now)
            if choice["selected"] is None:
                continue
            try:
                with session.begin_nested():
                    current = session.get(Task, task.id)
                    robot = session.get(Robot, choice["selected"])
                    if current is None or robot is None:
                        continue
                    if current.status != TaskStatus.PENDING.value:
                        continue
                    self._assign(session, current, robot, now, choice)
                assigned.append(task.id)
            except (NotEligibleError, RobotBusyError, IntegrityError):
                session.expire_all()
                continue
        return assigned

    def _choose(self, session: Session, task: Task, now) -> dict:
        del now
        robots = list(session.scalars(select(Robot).order_by(Robot.id)).all())
        candidates = []
        rejected = []
        for robot in robots:
            reason = self._reject_reason(session, task, robot)
            if reason is not None:
                rejected.append({"robot_id": robot.id, "reason": reason})
                continue
            travel = math.hypot(robot.pose_x - task.destination_x, robot.pose_y - task.destination_y)
            battery_penalty = (100.0 - robot.battery_percent) * self.settings.weight_battery
            congestion_penalty = traffic.congestion_count(
                session, robot, self.settings.congestion_radius_m
            ) * self.settings.weight_congestion
            queue_penalty = _occupying_count(session, robot.id) * self.settings.weight_queue
            total = (
                travel * self.settings.weight_travel
                + battery_penalty
                + congestion_penalty
                + queue_penalty
            )
            candidates.append(
                {
                    "robot_id": robot.id,
                    "travel_cost": round(travel, 4),
                    "battery_penalty": round(battery_penalty, 4),
                    "congestion_penalty": round(congestion_penalty, 4),
                    "task_queue_penalty": round(queue_penalty, 4),
                    "total": round(total, 4),
                }
            )
        candidates.sort(key=lambda item: (item["total"], item["robot_id"]))
        selected = candidates[0]["robot_id"] if candidates else None
        return {
            "distance_model": self.settings.distance_model,
            "selected": selected,
            "candidates": candidates,
            "rejected": rejected,
        }

    def _reject_reason(self, session: Session, task: Task, robot: Robot) -> str | None:
        if task.requested_robot_id and task.requested_robot_id != robot.id:
            return "not_requested_robot"
        if robot.mode == "physical":
            return "physical_blocked"
        if robot.mode == "gazebo" and self.settings.execution_mode != "gazebo":
            return "gazebo_mode_not_armed"
        if robot.mode not in {"mock", "gazebo"}:
            return "mode_not_mock"
        if robot.connection_status != "online":
            return "offline"
        if not robot.controller_ready:
            return "controller_not_ready"
        if robot.battery_percent < self.settings.battery_min_percent:
            return "battery_low"
        if robot.fault_code or robot.estop_latched:
            return "fault"
        if not robot.pose_trustworthy:
            return "pose_untrusted"
        if not self.agent.is_alive(robot.id):
            return "agent_offline"
        if not self.gateway.online:
            return "lease_unavailable"
        if _occupying_count(session, robot.id) > 0:
            return "active_task"
        zone = traffic.zone_for_point(session, task.destination_x, task.destination_y)
        if zone is not None and not traffic.zone_available_for(session, zone.zone_id, robot.id):
            return f"zone_conflict:{zone.zone_id}"
        if not math.isfinite(task.destination_x) or not math.isfinite(task.destination_y):
            return "destination_not_finite"
        return None

    def _assign(self, session: Session, task: Task, robot: Robot, now, choice: dict) -> None:
        if _occupying_count(session, robot.id) > 0:
            raise RobotBusyError(f"{robot.id} already has an active task")
        reason = self._reject_reason(session, task, robot)
        if reason is not None:
            raise NotEligibleError(reason)
        owner = f"fms:{self.gateway.boot_id}"

        def mutate(row: Task) -> None:
            row.robot_id = robot.id
            row.owner_id = owner
            row.owner_epoch += 1

        transition(
            session,
            task,
            TaskStatus.ASSIGNED,
            actor=owner,
            now=now,
            reason="scheduled",
            detail={"cost": next(item for item in choice["candidates"] if item["robot_id"] == robot.id)},
            mutate=mutate,
        )
        robot.operational_state = "busy"
        robot.updated_at = now
        record_ownership(session, task, "acquire", now, detail="scheduler")
        session.flush()

    def _remember_decision(self, session: Session, task: Task, choice: dict, now) -> None:
        key = json.dumps(
            {"selected": choice["selected"], "rejected": choice["rejected"]},
            ensure_ascii=False,
            sort_keys=True,
        )[:200]
        if task.last_decision_key == key:
            return
        task.last_decision_key = key
        session.add(
            SystemEvent(
                level="info",
                code="scheduler_decision",
                message=f"scheduler considered task {task.id}",
                payload_json=json.dumps({"task_id": task.id, **choice}, ensure_ascii=False),
                created_at=now,
            )
        )
