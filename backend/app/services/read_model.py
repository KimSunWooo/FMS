from __future__ import annotations

import math

from sqlalchemy import func, select
from sqlalchemy.orm import Session

from app.core.clock import iso
from app.core.state_machine import OCCUPYING, TaskStatus
from app.models.tables import (
    NavigationGoal,
    Robot,
    SystemEvent,
    Task,
    TaskEvent,
    TrafficZone,
    ZoneReservation,
)


def _goal_count(session: Session, task_id: str) -> int:
    value = session.scalar(
        select(func.count()).select_from(NavigationGoal).where(NavigationGoal.task_id == task_id)
    )
    return int(value or 0)


def robot_out(session: Session, robot: Robot) -> dict:
    active = session.scalar(
        select(Task.id).where(
            Task.robot_id == robot.id,
            Task.status.in_([status.value for status in OCCUPYING]),
        )
    )
    return {
        "robot_id": robot.id,
        "name": robot.name,
        "mode": robot.mode,
        "connection_status": robot.connection_status,
        "operational_state": robot.operational_state,
        "battery_percent": robot.battery_percent,
        "controller_ready": robot.controller_ready,
        "controller_kind": robot.controller_kind,
        "pose": {"x": robot.pose_x, "y": robot.pose_y, "yaw": robot.pose_yaw},
        "pose_trustworthy": robot.pose_trustworthy,
        "pose_source": robot.pose_source,
        "fault_code": robot.fault_code,
        "estop_latched": robot.estop_latched,
        "agent_online": robot.agent_online,
        "active_task_id": active,
        "last_heartbeat_at": iso(robot.last_heartbeat_at),
        "updated_at": iso(robot.updated_at),
    }


def task_out(session: Session, task: Task) -> dict:
    goal = session.get(NavigationGoal, task.goal_id) if task.goal_id else None
    events = session.scalars(
        select(TaskEvent).where(TaskEvent.task_id == task.id).order_by(TaskEvent.id)
    ).all()
    evidence = None
    if task.status == TaskStatus.COMPLETED.value and goal is not None and goal.status == "succeeded":
        observed_x = goal.observed_x if goal.observed_x is not None else task.destination_x
        observed_y = goal.observed_y if goal.observed_y is not None else task.destination_y
        evidence = {
            "goal_status": goal.status,
            "goal_id": goal.goal_id,
            "execution_id": goal.execution_id,
            "owner_epoch": task.owner_epoch,
            "session_id": task.session_id,
            "lease_id": task.lease_id,
            "distance_m": math.hypot(observed_x - task.destination_x, observed_y - task.destination_y),
            "observation_source": "recorded_on_goal",
            "disposition": goal.last_result_disposition,
        }
    return {
        "task_id": task.id,
        "task_type": task.task_type,
        "status": task.status,
        "robot_id": task.robot_id,
        "requested_robot_id": task.requested_robot_id,
        "destination": {
            "x": task.destination_x,
            "y": task.destination_y,
            "yaw": task.destination_yaw,
            "label": task.destination_label,
        },
        "priority": task.priority,
        "owner_id": task.owner_id,
        "owner_epoch": task.owner_epoch,
        "lease_id": task.lease_id,
        "lease_expires_at": iso(task.lease_expires_at),
        "execution_id": task.execution_id,
        "goal_id": task.goal_id,
        "session_id": task.session_id,
        "state_version": task.state_version,
        "failure_reason": task.failure_reason,
        "recovery_reason": task.recovery_reason,
        "goal_may_be_in_flight": task.goal_may_be_in_flight,
        "dispatch_attempts": task.dispatch_attempts,
        "navigation_goal_count": _goal_count(session, task.id),
        "created_at": iso(task.created_at),
        "updated_at": iso(task.updated_at),
        "last_heartbeat_at": iso(task.last_heartbeat_at),
        "cancel_requested_at": iso(task.cancel_requested_at),
        "completed_at": iso(task.completed_at),
        "goal": None
        if goal is None
        else {
            "goal_id": goal.goal_id,
            "status": goal.status,
            "execution_id": goal.execution_id,
            "owner_epoch": goal.owner_epoch,
            "last_result_outcome": goal.last_result_outcome,
            "last_result_disposition": goal.last_result_disposition,
            "nav2_action_goal_id": goal.nav2_action_goal_id,
            "nav2_action_goal_id_absent_reason": goal.nav2_action_goal_id_absent_reason,
            "observation_failures": goal.observation_failures,
        },
        "completion_evidence": evidence,
        "events": [
            {
                "from_status": event.from_status,
                "to_status": event.to_status,
                "actor": event.actor,
                "owner_epoch": event.owner_epoch,
                "execution_id": event.execution_id,
                "goal_id": event.goal_id,
                "reason": event.reason,
                "created_at": iso(event.created_at),
            }
            for event in events
        ],
    }


def zone_out(session: Session, zone: TrafficZone) -> dict:
    reservation = session.scalar(
        select(ZoneReservation).where(
            ZoneReservation.zone_id == zone.zone_id,
            ZoneReservation.reservation_status.in_(("held", "blocked")),
        )
    )
    return {
        "zone_id": zone.zone_id,
        "name": zone.name,
        "kind": zone.kind,
        "capacity": zone.capacity,
        "bounds": {
            "x": zone.bounds_x,
            "y": zone.bounds_y,
            "w": zone.bounds_w,
            "h": zone.bounds_h,
        },
        "reservation": None
        if reservation is None
        else {
            "reservation_id": reservation.reservation_id,
            "robot_id": reservation.robot_id,
            "task_id": reservation.task_id,
            "owner_epoch": reservation.owner_epoch,
            "reservation_status": reservation.reservation_status,
            "hold_reason": reservation.hold_reason,
            "acquired_at": iso(reservation.acquired_at),
            "expires_at": iso(reservation.expires_at),
        },
    }


def reservation_out(row: ZoneReservation) -> dict:
    return {
        "reservation_id": row.reservation_id,
        "zone_id": row.zone_id,
        "robot_id": row.robot_id,
        "task_id": row.task_id,
        "owner_epoch": row.owner_epoch,
        "reservation_status": row.reservation_status,
        "hold_reason": row.hold_reason,
        "acquired_at": iso(row.acquired_at),
        "expires_at": iso(row.expires_at),
        "released_at": iso(row.released_at),
    }


def event_feed(session: Session, limit: int) -> list[dict]:
    system_rows = session.scalars(select(SystemEvent).order_by(SystemEvent.id.desc()).limit(limit)).all()
    task_rows = session.scalars(select(TaskEvent).order_by(TaskEvent.id.desc()).limit(limit)).all()
    merged: list[dict] = []
    for row in system_rows:
        merged.append(
            {
                "kind": "system",
                "level": row.level,
                "code": row.code,
                "message": row.message,
                "payload_json": row.payload_json,
                "created_at": iso(row.created_at),
            }
        )
    for row in task_rows:
        merged.append(
            {
                "kind": "task",
                "level": "info",
                "code": "task_event",
                "message": f"{row.from_status or '-'} → {row.to_status} ({row.reason})",
                "payload_json": row.detail_json,
                "task_id": row.task_id,
                "created_at": iso(row.created_at),
            }
        )
    merged.sort(key=lambda item: item["created_at"] or "", reverse=True)
    return merged[:limit]
