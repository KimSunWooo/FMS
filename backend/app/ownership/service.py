from __future__ import annotations

import json
from collections.abc import Callable
from datetime import datetime

from sqlalchemy.orm import Session

from app.core.errors import IllegalTransitionError, StaleOwnerError, VersionConflictError
from app.core.logging import get_logger
from app.core.state_machine import TaskStatus, can_transition
from app.models.tables import Task, TaskEvent, TaskOwnership

log = get_logger()


def transition(
    session: Session,
    task: Task,
    to_status: TaskStatus | str,
    *,
    actor: str,
    now: datetime,
    expected_epoch: int | None = None,
    expected_version: int | None = None,
    reason: str = "",
    detail: dict | None = None,
    bump_epoch: bool = False,
    mutate: Callable[[Task], None] | None = None,
) -> Task:
    target = TaskStatus(to_status)
    if expected_version is not None and task.state_version != expected_version:
        raise VersionConflictError(
            f"task {task.id} version {task.state_version} does not match {expected_version}"
        )
    if expected_epoch is not None and task.owner_epoch != expected_epoch:
        raise StaleOwnerError(
            f"task {task.id} owner epoch {task.owner_epoch} rejected claim {expected_epoch}"
        )
    if not can_transition(task.status, target):
        raise IllegalTransitionError(f"{task.status} cannot transition to {target.value}")
    previous = task.status
    if mutate is not None:
        mutate(task)
    if bump_epoch:
        task.owner_epoch += 1
        task.owner_id = actor
    task.status = target.value
    task.state_version += 1
    task.updated_at = now
    session.add(
        TaskEvent(
            task_id=task.id,
            from_status=previous,
            to_status=target.value,
            actor=actor,
            owner_epoch=task.owner_epoch,
            execution_id=task.execution_id,
            goal_id=task.goal_id,
            reason=reason,
            detail_json=json.dumps(detail or {}, ensure_ascii=False),
            created_at=now,
        )
    )
    log.info(
        "task_transition %s -> %s (%s)",
        previous,
        target.value,
        reason,
        extra={
            "event": "task_transition",
            "task_id": task.id,
            "robot_id": task.robot_id,
            "execution_id": task.execution_id,
            "goal_id": task.goal_id,
            "owner_epoch": task.owner_epoch,
        },
    )
    return task


def record_ownership(
    session: Session,
    task: Task,
    action: str,
    now: datetime,
    *,
    detail: str = "",
    epoch: int | None = None,
) -> None:
    session.add(
        TaskOwnership(
            task_id=task.id,
            robot_id=task.robot_id,
            owner_id=task.owner_id,
            owner_epoch=task.owner_epoch if epoch is None else epoch,
            lease_id=task.lease_id,
            action=action,
            detail=detail,
            created_at=now,
        )
    )
