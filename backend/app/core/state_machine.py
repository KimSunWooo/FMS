from __future__ import annotations

from enum import Enum


class TaskStatus(str, Enum):
    CREATED = "CREATED"
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    DISPATCHING = "DISPATCHING"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    CANCEL_REQUESTED = "CANCEL_REQUESTED"
    CANCELED = "CANCELED"
    FAILED = "FAILED"
    RECOVERY_REQUIRED = "RECOVERY_REQUIRED"


TERMINAL = {
    TaskStatus.COMPLETED,
    TaskStatus.CANCELED,
    TaskStatus.FAILED,
}

# A goal may already be in the controller. Timeout must not leave this set
# by declaring success or a completed cancel.
IN_FLIGHT = {
    TaskStatus.DISPATCHING,
    TaskStatus.RUNNING,
    TaskStatus.CANCEL_REQUESTED,
}

# Statuses that hold the single-active-task slot for a robot.
OCCUPYING = {
    TaskStatus.ASSIGNED,
    TaskStatus.DISPATCHING,
    TaskStatus.RUNNING,
    TaskStatus.CANCEL_REQUESTED,
    TaskStatus.RECOVERY_REQUIRED,
}

ALLOWED: dict[TaskStatus, set[TaskStatus]] = {
    TaskStatus.CREATED: {TaskStatus.PENDING, TaskStatus.CANCELED, TaskStatus.FAILED},
    TaskStatus.PENDING: {TaskStatus.ASSIGNED, TaskStatus.CANCELED, TaskStatus.FAILED},
    TaskStatus.ASSIGNED: {
        TaskStatus.DISPATCHING,
        TaskStatus.PENDING,
        TaskStatus.CANCELED,
        TaskStatus.FAILED,
        TaskStatus.RECOVERY_REQUIRED,
    },
    TaskStatus.DISPATCHING: {
        TaskStatus.RUNNING,
        TaskStatus.CANCEL_REQUESTED,
        TaskStatus.RECOVERY_REQUIRED,
        TaskStatus.FAILED,
    },
    TaskStatus.RUNNING: {
        TaskStatus.COMPLETED,
        TaskStatus.CANCEL_REQUESTED,
        TaskStatus.RECOVERY_REQUIRED,
        TaskStatus.FAILED,
    },
    TaskStatus.CANCEL_REQUESTED: {
        TaskStatus.CANCELED,
        TaskStatus.RECOVERY_REQUIRED,
        TaskStatus.FAILED,
    },
    TaskStatus.COMPLETED: set(),
    TaskStatus.CANCELED: set(),
    TaskStatus.FAILED: set(),
    # Operator resolution only. Never back to RUNNING and never auto-complete.
    TaskStatus.RECOVERY_REQUIRED: {TaskStatus.CANCELED, TaskStatus.FAILED},
}


def can_transition(current: TaskStatus | str, nxt: TaskStatus | str) -> bool:
    return TaskStatus(nxt) in ALLOWED[TaskStatus(current)]
