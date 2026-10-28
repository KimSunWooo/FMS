"""Lifecycle for planned fleet tasks. Independent of the mock Task state machine."""
from __future__ import annotations

CREATED = "CREATED"
PLANNING = "PLANNING"
PLANNED = "PLANNED"
RESERVED = "RESERVED"
DISPATCHING = "DISPATCHING"
DELIVERED = "DELIVERED"
ACKNOWLEDGED = "ACKNOWLEDGED"
WAITING = "WAITING"
REPLANNING = "REPLANNING"
CANCELLED = "CANCELLED"
CANCEL_PENDING = "CANCEL_PENDING"
FAILED = "FAILED"
UNREACHABLE = "UNREACHABLE"

TERMINAL = {ACKNOWLEDGED, CANCELLED, FAILED, UNREACHABLE}

# ACKNOWLEDGED stays visible as current work. A receive ACK is not a verified completion.
ACTIVE_QUEUE = {
    CREATED, PLANNING, PLANNED, RESERVED, WAITING, REPLANNING,
    DISPATCHING, DELIVERED, ACKNOWLEDGED, CANCEL_PENDING,
}
HISTORY = {CANCELLED, FAILED, UNREACHABLE}

ALLOWED: dict[str, set[str]] = {
    CREATED: {PLANNING, CANCELLED, FAILED},
    PLANNING: {PLANNED, RESERVED, WAITING, UNREACHABLE, FAILED, CANCELLED, REPLANNING},
    PLANNED: {RESERVED, REPLANNING, CANCELLED, FAILED, UNREACHABLE},
    RESERVED: {DISPATCHING, REPLANNING, CANCELLED, FAILED},
    DISPATCHING: {DELIVERED, FAILED, CANCELLED, CANCEL_PENDING},
    DELIVERED: {ACKNOWLEDGED, FAILED, CANCELLED, CANCEL_PENDING},
    WAITING: {PLANNING, RESERVED, REPLANNING, CANCELLED, FAILED},
    REPLANNING: {PLANNING, CANCELLED, FAILED},
    ACKNOWLEDGED: {CANCEL_PENDING},
    CANCEL_PENDING: {CANCELLED},
    CANCELLED: set(),
    FAILED: {PLANNING},
    UNREACHABLE: {PLANNING, CANCELLED},
}


class FleetTransitionError(Exception):
    def __init__(self, current: str, nxt: str) -> None:
        super().__init__(f"Cannot move fleet task from {current} to {nxt}")
        self.current = current
        self.nxt = nxt


def can_transition(current: str, nxt: str) -> bool:
    return nxt in ALLOWED.get(current, set())


def require_transition(current: str, nxt: str) -> None:
    if not can_transition(current, nxt):
        raise FleetTransitionError(current, nxt)
