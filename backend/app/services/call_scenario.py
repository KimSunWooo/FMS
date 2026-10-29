"""Transport Mission labels. Separate from FleetTask machine states.

Inspired by the standalone hospital fleet_server.py business phases, but
implemented over two FleetTask rows (pickup + dropoff). No /hospital/ui or
/pinky/mission_* ROS topics are added.
"""
from __future__ import annotations

# Canonical Transport Mission phases (application layer).
REQUESTED = "REQUESTED"
PICKUP_PLANNING = "PICKUP_PLANNING"
PICKUP_READY = "PICKUP_READY"
PICKUP_DISPATCHED = "PICKUP_DISPATCHED"
PICKUP_ARRIVED = "PICKUP_ARRIVED"
WAIT_BOARD = "WAIT_BOARD"
BOARD_CONFIRMED = "BOARD_CONFIRMED"
DROPOFF_READY = "DROPOFF_READY"
DROPOFF_DISPATCHED = "DROPOFF_DISPATCHED"
DROPOFF_ARRIVED = "DROPOFF_ARRIVED"
WAIT_EXIT = "WAIT_EXIT"
EXIT_CONFIRMED = "EXIT_CONFIRMED"
DONE = "DONE"
CANCELING = "CANCELING"
CANCELED = "CANCELED"
CALL_FAILED = "CALL_FAILED"
CALL_BLOCKED = "CALL_BLOCKED"

# Backward-compatible aliases used by earlier call_request tests/docs.
CALL_CREATED = REQUESTED
PICKUP_PLANNED = PICKUP_READY
WAITING_FOR_BOARDING = WAIT_BOARD
BOARDING_CONFIRMED = BOARD_CONFIRMED

LEG_PICKUP = "pickup"
LEG_DROPOFF = "dropoff"

FINAL_STATES = {DONE, CANCELED, CALL_FAILED, CALL_BLOCKED}
