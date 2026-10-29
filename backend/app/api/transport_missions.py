"""Transport Mission HTTP alias for call-requests.

fleet_server.py used /hospital/ui topics. FMS exposes the same business gates
over HTTP without adding those ROS topics.
"""
from __future__ import annotations

from fastapi import APIRouter, Request
from pydantic import BaseModel, Field

from app.api.call_requests import (
    _BoardIn,
    _CancelIn,
    _CreateIn,
    _PointIn,
    _call,
    _calls,
)

router = APIRouter(prefix="/api/transport-missions", tags=["transport-missions"])


class _MissionCreateIn(BaseModel):
    pickup: _PointIn
    dropoff: _PointIn
    robot_id: str | None = None
    priority: int = Field(default=5, ge=1, le=10)
    idempotency_key: str | None = None
    # Optional hospital label fields from fleet_server.py; coordinates remain authoritative.
    source_label: str | None = None
    destination_label: str | None = None


@router.get("")
def list_missions(request: Request) -> dict:
    body = _call(lambda: _calls(request).list_calls())
    return {"missions": body["calls"]}


@router.post("")
def create_mission(body: _MissionCreateIn, request: Request) -> dict:
    return _call(lambda: _calls(request).create(
        pickup=body.pickup.model_dump(),
        dropoff=body.dropoff.model_dump(),
        priority=body.priority,
        robot_id=body.robot_id,
        idempotency_key=body.idempotency_key,
    ))


@router.get("/{mission_id}")
def get_mission(mission_id: str, request: Request) -> dict:
    return _call(lambda: _calls(request).get(mission_id))


@router.post("/{mission_id}/refresh")
def refresh_mission(mission_id: str, request: Request) -> dict:
    return _call(lambda: _calls(request).refresh_scenario(mission_id))


@router.post("/{mission_id}/pickup-arrived-simulated")
def pickup_arrived(mission_id: str, request: Request) -> dict:
    return _call(lambda: _calls(request).mark_pickup_arrived_simulated(mission_id))


@router.post("/{mission_id}/board")
def board(mission_id: str, body: _BoardIn, request: Request) -> dict:
    return _call(lambda: _calls(request).confirm_boarding(
        mission_id, event_id=body.event_id, source=body.source,
    ))


@router.post("/{mission_id}/dropoff-arrived-simulated")
def dropoff_arrived(mission_id: str, request: Request) -> dict:
    return _call(lambda: _calls(request).mark_dropoff_arrived_simulated(mission_id))


@router.post("/{mission_id}/exit")
def exit_mission(mission_id: str, body: _BoardIn, request: Request) -> dict:
    return _call(lambda: _calls(request).confirm_exit(
        mission_id, event_id=body.event_id, source=body.source,
    ))


@router.post("/{mission_id}/cancel")
def cancel_mission(mission_id: str, body: _CancelIn, request: Request) -> dict:
    return _call(lambda: _calls(request).cancel(mission_id, reason=body.reason))
