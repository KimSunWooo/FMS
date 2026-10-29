"""User-call HTTP API. Application orchestration only; no new ROS topics."""
from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, Field

from app.services.call_request_service import CallRequestError, CallRequestService
from app.services.fleet_task_service import FleetTaskError, FleetTaskService
from app.services.robot_state import robot_state

router = APIRouter(prefix="/api/call-requests", tags=["call-requests"])


class _PointIn(BaseModel):
    x: float
    y: float


class _CreateIn(BaseModel):
    pickup: _PointIn
    dropoff: _PointIn
    robot_id: str | None = None
    priority: int = Field(default=5, ge=1, le=10)
    idempotency_key: str | None = None


class _BoardIn(BaseModel):
    event_id: str
    source: str = "mock"


def _fleet(request: Request) -> FleetTaskService:
    existing = getattr(request.app.state, "fleet_tasks", None)
    if existing is not None:
        return existing

    def snapshot() -> dict | None:
        monitor = getattr(request.app.state, "physical_nav2", None)
        return None if monitor is None else monitor.snapshot()

    def state(robot_id: str) -> dict:
        sample = snapshot()
        if isinstance(sample, dict) and sample.get("robot_id") not in (None, robot_id):
            sample = None
        return robot_state(robot_id, request.app.state.settings, sample)

    service = FleetTaskService(
        request.app.state.settings, request.app.state.runtime.db, snapshot, state,
    )
    request.app.state.fleet_tasks = service
    return service


def _calls(request: Request) -> CallRequestService:
    existing = getattr(request.app.state, "call_requests", None)
    if existing is not None:
        return existing
    service = CallRequestService(_fleet(request))
    request.app.state.call_requests = service
    return service


def _call(action):
    try:
        return action()
    except CallRequestError as exc:
        raise HTTPException(status_code=exc.status_code, detail=exc.detail) from exc
    except FleetTaskError as exc:
        raise HTTPException(status_code=exc.status_code, detail=exc.detail) from exc


@router.get("")
def list_calls(request: Request) -> dict:
    return {"calls": _call(lambda: _calls(request).list_calls())}


@router.post("")
def create_call(body: _CreateIn, request: Request) -> dict:
    return _call(lambda: _calls(request).create(
        pickup=body.pickup.model_dump(),
        dropoff=body.dropoff.model_dump(),
        priority=body.priority,
        robot_id=body.robot_id,
        idempotency_key=body.idempotency_key,
    ))


@router.get("/{call_id}")
def get_call(call_id: str, request: Request) -> dict:
    return _call(lambda: _calls(request).get(call_id))


@router.post("/{call_id}/refresh")
def refresh_call(call_id: str, request: Request) -> dict:
    return _call(lambda: _calls(request).refresh_scenario(call_id))


@router.post("/{call_id}/pickup-arrived-simulated")
def pickup_arrived_simulated(call_id: str, request: Request) -> dict:
    """Mock arrival evidence for virtual receivers only. Not physical proof."""
    return _call(lambda: _calls(request).mark_pickup_arrived_simulated(call_id))


@router.post("/{call_id}/boarding")
def confirm_boarding(call_id: str, body: _BoardIn, request: Request) -> dict:
    """Mock/operator boarding confirmation. No ROS /fms/boarding topic is added."""
    return _call(lambda: _calls(request).confirm_boarding(
        call_id, event_id=body.event_id, source=body.source,
    ))


@router.post("/{call_id}/dropoff-arrived-simulated")
def dropoff_arrived_simulated(call_id: str, request: Request) -> dict:
    """Mock dropoff arrival for virtual receivers only."""
    return _call(lambda: _calls(request).mark_dropoff_arrived_simulated(call_id))


@router.post("/{call_id}/exit")
def confirm_exit(call_id: str, body: _BoardIn, request: Request) -> dict:
    """Mock/operator exit confirmation. No ROS exit topic is added."""
    return _call(lambda: _calls(request).confirm_exit(
        call_id, event_id=body.event_id, source=body.source,
    ))


class _CancelIn(BaseModel):
    reason: str = "operator cancel"


@router.post("/{call_id}/cancel")
def cancel_call(call_id: str, body: _CancelIn, request: Request) -> dict:
    return _call(lambda: _calls(request).cancel(call_id, reason=body.reason))
