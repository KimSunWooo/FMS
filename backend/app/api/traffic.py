from __future__ import annotations

from fastapi import APIRouter, Request

from app.api.schemas import ReleaseIn

router = APIRouter(prefix="/api/traffic", tags=["traffic"])


@router.get("/zones")
def zones(request: Request):
    return request.app.state.runtime.zones()


@router.get("/reservations")
def reservations(request: Request):
    return {"reservations": request.app.state.runtime.reservations()}


@router.post("/reservations/{reservation_id}/release")
def release(reservation_id: str, body: ReleaseIn, request: Request):
    row = request.app.state.runtime.release_reservation(
        reservation_id,
        reason=body.reason,
        position_observed_clear=body.position_observed_clear,
    )
    return {"reservation": row, "task_status_changed": False}
