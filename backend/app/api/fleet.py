from __future__ import annotations

from fastapi import APIRouter, Request

router = APIRouter(prefix="/api/fleet", tags=["fleet"])


@router.get("/status")
def fleet_status(request: Request):
    return request.app.state.runtime.fleet_status()


@router.get("/events")
def fleet_events(request: Request, limit: int = 100):
    capped = max(1, min(limit, 500))
    return {"events": request.app.state.runtime.events(capped)}
