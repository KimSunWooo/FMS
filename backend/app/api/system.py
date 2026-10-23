from __future__ import annotations

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse

router = APIRouter(prefix="/api/system", tags=["system"])


@router.get("/health")
def health(request: Request):
    return request.app.state.runtime.health()


@router.get("/readiness")
def readiness(request: Request):
    body = request.app.state.runtime.health()
    body["ready"] = body["database"] == "ok" and request.app.state.runtime.started
    if not body["ready"]:
        return JSONResponse(status_code=503, content=body)
    return body
