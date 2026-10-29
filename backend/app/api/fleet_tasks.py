"""Fleet task API. Separate from the mock /api/tasks scheduler."""
from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, Field

from app.services.fleet_task_machine import FleetTransitionError
from app.services.fleet_task_service import FleetTaskError, FleetTaskService
from app.services.robot_state import robot_state

router = APIRouter(prefix="/api/fleet-tasks", tags=["fleet-tasks"])


class _PointIn(BaseModel):
    x: float
    y: float


class _CreateIn(BaseModel):
    assignment_mode: str
    goal: _PointIn
    robot_id: str | None = None
    start: _PointIn | None = None
    priority: int = Field(default=5, ge=1, le=10)
    task_type: str = "navigate"
    label: str = ""
    idempotency_key: str | None = None


class _SelectIn(BaseModel):
    path_id: str


class _HomeIn(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    approve: bool = False


class _CancelIn(BaseModel):
    reason: str = ""


class _ArchiveIn(BaseModel):
    archived_by: str = "operator"


def _service(request: Request) -> FleetTaskService:
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

    existing = FleetTaskService(
        request.app.state.settings, request.app.state.runtime.db, snapshot, state,
    )
    request.app.state.fleet_tasks = existing
    return existing


def _call(action):
    try:
        return action()
    except FleetTaskError as exc:
        raise HTTPException(status_code=exc.status_code, detail=exc.detail) from exc
    except FleetTransitionError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@router.get("/queue")
def fleet_queue(request: Request) -> dict:
    return {"tasks": _call(lambda: _service(request).queue())}


@router.post("/replan")
def fleet_replan(request: Request) -> dict:
    """Re-evaluate waiting work. Does not deliver a task or start Nav2."""
    return {"results": _call(lambda: _service(request).plan_pending())}


@router.get("/browse")
def fleet_browse(
    request: Request,
    scope: str = "active",
    status: str | None = None,
    robot_id: str | None = None,
    q: str | None = None,
    page: int = 1,
    page_size: int = 10,
) -> dict:
    return _call(lambda: _service(request).browse(
        scope=scope,
        status=status or None,
        robot_id=robot_id or None,
        query=q or None,
        page=page,
        page_size=page_size,
    ))


@router.get("/facilities")
def fleet_facilities(request: Request) -> dict:
    return _call(lambda: _service(request).facilities())


@router.get("/availability")
def fleet_availability(request: Request) -> dict:
    return {"robots": _call(lambda: _service(request).robot_availability())}


@router.post("/robots/{robot_id}/home")
def fleet_approve_home(robot_id: str, body: _HomeIn, request: Request) -> dict:
    return _call(lambda: _service(request).approve_home(robot_id, body.x, body.y, body.yaw, body.approve))


@router.post("")
def fleet_create(body: _CreateIn, request: Request) -> dict:
    return _call(lambda: _service(request).create(
        mode=body.assignment_mode,
        goal_x=body.goal.x,
        goal_y=body.goal.y,
        robot_id=body.robot_id,
        start=None if body.start is None else body.start.model_dump(),
        priority=body.priority,
        task_type=body.task_type,
        label=body.label,
        idempotency_key=body.idempotency_key,
    ))


@router.get("")
def fleet_list(request: Request) -> dict:
    return {"tasks": _call(lambda: _service(request).list_tasks())}


@router.get("/{task_id}")
def fleet_get(task_id: str, request: Request) -> dict:
    return _call(lambda: _service(request).get(task_id))


@router.post("/{task_id}/plan")
def fleet_plan(task_id: str, request: Request) -> dict:
    return _call(lambda: _service(request).plan(task_id))


@router.post("/{task_id}/select")
def fleet_select(task_id: str, body: _SelectIn, request: Request) -> dict:
    return _call(lambda: _service(request).select_path(task_id, body.path_id))


@router.post("/{task_id}/dispatch")
def fleet_dispatch(task_id: str, request: Request) -> dict:
    return _call(lambda: _service(request).dispatch(task_id))


@router.post("/{task_id}/simulated-motion")
def fleet_simulated_motion(task_id: str, request: Request) -> dict:
    return _call(lambda: _service(request).mark_simulated_motion(task_id))


@router.post("/{task_id}/archive")
async def fleet_archive(task_id: str, request: Request) -> dict:
    actor = "operator"
    raw = await request.body()
    if raw.strip():
        actor = _ArchiveIn.model_validate_json(raw).archived_by or "operator"
    return _call(lambda: _service(request).set_archived(task_id, True, actor))


@router.post("/{task_id}/restore")
async def fleet_restore(task_id: str, request: Request) -> dict:
    actor = "operator"
    raw = await request.body()
    if raw.strip():
        actor = _ArchiveIn.model_validate_json(raw).archived_by or "operator"
    return _call(lambda: _service(request).set_archived(task_id, False, actor))


@router.post("/{task_id}/cancel")
async def fleet_cancel(task_id: str, request: Request) -> dict:
    reason = ""
    raw = await request.body()
    if raw.strip():
        reason = _CancelIn.model_validate_json(raw).reason
    return _call(lambda: _service(request).cancel(task_id, reason))
