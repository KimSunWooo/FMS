from __future__ import annotations

from fastapi import APIRouter, Header, Request

from app.api.schemas import ResolveIn, TaskCreate

router = APIRouter(prefix="/api/tasks", tags=["tasks"])


def _runtime(request: Request):
    return request.app.state.runtime


@router.post("", status_code=202)
def create_task(
    body: TaskCreate,
    request: Request,
    idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
):
    key = idempotency_key or body.idempotency_key
    task = _runtime(request).create_task(
        x=body.destination.x,
        y=body.destination.y,
        yaw=body.destination.yaw,
        label=body.destination.label,
        priority=body.priority,
        robot_id=body.robot_id,
        idempotency_key=key,
    )
    return {
        "accepted": True,
        "motion_completed": False,
        "task": task,
    }


@router.get("")
def list_tasks(request: Request, status: str | None = None):
    return {"tasks": _runtime(request).list_tasks(status)}


@router.get("/{task_id}")
def get_task(task_id: str, request: Request):
    return _runtime(request).get_task(task_id)


@router.post("/{task_id}/cancel", status_code=202)
def cancel_task(task_id: str, request: Request):
    task = _runtime(request).cancel_task(task_id)
    return {
        "accepted": True,
        "motion_completed": False,
        "task": task,
    }


@router.post("/{task_id}/resolve")
def resolve_task(task_id: str, body: ResolveIn, request: Request):
    task = _runtime(request).resolve_task(
        task_id,
        outcome=body.outcome,
        reason=body.reason,
        acknowledge_position_unknown=body.acknowledge_position_unknown,
    )
    return {"task": task, "goal_replayed": False}
