"""Route preview API. Calculation only: no task, assignment, or Nav2 goal."""
from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from app.services.route_planner import PlanningFailure
from app.services.fleet_task_service import FleetTaskService
from app.services.route_preview import (
    inspect_point,
    load_graph_or_fail,
    load_registered,
    preview_route,
    resolve_start,
    validate_robot_starts,
)

router = APIRouter(prefix="/api/planning", tags=["planning"])


class _PointIn(BaseModel):
    x: float
    y: float


class _PreviewIn(BaseModel):
    robot_id: str
    goal: _PointIn
    start: _PointIn | None = None
    max_candidates: int | None = Field(default=None, ge=1, le=10)
    allow_diagonal: bool | None = None


def _failure(exc: PlanningFailure) -> JSONResponse:
    body = dict(exc.payload)
    body["code"] = exc.code
    body["detail"] = exc.detail
    body["reachable"] = False
    body.setdefault("paths", [])
    body.setdefault("shortest", None)
    return JSONResponse(status_code=422, content=body)


def _graph(request: Request, allow_diagonal: bool | None):
    settings = request.app.state.settings
    diagonal = settings.planning_allow_diagonal if allow_diagonal is None else allow_diagonal
    return load_graph_or_fail(
        settings.physical_map_yaml,
        planning_resolution_m=settings.planning_resolution_m,
        robot_radius_m=settings.planning_robot_radius_m,
        allow_diagonal=diagonal,
    )


def _snapshot(request: Request) -> dict | None:
    monitor = getattr(request.app.state, "physical_nav2", None)
    if monitor is None:
        return None
    return monitor.snapshot()


@router.get("/starts")
def planning_starts(request: Request) -> dict:
    settings = request.app.state.settings
    try:
        graph = _graph(request, None)
        registry = load_registered(settings.physical_fleet_path)
    except PlanningFailure as exc:
        return _failure(exc)
    return {
        "map": {
            "source": graph.grid.geometry.yaml_path.name,
            "planning_resolution_m": graph.planning_resolution_m,
            "robot_radius_m": graph.robot_radius_m,
            "collision_safety_verified": False,
        },
        "robots": validate_robot_starts(graph, registry, _snapshot(request)),
    }


@router.post("/points/inspect")
def planning_inspect(body: _PointIn, request: Request) -> dict:
    try:
        graph = _graph(request, None)
    except PlanningFailure as exc:
        return _failure(exc)
    report = inspect_point(graph, body.x, body.y)
    report["accepted_as_goal"] = report["traversable"]
    report["collision_safety_verified"] = False
    return report


@router.get("/reservations")
def planning_reservations(request: Request) -> dict:
    service = getattr(request.app.state, "fleet_tasks", None)
    if service is None:
        service = FleetTaskService(
            request.app.state.settings,
            request.app.state.runtime.db,
            lambda: _snapshot(request),
        )
        request.app.state.fleet_tasks = service
    return {
        "reservations": service.reservations(),
        "occupancy_class": "planned",
        "actual_motion_verified": False,
        "collision_safety_verified": False,
    }


@router.get("/conflicts")
def planning_conflicts(request: Request) -> dict:
    service = getattr(request.app.state, "fleet_tasks", None)
    if service is None:
        return {"conflicts": [], "occupancy_class": "planned", "actual_motion_verified": False, "collision_safety_verified": False}
    return service.conflict_report()


@router.post("/routes/preview")
def planning_preview(body: _PreviewIn, request: Request):
    settings = request.app.state.settings
    try:
        registry = load_registered(settings.physical_fleet_path)
        record = next((item for item in registry if item["robot_id"] == body.robot_id), None)
        if record is None:
            raise HTTPException(status_code=404, detail=f"Robot is not registered: {body.robot_id}")
        graph = _graph(request, body.allow_diagonal)
        start_x, start_y, source = resolve_start(
            record,
            None if body.start is None else {"x": body.start.x, "y": body.start.y},
            _snapshot(request),
        )
        count = settings.planning_max_candidates if body.max_candidates is None else body.max_candidates
        return preview_route(
            graph,
            robot_id=body.robot_id,
            goal_x=body.goal.x,
            goal_y=body.goal.y,
            start_x=start_x,
            start_y=start_y,
            start_source=source,
            max_candidates=count,
        )
    except PlanningFailure as exc:
        payload = dict(exc.payload)
        payload.setdefault("robot_id", body.robot_id)
        exc.payload = payload
        return _failure(exc)
