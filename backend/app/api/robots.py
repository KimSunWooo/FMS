from __future__ import annotations

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse

from app.api.schemas import ClearEstopIn, EstopIn, RobotCreate
from app.core.errors import FmsError

router = APIRouter(prefix="/api/robots", tags=["robots"])


def _runtime(request: Request):
    return request.app.state.runtime


@router.get("")
def list_robots(request: Request):
    return {"robots": _runtime(request).list_robots()}


@router.post("", status_code=201)
def register_robot(body: RobotCreate, request: Request):
    robot = _runtime(request).register_robot(
        robot_id=body.robot_id,
        name=body.name,
        mode=body.mode,
        battery_percent=body.battery_percent,
        x=body.x,
        y=body.y,
        yaw=body.yaw,
        controller_ready=body.controller_ready,
    )
    return robot


@router.get("/{robot_id}")
def get_robot(robot_id: str, request: Request):
    return _runtime(request).get_robot(robot_id)


@router.get("/{robot_id}/status")
def robot_status(robot_id: str, request: Request):
    robot = _runtime(request).get_robot(robot_id)
    return {
        "robot_id": robot["robot_id"],
        "connection_status": robot["connection_status"],
        "operational_state": robot["operational_state"],
        "battery_percent": robot["battery_percent"],
        "controller_kind": robot["controller_kind"],
        "controller_ready": robot["controller_ready"],
        "agent_online": robot["agent_online"],
        "pose_trustworthy": robot["pose_trustworthy"],
        "pose_source": robot["pose_source"],
        "estop_latched": robot["estop_latched"],
        "active_task_id": robot["active_task_id"],
        "pose": robot["pose"],
    }


@router.post("/{robot_id}/offline")
def offline(robot_id: str, request: Request):
    return _runtime(request).set_offline(robot_id)


@router.post("/{robot_id}/online")
def online(robot_id: str, request: Request):
    return _runtime(request).set_online(robot_id)


@router.post("/{robot_id}/emergency-stop", status_code=202)
def emergency_stop(robot_id: str, body: EstopIn, request: Request):
    return _runtime(request).emergency_stop(robot_id, body.reason)


@router.post("/{robot_id}/estop/clear")
def clear_estop(robot_id: str, body: ClearEstopIn, request: Request):
    return _runtime(request).clear_estop(robot_id, body.acknowledge)


@router.post("/{robot_id}/mock-observe")
def mock_observe(robot_id: str, request: Request):
    return _runtime(request).mock_observe(robot_id)


def install_error_handler(app) -> None:
    @app.exception_handler(FmsError)
    async def _fms_error(_request, exc: FmsError):
        return JSONResponse(
            status_code=exc.http_status,
            content={"error": {"code": exc.code, "message": exc.message}},
        )
