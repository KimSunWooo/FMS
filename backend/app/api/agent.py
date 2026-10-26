from __future__ import annotations

from fastapi import APIRouter, Request

from app.api.schemas import (
    AgentClaimIn,
    AgentGoalEventIn,
    AgentHeartbeatIn,
    AgentPoseIn,
    AgentSessionIn,
    AgentStopIn,
)
from app.core.errors import FmsError

router = APIRouter(prefix="/api/agent", tags=["agent"])


def _runtime(request: Request):
    return request.app.state.runtime


@router.post("/session", status_code=201)
def open_session(body: AgentSessionIn, request: Request):
    return _runtime(request).open_agent_session(body.robot_id)


@router.post("/session/replace")
def replace_session(body: AgentSessionIn, request: Request):
    return _runtime(request).replace_agent_session(body.robot_id)


@router.post("/stop")
def stop_session(body: AgentStopIn, request: Request):
    return _runtime(request).agent_stop(body.robot_id, body.session_id)


@router.post("/heartbeat")
def heartbeat(body: AgentHeartbeatIn, request: Request):
    return _runtime(request).agent_heartbeat(body.robot_id, body.session_id)


@router.post("/claim")
def claim(body: AgentClaimIn, request: Request):
    return _runtime(request).agent_claim(
        robot_id=body.robot_id,
        session_id=body.session_id,
        owner_epoch=body.owner_epoch,
        lease_id=body.lease_id,
        goal_id=body.goal_id,
    )


@router.get("/assignment")
def assignment(robot_id: str, session_id: str, request: Request):
    return _runtime(request).agent_assignment(robot_id, session_id)


@router.post("/goal-event")
def goal_event(body: AgentGoalEventIn, request: Request):
    if body.kind == "result" and not body.outcome:
        raise FmsError("a result event needs an outcome")
    return _runtime(request).agent_goal_event(
        robot_id=body.robot_id,
        session_id=body.session_id,
        task_id=body.task_id,
        goal_id=body.goal_id,
        execution_id=body.execution_id,
        owner_epoch=body.owner_epoch,
        kind=body.kind,
        outcome=body.outcome,
        pose_x=body.pose_x,
        pose_y=body.pose_y,
        pose_yaw=body.pose_yaw,
        pose_trustworthy=body.pose_trustworthy,
        nav2_action_goal_id=body.nav2_action_goal_id,
        nav2_action_goal_id_absent_reason=body.nav2_action_goal_id_absent_reason,
    )


@router.post("/pose")
def pose(body: AgentPoseIn, request: Request):
    return _runtime(request).agent_pose(
        robot_id=body.robot_id,
        session_id=body.session_id,
        pose_x=body.pose_x,
        pose_y=body.pose_y,
        pose_yaw=body.pose_yaw,
        frame_id=body.frame_id,
    )
