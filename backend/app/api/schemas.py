from __future__ import annotations

import re

from pydantic import BaseModel, Field, field_validator

_ROBOT_ID = re.compile(r"^[a-z0-9][a-z0-9-]{0,31}$")


class DestinationIn(BaseModel):
    x: float
    y: float
    yaw: float = 0
    label: str = Field(min_length=1, max_length=80)

    @field_validator("x", "y", "yaw")
    @classmethod
    def finite(cls, value: float) -> float:
        if value != value or value in (float("inf"), float("-inf")):
            raise ValueError("must be finite")
        return value


class TaskCreate(BaseModel):
    destination: DestinationIn
    priority: int = Field(default=0, ge=-10, le=10)
    robot_id: str | None = None
    idempotency_key: str | None = Field(default=None, max_length=128)

    @field_validator("robot_id")
    @classmethod
    def robot(cls, value: str | None) -> str | None:
        if value is not None and _ROBOT_ID.fullmatch(value) is None:
            raise ValueError("invalid robot id")
        return value


class RobotCreate(BaseModel):
    robot_id: str
    name: str = Field(min_length=1, max_length=120)
    mode: str = "mock"
    battery_percent: float = Field(default=100, ge=0, le=100)
    x: float = 0
    y: float = 0
    yaw: float = 0
    controller_ready: bool = True

    @field_validator("robot_id")
    @classmethod
    def robot(cls, value: str) -> str:
        if _ROBOT_ID.fullmatch(value) is None:
            raise ValueError("invalid robot id")
        return value


class ResolveIn(BaseModel):
    outcome: str
    reason: str = Field(min_length=3, max_length=300)
    acknowledge_position_unknown: bool = False


class EstopIn(BaseModel):
    reason: str = Field(min_length=3, max_length=300)


class ClearEstopIn(BaseModel):
    acknowledge: bool = False


class ReleaseIn(BaseModel):
    reason: str = Field(min_length=3, max_length=300)
    position_observed_clear: bool = False


class AgentSessionIn(BaseModel):
    robot_id: str

    @field_validator("robot_id")
    @classmethod
    def robot(cls, value: str) -> str:
        if _ROBOT_ID.fullmatch(value) is None:
            raise ValueError("invalid robot id")
        return value


class AgentHeartbeatIn(AgentSessionIn):
    session_id: str = Field(min_length=8, max_length=64)


class AgentStopIn(AgentHeartbeatIn):
    pass


class AgentClaimIn(AgentHeartbeatIn):
    owner_epoch: int = Field(ge=0)
    lease_id: str = Field(min_length=8, max_length=64)
    goal_id: str = Field(min_length=8, max_length=64)


class AgentGoalEventIn(AgentClaimIn):
    task_id: str = Field(min_length=8, max_length=64)
    execution_id: str = Field(min_length=8, max_length=64)
    kind: str
    outcome: str | None = None
    pose_x: float | None = None
    pose_y: float | None = None
    pose_yaw: float | None = None
    pose_trustworthy: bool = False
    nav2_action_goal_id: str | None = Field(default=None, max_length=64)
    nav2_action_goal_id_absent_reason: str | None = Field(default=None, max_length=128)

    @field_validator("kind")
    @classmethod
    def kind_ok(cls, value: str) -> str:
        if value not in {"accepted", "result", "disconnected"}:
            raise ValueError("kind must be accepted, result, or disconnected")
        return value


class AgentPoseIn(AgentHeartbeatIn):
    pose_x: float
    pose_y: float
    pose_yaw: float = 0.0
    frame_id: str = "map"
