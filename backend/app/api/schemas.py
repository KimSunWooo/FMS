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
