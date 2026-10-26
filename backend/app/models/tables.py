from __future__ import annotations

from datetime import datetime

from sqlalchemy import Boolean, DateTime, Float, ForeignKey, Integer, String, Text, UniqueConstraint
from sqlalchemy.orm import Mapped, mapped_column

from app.models.base import Base


def _dt() -> DateTime:
    return DateTime(timezone=True)


class SchemaMigration(Base):
    __tablename__ = "schema_migrations"

    version: Mapped[int] = mapped_column(Integer, primary_key=True)
    applied_at: Mapped[datetime] = mapped_column(_dt())


class Robot(Base):
    __tablename__ = "robots"

    id: Mapped[str] = mapped_column(String(64), primary_key=True)
    name: Mapped[str] = mapped_column(String(120))
    mode: Mapped[str] = mapped_column(String(16))
    connection_status: Mapped[str] = mapped_column(String(16), default="online")
    operational_state: Mapped[str] = mapped_column(String(24), default="idle")
    battery_percent: Mapped[float] = mapped_column(Float, default=100)
    controller_ready: Mapped[bool] = mapped_column(Boolean, default=True)
    controller_kind: Mapped[str] = mapped_column(String(16), default="mock")
    pose_x: Mapped[float] = mapped_column(Float, default=0)
    pose_y: Mapped[float] = mapped_column(Float, default=0)
    pose_yaw: Mapped[float] = mapped_column(Float, default=0)
    pose_trustworthy: Mapped[bool] = mapped_column(Boolean, default=False)
    pose_source: Mapped[str] = mapped_column(String(64), default="unobserved")
    fault_code: Mapped[str | None] = mapped_column(String(64), nullable=True)
    estop_latched: Mapped[bool] = mapped_column(Boolean, default=False)
    agent_online: Mapped[bool] = mapped_column(Boolean, default=False)
    last_heartbeat_at: Mapped[datetime | None] = mapped_column(_dt(), nullable=True)
    created_at: Mapped[datetime] = mapped_column(_dt())
    updated_at: Mapped[datetime] = mapped_column(_dt())


class Task(Base):
    __tablename__ = "tasks"

    id: Mapped[str] = mapped_column(String(36), primary_key=True)
    idempotency_key: Mapped[str | None] = mapped_column(String(128), unique=True, nullable=True)
    robot_id: Mapped[str | None] = mapped_column(ForeignKey("robots.id"), nullable=True, index=True)
    requested_robot_id: Mapped[str | None] = mapped_column(String(64), nullable=True)
    task_type: Mapped[str] = mapped_column(String(32), default="navigate")
    status: Mapped[str] = mapped_column(String(32), index=True)
    destination_x: Mapped[float] = mapped_column(Float)
    destination_y: Mapped[float] = mapped_column(Float)
    destination_yaw: Mapped[float] = mapped_column(Float, default=0)
    destination_label: Mapped[str] = mapped_column(String(80))
    priority: Mapped[int] = mapped_column(Integer, default=0)
    queue_seq: Mapped[int] = mapped_column(Integer, default=0, index=True)
    owner_id: Mapped[str | None] = mapped_column(String(80), nullable=True)
    owner_epoch: Mapped[int] = mapped_column(Integer, default=0)
    lease_id: Mapped[str | None] = mapped_column(String(36), nullable=True)
    lease_expires_at: Mapped[datetime | None] = mapped_column(_dt(), nullable=True)
    execution_id: Mapped[str | None] = mapped_column(String(36), nullable=True, unique=True)
    goal_id: Mapped[str | None] = mapped_column(String(36), nullable=True)
    session_id: Mapped[str | None] = mapped_column(String(36), nullable=True)
    state_version: Mapped[int] = mapped_column(Integer, default=0)
    failure_reason: Mapped[str | None] = mapped_column(Text, nullable=True)
    recovery_reason: Mapped[str | None] = mapped_column(Text, nullable=True)
    goal_may_be_in_flight: Mapped[bool] = mapped_column(Boolean, default=False)
    dispatch_attempts: Mapped[int] = mapped_column(Integer, default=0)
    not_before: Mapped[datetime | None] = mapped_column(_dt(), nullable=True)
    last_decision_key: Mapped[str | None] = mapped_column(String(200), nullable=True)
    created_at: Mapped[datetime] = mapped_column(_dt())
    updated_at: Mapped[datetime] = mapped_column(_dt())
    last_heartbeat_at: Mapped[datetime | None] = mapped_column(_dt(), nullable=True)
    cancel_requested_at: Mapped[datetime | None] = mapped_column(_dt(), nullable=True)
    completed_at: Mapped[datetime | None] = mapped_column(_dt(), nullable=True)


class TaskEvent(Base):
    __tablename__ = "task_events"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    task_id: Mapped[str] = mapped_column(ForeignKey("tasks.id"), index=True)
    from_status: Mapped[str | None] = mapped_column(String(32), nullable=True)
    to_status: Mapped[str] = mapped_column(String(32))
    actor: Mapped[str] = mapped_column(String(80))
    owner_epoch: Mapped[int] = mapped_column(Integer, default=0)
    execution_id: Mapped[str | None] = mapped_column(String(36), nullable=True)
    goal_id: Mapped[str | None] = mapped_column(String(36), nullable=True)
    reason: Mapped[str] = mapped_column(Text, default="")
    detail_json: Mapped[str] = mapped_column(Text, default="{}")
    created_at: Mapped[datetime] = mapped_column(_dt())


class TaskOwnership(Base):
    __tablename__ = "task_ownership"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    task_id: Mapped[str] = mapped_column(ForeignKey("tasks.id"), index=True)
    robot_id: Mapped[str | None] = mapped_column(String(64), nullable=True)
    owner_id: Mapped[str | None] = mapped_column(String(80), nullable=True)
    owner_epoch: Mapped[int] = mapped_column(Integer, default=0)
    lease_id: Mapped[str | None] = mapped_column(String(36), nullable=True)
    action: Mapped[str] = mapped_column(String(32))
    detail: Mapped[str] = mapped_column(Text, default="")
    created_at: Mapped[datetime] = mapped_column(_dt())


class RobotSession(Base):
    __tablename__ = "robot_sessions"

    session_id: Mapped[str] = mapped_column(String(36), primary_key=True)
    robot_id: Mapped[str] = mapped_column(ForeignKey("robots.id"), index=True)
    agent_instance_id: Mapped[str] = mapped_column(String(36))
    started_at: Mapped[datetime] = mapped_column(_dt())
    ended_at: Mapped[datetime | None] = mapped_column(_dt(), nullable=True)


class NavigationGoal(Base):
    __tablename__ = "navigation_goals"
    __table_args__ = (UniqueConstraint("task_id", name="uq_navigation_goals_task"),)

    goal_id: Mapped[str] = mapped_column(String(36), primary_key=True)
    task_id: Mapped[str] = mapped_column(ForeignKey("tasks.id"), index=True)
    execution_id: Mapped[str] = mapped_column(String(36), unique=True)
    robot_id: Mapped[str] = mapped_column(ForeignKey("robots.id"))
    session_id: Mapped[str] = mapped_column(String(36))
    lease_id: Mapped[str] = mapped_column(String(36))
    owner_epoch: Mapped[int] = mapped_column(Integer)
    status: Mapped[str] = mapped_column(String(32))
    destination_x: Mapped[float] = mapped_column(Float)
    destination_y: Mapped[float] = mapped_column(Float)
    destination_yaw: Mapped[float] = mapped_column(Float)
    observed_x: Mapped[float | None] = mapped_column(Float, nullable=True)
    observed_y: Mapped[float | None] = mapped_column(Float, nullable=True)
    observed_yaw: Mapped[float | None] = mapped_column(Float, nullable=True)
    observation_failures: Mapped[int] = mapped_column(Integer, default=0)
    last_result_outcome: Mapped[str | None] = mapped_column(String(32), nullable=True)
    last_result_disposition: Mapped[str | None] = mapped_column(String(64), nullable=True)
    nav2_action_goal_id: Mapped[str | None] = mapped_column(String(64), nullable=True)
    nav2_action_goal_id_absent_reason: Mapped[str | None] = mapped_column(String(128), nullable=True)
    auto_complete_at: Mapped[datetime | None] = mapped_column(_dt(), nullable=True)
    created_at: Mapped[datetime] = mapped_column(_dt())
    updated_at: Mapped[datetime] = mapped_column(_dt())


class ControlLease(Base):
    __tablename__ = "control_leases"

    lease_id: Mapped[str] = mapped_column(String(36), primary_key=True)
    environment_id: Mapped[str] = mapped_column(String(80))
    robot_id: Mapped[str] = mapped_column(ForeignKey("robots.id"), index=True)
    session_id: Mapped[str] = mapped_column(String(36))
    owner_id: Mapped[str] = mapped_column(String(80))
    control_generation: Mapped[int] = mapped_column(Integer)
    gateway_boot_id: Mapped[str] = mapped_column(String(36))
    canonical_expires: Mapped[str] = mapped_column(String(40))
    expires_at: Mapped[datetime] = mapped_column(_dt())
    issued_at: Mapped[datetime] = mapped_column(_dt())
    revoked_at: Mapped[datetime | None] = mapped_column(_dt(), nullable=True)
    signature: Mapped[str] = mapped_column(String(128))
    status: Mapped[str] = mapped_column(String(16), default="active")


class TrafficZone(Base):
    __tablename__ = "traffic_zones"

    zone_id: Mapped[str] = mapped_column(String(32), primary_key=True)
    name: Mapped[str] = mapped_column(String(80))
    kind: Mapped[str] = mapped_column(String(32))
    capacity: Mapped[int] = mapped_column(Integer, default=1)
    bounds_x: Mapped[float] = mapped_column(Float)
    bounds_y: Mapped[float] = mapped_column(Float)
    bounds_w: Mapped[float] = mapped_column(Float)
    bounds_h: Mapped[float] = mapped_column(Float)


class ZoneReservation(Base):
    __tablename__ = "zone_reservations"

    reservation_id: Mapped[str] = mapped_column(String(36), primary_key=True)
    zone_id: Mapped[str] = mapped_column(ForeignKey("traffic_zones.zone_id"), index=True)
    robot_id: Mapped[str] = mapped_column(ForeignKey("robots.id"), index=True)
    task_id: Mapped[str | None] = mapped_column(ForeignKey("tasks.id"), nullable=True)
    owner_epoch: Mapped[int] = mapped_column(Integer, default=0)
    reservation_status: Mapped[str] = mapped_column(String(16), index=True)
    hold_reason: Mapped[str] = mapped_column(String(64), default="")
    acquired_at: Mapped[datetime] = mapped_column(_dt())
    expires_at: Mapped[datetime] = mapped_column(_dt())
    released_at: Mapped[datetime | None] = mapped_column(_dt(), nullable=True)
    updated_at: Mapped[datetime] = mapped_column(_dt())


class RobotTelemetry(Base):
    __tablename__ = "robot_telemetry"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    robot_id: Mapped[str] = mapped_column(ForeignKey("robots.id"), index=True)
    pose_x: Mapped[float] = mapped_column(Float)
    pose_y: Mapped[float] = mapped_column(Float)
    pose_yaw: Mapped[float] = mapped_column(Float)
    battery_percent: Mapped[float] = mapped_column(Float)
    source: Mapped[str] = mapped_column(String(64))
    trustworthy: Mapped[bool] = mapped_column(Boolean, default=False)
    observed_at: Mapped[datetime] = mapped_column(_dt())


class SystemEvent(Base):
    __tablename__ = "system_events"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    level: Mapped[str] = mapped_column(String(16))
    code: Mapped[str] = mapped_column(String(64), index=True)
    message: Mapped[str] = mapped_column(Text)
    payload_json: Mapped[str] = mapped_column(Text, default="{}")
    created_at: Mapped[datetime] = mapped_column(_dt(), index=True)
