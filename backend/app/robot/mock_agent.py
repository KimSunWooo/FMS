from __future__ import annotations

import threading
import uuid
from dataclasses import dataclass
from datetime import datetime

from sqlalchemy.orm import Session

from app.models.tables import Robot, RobotSession


@dataclass
class LiveSession:
    session_id: str
    robot_id: str
    started_at: datetime


class MockFleet:
    """In-process stand-in for three robot agents. It does not open a DDS socket."""

    def __init__(self) -> None:
        self.instance_id = str(uuid.uuid4())
        self._live: dict[str, LiveSession] = {}
        self.suppress_cancel_ack: set[str] = set()
        self._lock = threading.Lock()

    def is_alive(self, robot_id: str) -> bool:
        with self._lock:
            return robot_id in self._live

    def session_id(self, robot_id: str) -> str | None:
        with self._lock:
            live = self._live.get(robot_id)
            return None if live is None else live.session_id

    def start(self, session: Session, robot: Robot, now: datetime) -> str:
        with self._lock:
            existing = self._live.get(robot.id)
            if existing is not None:
                robot.agent_online = True
                return existing.session_id
            session_id = str(uuid.uuid4())
            self._live[robot.id] = LiveSession(session_id, robot.id, now)
        session.add(
            RobotSession(
                session_id=session_id,
                robot_id=robot.id,
                agent_instance_id=self.instance_id,
                started_at=now,
                ended_at=None,
            )
        )
        robot.agent_online = True
        robot.updated_at = now
        return session_id

    def stop(self, session: Session, robot: Robot, now: datetime) -> None:
        with self._lock:
            live = self._live.pop(robot.id, None)
        if live is not None:
            row = session.get(RobotSession, live.session_id)
            if row is not None:
                row.ended_at = now
        robot.agent_online = False
        robot.pose_trustworthy = False
        robot.pose_source = "agent_lost"
        robot.updated_at = now
        robot.last_heartbeat_at = robot.last_heartbeat_at
