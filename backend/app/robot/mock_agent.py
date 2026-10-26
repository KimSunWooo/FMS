from __future__ import annotations

import threading
import uuid
from dataclasses import dataclass
from datetime import datetime, timedelta

from sqlalchemy.orm import Session

from app.core.clock import as_utc
from app.models.tables import Robot, RobotSession


@dataclass
class LiveSession:
    session_id: str
    robot_id: str
    started_at: datetime
    last_heartbeat_at: datetime
    external: bool


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

    def start(self, session: Session, robot: Robot, now: datetime, *, external: bool = False) -> str:
        with self._lock:
            existing = self._live.get(robot.id)
            if existing is not None:
                robot.agent_online = True
                existing.last_heartbeat_at = now
                return existing.session_id
            session_id = str(uuid.uuid4())
            self._live[robot.id] = LiveSession(session_id, robot.id, now, now, external)
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

    def touch(self, robot_id: str, session_id: str, now: datetime) -> bool:
        with self._lock:
            live = self._live.get(robot_id)
            if live is None or live.session_id != session_id:
                return False
            live.last_heartbeat_at = now
            return True

    def expire_stale(self, session: Session, now: datetime, timeout_seconds: float) -> list[str]:
        """Drop external sessions whose heartbeat stopped.

        In-process mock sessions stay up with the FMS process. An external
        agent that stops heartbeating is treated as gone. The caller still has
        to move any in-flight goal to recovery; this method does not send one.
        """
        moment = as_utc(now)
        expired: list[LiveSession] = []
        if moment is None:
            return []
        limit = timedelta(seconds=timeout_seconds)
        with self._lock:
            for robot_id, live in list(self._live.items()):
                if not live.external:
                    continue
                beat = as_utc(live.last_heartbeat_at)
                if beat is None or moment - beat > limit:
                    expired.append(self._live.pop(robot_id))
        ended: list[str] = []
        for live in expired:
            row = session.get(RobotSession, live.session_id)
            if row is not None and row.ended_at is None:
                row.ended_at = now
            robot = session.get(Robot, live.robot_id)
            if robot is not None:
                robot.agent_online = False
                robot.pose_trustworthy = False
                robot.pose_source = "agent_lost"
                robot.updated_at = now
            ended.append(live.robot_id)
        return ended
