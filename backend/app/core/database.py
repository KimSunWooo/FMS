from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
from typing import Iterator

from sqlalchemy import create_engine, event, text
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session, sessionmaker
from sqlalchemy.pool import NullPool

from app.models.base import Base
from app.models.tables import SchemaMigration

LATEST_SCHEMA = 2

_PARTIAL_INDEXES = (
    """
    CREATE UNIQUE INDEX IF NOT EXISTS uq_tasks_one_active_per_robot
    ON tasks(robot_id)
    WHERE status IN (
        'ASSIGNED','DISPATCHING','RUNNING','CANCEL_REQUESTED','RECOVERY_REQUIRED'
    )
    AND robot_id IS NOT NULL
    """,
    """
    CREATE UNIQUE INDEX IF NOT EXISTS uq_zone_one_active
    ON zone_reservations(zone_id)
    WHERE reservation_status IN ('held','blocked')
    """,
)


def make_engine(database_url: str) -> Engine:
    engine = create_engine(
        database_url,
        poolclass=NullPool,
        connect_args={"check_same_thread": False, "timeout": 30},
    )

    @event.listens_for(engine, "connect")
    def _connect(dbapi_connection, _record) -> None:
        # Autocommit at the DBAPI layer so SQLAlchemy's begin hook can
        # issue BEGIN IMMEDIATE itself. That is the write lock we rely on.
        dbapi_connection.isolation_level = None
        cursor = dbapi_connection.cursor()
        cursor.execute("PRAGMA foreign_keys=ON")
        cursor.execute("PRAGMA journal_mode=WAL")
        cursor.execute("PRAGMA busy_timeout=8000")
        cursor.close()

    @event.listens_for(engine, "begin")
    def _begin(connection) -> None:
        connection.exec_driver_sql("BEGIN IMMEDIATE")

    return engine


class Database:
    def __init__(self, database_url: str):
        self.engine = make_engine(database_url)
        self.factory = sessionmaker(
            bind=self.engine,
            autoflush=False,
            expire_on_commit=False,
            future=True,
        )

    def migrate(self) -> None:
        Base.metadata.create_all(self.engine)
        with self.engine.begin() as connection:
            for statement in _PARTIAL_INDEXES:
                connection.exec_driver_sql(statement)
            columns = {
                row[1]
                for row in connection.execute(text("PRAGMA table_info(navigation_goals)")).fetchall()
            }
            if "nav2_action_goal_id" not in columns:
                connection.exec_driver_sql(
                    "ALTER TABLE navigation_goals ADD COLUMN nav2_action_goal_id VARCHAR(64)"
                )
            current = connection.execute(
                text("SELECT version FROM schema_migrations ORDER BY version DESC LIMIT 1")
            ).scalar()
            if current is None:
                connection.execute(
                    text("INSERT INTO schema_migrations (version, applied_at) VALUES (:v, :t)"),
                    {"v": LATEST_SCHEMA, "t": datetime.now(timezone.utc).isoformat()},
                )
            elif int(current) > LATEST_SCHEMA:
                raise RuntimeError(
                    f"Database schema {current} is newer than this build ({LATEST_SCHEMA})."
                )
            elif int(current) < LATEST_SCHEMA:
                connection.execute(
                    text("INSERT INTO schema_migrations (version, applied_at) VALUES (:v, :t)"),
                    {"v": LATEST_SCHEMA, "t": datetime.now(timezone.utc).isoformat()},
                )

    @contextmanager
    def write(self) -> Iterator[Session]:
        session = self.factory()
        try:
            yield session
            session.commit()
        except Exception:
            session.rollback()
            raise
        finally:
            session.close()

    @contextmanager
    def read(self) -> Iterator[Session]:
        """Read-only scope. Rollback on the way out so a GET cannot commit."""
        session = self.factory()
        try:
            yield session
        finally:
            session.rollback()
            session.close()
