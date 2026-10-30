"""Persist one auto-seed attempt across API restarts (same robot/domain/map)."""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def session_path(data_dir: Path, robot_id: str) -> Path:
    return Path(data_dir) / "ros_ipc" / robot_id / "initial_pose_session.json"


def load_session(data_dir: Path, robot_id: str) -> dict[str, Any] | None:
    path = session_path(data_dir, robot_id)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def save_session(data_dir: Path, robot_id: str, payload: dict[str, Any]) -> None:
    path = session_path(data_dir, robot_id)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


def clear_session(data_dir: Path, robot_id: str) -> None:
    path = session_path(data_dir, robot_id)
    try:
        path.unlink(missing_ok=True)
    except OSError:
        pass


def same_seed_context(session: dict[str, Any] | None, *, robot_id: str, domain: int, map_sha: str | None) -> bool:
    if not isinstance(session, dict):
        return False
    if session.get("robot_id") != robot_id:
        return False
    if int(session.get("ros_domain_id", -1)) != int(domain):
        return False
    if map_sha and session.get("map_sha256") and session.get("map_sha256") != map_sha:
        return False
    return True


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()
