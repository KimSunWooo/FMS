"""Classify Nav2 poll and live TF timestamps for the 5cm start gate.

A missing, invalid, or expired stamp stays that way. This module does not
replace it with the current time or copy an older pose forward.
"""
from __future__ import annotations

import math
from datetime import datetime, timezone

POLL_MAX_AGE_S = 45.0
LIVE_TF_MAX_AGE_S = 4.0


def stamp_status(value, *, max_age: float, now: datetime | None = None) -> tuple[str, float | None]:
    """Return one of missing, invalid, expired, fresh, plus the age in seconds."""
    now = now or datetime.now(timezone.utc)
    if value is None or value == "":
        return "missing", None
    if not isinstance(value, str):
        return "invalid", None
    try:
        stamp = datetime.fromisoformat(value)
    except ValueError:
        return "invalid", None
    if stamp.tzinfo is None:
        return "invalid", None
    age = (now - stamp.astimezone(timezone.utc)).total_seconds()
    if not math.isfinite(age) or age < 0:
        return "invalid", age
    if age > max_age:
        return "expired", age
    return "fresh", age


def _stamp_reason(name: str, status: str, age: float | None) -> str:
    if status == "missing":
        return f"{name} missing"
    if status == "expired" and age is not None:
        return f"{name} expired: {age:.1f}s"
    return f"{name} invalid"


def _finite_pose(pose) -> bool:
    if not isinstance(pose, dict):
        return False
    try:
        return all(math.isfinite(float(pose[key])) for key in ("x", "y", "yaw"))
    except (KeyError, TypeError, ValueError):
        return False


def observation_blockers(snapshot: dict, *, now: datetime | None = None) -> list[str]:
    """Reasons the 5cm test must not send a goal. Empty means the timestamps and map match."""
    reasons: list[str] = []
    status, age = stamp_status(snapshot.get("checked_at"), max_age=POLL_MAX_AGE_S, now=now)
    if status != "fresh":
        reasons.append(_stamp_reason("checked_at", status, age))

    live_status = snapshot.get("live_tf_status")
    if live_status == "lookup_failed":
        reasons.append("live TF lookup failed")
    elif live_status == "observer_down":
        reasons.append("live TF observer unavailable")
    elif live_status == "withheld":
        reasons.append(str(snapshot.get("live_tf_withheld_reason") or "live TF pose withheld"))
    else:
        live_stamp_status, live_age = stamp_status(
            snapshot.get("live_pose_checked_at"), max_age=LIVE_TF_MAX_AGE_S, now=now,
        )
        if live_stamp_status != "fresh":
            reasons.append(_stamp_reason("live_pose_checked_at", live_stamp_status, live_age))
        elif not _finite_pose(snapshot.get("pose")):
            reasons.append("live TF pose missing")

    if snapshot.get("map_matches_fms") is not True:
        reasons.append(str(snapshot.get("map_mismatch_detail") or "map identity not verified"))
    return reasons
