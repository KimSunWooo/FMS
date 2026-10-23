from __future__ import annotations

from datetime import datetime, timedelta, timezone


class Clock:
    """Injectable clock so recovery tests do not sleep."""

    def __init__(self) -> None:
        self._frozen: datetime | None = None

    def now(self) -> datetime:
        if self._frozen is not None:
            return self._frozen
        return datetime.now(timezone.utc)

    def freeze(self, moment: datetime | None = None) -> datetime:
        self._frozen = moment or datetime.now(timezone.utc)
        return self._frozen

    def advance(self, seconds: float) -> datetime:
        self._frozen = self.now() + timedelta(seconds=seconds)
        return self._frozen


def as_utc(value: datetime | None) -> datetime | None:
    if value is None:
        return None
    if value.tzinfo is None:
        return value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc)


def iso(value: datetime | None) -> str | None:
    moment = as_utc(value)
    if moment is None:
        return None
    return moment.isoformat()
