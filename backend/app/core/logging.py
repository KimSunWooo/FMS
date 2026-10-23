from __future__ import annotations

import json
import logging
from datetime import datetime, timezone
from pathlib import Path

_CONFIGURED = False


class _JsonFormatter(logging.Formatter):
    _FIELDS = (
        "task_id",
        "robot_id",
        "execution_id",
        "goal_id",
        "owner_epoch",
        "event",
    )

    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, object] = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }
        for key in self._FIELDS:
            value = getattr(record, key, None)
            if value is not None:
                payload[key] = value
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload, ensure_ascii=False)


def configure_logging(log_dir: Path) -> None:
    global _CONFIGURED
    if _CONFIGURED:
        return
    log_dir.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("care_move_fms")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    formatter = _JsonFormatter()
    stream = logging.StreamHandler()
    stream.setFormatter(formatter)
    file_handler = logging.FileHandler(log_dir / "fms.log", encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(stream)
    logger.addHandler(file_handler)
    _CONFIGURED = True


def get_logger() -> logging.Logger:
    return logging.getLogger("care_move_fms")
