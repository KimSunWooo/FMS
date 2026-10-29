"""Validate RobotStatus battery fields. Never invents SOC from raw voltage.

Accepted inputs (when present on the robot):
- std_msgs/Float32 on battery/percent (Pinky bringup: 0..100)
- sensor_msgs/BatteryState.percentage when finite (ROS convention 0..1;
  values in (1, 100] are treated as percent and normalized)
- Optional pinkylib I2C SDK percentage (already SOC; source tagged)

Voltage-only BatteryState samples are rejected (percentage stays invalid).
"""
from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Any

DEFAULT_BATTERY_MAX_AGE_S = 30.0


def normalize_soc(raw: float, *, assume_percent_if_gt_one: bool = True) -> float | None:
    if not math.isfinite(raw):
        return None
    value = float(raw)
    if assume_percent_if_gt_one and value > 1.0:
        value = value / 100.0
    if value < 0.0 or value > 1.0:
        return None
    return value


def evaluate_battery_sample(
    *,
    percentage: float | None,
    observed_at: datetime | None,
    source: str,
    now: datetime | None = None,
    max_age_s: float = DEFAULT_BATTERY_MAX_AGE_S,
    age_basis: str = "wall",
) -> dict[str, Any]:
    now = now or datetime.now(timezone.utc)
    if observed_at is not None and observed_at.tzinfo is None:
        observed_at = observed_at.replace(tzinfo=timezone.utc)
    if percentage is None:
        return _invalid("missing")
    soc = normalize_soc(percentage)
    if soc is None:
        return _invalid("invalid_data")
    if not source:
        return _invalid("invalid_data")
    if observed_at is None:
        return _invalid("missing_stamp")
    age = None
    if age_basis == "wall":
        age = (now - observed_at).total_seconds()
        if age < -1.0 or age > max_age_s:
            return _invalid("stale", age_s=age)
    elif age_basis == "ros":
        observed_at = now
    elif age_basis != "skip":
        return _invalid("invalid_data")
    return {
        "battery_valid": True,
        "battery_percentage": float(soc),
        "battery_source": str(source),
        "battery_observed_at": observed_at.isoformat(),
        "reason": "ok",
        "age_s": age,
    }


def status_battery_payload(report: dict[str, Any] | None) -> dict[str, Any]:
    if not report or not report.get("battery_valid"):
        return {
            "battery_valid": False,
            "battery_percentage": None,
            "battery_source": "",
            "battery_observed_at": None,
            "battery_reason": None if report is None else report.get("reason"),
        }
    return {
        "battery_valid": True,
        "battery_percentage": report["battery_percentage"],
        "battery_source": report["battery_source"],
        "battery_observed_at": report["battery_observed_at"],
        "battery_reason": "ok",
    }


def _invalid(reason: str, **extra: Any) -> dict[str, Any]:
    body = {
        "battery_valid": False,
        "battery_percentage": None,
        "battery_source": "",
        "battery_observed_at": None,
        "reason": reason,
    }
    body.update(extra)
    return body
