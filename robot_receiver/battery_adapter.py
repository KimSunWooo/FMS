"""Cache Pinky battery samples from ROS topics or the official pinkylib SDK.

Does not invent SOC from voltage in this module. Voltage-only BatteryState is
ignored. pinkylib.Battery.battery_percentage() is the manufacturer SDK path.
"""
from __future__ import annotations

import math
import os
import time
from datetime import datetime, timezone
from typing import Any, Callable

from robot_receiver.battery_report import DEFAULT_BATTERY_MAX_AGE_S, evaluate_battery_sample


class BatteryTelemetryCache:
    def __init__(
        self,
        *,
        max_age_s: float = DEFAULT_BATTERY_MAX_AGE_S,
        i2c_fallback: bool | None = None,
        i2c_period_s: float = 5.0,
        i2c_reader: Callable[[], tuple[float | None, float | None]] | None = None,
    ) -> None:
        self.max_age_s = max_age_s
        if i2c_fallback is None:
            i2c_fallback = os.environ.get("FMS_BATTERY_I2C_FALLBACK", "1") not in {"0", "false", "False"}
        self.i2c_fallback = i2c_fallback
        self.i2c_period_s = i2c_period_s
        self._i2c_reader = i2c_reader
        self._latest: dict[str, Any] | None = None
        self._last_i2c_monotonic = 0.0
        self._last_reason = "missing"

    @property
    def last_reason(self) -> str:
        return self._last_reason

    def update_percent_float32(self, value: float, *, source: str = "topic:battery/percent") -> None:
        # Bringup publishes 0..100.
        report = evaluate_battery_sample(
            percentage=float(value),
            observed_at=datetime.now(timezone.utc),
            source=source,
            max_age_s=self.max_age_s,
            age_basis="skip",
        )
        self._store(report)

    def update_battery_state(self, msg) -> None:
        """Accept sensor_msgs/BatteryState only when percentage is finite."""
        try:
            percentage = float(msg.percentage)
        except (TypeError, ValueError, AttributeError):
            self._last_reason = "voltage_only_or_missing"
            return
        if not math.isfinite(percentage):
            # Official pinky_sensor_adc publishes voltage with NaN percentage.
            self._last_reason = "voltage_only_or_missing"
            return
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        observed = datetime.now(timezone.utc)
        report = evaluate_battery_sample(
            percentage=percentage,
            observed_at=observed,
            source="topic:batt_state",
            max_age_s=self.max_age_s,
            age_basis="skip",
        )
        if report.get("battery_valid"):
            report["battery_ros_stamp"] = stamp
        self._store(report)

    def snapshot(self, *, now: datetime | None = None) -> dict[str, Any]:
        now = now or datetime.now(timezone.utc)
        if self._latest and self._latest.get("battery_valid"):
            refreshed = evaluate_battery_sample(
                percentage=self._latest["battery_percentage"],
                observed_at=datetime.fromisoformat(self._latest["battery_observed_at"]),
                source=self._latest["battery_source"],
                now=now,
                max_age_s=self.max_age_s,
                age_basis="wall",
            )
            if refreshed.get("battery_valid"):
                self._last_reason = "ok"
                return refreshed
            self._last_reason = str(refreshed.get("reason") or "stale")
        if self.i2c_fallback:
            self._maybe_read_i2c()
            if self._latest and self._latest.get("battery_valid"):
                refreshed = evaluate_battery_sample(
                    percentage=self._latest["battery_percentage"],
                    observed_at=datetime.fromisoformat(self._latest["battery_observed_at"]),
                    source=self._latest["battery_source"],
                    now=now,
                    max_age_s=self.max_age_s,
                    age_basis="wall",
                )
                if refreshed.get("battery_valid"):
                    self._last_reason = "ok"
                    return refreshed
                self._last_reason = str(refreshed.get("reason") or "stale")
        if not self._latest and self._last_reason in {"ok", ""}:
            self._last_reason = "missing"
        return {
            "battery_valid": False,
            "battery_percentage": None,
            "battery_source": "",
            "battery_observed_at": None,
            "reason": self._last_reason or "missing",
        }

    def _store(self, report: dict[str, Any]) -> None:
        if report.get("battery_valid"):
            self._latest = report
            self._last_reason = "ok"
        else:
            self._last_reason = str(report.get("reason") or "invalid_data")

    def _maybe_read_i2c(self) -> None:
        now_m = time.monotonic()
        if now_m - self._last_i2c_monotonic < self.i2c_period_s:
            return
        self._last_i2c_monotonic = now_m
        reader = self._i2c_reader or _default_pinkylib_reader
        try:
            percent_0_100, _voltage = reader()
        except Exception:  # noqa: BLE001
            self._last_reason = "i2c_unavailable"
            return
        if percent_0_100 is None:
            self._last_reason = "i2c_unavailable"
            return
        report = evaluate_battery_sample(
            percentage=float(percent_0_100),
            observed_at=datetime.now(timezone.utc),
            source="pinkylib:i2c",
            max_age_s=self.max_age_s,
            age_basis="skip",
        )
        self._store(report)


def _default_pinkylib_reader() -> tuple[float | None, float | None]:
    """Manufacturer SDK. Returns (percent_0_100, voltage_or_none)."""
    from pinkylib import Battery

    battery = Battery()
    try:
        voltage = battery.get_voltage()
        percent = battery.battery_percentage()
        return (None if percent is None else float(percent), None if voltage is None else float(voltage))
    finally:
        try:
            battery.close()
        except Exception:  # noqa: BLE001
            pass
