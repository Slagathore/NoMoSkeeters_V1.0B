"""SensorManager worker liveness: stalled sensor → close/reopen → recover.

A sensor whose read() goes quiet must not silently starve the pipeline —
the worker reopens it and surfaces SensorHealthEvents along the way.
"""
from __future__ import annotations

import time

import pytest

from config import settings
from sensors.base import SensorFrame, SensorRole
from sensors.sensor_manager import SensorManager


class FlakySensor:
    """Serves a few frames, dies, and revives on the second open()."""

    sensor_id = "flaky"
    role = SensorRole.TARGETING.value

    def __init__(self, frames_per_life: int = 3,
                 fail_opens_before_revive: int = 1) -> None:
        self.open_calls = 0
        self.close_calls = 0
        self._frames_left = 0
        self._frames_per_life = frames_per_life
        self._fail_opens = fail_opens_before_revive

    def open(self) -> bool:
        self.open_calls += 1
        if self.open_calls > 1 and self._fail_opens > 0:
            self._fail_opens -= 1
            return False
        self._frames_left = self._frames_per_life
        return True

    def read(self):
        if self._frames_left <= 0:
            return None
        self._frames_left -= 1
        f = SensorFrame(timestamp=time.time(), sensor_id=self.sensor_id,
                        sensor_role=self.role)
        return f

    def close(self) -> None:
        self.close_calls += 1
        self._frames_left = 0


def _wait_for(predicate, timeout_s: float = 5.0) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_stalled_sensor_is_reopened_and_recovers(
        monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(settings, "SENSOR_DEAD_AFTER_S", 0.05)
    monkeypatch.setattr(settings, "SENSOR_REOPEN_BACKOFF_S", 0.02)

    frames: list[SensorFrame] = []
    health: list = []
    sensor = FlakySensor()
    mgr = SensorManager(frame_sink=frames.append, health_sink=health.append)
    try:
        assert mgr.add_sensor(sensor)

        # First life: 3 frames, then the sensor goes quiet → the worker
        # must declare it stalled, survive one failed reopen, and recover.
        assert _wait_for(lambda: sensor.open_calls >= 3), \
            f"no reopen happened (open_calls={sensor.open_calls})"
        assert _wait_for(lambda: len(frames) >= 6), \
            "frames did not resume after recovery"

        states = [h.state for h in health]
        assert "stalled" in states
        assert "reopen_failed" in states     # the deliberate failed open
        assert "recovered" in states
        assert states.index("stalled") < states.index("recovered")
        assert mgr.latest_health("flaky") is not None
        assert sensor.close_calls >= 1
    finally:
        mgr.stop_all()


def test_reconnect_disabled_by_zero_setting(
        monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(settings, "SENSOR_DEAD_AFTER_S", 0.0)

    health: list = []
    sensor = FlakySensor()
    mgr = SensorManager(health_sink=health.append)
    try:
        assert mgr.add_sensor(sensor)
        time.sleep(0.3)                      # would be several stall windows
        assert sensor.open_calls == 1        # never reopened
        assert health == []
    finally:
        mgr.stop_all()
