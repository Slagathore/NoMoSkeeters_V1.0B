"""SensorManager — multiplexes active sensors, one worker thread each.

The §5.5 sketch uses Qt (QThread / Signal). The GUI layer (Steps 12-13) is
not built yet and PySide6 isn't a hard dependency of the core, so this uses
stdlib threading + a frame-sink callback instead. The contract is identical:
each sensor is polled on its own thread and every frame is delivered, tagged
with its sensor_id, to the registered sink.

Reference: BOOTSTRAP.md §5.5.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

from config import settings
from events.schemas import GoProStatusEvent, SensorHealthEvent
from sensors.base import Sensor, SensorFrame

_log = logging.getLogger(__name__)

FrameSink = Callable[[SensorFrame], None]
# Status events are tagged with their sensor_id since the event payload
# (GoProStatusEvent) does not carry one — safety needs to know the source.
StatusSink = Callable[[str, GoProStatusEvent], None]
HealthSink = Callable[[SensorHealthEvent], None]

# Idle nap when a non-blocking read() returns None, so a stalled sensor does
# not spin a core.
_IDLE_NAP_S = 0.002
_REOPEN_BACKOFF_CAP_S = 10.0


class _SensorWorker(threading.Thread):
    """Polls one sensor's read() and forwards frames to the manager.

    Liveness: if read() produces no frame for `dead_after_s` the sensor is
    presumed dead (unplugged USB, crashed decoder, wedged SDK) and the
    worker closes + reopens it with exponential backoff until it recovers
    or the worker is stopped. Health transitions are surfaced as
    SensorHealthEvents so a session can log/display them — before this,
    a dead sensor just meant silent frame starvation."""

    def __init__(self, sensor: Sensor, on_frame: FrameSink,
                 on_health: Optional[HealthSink] = None,
                 dead_after_s: Optional[float] = None):
        super().__init__(name=f"Sensor-{sensor.sensor_id}", daemon=True)
        self._sensor = sensor
        self._on_frame = on_frame
        self._on_health = on_health
        self._dead_after_s = (settings.SENSOR_DEAD_AFTER_S
                              if dead_after_s is None else dead_after_s)
        # Not `_stop` — that name collides with threading.Thread internals.
        self._stop_event = threading.Event()

    def run(self) -> None:
        last_frame_ts = time.monotonic()
        while not self._stop_event.is_set():
            try:
                frame = self._sensor.read()
            except Exception:
                _log.exception("sensor %s read() raised; continuing",
                               self._sensor.sensor_id)
                frame = None
            if frame is not None:
                last_frame_ts = time.monotonic()
                try:
                    self._on_frame(frame)
                except Exception:
                    _log.exception("frame sink raised for %s; suppressing",
                                   self._sensor.sensor_id)
                continue
            stalled_s = time.monotonic() - last_frame_ts
            if self._dead_after_s > 0.0 and stalled_s >= self._dead_after_s:
                self._recover(stalled_s)
                last_frame_ts = time.monotonic()   # grace after reopen
                continue
            self._stop_event.wait(_IDLE_NAP_S)

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        self.join(timeout=timeout_s)

    # ── Recovery ─────────────────────────────────────────────────────────

    def _emit_health(self, state: str, detail: str = "",
                     stalled_for_s: float = 0.0) -> None:
        if self._on_health is None:
            return
        try:
            self._on_health(SensorHealthEvent(
                timestamp=time.time(), sensor_id=self._sensor.sensor_id,
                state=state, detail=detail, stalled_for_s=stalled_for_s))
        except Exception:                                 # pragma: no cover
            _log.exception("health sink raised; suppressing")

    def _recover(self, stalled_s: float) -> None:
        """Close + reopen the sensor until it comes back or we're stopped."""
        _log.warning("sensor %s stalled %.1fs — reopening",
                     self._sensor.sensor_id, stalled_s)
        self._emit_health("stalled", stalled_for_s=stalled_s)
        backoff = max(0.1, settings.SENSOR_REOPEN_BACKOFF_S)
        while not self._stop_event.is_set():
            self._emit_health("reopening")
            try:
                self._sensor.close()
            except Exception:
                _log.exception("sensor %s close() raised during recovery",
                               self._sensor.sensor_id)
            opened = False
            try:
                opened = bool(self._sensor.open())
            except Exception:
                _log.exception("sensor %s open() raised during recovery",
                               self._sensor.sensor_id)
            if opened:
                _log.info("sensor %s recovered", self._sensor.sensor_id)
                self._emit_health("recovered")
                return
            self._emit_health("reopen_failed",
                              detail=f"retry in {backoff:.0f}s")
            self._stop_event.wait(backoff)
            backoff = min(backoff * 2.0, _REOPEN_BACKOFF_CAP_S)


class SensorManager:
    """Owns the active sensors and their worker threads."""

    def __init__(self, frame_sink: Optional[FrameSink] = None,
                 status_sink: Optional[StatusSink] = None,
                 health_sink: Optional[HealthSink] = None):
        self._sensors: dict[str, Sensor] = {}
        self._workers: dict[str, _SensorWorker] = {}
        self._latest: dict[str, SensorFrame] = {}
        self._latest_status: dict[str, GoProStatusEvent] = {}
        self._latest_health: dict[str, SensorHealthEvent] = {}
        self._lock = threading.Lock()
        self._sink = frame_sink
        self._status_sink = status_sink
        self._health_sink = health_sink

    @property
    def sensor_ids(self) -> list[str]:
        return list(self._sensors)

    def add_sensor(self, sensor: Sensor) -> bool:
        """Open a sensor and start its worker. Returns False if it is already
        registered or fails to open."""
        if sensor.sensor_id in self._sensors:
            _log.warning("sensor %s already registered", sensor.sensor_id)
            return False
        # Route any sensor status (e.g. GoPro /camera/state) onto the bus.
        # Must happen before open() — open() starts the status-poll thread.
        sink = getattr(sensor, "set_status_sink", None)
        if sink is not None:
            sid = sensor.sensor_id
            sink(lambda ev, sid=sid: self._dispatch_status(sid, ev))
        if not sensor.open():
            _log.error("sensor %s failed to open", sensor.sensor_id)
            return False
        worker = _SensorWorker(sensor, self._dispatch,
                               on_health=self._dispatch_health)
        self._sensors[sensor.sensor_id] = sensor
        self._workers[sensor.sensor_id] = worker
        worker.start()
        _log.info("sensor %s added (role=%s)", sensor.sensor_id, sensor.role)
        return True

    def remove_sensor(self, sensor_id: str) -> None:
        """Stop a sensor's worker and close it. Idempotent."""
        worker = self._workers.pop(sensor_id, None)
        if worker is not None:
            worker.stop()
        sensor = self._sensors.pop(sensor_id, None)
        if sensor is not None:
            sensor.close()
        with self._lock:
            self._latest.pop(sensor_id, None)
            self._latest_status.pop(sensor_id, None)

    def latest_frame(self, sensor_id: str) -> Optional[SensorFrame]:
        """Most recent frame seen from a sensor, or None."""
        with self._lock:
            return self._latest.get(sensor_id)

    def latest_status(self, sensor_id: str) -> Optional[GoProStatusEvent]:
        """Most recent status event from a sensor, or None."""
        with self._lock:
            return self._latest_status.get(sensor_id)

    def latest_health(self, sensor_id: str) -> Optional[SensorHealthEvent]:
        """Most recent liveness transition for a sensor, or None (healthy
        sensors that never stalled have no health event)."""
        with self._lock:
            return self._latest_health.get(sensor_id)

    def stop_all(self) -> None:
        for sensor_id in list(self._sensors):
            self.remove_sensor(sensor_id)

    # ── Internal ─────────────────────────────────────────────────────────

    def _dispatch(self, frame: SensorFrame) -> None:
        with self._lock:
            self._latest[frame.sensor_id] = frame
        if self._sink is not None:
            self._sink(frame)

    def _dispatch_status(self, sensor_id: str,
                         event: GoProStatusEvent) -> None:
        with self._lock:
            self._latest_status[sensor_id] = event
        if self._status_sink is not None:
            self._status_sink(sensor_id, event)

    def _dispatch_health(self, event: SensorHealthEvent) -> None:
        with self._lock:
            self._latest_health[event.sensor_id] = event
        if self._health_sink is not None:
            self._health_sink(event)
