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

from events.schemas import GoProStatusEvent
from sensors.base import Sensor, SensorFrame

_log = logging.getLogger(__name__)

FrameSink = Callable[[SensorFrame], None]
# Status events are tagged with their sensor_id since the event payload
# (GoProStatusEvent) does not carry one — safety needs to know the source.
StatusSink = Callable[[str, GoProStatusEvent], None]

# Idle nap when a non-blocking read() returns None, so a stalled sensor does
# not spin a core.
_IDLE_NAP_S = 0.002


class _SensorWorker(threading.Thread):
    """Polls one sensor's read() and forwards frames to the manager."""

    def __init__(self, sensor: Sensor, on_frame: FrameSink):
        super().__init__(name=f"Sensor-{sensor.sensor_id}", daemon=True)
        self._sensor = sensor
        self._on_frame = on_frame
        # Not `_stop` — that name collides with threading.Thread internals.
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                frame = self._sensor.read()
            except Exception:
                _log.exception("sensor %s read() raised; continuing",
                               self._sensor.sensor_id)
                frame = None
            if frame is None:
                self._stop_event.wait(_IDLE_NAP_S)
                continue
            try:
                self._on_frame(frame)
            except Exception:
                _log.exception("frame sink raised for %s; suppressing",
                               self._sensor.sensor_id)

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        self.join(timeout=timeout_s)


class SensorManager:
    """Owns the active sensors and their worker threads."""

    def __init__(self, frame_sink: Optional[FrameSink] = None,
                 status_sink: Optional[StatusSink] = None):
        self._sensors: dict[str, Sensor] = {}
        self._workers: dict[str, _SensorWorker] = {}
        self._latest: dict[str, SensorFrame] = {}
        self._latest_status: dict[str, GoProStatusEvent] = {}
        self._lock = threading.Lock()
        self._sink = frame_sink
        self._status_sink = status_sink

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
        if hasattr(sensor, "set_status_sink"):
            sid = sensor.sensor_id
            sensor.set_status_sink(
                lambda ev, sid=sid: self._dispatch_status(sid, ev))
        if not sensor.open():
            _log.error("sensor %s failed to open", sensor.sensor_id)
            return False
        worker = _SensorWorker(sensor, self._dispatch)
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
