"""KillCam — GoPro slow-mo witness of every laser shot.

The GoPro can't be a UVC targeting webcam *and* record to its SD card at the
same time, so the kill-cam uses it as a dedicated witness: targeting runs on the
OV9281 / phone / Kinect, and the GoPro captures a slow-mo clip of each
engagement.

Wiring: `LaserManager` already emits engagement events through its `event_sink`
(`op:"shoot"`, `op:"cone_done"` with `fired`, …). `KillCam.on_laser_event` is a
drop-in for that sink::

    killcam = KillCam.from_settings()
    killcam.start()
    lm = LaserManager(transport, event_sink=killcam.on_laser_event)

Two modes:
- **per_shot** (default): on a fire, load the slo-mo preset, roll the shutter
  for `record_seconds`, stop. Each kill is its own clip. Recording runs on a
  worker thread so it never stalls the laser stream.
- **continuous_hilight**: the GoPro records continuously in slo-mo and the
  kill-cam drops a HiLight tag at each fire so you can jump to kills in one long
  clip.

Triggers are debounced so a burst / cone reacquire doesn't thrash the camera.

Reference: settings.KILLCAM_*, sensors/gopro_interface.py (load_preset /
start_recording / stop_recording / hilight), laser/laser_manager.py.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Optional

from config import settings

_log = logging.getLogger(__name__)

MODE_PER_SHOT = "per_shot"
MODE_CONTINUOUS = "continuous_hilight"


class KillCam:
    """Drives a GoPro (duck-typed control) to witness laser shots in slo-mo."""

    def __init__(self,
                 control: Any,
                 *,
                 mode: str = settings.KILLCAM_MODE,
                 slomo_preset_id: int = settings.KILLCAM_SLOMO_PRESET_ID,
                 record_seconds: float = settings.KILLCAM_RECORD_SECONDS,
                 debounce_seconds: float = settings.KILLCAM_DEBOUNCE_SECONDS,
                 trigger_on: str = settings.KILLCAM_TRIGGER_ON,
                 record_async: bool = True,
                 clock: Callable[[], float] = time.monotonic,
                 sleeper: Callable[[float], None] = time.sleep):
        self._control = control
        self._mode = mode
        self._preset_id = slomo_preset_id
        self._record_s = record_seconds
        self._debounce_s = debounce_seconds
        self._trigger_on = trigger_on
        self._record_async = record_async
        self._clock = clock
        self._sleep = sleeper

        self._lock = threading.Lock()
        self._last_fire = -1e9
        self._continuous_started = False
        self.shots_witnessed = 0

    @classmethod
    def from_settings(cls, control: Optional[Any] = None, **over) -> "KillCam":
        if control is None:
            from sensors.gopro_interface import GoProInterface
            control = GoProInterface(ip=settings.KILLCAM_GOPRO_IP)
        return cls(control, **over)

    # ── Lifecycle ────────────────────────────────────────────────────────

    def start(self) -> None:
        """In continuous mode, load the slo-mo preset and begin rolling. In
        per_shot mode this is a no-op (recording starts on the first fire)."""
        if self._mode == MODE_CONTINUOUS:
            try:
                self._control.load_preset(self._preset_id)
                self._continuous_started = bool(self._control.start_recording())
                _log.info("killcam: continuous slo-mo recording started=%s",
                          self._continuous_started)
            except Exception:
                _log.exception("killcam: failed to start continuous recording")

    def stop(self) -> None:
        if self._mode == MODE_CONTINUOUS and self._continuous_started:
            try:
                self._control.stop_recording()
            except Exception:
                _log.exception("killcam: failed to stop continuous recording")
            self._continuous_started = False

    # ── The LaserManager event_sink ──────────────────────────────────────

    def on_laser_event(self, payload: dict) -> None:
        """Drop-in for `LaserManager(event_sink=...)`. Fires the witness on a
        shot (debounced); ignores non-fire ops."""
        if not self._is_fire(payload):
            return
        now = self._clock()
        with self._lock:
            if now - self._last_fire < self._debounce_s:
                return
            self._last_fire = now
        self.shots_witnessed += 1
        self._trigger()

    def _is_fire(self, payload: dict) -> bool:
        op = payload.get("op")
        if self._trigger_on == "any_shot":
            return op in ("shoot", "cone_done")
        # "fired": only count real kill pulses.
        if op == "shoot":
            return True
        if op == "cone_done":
            return bool(payload.get("fired"))
        return False

    # ── Recording ────────────────────────────────────────────────────────

    def _trigger(self) -> None:
        if self._mode == MODE_CONTINUOUS:
            self._safe(lambda: self._control.hilight(), "hilight")
            return
        if self._record_async:
            threading.Thread(target=self._record_clip, name="killcam-record",
                             daemon=True).start()
        else:
            self._record_clip()

    def _record_clip(self) -> None:
        """per_shot: preset → shutter on → wait → shutter off."""
        self._safe(lambda: self._control.load_preset(self._preset_id), "load_preset")
        if not self._safe(lambda: self._control.start_recording(), "start_recording"):
            return
        try:
            self._sleep(self._record_s)
        finally:
            self._safe(lambda: self._control.stop_recording(), "stop_recording")

    @staticmethod
    def _safe(fn: Callable[[], Any], what: str) -> bool:
        try:
            return bool(fn())
        except Exception:
            _log.exception("killcam: %s raised; continuing", what)
            return False


def maybe_build_killcam(control: Optional[Any] = None) -> Optional[KillCam]:
    """Build a KillCam if settings.KILLCAM_ENABLED, else None — so the
    orchestrator can wire it unconditionally."""
    if not settings.KILLCAM_ENABLED:
        return None
    return KillCam.from_settings(control=control)
