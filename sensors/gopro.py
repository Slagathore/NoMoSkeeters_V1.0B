"""GoProSensor — GoPro Hero 13 Black, the primary TARGETING camera.

The Hero 13 serves a UDP video-preview stream; we decode it with
cv2.VideoCapture on the CAP_FFMPEG backend (BOOTSTRAP.md §5.2). HTTP control
(start preview, ~2.5s keep-alive) is delegated to an injected GoProControl —
the concrete Open GoPro HTTP client belongs in sensors/gopro_interface.py and
must be written against the official API docs, NOT reconstructed from memory
(that is precisely the v1 fictional-protocol failure mode).

Reference: BOOTSTRAP.md §5.2.
"""
from __future__ import annotations

import logging
import os
import threading
import time
from typing import Callable, Optional, Protocol, runtime_checkable

import cv2

from events.schemas import GoProStatusEvent
from sensors.base import Sensor, SensorFrame, SensorRole

_log = logging.getLogger(__name__)

# Hero 13 preview keep-alive must be sent more often than the ~2.5s timeout.
_KEEPALIVE_INTERVAL_S = 2.0
# /gopro/camera/state poll cadence — 5-10s per HARDWARE_FINDINGS.md §2.4.
_STATUS_POLL_INTERVAL_S = 7.0
# Explicit IPv4 — udp://@:8554 resolves to IPv6 [::] on Windows ffmpeg builds
# and silently receives zero packets (HARDWARE_FINDINGS.md §3.2).
_DEFAULT_STREAM_URL = "udp://0.0.0.0:8554"
# Low-latency ffmpeg options; must be in the environment BEFORE VideoCapture
# is constructed (HARDWARE_FINDINGS.md §7.2). The Hero 13 preview is HEVC.
_DEFAULT_CAPTURE_OPTIONS = (
    "fflags;nobuffer|flags;low_delay|"
    "framedrop;1|rtbufsize;100M|overrun_nonfatal;1"
)

StatusCallback = Callable[[GoProStatusEvent], None]


@runtime_checkable
class GoProControl(Protocol):
    """HTTP control surface for the camera. Implemented by gopro_interface.py
    against the Open GoPro API; injected so GoProSensor stays decode-only and
    testable."""

    def start_preview(self) -> bool: ...
    def keep_alive(self) -> bool: ...
    def stop_preview(self) -> bool: ...
    def get_state(self) -> Optional[GoProStatusEvent]: ...


class GoProSensor(Sensor):
    """Decodes the Hero 13 UDP preview stream into SensorFrames."""

    def __init__(self,
                 stream_url: str = _DEFAULT_STREAM_URL,
                 role: SensorRole = SensorRole.TARGETING,
                 control: Optional[GoProControl] = None,
                 timestamp_uncertainty_ms: float = 20.0,
                 capture_factory: Optional[Callable[[], object]] = None,
                 capture_options: str = _DEFAULT_CAPTURE_OPTIONS,
                 on_status: Optional[StatusCallback] = None,
                 status_poll_interval_s: float = _STATUS_POLL_INTERVAL_S):
        self._stream_url = stream_url
        self._role = role
        self._control = control
        self._ts_uncertainty = timestamp_uncertainty_ms
        self._capture_factory = capture_factory or (
            lambda: cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG))
        self._capture_options = capture_options
        self._on_status = on_status
        self._status_interval_s = status_poll_interval_s
        self._cap: Optional[object] = None
        self._width = 0
        self._height = 0
        self._keepalive_stop = threading.Event()
        self._keepalive_thread: Optional[threading.Thread] = None
        self._status_stop = threading.Event()
        self._status_thread: Optional[threading.Thread] = None

    # ── Identity / capabilities ──────────────────────────────────────────

    @property
    def sensor_id(self) -> str:
        return "gopro"

    @property
    def role(self) -> SensorRole:
        return self._role

    @property
    def has_rgb(self) -> bool:
        return True

    @property
    def has_depth(self) -> bool:
        return False

    @property
    def has_ir(self) -> bool:
        return False

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    # ── Lifecycle ────────────────────────────────────────────────────────

    def open(self) -> bool:
        if self._control is not None and not self._control.start_preview():
            _log.error("gopro: start_preview() failed")
            return False
        # Low-latency ffmpeg options must be in the environment BEFORE the
        # VideoCapture is constructed (HARDWARE_FINDINGS.md §3.2, §7.2).
        if self._capture_options:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = self._capture_options
        cap = self._capture_factory()
        if cap is None or not cap.isOpened():
            _log.error("gopro: failed to open stream %s", self._stream_url)
            if cap is not None:
                cap.release()
            return False
        self._cap = cap
        if self._control is not None:
            self._start_keepalive()
            self._start_status_poll()
        _log.info("gopro: streaming from %s", self._stream_url)
        return True

    def close(self) -> None:
        self._stop_keepalive()
        self._stop_status_poll()
        if self._control is not None:
            try:
                self._control.stop_preview()
            except Exception:
                _log.exception("gopro: stop_preview() raised; closing anyway")
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def read(self) -> Optional[SensorFrame]:
        if self._cap is None:
            return None
        ok, image = self._cap.read()
        if not ok or image is None:
            return None
        self._height, self._width = image.shape[:2]
        frame = SensorFrame.now(self.sensor_id, self._role)
        frame.rgb = image
        frame.width = self._width
        frame.height = self._height
        frame.timestamp_uncertainty_ms = self._ts_uncertainty
        return frame

    # ── Keep-alive thread ────────────────────────────────────────────────

    def _start_keepalive(self) -> None:
        self._keepalive_stop.clear()
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop, name="GoProKeepAlive", daemon=True)
        self._keepalive_thread.start()

    def _stop_keepalive(self) -> None:
        self._keepalive_stop.set()
        if self._keepalive_thread is not None:
            self._keepalive_thread.join(timeout=2.0)
            self._keepalive_thread = None

    def _keepalive_loop(self) -> None:
        while not self._keepalive_stop.wait(_KEEPALIVE_INTERVAL_S):
            try:
                self._control.keep_alive()
            except Exception:
                # Never let the keep-alive thread die on a transient error.
                _log.exception("gopro: keep_alive() raised; will retry")

    # ── Status poll thread ───────────────────────────────────────────────

    def _start_status_poll(self) -> None:
        # Opt-in: only poll when a consumer wants the events and the control
        # surface can answer (a bare GoProControl fake may lack get_state).
        if self._on_status is None or not hasattr(self._control, "get_state"):
            return
        self._status_stop.clear()
        self._status_thread = threading.Thread(
            target=self._status_loop, name="GoProStatusPoll", daemon=True)
        self._status_thread.start()

    def _stop_status_poll(self) -> None:
        self._status_stop.set()
        if self._status_thread is not None:
            self._status_thread.join(timeout=2.0)
            self._status_thread = None

    def _status_loop(self) -> None:
        while not self._status_stop.wait(self._status_interval_s):
            try:
                event = self._control.get_state()
            except Exception:
                _log.exception("gopro: get_state() raised; will retry")
                event = None
            if event is None:
                # Camera unreachable — surface it so safety can react.
                event = GoProStatusEvent(
                    timestamp=time.monotonic(), reachable=False,
                    battery_percent=0, system_hot=False, system_busy=False,
                    preview_stream_active=False, thermal_throttle=False)
            try:
                self._on_status(event)
            except Exception:
                _log.exception("gopro: on_status callback raised; suppressing")
