"""OV9281Sensor — the OmniVision OV9281 global-shutter USB camera.

The V1 ground-truth targeting sensor (PHONE_SENSOR_BOOTSTRAP §10): a monochrome
global-shutter UVC camera that streams MJPEG over USB 2.0. Global shutter means
no rolling-shutter smear on a fast-flying mosquito; it is IR-sensitive, so it
pairs with the Kinect's 850nm flood for darkness operation.

It is a plain UVC device, so the transport is `cv2.VideoCapture` — but unlike
`LocalCamSensor` it must be *configured*: select the MJPEG fourcc FIRST, then
the resolution/fps (the high-speed modes only exist under MJPEG), then lock
exposure (a global shutter is wasted if auto-exposure drifts the shutter long
enough to blur). Those cv2 property semantics are backend-dependent, so the
capture backend is selectable and every `set()` is best-effort.

Reference: settings.OV9281_*, BOOTSTRAP_AMENDMENTS §5 (sensor roles).
"""
from __future__ import annotations

import logging
import sys
from typing import Any, Callable, Optional

import cv2

from config import settings
from sensors.base import Sensor, SensorFrame, SensorRole

_log = logging.getLogger(__name__)

_BACKENDS = {
    "dshow": cv2.CAP_DSHOW,
    "msmf": cv2.CAP_MSMF,
    "v4l2": cv2.CAP_V4L2,
    "any": cv2.CAP_ANY,
}


def _resolve_backend(name: str) -> int:
    """Map a backend name to a cv2 flag. 'auto' → DSHOW on Windows (best UVC
    MJPEG/high-fps support), CAP_ANY elsewhere."""
    name = (name or "auto").lower()
    if name == "auto":
        return cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
    return _BACKENDS.get(name, cv2.CAP_ANY)


class OV9281Sensor(Sensor):
    """Global-shutter UVC camera, MJPEG high-fps, manual exposure."""

    def __init__(self,
                 device_index: int = settings.OV9281_DEVICE_INDEX,
                 *,
                 backend: str = settings.OV9281_BACKEND,
                 fourcc: str = settings.OV9281_FOURCC,
                 width: int = settings.OV9281_WIDTH,
                 height: int = settings.OV9281_HEIGHT,
                 fps: int = settings.OV9281_FPS,
                 auto_exposure: bool = settings.OV9281_AUTO_EXPOSURE,
                 exposure: float = settings.OV9281_EXPOSURE,
                 gain: float = settings.OV9281_GAIN,
                 role: SensorRole = SensorRole.TARGETING,
                 timestamp_uncertainty_ms: float =
                     settings.OV9281_TIMESTAMP_UNCERTAINTY_MS,
                 capture_factory: Optional[Callable[[], Any]] = None):
        self._index = device_index
        self._backend = backend
        self._fourcc = fourcc
        self._req_width = width
        self._req_height = height
        self._req_fps = fps
        self._auto_exposure = auto_exposure
        self._exposure = exposure
        self._gain = gain
        self._role = role
        self._ts_uncertainty = timestamp_uncertainty_ms
        self._capture_factory = capture_factory or self._default_capture
        self._cap: Optional[Any] = None
        self._width = 0
        self._height = 0

    def _default_capture(self) -> Any:
        return cv2.VideoCapture(self._index, _resolve_backend(self._backend))

    # ── Identity / capabilities ──────────────────────────────────────────

    @property
    def sensor_id(self) -> str:
        return "ov9281"

    @property
    def role(self) -> SensorRole:
        return self._role

    @property
    def has_rgb(self) -> bool:
        # Monochrome, but the frame rides in the rgb slot as 3-channel BGR
        # (cv2 replicates the gray plane). Downstream detection works on either.
        return True

    @property
    def has_depth(self) -> bool:
        return False

    @property
    def has_ir(self) -> bool:
        # IR-sensitive in practice, but it is not a calibrated IR stream — the
        # image is exposed as visible-band rgb. Keep False so fusion treats it
        # like the GoPro, not like the Kinect IR plane.
        return False

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    # ── Lifecycle ────────────────────────────────────────────────────────

    def open(self) -> bool:
        cap = self._capture_factory()
        if cap is None or not cap.isOpened():
            _log.error("ov9281: failed to open capture index %d (backend=%s)",
                       self._index, self._backend)
            if cap is not None:
                cap.release()
            return False
        self._configure(cap)
        self._cap = cap
        # Report what the camera actually accepted — drivers silently clamp.
        got_w = int(self._safe_get(cap, cv2.CAP_PROP_FRAME_WIDTH, self._req_width))
        got_h = int(self._safe_get(cap, cv2.CAP_PROP_FRAME_HEIGHT, self._req_height))
        got_fps = self._safe_get(cap, cv2.CAP_PROP_FPS, self._req_fps)
        _log.info("ov9281: opened index %d @ %dx%d %.0ffps (requested %dx%d %d)",
                  self._index, got_w, got_h, got_fps,
                  self._req_width, self._req_height, self._req_fps)
        return True

    def _configure(self, cap: Any) -> None:
        """Apply MJPEG fourcc, then resolution/fps, then MJPEG fourcc AGAIN,
        then exposure. The high-speed modes only exist under MJPEG and the
        driver re-evaluates valid (w,h,fps) when the codec changes — but on
        DSHOW the first FOURCC set is silently dropped when W/H/FPS are set
        after, so the camera ends up streaming raw YUY2 at ~30fps. Setting
        FOURCC a second time *after* W/H/FPS forces it to stick (verified
        2026-05-23: yields 188 fps actual at 640x480@210 vs 31 fps without)."""
        fourcc = 0
        try:
            fourcc = cv2.VideoWriter_fourcc(*self._fourcc)  # type: ignore[attr-defined]
            self._safe_set(cap, cv2.CAP_PROP_FOURCC, fourcc)
        except Exception as exc:
            _log.warning("ov9281: fourcc %s not applied: %s", self._fourcc, exc)
        self._safe_set(cap, cv2.CAP_PROP_FRAME_WIDTH, self._req_width)
        self._safe_set(cap, cv2.CAP_PROP_FRAME_HEIGHT, self._req_height)
        self._safe_set(cap, cv2.CAP_PROP_FPS, self._req_fps)
        if fourcc:
            self._safe_set(cap, cv2.CAP_PROP_FOURCC, fourcc)
        # A depth-1 grab keeps read() returning the freshest frame (the driver
        # buffer otherwise queues stale frames at high fps).
        self._safe_set(cap, cv2.CAP_PROP_BUFFERSIZE, 1)

        # Exposure: 0.25 = manual, 0.75 = auto is the common DSHOW convention
        # (v4l2 uses 1 / 3); both are attempted best-effort.
        self._safe_set(cap, cv2.CAP_PROP_AUTO_EXPOSURE,
                       0.75 if self._auto_exposure else 0.25)
        if not self._auto_exposure:
            self._safe_set(cap, cv2.CAP_PROP_EXPOSURE, self._exposure)
            self._safe_set(cap, cv2.CAP_PROP_GAIN, self._gain)

    @staticmethod
    def _safe_set(cap: Any, prop: int, value: float) -> None:
        try:
            cap.set(prop, value)
        except Exception as exc:                       # pragma: no cover
            _log.debug("ov9281: set(%d, %s) failed: %s", prop, value, exc)

    @staticmethod
    def _safe_get(cap: Any, prop: int, default: float) -> float:
        try:
            v = cap.get(prop)
            return v if v else default
        except Exception:                              # pragma: no cover
            return default

    def close(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                _log.exception("ov9281: release() raised; ignoring")
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
