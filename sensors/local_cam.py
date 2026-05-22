"""LocalCamSensor — plain cv2.VideoCapture webcam, the development fallback.

Used when the GoPro isn't connected and for offline testing. The capture
backend is injectable so tests can run without a physical camera.

Reference: BOOTSTRAP.md §5.4.
"""
from __future__ import annotations

import logging
from typing import Any, Callable, Optional

import cv2

from config import settings
from sensors.base import Sensor, SensorFrame, SensorRole

_log = logging.getLogger(__name__)


class LocalCamSensor(Sensor):
    """A webcam at a cv2.VideoCapture index."""

    def __init__(self,
                 index: int = 0,
                 role: SensorRole = SensorRole.FALLBACK,
                 timestamp_uncertainty_ms: float = 5.0,
                 capture_factory: Optional[Callable[[], Any]] = None):
        self._index = index
        self._role = role
        self._ts_uncertainty = timestamp_uncertainty_ms
        self._capture_factory = capture_factory or (
            lambda: cv2.VideoCapture(index))
        self._cap: Optional[Any] = None
        self._width = 0
        self._height = 0

    # ── Identity / capabilities ──────────────────────────────────────────

    @property
    def sensor_id(self) -> str:
        return "local_cam"

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
        cap = self._capture_factory()
        if cap is None or not cap.isOpened():
            _log.error("local_cam: failed to open capture index %d", self._index)
            if cap is not None:
                cap.release()
            return False
        self._cap = cap
        _log.info("local_cam: opened capture index %d", self._index)
        return True

    def close(self) -> None:
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
