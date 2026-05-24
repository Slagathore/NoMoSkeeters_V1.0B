"""Person detection for the safety gate.

The default backend is OpenCV's pre-trained HOG full-body detector — it
ships with cv2 (no download, no model weights), runs CPU-only at usable
rates for safety polling (~10 Hz at 640×480), and gives bounding boxes
plus a hit score. Accuracy is meh for a primary detector, but safety
gates are biased toward false positives (an over-cautious inhibit is
fine; missing a person is not), so HOG is acceptable for room-scale
defensive use.

A `PersonDetector` ABC is exposed so a downstream wrapper can plug in a
DNN (YOLO via cv2.dnn, MediaPipe Pose, etc.) without touching the rest
of the safety pipeline.

Reference: BOOTSTRAP.md §12 (safety), §5 (SAFETY role sensors).
"""
from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

_log = logging.getLogger(__name__)


@dataclass
class PersonDetection:
    """One detected person in image coordinates."""
    x_px: int                # bbox upper-left x
    y_px: int                # bbox upper-left y
    w_px: int
    h_px: int
    confidence: float        # detector hit weight, normalized 0..1 when possible
    sensor_id: str = ""

    @property
    def center_px(self) -> tuple[int, int]:
        return (self.x_px + self.w_px // 2, self.y_px + self.h_px // 2)

    @property
    def area_px2(self) -> int:
        return self.w_px * self.h_px

    def center_norm(self, frame_w: int, frame_h: int) -> tuple[float, float]:
        cx, cy = self.center_px
        return (cx / max(1, frame_w - 1), cy / max(1, frame_h - 1))


class PersonDetector(ABC):
    """ABC for person-detection backends."""

    @abstractmethod
    def detect(self, frame_bgr: np.ndarray) -> list[PersonDetection]:
        """Return zero or more person detections in image coordinates."""


class HOGPersonDetector(PersonDetector):
    """OpenCV HOG + SVM full-body people detector — default safety backend.

    Tunables map to `cv2.HOGDescriptor.detectMultiScale`:
      - `win_stride`: stride of the sliding window. Larger = faster, less
        recall. Default (8, 8) is HOG's standard.
      - `padding`: extra pixels around each window. (16, 16) helps catch
        people partially clipped at the frame edge.
      - `scale`: image-pyramid step. 1.05 is the published default; 1.1 is
        ~2× faster with marginally worse recall.
      - `hit_threshold`: minimum SVM margin to keep a hit. Negative values
        are lenient (more detections), positive strict.
    """

    def __init__(self,
                 win_stride: tuple[int, int] = (8, 8),
                 padding: tuple[int, int] = (16, 16),
                 scale: float = 1.05,
                 hit_threshold: float = 0.0):
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self._win_stride = win_stride
        self._padding = padding
        self._scale = scale
        self._hit_threshold = hit_threshold

    def detect(self, frame_bgr: np.ndarray) -> list[PersonDetection]:
        if frame_bgr is None or frame_bgr.size == 0:
            return []
        if frame_bgr.ndim == 2:
            gray = frame_bgr
        else:
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        try:
            rects, weights = self._hog.detectMultiScale(
                gray,
                winStride=self._win_stride,
                padding=self._padding,
                scale=self._scale,
                hitThreshold=self._hit_threshold,
            )
        except cv2.error:
            _log.exception("HOG detectMultiScale failed")
            return []

        if len(rects) == 0:
            return []
        weights = np.asarray(weights).reshape(-1)
        # Clamp the SVM margin to a [0, 1] confidence-like value so downstream
        # gating uses a consistent scale across detectors. HOG margins above
        # ~1.5 are very strong; below 0 we already filtered via hit_threshold.
        confs = np.clip(weights / 1.5, 0.0, 1.0)
        out: list[PersonDetection] = []
        for (x, y, w, h), c in zip(rects, confs):
            out.append(PersonDetection(int(x), int(y), int(w), int(h),
                                       float(c)))
        return out


class FixedRegionPersonDetector(PersonDetector):
    """Test-only stub: always returns the configured rectangles.

    Useful in unit tests for SafetyModerator wiring — production code should
    never wire this in. Kept inside `safety.` so it's discoverable next to
    the real detectors but importable as
    `from safety.person_detector import FixedRegionPersonDetector`.
    """

    def __init__(self, detections: Optional[list[PersonDetection]] = None):
        self._detections = list(detections or [])

    def set(self, detections: list[PersonDetection]) -> None:
        self._detections = list(detections)

    def detect(self, frame_bgr: np.ndarray) -> list[PersonDetection]:
        return list(self._detections)
