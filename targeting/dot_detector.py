"""LaserDotDetector — finds the laser dot in a camera frame.

Calibration fires a known galvo coord and needs the pixel the camera saw it
at. The dot is the brightest thing in a dim room, so this is a bright-channel
threshold + largest-blob centroid (the §8.3 "auto" detection mode). Confidence
drops when more than one bright blob is present — the correspondence is then
ambiguous and the controller should fall back to manual.

Reference: BOOTSTRAP.md §8.3.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from config import settings

_log = logging.getLogger(__name__)


@dataclass(frozen=True)
class DotObservation:
    """Where the laser dot was seen, in one camera frame."""
    x_norm: float
    y_norm: float
    x_px: int
    y_px: int
    area_px: float
    peak_brightness: int
    confidence: float


class LaserDotDetector:
    """Bright-blob laser-dot finder for calibration capture."""

    def __init__(self, threshold: Optional[int] = None, min_area_px: int = 2):
        self._threshold = (threshold if threshold is not None
                           else settings.CALIBRATION_DOT_THRESHOLD)
        self._min_area = min_area_px

    def detect(self, frame_bgr: np.ndarray) -> Optional[DotObservation]:
        """Return the dot observation for one BGR frame, or None if no dot."""
        if frame_bgr is None or frame_bgr.size == 0:
            return None
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, self._threshold, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        blobs = [c for c in contours if cv2.contourArea(c) >= self._min_area]
        if not blobs:
            return None

        blobs.sort(key=cv2.contourArea, reverse=True)
        dot = blobs[0]
        m = cv2.moments(dot)
        if m["m00"] == 0.0:
            x, y, w, h = cv2.boundingRect(dot)
            cx, cy = x + w / 2.0, y + h / 2.0
        else:
            cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]

        frame_h, frame_w = gray.shape[:2]
        # Ambiguity penalty — more than one bright blob means we may have
        # locked onto a reflection or stray light, not the dot.
        confidence = 1.0 / len(blobs)
        return DotObservation(
            x_norm=cx / max(1, frame_w - 1),
            y_norm=cy / max(1, frame_h - 1),
            x_px=int(round(cx)),
            y_px=int(round(cy)),
            area_px=float(cv2.contourArea(dot)),
            peak_brightness=int(gray.max()),
            confidence=confidence,
        )
