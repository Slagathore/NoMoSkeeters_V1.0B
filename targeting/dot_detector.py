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

    def __init__(self, threshold: Optional[int] = None, min_area_px: int = 2,
                 diff_threshold: Optional[int] = None):
        self._threshold = (threshold if threshold is not None
                           else settings.CALIBRATION_DOT_THRESHOLD)
        self._diff_threshold = (diff_threshold if diff_threshold is not None
                                else settings.CALIBRATION_DOT_DIFF_THRESHOLD)
        self._min_area = min_area_px

    def detect(self, frame_bgr: np.ndarray) -> Optional[DotObservation]:
        """Bright-blob detection on a raw frame: the dot is assumed the
        brightest thing present. Unreliable in a room with windows/screens —
        prefer detect_diff() for calibration."""
        if frame_bgr is None or frame_bgr.size == 0:
            return None
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        return self._detect_gray(gray, self._threshold)

    def detect_diff(self, lit_bgr: np.ndarray,
                    background_bgr: np.ndarray) -> Optional[DotObservation]:
        """Temporal-difference detection: the laser dot is whatever changed
        between a laser-ON and a laser-OFF frame. Everything static — windows,
        screens, lit walls — cancels out. This is the calibration path.

        The dot is found as the brightest PEAK in the diff, not the largest
        blob: at low power the laser also throws a wide, dim scatter/glow,
        which is larger than the dot core but dimmer — thresholding relative
        to the peak drops it and keeps only the compact, bright core.
        """
        if (lit_bgr is None or background_bgr is None
                or lit_bgr.shape != background_bgr.shape):
            return None
        diff = cv2.absdiff(lit_bgr, background_bgr)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 0)
        _, peak, _, peak_loc = cv2.minMaxLoc(blur)
        if peak < self._diff_threshold:
            return None

        # Isolate the bright core — pixels within 60% of the peak. The
        # diffuse beam glow is dimmer than that and drops out.
        core = max(float(self._diff_threshold), peak * 0.6)
        _, mask = cv2.threshold(blur, core, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        blobs = [c for c in contours if cv2.contourArea(c) >= self._min_area]
        if not blobs:
            return None

        # The dot is the blob containing the brightest pixel.
        px, py = float(peak_loc[0]), float(peak_loc[1])
        dot = next((c for c in blobs
                    if cv2.pointPolygonTest(c, (px, py), False) >= 0),
                   max(blobs, key=cv2.contourArea))
        m = cv2.moments(dot)
        if m["m00"] == 0.0:
            x, y, w, h = cv2.boundingRect(dot)
            cx, cy = x + w / 2.0, y + h / 2.0
        else:
            cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]

        frame_h, frame_w = gray.shape[:2]
        confidence = 1.0 / len(blobs)        # >1 core blob ⇒ ambiguous
        return DotObservation(
            x_norm=cx / max(1, frame_w - 1),
            y_norm=cy / max(1, frame_h - 1),
            x_px=int(round(cx)),
            y_px=int(round(cy)),
            area_px=float(cv2.contourArea(dot)),
            peak_brightness=int(peak),
            confidence=confidence,
        )

    def _detect_gray(self, gray: np.ndarray,
                     threshold: int) -> Optional[DotObservation]:
        """Threshold a grayscale image, return the largest blob's centroid."""
        _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
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
        # Ambiguity penalty — more than one blob means we may have locked
        # onto a reflection or stray light, not the dot.
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
