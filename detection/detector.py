"""Detector — MOG2 background-subtraction blob detection.

Pipeline: grayscale → blur → MOG2 foreground mask → threshold → morphology →
contours → geometric pre-filter → optional classifier post-filter. Every
threshold is a config tunable; nothing is hardcoded in the pipeline.

Reference: BOOTSTRAP.md §6.1, BOOTSTRAP_AMENDMENTS.md §6.1 (the label/conf
default fix — defaults are set BEFORE the optional classifier branch).
"""
from __future__ import annotations

import logging
import time
from typing import Callable, Optional

import cv2
import numpy as np

from config import settings
from detection.classifier import DetectionClassifier
from events.schemas import DetectionEvent

_log = logging.getLogger(__name__)

# Optional world-position sampler: (x_norm, y_norm) → (x_m, y_m, z_m) or
# None. Supplied by depth-capable call sites (e.g. Kinect); populates the
# DetectionEvent's *_world_m fields that kalman_3d tracking and the
# object-size guard consume.
WorldFn = Callable[[float, float], Optional[tuple[float, float, float]]]


class Detector:
    """Per-sensor blob detector. Hold one Detector per camera stream — MOG2
    carries a background model that must not be shared across views."""

    def __init__(self,
                 classifier: Optional[DetectionClassifier] = None,
                 *,
                 mog2_history: int = 500,
                 mog2_var_threshold: float = 16.0,
                 mog2_detect_shadows: bool = False):
        self._classifier = classifier
        self._mog2 = cv2.createBackgroundSubtractorMOG2(
            history=mog2_history,
            varThreshold=mog2_var_threshold,
            detectShadows=mog2_detect_shadows,
        )

    def detect_bgsub(self,
                     frame: np.ndarray,
                     *,
                     sensor_id: str = "",
                     timestamp: Optional[float] = None,
                     world_fn: Optional[WorldFn] = None,
                     ) -> list[DetectionEvent]:
        """Run the background-subtraction pipeline on one BGR frame."""
        if timestamp is None:
            timestamp = time.monotonic()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if settings.DETECTION_BLUR_KERNEL > 1:
            k = settings.DETECTION_BLUR_KERNEL
            gray = cv2.GaussianBlur(gray, (k, k), 0)

        fg_mask = self._mog2.apply(gray)
        _, thresh = cv2.threshold(fg_mask, settings.DETECTION_THRESHOLD,
                                  255, cv2.THRESH_BINARY)
        morph_k = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (settings.DETECTION_MORPH_KERNEL,) * 2)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, morph_k)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, morph_k)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        detections: list[DetectionEvent] = []
        det_id = 0
        for c in contours:
            # Geometric pre-filter (cheap): area, then aspect.
            area = cv2.contourArea(c)
            if not (settings.DETECTION_MIN_AREA <= area
                    <= settings.DETECTION_MAX_AREA):
                continue
            _, _, w, h = cv2.boundingRect(c)
            aspect = w / h if h else 0.0
            if not (settings.DETECTION_MIN_ASPECT <= aspect
                    <= settings.DETECTION_MAX_ASPECT):
                continue

            # §6.1 fix: defaults set BEFORE the optional classifier branch so
            # _build_detection() always has a valid label/conf.
            label = "candidate"
            conf = 0.5
            if self._classifier is not None:
                label, conf = self._classifier.classify(frame, c)
                if label != "mosquito":
                    continue

            detections.append(self._build_detection(
                frame, c, label, conf, det_id, sensor_id, timestamp,
                world_fn))
            det_id += 1
        return detections

    @staticmethod
    def _build_detection(frame: np.ndarray,
                         contour: np.ndarray,
                         label: str,
                         conf: float,
                         detection_id: int,
                         sensor_id: str,
                         timestamp: float,
                         world_fn: Optional[WorldFn] = None) -> DetectionEvent:
        """Construct a DetectionEvent from one accepted contour."""
        area = float(cv2.contourArea(contour))
        x, y, w, h = cv2.boundingRect(contour)
        m = cv2.moments(contour)
        if m["m00"] != 0.0:
            cx = m["m10"] / m["m00"]
            cy = m["m01"] / m["m00"]
        else:                                   # fall back to bbox center
            cx, cy = x + w / 2.0, y + h / 2.0

        frame_h, frame_w = frame.shape[:2]
        x_norm = cx / max(1, frame_w - 1)
        y_norm = cy / max(1, frame_h - 1)

        x_w = y_w = z_w = None
        if world_fn is not None:
            try:
                w3 = world_fn(x_norm, y_norm)
            except Exception:                             # pragma: no cover
                _log.exception("world_fn raised; detection stays 2D")
                w3 = None
            if w3 is not None:
                x_w, y_w, z_w = float(w3[0]), float(w3[1]), float(w3[2])

        return DetectionEvent(
            timestamp=timestamp,
            sensor_id=sensor_id,
            detection_id=detection_id,
            x_norm=x_norm,
            y_norm=y_norm,
            x_px=int(round(cx)),
            y_px=int(round(cy)),
            area_pixels=area,
            bbox=(int(x), int(y), int(w), int(h)),
            classifier_label=label,
            classifier_confidence=float(conf),
            x_world_m=x_w,
            y_world_m=y_w,
            z_world_m=z_w,
        )
