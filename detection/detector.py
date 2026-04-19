"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Detector Module
---------------
Provides the MosquitoDetector class: a QThread that receives BGR camera frames
via a queue, runs detection, and emits a list of Detection results as a Qt signal.

Detection pipeline (two modes selectable at runtime):

MODE: BACKGROUND_SUBTRACTION (default — fast, no deep-learning required)
  1. Convert frame to grayscale.
  2. Apply Gaussian blur to reduce sensor noise.
  3. Feed to MOG2 background subtractor → foreground mask.
  4. Apply morphological open + close to clean isolated pixels.
  5. Find contours in the mask.
  6. Filter contours by: area (min/max), aspect ratio, solidity.
  7. Each surviving contour → one Detection namedtuple.

MODE: YOLO (more accurate, requires ultralytics + a suitable model)
  1. Run ultralytics YOLO inference on the frame.
  2. Filter predictions by confidence threshold.
  3. Wrap each detection bounding box in a Detection namedtuple.
  NOTE: For mosquito detection you'll need a custom-trained model.  The default
        yolov8n.pt (ImageNet classes) does not include mosquitos; it will
        detect them as "airplane" or "sports ball" at low confidence.
        Use background-subtraction mode until a custom model is trained.

Modules imported:
  cv2, numpy         — image processing
  queue, threading   — thread-safe frame queue
  PySide6.QtCore     — QThread, Signal
  config.settings    — detection constants
  utils.logging_utils — get_logger

Classes:
  Detection          — NamedTuple: (cx, cy, x1, y1, x2, y2, area, confidence, label)
  DetectionMode      — Enum: BACKGROUND_SUBTRACTION | YOLO
  MosquitoDetector   — QThread detection worker.

Methods (MosquitoDetector):
  set_mode(mode)           — Switch detection algorithm at runtime.
  push_frame(frame)        — Add a new frame to the processing queue.
  stop()                   — Gracefully stop the detection thread.
  set_sensitivity(value)   — Adjust detection sensitivity (0.0–1.0 scale).
  reset_background()       — Re-initialise the background subtractor model.

Variables (instance):
  _mode           — DetectionMode enum value.
  _frame_queue    — queue.Queue of incoming frames.
  _running        — bool loop control flag.
  _bg_subtractor  — cv2.BackgroundSubtractorMOG2 instance.
  _yolo_model     — ultralytics YOLO instance or None.
  _sensitivity    — float 0.0–1.0 controlling detection threshold.
  _morph_kernel   — cv2 structuring element for morphological ops.

#todo Train a custom YOLO model on a mosquito dataset for accurate detection.
#todo Add infrared support: mosquitos are very visible on IR cameras.
#todo Add a "learn background" on-demand button (clears and retrains MOG2).
#todo Measure and report detection latency per frame in the GUI stats panel.
#todo Add a confidence visualiser that shows the foreground mask as an overlay.
"""

from __future__ import annotations

import enum
import queue
import time
from typing import NamedTuple

import cv2
import numpy as np
from PySide6.QtCore import QThread, Signal

import config.settings as _s
from utils.logging_utils import get_logger

logger = get_logger(__name__)


# ─── Result type ──────────────────────────────────────────────────────────────

class Detection(NamedTuple):
    """
    Represents a single mosquito candidate detected in one frame.

    Attributes:
        cx         (float): Centroid X in camera pixel space.
        cy         (float): Centroid Y in camera pixel space.
        x1         (int):   Bounding box left pixel.
        y1         (int):   Bounding box top pixel.
        x2         (int):   Bounding box right pixel.
        y2         (int):   Bounding box bottom pixel.
        area       (float): Contour area in pixels².
        confidence (float): Detection confidence 0.0–1.0 (always 1.0 for BGSub).
        label      (str):   Short description, e.g. "mosquito" or YOLO class name.
    """
    cx: float
    cy: float
    x1: int
    y1: int
    x2: int
    y2: int
    area: float
    confidence: float
    label: str


class DetectionMode(enum.Enum):
    """
    Enum selecting which detection algorithm is used.

    Attributes:
        BACKGROUND_SUBTRACTION: MOG2 foreground extraction + contour filtering.
        YOLO: Ultralytics YOLO model inference.
    """
    BACKGROUND_SUBTRACTION = "bgsub"
    YOLO = "yolo"


# ─── Detector worker thread ───────────────────────────────────────────────────

class MosquitoDetector(QThread):
    """
    QThread worker that processes camera frames and emits detection results.

    Frames are pushed via push_frame() from the camera thread and processed
    asynchronously so the camera thread is never blocked.  Results are emitted
    as a Qt signal for the GUI and tracker threads to consume.

    Signals:
        detections_ready (list[Detection]): Emitted after each frame is processed.
        fps_updated      (float):           Detection processing FPS.

    Attributes:
        _mode          (DetectionMode): Current algorithm.
        _frame_queue   (queue.Queue):   Buffered incoming frames.
        _running       (bool):          Loop control flag.
        _bg_subtractor (cv2.BackgroundSubtractorMOG2): Background model.
        _yolo_model    (any | None):    Loaded YOLO model.
        _sensitivity   (float):         Detection sensitivity [0, 1].
        _morph_kernel  (np.ndarray):    Morphological structuring element.
        _fps_times     (list[float]):   Recent frame timestamps for FPS.
    """

    detections_ready = Signal(list)
    fps_updated = Signal(float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._mode: DetectionMode = DetectionMode.BACKGROUND_SUBTRACTION
        self._frame_queue: queue.Queue[np.ndarray] = queue.Queue(maxsize=4)
        self._running: bool = False
        self._bg_subtractor = self._make_bg_subtractor()
        self._yolo_model = None
        self._sensitivity: float = 0.5   # mid-range default
        self._morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (_s.DETECTION_MORPH_KERNEL, _s.DETECTION_MORPH_KERNEL),
        )
        self._fps_times: list[float] = []

    # ──────────────────────────────────────────────────────────────────────────
    # Configuration
    # ──────────────────────────────────────────────────────────────────────────

    def set_mode(self, mode: DetectionMode) -> None:
        """
        Switch detection algorithm.  Takes effect on the next processed frame.

        If switching to YOLO and the model is not yet loaded, attempts to load
        it now (blocking for a few seconds on first load).

        Args:
            mode: DetectionMode enum value.
        """
        self._mode = mode
        logger.info("Detection mode set to %s.", mode.value)
        if mode == DetectionMode.YOLO and self._yolo_model is None:
            self._load_yolo()

    def set_sensitivity(self, value: float) -> None:
        """
        Adjust overall detection sensitivity.

        For BACKGROUND_SUBTRACTION this scales the foreground threshold
        (lower threshold = more sensitive = more false positives).
        For YOLO it adjusts the confidence cutoff.

        Args:
            value: float in [0.0, 1.0]; 0.0 = most lenient, 1.0 = strictest.
        """
        self._sensitivity = max(0.0, min(1.0, value))
        logger.debug("Detection sensitivity = %.2f", self._sensitivity)

    def reset_background(self) -> None:
        """
        Reinitialise the background subtractor, discarding the current model.
        Useful after camera repositioning or large lighting changes.
        """
        self._bg_subtractor = self._make_bg_subtractor()
        logger.info("Background model reset.")

    # ──────────────────────────────────────────────────────────────────────────
    # Frame injection (called from camera thread)
    # ──────────────────────────────────────────────────────────────────────────

    def push_frame(self, frame: np.ndarray) -> None:
        """
        Add a frame to the processing queue.

        Drops the frame (with a debug log) if the queue is full to prevent
        memory growth when detection is slower than capture.

        Args:
            frame: BGR numpy array from the camera.
        """
        try:
            self._frame_queue.put_nowait(frame)
        except queue.Full:
            logger.debug("Detection queue full — frame dropped.")

    # ──────────────────────────────────────────────────────────────────────────
    # QThread entry point
    # ──────────────────────────────────────────────────────────────────────────

    def run(self) -> None:
        """
        Detection worker loop.  Blocks on the frame queue, processes each frame,
        then emits detections_ready with the result list.
        """
        self._running = True
        logger.info("MosquitoDetector thread started (mode=%s).", self._mode.value)

        while self._running:
            try:
                frame = self._frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                t0 = time.monotonic()
                if self._mode == DetectionMode.BACKGROUND_SUBTRACTION:
                    detections = self._detect_bgsub(frame)
                else:
                    detections = self._detect_yolo(frame)

                self.detections_ready.emit(detections)
                self._update_fps(time.monotonic() - t0)

            except Exception as exc:  # noqa: BLE001
                logger.error("Detection error: %s", exc, exc_info=True)

        logger.info("MosquitoDetector thread stopped.")

    def stop(self) -> None:
        """Gracefully stop the detection loop and wait for thread exit."""
        self._running = False
        self.wait(3000)

    # ──────────────────────────────────────────────────────────────────────────
    # Background subtraction detection
    # ──────────────────────────────────────────────────────────────────────────

    def _detect_bgsub(self, frame: np.ndarray) -> list[Detection]:
        """
        Run MOG2 background subtraction and contour filtering on one frame.

        Args:
            frame: BGR numpy array.

        Returns:
            List of Detection namedtuples for each mosquito candidate.
        """
        # Sensitivity → threshold: high sensitivity = lower threshold value
        threshold = int(_s.DETECTION_THRESHOLD * (1.0 + (0.5 - self._sensitivity)))
        threshold = max(5, min(100, threshold))

        # 1. Grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2. Blur
        blurred = cv2.GaussianBlur(
            gray,
            (_s.DETECTION_BLUR_KERNEL, _s.DETECTION_BLUR_KERNEL),
            0,
        )

        # 3. Background subtraction → foreground mask
        fg_mask = self._bg_subtractor.apply(blurred)

        # 4. Threshold the mask (removes shadows that MOG2 marks as 127)
        _, thresh = cv2.threshold(fg_mask, threshold, 255, cv2.THRESH_BINARY)

        # 5. Morphological cleanup
        opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, self._morph_kernel)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, self._morph_kernel)

        # 6. Find contours
        contours, _ = cv2.findContours(
            closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # 7. Filter contours and build detections
        detections: list[Detection] = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < _s.DETECTION_MIN_AREA or area > _s.DETECTION_MAX_AREA:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if h == 0:
                continue
            aspect = w / h
            if aspect < _s.DETECTION_MIN_ASPECT or aspect > _s.DETECTION_MAX_ASPECT:
                continue

            # Solidity check: mosquito contour should be fairly solid
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area > 0:
                solidity = area / hull_area
                if solidity < 0.2:  # very hollow → likely noise
                    continue

            cx = x + w / 2.0
            cy = y + h / 2.0
            detections.append(
                Detection(
                    cx=cx,
                    cy=cy,
                    x1=x,
                    y1=y,
                    x2=x + w,
                    y2=y + h,
                    area=area,
                    confidence=1.0,
                    label="mosquito",
                )
            )

        return detections

    # ──────────────────────────────────────────────────────────────────────────
    # YOLO detection
    # ──────────────────────────────────────────────────────────────────────────

    def _detect_yolo(self, frame: np.ndarray) -> list[Detection]:
        """
        Run YOLO inference and convert results to Detection namedtuples.

        Falls back to background subtraction if the model failed to load.

        Args:
            frame: BGR numpy array.

        Returns:
            List of Detection namedtuples.
        """
        if self._yolo_model is None:
            logger.warning("YOLO model not loaded; falling back to BGSub.")
            return self._detect_bgsub(frame)

        conf_threshold = _s.DETECTION_YOLO_CONF * (0.5 + self._sensitivity)
        conf_threshold = max(0.1, min(0.99, conf_threshold))

        try:
            results = self._yolo_model.predict(
                frame,
                conf=conf_threshold,
                verbose=False,
                device="cpu",
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("YOLO inference error: %s", exc)
            return []

        detections: list[Detection] = []
        for result in results:
            for box in result.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                label = result.names.get(cls_id, str(cls_id))
                x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                area = (x2 - x1) * (y2 - y1)
                detections.append(
                    Detection(
                        cx=cx,
                        cy=cy,
                        x1=x1,
                        y1=y1,
                        x2=x2,
                        y2=y2,
                        area=float(area),
                        confidence=conf,
                        label=label,
                    )
                )
        return detections

    # ──────────────────────────────────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _make_bg_subtractor(self) -> cv2.BackgroundSubtractorMOG2:
        """
        Create and return a fresh MOG2 background subtractor.

        Returns:
            cv2.BackgroundSubtractorMOG2 instance with settings from config.
        """
        return cv2.createBackgroundSubtractorMOG2(
            history=_s.DETECTION_MOG2_HISTORY,
            varThreshold=_s.DETECTION_MOG2_VARTH,
            detectShadows=False,  # shadows slow things down and we don't need them
        )

    def _load_yolo(self) -> None:
        """
        Attempt to import ultralytics and load the configured YOLO model.
        Logs an error and leaves _yolo_model=None if import/load fails.
        """
        try:
            from ultralytics import YOLO  # type: ignore[import]
            self._yolo_model = YOLO(_s.DETECTION_YOLO_MODEL)
            logger.info("YOLO model loaded: %s", _s.DETECTION_YOLO_MODEL)
        except ImportError:
            logger.error(
                "ultralytics not installed — pip install ultralytics; "
                "staying in BGSub mode."
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("YOLO model load failed: %s", exc)

    def _update_fps(self, elapsed: float) -> None:
        """
        Track per-frame detection time and emit fps_updated periodically.

        Args:
            elapsed: Processing time for the last frame in seconds.
        """
        now = time.monotonic()
        self._fps_times.append(now)
        self._fps_times = [t for t in self._fps_times if now - t <= 2.0]
        if len(self._fps_times) >= 2 and len(self._fps_times) % 10 == 0:
            fps = (len(self._fps_times) - 1) / (self._fps_times[-1] - self._fps_times[0])
            self.fps_updated.emit(fps)
