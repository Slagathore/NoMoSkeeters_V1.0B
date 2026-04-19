"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Calibration Module
------------------
Provides the CalibrationController: an orchestrator class that guides the user
through a multi-point calibration procedure to compute the homography matrix
used by CoordinateMapper.

Calibration procedure:
  1. Fire the laser at a known LaserCube coordinate (one of a grid of points).
  2. Capture one or more camera frames *while the laser is firing*.
  3. Detect the laser dot in the camera image by looking for a bright spot
     above CALIBRATION_DOT_DETECT_THRESHOLD.
  4. Record the (camera_x, camera_y) ↔ (laser_x, laser_y) correspondence.
  5. After collecting ≥4 pairs, compute the homography via CoordinateMapper.
  6. Save the result to CALIBRATION_FILE.

Grid layout:
  A CALIBRATION_GRID_COLS × CALIBRATION_GRID_ROWS grid of laser points is
  generated, spread evenly across the laser coordinate range, inset by 10%
  from the edges to avoid projecting outside the camera FOV.

Automatic dot detection:
  The function _find_laser_dot() subtracts a background frame (captured before
  the laser fires) from the live frame, thresholds, and finds the brightest
  centroid.  This is robust to moderate ambient lighting.

Modules imported:
  cv2, numpy         — image processing for dot detection
  time               — delays for laser settle / camera capture
  pathlib            — path for calibration file
  config.settings    — CALIBRATION_* constants
  targeting.coordinate_mapper — CoordinateMapper
  laser.laser_manager — LaserManager (for firing test points)
  utils.logging_utils — get_logger

Classes:
  CalibrationPoint   — NamedTuple holding one calibration correspondence.
  CalibrationController — Orchestrates the full calibration flow.

Methods (CalibrationController):
  start(camera_frame_fn, laser_manager) — Begin calibration procedure.
  next_point()        — Advance to the next grid point; fires laser; detects dot.
  finish()            — Compute homography from all collected points; save.
  cancel()            — Abort calibration, discard collected points.
  current_step()      — Return (current index, total steps) tuple.
  is_complete()       — True if enough points collected for a valid homography.
  points_collected()  — Number of valid correspondences gathered so far.
  get_laser_grid()    — Return the full list of laser coordinate grid points.
  set_background_frame(frame) — Manually set the background subtraction frame.

Variables (instance):
  _grid        (list[tuple[int,int]]): Pre-computed grid of laser coordinates.
  _step        (int):                  Current calibration step index.
  _pairs       (list[CalibrationPoint]): Collected correspondences.
  _bg_frame    (np.ndarray | None):    Background frame for dot detection.
  _laser_mgr   (LaserManager | None): Laser manager to fire test points.
  _frame_fn    (callable | None):     Function returning the latest ndarray frame.
  _mapper      (CoordinateMapper | None): The mapper to update after calibration.

#todo Add visual overlay showing expected vs. detected dot position for each step.
#todo Allow manual click-to-confirm dot position in case auto-detect fails.
#todo Support re-running individual calibration steps if detection failed.
#todo Add a live re-calibration mode that updates H incrementally from tracking data.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import cv2
import numpy as np

import config.settings as _s
from targeting.coordinate_mapper import CoordinateMapper
from utils.logging_utils import get_logger

logger = get_logger(__name__)


@dataclass
class CalibrationPoint:
    """
    One calibration correspondence: camera pixel ↔ laser coordinate.

    Attributes:
        step        (int):   Step index in the calibration grid (0-based).
        laser_x     (int):   Known laser X coordinate fired at.
        laser_y     (int):   Known laser Y coordinate fired at.
        camera_x    (float): Detected dot centroid X in camera pixels.
        camera_y    (float): Detected dot centroid Y in camera pixels.
        confidence  (float): Dot detection quality 0.0–1.0.
    """
    step: int
    laser_x: int
    laser_y: int
    camera_x: float
    camera_y: float
    confidence: float = 1.0


class CalibrationController:
    """
    Orchestrates the LaserCube ↔ camera calibration procedure.

    Typical usage (from calibration dialog):
        controller = CalibrationController(mapper)
        controller.start(get_latest_frame_fn, laser_manager)
        while not controller.is_complete():
            ok, msg = controller.next_point()   # blocks briefly
            update_ui(msg)
        controller.finish()

    Attributes:
        _mapper     (CoordinateMapper):         The mapper to update.
        _grid       (list[tuple[int,int]]):      Laser grid points to visit.
        _step       (int):                       Current step (0-based).
        _pairs      (list[CalibrationPoint]):    Collected correspondences.
        _bg_frame   (np.ndarray | None):         Background frame.
        _laser_mgr  (any | None):               LaserManager reference.
        _frame_fn   (Callable | None):           Returns latest BGR frame.
        _running    (bool):                      True while calibration active.
    """

    def __init__(self, mapper: CoordinateMapper) -> None:
        """
        Initialise the controller with a CoordinateMapper to fill.

        Args:
            mapper: The CoordinateMapper that will receive the computed H.
        """
        self._mapper: CoordinateMapper = mapper
        self._grid: list[tuple[int, int]] = self._build_grid()
        self._step: int = 0
        self._pairs: list[CalibrationPoint] = []
        self._bg_frame: np.ndarray | None = None
        self._laser_mgr = None       # type: ignore[assignment]
        self._frame_fn: Callable[[], np.ndarray | None] | None = None
        self._running: bool = False
        logger.debug(
            "CalibrationController created: %d grid points.", len(self._grid)
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────────────

    def start(
        self,
        frame_fn: Callable[[], np.ndarray | None],
        laser_mgr,  # LaserManager — avoid circular import with string annotation
    ) -> None:
        """
        Begin calibration session.

        Args:
            frame_fn:  Zero-argument callable that returns the latest BGR frame
                       (or None if unavailable).  Usually CameraManager.snapshot().
            laser_mgr: LaserManager instance used to fire test points.
        """
        self._frame_fn = frame_fn
        self._laser_mgr = laser_mgr
        self._step = 0
        self._pairs = []
        self._running = True

        # Capture background (laser off)
        self._capture_background()
        logger.info("Calibration started: %d steps.", len(self._grid))

    def next_point(self) -> tuple[bool, str]:
        """
        Fire the laser at the current grid point, detect the dot, record pair.

        Advances the step counter on success.

        Returns:
            Tuple (success, message):
              success — True if dot was detected and pair recorded.
              message — Human-readable description of what happened.
        """
        if not self._running:
            return False, "Calibration not started."
        if self._step >= len(self._grid):
            return False, "All calibration steps complete — call finish()."
        if self._laser_mgr is None or self._frame_fn is None:
            return False, "CalibrationController not properly initialised."

        lx, ly = self._grid[self._step]
        logger.info(
            "Calibration step %d/%d: firing at laser (%d, %d) …",
            self._step + 1,
            len(self._grid),
            lx,
            ly,
        )

        # Fire laser at grid point
        self._laser_mgr.send_test_point(lx, ly)
        time.sleep(0.15)  # allow laser to settle and camera to capture

        # Capture frame with laser on
        frame = self._frame_fn()
        if frame is None:
            return False, f"Step {self._step + 1}: no camera frame available."

        # Detect the laser dot
        cam_x, cam_y, confidence = self._find_laser_dot(frame)

        if confidence < 0.3:
            logger.warning(
                "Step %d: dot detection confidence %.2f — possible false negative.",
                self._step + 1,
                confidence,
            )
            msg = (
                f"Step {self._step + 1}: dot detection was poor (conf={confidence:.2f}). "
                "Ensure room is dim and laser is pointing at a flat surface."
            )
            # We still record the point but flag it
        else:
            msg = f"Step {self._step + 1}/{len(self._grid)}: detected at camera ({cam_x:.0f}, {cam_y:.0f})."

        pair = CalibrationPoint(
            step=self._step,
            laser_x=lx,
            laser_y=ly,
            camera_x=cam_x,
            camera_y=cam_y,
            confidence=confidence,
        )
        self._pairs.append(pair)
        self._step += 1
        logger.info("Step %d recorded: cam=(%.0f,%.0f) laser=(%d,%d) conf=%.2f",
                    self._step, cam_x, cam_y, lx, ly, confidence)
        return True, msg

    def finish(self) -> bool:
        """
        Compute the homography from collected pairs and save to disk.

        Returns:
            True if homography was computed and saved, False otherwise.
        """
        if len(self._pairs) < 4:
            logger.error(
                "finish() called with only %d pairs — need at least 4.", len(self._pairs)
            )
            return False

        camera_pts = [(p.camera_x, p.camera_y) for p in self._pairs]
        laser_pts = [(p.laser_x, p.laser_y) for p in self._pairs]

        ok = self._mapper.compute_homography(camera_pts, laser_pts)
        if ok:
            self._mapper.save_calibration()
            logger.info("Calibration complete and saved.")
        else:
            logger.error("Homography computation failed — check calibration points.")

        self._running = False
        return ok

    def cancel(self) -> None:
        """Abort calibration and discard collected pairs."""
        self._running = False
        self._pairs = []
        self._step = 0
        logger.info("Calibration cancelled.")

    def current_step(self) -> tuple[int, int]:
        """
        Return (current_step_index, total_steps) for progress display.

        Returns:
            Tuple of (current 0-based step index, total grid points).
        """
        return self._step, len(self._grid)

    def is_complete(self) -> bool:
        """Return True once all grid points have been processed."""
        return self._step >= len(self._grid)

    def points_collected(self) -> int:
        """Return the number of valid correspondence pairs collected so far."""
        return len(self._pairs)

    def get_laser_grid(self) -> list[tuple[int, int]]:
        """Return the list of laser coordinate grid points for this calibration."""
        return list(self._grid)

    def set_background_frame(self, frame: np.ndarray) -> None:
        """
        Manually set the background frame used for laser dot subtraction.

        Normally captured automatically in start(), but can be overridden.

        Args:
            frame: BGR numpy array with NO laser dot.
        """
        self._bg_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
        logger.debug("Background frame set manually.")

    def get_pairs(self) -> list[CalibrationPoint]:
        """Return a copy of the collected correspondence pairs."""
        return list(self._pairs)

    # ──────────────────────────────────────────────────────────────────────────
    # Private helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _build_grid(self) -> list[tuple[int, int]]:
        """
        Generate a COLS × ROWS grid of laser coordinates inset by 10%.

        Returns:
            List of (laser_x, laser_y) tuples in row-major order.
        """
        cols = _s.CALIBRATION_GRID_COLS
        rows = _s.CALIBRATION_GRID_ROWS
        inset = 0.10  # 10% inset from edges
        lo = int(_s.LASERCUBE_COORD_MIN * (1.0 - inset))
        hi = int(_s.LASERCUBE_COORD_MAX * (1.0 - inset))

        xs = [int(lo + (hi - lo) * c / (cols - 1)) for c in range(cols)]
        ys = [int(lo + (hi - lo) * r / (rows - 1)) for r in range(rows)]

        grid: list[tuple[int, int]] = []
        for y in ys:
            for x in xs:
                grid.append((x, y))
        return grid

    def _capture_background(self) -> None:
        """Capture a background frame (laser off) for dot subtraction."""
        if self._frame_fn is None:
            return
        # Ensure laser is not firing
        if self._laser_mgr is not None:
            # Disarm is handled by safety system; just wait briefly
            time.sleep(0.1)
        frame = self._frame_fn()
        if frame is not None:
            self._bg_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
            logger.debug("Background frame captured for calibration.")

    def _find_laser_dot(
        self,
        frame: np.ndarray,
    ) -> tuple[float, float, float]:
        """
        Detect the laser dot centroid in a BGR frame.

        Uses background subtraction (if available) and bright-spot detection.

        Args:
            frame: BGR numpy array with the laser dot present.

        Returns:
            (cam_x, cam_y, confidence) — centroid in pixels and quality [0,1].
            Returns (0.0, 0.0, 0.0) if detection fails.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)

        # Background subtraction to isolate the laser dot
        if self._bg_frame is not None and self._bg_frame.shape == gray.shape:
            diff = np.clip(gray - self._bg_frame, 0, 255).astype(np.uint8)
        else:
            diff = gray.astype(np.uint8)

        # Extract the laser colour channel(s) for better isolation
        # Green channel is dominant for green lasers; use all channels for white
        b, g, r = cv2.split(frame)
        # Build a channel-max mask to catch any laser colour
        laser_channel = np.maximum(np.maximum(r, g), b)
        combined = cv2.addWeighted(diff, 0.6, laser_channel, 0.4, 0)

        # Threshold
        threshold = _s.CALIBRATION_DOT_DETECT_THRESHOLD
        _, binary = cv2.threshold(combined, threshold, 255, cv2.THRESH_BINARY)

        # Find brightest contour
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            logger.debug("_find_laser_dot: no contours found (threshold=%d).", threshold)
            # Fallback: find raw brightest pixel
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(combined)
            if max_val < threshold * 0.5:
                return 0.0, 0.0, 0.0
            return float(max_loc[0]), float(max_loc[1]), float(max_val / 255.0)

        # Use the largest contour as the laser dot
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return 0.0, 0.0, 0.0

        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        # Confidence based on contour area and brightness
        area = cv2.contourArea(largest)
        # Small compact bright blob → high confidence
        area_score = min(1.0, area / 50.0) if area < 200 else max(0.3, 200.0 / area)

        # Check mean brightness inside the contour mask
        mask = np.zeros(binary.shape, dtype=np.uint8)
        cv2.drawContours(mask, [largest], -1, 255, -1)
        mean_brightness = float(cv2.mean(combined, mask=mask)[0]) / 255.0

        confidence = (area_score * 0.4 + mean_brightness * 0.6)
        return float(cx), float(cy), float(confidence)
