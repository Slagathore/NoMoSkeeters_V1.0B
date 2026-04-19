"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Coordinate Mapper Module
------------------------
Converts 2-D camera pixel coordinates to 2-D LaserCube galvo coordinates
using a perspective homography matrix computed during calibration.

Background
----------
The camera and laser projector see the same physical space from different
viewpoints (angles, distances) and have different native coordinate systems:
  - Camera:    pixel (0,0) top-left → (W-1, H-1) bottom-right
  - LaserCube: signed int (–32767, –32767) bottom-left → (+32767, +32767) top-right
               (Y axis is INVERTED relative to camera conventions)

A 3×3 perspective homography matrix H maps camera pixel coordinates to
normalised laser coordinates.  H is computed in CalibrationController by
collecting at least 4 point correspondences.

If no calibration has been performed, the mapper falls back to a linear
scale-and-flip transform (assumes camera FOV == laser FOV, centred).

Modules imported:
  cv2, numpy   — homography operations
  json, pathlib — load/save calibration JSON
  config.settings — CALIBRATION_FILE, LASERCUBE_COORD_MIN/MAX
  utils.logging_utils — get_logger

Classes:
  CoordinateMapper   — Applies a homography to camera→laser coordinate pairs.

Methods (CoordinateMapper):
  load_calibration(path)     — Load H matrix from a JSON calibration file.
  save_calibration(path)     — Save current H matrix to JSON.
  camera_to_laser(cx, cy)    — Map one camera pixel to (laser_x, laser_y).
  camera_to_laser_batch(pts) — Map an (N,2) array of points.
  compute_homography(src_pts, dst_pts) — Fit a new H from correspondence pairs.
  is_calibrated()            — True if a valid homography matrix is loaded.
  reset()                    — Clear calibration state.
  fallback_transform(cx,cy,w,h) — Linear fallback when not calibrated.

Variables (instance):
  _H          (np.ndarray | None): 3×3 homography matrix.
  _calibrated (bool):              True if H is valid.
  _frame_w    (int):               Camera frame width (for fallback).
  _frame_h    (int):               Camera frame height (for fallback).

#todo Add a reprojection error report after each calibration to flag bad points.
#todo Support higher-order polynomial distortion correction for wide-angle lenses.
#todo Allow the user to manually nudge calibration if hardware has moved slightly.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

import config.settings as _s
from utils.logging_utils import get_logger

logger = get_logger(__name__)


class CoordinateMapper:
    """
    Maps camera pixel coordinates to LaserCube galvo coordinates via homography.

    After calibration, call camera_to_laser(cx, cy) to transform a centroid
    returned by the detector/tracker into the (laser_x, laser_y) pair expected
    by LaserCubeInterface.send_frame().

    Attributes:
        _H          (np.ndarray | None): 3×3 float64 homography matrix.
        _calibrated (bool):              True when a valid H is loaded/computed.
        _frame_w    (int):               Last-known camera frame width.
        _frame_h    (int):               Last-known camera frame height.
    """

    def __init__(self, frame_w: int = 1920, frame_h: int = 1080) -> None:
        """
        Initialise the mapper with optional frame dimensions for the fallback.

        Args:
            frame_w: Camera frame width in pixels.
            frame_h: Camera frame height in pixels.
        """
        self._H: np.ndarray | None = None
        self._calibrated: bool = False
        self._frame_w: int = frame_w
        self._frame_h: int = frame_h

        # Attempt auto-load from the default calibration file
        if _s.CALIBRATION_FILE.exists():
            self.load_calibration(_s.CALIBRATION_FILE)

    # ──────────────────────────────────────────────────────────────────────────
    # Calibration state
    # ──────────────────────────────────────────────────────────────────────────

    def is_calibrated(self) -> bool:
        """Return True if a valid homography matrix is loaded."""
        return self._calibrated and self._H is not None

    def reset(self) -> None:
        """Clear the current calibration — mapper will use the fallback transform."""
        self._H = None
        self._calibrated = False
        logger.info("CoordinateMapper calibration cleared.")

    def set_frame_size(self, w: int, h: int) -> None:
        """
        Update the camera frame dimensions used by the fallback transform.

        Args:
            w: Frame width in pixels.
            h: Frame height in pixels.
        """
        self._frame_w = w
        self._frame_h = h

    # ──────────────────────────────────────────────────────────────────────────
    # Homography computation
    # ──────────────────────────────────────────────────────────────────────────

    def compute_homography(
        self,
        camera_pts: list[tuple[float, float]],
        laser_pts: list[tuple[int, int]],
    ) -> bool:
        """
        Compute and store the homography H from camera ↔ laser point pairs.

        Requires at least 4 non-collinear correspondence pairs for a solvable
        perspective transform.

        Args:
            camera_pts: List of (x, y) in camera pixel space.
            laser_pts:  List of (x, y) in LaserCube coordinate space
                        (–32767 … +32767 for both axes).

        Returns:
            True if homography was computed successfully.
        """
        if len(camera_pts) < 4 or len(laser_pts) < 4:
            logger.error("Need at least 4 calibration points (got %d).", len(camera_pts))
            return False
        if len(camera_pts) != len(laser_pts):
            logger.error("camera_pts and laser_pts must have equal length.")
            return False

        src = np.array(camera_pts, dtype=np.float32)
        dst = np.array(laser_pts, dtype=np.float32)

        H, mask = cv2.findHomography(src, dst, method=cv2.RANSAC, ransacReprojThreshold=8.0)

        if H is None:
            logger.error("findHomography returned None — check calibration points.")
            return False

        inlier_count = int(mask.ravel().sum()) if mask is not None else len(camera_pts)
        logger.info(
            "Homography computed: %d/%d inliers.", inlier_count, len(camera_pts)
        )

        self._H = H
        self._calibrated = True

        # Compute and log reprojection error
        error = self._reprojection_error(src, dst, H)
        logger.info("Calibration reprojection error: %.2f px", error)
        if error > 500:
            logger.warning(
                "High reprojection error (%.2f) — calibration may be inaccurate. "
                "Retry with better-spread calibration points.", error
            )
        return True

    # ──────────────────────────────────────────────────────────────────────────
    # Coordinate transformation
    # ──────────────────────────────────────────────────────────────────────────

    def camera_to_laser(
        self,
        cx: float,
        cy: float,
    ) -> tuple[int, int]:
        """
        Transform one camera pixel coordinate to LaserCube coordinates.

        Uses the calibrated homography if available, otherwise falls back to
        a simple linear scale and Y-flip transform.

        Args:
            cx: Camera pixel X.
            cy: Camera pixel Y.

        Returns:
            (laser_x, laser_y) clamped to [LASERCUBE_COORD_MIN, LASERCUBE_COORD_MAX].
        """
        if self._calibrated and self._H is not None:
            return self._apply_homography(cx, cy)
        return self.fallback_transform(cx, cy, self._frame_w, self._frame_h)

    def camera_to_laser_batch(
        self,
        points: np.ndarray,
    ) -> np.ndarray:
        """
        Batch-transform an (N, 2) array of camera pixel points.

        Args:
            points: numpy array shape (N, 2), dtype float.

        Returns:
            numpy array shape (N, 2) of (laser_x, laser_y) pairs as int32.
        """
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("points must be shape (N, 2)")

        if self._calibrated and self._H is not None:
            pts_h = points.reshape(-1, 1, 2).astype(np.float32)
            transformed = cv2.perspectiveTransform(pts_h, self._H)
            result = transformed.reshape(-1, 2)
        else:
            result = np.zeros_like(points)
            for i, (x, y) in enumerate(points):
                lx, ly = self.fallback_transform(x, y, self._frame_w, self._frame_h)
                result[i] = [lx, ly]

        # Clamp
        result = np.clip(
            result,
            _s.LASERCUBE_COORD_MIN,
            _s.LASERCUBE_COORD_MAX,
        )
        return result.astype(np.int32)

    def fallback_transform(
        self,
        cx: float,
        cy: float,
        frame_w: int,
        frame_h: int,
    ) -> tuple[int, int]:
        """
        Simple linear fallback when homography is not calibrated.

        Maps camera pixels linearly to laser coordinate range, inverting Y.

        Args:
            cx:      Camera X pixel.
            cy:      Camera Y pixel.
            frame_w: Frame width in pixels.
            frame_h: Frame height in pixels.

        Returns:
            (laser_x, laser_y) clamped to laser coordinate range.
        """
        coord_range = _s.LASERCUBE_COORD_MAX - _s.LASERCUBE_COORD_MIN
        if frame_w == 0 or frame_h == 0:
            return 0, 0

        lx = int(_s.LASERCUBE_COORD_MIN + (cx / frame_w) * coord_range)
        # Invert Y: camera top = laser coord max
        ly = int(_s.LASERCUBE_COORD_MAX - (cy / frame_h) * coord_range)

        lx = max(_s.LASERCUBE_COORD_MIN, min(_s.LASERCUBE_COORD_MAX, lx))
        ly = max(_s.LASERCUBE_COORD_MIN, min(_s.LASERCUBE_COORD_MAX, ly))
        return lx, ly

    # ──────────────────────────────────────────────────────────────────────────
    # Persistence
    # ──────────────────────────────────────────────────────────────────────────

    def save_calibration(self, path: Path | None = None) -> bool:
        """
        Save the current homography matrix to a JSON file.

        Args:
            path: File path to write.  Defaults to CALIBRATION_FILE.

        Returns:
            True if saved successfully.
        """
        if self._H is None:
            logger.warning("No calibration to save.")
            return False
        path = path or _s.CALIBRATION_FILE
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            data = {
                "homography": self._H.tolist(),
                "frame_w": self._frame_w,
                "frame_h": self._frame_h,
            }
            with path.open("w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2)
            logger.info("Calibration saved to %s", path)
            return True
        except OSError as exc:
            logger.error("Failed to save calibration: %s", exc)
            return False

    def load_calibration(self, path: Path | None = None) -> bool:
        """
        Load a homography matrix from a JSON calibration file.

        Args:
            path: File path to read.  Defaults to CALIBRATION_FILE.

        Returns:
            True if loaded successfully.
        """
        path = path or _s.CALIBRATION_FILE
        if not path.exists():
            logger.debug("Calibration file not found: %s", path)
            return False
        try:
            with path.open("r", encoding="utf-8") as fh:
                data = json.load(fh)
            H = np.array(data["homography"], dtype=np.float64)
            if H.shape != (3, 3):
                logger.error("Invalid homography shape in %s: %s", path, H.shape)
                return False
            self._H = H
            self._calibrated = True
            self._frame_w = data.get("frame_w", self._frame_w)
            self._frame_h = data.get("frame_h", self._frame_h)
            logger.info("Calibration loaded from %s", path)
            return True
        except (OSError, KeyError, ValueError) as exc:
            logger.error("Failed to load calibration: %s", exc)
            return False

    # ──────────────────────────────────────────────────────────────────────────
    # Private helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _apply_homography(self, cx: float, cy: float) -> tuple[int, int]:
        """
        Apply the stored homography matrix to one point.

        Args:
            cx, cy: Source point in camera pixel space.

        Returns:
            (laser_x, laser_y) as integers.
        """
        pt = np.array([[[cx, cy]]], dtype=np.float32)
        out = cv2.perspectiveTransform(pt, self._H)
        lx = int(round(out[0, 0, 0]))
        ly = int(round(out[0, 0, 1]))
        lx = max(_s.LASERCUBE_COORD_MIN, min(_s.LASERCUBE_COORD_MAX, lx))
        ly = max(_s.LASERCUBE_COORD_MIN, min(_s.LASERCUBE_COORD_MAX, ly))
        return lx, ly

    @staticmethod
    def _reprojection_error(
        src: np.ndarray,
        dst: np.ndarray,
        H: np.ndarray,
    ) -> float:
        """
        Compute mean reprojection error for a set of point pairs.

        Args:
            src: Source points (N, 2) float32.
            dst: Destination points (N, 2) float32.
            H:   3×3 homography matrix.

        Returns:
            Mean Euclidean reprojection error.
        """
        src_h = src.reshape(-1, 1, 2)
        projected = cv2.perspectiveTransform(src_h, H).reshape(-1, 2)
        diff = projected - dst
        return float(np.mean(np.sqrt((diff ** 2).sum(axis=1))))
