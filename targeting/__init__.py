"""
Iron Dome Anti-Mosquito System
================================
Targeting Package
-----------------
Coordinate mapping and calibration between camera pixel space and laser
projector space.

Modules:
  coordinate_mapper.py — Homography-based camera→laser coordinate transform.
  calibration.py       — Interactive calibration controller that computes the
                         homography by firing the laser at known points and
                         finding the resulting dot in the camera image.

Classes (re-exported):
  CoordinateMapper       — Applies a perspective homography matrix.
  CalibrationController  — Orchestrates the calibration procedure.

#todo Add radial distortion correction (camera lens undistortion) before mapping.
#todo Support saving multiple calibration profiles for different room setups.
"""

from .calibration import CalibrationController
from .coordinate_mapper import CoordinateMapper

__all__ = ["CoordinateMapper", "CalibrationController"]
