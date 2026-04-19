"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Goals:
  - Provide an intuitive GUI for monitoring, control, and calibration.
  - Use computer vision (background subtraction + optional YOLO) to detect mosquitos.
  - Map camera pixel coordinates to laser angles via perspective homography.
  - Fire the LaserCube autonomously at detected mosquito positions with safety interlocks.

How it achieves this:
  1. GoPro streams live video over WiFi (Open GoPro API + RTSP/UDP capture).
  2. Frames are processed each tick by the detector to find mosquito-sized moving objects.
  3. Detected objects are tracked across frames using Kalman filters.
  4. A calibrated homography matrix transforms camera pixel coords → laser coords.
  5. Laser coordinates are sent to the LaserCube over UDP for targeting.
  6. All firing is gated behind a safety system (arm/disarm, no-fire zones, e-stop).

Config Package
--------------
Exports project-wide settings constants and the ConfigManager class for
persistent user-defined overrides stored as JSON on disk.

Modules in this package:
  settings.py       — All global constants and default configuration values.
  config_manager.py — Reads/writes user config overrides to/from a JSON file.

Classes:
  ConfigManager     — Loads, merges, and saves user config.

Key Variables / Constants (from settings.py):
  PROJECT_NAME           — Human-readable application name.
  VERSION                — Semantic version string.
  BASE_DIR               — Absolute path to the project root directory.
  GOPRO_DEFAULT_IP       — Default GoPro WiFi AP IP address.
  LASERCUBE_DEFAULT_IP   — Default LaserCube WiFi AP IP address.
  ... (see settings.py for full list)
"""

from .config_manager import ConfigManager
from .settings import *  # noqa: F401, F403  — all constants exposed at package level

__all__ = ["ConfigManager"]
