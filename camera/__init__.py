"""
Iron Dome Anti-Mosquito System
================================
Camera Package
--------------
Abstractions for camera devices used in the Iron Dome system.

Modules:
  gopro_interface.py  — Low-level GoPro Open API HTTP client + keep-alive.
  camera_manager.py   — QThread-based capture loop; emits frames as Qt signals.

Classes (re-exported):
  GoProInterface    — HTTP wrapper for the GoPro Open GoPro API.
  CameraManager     — QThread that captures and emits video frames.

#todo Add support for USB cameras as a fallback when GoPro is unavailable.
"""

from .camera_manager import CameraManager
from .gopro_interface import GoProInterface

__all__ = ["GoProInterface", "CameraManager"]
