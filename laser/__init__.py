"""
Iron Dome Anti-Mosquito System
================================
Laser Package
-------------
Low-level and high-level control of the LaserCube RGB laser projector.

Modules:
  lasercube_interface.py — UDP protocol implementation for LaserCube hardware.
  laser_manager.py       — QThread-based targeting loop with safety integration.

Classes (re-exported):
  LaserCubeInterface     — Low-level UDP socket wrapper for the LaserCube.
  LaserPoint             — NamedTuple (x, y, r, g, b) for a single laser point.
  LaserManager           — High-level QThread laser controller.

⚠️  SAFETY WARNING:
    This package controls a CLASS 3B/4 laser device.
    Improper use can cause permanent eye damage and fire.
    Never bypass the SafetySystem checks in laser_manager.py.

#todo Add power ramping (soft-start) to avoid laser driver stress.
"""

from .laser_manager import LaserManager
from .lasercube_interface import LaserCubeInterface, LaserPoint

__all__ = ["LaserCubeInterface", "LaserPoint", "LaserManager"]
