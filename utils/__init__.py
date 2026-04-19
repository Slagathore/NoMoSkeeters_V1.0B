"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Utils Package
-------------
Shared utility modules used across the entire application.

Modules in this package:
  logging_utils.py  — Application-wide logging initialisation.
  safety.py         — Laser safety system (arm/disarm, zones, e-stop).

Classes (re-exported):
  SafetySystem      — Thread-safe laser safety gatekeeper.

#todo Add a metrics/telemetry utility for in-app performance profiling.
"""

from .safety import SafetySystem

__all__ = ["SafetySystem"]
