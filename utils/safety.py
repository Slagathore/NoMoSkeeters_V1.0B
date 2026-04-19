"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Safety System Module
--------------------
⚠️  CRITICAL SAFETY MODULE ⚠️

Lasers can cause permanent eye damage, fire, and other hazards.
This module is the gatekeeper between the rest of the application and
the LaserCube hardware.  The laser CANNOT fire unless ALL safety checks pass.

Safety levels enforced:
  1. Arm/Disarm state  — User must explicitly press "Arm" in the GUI.
  2. Emergency Stop    — Instantly disarms; cannot be re-armed without reset.
  3. No-Fire Zones     — Axis-aligned rectangles in camera pixel space where
                         the laser is NEVER allowed to point.
  4. Dwell time limit  — Enforced externally by LaserManager using settings.

Thread safety: all state changes are protected by a reentrant lock so this
module is safe to use from both the GUI thread and the laser worker thread.

Modules imported:
  threading          — RLock for thread-safe state
  logging            — event logging
  config.settings    — LASER_SAFE_MODE, LASER_ARM_REQUIRED
  utils.logging_utils — get_logger

Classes:
  NoFireZone         — Named tuple describing a rectangular exclusion region.
  SafetySystem       — Singleton-like safety gatekeeper.

Functions (SafetySystem):
  arm()              — Move from DISARMED → ARMED state (if no e-stop active).
  disarm()           — Move from any OK state → DISARMED.
  emergency_stop()   — Immediately halt all laser activity; latch e-stop flag.
  reset_estop()      — Clear the e-stop latch (does NOT re-arm; user must arm).
  is_armed()         — True if laser is currently authorised to fire.
  is_estop_active()  — True if emergency stop has been triggered.
  add_no_fire_zone(zone) — Register a new no-fire zone.
  remove_no_fire_zone(zone_id) — Remove a no-fire zone by its ID.
  clear_no_fire_zones()   — Remove all registered no-fire zones.
  check_point(x, y)  — Return True if laser MAY fire at camera pixel (x, y).
  get_zones()        — Return list of currently registered no-fire zones.
  status_dict()      — Return dict with full current safety state for GUI.

Variables (instance):
  _armed             — bool: current arm state.
  _estop             — bool: latched emergency-stop flag.
  _zones             — list[NoFireZone]: registered exclusion rectangles.
  _lock              — threading.RLock for thread safety.
  _callbacks         — list of callables notified on state change.

#todo Add a physical hardware interlock input (GPIO or USB HID) for a kill-switch.
#todo Log all firing events (timestamp, camera coord, laser coord) to audit CSV.
#todo Add configurable "safe zones" (whitelist) as an alternative to no-fire zones.
#todo Implement a pre-flight checklist dialog that must be completed before arming.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable

from utils.logging_utils import get_logger

logger = get_logger(__name__)


@dataclass
class NoFireZone:
    """
    Axis-aligned rectangular region in camera pixel space where the laser
    must never be directed.

    Attributes:
        zone_id (int):  Unique identifier assigned by SafetySystem.
        x1 (int):       Left pixel coordinate (inclusive).
        y1 (int):       Top pixel coordinate (inclusive).
        x2 (int):       Right pixel coordinate (inclusive).
        y2 (int):       Bottom pixel coordinate (inclusive).
        label (str):    Human-readable description (e.g., "Person sitting area").
    """
    zone_id: int
    x1: int
    y1: int
    x2: int
    y2: int
    label: str = ""

    def contains(self, x: float, y: float) -> bool:
        """
        Return True if pixel coordinate (x, y) falls within this zone.

        Args:
            x: Pixel column.
            y: Pixel row.

        Returns:
            True if the point is inside (or on the border of) this zone.
        """
        return self.x1 <= x <= self.x2 and self.y1 <= y <= self.y2


class SafetySystem:
    """
    Thread-safe laser safety gatekeeper.

    The laser may only fire when:
      - self._armed is True, AND
      - self._estop is False, AND
      - the target point is not inside any registered NoFireZone.

    All state transitions are logged at WARNING level or above.

    Attributes:
        _armed      (bool):            Whether the system is currently armed.
        _estop      (bool):            Whether emergency stop has been triggered.
        _zones      (list[NoFireZone]):Registered no-fire zones.
        _lock       (threading.RLock): Reentrant lock protecting all state.
        _next_zone_id (int):           Auto-incrementing zone ID counter.
        _callbacks  (list[Callable]):  Listeners called on any state change.
    """

    def __init__(self) -> None:
        self._armed: bool = False
        self._estop: bool = False
        self._zones: list[NoFireZone] = []
        self._lock: threading.RLock = threading.RLock()
        self._next_zone_id: int = 1
        self._callbacks: list[Callable[[], None]] = []
        logger.info("SafetySystem initialised — DISARMED")

    # ──────────────────────────────────────────────────────────────────────────
    # Arm / Disarm
    # ──────────────────────────────────────────────────────────────────────────

    def arm(self) -> bool:
        """
        Attempt to arm the laser system.

        Will fail (return False and log a warning) if the emergency stop latch
        is active.  The caller is responsible for presenting UI feedback.

        Returns:
            True if the system was successfully armed, False otherwise.
        """
        with self._lock:
            if self._estop:
                logger.warning("ARM request denied — emergency stop is active.")
                return False
            self._armed = True
            logger.warning("⚡ Laser system ARMED — targeting enabled.")
            self._notify()
            return True

    def disarm(self) -> None:
        """
        Disarm the laser system.  Safe to call at any time, from any thread.
        Does NOT clear an active emergency stop — use reset_estop() for that.
        """
        with self._lock:
            if self._armed:
                self._armed = False
                logger.warning("Laser system DISARMED.")
                self._notify()

    # ──────────────────────────────────────────────────────────────────────────
    # Emergency Stop
    # ──────────────────────────────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        """
        IMMEDIATELY disarm and latch the emergency stop flag.

        The system CANNOT be re-armed until reset_estop() is called explicitly.
        Designed to be callable from any thread or signal handler.
        """
        with self._lock:
            self._armed = False
            self._estop = True
            logger.critical("🚨 EMERGENCY STOP ACTIVATED — laser output inhibited.")
            self._notify()

    def reset_estop(self) -> None:
        """
        Clear the emergency stop latch.

        Does NOT re-arm the system.  The user must explicitly call arm() again.
        """
        with self._lock:
            self._estop = False
            logger.warning("Emergency stop CLEARED — system ready to arm (not armed).")
            self._notify()

    # ──────────────────────────────────────────────────────────────────────────
    # State queries
    # ──────────────────────────────────────────────────────────────────────────

    def is_armed(self) -> bool:
        """Return True if the system is currently armed and may fire."""
        with self._lock:
            return self._armed and not self._estop

    def is_estop_active(self) -> bool:
        """Return True if the emergency stop latch is currently set."""
        with self._lock:
            return self._estop

    # ──────────────────────────────────────────────────────────────────────────
    # No-Fire Zone management
    # ──────────────────────────────────────────────────────────────────────────

    def add_no_fire_zone(
        self,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        label: str = "",
    ) -> int:
        """
        Register a new no-fire exclusion zone.

        Coordinates are in camera pixel space. (0,0) is top-left.

        Args:
            x1:    Left column (inclusive).
            y1:    Top row (inclusive).
            x2:    Right column (inclusive).
            y2:    Bottom row (inclusive).
            label: Human-readable description.

        Returns:
            Assigned zone_id for later removal.
        """
        with self._lock:
            # Normalise so x1 <= x2 and y1 <= y2
            zone = NoFireZone(
                zone_id=self._next_zone_id,
                x1=min(x1, x2),
                y1=min(y1, y2),
                x2=max(x1, x2),
                y2=max(y1, y2),
                label=label or f"Zone {self._next_zone_id}",
            )
            self._zones.append(zone)
            self._next_zone_id += 1
            logger.info(
                "No-fire zone #%d added: (%d,%d)–(%d,%d) '%s'",
                zone.zone_id,
                zone.x1,
                zone.y1,
                zone.x2,
                zone.y2,
                zone.label,
            )
            self._notify()
            return zone.zone_id

    def remove_no_fire_zone(self, zone_id: int) -> bool:
        """
        Remove the no-fire zone with the given ID.

        Args:
            zone_id: ID returned by add_no_fire_zone().

        Returns:
            True if found and removed, False if zone_id was not registered.
        """
        with self._lock:
            before = len(self._zones)
            self._zones = [z for z in self._zones if z.zone_id != zone_id]
            removed = len(self._zones) < before
            if removed:
                logger.info("No-fire zone #%d removed.", zone_id)
                self._notify()
            return removed

    def clear_no_fire_zones(self) -> None:
        """Remove all registered no-fire zones."""
        with self._lock:
            count = len(self._zones)
            self._zones.clear()
            logger.info("All %d no-fire zones cleared.", count)
            self._notify()

    def get_zones(self) -> list[NoFireZone]:
        """Return a snapshot copy of the current no-fire zone list."""
        with self._lock:
            return list(self._zones)

    # ──────────────────────────────────────────────────────────────────────────
    # Firing authorisation
    # ──────────────────────────────────────────────────────────────────────────

    def check_point(self, x: float, y: float) -> bool:
        """
        Return True if the laser IS authorised to fire at camera pixel (x, y).

        Checks together:
          1. System is armed       (not disarmed, not e-stopped).
          2. Point is NOT inside any registered no-fire zone.

        Args:
            x: Camera pixel column of the target.
            y: Camera pixel row of the target.

        Returns:
            True → proceed with firing.
            False → do NOT fire.
        """
        with self._lock:
            if not self._armed or self._estop:
                return False
            for zone in self._zones:
                if zone.contains(x, y):
                    logger.debug(
                        "Point (%.0f, %.0f) blocked by no-fire zone #%d '%s'.",
                        x,
                        y,
                        zone.zone_id,
                        zone.label,
                    )
                    return False
            return True

    def status_dict(self) -> dict:
        """
        Return a dict summarising the current safety state for GUI display.

        Returns:
            dict with keys: armed, estop, zone_count, zones.
        """
        with self._lock:
            return {
                "armed": self._armed,
                "estop": self._estop,
                "zone_count": len(self._zones),
                "zones": [
                    {
                        "id": z.zone_id,
                        "x1": z.x1,
                        "y1": z.y1,
                        "x2": z.x2,
                        "y2": z.y2,
                        "label": z.label,
                    }
                    for z in self._zones
                ],
            }

    # ──────────────────────────────────────────────────────────────────────────
    # Callback / observer support
    # ──────────────────────────────────────────────────────────────────────────

    def register_callback(self, callback: Callable[[], None]) -> None:
        """
        Register a zero-argument callable to be called on any state change.

        The callback is called while holding the internal lock, so it must NOT
        call back into SafetySystem methods (deadlock risk).

        Args:
            callback: Zero-argument callable.
        """
        with self._lock:
            self._callbacks.append(callback)

    def unregister_callback(self, callback: Callable[[], None]) -> None:
        """Remove a previously registered callback."""
        with self._lock:
            self._callbacks = [c for c in self._callbacks if c is not callback]

    def _notify(self) -> None:
        """Call all registered callbacks.  Must be called while holding _lock."""
        for cb in self._callbacks:
            try:
                cb()
            except Exception as exc:  # noqa: BLE001
                logger.error("Safety callback raised: %s", exc)
