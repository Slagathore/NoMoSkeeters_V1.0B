"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Laser Manager Module
--------------------
High-level QThread-based laser controller that integrates:
  - LaserCubeInterface (hardware driver)
  - SafetySystem (arm/disarm, e-stop, no-fire zones)
  - Targeting dwell-time enforcement
  - Idle blanking

The LaserManager runs in its own thread and accepts targeting jobs via
push_target().  It validates each target against the SafetySystem, converts
laser coordinates (already in LaserCube space) to a frame and sends it.

Firing is only possible when ALL of these conditions are true:
  1. LaserCubeInterface is connected.
  2. SafetySystem.is_armed() returns True.
  3. SafetySystem.check_point(camera_x, camera_y) returns True.
  4. The target point has not exceeded LASER_MAX_DWELL_MS.

After LASER_MAX_DWELL_MS milliseconds on a single target, the manager
automatically stops firing until a new target is received.

Signals:
  laser_fired(int, int)    — Emitted when a frame is sent: (laser_x, laser_y).
  output_changed(bool)    — Emitted when laser output is enabled/disabled.
  connection_changed(bool)— Emitted when LaserCube connects/disconnects.
  error_occurred(str)     — Emitted on hardware or safety error.

Modules imported:
  queue, threading, time  — worker queue and timing
  PySide6.QtCore          — QThread, Signal
  laser.lasercube_interface — LaserCubeInterface, LaserPoint
  utils.safety            — SafetySystem
  config.settings         — LASER_*, LASERCUBE_* constants
  utils.logging_utils     — get_logger

Classes:
  TargetJob              — NamedTuple for queued targeting requests.
  LaserManager           — QThread laser controller.

Methods (LaserManager):
  set_laser_interface(interface)  — Provide the LaserCubeInterface to use.
  set_safety_system(safety)       — Provide the SafetySystem to use.
  connect_laser()                 — Open LaserCube connection and enable output.
  disconnect_laser()              — Disable output and close connection.
  push_target(lx, ly, cam_x, cam_y) — Queue a new targeting command.
  stop()                          — Stop the worker thread gracefully.
  arm()                           — Delegate arm to SafetySystem.
  disarm()                        — Delegate disarm to SafetySystem.
  emergency_stop()                — Delegate e-stop to SafetySystem + blank laser.
  send_test_point(lx, ly)         — Send a single test point (requires arm).

Variables (instance):
  _interface     (LaserCubeInterface | None): Hardware driver.
  _safety        (SafetySystem | None):       Safety gatekeeper.
  _target_queue  (queue.Queue[TargetJob]):    Incoming targeting requests.
  _running       (bool):                      Loop control flag.
  _current_target(TargetJob | None):          Active target being fired at.
  _target_start  (float | None):              monotonic time target was acquired.

#todo Add scan-pattern idle animations (e.g. raster scan, lissajous) when disarmed.
#todo Add a test/calibration mode that fires a grid of known points.
#todo Log all fire events (timestamp, camera coords, laser coords) to CSV audit log.
#todo Add multi-target prioritisation (queue multiple active tracks, cycle between them).
"""

from __future__ import annotations

import queue
import time
from typing import NamedTuple, Optional

from PySide6.QtCore import QThread, Signal

import config.settings as _s
from laser.lasercube_interface import LaserCubeInterface, LaserPoint
from utils.logging_utils import get_logger
from utils.safety import SafetySystem

logger = get_logger(__name__)


class TargetJob(NamedTuple):
    """
    Queued laser targeting request.

    Attributes:
        laser_x (int):   Target X in LaserCube coordinate space (–32767 to +32767).
        laser_y (int):   Target Y in LaserCube coordinate space.
        cam_x   (float): Target X in camera pixel space (used for safety zone check).
        cam_y   (float): Target Y in camera pixel space.
        r       (int):   Red channel 0–255.
        g       (int):   Green channel 0–255.
        b       (int):   Blue channel 0–255.
    """
    laser_x: int
    laser_y: int
    cam_x: float
    cam_y: float
    r: int = _s.LASER_DEFAULT_POWER_R
    g: int = _s.LASER_DEFAULT_POWER_G
    b: int = _s.LASER_DEFAULT_POWER_B


class LaserManager(QThread):
    """
    High-level QThread that manages laser targeting with safety enforcement.

    Consumes TargetJob objects from an internal queue.  For each job:
      1. Checks SafetySystem.check_point(cam_x, cam_y).
      2. Sends a targeting frame to LaserCubeInterface.
      3. Monitors dwell time; blanks if LASER_MAX_DWELL_MS is exceeded.

    Signals:
        laser_fired     (int, int): Emitted after each firing frame (lx, ly).
        output_changed  (bool):     Laser output enable state changed.
        connection_changed(bool):   LaserCube hardware connected state changed.
        error_occurred  (str):      Error description string.
    """

    laser_fired = Signal(int, int)
    output_changed = Signal(bool)
    connection_changed = Signal(bool)
    error_occurred = Signal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._interface: LaserCubeInterface | None = None
        self._safety: SafetySystem | None = None
        self._target_queue: queue.Queue[TargetJob] = queue.Queue(maxsize=8)
        self._running: bool = False
        self._current_target: TargetJob | None = None
        self._target_start: float | None = None

    # ──────────────────────────────────────────────────────────────────────────
    # Configuration
    # ──────────────────────────────────────────────────────────────────────────

    def set_laser_interface(self, interface: LaserCubeInterface) -> None:
        """
        Provide the LaserCubeInterface hardware driver to use.

        Args:
            interface: LaserCubeInterface instance (not yet connected).
        """
        self._interface = interface

    def set_safety_system(self, safety: SafetySystem) -> None:
        """
        Provide the SafetySystem gatekeeper.

        Args:
            safety: SafetySystem instance shared with the GUI.
        """
        self._safety = safety

    # ──────────────────────────────────────────────────────────────────────────
    # Connection
    # ──────────────────────────────────────────────────────────────────────────

    def connect_laser(self) -> bool:
        """
        Open the LaserCube connection.

        Returns:
            True if connected, False on failure.
        """
        if self._interface is None:
            logger.error("connect_laser called but no LaserCubeInterface set.")
            return False

        ok = self._interface.connect()
        self.connection_changed.emit(ok)
        if ok:
            # Set scan rate
            self._interface.set_scan_rate(_s.LASERCUBE_SCAN_RATE)
            # Send initial blank frame
            self._interface.send_blank_frame()
        return ok

    def disconnect_laser(self) -> None:
        """Blank and disconnect the LaserCube."""
        if self._interface:
            self._interface.send_blank_frame()
            self._interface.disconnect()
        self.connection_changed.emit(False)
        self.output_changed.emit(False)

    # ──────────────────────────────────────────────────────────────────────────
    # Safety delegation
    # ──────────────────────────────────────────────────────────────────────────

    def arm(self) -> bool:
        """
        Attempt to arm the laser via the SafetySystem.

        Returns:
            True if successfully armed.
        """
        if self._safety is None:
            logger.error("arm() called but no SafetySystem set.")
            return False
        result = self._safety.arm()
        if result and self._interface:
            self._interface.enable_output(True)
            self.output_changed.emit(True)
        return result

    def disarm(self) -> None:
        """Disarm via SafetySystem and blank the laser output."""
        if self._safety:
            self._safety.disarm()
        self._blank_and_disable()

    def emergency_stop(self) -> None:
        """
        Trigger emergency stop: immediately blank the laser and engage e-stop.
        """
        logger.critical("LaserManager: EMERGENCY STOP triggered.")
        self._blank_and_disable()
        if self._safety:
            self._safety.emergency_stop()

    # ──────────────────────────────────────────────────────────────────────────
    # Targeting
    # ──────────────────────────────────────────────────────────────────────────

    def push_target(
        self,
        laser_x: int,
        laser_y: int,
        cam_x: float,
        cam_y: float,
        r: int = _s.LASER_DEFAULT_POWER_R,
        g: int = _s.LASER_DEFAULT_POWER_G,
        b: int = _s.LASER_DEFAULT_POWER_B,
    ) -> None:
        """
        Queue a laser targeting command.

        Drops the command (with a debug log) if the queue is full.

        Args:
            laser_x: X in LaserCube space.
            laser_y: Y in LaserCube space.
            cam_x:   X in camera pixel space (for safety zone check).
            cam_y:   Y in camera pixel space.
            r, g, b: RGB colour channels.
        """
        job = TargetJob(laser_x, laser_y, cam_x, cam_y, r, g, b)
        try:
            self._target_queue.put_nowait(job)
        except queue.Full:
            logger.debug("Laser target queue full — command dropped.")

    def send_test_point(self, lx: int, ly: int) -> bool:
        """
        Send a single test/calibration point directly (bypasses normal queue).

        The laser MUST be armed for this to fire.  Used by CalibrationController.

        Args:
            lx: Laser X coordinate (–32767 … 32767).
            ly: Laser Y coordinate (–32767 … 32767).

        Returns:
            True if the frame was sent to hardware.
        """
        if not self._check_safety(lx, ly, lx, ly):
            return False
        if self._interface is None:
            return False
        frame = self._interface.build_single_point_frame(
            lx, ly,
            _s.CALIBRATION_LASER_R,
            _s.CALIBRATION_LASER_G,
            _s.CALIBRATION_LASER_B,
        )
        return self._interface.send_frame(frame)

    # ──────────────────────────────────────────────────────────────────────────
    # QThread main loop
    # ──────────────────────────────────────────────────────────────────────────

    def run(self) -> None:
        """
        Main laser manager loop.

        Blocks on the target queue.  For each received job:
          - Validates safety conditions.
          - Sends a targeting frame or blank frame.
          - Enforces the max dwell time.
        """
        self._running = True
        logger.info("LaserManager thread started.")

        while self._running:
            try:
                job = self._target_queue.get(timeout=0.05)
            except queue.Empty:
                # No new target — maintain current or blank
                self._maintain_or_blank()
                continue

            if not self._check_safety(
                job.laser_x, job.laser_y, job.cam_x, job.cam_y
            ):
                self._send_blank()
                self._current_target = None
                self._target_start = None
                continue

            # New valid target
            if self._current_target is None or (
                job.laser_x != self._current_target.laser_x
                or job.laser_y != self._current_target.laser_y
            ):
                self._current_target = job
                self._target_start = time.monotonic()

            # Check dwell limit
            elapsed_ms = (time.monotonic() - self._target_start) * 1000.0
            if elapsed_ms > _s.LASER_MAX_DWELL_MS:
                logger.debug(
                    "Dwell limit reached (%.0fms) — blanking.", elapsed_ms
                )
                self._send_blank()
                self._current_target = None
                self._target_start = None
                continue

            # Fire at target
            self._fire_at(job)

        # Thread stopping — blank and disable
        self._blank_and_disable()
        logger.info("LaserManager thread stopped.")

    def stop(self) -> None:
        """Signal the laser loop to stop and wait for the thread to exit."""
        self._running = False
        self.wait(3000)

    # ──────────────────────────────────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _check_safety(
        self,
        laser_x: int,
        laser_y: int,
        cam_x: float,
        cam_y: float,
    ) -> bool:
        """
        Validate all safety conditions before firing.

        Args:
            laser_x, laser_y: LaserCube coordinates (not used for zone check).
            cam_x, cam_y:     Camera pixel coordinates (used for zone check).

        Returns:
            True if all safety checks pass.
        """
        if self._interface is None or not self._interface.is_connected():
            return False
        if self._safety is None:
            return False
        return self._safety.check_point(cam_x, cam_y)

    def _fire_at(self, job: TargetJob) -> None:
        """
        Send a targeting frame for the given job.

        Args:
            job: TargetJob with laser coordinates and colour.
        """
        if self._interface is None:
            return
        frame = self._interface.build_single_point_frame(
            job.laser_x,
            job.laser_y,
            job.r,
            job.g,
            job.b,
            repeat=_s.LASER_BURST_REPEAT,
        )
        if self._interface.send_frame(frame):
            self.laser_fired.emit(job.laser_x, job.laser_y)

    def _send_blank(self) -> None:
        """Send a blank (dark) frame to the laser."""
        if self._interface and self._interface.is_connected():
            self._interface.send_blank_frame()

    def _blank_and_disable(self) -> None:
        """Send a blank frame and disable output via SET_OUTPUT(0)."""
        self._send_blank()
        if self._interface and self._interface.is_connected():
            self._interface.enable_output(False)
        self.output_changed.emit(False)

    def _maintain_or_blank(self) -> None:
        """
        When no new target is available, either re-fire the current target
        (if within dwell limit) or send a blank to keep the hardware happy.
        """
        if self._current_target is None or self._target_start is None:
            self._send_blank()
            return

        elapsed_ms = (time.monotonic() - self._target_start) * 1000.0
        if elapsed_ms > _s.LASER_MAX_DWELL_MS:
            self._send_blank()
            self._current_target = None
            self._target_start = None
        else:
            # Re-fire to maintain dwell lock
            self._fire_at(self._current_target)
