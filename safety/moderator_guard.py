"""ModeratorGuard — drop-in safety moderator for bench scripts.

The live-fire orchestrator wires its own moderator; the bench scripts
(cone_live, step11_first_light, step11_calibration) used to run with no
software backstop at all — one-shot preflight + a typed "yes", then a
multi-second pattern with nothing watching the room. This guard packages
the same doctrine into one object:

  - Kinect depth person check (opens its own KinectV2Sensor),
  - cube interlock/over-temp via CubeHeartbeat,
  - a SafetyModerator that OWNS enable_output/disable_output,
  - a background pump thread so the moderator stays fresh while the
    script blocks in its pattern loop.

Usage::

    guard = open_moderator_guard(cube)      # None → abort (guidance printed)
    guard.start()
    guard.moderator.arm()
    if not guard.wait_safe():
        ...abort...
    # moderator has called enable_output; pattern may stream.
    ...
    guard.stop()                            # disarm → disable_output

If a person walks into the Kinect's view mid-pattern the moderator drops
the gate — the script keeps streaming samples, but no photons leave.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

from config import settings
from laser.heartbeat import CubeHeartbeat
from safety.kinect_safety_check import KinectSafetyCheck
from safety.safety_moderator import (SafetyModerator, make_cube_health_check,
                                     make_kinect_person_check)
from sensors.base import SensorFrame, SensorRole
from sensors.kinect_v2 import KinectV2Sensor, pykinect2_available


class ModeratorGuard:
    """Kinect person check + cube heartbeat feeding a SafetyModerator that
    owns the cube's output gate for the duration of a bench pattern."""

    def __init__(self, cube, kinect: KinectV2Sensor) -> None:
        self._kinect = kinect
        self.moderator = SafetyModerator(
            stale_after_s=settings.SAFETY_MODERATOR_STALE_AFTER_S,
            safe_cooldown_s=settings.SAFETY_MODERATOR_COOLDOWN_S,
            require_explicit_arm=True,
        )
        self.moderator.set_output_callbacks(
            enable=cube.enable_output, disable=cube.disable_output)
        self.moderator.add_check("kinect_v2", make_kinect_person_check(
            KinectSafetyCheck(
                min_depth_m=settings.SAFETY_KINECT_DEPTH_MIN_M,
                max_depth_m=settings.SAFETY_KINECT_DEPTH_MAX_M,
                min_height_m=settings.SAFETY_KINECT_PERSON_MIN_HEIGHT_M,
                max_height_m=settings.SAFETY_KINECT_PERSON_MAX_HEIGHT_M)))

        def _on_cube_status(_info) -> None:
            tick = SensorFrame(timestamp=time.time(), sensor_id="lasercube",
                               sensor_role=SensorRole.SAFETY.value)
            self.moderator.on_frame(tick)

        self._heartbeat = CubeHeartbeat(cube, on_status=_on_cube_status)
        # Two missed 1.5s beats close the gate.
        self.moderator.add_check("lasercube",
                                 make_cube_health_check(self._heartbeat),
                                 stale_after_s=3.5)

        self._stop = threading.Event()
        self._pump = threading.Thread(target=self._pump_kinect,
                                      name="GuardKinectPump", daemon=True)

    def start(self) -> None:
        self._heartbeat.start()
        self._pump.start()

    def wait_safe(self, timeout_s: float = 8.0) -> bool:
        """Block until the gate opens (first Kinect frames + first
        heartbeat + cooldown) or the timeout passes."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if self.moderator.verdict().safe:
                return True
            time.sleep(0.1)
        return False

    def stop(self) -> None:
        self._stop.set()
        self.moderator.disarm()
        self._heartbeat.stop()
        if self._pump.is_alive():
            self._pump.join(timeout=2.0)
        self._kinect.close()

    def _pump_kinect(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._kinect.read()
            except Exception:
                frame = None            # staleness closes the gate
            if frame is not None and frame.depth is not None:
                self.moderator.on_frame(frame)
            time.sleep(0.03)


def open_moderator_guard(cube) -> Optional[ModeratorGuard]:
    """Open the Kinect and build a guard. None → caller must abort
    (guidance already printed)."""
    if not pykinect2_available():
        print("FAIL: moderated run requires the Kinect (pykinect2/SDK not "
              "available). Re-run with --no-moderator to accept a bench "
              "session with no software safety backstop.")
        return None
    kinect = KinectV2Sensor()
    if not kinect.open():
        print("FAIL: Kinect did not open. Re-run with --no-moderator to "
              "accept a bench session with no software safety backstop.")
        return None
    print(f"sensor: {kinect.sensor_id} opened (safety)")
    return ModeratorGuard(cube, kinect)
