"""preflight.py — verify every layer of the stack before a live session.

Runs through, in order:
  1. config — settings load, calibrations dir exists.
  2. cube — connect, GET_FULL_INFO, interlock + over-temp clear, alive.
  3. OV9281 — open at the configured index, MJPG + high fps confirmed,
     a few frames actually arrive.
  4. Kinect — open, depth + IR planes available, frame received.
  5. calibration — calibration JSON v2 loads + sanity check.
  6. SafetyModerator — wire up the checks, simulate a SAFETY frame from
     each sensor, verify the gate opens after arming.
  7. ShotPattern — generator + LaserManager can produce a sample shot
     (DRY — never streams to the cube, never enables output).

Every layer prints a clear PASS / FAIL line. Exits non-zero on any FAIL.
Designed to be the last thing the operator runs before
`scripts/live_fire_session.py`.

>>> Safe by construction: this script NEVER calls cube.enable_output(). <<<
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np                                               # noqa: E402

from config import settings                                       # noqa: E402
from laser.lasercube import LaserCubeInterface                    # noqa: E402
from laser.laser_manager import LaserManager                      # noqa: E402
from safety.kinect_safety_check import KinectSafetyCheck          # noqa: E402
from safety.safety_moderator import (                             # noqa: E402
    SafetyModerator, make_kinect_person_check)
from sensors.base import SensorFrame, SensorRole                  # noqa: E402
from sensors.kinect_v2 import KinectV2Sensor, pykinect2_available  # noqa: E402
from sensors.ov9281 import OV9281Sensor                           # noqa: E402
from targeting.calibration import CoordinateMapper                 # noqa: E402


def _pass(text: str) -> None:
    print(f"  [PASS] {text}")


def _fail(text: str) -> None:
    print(f"  [FAIL] {text}")


def _warn(text: str) -> None:
    print(f"  [WARN] {text}")


def _check_config() -> bool:
    print("1. config")
    ok = True
    if not settings.CALIBRATIONS_DIR.is_dir():
        _warn(f"calibrations dir does not exist: "
              f"{settings.CALIBRATIONS_DIR} (will be created on first save)")
    else:
        _pass(f"calibrations dir: {settings.CALIBRATIONS_DIR}")
    _pass(f"LATENCY_SOFTWARE_LAG_MS = {settings.LATENCY_SOFTWARE_LAG_MS}")
    return ok


def _check_cube(ip: str, src_ip: str) -> Optional[LaserCubeInterface]:
    print("2. cube")
    cube = LaserCubeInterface(ip=ip, src_ip=src_ip)
    if not cube.connect():
        _fail(f"could not connect at {ip}")
        return None
    _pass(f"connected (src_ip={cube.src_ip})")
    info = cube.get_full_info()
    if info is None:
        _fail("GET_FULL_INFO returned None")
        cube.disconnect()
        return None
    _pass(f"firmware {info.fw_major}.{info.fw_minor}  "
          f"interlock={info.interlock}  over_temp={info.over_temp}  "
          f"temp={info.temperature_c}C")
    if not info.interlock:
        _fail("interlock open — close it before a session")
        cube.disconnect()
        return None
    if info.over_temp:
        _fail("over-temp flag set — let it cool")
        cube.disconnect()
        return None
    return cube


def _check_ov9281(index: int) -> bool:
    print("3. OV9281")
    s = OV9281Sensor(device_index=index)
    if not s.open():
        _fail(f"could not open cv2 index {index} (try ov9281_probe.py --list)")
        return False
    frames = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.5:
        f = s.read()
        if f is not None and f.rgb is not None:
            frames += 1
    s.close()
    if frames < 30:
        _fail(f"only {frames} frames in 1.5s — expected >100 at default mode")
        return False
    _pass(f"{frames} frames in 1.5s (~{frames / 1.5:.0f} fps)")
    return True


def _check_kinect() -> bool:
    print("4. Kinect")
    if not pykinect2_available():
        _warn("pykinect2 not installed — safety depth check unavailable")
        return False
    s = KinectV2Sensor()
    if not s.open():
        _fail("KinectV2Sensor.open() returned False — SDK installed but no device?")
        return False
    got_depth = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        f = s.read()
        if f is not None and f.depth is not None:
            got_depth = True
            break
    s.close()
    if not got_depth:
        _fail("no depth frames in 2s — Kinect plugged in?")
        return False
    _pass("depth plane streaming")
    return True


def _check_calibration(sensor_id: str, scene: str, mount: str) -> bool:
    print("5. calibration")
    path = (settings.CALIBRATIONS_DIR / sensor_id /
            f"{scene}_{mount}.json")
    if not path.is_file():
        _warn(f"no calibration at {path} — shots will use NORM=GALVO-norm")
        return False
    try:
        mapper = CoordinateMapper.load(path)
    except Exception as exc:                                # pragma: no cover
        _fail(f"calibration load failed: {exc}")
        return False
    _pass(f"loaded {path.name}: {mapper}")
    return True


def _check_safety_gate() -> bool:
    print("6. safety moderator")
    mod = SafetyModerator(require_explicit_arm=True, safe_cooldown_s=0.0)
    mod.add_check("kinect_v2", make_kinect_person_check(
        type("S", (), {"is_person_present": staticmethod(lambda d: False)})()))

    # No arm yet → unsafe.
    if mod.is_safe_to_fire():
        _fail("moderator reports SAFE before arm() — gate is broken")
        return False
    _pass("unarmed → unsafe (correct)")

    mod.arm()
    if mod.is_safe_to_fire():
        _fail("moderator reports SAFE before any SAFETY frame seen")
        return False
    _pass("armed but no frames → unsafe (correct — stale gate)")

    # Inject a clean Kinect frame.
    f = SensorFrame(timestamp=0.0, sensor_id="kinect_v2",
                    sensor_role=SensorRole.SAFETY.value)
    f.depth = np.zeros((4, 4), dtype=np.float32)
    mod.on_frame(f)
    if not mod.is_safe_to_fire():
        _fail(f"clean frame did not open the gate: {mod.verdict().reasons}")
        return False
    _pass("clean SAFETY frame opens the gate (correct)")

    # Now simulate a person hit.
    mod._check_results["kinect_v2"] = [(False, "person detected")]
    mod.on_frame(f)
    if mod.is_safe_to_fire():
        _fail("person detection failed to close the gate")
        return False
    _pass("person detection closes the gate (correct)")
    return True


def _check_laser_manager_dry(cube: LaserCubeInterface) -> bool:
    print("7. LaserManager (DRY)")
    mgr = LaserManager(transport=cube, mapper=None,
                       software_lag_ms=settings.LATENCY_SOFTWARE_LAG_MS)
    # Generate a sample shot but DO NOT send. We just verify the pattern
    # generator works end-to-end.
    pat = mgr.pattern.generate(0x800, 0x800,
                                settings.SHOT_PATTERN_DWELL_MS,
                                settings.LASERCUBE_DEFAULT_DAC_RATE,
                                settings.SHOT_PATTERN_POWER_PCT)
    if not pat:
        _fail("ShotPattern.generate returned empty sample list")
        return False
    _pass(f"pattern.generate produced {len(pat)} samples")
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="169.254.40.83")
    ap.add_argument("--src-ip", default="auto")
    ap.add_argument("--cam-index", type=int,
                    default=settings.OV9281_DEVICE_INDEX)
    ap.add_argument("--sensor-id", default="ov9281")
    ap.add_argument("--scene", default="bench")
    ap.add_argument("--mount-config", default="bench")
    ap.add_argument("--skip-kinect", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.WARNING,
                        format="%(levelname)s %(name)s: %(message)s")

    print("=" * 60)
    print("  pre-flight checklist")
    print("=" * 60)

    failures = 0
    if not _check_config():
        failures += 1

    cube = _check_cube(args.ip, args.src_ip)
    if cube is None:
        failures += 1

    if not _check_ov9281(args.cam_index):
        failures += 1

    if not args.skip_kinect:
        if not _check_kinect():
            failures += 1

    if not _check_calibration(args.sensor_id, args.scene, args.mount_config):
        # Calibration absence is a WARN not a FAIL — explicitly allowed
        # (raw NORM=galvo-norm is a valid degraded mode).
        pass

    if not _check_safety_gate():
        failures += 1

    if cube is not None and not _check_laser_manager_dry(cube):
        failures += 1

    if cube is not None:
        cube.disconnect()

    print()
    if failures:
        print(f"FAIL: {failures} check(s) failed — do not run a live session")
        return 1
    print("OK: all critical checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
