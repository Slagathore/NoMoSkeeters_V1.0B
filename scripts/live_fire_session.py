"""live_fire_session.py — orchestrated live-fire mosquito session.

Wires together everything Steps 5-12 built:
  - LaserCubeInterface (laser/lasercube.py)
  - OV9281 targeting sensor (sensors/ov9281.py)
  - Kinect SAFETY sensor (sensors/kinect_v2.py)
  - SafetyModerator with Kinect-depth + optional HOG checks
  - Detector + Tracker on the OV9281 feed
  - LaserManager with lead-aim and the configured ShotPattern

The SafetyModerator owns the cube's enable_output gate via the callbacks
wired in main(). LaserManager NEVER calls enable_output (per its
docstring) — it only streams sample data. So every photon emitted is
gated by the moderator's verdict.

>>> Class 3B laser. Eye protection + safety lens REQUIRED. <<<
>>> Run step11_first_light.py FIRST. Run after a clean calibration.   <<<

    python scripts/live_fire_session.py --dry-fire        # safe smoke test
    python scripts/live_fire_session.py --cam-index 2     # real session
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import cv2                                                       # noqa: E402

from config import settings                                       # noqa: E402
from detection.detector import Detector                           # noqa: E402
from laser.laser_manager import LaserManager                      # noqa: E402
from laser.lasercube import LaserCubeInterface                    # noqa: E402
from safety.kinect_safety_check import KinectSafetyCheck          # noqa: E402
from safety.person_detector import HOGPersonDetector              # noqa: E402
from safety.safety_moderator import (                             # noqa: E402
    SafetyModerator, make_kinect_person_check, make_person_detector_check,
)
from safety.voice_warning import VoiceWarning                     # noqa: E402
from sensors.base import SensorFrame, SensorRole                  # noqa: E402
from sensors.kinect_v2 import KinectV2Sensor, pykinect2_available  # noqa: E402
from sensors.ov9281 import OV9281Sensor                           # noqa: E402
from targeting.calibration import CoordinateMapper                 # noqa: E402
from tracking.tracker import Tracker                              # noqa: E402


PER_TRACK_FIRE_COOLDOWN_S = 0.8     # min interval between shots on the
                                     # same track id — guards against
                                     # spam-firing a coasting track.
MAX_SHOT_RATE_HZ = 8.0               # global cap across all tracks.


@dataclass
class SessionCounters:
    detections: int = 0
    confirmed_tracks: int = 0
    shots_fired: int = 0
    shots_blocked_safety: int = 0
    shots_blocked_fov: int = 0
    shots_blocked_cooldown: int = 0


def _open_ov9281(index: int) -> Optional[OV9281Sensor]:
    s = OV9281Sensor(device_index=index)
    return s if s.open() else None


def _open_kinect() -> Optional[KinectV2Sensor]:
    if not pykinect2_available():
        return None
    s = KinectV2Sensor()
    return s if s.open() else None


def _load_mapper(sensor_id: str, scene: str,
                 mount: str) -> Optional[CoordinateMapper]:
    """Find the most recent calibration JSON v2 for the active sensor. The
    convention is `user_data/calibrations/<sensor_id>/<scene>_<mount>.json`."""
    path = (settings.CALIBRATIONS_DIR / sensor_id /
            f"{scene}_{mount}.json")
    if not path.is_file():
        return None
    return CoordinateMapper.load(path)


def _preflight(cube: LaserCubeInterface) -> bool:
    info = cube.get_full_info()
    if info is None:
        print("FAIL: no GET_FULL_INFO response")
        return False
    print(f"  firmware {info.fw_major}.{info.fw_minor}  "
          f"interlock={info.interlock}  over_temp={info.over_temp}  "
          f"temp={info.temperature_c}C")
    if not info.interlock or info.over_temp:
        print("FAIL: pre-flight not safe (interlock/over-temp)")
        return False
    return True


def main(argv=None) -> int:                                  # noqa: C901
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="169.254.40.83")
    ap.add_argument("--src-ip", default="auto")
    ap.add_argument("--cam-index", type=int,
                    default=settings.OV9281_DEVICE_INDEX)
    ap.add_argument("--scene", default="bench")
    ap.add_argument("--mount-config", default="bench")
    ap.add_argument("--sensor-id", default="ov9281",
                    help="sensor_id used to look up the calibration JSON")
    ap.add_argument("--no-kinect", action="store_true")
    ap.add_argument("--no-person-rgb", action="store_true")
    ap.add_argument("--fov-margin", type=float, default=0.05)
    ap.add_argument("--dry-fire", action="store_true",
                    help="don't open the cube at all; print would-fire "
                         "decisions and exit. Use for smoke tests.")
    ap.add_argument("--no-voice", action="store_true",
                    help="disable spoken safety warnings.")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="run for N seconds (0 = until q/ESC).")
    ap.add_argument("--no-window", action="store_true")
    args = ap.parse_args(argv)

    print("=" * 60)
    print("  LIVE FIRE SESSION")
    print("  Class 3B laser. Eye protection + safety lens REQUIRED.")
    print("=" * 60)

    cube: Optional[LaserCubeInterface] = None
    if not args.dry_fire:
        cube = LaserCubeInterface(ip=args.ip, src_ip=args.src_ip)
        if not cube.connect():
            print("FAIL: could not connect to the cube")
            return 1
        print(f"connected (src_ip={cube.src_ip})")
        if not _preflight(cube):
            cube.disconnect()
            return 1
    else:
        print("DRY-FIRE: cube not opened, no photons will leave the box")

    ov = _open_ov9281(args.cam_index)
    if ov is None:
        print(f"FAIL: could not open OV9281 idx {args.cam_index}")
        if cube is not None:
            cube.disconnect()
        return 1
    print(f"sensor: {ov.sensor_id} opened")

    kinect = None if args.no_kinect else _open_kinect()
    if kinect is not None:
        print(f"sensor: {kinect.sensor_id} opened")
    elif not args.no_kinect:
        print("WARN: Kinect not available — depth-based safety check is OFF")

    # Calibration mapper for NORM → GALVO (None = no fit yet; LaserManager
    # treats NORM as galvo-norm directly when mapper is None).
    mapper = _load_mapper(args.sensor_id, args.scene, args.mount_config)
    if mapper is None and not args.dry_fire:
        print(f"WARN: no calibration found at "
              f"{settings.CALIBRATIONS_DIR / args.sensor_id} "
              f"— shots will use raw NORM as GALVO-norm. Calibrate first.")
    elif mapper is not None:
        print(f"calibration: {mapper}")

    moderator = SafetyModerator(
        stale_after_s=settings.SAFETY_MODERATOR_STALE_AFTER_S,
        safe_cooldown_s=settings.SAFETY_MODERATOR_COOLDOWN_S,
        require_explicit_arm=True,
    )
    if cube is not None:
        moderator.set_output_callbacks(
            enable=cube.enable_output,
            disable=cube.disable_output,
        )
    if not args.no_voice and settings.SAFETY_VOICE_WARNINGS_ENABLED:
        moderator.set_voice_warning(VoiceWarning(
            cooldown_s=settings.SAFETY_VOICE_COOLDOWN_S,
            voice_rate=settings.SAFETY_VOICE_RATE,
        ))

    if kinect is not None and settings.SAFETY_KINECT_DEPTH_CHECK_ENABLED:
        moderator.add_check("kinect_v2", make_kinect_person_check(
            KinectSafetyCheck(
                min_depth_m=settings.SAFETY_KINECT_DEPTH_MIN_M,
                max_depth_m=settings.SAFETY_KINECT_DEPTH_MAX_M,
                min_height_m=settings.SAFETY_KINECT_PERSON_MIN_HEIGHT_M,
                max_height_m=settings.SAFETY_KINECT_PERSON_MAX_HEIGHT_M)))
        print("  safety: kinect depth person check ENABLED")

    hog: Optional[HOGPersonDetector] = None
    if not args.no_person_rgb and settings.SAFETY_PERSON_DETECT_ENABLED:
        hog = HOGPersonDetector(
            hit_threshold=settings.SAFETY_PERSON_HOG_HIT_THRESHOLD)
        moderator.add_check("ov9281_safety",
                            make_person_detector_check(hog))
        print("  safety: HOG on OV9281 ENABLED")

    print("\nOperator checklist:")
    print("  - safety lens fitted on aperture")
    print("  - eye protection on for everyone in the room")
    print("  - the room is clear of people who don't have eye protection")
    print("  - target volume is bounded (a wall or backdrop is safe)")
    answer = input("\nProceed to arming? [type 'yes'] ").strip().lower()
    if answer != "yes":
        ov.close()
        if kinect is not None:
            kinect.close()
        if cube is not None:
            cube.disconnect()
        print("session aborted at arm prompt")
        return 0

    moderator.arm()
    print("\nARMED. Press q or ESC in the preview window to exit.\n")

    detector = Detector()
    tracker = Tracker()
    counters = SessionCounters()

    laser_mgr: Optional[LaserManager] = None
    if cube is not None:
        laser_mgr = LaserManager(transport=cube._transport
                                  if hasattr(cube, "_transport") else cube,
                                  mapper=mapper,
                                  software_lag_ms=settings.LATENCY_SOFTWARE_LAG_MS)

    # Kinect frame poller in its own thread — the moderator needs frames
    # at a steady cadence to keep the gate from going stale.
    stop = threading.Event()
    if kinect is not None:
        def _kinect_loop():
            while not stop.is_set():
                f = kinect.read()
                if f is not None:
                    moderator.on_frame(f)
                else:
                    time.sleep(0.002)
        threading.Thread(target=_kinect_loop, daemon=True,
                         name="Spotter-Kinect").start()

    win = "live_fire — q/ESC quit"
    if not args.no_window:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    last_track_fire: dict[int, float] = {}
    last_any_fire = 0.0
    min_fire_interval = 1.0 / MAX_SHOT_RATE_HZ
    t_start = time.monotonic()

    try:
        while True:
            frame = ov.read()
            if frame is None or frame.rgb is None:
                if args.seconds and time.monotonic() - t_start > args.seconds:
                    break
                continue

            if hog is not None:
                synth = SensorFrame(timestamp=frame.timestamp,
                                    sensor_id="ov9281_safety",
                                    sensor_role=SensorRole.SAFETY.value)
                synth.rgb = frame.rgb
                synth.width = frame.width
                synth.height = frame.height
                moderator.on_frame(synth)

            now = time.monotonic()
            dets = detector.detect_bgsub(frame.rgb,
                                         sensor_id=ov.sensor_id,
                                         timestamp=now)
            counters.detections += len(dets)
            tracks = tracker.update(dets, now)

            verdict = moderator.verdict()
            safe = verdict.safe

            for t in tracks:
                if not t.fire_eligible:
                    continue
                counters.confirmed_tracks += 1
                x_n, y_n = float(t.state[0]), float(t.state[1])
                if not (args.fov_margin <= x_n <= 1.0 - args.fov_margin
                        and args.fov_margin <= y_n <= 1.0 - args.fov_margin):
                    counters.shots_blocked_fov += 1
                    continue
                if not safe:
                    counters.shots_blocked_safety += 1
                    continue
                if (now - last_track_fire.get(t.track_id, 0.0)
                        < PER_TRACK_FIRE_COOLDOWN_S):
                    counters.shots_blocked_cooldown += 1
                    continue
                if now - last_any_fire < min_fire_interval:
                    counters.shots_blocked_cooldown += 1
                    continue
                last_track_fire[t.track_id] = now
                last_any_fire = now
                counters.shots_fired += 1
                if laser_mgr is not None:
                    laser_mgr.engage_track(t)
                else:
                    print(f"DRY-FIRE: would shoot track {t.track_id} at "
                          f"({x_n:.3f}, {y_n:.3f})")

            if not args.no_window:
                vis = frame.rgb.copy()
                _draw_session_hud(vis, tracks, counters, verdict,
                                  args.fov_margin)
                cv2.imshow(win, vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break

            if args.seconds and time.monotonic() - t_start > args.seconds:
                break
    finally:
        stop.set()
        # Order matters: disarm before cube.disconnect to drive disable_output.
        moderator.disarm()
        if cube is not None:
            try:
                cube.disable_output()
            except Exception:
                pass
        if not args.no_window:
            cv2.destroyAllWindows()
        ov.close()
        if kinect is not None:
            kinect.close()
        if cube is not None:
            cube.disconnect()

    elapsed = time.monotonic() - t_start
    print("\n" + "-" * 60)
    print(f"  ran {elapsed:.1f}s")
    print(f"  detections          : {counters.detections}")
    print(f"  confirmed-track hits: {counters.confirmed_tracks}")
    print(f"  shots fired         : {counters.shots_fired}")
    print(f"  blocked by safety   : {counters.shots_blocked_safety}")
    print(f"  blocked outside FOV : {counters.shots_blocked_fov}")
    print(f"  blocked by cooldown : {counters.shots_blocked_cooldown}")
    return 0


def _draw_session_hud(img, tracks, counters, verdict, fov_margin) -> None:
    h, w = img.shape[:2]
    x0 = int(fov_margin * w); y0 = int(fov_margin * h)
    x1 = int((1.0 - fov_margin) * w); y1 = int((1.0 - fov_margin) * h)
    cv2.rectangle(img, (x0, y0), (x1, y1), (80, 200, 80), 1)

    for t in tracks:
        x_n, y_n = float(t.state[0]), float(t.state[1])
        cx, cy = int(x_n * (w - 1)), int(y_n * (h - 1))
        col = (0, 255, 0) if t.fire_eligible else (0, 165, 255)
        cv2.circle(img, (cx, cy), 6, col, 1)
        cv2.putText(img, f"t{t.track_id}", (cx + 8, cy - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)

    safe_col = (0, 200, 0) if verdict.safe else (0, 0, 255)
    cv2.rectangle(img, (0, 0), (w, 60), (32, 32, 32), -1)
    cv2.putText(img,
                f"FIRED {counters.shots_fired}   "
                f"DET {counters.detections}   "
                f"TRK {counters.confirmed_tracks}",
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    state = verdict.state.value.upper()
    reasons = "; ".join(verdict.reasons[:2])
    cv2.putText(img, f"SAFETY: {state}   {reasons}",
                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, safe_col, 1)


if __name__ == "__main__":
    raise SystemExit(main())
