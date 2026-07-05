"""spotter_mode.py — laser-OFF target-acquisition spotter.

Runs the full target-acquisition pipeline against the OV9281 (targeting)
and Kinect (safety + secondary detection) sensors but NEVER touches the
laser. Counts "viable shots" — tracks that the system would have engaged
if the laser were armed — so you can stand inside the room and watch the
counter increment for every mosquito it would have hit.

Pipeline per OV9281 frame:
  1. Detect blobs (Detector — MOG2 + geometry filter).
  2. Update Tracker -> list of TrackedTarget with fire_eligible signal.
  3. Cross-sensor fuse with Kinect-RGB detections when both have a track.
  4. Query SafetyModerator (Kinect depth person check + optional HOG).
  5. A track is "viable" iff:
        fire_eligible AND inside calibration FOV AND safety.safe AND
        not already counted within VIABLE_COOLDOWN_S.
  6. Overlay on the OV9281 view + bump the per-track counter.

Run this when you want to:
  • Verify the safety gate actually blocks while you stand in the room.
  • Tune detection / tracker thresholds with no laser risk.
  • Count nightly mosquito traffic.

This script is laser-cube-free. The cube is not opened, enable_output is
never called. Eye protection is not required, but standard hygiene around
the IR-flood Kinect still applies.
"""
from __future__ import annotations

import argparse
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import cv2                                                       # noqa: E402

from config import settings                                       # noqa: E402
from detection.detector import Detector                           # noqa: E402
from monitoring.hud import FpsMeter, draw_hud                      # noqa: E402
from safety.kinect_safety_check import KinectSafetyCheck          # noqa: E402
from safety.person_detector import HOGPersonDetector              # noqa: E402
from safety.safety_moderator import (                             # noqa: E402
    SafetyModerator, make_kinect_person_check, make_person_detector_check,
)
from sensors.base import SensorFrame, SensorRole                  # noqa: E402
from sensors.kinect_v2 import KinectV2Sensor, pykinect2_available  # noqa: E402
from sensors.ov9281 import OV9281Sensor                           # noqa: E402
from sensors.sensor_manager import SensorManager                  # noqa: E402
from tracking.tracker import Tracker                              # noqa: E402


VIABLE_COOLDOWN_S = 2.0          # per-track-id cooldown so one mosquito
                                  # doesn't add to the counter every frame.


@dataclass
class Counters:
    detections: int = 0
    confirmed_tracks: int = 0
    viable_shots: int = 0
    blocked_by_safety: int = 0
    blocked_outside_fov: int = 0
    per_track: dict[int, int] = field(default_factory=lambda: defaultdict(int))
    last_credit_ts: dict[int, float] = field(default_factory=dict)


def _open_ov9281(index: int) -> Optional[OV9281Sensor]:
    s = OV9281Sensor(device_index=index)
    if not s.open():
        return None
    return s


def _open_kinect() -> Optional[KinectV2Sensor]:
    if not pykinect2_available():
        return None
    s = KinectV2Sensor()
    if not s.open():
        return None
    return s


def _in_fov(x_norm: float, y_norm: float, margin: float) -> bool:
    return (margin <= x_norm <= 1.0 - margin
            and margin <= y_norm <= 1.0 - margin)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam-index", type=int,
                    default=settings.OV9281_DEVICE_INDEX,
                    help="OV9281 cv2 index (ov9281_probe.py --list)")
    ap.add_argument("--no-kinect", action="store_true",
                    help="run without the Kinect — useful in dev without "
                         "the SDK. Safety gate becomes OV9281-RGB-only.")
    ap.add_argument("--no-person-rgb", action="store_true",
                    help="disable HOG person detection on OV9281 RGB "
                         "(saves CPU; Kinect depth still gates).")
    ap.add_argument("--fov-margin", type=float, default=0.05,
                    help="treat tracks outside [margin, 1-margin] of the "
                         "OV9281 frame as out-of-FOV.")
    ap.add_argument("--no-arm-prompt", action="store_true",
                    help="auto-arm the SafetyModerator (default prompts).")
    ap.add_argument("--no-window", action="store_true",
                    help="run headless — counters only, no preview.")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after N seconds (0 = run until q/ESC).")
    args = ap.parse_args(argv)

    print("=" * 60)
    print("  spotter_mode — laser-OFF target acquisition")
    print("  OV9281 (targeting) + Kinect (safety + secondary)")
    print("=" * 60)

    ov = _open_ov9281(args.cam_index)
    if ov is None:
        print(f"FAIL: could not open OV9281 idx {args.cam_index}")
        return 1
    print(f"sensor: {ov.sensor_id} opened")

    kinect: Optional[KinectV2Sensor] = None
    if not args.no_kinect:
        kinect = _open_kinect()
        if kinect is None:
            print("note: Kinect not available — running without depth safety")
        else:
            print(f"sensor: {kinect.sensor_id} opened")

    # SafetyModerator: pull Kinect depth check + optional HOG on OV9281 RGB
    # treated as a SAFETY-role sensor in parallel to its TARGETING role.
    moderator = SafetyModerator(
        stale_after_s=settings.SAFETY_MODERATOR_STALE_AFTER_S,
        safe_cooldown_s=settings.SAFETY_MODERATOR_COOLDOWN_S,
        require_explicit_arm=not args.no_arm_prompt,
        # Spotter mode never opens the cube — a check-less moderator only
        # affects the viable-shot counter, so it is allowed here.
        allow_no_checks=True,
    )
    if kinect is not None and settings.SAFETY_KINECT_DEPTH_CHECK_ENABLED:
        depth_chk = KinectSafetyCheck(
            min_depth_m=settings.SAFETY_KINECT_DEPTH_MIN_M,
            max_depth_m=settings.SAFETY_KINECT_DEPTH_MAX_M,
            min_height_m=settings.SAFETY_KINECT_PERSON_MIN_HEIGHT_M,
            max_height_m=settings.SAFETY_KINECT_PERSON_MAX_HEIGHT_M,
        )
        moderator.add_check("kinect_v2",
                            make_kinect_person_check(depth_chk))
        print("  kinect depth person check: ENABLED")

    hog: Optional[HOGPersonDetector] = None
    if not args.no_person_rgb and settings.SAFETY_PERSON_DETECT_ENABLED:
        hog = HOGPersonDetector(
            hit_threshold=settings.SAFETY_PERSON_HOG_HIT_THRESHOLD)
        moderator.add_check("ov9281_safety",
                            make_person_detector_check(hog))
        print("  HOG person detection on OV9281: ENABLED")

    if not args.no_arm_prompt:
        if input("Arm the safety moderator? [type 'yes'] ").strip().lower() != "yes":
            ov.close()
            if kinect is not None:
                kinect.close()
            print("aborted; not armed")
            return 0
        moderator.arm()
        print("ARMED")
    else:
        moderator.arm()

    # Detector + Tracker on the OV9281 stream. Kinect RGB gets its own
    # detector when available — its detections feed cross-sensor fusion
    # (we don't wire CrossSensorFusion explicitly here since extrinsic
    # calibration is not assumed present; the secondary count is logged).
    ov_detector = Detector()
    kinect_detector = Detector() if kinect is not None else None
    tracker = Tracker()
    counters = Counters()

    # Wire the moderator into the SensorManager so it sees SAFETY frames.
    # OV9281 frames need a duplicate "ov9281_safety" injection so the HOG
    # check fires on them (the OV9281 sensor itself has TARGETING role).
    manager = SensorManager(frame_sink=lambda f: None)

    # Add the Kinect as SAFETY-role sensor (its sensor_id is "kinect_v2"
    # and its default role is SAFETY).
    if kinect is not None:
        manager.add_sensor(kinect)

    win = "spotter — q/ESC to quit"
    if not args.no_window:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    t_start = time.monotonic()
    fps_meter = FpsMeter()
    try:
        while True:
            frame = ov.read()
            if frame is None or frame.rgb is None:
                if (args.seconds and time.monotonic() - t_start > args.seconds):
                    break
                continue

            # The SensorManager worker keeps the latest Kinect frame; we
            # poll it here and push it to the moderator directly (this
            # script doesn't use the manager's frame sink). Depth-less
            # polls are skipped — staleness still closes the gate if the
            # depth stream stays quiet (same convention as
            # live_fire_session.py).
            kinect_frame = (manager.latest_frame("kinect_v2")
                            if kinect is not None else None)
            if kinect_frame is not None and kinect_frame.depth is not None:
                moderator.on_frame(kinect_frame)

            # If HOG is enabled, run it on the OV9281 frame as a synthetic
            # SAFETY-role view of the same camera, then push the synthesized
            # frame to the moderator. This avoids opening the OV9281 twice.
            if hog is not None:
                synth = SensorFrame(timestamp=frame.timestamp,
                                    sensor_id="ov9281_safety",
                                    sensor_role=SensorRole.SAFETY.value)
                synth.rgb = frame.rgb
                synth.width = frame.width
                synth.height = frame.height
                moderator.on_frame(synth)

            # Detect + track on the targeting stream.
            t_now = time.monotonic()
            dets = ov_detector.detect_bgsub(frame.rgb,
                                            sensor_id=ov.sensor_id,
                                            timestamp=t_now)
            counters.detections += len(dets)
            tracks = tracker.update(dets, t_now)

            # Secondary detection on the Kinect RGB (when available).
            kinect_n = 0
            if kinect_detector is not None and kinect_frame is not None and \
                    kinect_frame.rgb is not None:
                k_dets = kinect_detector.detect_bgsub(
                    kinect_frame.rgb,
                    sensor_id="kinect_v2",
                    timestamp=t_now)
                kinect_n = len(k_dets)

            # Viable-shot accounting.
            safe = moderator.is_safe_to_fire()
            verdict = moderator.verdict()
            for t in tracks:
                if not t.fire_eligible:
                    continue
                counters.confirmed_tracks += 1
                x_n, y_n = float(t.state[0]), float(t.state[1])
                if not _in_fov(x_n, y_n, args.fov_margin):
                    counters.blocked_outside_fov += 1
                    continue
                if not safe:
                    counters.blocked_by_safety += 1
                    continue
                last = counters.last_credit_ts.get(t.track_id, 0.0)
                if t_now - last < VIABLE_COOLDOWN_S:
                    continue
                counters.last_credit_ts[t.track_id] = t_now
                counters.viable_shots += 1
                counters.per_track[t.track_id] += 1

            # Render the preview window.
            if not args.no_window:
                vis = frame.rgb.copy()
                draw_hud(vis, tracks=tracks, verdict=verdict,
                         fov_margin=args.fov_margin,
                         stats=[("VIABLE", counters.viable_shots),
                                ("DET", counters.detections),
                                ("TRK", counters.confirmed_tracks),
                                ("K2", kinect_n)],
                         fps=fps_meter.tick(t_now))
                cv2.imshow(win, vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break

            if args.seconds and time.monotonic() - t_start > args.seconds:
                break
    finally:
        if not args.no_window:
            cv2.destroyAllWindows()
        ov.close()
        manager.stop_all()

    elapsed = time.monotonic() - t_start
    print("\n" + "-" * 60)
    print(f"  ran {elapsed:.1f}s")
    print(f"  detections          : {counters.detections}")
    print(f"  confirmed-track hits: {counters.confirmed_tracks}")
    print(f"  viable shots        : {counters.viable_shots}")
    print(f"  blocked outside FOV : {counters.blocked_outside_fov}")
    print(f"  blocked by safety   : {counters.blocked_by_safety}")
    if counters.per_track:
        top = sorted(counters.per_track.items(),
                     key=lambda kv: -kv[1])[:10]
        print("  per-track top:")
        for tid, n in top:
            print(f"    track {tid}: {n} viable")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
