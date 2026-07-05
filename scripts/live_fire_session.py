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
from laser.heartbeat import CubeHeartbeat                          # noqa: E402
from laser.laser_manager import LaserManager                      # noqa: E402
from laser.lasercube import LaserCubeInterface                    # noqa: E402
from monitoring.hud import FpsMeter, draw_hud                      # noqa: E402
from monitoring.session_recorder import SessionRecorder            # noqa: E402
from safety.kinect_safety_check import KinectSafetyCheck          # noqa: E402
from safety.person_detector import HOGPersonDetector              # noqa: E402
from safety.safety_moderator import (                             # noqa: E402
    SafetyModerator, make_cube_health_check, make_kinect_person_check,
    make_person_detector_check,
)
from safety.voice_warning import VoiceWarning                     # noqa: E402
from sensors.base import SensorFrame, SensorRole                  # noqa: E402
from sensors.kinect_v2 import KinectV2Sensor, pykinect2_available  # noqa: E402
from sensors.ov9281 import OV9281Sensor                           # noqa: E402
from targeting import fire_control                                 # noqa: E402
from targeting.calibration import CoordinateMapper                 # noqa: E402
from targeting.extrinsics import CrossSensorExtrinsic              # noqa: E402
from targeting.fire_control import FireGate                        # noqa: E402
from tracking.cross_sensor_fusion import CrossSensorFusion         # noqa: E402
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
    shots_blocked_oversize: int = 0


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


def _read_hardened(sensor, label: str, state: dict,
                   recorder=None) -> Optional[SensorFrame]:
    """sensor.read() that survives driver throws and reopens a stalled
    sensor inline. The live loop stays pull-based (a manager poll would
    add jitter to the OV9281's ~5ms frame budget), so the reconnect
    doctrine from SensorManager is replicated here in miniature: >2s
    without a frame → close + reopen, retried at most every 2s. While a
    SAFETY sensor is down, moderator staleness keeps the gate closed."""
    now = time.monotonic()
    try:
        frame = sensor.read()
    except Exception as exc:
        frame = None
        if now - state.get("last_err_log", 0.0) > 5.0:
            state["last_err_log"] = now
            print(f"WARN: {label} read() raised ({exc!r}) — treating as "
                  f"stalled")
    if frame is not None:
        state["last_frame"] = now
        return frame
    last = state.setdefault("last_frame", now)
    if now - last > 2.0 and now - state.get("last_reopen", 0.0) > 2.0:
        state["last_reopen"] = now
        print(f"WARN: {label} stalled {now - last:.1f}s — reopening")
        if recorder is not None:
            recorder.record_note(f"{label} stalled {now - last:.1f}s; "
                                 f"reopening")
        try:
            sensor.close()
        except Exception:
            pass
        try:
            if sensor.open():
                print(f"  {label} reopened")
                if recorder is not None:
                    recorder.record_note(f"{label} reopened")
        except Exception as exc:
            print(f"  {label} reopen failed ({exc!r}); will retry")
    return None


def _make_depth_sampler(kinect: KinectV2Sensor, depth):
    """(x_norm, y_norm) → camera-relative (x_m, y_m, z_m) via the depth
    plane. RGB→depth registration is approximated by normalized coords
    (the RGB and depth FoVs differ; exact registration needs the SDK
    CoordinateMapper — same acknowledged approximation as the nominal
    intrinsics in sensors/kinect_v2.py)."""
    dh, dw = depth.shape[:2]

    def sample(x_norm: float, y_norm: float):
        xd = int(x_norm * (dw - 1))
        yd = int(y_norm * (dh - 1))
        return kinect.world_position(xd, yd, float(depth[yd, xd]))
    return sample


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
    ap.add_argument("--extrinsic", default=None, metavar="JSON",
                    help="path to CrossSensorExtrinsic for kinect->ov9281 "
                         "fusion. If omitted, each sensor's detections are "
                         "tracked independently (no cross-confirmation).")
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
        # Dry-fire opens no cube, so a check-less moderator is harmless
        # there; a live session with zero checks is refused below.
        allow_no_checks=args.dry_fire,
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

    # A live session must have at least one automated person check. The
    # cube-health check below doesn't count — it watches the laser, not
    # the room.
    if not args.dry_fire and not moderator.has_checks:
        print("FAIL: no person check registered (Kinect unavailable/"
              "disabled and HOG disabled). Refusing to arm a live session "
              "with no automated human-presence check — use --dry-fire "
              "for sensor-less smoke tests.")
        ov.close()
        if kinect is not None:
            kinect.close()
        if cube is not None:
            cube.disconnect()
        return 1

    # Cube health: a heartbeat thread pings GET_FULL_INFO on a fixed
    # cadence (BOOTSTRAP_AMENDMENTS §9.10 — also keeps the cube's 4s
    # comms timer alive between shots) and feeds interlock/over-temp
    # into the moderator as a SAFETY check. The staleness override lets
    # one 1.5s beat go missing; two missed beats close the gate.
    heartbeat: Optional[CubeHeartbeat] = None
    recorder: Optional[SessionRecorder] = None
    if cube is not None:
        def _on_cube_status(info) -> None:
            tick = SensorFrame(timestamp=time.time(), sensor_id="lasercube",
                               sensor_role=SensorRole.SAFETY.value)
            moderator.on_frame(tick)
            if recorder is not None:
                recorder.record_event({
                    "op": "cube_status",
                    "interlock": info.interlock,
                    "over_temp": info.over_temp,
                    "temp_warn": info.temp_warn,
                    "temperature_c": info.temperature_c,
                    "output_enabled": info.output_enabled,
                    "buffer_free": info.buffer_free,
                })
        heartbeat = CubeHeartbeat(cube, on_status=_on_cube_status)
        moderator.add_check("lasercube", make_cube_health_check(heartbeat),
                            stale_after_s=3.5)
        print("  safety: cube interlock/over-temp heartbeat ENABLED")

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

    recorder = SessionRecorder(tag="live_fire")
    if recorder.path is not None:
        print(f"  session log: {recorder.path}")
    moderator.set_verdict_sink(recorder.record_verdict)
    if heartbeat is not None:
        heartbeat.start()

    detector = Detector()
    kinect_detector = Detector() if kinect is not None else None
    tracker = Tracker()
    counters = SessionCounters()

    extrinsic = None
    if args.extrinsic is not None:
        try:
            extrinsic = CrossSensorExtrinsic.load(args.extrinsic)
            print(f"  extrinsic loaded: {extrinsic.src_sensor} -> "
                  f"{extrinsic.dst_sensor}  (n={extrinsic.n_points}, "
                  f"residual={extrinsic.residual_norm:.4f})")
        except Exception as exc:
            print(f"WARN: extrinsic load failed ({exc}) — falling back to "
                  f"sensor-independent tracking")
    fusion = CrossSensorFusion(extrinsic=extrinsic)

    laser_mgr: Optional[LaserManager] = None
    if cube is not None:
        # LaserCubeInterface inherits from LaserCubeTransport, so pass it
        # directly to LaserManager.
        laser_mgr = LaserManager(transport=cube, mapper=mapper,
                                  software_lag_ms=settings.LATENCY_SOFTWARE_LAG_MS,
                                  event_sink=recorder.record_event)

    # Kinect frames are read inline in the main loop below (the Kinect's
    # 30fps cadence is comfortable inside the OV9281's ~200fps loop).
    stop = threading.Event()

    win = "live_fire — q/ESC quit"
    if not args.no_window:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    fire_gate = FireGate(
        moderator=moderator,
        fov_margin=args.fov_margin,
        per_track_cooldown_s=PER_TRACK_FIRE_COOLDOWN_S,
        max_rate_hz=MAX_SHOT_RATE_HZ,
        frame_width_px=settings.OV9281_WIDTH,
    )
    t_start = time.monotonic()
    fps_meter = FpsMeter()
    last_shot: Optional[tuple[float, float, float]] = None  # (t, x_n, y_n)
    ov_state: dict = {}
    kinect_state: dict = {}

    try:
        while True:
            frame = _read_hardened(ov, "ov9281", ov_state, recorder)
            if frame is None or frame.rgb is None:
                if args.seconds and time.monotonic() - t_start > args.seconds:
                    break
                time.sleep(0.001)
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
            for d in dets:
                recorder.record_detection(d)
            tracks = tracker.update(dets, now)
            for d in dets:
                fusion.ingest_detection(d)

            # Kinect detections feed the fusion only — they don't drive the
            # firing tracker (OV9281 is the primary). With an extrinsic loaded
            # this gives cross-sensor confirmation; without one, the calls
            # are still cheap and the solo tracks just sit in the fusion.
            if kinect_detector is not None:
                kinect_frame = (_read_hardened(kinect, "kinect_v2",
                                               kinect_state, recorder)
                                if kinect is not None else None)
                if kinect_frame is not None and kinect_frame.rgb is not None:
                    sampler = (_make_depth_sampler(kinect, kinect_frame.depth)
                               if kinect is not None
                               and kinect_frame.depth is not None else None)
                    for d in kinect_detector.detect_bgsub(
                            kinect_frame.rgb,
                            sensor_id="kinect_v2",
                            timestamp=now,
                            world_fn=sampler):
                        recorder.record_detection(d)
                        fusion.ingest_detection(d)
                # Kinect frames also need to reach the moderator — push the
                # depth frame through if available.
                if kinect_frame is not None and kinect_frame.depth is not None:
                    moderator.on_frame(kinect_frame)

            verdict = moderator.verdict()
            safe = verdict.safe

            for t in tracks:
                if not t.fire_eligible:
                    continue
                counters.confirmed_tracks += 1
                decision = fire_gate.evaluate(t, now, safe_hint=safe)
                if not decision.fire:
                    if decision.reason == fire_control.OUTSIDE_FOV:
                        counters.shots_blocked_fov += 1
                    elif decision.reason == fire_control.UNSAFE:
                        counters.shots_blocked_safety += 1
                    elif decision.reason == fire_control.OVERSIZE:
                        counters.shots_blocked_oversize += 1
                    elif decision.reason in (fire_control.COOLDOWN_TRACK,
                                             fire_control.COOLDOWN_GLOBAL):
                        counters.shots_blocked_cooldown += 1
                    continue
                counters.shots_fired += 1
                x_n, y_n = float(t.state[0]), float(t.state[1])
                det = getattr(t, "last_detection", None)
                recorder.record_track({
                    "track_id": t.track_id,
                    "detection_id": (det.detection_id
                                     if det is not None else None),
                    "det_to_fire_ms": (now - t.last_update_ts) * 1000.0,
                    "x_norm": x_n,
                    "y_norm": y_n,
                    "target_area_mm2": decision.target_area_mm2,
                    "fired": True,
                })
                last_shot = (time.monotonic(), x_n, y_n)
                if laser_mgr is not None:
                    laser_mgr.engage_track(t)
                else:
                    print(f"DRY-FIRE: would shoot track {t.track_id} at "
                          f"({x_n:.3f}, {y_n:.3f})")

            if not args.no_window:
                vis = frame.rgb.copy()
                flash = None
                if last_shot is not None:
                    flash = (last_shot[0],
                             (int(last_shot[1] * (vis.shape[1] - 1)),
                              int(last_shot[2] * (vis.shape[0] - 1))))
                draw_hud(vis, tracks=tracks, verdict=verdict,
                         fov_margin=args.fov_margin,
                         stats=[("FIRED", counters.shots_fired),
                                ("DET", counters.detections),
                                ("TRK", counters.confirmed_tracks)],
                         fps=fps_meter.tick(now),
                         lag_ms=settings.LATENCY_SOFTWARE_LAG_MS,
                         flash=flash)
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
        if heartbeat is not None:
            heartbeat.stop()
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
        recorder.close()

    elapsed = time.monotonic() - t_start
    print("\n" + "-" * 60)
    print(f"  ran {elapsed:.1f}s")
    print(f"  detections          : {counters.detections}")
    print(f"  confirmed-track hits: {counters.confirmed_tracks}")
    print(f"  shots fired         : {counters.shots_fired}")
    print(f"  blocked by safety   : {counters.shots_blocked_safety}")
    print(f"  blocked outside FOV : {counters.shots_blocked_fov}")
    print(f"  blocked by cooldown : {counters.shots_blocked_cooldown}")
    print(f"  blocked oversize    : {counters.shots_blocked_oversize}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
