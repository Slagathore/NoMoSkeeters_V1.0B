"""Step 11b — live calibration + multi-depth validation.

Run AFTER step11_first_light.py. Connects the cube and a camera, fires the
calibration pattern, detects each laser dot in the camera, fits the
CoordinateMapper, then validates at the configured depths. Persists a
calibration JSON v2. Always disables output on exit.

    python scripts/step11_calibration.py --camera local --cam-index 0
    python scripts/step11_calibration.py --camera gopro      # preview must run

Acceptance (§17 Step 11): multi-depth validation passes at all three depths
within CALIBRATION_MAX_RESIDUAL_NORM.

SAFETY: Class 3B laser. Eye protection + safety lens. Calibration target a
matte surface; beam path clear.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from config import settings                                       # noqa: E402
from laser.lasercube import LaserCubeInterface                     # noqa: E402
from sensors.gopro import GoProSensor                              # noqa: E402
from sensors.gopro_interface import GoProInterface                 # noqa: E402
from sensors.local_cam import LocalCamSensor                        # noqa: E402
from targeting.calibration import validate_multi_depth             # noqa: E402
from targeting.calibration_controller import (                     # noqa: E402
    live_validation_sweep, run_live_calibration,
)
from targeting.dot_detector import DotObservation                  # noqa: E402


def _confirm(prompt: str) -> bool:
    return input(f"{prompt} [type 'yes' to proceed] ").strip().lower() == "yes"


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


def _progress(idx: int, total: int, obs) -> None:
    if obs is None:
        print(f"  point {idx + 1:>3}/{total}  MISSED — no dot detected")
    else:
        print(f"  point {idx + 1:>3}/{total}  dot=({obs.x_norm:.3f},"
              f"{obs.y_norm:.3f})  conf={obs.confidence:.2f}")


_FOV_CODES = {"default": None, "wide": 0, "linear": 4, "superview": 3}


def _open_camera(kind: str, index: int, gopro_ip: str, decoder: str,
                 fov: object, record: object):
    if kind == "gopro":
        # HTTP control over the camera's IP (USB-tethered = .51).
        cam = GoProSensor(
            control=GoProInterface(ip=gopro_ip, webcam_fov=fov),
            decoder=decoder, record_path=record)
    else:
        cam = LocalCamSensor(index=index)
    return cam if cam.open() else None


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="LaserCube live calibration.")
    parser.add_argument("--ip", default="169.254.40.83")
    parser.add_argument("--src-ip", default="auto")
    parser.add_argument("--camera", choices=["local", "gopro"], default="local")
    parser.add_argument("--cam-index", type=int, default=0)
    parser.add_argument("--gopro-ip", default="10.5.5.9",
                        help="GoPro HTTP API IP. USB-tethered is 172.X.Y.51 "
                             "(run Find-GoPro to discover); 10.5.5.9 is the "
                             "WiFi-AP-mode fallback.")
    parser.add_argument("--decoder", choices=["opencv", "ffmpeg"],
                        default="opencv",
                        help="GoPro stream decoder. 'ffmpeg' pipes a "
                             "dedicated ffmpeg subprocess — use it if cv2 "
                             "cannot decode the Hero 13 HEVC stream.")
    parser.add_argument("--fov", choices=list(_FOV_CODES),
                        default="default",
                        help="GoPro webcam field of view. 'linear' removes "
                             "the fisheye warp (some firmware ignores it).")
    parser.add_argument("--view", action="store_true",
                        help="show a live camera-view window during the "
                             "sweep — useful for confirming the laser dot "
                             "is actually visible to the camera.")
    parser.add_argument("--record", default=None, metavar="FILE.ts",
                        help="record the GoPro feed for the whole run to "
                             "this .ts file (remux: ffmpeg -i x.ts -c copy "
                             "x.mp4). GoPro decoder only.")
    parser.add_argument("--single", action="store_true",
                        help="run one calibration sweep only — skip the "
                             "multi-depth validation (no target repositioning).")
    parser.add_argument("--laser-power", type=float, default=100.0,
                        metavar="PCT",
                        help="calibration laser brightness, percent of full "
                             "(default 100). Lower (e.g. 5) gives a crisp dot "
                             "instead of an overexposed bloom.")
    parser.add_argument("--background", choices=["per-point", "single"],
                        default="per-point",
                        help="laser-OFF reference for temporal-diff "
                             "detection: 'per-point' (fresh each point, "
                             "robust to auto-exposure drift) or 'single' "
                             "(one reference for the whole sweep, faster).")
    parser.add_argument("--hold", type=float, default=None, metavar="SEC",
                        help="seconds to hold the laser steady per point "
                             f"(default {settings.CALIBRATION_DWELL_HOLD_S}).")
    parser.add_argument("--settle", type=float, default=None, metavar="SEC",
                        help="seconds to wait out camera latency before "
                             "detecting (default "
                             f"{settings.CALIBRATION_DWELL_SETTLE_S}).")
    parser.add_argument("--pattern", default=settings.CALIBRATION_PATTERN)
    parser.add_argument("--sensor-id", default="gopro")
    parser.add_argument("--scene", default="bench")
    parser.add_argument("--mount-config", default="bench")
    args = parser.parse_args(argv)

    print("=" * 60)
    print("  Step 11b — LIVE CALIBRATION")
    print("  Class 3B laser. Eye protection + safety lens REQUIRED.")
    print("=" * 60)

    cube = LaserCubeInterface(ip=args.ip, src_ip=args.src_ip)
    if not cube.connect():
        print("FAIL: could not connect to the cube")
        return 1
    print(f"connected (src_ip={cube.src_ip})")
    if not _preflight(cube):
        cube.disconnect()
        return 1

    camera = _open_camera(args.camera, args.cam_index, args.gopro_ip,
                          args.decoder, _FOV_CODES[args.fov], args.record)
    if camera is None:
        print(f"FAIL: could not open {args.camera} camera")
        cube.disconnect()
        return 1
    print(f"camera: {camera.sensor_id} opened")

    # Optional live camera-view window for the duration of the sweep.
    on_frame = None
    cv2mod = None
    if args.view:
        import cv2 as cv2mod          # noqa: F401 — only needed for --view

        def on_frame(rgb):
            cv2mod.imshow("calibration - camera view", rgb)
            cv2mod.waitKey(1)

    if not _confirm(f"Run a '{args.pattern}' calibration sweep now?"):
        camera.close()
        cube.disconnect()
        print("aborted by operator")
        return 0

    # Calibration laser colour, scaled by --laser-power (0xFFF = full).
    _lp = max(0, min(0xFFF, int(round(0xFFF * args.laser_power / 100.0))))
    laser_rgb = (_lp, _lp, _lp)
    print(f"calibration laser power: {args.laser_power:.0f}%  (rgb={_lp})")
    _hold = args.hold if args.hold is not None else settings.CALIBRATION_DWELL_HOLD_S
    _settle = (args.settle if args.settle is not None
               else settings.CALIBRATION_DWELL_SETTLE_S)
    print(f"dwell: hold={_hold}s settle={_settle}s  "
          f"background={args.background}")

    rc = 1
    try:
        cube.enable_output()
        print("\nLASER ON — calibration sweep")
        try:
            run = run_live_calibration(
                cube, camera, pattern=args.pattern, sensor_id=args.sensor_id,
                scene=args.scene, mount_tilt_config=args.mount_config,
                laser_rgb=laser_rgb, background_mode=args.background,
                hold_s=args.hold, settle_s=args.settle,
                on_point=_progress, on_frame=on_frame)
        except (ValueError, RuntimeError) as exc:
            print(f"\nsweep finished but the homography fit failed: {exc}")
            print("This usually means the camera never cleanly saw the laser "
                  "dot at distinct points. If you passed --record, the GoPro "
                  "feed was still captured — review it to see what happened.")
            return rc                    # finally still runs: laser off, etc.
        mapper = run.mapper
        print(f"\nfit: {mapper.n_points} points, "
              f"residual_norm={mapper.residual_norm:.4f}, "
              f"residual_galvo={mapper.residual_galvo:.1f}")

        if args.single:
            mapper.save(run.json_path)
            print(f"\nsingle cycle done — calibration saved to {run.json_path}")
            print("(multi-depth validation skipped: --single)")
            rc = 0
        else:
            # Multi-depth validation — operator repositions the target each
            # depth. Depths shown in CALIBRATION_DEPTH_UNITS; math in metres.
            unit = settings.CALIBRATION_DEPTH_UNITS
            all_targets = []
            for disp, depth_m in zip(settings.CALIBRATION_VALIDATION_DEPTHS,
                                     settings.CALIBRATION_VALIDATION_DEPTHS_M):
                input(f"\nPlace the calibration target at {disp:.1f} {unit}, "
                      f"then press Enter...")
                targets = live_validation_sweep(cube, camera, depth_m,
                                                pattern=args.pattern,
                                                laser_rgb=laser_rgb,
                                                background_mode=args.background,
                                                hold_s=args.hold,
                                                settle_s=args.settle,
                                                on_point=_progress,
                                                on_frame=on_frame)
                print(f"  captured {len(targets)} validation points "
                      f"at {disp:.1f} {unit}")
                all_targets.extend(targets)

            result = validate_multi_depth(mapper, all_targets)
            mapper.validation = result
            mapper.save(run.json_path)

            print("\n" + "-" * 60)
            for depth_m, resid in sorted(
                    result.per_depth_residual_norm.items()):
                disp = depth_m / settings.FT_TO_M if unit == "ft" else depth_m
                print(f"  depth {disp:>5.1f} {unit} : "
                      f"residual_norm={resid:.4f}")
            print(f"  max residual : {result.max_residual_norm:.4f}  "
                  f"(threshold {settings.CALIBRATION_MAX_RESIDUAL_NORM})")
            print(f"  calibration  : {run.json_path}")
            if result.passed:
                print("\nPASS — calibration valid at all depths. "
                      "Step 11 complete.")
                rc = 0
            else:
                print("\nFAIL — a planar homography is insufficient for this "
                      "volume. Re-seat the camera, recalibrate, or escalate "
                      "to a depth-aware mapper (v0.3).")
    finally:
        cube.disable_output()
        print("LASER OFF")
        camera.close()
        cube.disconnect()
        if cv2mod is not None:
            cv2mod.destroyAllWindows()
        print("disconnected")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
