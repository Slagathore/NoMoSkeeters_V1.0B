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


def _open_camera(kind: str, index: int):
    if kind == "gopro":
        cam = GoProSensor(control=GoProInterface())   # HTTP start + keep-alive
    else:
        cam = LocalCamSensor(index=index)
    return cam if cam.open() else None


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="LaserCube live calibration.")
    parser.add_argument("--ip", default="169.254.40.83")
    parser.add_argument("--src-ip", default="auto")
    parser.add_argument("--camera", choices=["local", "gopro"], default="local")
    parser.add_argument("--cam-index", type=int, default=0)
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

    camera = _open_camera(args.camera, args.cam_index)
    if camera is None:
        print(f"FAIL: could not open {args.camera} camera")
        cube.disconnect()
        return 1
    print(f"camera: {camera.sensor_id} opened")

    if not _confirm(f"Run a '{args.pattern}' calibration sweep now?"):
        camera.close()
        cube.disconnect()
        print("aborted by operator")
        return 0

    rc = 1
    try:
        cube.enable_output()
        print("\nLASER ON — calibration sweep")
        run = run_live_calibration(
            cube, camera, pattern=args.pattern, sensor_id=args.sensor_id,
            scene=args.scene, mount_tilt_config=args.mount_config,
            on_point=_progress)
        mapper = run.mapper
        print(f"\nfit: {mapper.n_points} points, "
              f"residual_norm={mapper.residual_norm:.4f}, "
              f"residual_galvo={mapper.residual_galvo:.1f}")

        # Multi-depth validation — operator repositions the target each depth.
        depths = settings.CALIBRATION_VALIDATION_DEPTHS_M
        all_targets = []
        for depth in depths:
            input(f"\nPlace the calibration target at {depth:.1f} m, "
                  f"then press Enter...")
            targets = live_validation_sweep(cube, camera, depth,
                                            pattern=args.pattern,
                                            on_point=_progress)
            print(f"  captured {len(targets)} validation points at {depth} m")
            all_targets.extend(targets)

        result = validate_multi_depth(mapper, all_targets)
        mapper.validation = result
        mapper.save(run.json_path)

        print("\n" + "-" * 60)
        for depth, resid in sorted(result.per_depth_residual_norm.items()):
            print(f"  depth {depth:.1f} m : residual_norm={resid:.4f}")
        print(f"  max residual : {result.max_residual_norm:.4f}  "
              f"(threshold {settings.CALIBRATION_MAX_RESIDUAL_NORM})")
        print(f"  calibration  : {run.json_path}")
        if result.passed:
            print("\nPASS — calibration valid at all depths. Step 11 complete.")
            rc = 0
        else:
            print("\nFAIL — a planar homography is insufficient for this "
                  "volume. Re-seat the camera, recalibrate, or escalate to a "
                  "depth-aware mapper (v0.3).")
    finally:
        cube.disable_output()
        print("LASER OFF")
        camera.close()
        cube.disconnect()
        print("disconnected")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
