#!/usr/bin/env python3
"""fov_box.py — draw the calibration-bounds box with the laser while showing
the camera's live view, so the camera can be positioned to cover the full
calibration FOV before running step11_calibration.

The box outlines the exact galvo-norm region the calibration pattern sweeps
([margin, 1-margin]² — default margin=0.2, so a 0.6×0.6 square centred on
the galvo). Aim the camera so all four corners of the laser box are visible
in the preview window; that guarantees every calibration point will land
inside the frame.

>>> Class 3B laser. Eye protection + safety lens REQUIRED. <<<

    python tools/fov_box.py --camera ov9281 --cam-index 2
    python tools/fov_box.py --camera ov9281 --cam-index 2 --margin 0.1   # bigger box
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2                                                       # noqa: E402

from config import settings                                       # noqa: E402
from laser.lasercube import LaserCubeInterface                    # noqa: E402
from laser.types import LaserPoint                                # noqa: E402
from sensors.ov9281 import OV9281Sensor                           # noqa: E402
from sensors.local_cam import LocalCamSensor                      # noqa: E402
from targeting.patterns import galvo_norm_to_dac                  # noqa: E402


def _box_path(margin: float, points_per_side: int) -> list[tuple[float, float]]:
    """Closed perimeter of the calibration bounds in galvo-norm space."""
    lo, hi = margin, 1.0 - margin
    out: list[tuple[float, float]] = []
    n = max(2, points_per_side)
    # Bottom edge: lo,lo -> hi,lo
    for i in range(n):
        t = i / (n - 1)
        out.append((lo + (hi - lo) * t, lo))
    # Right: hi,lo -> hi,hi
    for i in range(1, n):
        t = i / (n - 1)
        out.append((hi, lo + (hi - lo) * t))
    # Top: hi,hi -> lo,hi
    for i in range(1, n):
        t = i / (n - 1)
        out.append((hi - (hi - lo) * t, hi))
    # Left: lo,hi -> lo,lo
    for i in range(1, n):
        t = i / (n - 1)
        out.append((lo, hi - (hi - lo) * t))
    return out


def _to_points(path, power_pct: float) -> list[LaserPoint]:
    lp = max(0, min(0xFFF, int(round(0xFFF * power_pct / 100.0))))
    return [LaserPoint(x=galvo_norm_to_dac(x), y=galvo_norm_to_dac(y),
                       r=lp, g=lp, b=lp) for x, y in path]


def _open_camera(kind: str, index: int):
    if kind == "ov9281":
        cam = OV9281Sensor(device_index=index)
    else:
        cam = LocalCamSensor(index=index)
    return cam if cam.open() else None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="169.254.40.83", help="LaserCube IP")
    ap.add_argument("--src-ip", default="auto")
    ap.add_argument("--camera", choices=["ov9281", "local"], default="ov9281")
    ap.add_argument("--cam-index", type=int,
                    default=settings.OV9281_DEVICE_INDEX,
                    help="cv2 device index (find with ov9281_probe.py --list)")
    ap.add_argument("--margin", type=float, default=0.2,
                    help="galvo-norm margin matching CALIBRATION_PATTERN "
                         "(default 0.2 = 0.6x0.6 square centred on galvo)")
    ap.add_argument("--points-per-side", type=int, default=250,
                    help="more points = smoother box outline (default 250)")
    ap.add_argument("--laser-power", type=float, default=20.0,
                    help="percent of full brightness (default 20)")
    args = ap.parse_args(argv)

    print("=" * 60)
    print("  FOV alignment box — laser perimeter + live camera view")
    print("  Class 3B laser. Eye protection + safety lens REQUIRED.")
    print(f"  bounds: galvo-norm [{args.margin:.2f}, {1 - args.margin:.2f}]^2"
          f"   power: {args.laser_power:.0f}%")
    print("=" * 60)

    cube = LaserCubeInterface(ip=args.ip, src_ip=args.src_ip)
    if not cube.connect():
        print("FAIL: could not connect to the cube")
        return 1
    print(f"connected (src_ip={cube.src_ip})")

    cam = _open_camera(args.camera, args.cam_index)
    if cam is None:
        cube.disconnect()
        print(f"FAIL: could not open {args.camera} idx {args.cam_index}")
        return 1
    print(f"camera: {cam.sensor_id} opened")

    if input("Fire the laser to draw the FOV box? [type 'yes'] "
             ).strip().lower() != "yes":
        cam.close()
        cube.disconnect()
        print("aborted by operator")
        return 0

    path = _box_path(args.margin, args.points_per_side)
    box_points = _to_points(path, args.laser_power)

    win = "fov_box — q/ESC to exit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    try:
        cube.enable_output()
        print("\nLASER ON — adjust the camera until you see all four corners "
              "of the box.\n  press q or ESC in the preview window to exit\n")
        while True:
            cube.send_frame(box_points, frame_num=0)
            frame = cam.read()
            if frame is not None and frame.rgb is not None:
                img = frame.rgb.copy()
                # Crosshair at frame centre as a positioning aid (the calibration
                # centre lands somewhere inside the box, not necessarily at the
                # camera centre).
                h, w = img.shape[:2]
                cv2.line(img, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)
                cv2.line(img, (0, h // 2), (w, h // 2), (0, 255, 0), 1)
                cv2.imshow(win, img)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
    finally:
        cube.disable_output()
        print("LASER OFF")
        cv2.destroyAllWindows()
        cam.close()
        cube.disconnect()
        print("disconnected")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
