#!/usr/bin/env python3
"""ov9281_probe.py — find and exercise the OV9281 global-shutter USB camera.

Opens the camera through OV9281Sensor (so it configures MJPEG + the requested
resolution/fps + manual exposure exactly like the live system), reads frames
for a few seconds, and reports the achieved fps / frame shape. Use --list first
if you don't know which cv2 index the camera enumerated as.

    python tools/ov9281_probe.py --list
    python tools/ov9281_probe.py --index 0
    python tools/ov9281_probe.py --index 0 --width 1280 --height 800 --fps 120 --view
    python tools/ov9281_probe.py --index 0 --width 320 --height 240 --fps 420

Network-free, no laser, no admin. Just the camera.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import cv2                                                       # noqa: E402

from config import settings                                      # noqa: E402
from sensors.ov9281 import OV9281Sensor, _resolve_backend        # noqa: E402


def _list_indices(max_index: int, backend: str) -> None:
    print(f"scanning cv2 indices 0..{max_index} (backend={backend})...")
    flag = _resolve_backend(backend)
    found = 0
    for i in range(max_index + 1):
        cap = cv2.VideoCapture(i, flag)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"  index {i}: OPEN  default {w}x{h}")
            found += 1
        cap.release()
    if not found:
        print("  no cameras opened — check the USB connection / try --backend msmf")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="OV9281 probe.")
    p.add_argument("--list", action="store_true",
                   help="scan indices and report which open, then exit")
    p.add_argument("--max-index", type=int, default=5)
    p.add_argument("--index", type=int, default=settings.OV9281_DEVICE_INDEX)
    p.add_argument("--backend", default=settings.OV9281_BACKEND,
                   choices=["auto", "dshow", "msmf", "v4l2", "any"])
    p.add_argument("--width", type=int, default=settings.OV9281_WIDTH)
    p.add_argument("--height", type=int, default=settings.OV9281_HEIGHT)
    p.add_argument("--fps", type=int, default=settings.OV9281_FPS)
    p.add_argument("--auto-exposure", action="store_true")
    p.add_argument("--exposure", type=float, default=settings.OV9281_EXPOSURE)
    p.add_argument("--gain", type=float, default=settings.OV9281_GAIN)
    p.add_argument("--seconds", type=float, default=6.0)
    p.add_argument("--view", action="store_true", help="live preview window")
    args = p.parse_args(argv)

    print("=" * 60)
    print("  OV9281 global-shutter camera probe")
    print("=" * 60)

    if args.list:
        _list_indices(args.max_index, args.backend)
        return 0

    sensor = OV9281Sensor(
        device_index=args.index, backend=args.backend,
        width=args.width, height=args.height, fps=args.fps,
        auto_exposure=args.auto_exposure, exposure=args.exposure, gain=args.gain)

    if not sensor.open():
        print(f"FAIL: could not open index {args.index} (try --list / --backend msmf)")
        return 1

    print(f"\nstreaming {args.width}x{args.height}@{args.fps} for {args.seconds:.0f}s "
          f"(auto_exposure={args.auto_exposure})...")
    n = 0
    shape = None
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds:
            frame = sensor.read()
            if frame is None or frame.rgb is None:
                time.sleep(0.0005)
                continue
            n += 1
            shape = frame.rgb.shape
            if args.view:
                cv2.imshow("ov9281", frame.rgb)
                cv2.waitKey(1)
    finally:
        elapsed = time.time() - t0
        sensor.close()
        if args.view:
            cv2.destroyAllWindows()

    print(f"  {n} frames in {elapsed:.1f}s = {n / elapsed:.0f} fps   shape={shape}")
    if n and n / elapsed < args.fps * 0.5:
        print("  NOTE: achieved fps is well below requested — the driver may have "
              "clamped the mode, or exposure is too long. Try a smaller resolution "
              "or shorter --exposure.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
