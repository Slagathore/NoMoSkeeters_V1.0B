#!/usr/bin/env python3
"""kinect_probe.py — confirm the Kinect v2 is alive and streaming.

Opens the Kinect, polls for a few seconds, and reports per-stream frame rate,
resolution, dtype, and sane-value checks (depth range in metres, IR levels).
With --manager it routes through SensorManager instead — the same path the
live system uses — so this doubles as the dual-sensor wiring check.

    python tools/kinect_probe.py
    python tools/kinect_probe.py --seconds 10 --view
    python tools/kinect_probe.py --manager

No laser, no GoPro, no admin needed. If the pykinect2 binding is missing or
unpatched, open() fails honestly and this prints why.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np                                            # noqa: E402

from sensors.kinect_v2 import KinectV2Sensor, pykinect2_available  # noqa: E402


def _summarize(counts: dict, shapes: dict, depth_rng, ir_stats,
               elapsed: float) -> None:
    print("\n" + "-" * 56)
    for stream in ("rgb", "depth", "ir"):
        n = counts[stream]
        if n == 0:
            print(f"  {stream:5s} : no frames")
            continue
        shp, dt = shapes[stream]
        print(f"  {stream:5s} : {n:4d} frames  {n / elapsed:5.1f} fps  "
              f"{shp} {dt}")
    if depth_rng is not None:
        print(f"  depth range (non-zero) : {depth_rng[0]:.2f}-"
              f"{depth_rng[1]:.2f} m")
    if ir_stats is not None:
        print(f"  ir levels (min/med/max): {ir_stats[0]}/{ir_stats[1]}/"
              f"{ir_stats[2]}")
    print("-" * 56)
    ok = counts["rgb"] and counts["depth"] and counts["ir"]
    print("RESULT:", "all three streams live" if ok
          else "INCOMPLETE - a stream did not deliver")


def _poll(read, seconds: float, on_view=None) -> int:
    counts = {"rgb": 0, "depth": 0, "ir": 0}
    shapes: dict = {}
    depth_rng = ir_stats = None
    t0 = time.time()
    while time.time() - t0 < seconds:
        frame = read()
        if frame is None:
            time.sleep(0.004)
            continue
        for stream in ("rgb", "depth", "ir"):
            arr = getattr(frame, stream)
            if arr is None:
                continue
            counts[stream] += 1
            shapes[stream] = (arr.shape, str(arr.dtype))
            if stream == "depth":
                nz = arr[arr > 0]
                if nz.size:
                    depth_rng = (float(nz.min()), float(nz.max()))
            elif stream == "ir":
                ir_stats = (int(arr.min()), int(np.median(arr)),
                            int(arr.max()))
        if on_view is not None:
            on_view(frame)
        time.sleep(0.004)
    elapsed = time.time() - t0
    _summarize(counts, shapes, depth_rng, ir_stats, elapsed)
    return 0 if all(counts.values()) else 1


def _make_viewer():
    import cv2

    def view(frame):
        if frame.rgb is not None:
            cv2.imshow("kinect rgb", cv2.resize(frame.rgb, (640, 360)))
        if frame.depth is not None:
            # 0-8 m mapped to 0-255 for display.
            d = np.clip(frame.depth / 8.0, 0, 1)
            cv2.imshow("kinect depth", (d * 255).astype(np.uint8))
        if frame.ir is not None:
            # IR is 16-bit; gamma-stretch the low end so the room is visible.
            ir = np.sqrt(frame.ir.astype(np.float32) / 65535.0)
            cv2.imshow("kinect ir", (ir * 255).astype(np.uint8))
        cv2.waitKey(1)

    return view


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Kinect v2 stream probe.")
    p.add_argument("--seconds", type=float, default=6.0,
                   help="how long to poll (default 6)")
    p.add_argument("--view", action="store_true",
                   help="show rgb/depth/ir windows while polling")
    p.add_argument("--manager", action="store_true",
                   help="route through SensorManager (the live-system path) "
                        "instead of reading the sensor directly")
    args = p.parse_args(argv)

    print("=" * 56)
    print("  Kinect v2 probe")
    print(f"  pykinect2 binding available: {pykinect2_available()}")
    print("=" * 56)
    if not pykinect2_available():
        print("FAIL: pykinect2 not importable — see README 'Kinect v2' notes.")
        return 1

    on_view = _make_viewer() if args.view else None

    if args.manager:
        from sensors.sensor_manager import SensorManager
        mgr = SensorManager()
        if not mgr.add_sensor(KinectV2Sensor()):
            print("FAIL: SensorManager could not add the Kinect")
            return 1
        print("Kinect added to SensorManager - polling latest_frame()...")
        try:
            rc = _poll(lambda: mgr.latest_frame("kinect_v2"),
                       args.seconds, on_view)
        finally:
            mgr.stop_all()
    else:
        kinect = KinectV2Sensor()
        if not kinect.open():
            print("FAIL: KinectV2Sensor.open() returned False")
            return 1
        print(f"open OK - polling read() for {args.seconds:.0f}s...")
        try:
            rc = _poll(kinect.read, args.seconds, on_view)
        finally:
            kinect.close()

    if on_view is not None:
        import cv2
        cv2.destroyAllWindows()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
