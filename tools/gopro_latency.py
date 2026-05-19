#!/usr/bin/env python3
"""gopro_latency.py — measure the command->seen latency of the GoPro feed.

Each trial: blank the laser and capture a laser-OFF reference, then issue
the laser-ON command (timestamped) and read camera frames until the dot
appears in the temporal diff. The delta is the full end-to-end latency —
command -> laser on -> camera sensor -> encode -> USB -> ffmpeg decode ->
read(). Repeated for a mean and spread.

This is what `--settle` in the calibration has to cover, and what lead-aim
targeting has to compensate for.

>>> Class 3B laser. Eye protection + safety lens REQUIRED. <<<

    python tools/gopro_latency.py --gopro-ip 172.27.109.51
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np                                               # noqa: E402

from config import settings                                      # noqa: E402
from laser.lasercube import LaserCubeInterface                    # noqa: E402
from laser.types import LaserPoint                                # noqa: E402
from sensors.gopro import GoProSensor                             # noqa: E402
from sensors.gopro_interface import GoProInterface                # noqa: E402
from targeting.dot_detector import LaserDotDetector               # noqa: E402
from targeting.patterns import galvo_norm_to_dac                  # noqa: E402


def _capture_background(cube, cam, blank, settle_s):
    """Blank the laser long enough to clear the pipeline, then average a
    few laser-OFF frames into a reference."""
    start = time.monotonic()
    frames = []
    while time.monotonic() - start < settle_s + 1.0:
        cube.send_frame(blank, frame_num=0)
        f = cam.read()
        if (f is not None and f.rgb is not None
                and time.monotonic() - start >= settle_s):
            frames.append(f.rgb.astype(np.float32))
            if len(frames) >= 5:
                break
    return np.mean(frames, axis=0).astype(np.uint8) if frames else None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="169.254.40.83", help="LaserCube IP")
    ap.add_argument("--src-ip", default="auto")
    ap.add_argument("--gopro-ip", default="172.27.109.51")
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--laser-power", type=float, default=30.0,
                    help="percent — a clear, bright dot times cleanly")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="give up on a trial after this many seconds")
    args = ap.parse_args(argv)

    print("=" * 60)
    print("  GoPro command->seen latency probe")
    print("  Class 3B laser. Eye protection + safety lens REQUIRED.")
    print("=" * 60)

    cube = LaserCubeInterface(ip=args.ip, src_ip=args.src_ip)
    if not cube.connect():
        print("FAIL: could not connect to the cube")
        return 1
    print(f"connected (src_ip={cube.src_ip})")
    cam = GoProSensor(control=GoProInterface(ip=args.gopro_ip),
                      decoder="ffmpeg")
    if not cam.open():
        print("FAIL: could not open the GoPro feed")
        cube.disconnect()
        return 1
    print("camera: gopro opened")

    if input("Fire the laser for the latency probe? [type 'yes'] "
             ).strip().lower() != "yes":
        cam.close()
        cube.disconnect()
        print("aborted by operator")
        return 0

    lp = max(0, min(0xFFF, int(round(0xFFF * args.laser_power / 100.0))))
    gc = galvo_norm_to_dac(0.5)                  # dot at galvo centre
    dwell = [LaserPoint(x=gc, y=gc, r=lp, g=lp, b=lp)] * 3000
    blank = [LaserPoint(x=gc, y=gc, r=0, g=0, b=0)] * 3000
    detector = LaserDotDetector()
    settle_s = settings.CALIBRATION_DWELL_SETTLE_S

    latencies: list[float] = []
    try:
        cube.enable_output()
        print(f"\nLASER ON — {args.trials} trials at {args.laser_power:.0f}%\n")
        for trial in range(1, args.trials + 1):
            background = _capture_background(cube, cam, blank, settle_s)
            if background is None:
                print(f"  trial {trial:>2}: no background frame — skipped")
                continue
            # Fire and time: hold the dot lit, read until the diff shows it.
            t_cmd = time.monotonic()
            t_seen = None
            while time.monotonic() - t_cmd < args.timeout:
                cube.send_frame(dwell, frame_num=0)
                f = cam.read()
                if (f is not None and f.rgb is not None
                        and detector.detect_diff(f.rgb, background) is not None):
                    t_seen = time.monotonic()
                    break
            if t_seen is None:
                print(f"  trial {trial:>2}: TIMEOUT — no dot in {args.timeout}s")
            else:
                ms = (t_seen - t_cmd) * 1000.0
                latencies.append(ms)
                print(f"  trial {trial:>2}: {ms:6.0f} ms")
    finally:
        cube.disable_output()
        print("LASER OFF")
        cam.close()
        cube.disconnect()
        print("disconnected")

    print("\n" + "-" * 60)
    if not latencies:
        print("no successful measurements")
        return 1
    mean = statistics.mean(latencies)
    spread = statistics.pstdev(latencies) if len(latencies) > 1 else 0.0
    print(f"command->seen latency over {len(latencies)} trials:")
    print(f"  mean {mean:.0f} ms   min {min(latencies):.0f}   "
          f"max {max(latencies):.0f}   stdev {spread:.0f}")
    print(f"\n  → calibration --settle could be ~{(mean + 3 * spread) / 1000:.1f}s "
          f"(mean + 3*stdev, with margin)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
