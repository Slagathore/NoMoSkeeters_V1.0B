#!/usr/bin/env python3
"""gopro_latency.py — measure the command->seen latency of the GoPro feed.

Each trial: capture a laser-OFF reference, measure the laser-OFF diff-peak
noise floor, then issue the laser-ON command (timestamped) and read camera
frames until the diff peak jumps WELL above that noise floor and STAYS
there for two frames. The delta is the end-to-end latency — command ->
laser on -> camera sensor -> H.264 encode -> USB -> ffmpeg decode -> read().

The "above noise floor + persists 2 frames" rule matters: a single noisy
frame can spike the diff on its own, which would mis-report a near-zero
latency. The real laser dot is bright and steady.

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

import cv2                                                       # noqa: E402
import numpy as np                                               # noqa: E402

from laser.lasercube import LaserCubeInterface                    # noqa: E402
from laser.types import LaserPoint                                # noqa: E402
from sensors.gopro import GoProSensor                             # noqa: E402
from sensors.gopro_interface import GoProInterface                # noqa: E402
from targeting.patterns import galvo_norm_to_dac                  # noqa: E402


def _diff_peak(frame_bgr, background_bgr) -> float:
    """Brightest point of the (blurred) temporal difference."""
    diff = cv2.absdiff(frame_bgr, background_bgr)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, peak, _, _ = cv2.minMaxLoc(cv2.GaussianBlur(gray, (9, 9), 0))
    return float(peak)


def _capture_background(cube, cam, blank, hold_s):
    """Blank the laser for hold_s (clears the pipeline), average 5 frames."""
    start = time.monotonic()
    frames = []
    while time.monotonic() - start < hold_s + 1.0:
        cube.send_frame(blank, frame_num=0)
        f = cam.read()
        if (f is not None and f.rgb is not None
                and time.monotonic() - start >= hold_s):
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
                    help="percent — a bright dot separates cleanly from noise")
    ap.add_argument("--timeout", type=float, default=5.0)
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

    latencies: list[float] = []
    try:
        cube.enable_output()
        print(f"\nLASER ON — {args.trials} trials at {args.laser_power:.0f}%\n")
        for trial in range(1, args.trials + 1):
            background = _capture_background(cube, cam, blank, hold_s=1.5)
            if background is None:
                print(f"  trial {trial:>2}: no background — skipped")
                continue

            # Laser-OFF noise floor: peak diff with nothing changing.
            floor = 0.0
            for _ in range(12):
                cube.send_frame(blank, frame_num=0)
                f = cam.read()
                if f is not None and f.rgb is not None:
                    floor = max(floor, _diff_peak(f.rgb, background))
            threshold = max(floor + 60.0, 120.0)

            # Let the cube ringbuffer drain so the dwell isn't queued behind
            # a backlog of blank samples, then fire and time.
            time.sleep(0.25)
            t_cmd = time.monotonic()
            t_seen = None
            over_since = None
            while time.monotonic() - t_cmd < args.timeout:
                cube.send_frame(dwell, frame_num=0)
                f = cam.read()
                if f is None or f.rgb is None:
                    continue
                now = time.monotonic()
                if _diff_peak(f.rgb, background) > threshold:
                    if over_since is None:
                        over_since = now            # provisional first sight
                    else:
                        t_seen = over_since          # persisted ⇒ real
                        break
                else:
                    over_since = None                # noise blip — reset

            if t_seen is None:
                print(f"  trial {trial:>2}: TIMEOUT  (floor {floor:.0f}, "
                      f"thresh {threshold:.0f})")
            else:
                ms = (t_seen - t_cmd) * 1000.0
                latencies.append(ms)
                print(f"  trial {trial:>2}: {ms:6.0f} ms   "
                      f"(floor {floor:.0f}, thresh {threshold:.0f})")
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
    print(f"\n  → calibration --settle ~{(mean + 3 * spread) / 1000:.1f}s "
          f"(mean + 3*stdev)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
