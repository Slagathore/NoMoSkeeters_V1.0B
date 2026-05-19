#!/usr/bin/env python3
"""kinect_latency.py — measure command->seen latency of the Kinect feed.

This is the Kinect twin of tools/gopro_latency.py. Each trial captures a
laser-OFF reference, measures the laser-OFF diff-peak noise floor, commands a
centered laser dot, then reads Kinect frames until the selected stream's diff
peak jumps WELL above that floor and STAYS there for two frames.

The reported delta is command -> laser on -> Kinect sensor -> SDK delivery ->
KinectV2Sensor.read().

RGB is the practical default because the visible laser dot should separate
cleanly there. IR and depth are available for experiments, but they may not
respond to the visible dot in a useful way.

>>> Class 3B laser. Eye protection + safety lens REQUIRED. <<<

    python tools/kinect_latency.py
    python tools/kinect_latency.py --stream ir --trials 5
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
from sensors.kinect_v2 import KinectV2Sensor, pykinect2_available  # noqa: E402
from targeting.patterns import galvo_norm_to_dac                  # noqa: E402


def _stream_image(frame, stream: str):
    arr = getattr(frame, stream)
    if arr is None:
        return None
    if stream == "rgb":
        return arr
    if stream == "ir":
        # Kinect IR is 16-bit. Gamma-stretch to 8-bit before using the same
        # diff-peak detector as the GoPro probe.
        ir = np.sqrt(np.clip(arr.astype(np.float32) / 65535.0, 0.0, 1.0))
        gray = (ir * 255.0).astype(np.uint8)
    else:
        # Depth is metres float32. Map 0-8 m to 8-bit for temporal diff.
        depth = np.clip(arr.astype(np.float32) / 8.0, 0.0, 1.0)
        gray = (depth * 255.0).astype(np.uint8)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def _diff_peak(frame_bgr, background_bgr) -> float:
    """Brightest point of the blurred temporal difference."""
    diff = cv2.absdiff(frame_bgr, background_bgr)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, peak, _, _ = cv2.minMaxLoc(cv2.GaussianBlur(gray, (9, 9), 0))
    return float(peak)


def _read_stream(cam: KinectV2Sensor, stream: str, timeout_s: float = 1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = cam.read()
        if frame is None:
            time.sleep(0.004)
            continue
        image = _stream_image(frame, stream)
        if image is not None:
            return image
    return None


def _capture_background(cube, cam, stream: str, blank, hold_s: float):
    """Blank the laser for hold_s, then average five selected-stream frames."""
    start = time.monotonic()
    frames = []
    while time.monotonic() - start < hold_s + 1.0:
        cube.send_frame(blank, frame_num=0)
        image = _read_stream(cam, stream, timeout_s=0.02)
        if image is not None and time.monotonic() - start >= hold_s:
            frames.append(image.astype(np.float32))
            if len(frames) >= 5:
                break
    return np.mean(frames, axis=0).astype(np.uint8) if frames else None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="169.254.40.83", help="LaserCube IP")
    ap.add_argument("--src-ip", default="auto")
    ap.add_argument("--stream", choices=["rgb", "ir", "depth"],
                    default="rgb", help="Kinect stream to watch")
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--laser-power", type=float, default=30.0,
                    help="percent — a bright dot separates cleanly from noise")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args(argv)

    print("=" * 60)
    print("  Kinect command->seen latency probe")
    print("  Class 3B laser. Eye protection + safety lens REQUIRED.")
    print(f"  stream: {args.stream}")
    print("=" * 60)
    print(f"pykinect2 binding available: {pykinect2_available()}")
    if not pykinect2_available():
        print("FAIL: pykinect2 not importable — see README 'Kinect v2' notes.")
        return 1

    cube = LaserCubeInterface(ip=args.ip, src_ip=args.src_ip)
    if not cube.connect():
        print("FAIL: could not connect to the cube")
        return 1
    print(f"connected (src_ip={cube.src_ip})")

    cam = KinectV2Sensor()
    if not cam.open():
        cube.disconnect()
        print("FAIL: could not open the Kinect")
        return 1
    print("camera: kinect_v2 opened")

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
        print(f"\nLASER ON — {args.trials} trials at "
              f"{args.laser_power:.0f}%\n")
        for trial in range(1, args.trials + 1):
            background = _capture_background(
                cube, cam, args.stream, blank, hold_s=1.5)
            if background is None:
                print(f"  trial {trial:>2}: no background — skipped")
                continue

            # Laser-OFF noise floor: peak diff with nothing changing.
            floor = 0.0
            for _ in range(12):
                cube.send_frame(blank, frame_num=0)
                image = _read_stream(cam, args.stream, timeout_s=0.08)
                if image is not None:
                    floor = max(floor, _diff_peak(image, background))
            threshold = max(floor + 60.0, 120.0)

            # Let the cube ringbuffer drain so dwell is not queued behind
            # blank samples, then fire and time.
            time.sleep(0.25)
            t_cmd = time.monotonic()
            t_seen = None
            over_since = None
            while time.monotonic() - t_cmd < args.timeout:
                cube.send_frame(dwell, frame_num=0)
                image = _read_stream(cam, args.stream, timeout_s=0.02)
                if image is None:
                    continue
                now = time.monotonic()
                if _diff_peak(image, background) > threshold:
                    if over_since is None:
                        over_since = now            # provisional first sight
                    else:
                        t_seen = over_since          # persisted => real
                        break
                else:
                    over_since = None                # noise blip; reset

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
    print(f"\n  -> calibration --settle ~{(mean + 3 * spread) / 1000:.1f}s "
          f"(mean + 3*stdev)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
