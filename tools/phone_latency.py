#!/usr/bin/env python3
"""phone_latency.py — measure command->seen latency of the phone feed.

The phone twin of `tools/gopro_latency.py` and `tools/kinect_latency.py`. Each
trial captures a laser-OFF reference, measures the laser-OFF diff-peak noise
floor, commands a centred laser dot, then reads phone frames until the diff
peak jumps WELL above floor and STAYS there for two frames.

Reports command -> laser on -> phone sensor -> phone ISP -> encode -> UDP ->
PC decode -> numpy frame. This is the metric that decides whether the phone
path can compete with the OV9281 (PHONE_SENSOR_BOOTSTRAP.md §7).

>>> Class 3B laser. Eye protection + safety lens REQUIRED. <<<

    python tools/phone_latency.py
    python tools/phone_latency.py --mode raw_yuv --phone-camera telephoto
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

from config import settings                                       # noqa: E402
from laser.lasercube import LaserCubeInterface                    # noqa: E402
from laser.types import LaserPoint                                # noqa: E402
from phone_sensor.client import PhoneSensorClient                 # noqa: E402
from sensors.phone import PhoneSensor                             # noqa: E402
from targeting.patterns import galvo_norm_to_dac                  # noqa: E402


def _diff_peak(frame_bgr, background_bgr) -> float:
    diff = cv2.absdiff(frame_bgr, background_bgr)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, peak, _, _ = cv2.minMaxLoc(cv2.GaussianBlur(gray, (9, 9), 0))
    return float(peak)


def _read_rgb(sensor: PhoneSensor, timeout_s: float = 0.5):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = sensor.read()
        if frame is not None and frame.rgb is not None:
            return frame.rgb
        time.sleep(0.002)
    return None


def _capture_background(cube, sensor, blank, hold_s: float):
    start = time.monotonic()
    frames = []
    while time.monotonic() - start < hold_s + 1.0:
        cube.send_frame(blank, frame_num=0)
        image = _read_rgb(sensor, timeout_s=0.02)
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
    ap.add_argument("--phone-ip", default=settings.PHONE_IP)
    ap.add_argument("--phone-camera",
                    choices=["ultrawide", "main", "telephoto"],
                    default=settings.PHONE_DEFAULT_CAMERA)
    ap.add_argument("--mode",
                    choices=["raw_yuv", "h264_lowlat", "h264_quality"],
                    default=settings.PHONE_DEFAULT_STREAM_MODE)
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--laser-power", type=float, default=30.0)
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args(argv)

    print("=" * 60)
    print("  Phone command->seen latency probe")
    print("  Class 3B laser. Eye protection + safety lens REQUIRED.")
    print(f"  camera: phone_{args.phone_camera}   mode: {args.mode}")
    print("=" * 60)

    cube = LaserCubeInterface(ip=args.ip, src_ip=args.src_ip)
    if not cube.connect():
        print("FAIL: could not connect to the cube")
        return 1
    print(f"connected (src_ip={cube.src_ip})")

    client = PhoneSensorClient(host=args.phone_ip)
    sensor = PhoneSensor(client=client, active_camera=args.phone_camera,
                         stream_mode=args.mode)
    if not sensor.open():
        cube.disconnect()
        print("FAIL: could not open the phone")
        return 1
    print(f"camera: {sensor.sensor_id} opened")

    if input("Fire the laser for the latency probe? [type 'yes'] "
             ).strip().lower() != "yes":
        sensor.close()
        cube.disconnect()
        print("aborted by operator")
        return 0

    lp = max(0, min(0xFFF, int(round(0xFFF * args.laser_power / 100.0))))
    gc = galvo_norm_to_dac(0.5)
    dwell = [LaserPoint(x=gc, y=gc, r=lp, g=lp, b=lp)] * 3000
    blank = [LaserPoint(x=gc, y=gc, r=0, g=0, b=0)] * 3000

    latencies: list = []
    try:
        cube.enable_output()
        print(f"\nLASER ON - {args.trials} trials at "
              f"{args.laser_power:.0f}%\n")
        for trial in range(1, args.trials + 1):
            background = _capture_background(cube, sensor, blank, hold_s=1.5)
            if background is None:
                print(f"  trial {trial:>2}: no background - skipped")
                continue

            floor = 0.0
            for _ in range(12):
                cube.send_frame(blank, frame_num=0)
                image = _read_rgb(sensor, timeout_s=0.08)
                if image is not None:
                    floor = max(floor, _diff_peak(image, background))
            threshold = max(floor + 60.0, 120.0)

            time.sleep(0.25)
            t_cmd = time.monotonic()
            t_seen = None
            over_since = None
            while time.monotonic() - t_cmd < args.timeout:
                cube.send_frame(dwell, frame_num=0)
                image = _read_rgb(sensor, timeout_s=0.02)
                if image is None:
                    continue
                now = time.monotonic()
                if _diff_peak(image, background) > threshold:
                    if over_since is None:
                        over_since = now
                    else:
                        t_seen = over_since
                        break
                else:
                    over_since = None

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
        sensor.close()
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
