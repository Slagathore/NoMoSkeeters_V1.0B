#!/usr/bin/env python3
"""ov9281_latency.py — measure command->seen latency of the OV9281 feed.

The OV9281 twin of `tools/phone_latency.py` / `tools/gopro_latency.py`. Each
trial captures a laser-OFF reference, measures the laser-OFF diff-peak noise
floor, commands a centred laser dot, then reads OV9281 frames until the diff
peak jumps WELL above floor and STAYS there for two frames.

Reports command -> laser on -> camera sensor -> MJPEG encode -> USB 2.0 ->
DSHOW -> cv2 decode -> numpy frame. This is the V1 ground-truth latency:
no network hop, no app pipeline, just sensor + USB. Expected to land well
under the phone's 194 ms and the GoPro's 810 ms.

>>> Class 3B laser. Eye protection + safety lens REQUIRED. <<<

    python tools/ov9281_latency.py
    python tools/ov9281_latency.py --width 1280 --height 800 --fps 120
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
from sensors.ov9281 import OV9281Sensor                           # noqa: E402
from targeting.patterns import galvo_norm_to_dac                  # noqa: E402


def _diff_peak(frame_bgr, background_bgr) -> float:
    diff = cv2.absdiff(frame_bgr, background_bgr)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, peak, _, _ = cv2.minMaxLoc(cv2.GaussianBlur(gray, (9, 9), 0))
    return float(peak)


def _read_rgb(sensor: OV9281Sensor, timeout_s: float = 0.1):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = sensor.read()
        if frame is not None and frame.rgb is not None:
            return frame.rgb
        time.sleep(0.0005)
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
    ap.add_argument("--index", type=int, default=settings.OV9281_DEVICE_INDEX,
                    help="cv2 device index (use ov9281_probe.py --list to find)")
    ap.add_argument("--backend", default=settings.OV9281_BACKEND,
                    choices=["auto", "dshow", "msmf", "v4l2", "any"])
    ap.add_argument("--width", type=int, default=settings.OV9281_WIDTH)
    ap.add_argument("--height", type=int, default=settings.OV9281_HEIGHT)
    ap.add_argument("--fps", type=int, default=settings.OV9281_FPS)
    ap.add_argument("--exposure", type=float, default=settings.OV9281_EXPOSURE)
    ap.add_argument("--gain", type=float, default=settings.OV9281_GAIN)
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--laser-power", type=float, default=30.0)
    ap.add_argument("--timeout", type=float, default=3.0)
    args = ap.parse_args(argv)

    print("=" * 60)
    print("  OV9281 command->seen latency probe")
    print("  Class 3B laser. Eye protection + safety lens REQUIRED.")
    print(f"  camera: ov9281 idx {args.index} @ "
          f"{args.width}x{args.height}@{args.fps}")
    print("=" * 60)

    cube = LaserCubeInterface(ip=args.ip, src_ip=args.src_ip)
    if not cube.connect():
        print("FAIL: could not connect to the cube")
        return 1
    print(f"connected (src_ip={cube.src_ip})")

    sensor = OV9281Sensor(
        device_index=args.index, backend=args.backend,
        width=args.width, height=args.height, fps=args.fps,
        auto_exposure=False, exposure=args.exposure, gain=args.gain)
    if not sensor.open():
        cube.disconnect()
        print(f"FAIL: could not open ov9281 idx {args.index} "
              f"(try `python tools/ov9281_probe.py --list`)")
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
            background = _capture_background(cube, sensor, blank, hold_s=1.0)
            if background is None:
                print(f"  trial {trial:>2}: no background - skipped")
                continue

            floor = 0.0
            for _ in range(12):
                cube.send_frame(blank, frame_num=0)
                image = _read_rgb(sensor, timeout_s=0.05)
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
