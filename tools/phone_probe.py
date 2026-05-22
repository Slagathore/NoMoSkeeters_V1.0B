#!/usr/bin/env python3
"""phone_probe.py — confirm the phone is reachable and streaming.

Connects to the phone over TCP, prints the capabilities manifest (which lenses
the phone reports), starts a stream from the chosen lens, then reads decoded
frames for a few seconds and reports the achieved fps / resolution. The phone
twin of `tools/kinect_probe.py`.

    python tools/phone_probe.py
    python tools/phone_probe.py --phone-camera telephoto --mode h264_lowlat
    python tools/phone_probe.py --view --seconds 10
    python tools/phone_probe.py --manager

If the Android companion app isn't running yet, this prints an honest TCP
connect failure rather than hanging. Network only; no laser, no admin.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Callable, Optional

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from config import settings                                       # noqa: E402
from phone_sensor.client import PhoneSensorClient                 # noqa: E402
from sensors.phone import PhoneSensor                             # noqa: E402


def _print_manifest(caps) -> None:
    if caps is None:
        print("  (no capabilities — handshake returned empty manifest)")
        return
    print(f"  phone: {caps.phone_model}  protocol v{caps.protocol_version}")
    for cam in caps.cameras:
        zoom = f"  zoom={cam.optical_zoom_factor:.1f}x" if cam.has_optical_zoom else ""
        print(f"    - {cam.id:10s}  fov_h={cam.fov_h_deg:5.1f}deg  "
              f"max={cam.max_resolution[0]}x{cam.max_resolution[1]}  "
              f"fps={cam.max_fps_at_streaming_res}{zoom}")


def _poll(read, seconds: float, on_view=None) -> tuple:
    count = 0
    shape = dtype = None
    t0 = time.time()
    while time.time() - t0 < seconds:
        frame = read()
        if frame is None:
            time.sleep(0.004)
            continue
        if frame.rgb is None:
            continue
        count += 1
        shape = frame.rgb.shape
        dtype = str(frame.rgb.dtype)
        if on_view is not None:
            on_view(frame.rgb)
    elapsed = time.time() - t0
    return count, shape, dtype, elapsed


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Phone-as-sensor probe.")
    p.add_argument("--phone-ip", default=settings.PHONE_IP)
    p.add_argument("--phone-camera",
                   choices=["ultrawide", "main", "telephoto"],
                   default=settings.PHONE_DEFAULT_CAMERA)
    p.add_argument("--mode",
                   choices=["raw_yuv", "h264_lowlat", "h264_quality"],
                   default=settings.PHONE_DEFAULT_STREAM_MODE)
    p.add_argument("--width", type=int,
                   default=settings.PHONE_DEFAULT_STREAM_WIDTH)
    p.add_argument("--height", type=int,
                   default=settings.PHONE_DEFAULT_STREAM_HEIGHT)
    p.add_argument("--fps", type=int,
                   default=settings.PHONE_DEFAULT_STREAM_FPS)
    p.add_argument("--seconds", type=float, default=6.0)
    p.add_argument("--view", action="store_true",
                   help="show a live preview window")
    p.add_argument("--manager", action="store_true",
                   help="route through SensorManager (the live-system path)")
    p.add_argument("--switch-camera", default=None,
                   help="after the initial poll, switch to this lens and "
                        "poll again (exercises the lens-switch dance)")
    args = p.parse_args(argv)

    print("=" * 60)
    print("  Phone-as-sensor probe")
    print(f"  TCP {args.phone_ip}:{settings.PHONE_CMD_PORT}  "
          f"UDP frame :{settings.PHONE_FRAME_PORT}")
    print("=" * 60)

    on_view: Optional[Callable[[Any], None]] = None
    if args.view:
        import cv2

        def _show_view(rgb):
            cv2.imshow("phone preview", cv2.resize(rgb, (960, 540)))
            cv2.waitKey(1)

        on_view = _show_view

    client = PhoneSensorClient(host=args.phone_ip)
    sensor = PhoneSensor(client=client, active_camera=args.phone_camera,
                         stream_mode=args.mode, stream_width=args.width,
                         stream_height=args.height, stream_fps=args.fps)

    if args.manager:
        from sensors.sensor_manager import SensorManager
        mgr = SensorManager()
        if not mgr.add_sensor(sensor):
            print("FAIL: SensorManager could not add the phone")
            return 1
        print("  capabilities:")
        _print_manifest(client.capabilities)
        try:
            print(f"\npolling latest_frame() for {args.seconds:.0f}s...")
            n, shp, dt, elapsed = _poll(
                lambda: mgr.latest_frame(sensor.sensor_id),
                args.seconds, on_view)
            print(f"  {n} frames  {n/elapsed:.1f} fps  shape={shp} {dt}")
        finally:
            mgr.stop_all()
    else:
        if not sensor.open():
            print("FAIL: PhoneSensor.open() returned False — is the "
                  "companion app running and reachable?")
            return 1
        print("  capabilities:")
        _print_manifest(client.capabilities)
        try:
            print(f"\npolling read() for {args.seconds:.0f}s "
                  f"(camera={sensor.active_camera}, mode={args.mode})...")
            n, shp, dt, elapsed = _poll(sensor.read, args.seconds, on_view)
            print(f"  {n} frames  {n/elapsed:.1f} fps  shape={shp} {dt}")

            if args.switch_camera:
                print(f"\nswitching to {args.switch_camera}...")
                if not sensor.switch_camera(args.switch_camera):
                    print(f"FAIL: switch_camera({args.switch_camera})")
                    return 1
                # Lens switches take 300-800 ms; give it a beat to settle.
                time.sleep(1.0)
                n2, shp2, dt2, e2 = _poll(sensor.read, args.seconds, on_view)
                print(f"  {n2} frames  {n2/e2:.1f} fps  shape={shp2}  "
                      f"(camera={sensor.active_camera})")
        finally:
            sensor.close()

    if on_view is not None:
        import cv2
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
