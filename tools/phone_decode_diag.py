#!/usr/bin/env python3
"""phone_decode_diag.py — isolate the ffmpeg decode stage.

The wire diag (phone_frame_diag.py) proved frames arrive intact at ~57fps, yet
the probe decodes ~1 frame. That points at the ffmpeg decode stage. This tool
runs the REAL PhoneFrameDecoder with ffmpeg stderr surfaced (logging=DEBUG) and
counts decoded frames, so we can see what ffmpeg complains about — and toggle
hwaccel to tell a CUDA/NVDEC problem from a stream problem.

It also tees the raw H.264 to a file so the stream can be analysed offline
(`ffprobe phone_capture.h264`, `ffmpeg -i phone_capture.h264 -f null -`).

    python tools/phone_decode_diag.py                 # uses settings hwaccel (cuda)
    python tools/phone_decode_diag.py --hwaccel none  # force software decode
    python tools/phone_decode_diag.py --no-record     # skip the raw capture file

Network only; no laser, no admin.
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from config import settings                                       # noqa: E402
from phone_sensor.client import PhoneSensorClient                 # noqa: E402
from phone_sensor.frame_decoder import PhoneFrameDecoder          # noqa: E402


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Phone ffmpeg-decode diagnosis.")
    p.add_argument("--phone-ip", default=settings.PHONE_IP)
    p.add_argument("--phone-camera", default=settings.PHONE_DEFAULT_CAMERA)
    p.add_argument("--mode", default=settings.PHONE_DEFAULT_STREAM_MODE)
    p.add_argument("--width", type=int, default=settings.PHONE_DEFAULT_STREAM_WIDTH)
    p.add_argument("--height", type=int, default=settings.PHONE_DEFAULT_STREAM_HEIGHT)
    p.add_argument("--fps", type=int, default=settings.PHONE_DEFAULT_STREAM_FPS)
    p.add_argument("--seconds", type=float, default=6.0)
    p.add_argument("--hwaccel", default=settings.PHONE_FFMPEG_HWACCEL or "none",
                   help="cuda | none (default: settings value)")
    p.add_argument("--no-record", action="store_true")
    args = p.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    hwaccel = None if args.hwaccel.lower() == "none" else args.hwaccel
    record = None if args.no_record else str(ROOT / "phone_capture.h264")

    print("=" * 60)
    print("  Phone ffmpeg-decode diagnosis")
    print(f"  hwaccel={hwaccel}  record={record}")
    print("=" * 60)

    client = PhoneSensorClient(host=args.phone_ip)
    if not client.connect():
        print("FAIL: TCP connect")
        return 1
    if not client.set_active_camera(args.phone_camera):
        print("FAIL: set_active_camera")
        client.disconnect()
        return 1
    negotiated = client.stream_start(mode=args.mode, width=args.width,
                                     height=args.height, fps=args.fps)
    if negotiated is None:
        print("FAIL: stream_start")
        client.disconnect()
        return 1

    dec = PhoneFrameDecoder(width=args.width, height=args.height,
                            codec="h264", hwaccel=hwaccel, record_path=record)
    count = 0
    last_report = time.time()
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds:
            ok, frame = dec.read()
            if ok and frame is not None:
                count += 1
            if not dec.isOpened():
                print("  !! decoder reports not open (ffmpeg died?)")
                break
            if time.time() - last_report >= 1.0:
                print(f"  t+{time.time()-t0:4.1f}s  decoded {count} frames")
                last_report = time.time()
    finally:
        dec.release()
        try:
            client.stream_stop()
        except Exception:
            pass
        client.disconnect()

    elapsed = time.time() - t0
    print(f"\n  decoded {count} frames in {elapsed:.1f}s "
          f"({count/elapsed:.1f} fps)")
    if record:
        sz = Path(record).stat().st_size if Path(record).exists() else 0
        print(f"  raw H.264 captured: {record} ({sz/1e6:.2f} MB)")
        print("  analyse offline:")
        print(f"    ffprobe -v error -show_frames -select_streams v "
              f"-show_entries frame=pict_type {record} | findstr pict_type")
        print(f"    ffmpeg -i {record} -f null -")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
