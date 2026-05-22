#!/usr/bin/env python3
"""gp_py_smoke_subprocess.py — end-to-end smoke test of the GoPro video feed.

Starts the Hero 13 in webcam mode, opens FfmpegStreamCapture (the exact
decoder the targeting pipeline uses), and reports decoded frames. A PASS
here means the live video source is good to go.

No laser involved. Requires the camera USB-tethered and reachable.

    python gp_py_smoke_subprocess.py [--ip 172.27.109.51]
"""
import argparse
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sensors.ffmpeg_capture import FfmpegStreamCapture, local_ip_for  # noqa: E402


def http_get(ip: str, path: str):
    try:
        with urllib.request.urlopen(f"http://{ip}:8080{path}", timeout=8) as r:
            return r.status
    except Exception as exc:                                  # noqa: BLE001
        return f"ERR {exc}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ip", default="172.27.109.51", help="camera HTTP IP")
    ap.add_argument("--seconds", type=int, default=12)
    args = ap.parse_args(argv)

    print(f"camera {args.ip}; host interface {local_ip_for(args.ip)}")
    print("webcam/start ->", http_get(args.ip, "/gopro/webcam/start"))

    frames = 0
    first = None
    try:
        cap = FfmpegStreamCapture(camera_ip=args.ip)
        if not cap.isOpened():
            print("FAIL: FfmpegStreamCapture did not open")
            return 1
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue                       # feed warming up — keep trying
            if first is None:
                first = time.monotonic()
                print(f"first frame: shape={frame.shape}, dtype={frame.dtype}")
            frames += 1
        cap.release()
    finally:
        http_get(args.ip, "/gopro/webcam/stop")
        http_get(args.ip, "/gopro/webcam/exit")

    if not frames or first is None:
        print("FAIL: zero frames decoded")
        return 2
    elapsed = time.monotonic() - first
    print(f"PASS: {frames} frames in {elapsed:.1f}s = {frames / elapsed:.1f} fps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
