#!/usr/bin/env python3
"""gopro_view.py — live preview window for the GoPro webcam feed.

Starts the camera in webcam mode, opens FfmpegStreamCapture, and shows the
decoded video in an OpenCV window with a running FPS readout. Press q or Esc
to quit. No laser involved — this is just the camera.

    python tools/gopro_view.py --ip 172.27.109.51 --fov linear
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2                                                       # noqa: E402

from sensors.gopro import GoProSensor                            # noqa: E402
from sensors.gopro_interface import GoProInterface               # noqa: E402

# Webcam FOV codes: 0=Wide (fisheye), 4=Linear (de-warped), 3=Superview.
_FOV_CODES = {"default": None, "wide": 0, "linear": 4, "superview": 3}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ip", default="172.27.109.51", help="camera HTTP IP")
    ap.add_argument("--fov", choices=list(_FOV_CODES), default="default",
                    help="webcam field of view; 'linear' removes the "
                         "fisheye warp (some firmware ignores it)")
    ap.add_argument("--record", default=None, metavar="FILE.ts",
                    help="also save the raw camera stream to this .ts file "
                         "(remux to mp4 later: ffmpeg -i x.ts -c copy x.mp4)")
    args = ap.parse_args(argv)

    cam = GoProSensor(
        control=GoProInterface(ip=args.ip, webcam_fov=_FOV_CODES[args.fov]),
        decoder="ffmpeg", record_path=args.record)
    if not cam.open():
        print("FAIL: could not open the GoPro feed — is it USB-tethered?")
        return 1
    print("GoPro feed open. Press q or Esc in the window to quit.")
    if args.record:
        print(f"recording to {args.record}")

    frames = 0
    fps = 0.0
    window = time.monotonic()
    start = time.monotonic()
    first = None
    try:
        while True:
            frame = cam.read()
            if frame is None or frame.rgb is None:
                # Bail with a diagnostic rather than spinning silently if the
                # webcam never starts delivering.
                if first is None and time.monotonic() - start > 15:
                    print("FAIL: no frames after 15s. The webcam may need a "
                          "stop/exit before it restarts — re-run this, or "
                          "check tools/gopro_stream_probe.py.")
                    return 2
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
                continue
            first = first or time.monotonic()
            frames += 1
            now = time.monotonic()
            if now - window >= 1.0:
                fps = frames / (now - window)
                frames = 0
                window = now
            view = frame.rgb
            cv2.putText(view,
                        f"{fps:5.1f} fps  {view.shape[1]}x{view.shape[0]}",
                        (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0), 2)
            cv2.imshow("GoPro feed - q/Esc to quit", view)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        cam.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
