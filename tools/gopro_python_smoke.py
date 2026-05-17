#!/usr/bin/env python3
"""
gopro_python_smoke.py — verify OpenCV can pull frames from the GoPro
preview stream after Start-GoProStream has been run.

PREREQUISITE: ffmpeg/ffplay must NOT be bound to port 8554 (single-consumer
rule). Use Start-GoProStream -Mode headless or close any open ffplay first.
Then start the stream fresh and run this within ~10 seconds.
"""

import cv2
import os
import time

# Critical: set these BEFORE creating VideoCapture.
# Forces ffmpeg backend to use low-buffer / drop-on-overrun behavior.
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "fflags;nobuffer|flags;low_delay|"
    "framedrop;1|"
    "rtbufsize;100M|"
    "overrun_nonfatal;1"
)

URL = "udp://0.0.0.0:8554"  # explicit IPv4 — udp://@:8554 binds IPv6 on Windows

print(f"Opening {URL} ...")
cap = cv2.VideoCapture(URL, cv2.CAP_FFMPEG)

if not cap.isOpened():
    print("FAIL: VideoCapture.isOpened() returned False")
    raise SystemExit(1)

# Try for up to 15 seconds to receive at least one frame.
# UDP streams take a beat to lock on; first frame is usually 1-3s after open.
deadline = time.monotonic() + 15
first_frame_at = None
frame_count = 0
last_print = time.monotonic()

while time.monotonic() < deadline:
    ret, frame = cap.read()
    now = time.monotonic()
    if ret:
        if first_frame_at is None:
            first_frame_at = now
            print(f"First frame received {first_frame_at - (deadline - 15):.2f}s "
                  f"after capture open. Shape={frame.shape}, dtype={frame.dtype}")
        frame_count += 1
        if now - last_print > 2.0:
            elapsed = now - first_frame_at
            print(f"  {frame_count} frames in {elapsed:.1f}s "
                  f"= {frame_count/elapsed:.1f} fps")
            last_print = now

cap.release()

if frame_count == 0:
    print("FAIL: zero frames received in 15s. Likely causes:")
    print("  1. Stream not actually running on camera "
          "(check /gopro/camera/state field 32)")
    print("  2. Another process bound to UDP 8554 "
          "(check netstat -ano | findstr 8554)")
    print("  3. OpenCV's ffmpeg backend can't decode HEVC "
          "(try ffmpeg -i udp://0.0.0.0:8554 -t 5 -f null - to verify)")
    raise SystemExit(2)

total_elapsed = time.monotonic() - first_frame_at
print(f"\nPASS: {frame_count} frames in {total_elapsed:.1f}s "
      f"= {frame_count/total_elapsed:.1f} fps")
