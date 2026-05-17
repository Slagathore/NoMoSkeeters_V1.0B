#!/usr/bin/env python3
"""gp_py_smoke_subprocess.py — smoke test for the subprocess-ffmpeg GoPro
decoder.

Exercises sensors/ffmpeg_capture.FfmpegStreamCapture — the very decoder that
GoProSensor(decoder="ffmpeg") feeds the targeting pipeline with. If this
PASSes, the subprocess decoder is good to use as the live video source.

PREREQUISITE: the GoPro preview stream must be running and nothing else
bound to UDP 8554 (single-consumer — HARDWARE_FINDINGS.md §3.2). Start it
with tools/gopro_stream_helper.ps1 (Start-GoProStream) first.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sensors.ffmpeg_capture import FfmpegStreamCapture   # noqa: E402

URL = "udp://0.0.0.0:8554"
DURATION_S = 10

print(f"Opening {URL} via subprocess ffmpeg ...")
cap = FfmpegStreamCapture(URL)
if not cap.isOpened():
    print("FAIL: ffmpeg subprocess did not start")
    raise SystemExit(1)

frame_count = 0
first_frame_at: float | None = None
start = time.monotonic()
deadline = start + DURATION_S

print(f"Reading frames for {DURATION_S} seconds...")
while time.monotonic() < deadline:
    ok, frame = cap.read()
    if not ok or frame is None:
        print("Stream ended or broke before the deadline")
        break
    if first_frame_at is None:
        first_frame_at = time.monotonic()
        print(f"First frame at {first_frame_at - start:.2f}s, "
              f"shape={frame.shape}, dtype={frame.dtype}")
    frame_count += 1

cap.release()

if frame_count == 0:
    print("FAIL: zero frames received. Check the stream is running and that "
          "nothing else holds UDP 8554 (netstat -ano | findstr 8554).")
    raise SystemExit(2)

assert first_frame_at is not None  # frame_count > 0 ⟹ set in the loop
elapsed = time.monotonic() - first_frame_at
print(f"PASS: {frame_count} frames in {elapsed:.1f}s "
      f"= {frame_count / elapsed:.1f} fps")
