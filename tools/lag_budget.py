"""lag_budget.py — print the measured lag budget across the pipeline.

The system end-to-end lag is the sum of (a) camera-side pipeline
(exposure + transport + decode), (b) the per-frame processing (detect ->
track), and (c) the laser-side queue (cube ringbuffer + galvo settle).
This tool tallies the figures we've actually measured so far and shows
where the budget is being spent.

The data is pulled from project memory / observed numbers, not from a
live measurement — re-run the individual `*_latency.py` probes after any
hardware change.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from config import settings                                       # noqa: E402


def _print_row(name: str, value_ms: float, note: str = "") -> None:
    bar = "#" * max(1, int(round(value_ms / 5.0)))
    print(f"  {name:<28} {value_ms:>6.0f} ms  {bar}  {note}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--camera", choices=["ov9281", "phone", "gopro"],
                    default="ov9281")
    args = ap.parse_args(argv)

    if args.camera == "ov9281":
        e2e = 24.0
        camera_pipeline = 16.0    # exposure (≤5ms) + USB MJPEG + cv2 decode
        cube_side = 8.0            # ringbuffer-drain + galvo-settle, observed
        notes_e2e = "verified 2026-05-23, sinusoid not yet remeasured"
    elif args.camera == "phone":
        e2e = 194.0
        camera_pipeline = 180.0   # ISP + h264 encode + UDP + decode
        cube_side = 14.0
        notes_e2e = "verified 2026-05-22 (phone_latency.py, h264_lowlat)"
    else:   # gopro
        e2e = 810.0
        camera_pipeline = 780.0
        cube_side = 30.0
        notes_e2e = "verified 2026-05-18 (gopro_latency.py, webcam mode)"

    detect_track = 5.0            # MOG2 + Hungarian + Kalman update, sub-frame

    print("=" * 60)
    print(f"  lag budget — sensor = {args.camera}")
    print("=" * 60)
    _print_row("camera pipeline", camera_pipeline,
               "sensor exposure + transport + decode")
    _print_row("detect + track", detect_track,
               "MOG2 + assignment + Kalman update")
    _print_row("cube + galvo", cube_side,
               "ringbuffer drain + galvo mechanical settle")
    print("  " + "-" * 56)
    _print_row("total end-to-end", e2e, notes_e2e)
    print()
    cfg = settings.LATENCY_SOFTWARE_LAG_MS
    print(f"  settings.LATENCY_SOFTWARE_LAG_MS = {cfg:.0f} ms")
    if abs(cfg - e2e) > 30:
        print(f"  -> MISMATCH: set this to {e2e:.0f} for {args.camera} lead-aim")
    else:
        print("  -> matches the measured budget within ±30 ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
