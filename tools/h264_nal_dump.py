#!/usr/bin/env python3
"""h264_nal_dump.py — dump the NAL-unit structure of an Annex-B H.264 file.

Used to diagnose "sps_id out of range / non-existing PPS" decode failures: it
shows the start-code framing and NAL types as they actually appear on the wire,
so we can see whether SPS(7)/PPS(8)/IDR(5) are present, well-formed, and framed
with proper start codes.

    python tools/h264_nal_dump.py phone_capture.h264 --limit 60
"""
from __future__ import annotations

import argparse
import sys

_NAL_NAMES = {
    1: "non-IDR-slice(P/B)", 5: "IDR-slice", 6: "SEI", 7: "SPS", 8: "PPS",
    9: "AUD", 2: "slice-A", 3: "slice-B", 4: "slice-C", 12: "filler",
}


def _find_start_codes(buf: bytes):
    """Yield (offset, start_code_len) for every Annex-B start code."""
    i, n = 0, len(buf)
    while i < n - 3:
        if buf[i] == 0 and buf[i + 1] == 0:
            if buf[i + 2] == 1:
                yield i, 3
                i += 3
                continue
            if buf[i + 2] == 0 and i < n - 4 and buf[i + 3] == 1:
                yield i, 4
                i += 4
                continue
        i += 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Dump H.264 Annex-B NAL structure.")
    p.add_argument("path")
    p.add_argument("--limit", type=int, default=60,
                   help="max NAL units to print")
    p.add_argument("--bytes", type=int, default=64 * 1024,
                   help="how many bytes of the file to scan")
    args = p.parse_args(argv)

    with open(args.path, "rb") as fh:
        buf = fh.read(args.bytes)

    starts = list(_find_start_codes(buf))
    if not starts:
        print("no Annex-B start codes found in the scanned region — "
              "the stream may be length-prefixed (AVCC), not Annex-B.")
        # Show the leading bytes so we can eyeball the framing.
        print("first 32 bytes:", buf[:32].hex(" "))
        return 0

    print(f"scanned {len(buf)} B, found {len(starts)} NAL units "
          f"(showing up to {args.limit})\n")
    print(f"{'#':>3}  {'offset':>8}  {'sc':>2}  {'type':>2}  "
          f"{'nri':>3}  {'len':>7}  name")
    counts = {}
    for idx in range(min(len(starts), args.limit)):
        off, sc = starts[idx]
        nal_start = off + sc
        nal_end = starts[idx + 1][0] if idx + 1 < len(starts) else len(buf)
        if nal_start >= len(buf):
            break
        header = buf[nal_start]
        nal_type = header & 0x1F
        nal_ref_idc = (header >> 5) & 0x3
        forbidden = (header >> 7) & 0x1
        nal_len = nal_end - nal_start
        counts[nal_type] = counts.get(nal_type, 0) + 1
        flag = " FORBIDDEN_BIT_SET!" if forbidden else ""
        name = _NAL_NAMES.get(nal_type, "?")
        print(f"{idx:>3}  {off:>8}  {sc:>2}  {nal_type:>2}  {nal_ref_idc:>3}  "
              f"{nal_len:>7}  {name}{flag}")

    print("\nNAL type counts in scanned region:")
    for t in sorted(counts):
        print(f"  type {t:>2} ({_NAL_NAMES.get(t, '?')}): {counts[t]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
