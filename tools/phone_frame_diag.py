#!/usr/bin/env python3
"""phone_frame_diag.py — wire-level diagnosis of the phone UDP frame channel.

Runs the same connect / set_active_camera / stream_start handshake the real
sensor does, but instead of decoding, it binds the UDP frame port directly and
reports what is actually arriving on the wire:

  - magic (confirms NMS1 vs NMS2 — i.e. whether the phone app is rebuilt)
  - chunk_count distribution (how many frames fragment, and into how many)
  - per-frame reassembly outcome (complete vs dropped) via the real reassembler
  - frame_id gaps (UDP loss / encoder drops)
  - payload sizes (datagram + whole-frame)

This isolates "is the phone sending good data?" from "is ffmpeg decoding it?".
Network only; no laser, no admin.

    python tools/phone_frame_diag.py
    python tools/phone_frame_diag.py --seconds 8 --mode h264_lowlat
"""
from __future__ import annotations

import argparse
import socket
import sys
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from config import settings                                       # noqa: E402
from phone_sensor.client import PhoneSensorClient                 # noqa: E402
from phone_sensor.frame_decoder import _FrameReassembler          # noqa: E402
from phone_sensor.protocol import parse_frame_packet              # noqa: E402


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Phone frame-channel wire diag.")
    p.add_argument("--phone-ip", default=settings.PHONE_IP)
    p.add_argument("--phone-camera", default=settings.PHONE_DEFAULT_CAMERA)
    p.add_argument("--mode", default=settings.PHONE_DEFAULT_STREAM_MODE)
    p.add_argument("--width", type=int, default=settings.PHONE_DEFAULT_STREAM_WIDTH)
    p.add_argument("--height", type=int, default=settings.PHONE_DEFAULT_STREAM_HEIGHT)
    p.add_argument("--fps", type=int, default=settings.PHONE_DEFAULT_STREAM_FPS)
    p.add_argument("--seconds", type=float, default=6.0)
    args = p.parse_args(argv)

    print("=" * 60)
    print("  Phone frame-channel wire diagnosis")
    print(f"  TCP {args.phone_ip}:{settings.PHONE_CMD_PORT}  "
          f"UDP frame :{settings.PHONE_FRAME_PORT}")
    print("=" * 60)

    # Bind UDP first so we don't miss the first frames after stream_start.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((settings.PHONE_BIND_IP, settings.PHONE_FRAME_PORT))
    sock.settimeout(0.5)

    client = PhoneSensorClient(host=args.phone_ip)
    if not client.connect():
        print("FAIL: TCP connect — is the app running and reachable?")
        return 1
    if not client.set_active_camera(args.phone_camera):
        print(f"FAIL: set_active_camera({args.phone_camera})")
        client.disconnect()
        return 1
    negotiated = client.stream_start(mode=args.mode, width=args.width,
                                     height=args.height, fps=args.fps)
    if negotiated is None:
        print("FAIL: stream_start")
        client.disconnect()
        return 1
    print(f"  streaming mode={negotiated} {args.width}x{args.height}@{args.fps}, "
          f"sniffing {args.seconds:.0f}s...\n")

    reassembler = _FrameReassembler(settings.PHONE_FRAME_REASSEMBLY_MAX)
    datagrams = 0
    bad = 0
    total_bytes = 0
    magics: Counter = Counter()
    fmts: Counter = Counter()
    chunkcount_hist: Counter = Counter()
    datagram_sizes = []
    completed_frames = 0
    completed_sizes = []
    frame_ids_seen = set()
    max_datagram = 0

    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds:
            try:
                data, _ = sock.recvfrom(2_000_000)
            except socket.timeout:
                continue
            datagrams += 1
            total_bytes += len(data)
            datagram_sizes.append(len(data))
            max_datagram = max(max_datagram, len(data))
            magics[bytes(data[:4])] += 1
            try:
                chunk = parse_frame_packet(data)
            except ValueError:
                bad += 1
                continue
            fmts[chunk.fmt] += 1
            chunkcount_hist[chunk.chunk_count] += 1
            frame_ids_seen.add(chunk.frame_id)
            whole = reassembler.add(chunk)
            if whole is not None:
                completed_frames += 1
                completed_sizes.append(len(whole.payload))
    finally:
        try:
            client.stream_stop()
        except Exception:
            pass
        client.disconnect()
        sock.close()

    elapsed = time.time() - t0
    print(f"  datagrams        : {datagrams}  ({datagrams/elapsed:.0f}/s)")
    print(f"  unparseable      : {bad}")
    print(f"  bytes            : {total_bytes/1e6:.2f} MB  "
          f"({total_bytes*8/elapsed/1e6:.1f} Mbps)")
    print(f"  magic            : "
          + ", ".join(f"{m!r}x{n}" for m, n in magics.most_common()))
    print(f"  fmt              : "
          + ", ".join(f"{f}x{n}" for f, n in fmts.most_common()))
    print(f"  max datagram     : {max_datagram} B (ceiling 65507)")
    print(f"  chunk_count hist : "
          + ", ".join(f"{c}->{n}" for c, n in sorted(chunkcount_hist.items())))
    print(f"  frames completed : {completed_frames}  "
          f"({completed_frames/elapsed:.1f}/s)")
    if completed_sizes:
        print(f"  frame size       : min {min(completed_sizes)} / "
              f"max {max(completed_sizes)} B")
    if frame_ids_seen:
        lo, hi = min(frame_ids_seen), max(frame_ids_seen)
        span = hi - lo + 1
        print(f"  frame_id span    : {lo}..{hi} ({span} ids, "
              f"{len(frame_ids_seen)} seen, {span - len(frame_ids_seen)} gaps)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
