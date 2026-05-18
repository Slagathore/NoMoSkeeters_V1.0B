#!/usr/bin/env python3
"""gopro_stream_probe.py — find where the GoPro actually pushes its video.

Starts the camera streaming (webcam or preview mode), then listens on a wide
range of UDP ports at once and reports which port receives the MPEG-TS push
and from which address. No laser involved, no admin needed.

    python tools/gopro_stream_probe.py --ip 172.27.109.51 --mode webcam

The Hero 13 was observed delivering ~5.7 Mbps to the USB network adapter
while webcam mode was active, but nothing on UDP 8554 — hence the scan.
"""
from __future__ import annotations

import argparse
import socket
import time
import urllib.request


def http_get(url: str, timeout: float = 8.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return resp.status, resp.read().decode("utf-8", "replace").strip()
    except Exception as exc:                                  # noqa: BLE001
        return None, repr(exc)


def parse_ports(spec: str) -> list:
    ports: list = []
    for part in spec.split(","):
        part = part.strip()
        if "-" in part:
            lo, hi = part.split("-")
            ports += list(range(int(lo), int(hi) + 1))
        elif part:
            ports.append(int(part))
    return ports


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ip", default="172.27.109.51", help="camera HTTP IP")
    ap.add_argument("--mode", choices=["webcam", "preview"], default="webcam")
    ap.add_argument("--seconds", type=int, default=10)
    ap.add_argument("--ports", default="5000-5010,8000-9100,15000-15200",
                    help="comma list of ports / ranges to listen on")
    ap.add_argument("--bind", default="0.0.0.0",
                    help="local IP to bind on (use the GoPro interface IP to "
                         "catch ports another process holds on other NICs)")
    ap.add_argument("--save", default=None,
                    help="write all received stream bytes to this file")
    args = ap.parse_args(argv)
    base = f"http://{args.ip}:8080"

    socks: dict = {}
    for port in parse_ports(args.ports):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((args.bind, port))
            s.setblocking(False)
            socks[port] = s
        except OSError:
            pass                                  # port busy — skip it
    if not socks:
        print("FAIL: could not bind any ports")
        return 2
    print(f"listening on {len(socks)} UDP ports "
          f"({min(socks)}..{max(socks)})")

    if args.mode == "webcam":
        print("webcam/start ->", http_get(f"{base}/gopro/webcam/start"))
        stop_path = "/gopro/webcam/stop"
    else:
        print("stream/start ->",
              http_get(f"{base}/gopro/camera/stream/start"))
        stop_path = "/gopro/camera/stream/stop"

    hits: dict = {}                               # port -> [pkts, bytes, set]
    save_fh = open(args.save, "wb") if args.save else None
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            for port, s in socks.items():
                try:
                    while True:
                        data, addr = s.recvfrom(65536)
                        rec = hits.setdefault(port, [0, 0, set()])
                        rec[0] += 1
                        rec[1] += len(data)
                        rec[2].add(addr)
                        if save_fh is not None:
                            save_fh.write(data)
                except BlockingIOError:
                    pass
                except ConnectionResetError:
                    pass                          # ICMP unreachable — ignore
            time.sleep(0.02)
    finally:
        http_get(f"{base}{stop_path}")
        if args.mode == "webcam":
            http_get(f"{base}/gopro/webcam/exit")
        for s in socks.values():
            s.close()
        if save_fh is not None:
            save_fh.close()
            print(f"saved stream bytes -> {args.save}")

    print()
    if hits:
        for port in sorted(hits):
            pkts, nbytes, senders = hits[port]
            print(f"PORT {port}: {pkts} pkts, {nbytes} bytes, "
                  f"from {sorted(senders)}")
        print("\nRESULT: found the stream — point the decoder at that port.")
        return 0
    print("RESULT: nothing on any scanned port — the camera is sending to "
          "an IP this host does not hold (needs a packet capture to see).")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
