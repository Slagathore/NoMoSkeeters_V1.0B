#!/usr/bin/env python3
"""
sha204_cold_test.py
===================
Single-purpose go/no-go diagnostic: does this LaserCube emit visible light
when sent SET_OUTPUT(0x01) cold, with no SHA204 handshake?

If YES — your design is on track. Proceed with the rewrite as planned.
If NO  — handshake required. Halt rewrite. Reverse-engineer the SHA204
         exchange (Wireshark capture of LaserOS connect) before doing
         anything else. Discovering this in week 4 of the rewrite is
         vastly worse than discovering it now.

WHAT THIS SCRIPT DOES (in order):
  1. Connects to the cube using the source-IP binding pattern from
     lasercube_protocol_probe.py (same multi-NIC fix).
  2. Reads baseline state via GET_FULL_INFO.
  3. Sends SET_OUTPUT(0x01) with no prior auth exchange.
  4. Streams a DIM single-channel point at galvo center for a short
     duration, while a daemon thread heartbeats GET_FULL_INFO at 1.5s
     intervals so the cube's 4-second comms timer doesn't kill us.
  5. Sends SET_OUTPUT(0x00) (always, even on error — try/finally).
  6. Disconnects.
  7. Asks the operator whether they saw laser light.
  8. Reports pass/fail with diagnostic interpretation.

NEVER SENT (would modify persistent state):
  SET_ILDA_RATE, CLEAR_RINGBUFFER, SET_NV_MODEL_INFO, SET_DAC_BUF_THOLD_LVL,
  any 0xB0/0xB1 SHA204 commands.

SAFETY (read every line):
  - Default --power-pct=5 puts the red channel at ~125mW raw — Class 3B,
    well above eye-safe. This is NOT a low-risk test.
  - DO before running:
      [1] Aperture pointed at a non-reflective surface (wall/beam dump).
      [2] OD2+ goggles rated for 520nm green specifically (the cube's
          highest-output channel; even when testing red, scattered green
          from the diode body can leak).
      [3] No humans/pets in the beam path or the path of any plausible
          specular reflection.
      [4] Any safety lens/cap removed from the aperture (otherwise you'll
          falsely report "no light" because the cap blocked it).
  - The script defaults to a stationary parked-galvo single point, which
    concentrates energy. Keep --duration-s short.

USAGE:
  python sha204_cold_test.py --src-ip 169.254.25.216
  python sha204_cold_test.py --src-ip 169.254.25.216 --power-pct 10 --duration-s 5
  python sha204_cold_test.py --src-ip 169.254.25.216 --color green --power-pct 2
  python sha204_cold_test.py --src-ip 169.254.25.216 --dry  # all logic, no UDP

REQUIREMENTS:
  Python 3.8+ stdlib only.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional


# ──────────────────────────────────────────────────────────────────────────────
# Protocol constants — verified against libLaserdockCore via probe.
# ──────────────────────────────────────────────────────────────────────────────
DEFAULT_CUBE_IP = "169.254.40.83"
ALIVE_PORT = 45456
CMD_PORT   = 45457
DATA_PORT  = 45458

LC_CMD_GET_FULL_INFO = 0x77
LC_CMD_SET_OUTPUT    = 0x80
LC_DATA_SAMPLE_ID    = 0xA9

COORD_MIN     = 0
COORD_MAX     = 0xFFF       # 12-bit unsigned, the cube's actual range
GALVO_CENTER  = 0x800       # 2048 — middle of 12-bit range

MAX_SAMPLES_PER_PACKET = 140   # 140 * 10 + 4 header = 1404 B, under 1500 MTU
DAC_RATE_DEFAULT       = 30000 # cube default; we don't change it


# ──────────────────────────────────────────────────────────────────────────────
# Tiny helpers
# ──────────────────────────────────────────────────────────────────────────────
def banner(text: str, char: str = "=") -> None:
    bar = char * 64
    print()
    print(bar)
    print(text)
    print(bar)


def confirm(prompt: str, default_no: bool = True) -> bool:
    suffix = "[y/N]" if default_no else "[Y/n]"
    while True:
        try:
            resp = input(f"{prompt} {suffix}: ").strip().lower()
        except EOFError:
            return False
        if resp in ("y", "yes"):
            return True
        if resp in ("n", "no"):
            return False
        if resp == "":
            return not default_no


# ──────────────────────────────────────────────────────────────────────────────
# Minimal GET_FULL_INFO parser — just the fields this test cares about.
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class FullInfo:
    output_enabled: bool
    interlock: bool
    temp_warn: bool
    over_temp: bool
    fw_major: int
    fw_minor: int
    serial: str
    temperature_c: int
    raw_flags: int
    raw_bytes: bytes


def parse_full_info(data: bytes) -> Optional[FullInfo]:
    """Lightweight parser. Full version lives in laser/protocol.py later."""
    if len(data) < 38 or data[0] != LC_CMD_GET_FULL_INFO:
        return None
    flags = data[5]
    serial = ":".join(f"{b:02x}" for b in data[26:32])
    temp_signed = struct.unpack_from("<b", data, 24)[0]
    return FullInfo(
        output_enabled=bool(flags & 0x01),
        interlock     =bool(flags & 0x02),
        temp_warn     =bool(flags & 0x04),
        over_temp     =bool(flags & 0x08),
        fw_major      =data[3],
        fw_minor      =data[4],
        serial        =serial,
        temperature_c =temp_signed,
        raw_flags     =flags,
        raw_bytes     =data,
    )


# ──────────────────────────────────────────────────────────────────────────────
# CubeSession: minimal cmd/data sockets + filtered GET_FULL_INFO.
# ──────────────────────────────────────────────────────────────────────────────
class CubeSession:
    """Two-socket session against the cube. CMD_PORT for commands and replies,
    DATA_PORT for outbound sample streaming. Both sockets bound to (src_ip, port)
    so Windows uses the right NIC (multi-NIC fix from probe phase)."""

    def __init__(self, src_ip: str, cube_ip: str,
                 reply_timeout_s: float = 1.5, dry: bool = False):
        self.src_ip = src_ip
        self.cube_ip = cube_ip
        self.reply_timeout_s = reply_timeout_s
        self.dry = dry  # if True, don't actually open sockets / send anything
        self.cmd_sock: Optional[socket.socket] = None
        self.data_sock: Optional[socket.socket] = None
        self._send_lock = threading.Lock()  # protects send order across threads

    def connect(self) -> bool:
        if self.dry:
            print("[DRY] Would bind cmd_sock to", (self.src_ip, CMD_PORT))
            print("[DRY] Would bind data_sock to", (self.src_ip, DATA_PORT))
            return True
        try:
            # CMD socket — bound to source IP per the libLaserdockCore pattern.
            # Port 45457 is the official binding; the cube replies *back* to
            # this same port. Ephemeral source ports caused our earlier probes
            # to receive nothing.
            self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.cmd_sock.bind((self.src_ip, CMD_PORT))
            self.cmd_sock.settimeout(self.reply_timeout_s)

            # DATA socket — bound to source IP, port 45458.
            self.data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.data_sock.bind((self.src_ip, DATA_PORT))
            return True
        except OSError as e:
            print(f"[ERROR] socket setup failed: {e}", file=sys.stderr)
            self.close()
            return False

    def close(self) -> None:
        for attr in ("cmd_sock", "data_sock"):
            sock = getattr(self, attr, None)
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
                setattr(self, attr, None)

    def get_full_info(self) -> Optional[FullInfo]:
        """Send GET_FULL_INFO, filter response by source IP/port and command
        echo, return parsed dataclass. Drops mismatched datagrams (could be
        LaserOS or stale crud on the same socket)."""
        if self.dry:
            # Synthetic response for dry-run testing of script logic.
            return FullInfo(
                output_enabled=False, interlock=True, temp_warn=False,
                over_temp=False, fw_major=0, fw_minor=23,
                serial="c4:5b:be:88:53:24", temperature_c=51,
                raw_flags=0x02, raw_bytes=b"",
            )
        if self.cmd_sock is None:
            return None
        with self._send_lock:
            try:
                self.cmd_sock.sendto(bytes([LC_CMD_GET_FULL_INFO]),
                                     (self.cube_ip, CMD_PORT))
            except OSError:
                return None
        deadline = time.monotonic() + self.reply_timeout_s
        while time.monotonic() < deadline:
            try:
                self.cmd_sock.settimeout(max(0.01, deadline - time.monotonic()))
                data, addr = self.cmd_sock.recvfrom(4096)
            except socket.timeout:
                return None
            except OSError:
                return None
            # Filter: must be from our cube, must echo our command byte.
            if addr[0] != self.cube_ip or addr[1] != CMD_PORT:
                continue
            if len(data) < 1 or data[0] != LC_CMD_GET_FULL_INFO:
                continue
            return parse_full_info(data)
        return None

    def set_output(self, enabled: bool, repeat: int = 2) -> bool:
        """Send SET_OUTPUT. UDP, no reply expected. Repeat for reliability —
        if the first packet drops, the second usually arrives."""
        if self.dry:
            print(f"[DRY] Would SET_OUTPUT({0x01 if enabled else 0x00}) x{repeat}")
            return True
        if self.cmd_sock is None:
            return False
        payload = bytes([LC_CMD_SET_OUTPUT, 0x01 if enabled else 0x00])
        with self._send_lock:
            try:
                for _ in range(repeat):
                    self.cmd_sock.sendto(payload, (self.cube_ip, CMD_PORT))
                    time.sleep(0.005)  # 5ms spacing — gentler on UDP stack
                return True
            except OSError:
                return False

    def send_data_chunk(self,
                        points: list[tuple[int, int, int, int, int]],
                        msg_num: int,
                        frame_num: int) -> bool:
        """Send one UDP packet of sample data on DATA_PORT.
        points = list of (x, y, r, g, b) tuples in 12-bit galvo space."""
        if self.dry:
            return True
        if self.data_sock is None:
            return False
        # Header: [LC_DATA_SAMPLE_ID, 0x00, msg_num, frame_num] (4 bytes)
        header = bytes([LC_DATA_SAMPLE_ID, 0x00,
                        msg_num & 0xFF, frame_num & 0xFF])
        # Body: each sample is <HHHHH> = 5 uint16 LE = 10 bytes.
        # Values are clamped to 12-bit unsigned per the wire spec.
        body = b"".join(
            struct.pack("<HHHHH",
                        max(COORD_MIN, min(COORD_MAX, x)),
                        max(COORD_MIN, min(COORD_MAX, y)),
                        max(COORD_MIN, min(COORD_MAX, r)),
                        max(COORD_MIN, min(COORD_MAX, g)),
                        max(COORD_MIN, min(COORD_MAX, b)))
            for (x, y, r, g, b) in points
        )
        try:
            self.data_sock.sendto(header + body,
                                  (self.cube_ip, DATA_PORT))
            return True
        except OSError:
            return False


# ──────────────────────────────────────────────────────────────────────────────
# Heartbeat thread — keeps cube's 4-second comms timer alive during the test.
# Critical because if the main thread blocks (debugger, stutter) for >4s, the
# cube assumes we crashed and disables itself. As a daemon thread, this also
# stops on its own when the process exits.
# ──────────────────────────────────────────────────────────────────────────────
class Heartbeat(threading.Thread):
    # NOTE on the rename: threading.Thread has a private internal method
    # named _stop() that Thread.join() calls to mark the thread terminated.
    # Naming our own attribute self._stop SHADOWED that method and caused
    # join() to crash with 'Event' object is not callable. Use _stop_event
    # (or any name that isn't on the parent class) instead.
    def __init__(self, session: CubeSession, interval_s: float = 1.5):
        super().__init__(name="cube_heartbeat", daemon=True)
        self.session = session
        self.interval_s = interval_s
        self._stop_event = threading.Event()
        self.last_info: Optional[FullInfo] = None
        self.tick_count = 0

    def run(self) -> None:
        # NEVER raise from here — the main test must not crash if the cube
        # blips. If we lose comms permanently, the next get_full_info() in
        # the main thread will return None and the operator will see it.
        while not self._stop_event.wait(self.interval_s):
            try:
                info = self.session.get_full_info()
                if info is not None:
                    self.last_info = info
                    self.tick_count += 1
            except Exception:
                pass

    def stop(self) -> None:
        self._stop_event.set()
        self.join(timeout=2.0)


# ──────────────────────────────────────────────────────────────────────────────
# Main test flow
# ──────────────────────────────────────────────────────────────────────────────
def main() -> int:
    parser = argparse.ArgumentParser(
        description="SHA204 cold-test for LaserCube SET_OUTPUT.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--src-ip", required=True,
                        help="Local IP to bind to (your APIPA address)")
    parser.add_argument("--cube-ip", default=DEFAULT_CUBE_IP)
    parser.add_argument("--power-pct", type=int, default=5,
                        help="Output power percentage (0-100)")
    parser.add_argument("--duration-s", type=float, default=3.0,
                        help="How long to stream the test point")
    parser.add_argument("--color", default="red",
                        choices=["red", "green", "blue", "white"],
                        help="Which channel(s) to test "
                             "(red is least visually intense)")
    parser.add_argument("--dry", action="store_true",
                        help="Run all logic without opening sockets — "
                             "use to verify script behavior")
    parser.add_argument("--skip-confirm", action="store_true",
                        help="Skip the safety confirmation prompt "
                             "(NOT recommended)")
    args = parser.parse_args()

    if not (0 <= args.power_pct <= 100):
        print("[ERROR] --power-pct must be 0-100", file=sys.stderr)
        return 2
    if args.duration_s <= 0 or args.duration_s > 30:
        print("[ERROR] --duration-s must be (0, 30]", file=sys.stderr)
        return 2

    # ── Header ────────────────────────────────────────────────────────────
    banner("SHA204 COLD-TEST — LaserCube SET_OUTPUT verification")
    print(f"  Source IP   : {args.src_ip}")
    print(f"  Cube IP     : {args.cube_ip}")
    print(f"  Power       : {args.power_pct}%")
    print(f"  Duration    : {args.duration_s}s")
    print(f"  Color       : {args.color}")
    print(f"  Mode        : {'DRY (no UDP)' if args.dry else 'LIVE'}")
    print()
    print("BEFORE PROCEEDING, CONFIRM ALL OF THE FOLLOWING:")
    print("  [ ] Aperture pointed at non-reflective surface (wall/beam dump)")
    print("  [ ] OD2+ eye protection on (rated for 520nm specifically)")
    print("  [ ] No people/pets in the beam path")
    print("  [ ] Safety lens/cap REMOVED from aperture")

    if not args.skip_confirm and not args.dry:
        if not confirm("All safety conditions confirmed?"):
            print("Aborted by operator.")
            return 1

    # ── Build the test point ──────────────────────────────────────────────
    rgb_value = (args.power_pct * COORD_MAX) // 100
    color_map = {
        "red":   (rgb_value, 0, 0),
        "green": (0, rgb_value, 0),
        "blue":  (0, 0, rgb_value),
        "white": (rgb_value, rgb_value, rgb_value),
    }
    r, g, b = color_map[args.color]
    test_point: tuple[int, int, int, int, int] = (GALVO_CENTER, GALVO_CENTER, r, g, b)
    print(f"\nTest point: galvo=({GALVO_CENTER:#x}, {GALVO_CENTER:#x}) "
          f"rgb=({r:#x}, {g:#x}, {b:#x})")

    # ── Phase 1: Connect ──────────────────────────────────────────────────
    banner("Phase 1: Connect", char="-")
    session = CubeSession(src_ip=args.src_ip, cube_ip=args.cube_ip,
                          dry=args.dry)
    if not session.connect():
        print("[FAIL] Could not bind sockets. Check --src-ip and firewall.")
        return 3
    print("[OK] Sockets bound.")

    # ── Phase 2: Baseline state ───────────────────────────────────────────
    banner("Phase 2: Baseline state", char="-")
    baseline = session.get_full_info()
    if baseline is None:
        print("[FAIL] No GET_FULL_INFO response. Cube unreachable.")
        session.close()
        return 4
    print(f"  fw            : {baseline.fw_major}.{baseline.fw_minor}")
    print(f"  serial        : {baseline.serial}")
    print(f"  temperature_c : {baseline.temperature_c}")
    print(f"  output_enabled: {baseline.output_enabled}")
    print(f"  interlock     : {baseline.interlock}")
    print(f"  temp_warn     : {baseline.temp_warn}")
    print(f"  over_temp     : {baseline.over_temp}")
    if baseline.over_temp:
        print("[FAIL] Cube reports over_temp. Let it cool, retry later.")
        session.close()
        return 5
    if baseline.output_enabled:
        print("[WARN] Cube reports output_enabled=True at baseline. Unusual.")
        print("       Continuing — this test will still tell you something.")

    # ── Phase 3: Start heartbeat ──────────────────────────────────────────
    banner("Phase 3: Start heartbeat", char="-")
    heartbeat = Heartbeat(session, interval_s=1.5)
    heartbeat.start()
    time.sleep(0.1)  # give it a tick to settle
    print(f"[OK] Heartbeat running (interval=1.5s, daemon thread).")

    light_observed = False  # set in phase 8
    try:
        # ── Phase 4: SET_OUTPUT(1) cold ───────────────────────────────────
        banner("Phase 4: SET_OUTPUT(0x01) cold (no SHA204)", char="-")
        if not session.set_output(True, repeat=2):
            print("[FAIL] set_output(True) send failed.")
            return 6
        time.sleep(0.2)  # let cube process before re-querying
        post_set = session.get_full_info()
        if post_set is None:
            print("[WARN] No GET_FULL_INFO response immediately after "
                  "SET_OUTPUT. Cube may be processing or may have dropped.")
        else:
            print(f"  output_enabled (post-cmd): {post_set.output_enabled}")
            if not post_set.output_enabled:
                print("  ↑ The output_enabled bit did NOT flip.")
                print("    Likely meanings:")
                print("      • SET_OUTPUT silently rejected (SHA204 needed)")
                print("      • Cube needs sample data flowing first — Phase 5")
                print("        will test that.")

        # ── Phase 5: Stream test point ────────────────────────────────────
        banner("Phase 5: Stream test point — WATCH NOW", char="-")
        print(f"Streaming {args.color} dot at galvo center for "
              f"{args.duration_s}s...")
        print(">>> WATCH THE BEAM PATH NOW <<<\n")

        # 140 identical points per UDP packet, paced to match the cube's
        # consumption rate (~30k pps default → ~140 samples scan out per 4.7ms).
        # Sleeping for that interval keeps the ringbuffer roughly half-full.
        chunk_points: list = [test_point] * MAX_SAMPLES_PER_PACKET
        packet_interval_s = MAX_SAMPLES_PER_PACKET / DAC_RATE_DEFAULT
        deadline = time.monotonic() + args.duration_s
        msg_num = 0
        frame_num = 0
        packets_sent = 0
        send_failures = 0
        while time.monotonic() < deadline:
            if session.send_data_chunk(chunk_points, msg_num, frame_num):
                packets_sent += 1
            else:
                send_failures += 1
                if send_failures > 5:
                    print("[WARN] Repeated send failures, breaking.")
                    break
            msg_num = (msg_num + 1) & 0xFF
            time.sleep(packet_interval_s)
        print(f"\n[OK] Sent {packets_sent} data packets "
              f"({send_failures} failures).")

        # ── Phase 6: Re-check state during/just-after fire ────────────────
        banner("Phase 6: Re-check state", char="-")
        # Heartbeat thread caches the latest info; use that if recent.
        latest = heartbeat.last_info or session.get_full_info()
        if latest is not None:
            print(f"  output_enabled (latest): {latest.output_enabled}")
            print(f"  interlock              : {latest.interlock}")
            print(f"  temp_warn              : {latest.temp_warn}")
            print(f"  over_temp              : {latest.over_temp}")
            print(f"  temperature_c          : {latest.temperature_c}")
        print(f"  heartbeat ticks during test: {heartbeat.tick_count}")

    finally:
        # ── Phase 7: Cleanup (always runs) ────────────────────────────────
        banner("Phase 7: SET_OUTPUT(0x00) cleanup", char="-")
        # Send disable 3x — if any of the previous packets dropped and the
        # cube is currently emitting, we want maximum reliability shutting
        # it off. UDP, so retransmission is the only mechanism.
        session.set_output(False, repeat=3)
        time.sleep(0.2)
        post_off = session.get_full_info()
        if post_off is not None:
            print(f"  output_enabled (final): {post_off.output_enabled}")
            if post_off.output_enabled:
                print("  [!] Output STILL enabled after SET_OUTPUT(0). "
                      "Manually power-cycle the cube.")
        heartbeat.stop()
        session.close()
        print("[OK] Cleanup complete.")

    # ── Phase 8: Operator verdict ─────────────────────────────────────────
    banner("Phase 8: Operator verdict")
    if args.dry:
        print("[DRY] Skipping operator prompt; assuming light_observed=False.")
        light_observed = False
    else:
        light_observed = confirm(
            "Did you SEE laser light during Phase 5?", default_no=True)

    # ── Result ────────────────────────────────────────────────────────────
    banner("RESULT")
    if light_observed:
        print("  [PASS] SET_OUTPUT works cold. No SHA204 handshake required.")
        print()
        print("  Implication: proceed with the rewrite as designed.")
        print("  Section 9.7 of BOOTSTRAP.md can be marked 'not required'.")
        return 0
    else:
        print("  [FAIL] No light observed despite SET_OUTPUT cold.")
        print()
        print("  Possible causes (in rough order of likelihood):")
        print("    1. SHA204 handshake is required before optical output.")
        print("       → Capture LaserOS connect with Wireshark on")
        print("         udp.port==45457; reverse the 0xB0/0xB1 exchange.")
        print("    2. Interlock physically engaged.")
        print(f"       → baseline.interlock = {baseline.interlock} "
              "(True usually = OK; verify on your unit).")
        print("       → Check aperture cap, hardware switch, etc.")
        print("    3. Power level too low to perceive at this color.")
        print("       → Re-run with --power-pct 20 --color green")
        print("         (green channel usually brightest on these cubes).")
        print("    4. Sample data didn't actually stream.")
        print(f"       → Sent {packets_sent} packets; if 0, debug DATA_PORT.")
        print("    5. Galvos parked outside their visible scan angle.")
        print("       → Unlikely; we sent center 0x800,0x800.")
        print()
        print("  RECOMMENDATION: HALT REWRITE until cause is identified.")
        print("  Cheapest next move: re-run with --color green --power-pct 15.")
        print("  If still no light, capture LaserOS traffic and reverse the")
        print("  SHA204 exchange before any further architecture work.")
        return 7


if __name__ == "__main__":
    raise SystemExit(main())
