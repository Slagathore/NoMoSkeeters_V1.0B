#!/usr/bin/env python3
"""
lasercube_protocol_probe.py
============================
Read-only LaserCube protocol identification + diagnostic tool.

PROTOCOL REFERENCE (canonical, verified against multiple sources):

  1. Wickedlasers/libLaserdockCore  (official manufacturer source — what
     LaserOS itself uses internally)
       https://github.com/Wickedlasers/libLaserdockCore
       3rdparty/laserdocklib/src/LaserDockNetworkDevice.cpp

  2. lasercube-core Rust crate v0.1.0  (March 2025, mirrors official protocol)
       https://docs.rs/lasercube-core

  3. ModulaserApp/laser-dac-rs  (active Rust DAC abstraction, verified to
     work with WiFi LaserCubes — recommends LAN over WiFi)
       https://github.com/ModulaserApp/laser-dac-rs

KEY FACTS FROM THE OFFICIAL SOURCE:

  Three UDP ports:
    45456 = ALIVE port (accepts only GET_ALIVE 0x27, broadcast discovery)
    45457 = CMD port   (all commands and most replies)
    45458 = DATA port  (sample data only, may also reply with buffer status)

  CRITICAL: The official client BINDS its command socket to port 45457
  (not an ephemeral port). It sends to (cube_ip, 45457) from src port 45457
  and the cube replies back to port 45457. This is why earlier probes from
  ephemeral source ports got nothing — the replies went to a port we were
  not listening on.

  Read-only commands (the ONLY ones this probe sends):
    0x27  GET_ALIVE                  -> 2-byte reply [0x27, 0x00]    (port 45456)
    0x77  GET_FULL_INFO              -> 64-byte reply with full state (port 45457)
    0x8a  GET_RINGBUF_EMPTY_COUNT    -> 4-byte reply with free count  (port 45457)

  Commands this probe NEVER sends (they would change device state):
    0x78 ENABLE_BUFFER_SIZE_RESPONSE   (modifies behavior)
    0x80 SET_OUTPUT                    (could enable the laser)
    0x82 SET_ILDA_RATE                 (changes scan rate)
    0x8d CLEAR_RINGBUFFER               (data loss)
    0x97 SET_NV_MODEL_INFO             (writes non-volatile model data)
    0xa0 SET_DAC_BUF_THOLD_LVL         (changes buffer threshold)
    0xb0 SECURITY_CMD_REQUEST          (auth state machine)

USAGE:
  python lasercube_protocol_probe.py
  python lasercube_protocol_probe.py --ip 169.254.40.83 --verbose
  python lasercube_protocol_probe.py --discover
  python lasercube_protocol_probe.py --save responses.bin

REQUIREMENTS:
  Python 3.8+ stdlib only.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Optional

# ──────────────────────────────────────────────────────────────────────────────
# Protocol constants  (from Wickedlasers/libLaserdockCore)
# ──────────────────────────────────────────────────────────────────────────────
DEFAULT_IP = "169.254.40.83"  # Cole's current LAN-client APIPA address

ALIVE_PORT = 45456
CMD_PORT   = 45457
DATA_PORT  = 45458

# Read-only command bytes (verified against official source).
CMD_GET_ALIVE                 = 0x27   # on ALIVE_PORT, broadcast
CMD_GET_FULL_INFO             = 0x77   # on CMD_PORT
CMD_GET_RINGBUF_EMPTY_COUNT   = 0x8a   # on CMD_PORT

# Connection-type enum from official source (data[25] in GET_FULL_INFO reply).
CONNECTION_TYPES = {
    0: "UNKNOWN",
    1: "USB",
    2: "WIFI_SERVER",     # cube is its own AP
    3: "WIFI_CLIENT",     # cube joined existing WiFi
    4: "ETHERNET_SERVER", # cube provides DHCP on ethernet
    5: "ETHERNET_CLIENT", # cube on existing LAN (Cole's mode)
}


# ──────────────────────────────────────────────────────────────────────────────
# Result data class
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class ProbeResult:
    """Captures the outcome of a single command probe."""
    name: str
    cmd_byte: int
    target_ip: str
    target_port: int
    sent_bytes: bytes = b""
    response: Optional[bytes] = None
    response_addr: Optional[tuple] = None
    error: Optional[str] = None
    elapsed_ms: float = 0.0
    parsed: Optional[dict] = None

    @property
    def responded(self) -> bool:
        return self.response is not None and len(self.response) > 0


# ──────────────────────────────────────────────────────────────────────────────
# Response parsers
# ──────────────────────────────────────────────────────────────────────────────
def parse_full_info(payload: bytes) -> Optional[dict]:
    """
    Parse a 64-byte GET_FULL_INFO (0x77) response per libLaserdockCore.

    Layout (little-endian, totals 64 bytes):
      byte  0    : command echo (0x77)
      byte  1    : result/status (0x00 = success)
      byte  2    : payload version id (currently 0)
      byte  3    : fw_major
      byte  4    : fw_minor
      byte  5    : flags  (bit 0=output_enabled, bit 1=interlock,
                           bit 2=temp_warn,    bit 3=over_temp,
                           bits 4-7=packet_errors  on fw>=0.13)
      bytes 6-9  : reserved
      bytes 10-13: dac_rate (uint32 LE)
      bytes 14-17: max_dac_rate (uint32 LE)
      byte  18   : reserved
      bytes 19-20: rx_buffer_free (uint16 LE)
      bytes 21-22: rx_buffer_size (uint16 LE)
      byte  23   : battery_percent
      byte  24   : temperature (int8, degC)
      byte  25   : connection_type (0-based, see CONNECTION_TYPES; +1 in source)
      bytes 26-31: serial_number (6 bytes)
      bytes 32-35: ip_address (4 bytes)
      byte  36   : reserved
      byte  37   : model_number
      bytes 38+  : model_name (null-terminated UTF-8)
    """
    if len(payload) < 38:
        return {"_parse_error": f"too short: {len(payload)} bytes"}

    try:
        cmd_echo = payload[0]
        status   = payload[1]
        if cmd_echo != CMD_GET_FULL_INFO:
            return {"_parse_error": f"unexpected cmd echo 0x{cmd_echo:02x}"}

        # Pad if shorter than 64 (defensive)
        buf = payload if len(payload) >= 64 else payload + b"\x00" * (64 - len(payload))

        payload_ver = buf[2]
        fw_major    = buf[3]
        fw_minor    = buf[4]
        flags       = buf[5]
        # New flag layout for fw >= 0.13 (which yours is — 0.23)
        output_enabled = bool(flags & 0x01)
        interlock      = bool(flags & 0x02)
        temp_warn      = bool(flags & 0x04)
        over_temp      = bool(flags & 0x08)
        packet_errors  = (flags >> 4) & 0x0f

        dac_rate     = struct.unpack_from("<I", buf, 10)[0]
        max_dac_rate = struct.unpack_from("<I", buf, 14)[0]
        buffer_free  = struct.unpack_from("<H", buf, 19)[0]
        buffer_size  = struct.unpack_from("<H", buf, 21)[0]
        battery_pct  = buf[23]
        temperature  = struct.unpack_from("<b", buf, 24)[0]  # int8
        contype_raw  = buf[25]
        # Note: source does ConnectionType(data[25]+1) — so we add 1 to match.
        contype      = CONNECTION_TYPES.get(contype_raw + 1, f"unknown({contype_raw})")

        serial_bytes = buf[26:32]
        ip_bytes     = buf[32:36]
        model_number = buf[37]
        name_blob    = payload[38:].split(b"\x00", 1)[0]
        model_name   = name_blob.decode("utf-8", errors="replace")

        return {
            "status":          status,
            "payload_version": payload_ver,
            "model_name":      model_name,
            "model_number":    model_number,
            "firmware":        f"{fw_major}.{fw_minor}",
            "fw_major":        fw_major,
            "fw_minor":        fw_minor,
            "output_enabled":  output_enabled,
            "interlock":       interlock,
            "temp_warn":       temp_warn,
            "over_temp":       over_temp,
            "packet_errors":   packet_errors,
            "dac_rate":        dac_rate,
            "max_dac_rate":    max_dac_rate,
            "buffer_free":     buffer_free,
            "buffer_size":     buffer_size,
            "battery_percent": battery_pct,
            "temperature_c":   temperature,
            "connection_type": contype,
            "serial_number":   ":".join("%02x" % b for b in serial_bytes),
            "reported_ip":     ".".join(str(b) for b in ip_bytes),
        }
    except Exception as exc:
        return {"_parse_error": repr(exc)}


def parse_ringbuf_empty(payload: bytes) -> Optional[dict]:
    """
    Parse GET_RINGBUF_EMPTY_COUNT (0x8a) response.
    Layout (4 bytes):
      byte 0 : cmd echo (0x8a)
      byte 1 : status (0x00)
      bytes 2-3 : empty_sample_count (uint16 LE)
    """
    if len(payload) < 4:
        return {"_parse_error": f"too short: {len(payload)} bytes"}
    try:
        cmd_echo, status = payload[0], payload[1]
        empty_count = struct.unpack_from("<H", payload, 2)[0]
        return {
            "cmd_echo":           f"0x{cmd_echo:02x}",
            "status":             status,
            "empty_sample_count": empty_count,
        }
    except Exception as exc:
        return {"_parse_error": repr(exc)}


def parse_alive(payload: bytes) -> Optional[dict]:
    """
    Parse GET_ALIVE (0x27) response.
    Layout (2 bytes): [0x27, 0x00]
    """
    if len(payload) < 2:
        return {"_parse_error": f"too short: {len(payload)} bytes"}
    return {
        "cmd_echo": f"0x{payload[0]:02x}",
        "status":   payload[1],
        "valid":    payload[0] == 0x27 and payload[1] == 0x00,
    }


# ──────────────────────────────────────────────────────────────────────────────
# Network helpers
# ──────────────────────────────────────────────────────────────────────────────
def send_and_recv(
    sock: socket.socket,
    target_ip: str,
    target_port: int,
    payload: bytes,
    timeout: float,
    expect_echo_byte: Optional[int] = None,
    filter_source: bool = True,
) -> tuple[Optional[bytes], Optional[tuple], Optional[str], float]:
    """Send `payload` and wait for one matching UDP response.

    Filters by source (target_ip, target_port) so stray datagrams from
    LaserOS / other cubes / leftover socket crud don't masquerade as our
    cube's reply. Drops mismatched packets and keeps reading until either
    a match or the timeout.
    """
    start = time.monotonic()
    deadline = start + timeout
    try:
        sock.sendto(payload, (target_ip, target_port))
    except OSError as exc:
        return None, None, f"OSError: {exc}", (time.monotonic() - start) * 1000.0

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None, None, "timeout", (time.monotonic() - start) * 1000.0
        sock.settimeout(max(0.01, remaining))
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            return None, None, "timeout", (time.monotonic() - start) * 1000.0
        except OSError as exc:
            return None, None, f"OSError: {exc}", (time.monotonic() - start) * 1000.0
        if filter_source and (addr[0] != target_ip or addr[1] != target_port):
            continue   # not from our cube; drop and keep waiting
        if expect_echo_byte is not None and (len(data) < 1 or data[0] != expect_echo_byte):
            continue   # echo mismatch; drop and keep waiting
        return data, addr, None, (time.monotonic() - start) * 1000.0


# ──────────────────────────────────────────────────────────────────────────────
# Probes
# ──────────────────────────────────────────────────────────────────────────────
def probe_unicast(target_ip: str, timeout: float, verbose: bool, src_ip: str = "0.0.0.0") -> list[ProbeResult]:
    """
    Probe the LaserCube using the official binding pattern:
    bind a UDP socket to port 45457 and use it for both send and receive.

    src_ip: local IP to bind to. Defaults to 0.0.0.0 (any interface), but on
            multi-NIC systems (WiFi + ethernet, VPN active, etc.) Windows may
            send packets out the wrong adapter unless src_ip is explicit.
            For APIPA setups, pass your 169.254.x.x address here.
    """
    print(f"\n=== Unicast probe: {target_ip}:{CMD_PORT}  (src={src_ip}:{CMD_PORT}, timeout={timeout}s) ===")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((src_ip, CMD_PORT))
    except OSError as exc:
        print(f"  [!] Could not bind to {src_ip}:{CMD_PORT}: {exc}")
        print(f"      Another process is holding it — likely LaserOS still running.")
        print(f"      Close LaserOS completely, then re-run this probe.")
        sock.close()
        return []

    # Larger buffers, mirroring libLaserdockCore practice.
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 5_250_000)

    probes = [
        ("GET_FULL_INFO         (0x77)", CMD_GET_FULL_INFO,           parse_full_info),
        ("GET_RINGBUF_EMPTY     (0x8a)", CMD_GET_RINGBUF_EMPTY_COUNT, parse_ringbuf_empty),
    ]

    results: list[ProbeResult] = []
    try:
        for label, cmd_byte, parser in probes:
            payload = bytes([cmd_byte])
            data, addr, err, elapsed = send_and_recv(
                sock, target_ip, CMD_PORT, payload, timeout,
                expect_echo_byte=cmd_byte, filter_source=True,
            )
            r = ProbeResult(
                name=label,
                cmd_byte=cmd_byte,
                target_ip=target_ip,
                target_port=CMD_PORT,
                sent_bytes=payload,
                response=data,
                response_addr=addr,
                error=err,
                elapsed_ms=elapsed,
            )
            if data and parser is not None:
                r.parsed = parser(data)
            results.append(r)
            print_probe_line(r, verbose=verbose)
    finally:
        sock.close()

    return results


def probe_alive_unicast(target_ip: str, timeout: float, verbose: bool, src_ip: str = "0.0.0.0") -> list[ProbeResult]:
    """
    Probe the alive port (45456) with GET_ALIVE (0x27) as a unicast message.
    The official protocol uses broadcast for this, but unicast usually works
    too and is a useful sanity check.
    """
    print(f"\n=== Alive-port unicast probe: {target_ip}:{ALIVE_PORT}  (src={src_ip}:{ALIVE_PORT}) ===")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((src_ip, ALIVE_PORT))
    except OSError as exc:
        print(f"  [!] Could not bind to {src_ip}:{ALIVE_PORT}: {exc}")
        sock.close()
        return []

    payload = bytes([CMD_GET_ALIVE])
    data, addr, err, elapsed = send_and_recv(
        sock, target_ip, ALIVE_PORT, payload, timeout,
        expect_echo_byte=CMD_GET_ALIVE, filter_source=True,
    )
    r = ProbeResult(
        name="GET_ALIVE             (0x27)",
        cmd_byte=CMD_GET_ALIVE,
        target_ip=target_ip,
        target_port=ALIVE_PORT,
        sent_bytes=payload,
        response=data,
        response_addr=addr,
        error=err,
        elapsed_ms=elapsed,
        parsed=parse_alive(data) if data else None,
    )
    print_probe_line(r, verbose=verbose)
    sock.close()
    return [r]


def probe_broadcast_alive(
    bind_ip: str,
    broadcast_ip: str,
    timeout: float,
    verbose: bool,
) -> list[ProbeResult]:
    """
    Broadcast GET_ALIVE (0x27) on port 45456 to discover any LaserCubes on
    the local subnet. This is what the official protocol uses for discovery.
    """
    print(f"\n=== Broadcast alive: {broadcast_ip}:{ALIVE_PORT}  (bind={bind_ip}:{ALIVE_PORT}) ===")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((bind_ip, ALIVE_PORT))
    except OSError as exc:
        print(f"  [!] Could not bind to {bind_ip}:{ALIVE_PORT}: {exc}")
        sock.close()
        return []

    payload = bytes([CMD_GET_ALIVE])
    print(f"  Sending GET_ALIVE (0x27) to broadcast address ...")
    try:
        sock.sendto(payload, (broadcast_ip, ALIVE_PORT))
    except OSError as exc:
        print(f"  [!] Send failed: {exc}")
        sock.close()
        return []

    deadline = time.monotonic() + timeout
    results: list[ProbeResult] = []
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        sock.settimeout(remaining)
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            break
        except OSError as exc:
            print(f"  [!] Recv error: {exc}")
            break

        r = ProbeResult(
            name=f"BROADCAST alive reply from {addr[0]}",
            cmd_byte=CMD_GET_ALIVE,
            target_ip=broadcast_ip,
            target_port=ALIVE_PORT,
            sent_bytes=payload,
            response=data,
            response_addr=addr,
            elapsed_ms=0.0,
            parsed=parse_alive(data),
        )
        results.append(r)
        print_probe_line(r, verbose=verbose)

    sock.close()
    if not results:
        print("  No alive responses received within timeout window.")
    return results


# ──────────────────────────────────────────────────────────────────────────────
# Output formatting
# ──────────────────────────────────────────────────────────────────────────────
def hexdump(data: bytes, width: int = 16, max_lines: int = 8) -> str:
    """Compact hex+ASCII dump."""
    lines = []
    for offset in range(0, len(data), width):
        chunk = data[offset:offset + width]
        hex_part   = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"    {offset:04x}  {hex_part:<{width * 3}}  {ascii_part}")
        if len(lines) >= max_lines:
            remaining = len(data) - (offset + width)
            if remaining > 0:
                lines.append(f"    ... ({remaining} more bytes)")
            break
    return "\n".join(lines)


def print_probe_line(r: ProbeResult, verbose: bool) -> None:
    if r.responded and r.response is not None and r.response_addr is not None:
        marker = "[OK]  "
        detail = f"{len(r.response)}B in {r.elapsed_ms:.1f}ms from {r.response_addr[0]}:{r.response_addr[1]}"
    else:
        marker = "[----]"
        detail = f"{r.error} ({r.elapsed_ms:.1f}ms)"

    print(f"  {marker} {r.name:<42} -> {detail}")

    if verbose and r.responded and r.response is not None:
        print(hexdump(r.response))
        if r.parsed:
            for k, v in r.parsed.items():
                print(f"      {k}: {v}")


def print_verdict(unicast_results: list[ProbeResult]) -> None:
    print("\n=== Verdict ===")

    full_info = next(
        (r for r in unicast_results if r.cmd_byte == CMD_GET_FULL_INFO and r.responded),
        None,
    )

    if full_info and full_info.parsed and "_parse_error" not in full_info.parsed:
        info = full_info.parsed
        print()
        print("  >>> GET_FULL_INFO parsed cleanly. Cube reports:")
        print(f"        model_name      : {info.get('model_name')!r}")
        print(f"        model_number    : {info.get('model_number')}")
        print(f"        firmware        : {info.get('firmware')}")
        print(f"        connection_type : {info.get('connection_type')}")
        print(f"        output_enabled  : {info.get('output_enabled')}")
        print(f"        interlock       : {info.get('interlock')}")
        print(f"        temp_warn       : {info.get('temp_warn')}")
        print(f"        over_temp       : {info.get('over_temp')}")
        print(f"        packet_errors   : {info.get('packet_errors')}")
        print(f"        dac_rate        : {info.get('dac_rate')} pps")
        print(f"        max_dac_rate    : {info.get('max_dac_rate')} pps")
        print(f"        rx_buffer       : {info.get('buffer_free')} free / {info.get('buffer_size')} total")
        print(f"        battery         : {info.get('battery_percent')}%")
        print(f"        temperature     : {info.get('temperature_c')} degC")
        print(f"        serial          : {info.get('serial_number')}")
        print(f"        reported_ip     : {info.get('reported_ip')}")
        print()
        print("  CONCLUSION: Cube is alive and speaks the canonical libLaserdockCore")
        print("              protocol. Use these values in config/settings.py and")
        print("              start writing the real LaserCube interface.")
        return

    if any(r.responded for r in unicast_results):
        print("  Partial responses received but GET_FULL_INFO did not parse.")
        print("  Run with --verbose --save responses.bin and share the output for analysis.")
        return

    print("  No responses received. Most likely causes (in order):")
    print("    1. LaserOS is still running and holding port 45457 — close it completely")
    print("    2. Windows firewall is blocking inbound UDP — try disabling temporarily")
    print("    3. Multi-NIC routing issue — Mullvad VPN or virtual adapter capturing the route")
    print()
    print("  Diagnostics to run:")
    print("    Get-NetUDPEndpoint | Where-Object { $_.LocalPort -in 45456,45457,45458 }")
    print("    Get-NetRoute -DestinationPrefix '169.254.0.0/16' | Format-Table -AutoSize")
    print("    ipconfig | Select-String '169.254'")


def save_raw_responses(path: str, results: list[ProbeResult]) -> None:
    """Append all raw responses to a binary file for later forensic review."""
    with open(path, "ab") as fh:
        fh.write(f"\n=== probe run @ {time.strftime('%Y-%m-%dT%H:%M:%S')} ===\n".encode())
        for r in results:
            fh.write(f"\n--- {r.name} (cmd 0x{r.cmd_byte:02x} @ {r.target_ip}:{r.target_port}) ---\n".encode())
            fh.write(f"sent: {r.sent_bytes.hex()}\n".encode())
            if r.response:
                fh.write(f"recv from {r.response_addr}: {r.response.hex()}\n".encode())
            else:
                fh.write(f"error: {r.error}\n".encode())
    print(f"\n  Raw responses appended to {path}")


def save_raw_per_command(out_dir: str, results: list[ProbeResult]) -> None:
    """Write one .bin per responding command for golden-fixture capture
    (BOOTSTRAP_AMENDMENTS.md §16.3 / Appendix Q).

    Filename pattern: golden_<name>_<len>b.bin, where <name> derives from the
    command (full_info, ringbuf_empty, alive). Tests in tests/unit/ load these
    by stable name so future parser drift breaks the test immediately.
    """
    import os
    os.makedirs(out_dir, exist_ok=True)
    name_for_cmd = {
        CMD_GET_FULL_INFO:           "full_info",
        CMD_GET_RINGBUF_EMPTY_COUNT: "ringbuf_empty",
        CMD_GET_ALIVE:               "alive",
    }
    written = 0
    for r in results:
        if not r.responded or r.response is None:
            continue
        slug = name_for_cmd.get(r.cmd_byte, f"cmd_{r.cmd_byte:02x}")
        fname = f"golden_{slug}_{len(r.response)}b.bin"
        path = os.path.join(out_dir, fname)
        with open(path, "wb") as fh:
            fh.write(r.response)
        print(f"  wrote {len(r.response):>3}B -> {path}")
        written += 1
    if not written:
        print(f"  [!] No responding commands to save in {out_dir}")


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────
def main() -> int:
    parser = argparse.ArgumentParser(
        prog="lasercube_protocol_probe",
        description="Identify and diagnose the LaserCube UDP protocol (read-only).",
    )
    parser.add_argument("--ip", default=DEFAULT_IP,
                        help=f"LaserCube IP address (default: {DEFAULT_IP})")
    parser.add_argument("--timeout", type=float, default=1.5,
                        help="Seconds to wait for each response (default: 1.5)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Show hex dumps and parsed fields for each response")
    parser.add_argument("--discover", action="store_true",
                        help="Broadcast GET_ALIVE on port 45456 to find any cubes on the subnet")
    parser.add_argument("--bind-ip", default="0.0.0.0",
                        help="Local IP to bind for broadcast discovery (default: 0.0.0.0)")
    parser.add_argument("--broadcast-ip", default="255.255.255.255",
                        help="Broadcast address (default: 255.255.255.255)")
    parser.add_argument("--save", metavar="PATH", default=None,
                        help="Append raw responses to this file for later analysis")
    parser.add_argument("--save-raw", metavar="DIR", default=None,
                        help="Write one golden_<cmd>_<len>b.bin per responding "
                             "command into DIR (use tests/fixtures/ to feed parser tests)")
    parser.add_argument("--src-ip", default="0.0.0.0",
                        help="Local source IP to bind for unicast probes. Use this on multi-NIC "
                             "systems (e.g. WiFi+ethernet+VPN) to force outbound traffic out the "
                             "correct adapter. Default 0.0.0.0 lets Windows pick — which often "
                             "picks WRONG when an APIPA cube is on a secondary adapter.")
    parser.add_argument("--skip-alive", action="store_true",
                        help="Skip the unicast alive-port probe (port 45456)")
    args = parser.parse_args()

    print("=" * 72)
    print("  LaserCube Protocol Probe (read-only diagnostic)")
    print("  Protocol: Wickedlasers/libLaserdockCore, ports 45456/45457/45458")
    print("  No SET_OUTPUT or DAC writes will be sent.")
    print("=" * 72)

    all_results: list[ProbeResult] = []

    # Main probe: GET_FULL_INFO + GET_RINGBUF_EMPTY on port 45457.
    unicast_results = probe_unicast(args.ip, args.timeout, args.verbose, args.src_ip)
    all_results.extend(unicast_results)

    # Optional: alive-port unicast probe (port 45456).
    if not args.skip_alive:
        alive_results = probe_alive_unicast(args.ip, args.timeout, args.verbose, args.src_ip)
        all_results.extend(alive_results)

    # Optional: broadcast discovery.
    if args.discover:
        broadcast_targets = [args.broadcast_ip]
        if args.broadcast_ip == "255.255.255.255":
            broadcast_targets.insert(0, "169.254.255.255")
        for bc in broadcast_targets:
            bcast_results = probe_broadcast_alive(args.bind_ip, bc, args.timeout, args.verbose)
            all_results.extend(bcast_results)

    print_verdict(unicast_results)

    if args.save:
        save_raw_responses(args.save, all_results)

    if args.save_raw:
        save_raw_per_command(args.save_raw, all_results)

    return 0


if __name__ == "__main__":
    sys.exit(main())
