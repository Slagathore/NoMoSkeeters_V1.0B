"""LaserCube wire-protocol pure functions.

All command/response parsing lives here as pure functions over bytes —
no sockets, no I/O. Tests load golden fixtures from disk and assert that
parsers produce the known-correct dataclass.

Reference: BOOTSTRAP.md §9.2-§9.4, BOOTSTRAP_AMENDMENTS.md §9.5.
"""
from __future__ import annotations

import struct
from typing import Optional

from laser.types import LaserInfo


# ── Ports ────────────────────────────────────────────────────────────────
ALIVE_PORT = 45456
CMD_PORT   = 45457
DATA_PORT  = 45458


# ── Command bytes (verified against libLaserdockCore) ────────────────────
LC_CMD_GET_ALIVE                  = 0x27   # ALIVE_PORT only
LC_CMD_GET_FULL_INFO              = 0x77   # CMD_PORT
LC_CMD_GET_RINGBUF_EMPTY_COUNT    = 0x8A   # CMD_PORT
LC_CMD_ENABLE_BUFFER_RESPONSE     = 0x78
LC_CMD_SET_OUTPUT                 = 0x80
LC_CMD_SET_ILDA_RATE              = 0x82
LC_CMD_CLEAR_RINGBUFFER           = 0x8D
LC_CMD_SET_NV_MODEL_INFO          = 0x97
LC_CMD_SET_DAC_BUF_THOLD_LVL      = 0xA0
LC_CMD_SECURITY_REQUEST           = 0xB0
LC_CMD_SECURITY_RESPONSE          = 0xB1

# Sample-data packet IDs (sent on DATA_PORT).
LC_DATA_SAMPLE_ID                 = 0xA9
LC_DATA_SAMPLE_COMPRESSED_ID      = 0x9A


# ── Connection-type enum (raw byte at offset 25, +1 to map to enum) ──────
CONNECTION_TYPES: dict[int, str] = {
    0: "UNKNOWN",
    1: "USB",
    2: "WIFI_SERVER",
    3: "WIFI_CLIENT",
    4: "ETHERNET_SERVER",
    5: "ETHERNET_CLIENT",
}


# ── Sample-data packing ──────────────────────────────────────────────────
MAX_SAMPLES_PER_PACKET = 140


def build_sample_packet(samples: list[bytes],
                         msg_num: int,
                         frame_num: int) -> bytes:
    """Wrap up to MAX_SAMPLES_PER_PACKET pre-packed 10-byte LaserPoint
    samples in a sample-data UDP packet header."""
    header = bytes([
        LC_DATA_SAMPLE_ID,
        0x00,
        msg_num & 0xFF,
        frame_num & 0xFF,
    ])
    return header + b"".join(samples)


# ── Response parsers ─────────────────────────────────────────────────────

def parse_full_info(data: bytes) -> Optional[LaserInfo]:
    """Parse a 64-byte GET_FULL_INFO (0x77) response into LaserInfo.

    Returns None on:
      - too short (< 38 bytes)
      - wrong cmd echo
      - non-zero status byte
    """
    if len(data) < 38:
        return None
    if data[0] != LC_CMD_GET_FULL_INFO:
        return None
    if data[1] != 0x00:
        return None

    buf = data if len(data) >= 64 else data + b"\x00" * (64 - len(data))
    flags = buf[5]
    serial = ":".join(f"{b:02x}" for b in buf[26:32])
    ip_str = ".".join(str(b) for b in buf[32:36])
    name_blob = data[38:].split(b"\x00", 1)[0]
    return LaserInfo(
        model_name=name_blob.decode("utf-8", errors="replace"),
        model_number=buf[37],
        fw_major=buf[3],
        fw_minor=buf[4],
        output_enabled=bool(flags & 0x01),
        interlock=bool(flags & 0x02),
        temp_warn=bool(flags & 0x04),
        over_temp=bool(flags & 0x08),
        packet_errors=(flags >> 4) & 0x0F,
        dac_rate=struct.unpack_from("<I", buf, 10)[0],
        max_dac_rate=struct.unpack_from("<I", buf, 14)[0],
        buffer_free=struct.unpack_from("<H", buf, 19)[0],
        buffer_size=struct.unpack_from("<H", buf, 21)[0],
        battery_percent=buf[23],
        temperature_c=struct.unpack_from("<b", buf, 24)[0],
        connection_type_raw=buf[25],
        serial_number=serial,
        reported_ip=ip_str,
    )


def connection_type_name(raw_byte: int) -> str:
    """Map a connection-type raw byte to its human name. The official source
    does +1 to map the raw byte to the public enum; we follow that."""
    return CONNECTION_TYPES.get(raw_byte + 1, f"unknown({raw_byte})")


def parse_ringbuf_empty(data: bytes) -> Optional[int]:
    """Parse GET_RINGBUF_EMPTY_COUNT (0x8A) reply. Returns the empty-sample
    count, or None on malformed input."""
    if len(data) < 4:
        return None
    if data[0] != LC_CMD_GET_RINGBUF_EMPTY_COUNT:
        return None
    if data[1] != 0x00:
        return None
    return struct.unpack_from("<H", data, 2)[0]


def parse_alive(data: bytes) -> bool:
    """Parse GET_ALIVE (0x27) reply. Returns True if the reply is the
    canonical [0x27, 0x00], False otherwise."""
    return len(data) >= 2 and data[0] == 0x27 and data[1] == 0x00


# ── Synthetic fixture builder (matches Cole's verified probe output) ─────

def synth_full_info(
    fw_major: int = 0,
    fw_minor: int = 23,
    output_enabled: bool = False,
    interlock: bool = True,
    temp_warn: bool = False,
    over_temp: bool = False,
    packet_errors: int = 0,
    dac_rate: int = 30000,
    max_dac_rate: int = 30000,
    buffer_free: int = 6000,
    buffer_size: int = 6000,
    battery_percent: int = 0xFF,
    temperature_c: int = 51,
    connection_type_raw: int = 2,
    serial: bytes = bytes.fromhex("c45bbe885324"),
    reported_ip: tuple = (169, 254, 40, 83),
    model_number: int = 1,
    model_name: str = "Wifi LaserCube 2.5W",
) -> bytes:
    """Build a 64-byte GET_FULL_INFO response matching Cole's verified
    hardware probe output. Used for parser regression tests until a real
    capture from `probe --save-raw` lands at tests/fixtures/."""
    flags = (
        (0x01 if output_enabled else 0)
        | (0x02 if interlock else 0)
        | (0x04 if temp_warn else 0)
        | (0x08 if over_temp else 0)
        | ((packet_errors & 0x0F) << 4)
    )
    buf = bytearray(64)
    buf[0] = LC_CMD_GET_FULL_INFO
    buf[1] = 0x00                  # status: success
    buf[2] = 0x00                  # payload version
    buf[3] = fw_major
    buf[4] = fw_minor
    buf[5] = flags
    # bytes 6-9 reserved (zero)
    struct.pack_into("<I", buf, 10, dac_rate)
    struct.pack_into("<I", buf, 14, max_dac_rate)
    # byte 18 reserved
    struct.pack_into("<H", buf, 19, buffer_free)
    struct.pack_into("<H", buf, 21, buffer_size)
    buf[23] = battery_percent
    struct.pack_into("<b", buf, 24, temperature_c)
    buf[25] = connection_type_raw
    buf[26:32] = serial
    buf[32:36] = bytes(reported_ip)
    # byte 36 reserved
    buf[37] = model_number
    name_bytes = model_name.encode("utf-8") + b"\x00"
    buf[38:38 + len(name_bytes)] = name_bytes
    return bytes(buf)


def synth_ringbuf_empty(empty_count: int = 6000) -> bytes:
    """Build a 4-byte GET_RINGBUF_EMPTY_COUNT response."""
    out = bytearray(4)
    out[0] = LC_CMD_GET_RINGBUF_EMPTY_COUNT
    out[1] = 0x00
    struct.pack_into("<H", out, 2, empty_count)
    return bytes(out)


def synth_alive() -> bytes:
    """Canonical 2-byte ALIVE reply [0x27, 0x00]."""
    return bytes([LC_CMD_GET_ALIVE, 0x00])
