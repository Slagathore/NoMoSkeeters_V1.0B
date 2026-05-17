"""LaserCube dataclasses: LaserPoint (one wire-format sample), LaserInfo
(parsed GET_FULL_INFO), BufferEstimate (ringbuffer fullness with freshness clock).

Reference: BOOTSTRAP.md §9.5, BOOTSTRAP_AMENDMENTS.md §9.5.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass


COORD_MIN = 0
COORD_MAX = 0xFFF


@dataclass
class LaserPoint:
    """One galvo position + RGB. Values clamped to 12-bit unsigned on pack."""
    x: int = 0
    y: int = 0
    r: int = 0
    g: int = 0
    b: int = 0

    def packed(self) -> bytes:
        """10-byte wire format: 5 uint16 LE (x, y, r, g, b)."""
        return struct.pack(
            "<HHHHH",
            max(COORD_MIN, min(COORD_MAX, self.x)),
            max(COORD_MIN, min(COORD_MAX, self.y)),
            max(COORD_MIN, min(COORD_MAX, self.r)),
            max(COORD_MIN, min(COORD_MAX, self.g)),
            max(COORD_MIN, min(COORD_MAX, self.b)),
        )


@dataclass
class LaserInfo:
    """Parsed GET_FULL_INFO response (64 bytes)."""
    model_name: str
    model_number: int
    fw_major: int
    fw_minor: int
    output_enabled: bool
    interlock: bool
    temp_warn: bool
    over_temp: bool
    packet_errors: int
    dac_rate: int
    max_dac_rate: int
    buffer_free: int
    buffer_size: int
    battery_percent: int
    temperature_c: int
    connection_type_raw: int
    serial_number: str
    reported_ip: str


@dataclass
class BufferEstimate:
    """Ringbuffer fullness as an estimate with a freshness clock.

    Decisions that gate on buffer state must consult is_fresh() and refresh
    via get_ringbuf_empty() if stale. The estimate decrements on send, but
    UDP loss can desync it from reality.
    """
    free: int = 0
    size: int = 6000
    last_refresh_ts: float = 0.0

    def age_ms(self) -> float:
        return (time.monotonic() - self.last_refresh_ts) * 1000.0

    def is_fresh(self, max_age_ms: float = 100.0) -> bool:
        return self.age_ms() <= max_age_ms
