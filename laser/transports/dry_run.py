"""DryRunTransport — full LaserCubeTransport surface, zero UDP.

Returns canned successes. get_full_info() reads the synthetic fixture so
callers see realistic LaserInfo. Every operation is logged via stdlib
logging AND through an optional event-sink callable so a session recorder
(or test) can capture the call stream.

Reference: BOOTSTRAP_AMENDMENTS.md §17 step 3.
"""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Callable, Optional

from laser.protocol import (
    parse_full_info, parse_ringbuf_empty,
    synth_full_info, synth_ringbuf_empty,
)
from laser.transport import LaserCubeTransport
from laser.types import BufferEstimate, LaserInfo, LaserPoint


_log = logging.getLogger(__name__)


EventSink = Callable[[dict], None]


class DryRunTransport(LaserCubeTransport):
    """No-photon transport for offline development.

    Behavior:
      - connect/disconnect flip an internal flag; no sockets opened
      - get_full_info returns LaserInfo parsed from the synthetic fixture
        (or a custom fixture path if provided)
      - get_ringbuf_empty returns the simulated ring-buffer free count
      - send_chunk advances msg_num, decrements simulated buffer free,
        returns True
      - enable_output / disable_output toggle the output_enabled flag
        reflected back through the next get_full_info()
    """

    def __init__(self,
                 fixture_path: Optional[Path] = None,
                 event_sink: Optional[EventSink] = None,
                 buffer_size: int = 6000):
        self._connected = False
        self._output_enabled = False
        self._dac_rate = 30000
        self._buffer = BufferEstimate(free=buffer_size, size=buffer_size,
                                       last_refresh_ts=time.monotonic())
        self._msg_num = 0
        self._packet_count = 0
        self._sink = event_sink
        # Pre-load fixture bytes once.
        if fixture_path is not None and Path(fixture_path).exists():
            self._fixture_bytes = Path(fixture_path).read_bytes()
        else:
            # Reflect current output_enabled state in synthesized response below.
            self._fixture_bytes = None  # generated on demand from current state

    # ── Lifecycle ────────────────────────────────────────────────────────

    def connect(self) -> bool:
        self._connected = True
        self._emit("connect", {})
        return True

    def disconnect(self) -> None:
        # Best-effort disable on the way out (mirrors live transport behavior).
        if self._output_enabled:
            self.disable_output()
        self._connected = False
        self._emit("disconnect", {})

    def is_connected(self) -> bool:
        return self._connected

    # ── Read-only commands ───────────────────────────────────────────────

    def get_full_info(self) -> Optional[LaserInfo]:
        raw = self._current_fixture_bytes()
        info = parse_full_info(raw)
        self._emit("get_full_info", {"output_enabled": self._output_enabled})
        if info is not None:
            # Surface buffer freshness honestly.
            self._buffer.free = info.buffer_free
            self._buffer.size = info.buffer_size
            self._buffer.last_refresh_ts = time.monotonic()
        return info

    def get_ringbuf_empty(self) -> Optional[int]:
        raw = synth_ringbuf_empty(empty_count=self._buffer.free)
        free = parse_ringbuf_empty(raw)
        self._buffer.last_refresh_ts = time.monotonic()
        self._emit("get_ringbuf_empty", {"free": free})
        return free

    # ── State-modifying commands ─────────────────────────────────────────

    def enable_output(self) -> bool:
        self._output_enabled = True
        self._emit("enable_output", {})
        return True

    def disable_output(self) -> bool:
        self._output_enabled = False
        self._emit("disable_output", {})
        return True

    def set_dac_rate(self, rate: int) -> bool:
        self._dac_rate = max(1000, min(30_000, int(rate)))
        self._emit("set_dac_rate", {"rate": self._dac_rate})
        return True

    def clear_ringbuffer(self) -> bool:
        self._buffer.free = self._buffer.size
        self._buffer.last_refresh_ts = time.monotonic()
        self._emit("clear_ringbuffer", {})
        return True

    # ── Sample streaming ─────────────────────────────────────────────────

    def send_chunk(self,
                   points: list[LaserPoint],
                   msg_num: int,
                   frame_num: int) -> bool:
        # Mirror live behavior: estimate decrements; never gate on it without
        # a fresh get_ringbuf_empty.
        self._buffer.free = max(0, self._buffer.free - len(points))
        self._packet_count += 1
        self._emit("send_chunk", {
            "n_points":  len(points),
            "msg_num":   msg_num,
            "frame_num": frame_num,
            "buffer_free_estimate": self._buffer.free,
        })
        return True

    def _next_msg_num(self) -> int:
        n = self._msg_num
        self._msg_num = (self._msg_num + 1) & 0xFF
        return n

    # ── State surface ────────────────────────────────────────────────────

    def buffer_estimate(self) -> BufferEstimate:
        return self._buffer

    @property
    def output_enabled(self) -> bool:
        return self._output_enabled

    @property
    def packet_count(self) -> int:
        return self._packet_count

    # ── Internal ─────────────────────────────────────────────────────────

    def _current_fixture_bytes(self) -> bytes:
        """Return fixture bytes reflecting the current simulated state."""
        if self._fixture_bytes is not None:
            return self._fixture_bytes
        return synth_full_info(
            output_enabled=self._output_enabled,
            buffer_free=self._buffer.free,
            buffer_size=self._buffer.size,
            dac_rate=self._dac_rate,
            max_dac_rate=30000,
        )

    def _emit(self, op: str, payload: dict) -> None:
        event = {"ts": time.monotonic(), "op": op, **payload}
        _log.debug("DryRun %s %s", op, payload)
        if self._sink is not None:
            try:
                self._sink(event)
            except Exception:
                _log.exception("DryRun event sink raised; suppressing")
