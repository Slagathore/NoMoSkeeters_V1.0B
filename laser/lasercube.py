"""LaserCubeInterface — live UDP transport for the WiFi LaserCube.

Implements the canonical libLaserdockCore protocol (verified against Cole's
hardware via lasercube_protocol_probe.py). Satisfies the LaserCubeTransport
ABC so the rest of the bus is agnostic to live-vs-dry-run.

Reference: BOOTSTRAP.md §9.5, BOOTSTRAP_AMENDMENTS.md §9.5 (strict reply
validation, best-effort disconnect, BufferEstimate) + §13.5 (source-IP
auto-detection for the multi-NIC routing fix).
"""
from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from typing import Callable, Optional

from laser.protocol import (
    LC_CMD_GET_FULL_INFO,
    LC_CMD_GET_RINGBUF_EMPTY_COUNT,
    LC_CMD_SET_OUTPUT,
    LC_CMD_SET_ILDA_RATE,
    LC_CMD_CLEAR_RINGBUFFER,
    CMD_PORT, DATA_PORT, ALIVE_PORT,
    build_sample_packet,
    parse_full_info, parse_ringbuf_empty,
)
from laser.transport import LaserCubeTransport
from laser.types import BufferEstimate, LaserInfo, LaserPoint


_log = logging.getLogger(__name__)

EventSink = Callable[[dict], None]

# Socket buffer sizing, per libLaserdockCore (BOOTSTRAP.md §9.5).
_SOCK_BUF_BYTES = 5_250_000


def _autodetect_src_ip(cube_ip: str) -> str:
    """Pick a local bind address on the same /16 as the cube (§13.5).

    The multi-NIC bug: binding to 0.0.0.0 lets Windows routing pick a source
    interface, and it sometimes picks WiFi — the cube then replies to an IP
    that isn't on its network and the datagram is dropped. So we enumerate
    local IPv4 addresses and prefer one sharing the cube's first two octets
    (e.g. 169.254.x.x for an APIPA direct-ethernet link). Falls back to
    0.0.0.0 if nothing matches.
    """
    prefix = ".".join(cube_ip.split(".")[:2]) + "."   # "169.254."
    candidates: set[str] = set()
    try:
        host = socket.gethostname()
        for info in socket.getaddrinfo(host, None, socket.AF_INET):
            candidates.add(info[4][0])
        candidates.update(socket.gethostbyname_ex(host)[2])
    except OSError:
        pass
    for addr in sorted(candidates):
        if addr.startswith(prefix):
            return addr
    return "0.0.0.0"


class LaserCubeInterface(LaserCubeTransport):
    """Live network LaserCube driver. Thread-safe.

    Usage:
        cube = LaserCubeInterface(ip="169.254.40.83", src_ip="auto")
        cube.connect()
        info = cube.get_full_info()
        cube.send_frame(points, frame_num=0)
        cube.disconnect()
    """

    def __init__(self,
                 ip: str,
                 src_ip: str = "auto",
                 cmd_port: int = CMD_PORT,
                 data_port: int = DATA_PORT,
                 alive_port: int = ALIVE_PORT,
                 reply_timeout_s: float = 1.5,
                 buffer_size: int = 6000,
                 event_sink: Optional[EventSink] = None):
        self.ip = ip
        self.src_ip = src_ip          # "auto" is resolved in connect()
        self.cmd_port = cmd_port
        self.data_port = data_port
        self.alive_port = alive_port
        self.reply_timeout_s = reply_timeout_s

        self._cmd_sock: Optional[socket.socket] = None
        self._data_sock: Optional[socket.socket] = None

        # _lock guards socket lifecycle + shared mutable state (sockets,
        # _connected, _msg_num, _buffer). RLock so connect()/disconnect()
        # can call _send_cmd_no_reply() reentrantly.
        self._lock = threading.RLock()
        # _txn_lock serializes whole request/response transactions so two
        # callers (e.g. heartbeat thread + main thread) can't steal each
        # other's reply datagrams. Deliberately NOT held by _send_cmd_no_reply
        # so an e-stop disable_output() is never blocked behind a 1.5s recv.
        self._txn_lock = threading.Lock()

        self._connected = False
        self._msg_num = 0
        self._buffer = BufferEstimate(free=0, size=buffer_size,
                                      last_refresh_ts=0.0)
        self._sink = event_sink

    # ── Lifecycle ────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """Open the cmd + data UDP sockets and verify with GET_FULL_INFO."""
        with self._lock:
            if self.src_ip == "auto":
                self.src_ip = _autodetect_src_ip(self.ip)
                _log.info("auto-detected source IP %s for cube %s",
                          self.src_ip, self.ip)
            try:
                # CMD socket: bound to (src_ip, CMD_PORT), send + recv.
                self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, _SOCK_BUF_BYTES)
                self._cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, _SOCK_BUF_BYTES)
                self._cmd_sock.bind((self.src_ip, self.cmd_port))
                self._cmd_sock.settimeout(self.reply_timeout_s)

                # DATA socket: bound to (src_ip, DATA_PORT), send-only.
                self._data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, _SOCK_BUF_BYTES)
                self._data_sock.bind((self.src_ip, self.data_port))
            except OSError as exc:
                _log.error("socket setup failed for %s (src=%s): %s",
                           self.ip, self.src_ip, exc)
                self._close_sockets_unlocked()
                return False

        # Verify the cube actually answers. _send_cmd_recv takes its own locks.
        data = self._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO]),
                                   expect_min_bytes=38,
                                   expect_echo_byte=LC_CMD_GET_FULL_INFO)
        if data is None:
            _log.error("no GET_FULL_INFO reply from %s; connect failed", self.ip)
            with self._lock:
                self._close_sockets_unlocked()
            return False

        info = parse_full_info(data)
        with self._lock:
            self._connected = True
            if info is not None:
                self._buffer = BufferEstimate(
                    free=info.buffer_free, size=info.buffer_size,
                    last_refresh_ts=time.monotonic())
        _log.info("connected to %s (fw %s.%s, %s free / %s)",
                  self.ip,
                  info.fw_major if info else "?",
                  info.fw_minor if info else "?",
                  self._buffer.free, self._buffer.size)
        self._emit("connect", {"ip": self.ip, "src_ip": self.src_ip})
        return True

    def disconnect(self) -> None:
        """Disable output (best-effort) and close sockets — §9.5 amendment.

        Attempts SET_OUTPUT(0) whenever the cmd socket exists, regardless of
        _connected, since that flag can be stale after a comms loss.
        """
        with self._lock:
            if self._cmd_sock is not None:
                try:
                    self._send_cmd_no_reply(
                        bytes([LC_CMD_SET_OUTPUT, 0x00]), repeat=2)
                except Exception:
                    _log.exception("disconnect: SET_OUTPUT(0) failed; "
                                    "closing anyway")
            self._close_sockets_unlocked()
            self._connected = False
        self._emit("disconnect", {})

    def is_connected(self) -> bool:
        return self._connected

    # ── Read-only commands ───────────────────────────────────────────────

    def get_full_info(self) -> Optional[LaserInfo]:
        data = self._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO]),
                                   expect_min_bytes=38,
                                   expect_echo_byte=LC_CMD_GET_FULL_INFO)
        if data is None:
            return None
        info = parse_full_info(data)
        if info is not None:
            with self._lock:
                self._buffer.free = info.buffer_free
                self._buffer.size = info.buffer_size
                self._buffer.last_refresh_ts = time.monotonic()
        self._emit("get_full_info",
                   {"output_enabled": info.output_enabled if info else None})
        return info

    def get_ringbuf_empty(self) -> Optional[int]:
        data = self._send_cmd_recv(bytes([LC_CMD_GET_RINGBUF_EMPTY_COUNT]),
                                   expect_min_bytes=4,
                                   expect_echo_byte=LC_CMD_GET_RINGBUF_EMPTY_COUNT)
        if data is None:
            return None
        free = parse_ringbuf_empty(data)
        if free is not None:
            with self._lock:
                self._buffer.free = free
                self._buffer.last_refresh_ts = time.monotonic()
        self._emit("get_ringbuf_empty", {"free": free})
        return free

    # ── State-modifying commands ─────────────────────────────────────────

    def enable_output(self) -> bool:
        """Enable laser output (SET_OUTPUT 0x01).

        DANGER: this requests Class 3B laser emission. No production path
        calls it — per BOOTSTRAP_AMENDMENTS §17 Step 5, only the §9.12
        SHA204 cold-test fixture invokes enable_output(). The safety layer
        (Step 12+) owns the production fire gate.
        """
        _log.warning("enable_output() called — laser output ENABLE requested")
        ok = self._send_cmd_no_reply(bytes([LC_CMD_SET_OUTPUT, 0x01]), repeat=2)
        self._emit("enable_output", {"ok": ok})
        return ok

    def disable_output(self) -> bool:
        """Disable laser output (SET_OUTPUT 0x00). Idempotent."""
        ok = self._send_cmd_no_reply(bytes([LC_CMD_SET_OUTPUT, 0x00]), repeat=2)
        self._emit("disable_output", {"ok": ok})
        return ok

    def set_dac_rate(self, rate: int) -> bool:
        """Set scan rate in points/second. Clamped to [1000, 30000]."""
        rate = max(1000, min(30_000, int(rate)))
        payload = bytes([LC_CMD_SET_ILDA_RATE]) + struct.pack("<I", rate)
        ok = self._send_cmd_no_reply(payload, repeat=2)
        self._emit("set_dac_rate", {"rate": rate, "ok": ok})
        return ok

    def clear_ringbuffer(self) -> bool:
        """Drop all queued samples. Useful on e-stop."""
        ok = self._send_cmd_no_reply(bytes([LC_CMD_CLEAR_RINGBUFFER]), repeat=2)
        if ok:
            with self._lock:
                # The cube's buffer is now empty; mark the estimate stale so
                # callers refresh via get_ringbuf_empty() before gating on it.
                self._buffer.free = self._buffer.size
                self._buffer.last_refresh_ts = 0.0
        self._emit("clear_ringbuffer", {"ok": ok})
        return ok

    # ── Sample data streaming ────────────────────────────────────────────

    def send_chunk(self,
                   points: list[LaserPoint],
                   msg_num: int,
                   frame_num: int) -> bool:
        """Send ONE sample-data UDP packet on DATA_PORT."""
        packet = build_sample_packet([p.packed() for p in points],
                                     msg_num, frame_num)
        with self._lock:
            if self._data_sock is None:
                return False
            try:
                self._data_sock.sendto(packet, (self.ip, self.data_port))
            except OSError as exc:
                _log.error("send_chunk: sendto failed: %s", exc)
                return False
            # Estimate only — decrement goes stale on dropped replies (§9.5).
            self._buffer.free = max(0, self._buffer.free - len(points))
            buf_free = self._buffer.free
        self._emit("send_chunk", {"n_points": len(points), "msg_num": msg_num,
                                  "frame_num": frame_num,
                                  "buffer_free_estimate": buf_free})
        return True

    def send_frame(self,
                   points: list[LaserPoint],
                   frame_num: int = 0) -> bool:
        """Split a point list across UDP packets and send, advancing msg_num
        per packet (overrides the ABC default for tight sequencing)."""
        if not points:
            return True
        from laser.protocol import MAX_SAMPLES_PER_PACKET
        ok = True
        for i in range(0, len(points), MAX_SAMPLES_PER_PACKET):
            chunk = points[i:i + MAX_SAMPLES_PER_PACKET]
            if not self.send_chunk(chunk, self._next_msg_num(), frame_num):
                ok = False
                break
        return ok

    def _next_msg_num(self) -> int:
        with self._lock:
            n = self._msg_num
            self._msg_num = (self._msg_num + 1) & 0xFF
            return n

    # ── State surface ────────────────────────────────────────────────────

    def buffer_estimate(self) -> BufferEstimate:
        """Last-known buffer state — MAY BE STALE. Consult is_fresh() and
        refresh via get_ringbuf_empty() before gating any decision on it."""
        return self._buffer

    # ── Internal ─────────────────────────────────────────────────────────

    def _send_cmd_recv(self,
                       payload: bytes,
                       expect_min_bytes: int = 1,
                       expect_echo_byte: Optional[int] = None) -> Optional[bytes]:
        """Send a command on CMD_PORT and wait for a matching reply (§9.5).

        Accepts a datagram only if: source == (self.ip, self.cmd_port),
        len >= expect_min_bytes, reply[0] == expect_echo_byte, and the status
        byte reply[1] == 0x00. Mismatched datagrams (LaserOS chatter, stale
        crud) are dropped; loops until a match or the timeout.
        """
        if expect_echo_byte is None:
            expect_echo_byte = payload[0]
        deadline = time.monotonic() + self.reply_timeout_s

        with self._txn_lock:
            with self._lock:
                if self._cmd_sock is None:
                    return None
                sock = self._cmd_sock
                try:
                    sock.sendto(payload, (self.ip, self.cmd_port))
                except OSError as exc:
                    _log.error("_send_cmd_recv: sendto failed: %s", exc)
                    return None

            while time.monotonic() < deadline:
                remaining = max(0.01, deadline - time.monotonic())
                try:
                    sock.settimeout(remaining)
                    data, addr = sock.recvfrom(4096)
                except (socket.timeout, OSError):
                    return None
                # Source filter — must be our cube's command port.
                if addr[0] != self.ip or addr[1] != self.cmd_port:
                    continue
                # Length filter.
                if len(data) < expect_min_bytes:
                    continue
                # Command-echo filter.
                if expect_echo_byte is not None and data[0] != expect_echo_byte:
                    continue
                # Status filter — non-zero status is a cube-reported failure.
                if len(data) >= 2 and data[1] != 0x00:
                    return None
                return data
        return None

    def _send_cmd_no_reply(self, payload: bytes, repeat: int = 1) -> bool:
        """Send a fire-and-forget command on CMD_PORT, with UDP retransmits.

        Does not take _txn_lock, so an e-stop disable_output() is never
        blocked behind an in-flight _send_cmd_recv() recv timeout.
        """
        with self._lock:
            if self._cmd_sock is None:
                return False
            try:
                for _ in range(max(1, repeat)):
                    self._cmd_sock.sendto(payload, (self.ip, self.cmd_port))
                return True
            except OSError as exc:
                _log.error("_send_cmd_no_reply: sendto failed: %s", exc)
                return False

    def _close_sockets_unlocked(self) -> None:
        for attr in ("_cmd_sock", "_data_sock"):
            sock = getattr(self, attr, None)
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
                setattr(self, attr, None)

    def _emit(self, op: str, payload: dict) -> None:
        _log.debug("LaserCube %s %s", op, payload)
        if self._sink is not None:
            try:
                self._sink({"ts": time.monotonic(), "op": op, **payload})
            except Exception:
                _log.exception("LaserCube event sink raised; suppressing")
