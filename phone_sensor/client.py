"""PhoneSensorClient — TCP command channel to the phone.

Owns the connection, sends commands and awaits their typed replies, runs a
heartbeat thread that flags the link unhealthy after PHONE_HEARTBEAT_TIMEOUT_S
of silence, dispatches unsolicited events (af_settled, thermal_warning, …) to
a callback, and reconnects with exponential backoff when the link drops.

The phone is dumb-on-purpose, so this class encodes most of the PC-side
orchestration: switching cameras, locking focus, starting streams. PhoneSensor
calls into it; tests can hand it a `connect_fn` factory that returns a fake
socket.

Reference: PHONE_SENSOR_BOOTSTRAP.md §2, §3.2.
"""
from __future__ import annotations

import logging
import socket
import threading
import time
from typing import Callable, Optional

from config import settings
from phone_sensor.protocol import (PROTOCOL_VERSION, PhoneCapabilities,
                                    StreamMode, pack_command, parse_message)

_log = logging.getLogger(__name__)

EventSink = Callable[[dict], None]
ConnectFn = Callable[[str, int, float], object]


def _default_connect(host: str, port: int, timeout_s: float) -> socket.socket:
    s = socket.create_connection((host, port), timeout=timeout_s)
    s.settimeout(None)                # blocking after connect; reader is on its own thread
    return s


class PhoneSensorClient:
    """One TCP control connection to the phone. Thread-safe."""

    def __init__(self,
                 host: str = settings.PHONE_IP,
                 port: int = settings.PHONE_CMD_PORT,
                 *,
                 heartbeat_interval_s: float = settings.PHONE_HEARTBEAT_INTERVAL_S,
                 heartbeat_timeout_s: float = settings.PHONE_HEARTBEAT_TIMEOUT_S,
                 reconnect_backoff_s: float = settings.PHONE_RECONNECT_BACKOFF_S,
                 connect_timeout_s: float = 3.0,
                 connect_fn: Optional[ConnectFn] = None,
                 on_event: Optional[EventSink] = None):
        self._host = host
        self._port = port
        self._hb_interval = heartbeat_interval_s
        self._hb_timeout = heartbeat_timeout_s
        self._backoff_s = reconnect_backoff_s
        self._connect_timeout = connect_timeout_s
        self._connect_fn = connect_fn or _default_connect
        self._on_event = on_event

        self._sock: Optional[object] = None
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._hb_thread: Optional[threading.Thread] = None

        self._cmd_id = 0
        self._cmd_lock = threading.Lock()
        # Outstanding cmd_id → (Event, [reply or None]) so callers can block
        # on send_command() until the matching reply arrives.
        self._pending: dict = {}
        self._pending_lock = threading.Lock()

        self._capabilities: Optional[PhoneCapabilities] = None
        self._connected = False
        self._last_rx_ts = 0.0
        self._healthy = False

    # ── Identity / state ─────────────────────────────────────────────────

    @property
    def host(self) -> str:
        return self._host

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_healthy(self) -> bool:
        """Connected AND heartbeat has been answered within the timeout."""
        return self._healthy

    @property
    def capabilities(self) -> Optional[PhoneCapabilities]:
        return self._capabilities

    def set_event_sink(self, on_event: Optional[EventSink]) -> None:
        self._on_event = on_event

    # ── Lifecycle ────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """Open the TCP link, send the connect handshake, populate
        `capabilities`, and start the reader + heartbeat threads. Idempotent;
        returns True if already connected."""
        if self._connected:
            return True
        try:
            self._sock = self._connect_fn(self._host, self._port,
                                          self._connect_timeout)
        except OSError as exc:
            _log.error("phone: TCP connect to %s:%d failed: %s",
                       self._host, self._port, exc)
            return False
        self._stop.clear()
        self._connected = True
        self._last_rx_ts = time.monotonic()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="PhoneReader", daemon=True)
        self._reader_thread.start()

        # Handshake — exchange protocol version + capabilities manifest.
        reply = self.send_command(
            "connect", protocol_version=PROTOCOL_VERSION, timeout_s=3.0)
        if reply is None or reply.get("ok") is False:
            _log.error("phone: handshake failed (reply=%r)", reply)
            self.disconnect()
            return False
        caps = reply.get("capabilities")
        if caps:
            self._capabilities = PhoneCapabilities.from_dict(caps)
            _log.info("phone: connected — %s, %d cameras",
                      self._capabilities.phone_model,
                      len(self._capabilities.cameras))

        # Heartbeat last — it tears down on timeout, so we want the link
        # actually usable before it starts watching.
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, name="PhoneHeartbeat", daemon=True)
        self._hb_thread.start()
        self._healthy = True
        return True

    def disconnect(self) -> None:
        """Close the link and stop threads. Idempotent."""
        self._stop.set()
        self._connected = False
        self._healthy = False
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        # Fail any pending commands so callers unblock cleanly.
        with self._pending_lock:
            for evt, slot in self._pending.values():
                slot[0] = None
                evt.set()
            self._pending.clear()
        for t in (self._reader_thread, self._hb_thread):
            if t is not None and t is not threading.current_thread():
                t.join(timeout=1.5)
        self._reader_thread = self._hb_thread = None

    # ── Commands ─────────────────────────────────────────────────────────

    def send_command(self, cmd: str, *, timeout_s: float = 2.0,
                     **params) -> Optional[dict]:
        """Send a command and block until its reply arrives, or `timeout_s`
        expires. Returns the parsed reply dict, or None on timeout / link
        loss."""
        if not self._connected or self._sock is None:
            _log.error("phone: send_command(%s) — not connected", cmd)
            return None
        with self._cmd_lock:
            self._cmd_id += 1
            cid = self._cmd_id
        wire = pack_command(cmd, cid, **params)
        evt = threading.Event()
        slot: list = [None]
        with self._pending_lock:
            self._pending[cid] = (evt, slot)
        try:
            with self._send_lock:
                self._sock.sendall(wire)
        except OSError as exc:
            _log.error("phone: send(%s) failed: %s", cmd, exc)
            with self._pending_lock:
                self._pending.pop(cid, None)
            return None
        if not evt.wait(timeout=timeout_s):
            _log.warning("phone: command %s (cmd_id=%d) timed out", cmd, cid)
            with self._pending_lock:
                self._pending.pop(cid, None)
            return None
        return slot[0]

    # ── Convenience wrappers (the PC orchestration vocabulary) ───────────

    def ping(self) -> bool:
        reply = self.send_command("ping", timeout_s=self._hb_timeout)
        return reply is not None and reply.get("ok", True)

    def set_active_camera(self, camera_id: str,
                          timeout_s: float = 2.5) -> bool:
        """Switch the active lens. Lens transitions take 300-800 ms; the reply
        comes after the camera is open and producing frames."""
        reply = self.send_command("set_active_camera", camera_id=camera_id,
                                  timeout_s=timeout_s)
        return reply is not None and reply.get("ok", False)

    def stream_start(self, mode: str = settings.PHONE_DEFAULT_STREAM_MODE,
                     width: int = settings.PHONE_DEFAULT_STREAM_WIDTH,
                     height: int = settings.PHONE_DEFAULT_STREAM_HEIGHT,
                     fps: int = settings.PHONE_DEFAULT_STREAM_FPS) -> Optional[str]:
        """Begin streaming. Returns the negotiated mode the phone actually
        chose (it may downgrade), or None on failure."""
        if mode not in {m.value for m in StreamMode}:
            raise ValueError(f"unknown stream mode: {mode!r}")
        reply = self.send_command("stream_start", mode=mode,
                                  width=width, height=height, fps=fps,
                                  timeout_s=3.0)
        if reply is None or not reply.get("ok", False):
            return None
        return reply.get("mode", mode)

    def stream_stop(self) -> bool:
        reply = self.send_command("stream_stop")
        return reply is not None and reply.get("ok", False)

    def set_af_region(self, x: float, y: float, w: float, h: float) -> bool:
        reply = self.send_command("set_af_region", x=x, y=y, w=w, h=h)
        return reply is not None and reply.get("ok", False)

    def lock_focus(self) -> bool:
        reply = self.send_command("lock_focus")
        return reply is not None and reply.get("ok", False)

    def get_status(self) -> Optional[dict]:
        return self.send_command("get_status")

    # ── Reader thread (replies + unsolicited events) ─────────────────────

    def _reader_loop(self) -> None:
        buf = b""
        sock = self._sock
        while not self._stop.is_set() and sock is not None:
            try:
                chunk = sock.recv(8192)
            except OSError:
                break
            if not chunk:
                break                # phone closed the link
            buf += chunk
            self._last_rx_ts = time.monotonic()
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line:
                    continue
                try:
                    msg = parse_message(line)
                except ValueError as exc:
                    _log.warning("phone: %s — dropping line", exc)
                    continue
                self._dispatch(msg)
        # If we fell out of the loop without an explicit disconnect(), the
        # heartbeat will notice (timeout) and the higher-level reconnect
        # logic in PhoneSensor brings the link back. Mark unhealthy now so
        # send_command stops trying.
        self._healthy = False

    def _dispatch(self, msg: dict) -> None:
        cid = msg.get("cmd_id")
        if cid is not None:
            with self._pending_lock:
                pending = self._pending.pop(cid, None)
            if pending is not None:
                evt, slot = pending
                slot[0] = msg
                evt.set()
                return
        # No matching pending → unsolicited event.
        if self._on_event is not None:
            try:
                self._on_event(msg)
            except Exception:
                _log.exception("phone: event sink raised; suppressing")

    # ── Heartbeat thread ─────────────────────────────────────────────────

    def _heartbeat_loop(self) -> None:
        while not self._stop.wait(self._hb_interval):
            if not self._connected:
                return
            # Drive a ping AND check for receive silence. Either failure mode
            # (no reply, no recv at all) flips us to unhealthy.
            ok = self.ping()
            silence = time.monotonic() - self._last_rx_ts
            healthy = ok and silence <= self._hb_timeout
            if not healthy and self._healthy:
                _log.warning("phone: link unhealthy (ping_ok=%s, silence=%.1fs)",
                             ok, silence)
            self._healthy = healthy
