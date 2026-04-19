"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

LaserCube Interface Module
--------------------------
Low-level UDP communication driver for the Wicked Lasers / LaserOS LaserCube
RGB laser projector.

⚠️  SAFETY WARNING ⚠️
    The LaserCube is a CLASS 3B laser device.  Never look into the beam.
    Never point at people, animals, or flammable materials.
    Always use the safety interlocks in utils/safety.py.

━━━ Protocol Overview ━━━
The LaserCube connects over its own WiFi AP (SSID: "LaserCube-XXXXXX").
Default IP: 192.168.1.1
Two UDP channels:
  DATA_PORT  (45456): Stream point frame packets for galvo scanning.
  CMD_PORT   (45457): Send commands; receive status replies.

Command bytes (sent to CMD_PORT):
  0x40  GET_ALIVE                 → Device echoes back alive packet
  0x41  SET_ILDA_RATE  + 4-byte rate (uint32 LE) → Set galvo scan rate
  0x42  GET_ILDA_RATE             → Returns current rate
  0x43  GET_RINGBUFFER_EMPTY      → Returns empty sample count
  0x44  GET_LAST_STATUS           → Returns status byte
  0x45  SET_OUTPUT + 1-byte bool  → Enable (1) / Disable (0) laser output
  0x46  WRITE_DAC + [points]      → Stream N points (DAC frame data)

Point format for WRITE_DAC (each point = 7 bytes):
  Bytes 0–1: X  (int16, little-endian, range –32767 … +32767)
  Bytes 2–3: Y  (int16, little-endian, range –32767 … +32767)
  Byte  4:   R  (uint8, 0–255)
  Byte  5:   G  (uint8, 0–255)
  Byte  6:   B  (uint8, 0–255)

NOTE: Official protocol documentation is available in the LaserOS SDK.
      Some details here are derived from community reverse-engineering.
      If behaviour differs from hardware, consult the SDK docs.

Modules imported:
  socket, struct, threading — standard library networking / binary packing
  numpy                     — point array construction
  config.settings           — LASERCUBE_* constants
  utils.logging_utils       — get_logger

Classes:
  LaserPoint             — NamedTuple (x, y, r, g, b) describing one point.
  LaserCubeInterface     — UDP socket-based driver for the LaserCube.

Methods (LaserCubeInterface):
  connect()              — Open UDP sockets and verify device responds to alive.
  disconnect()           — Disable output and close sockets.
  enable_output(enabled) — Send SET_OUTPUT command to turn laser on/off.
  set_scan_rate(rate)    — Set the galvo scan rate in points-per-second.
  send_frame(points)     — Send a list of LaserPoints as one DAC frame.
  send_blank_frame()     — Send a frame of dark (r=g=b=0) points to black out.
  get_alive()            — Ping device; True if it responds.
  is_connected()         — True if sockets are open and alive check passed.
  build_blank_points(n)  — Return N blank (off) LaserPoint objects.
  build_single_point_frame(x,y,r,g,b,repeat) — Helper for targeting single dot.

Variables (instance):
  ip               (str):  Target device IP address.
  data_port        (int):  UDP data port.
  cmd_port         (int):  UDP command port.
  _data_sock       (socket.socket | None): UDP socket for frames.
  _cmd_sock        (socket.socket | None): UDP socket for commands.
  _connected       (bool): Whether sockets are open.
  _output_enabled  (bool): Whether SET_OUTPUT(1) has been called.
  _alive_timeout   (float): Seconds to wait for alive reply.

#todo Add support for the LaserCube SDK DLL via ctypes as an alternative path.
#todo Implement ringbuffer fill monitoring to avoid over/underflow artifacts.
#todo Add colour correction LUT (gamma table) per-channel for accurate targeting dot.
#todo Support LaserCube USB connection path (vs WiFi) using HID protocol.
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from typing import NamedTuple

import config.settings as _s
from utils.logging_utils import get_logger

logger = get_logger(__name__)

# Per-point byte format: 2×int16 + 3×uint8 = 7 bytes
_POINT_STRUCT = struct.Struct("<hhBBB")
_POINT_SIZE = _POINT_STRUCT.size  # 7 bytes


class LaserPoint(NamedTuple):
    """
    Single point for the LaserCube DAC stream.

    Attributes:
        x (int): Horizontal position, –32767 (left) to +32767 (right).
        y (int): Vertical position,   –32767 (bottom) to +32767 (top).
        r (int): Red channel intensity 0–255.
        g (int): Green channel intensity 0–255.
        b (int): Blue channel intensity 0–255.
    """
    x: int
    y: int
    r: int
    g: int
    b: int


class LaserCubeInterface:
    """
    Low-level UDP communication driver for the LaserCube RGB laser projector.

    All public methods are thread-safe (protected by a single reentrant lock).
    Network calls have configurable timeouts; failures are logged and return
    False / empty rather than raising exceptions.

    Attributes:
        ip              (str):   LaserCube IP address.
        data_port       (int):   UDP port for DAC frame streaming.
        cmd_port        (int):   UDP port for command/status.
        _data_sock      (socket.socket | None): Streaming socket.
        _cmd_sock       (socket.socket | None): Command socket.
        _connected      (bool):  True after successful open + alive check.
        _output_enabled (bool):  True after SET_OUTPUT(1).
        _lock           (threading.RLock): Protects all socket operations.
        _alive_timeout  (float): Seconds to wait for GET_ALIVE reply.
    """

    def __init__(
        self,
        ip: str = _s.LASERCUBE_DEFAULT_IP,
        data_port: int = _s.LASERCUBE_DATA_PORT,
        cmd_port: int = _s.LASERCUBE_CMD_PORT,
    ) -> None:
        """
        Initialise interface parameters but do NOT open sockets yet.

        Args:
            ip:        LaserCube IP address.
            data_port: UDP data port (default 45456).
            cmd_port:  UDP command port (default 45457).
        """
        self.ip: str = ip
        self.data_port: int = data_port
        self.cmd_port: int = cmd_port
        self._data_sock: socket.socket | None = None
        self._cmd_sock: socket.socket | None = None
        self._connected: bool = False
        self._output_enabled: bool = False
        self._lock: threading.RLock = threading.RLock()
        self._alive_timeout: float = 2.0
        logger.debug("LaserCubeInterface created for %s:%d/%d", ip, data_port, cmd_port)

    # ──────────────────────────────────────────────────────────────────────────
    # Connection management
    # ──────────────────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Open UDP sockets and verify the LaserCube is alive.

        Returns:
            True if device responded to GET_ALIVE, False otherwise.
        """
        with self._lock:
            try:
                # Data socket (send-only)
                self._data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)

                # Command socket (send + receive)
                self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._cmd_sock.settimeout(self._alive_timeout)

                logger.info("LaserCube sockets opened; pinging %s …", self.ip)
                alive = self._get_alive_unlocked()
                if alive:
                    self._connected = True
                    logger.info("LaserCube connected at %s.", self.ip)
                else:
                    logger.error(
                        "LaserCube at %s did not respond to alive ping. "
                        "Check WiFi connection.", self.ip
                    )
                    self._close_sockets_unlocked()
                return alive

            except OSError as exc:
                logger.error("LaserCube socket error: %s", exc)
                self._close_sockets_unlocked()
                return False

    def disconnect(self) -> None:
        """
        Disable laser output and close all sockets.
        Safe to call even if not connected.
        """
        with self._lock:
            if self._output_enabled:
                self._send_command_unlocked(bytes([_s.LC_CMD_SET_OUTPUT, 0]))
                self._output_enabled = False
            self._close_sockets_unlocked()
            self._connected = False
            logger.info("LaserCube disconnected.")

    def is_connected(self) -> bool:
        """Return True if the interface currently has open, validated sockets."""
        with self._lock:
            return self._connected

    # ──────────────────────────────────────────────────────────────────────────
    # Output control
    # ──────────────────────────────────────────────────────────────────────────

    def enable_output(self, enabled: bool) -> bool:
        """
        Enable or disable laser beam output via SET_OUTPUT command.

        ⚠️  Enabling output allows the laser to actually fire.
        Only call this after safety checks have passed.

        Args:
            enabled: True to enable, False to disable.

        Returns:
            True if the command was sent successfully.
        """
        with self._lock:
            byte_val = 1 if enabled else 0
            ok = self._send_command_unlocked(bytes([_s.LC_CMD_SET_OUTPUT, byte_val]))
            if ok:
                self._output_enabled = enabled
                state = "ENABLED" if enabled else "DISABLED"
                logger.warning("LaserCube output %s.", state)
            return ok

    def set_scan_rate(self, rate: int) -> bool:
        """
        Set the DAC / galvo scan rate in points-per-second.

        Args:
            rate: Desired scan rate. Must not exceed LASERCUBE_SCAN_RATE.

        Returns:
            True if command sent successfully.
        """
        rate = min(rate, _s.LASERCUBE_SCAN_RATE)
        with self._lock:
            payload = bytes([_s.LC_CMD_SET_ILDA_RATE]) + struct.pack("<I", rate)
            ok = self._send_command_unlocked(payload)
            if ok:
                logger.debug("Scan rate set to %d pps.", rate)
            return ok

    # ──────────────────────────────────────────────────────────────────────────
    # Frame streaming
    # ──────────────────────────────────────────────────────────────────────────

    def send_frame(self, points: list[LaserPoint]) -> bool:
        """
        Stream a list of LaserPoints as a single DAC frame.

        The LaserCube loops over frames continuously — sending a frame replaces
        the current contents of the scan loop with the new point list.

        Args:
            points: List of LaserPoint objects.  Maximum LASERCUBE_MAX_POINTS_PER_FRAME.

        Returns:
            True if the UDP packet was sent without OS error, False otherwise.
        """
        if not points:
            return self.send_blank_frame()

        # Clamp to max frame size
        if len(points) > _s.LASERCUBE_MAX_POINTS_PER_FRAME:
            points = points[: _s.LASERCUBE_MAX_POINTS_PER_FRAME]

        with self._lock:
            if self._data_sock is None:
                return False
            try:
                # Build payload: CMD byte + count (uint16 LE) + packed points
                n = len(points)
                header = bytes([_s.LC_CMD_WRITE_DAC]) + struct.pack("<H", n)
                point_bytes = bytearray(n * _POINT_SIZE)
                for i, pt in enumerate(points):
                    _POINT_STRUCT.pack_into(
                        point_bytes,
                        i * _POINT_SIZE,
                        self._clamp_coord(pt.x),
                        self._clamp_coord(pt.y),
                        self._clamp_color(pt.r),
                        self._clamp_color(pt.g),
                        self._clamp_color(pt.b),
                    )
                packet = header + bytes(point_bytes)
                self._data_sock.sendto(packet, (self.ip, self.data_port))
                return True
            except OSError as exc:
                logger.error("send_frame error: %s", exc)
                self._connected = False
                return False

    def send_blank_frame(self, n: int = 100) -> bool:
        """
        Send a frame of *n* dark (r=g=b=0) points to blank the laser output.

        Args:
            n: Number of blank points (affects scan repetition rate).

        Returns:
            True if sent successfully.
        """
        blank_points = self.build_blank_points(n)
        return self.send_frame(blank_points)

    def build_blank_points(self, n: int = 100) -> list[LaserPoint]:
        """
        Return a list of *n* laser-off (black) points at the origin.

        Args:
            n: Number of blank points to generate.

        Returns:
            List of LaserPoint(0, 0, 0, 0, 0).
        """
        return [LaserPoint(0, 0, 0, 0, 0)] * n

    def build_single_point_frame(
        self,
        x: int,
        y: int,
        r: int = _s.LASER_DEFAULT_POWER_R,
        g: int = _s.LASER_DEFAULT_POWER_G,
        b: int = _s.LASER_DEFAULT_POWER_B,
        repeat: int = _s.LASER_BURST_REPEAT,
    ) -> list[LaserPoint]:
        """
        Build a frame that dwells on a single point by repeating it *repeat* times.

        This is the standard targeting frame: pointing the laser at one spot.

        Args:
            x:      Laser X coordinate (–32767 … +32767).
            y:      Laser Y coordinate (–32767 … +32767).
            r:      Red intensity 0–255.
            g:      Green intensity 0–255.
            b:      Blue intensity 0–255.
            repeat: Number of times to repeat the point in the frame.

        Returns:
            List of LaserPoint objects (all identical).
        """
        pt = LaserPoint(
            x=self._clamp_coord(x),
            y=self._clamp_coord(y),
            r=self._clamp_color(r),
            g=self._clamp_color(g),
            b=self._clamp_color(b),
        )
        return [pt] * max(1, repeat)

    # ──────────────────────────────────────────────────────────────────────────
    # Status queries
    # ──────────────────────────────────────────────────────────────────────────

    def get_alive(self) -> bool:
        """
        Send GET_ALIVE and return True if the device replies.

        Returns:
            True if alive response received within _alive_timeout seconds.
        """
        with self._lock:
            return self._get_alive_unlocked()

    def get_last_status(self) -> int | None:
        """
        Request and return the last status byte from the LaserCube.

        Returns:
            Status byte as int, or None if no response.
        """
        with self._lock:
            return self._query_command_unlocked(bytes([_s.LC_CMD_GET_LAST_STATUS]))

    def get_scan_rate(self) -> int | None:
        """
        Request and return the current scan rate in points-per-second.

        Returns:
            Scan rate as int, or None if no response.
        """
        with self._lock:
            raw = self._query_command_unlocked(bytes([_s.LC_CMD_GET_ILDA_RATE]), expect_bytes=4)
            if raw is not None and len(raw) >= 4:
                return struct.unpack("<I", raw[:4])[0]
            return None

    # ──────────────────────────────────────────────────────────────────────────
    # Private helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _get_alive_unlocked(self) -> bool:
        """Send GET_ALIVE and return True if any reply received.  Lock must be held."""
        resp = self._query_command_unlocked(bytes([_s.LC_CMD_GET_ALIVE]), expect_bytes=1)
        return resp is not None

    def _send_command_unlocked(self, payload: bytes) -> bool:
        """
        Send a command packet to the CMD_PORT without waiting for a response.
        Lock must be held by caller.
        """
        if self._cmd_sock is None:
            return False
        try:
            self._cmd_sock.sendto(payload, (self.ip, self.cmd_port))
            return True
        except OSError as exc:
            logger.error("LaserCube command send error: %s", exc)
            self._connected = False
            return False

    def _query_command_unlocked(
        self,
        payload: bytes,
        expect_bytes: int = 1,
    ) -> bytes | None:
        """
        Send a command packet and wait for a UDP reply.
        Lock must be held by caller.

        Args:
            payload:      Bytes to transmit.
            expect_bytes: How many bytes to expect in the reply.

        Returns:
            Reply bytes, or None on timeout/error.
        """
        if self._cmd_sock is None:
            return None
        try:
            self._cmd_sock.sendto(payload, (self.ip, self.cmd_port))
            data, _ = self._cmd_sock.recvfrom(256)
            return data[:expect_bytes] if data else None
        except socket.timeout:
            logger.debug("LaserCube command timed out (payload[0]=%02X).", payload[0])
            return None
        except OSError as exc:
            logger.error("LaserCube query error: %s", exc)
            self._connected = False
            return None

    def _close_sockets_unlocked(self) -> None:
        """Close both UDP sockets.  Lock must be held by caller."""
        for sock_attr in ("_data_sock", "_cmd_sock"):
            sock = getattr(self, sock_attr, None)
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
                setattr(self, sock_attr, None)

    @staticmethod
    def _clamp_coord(value: int) -> int:
        """Clamp a coordinate to the valid signed-int16 laser range."""
        return max(_s.LASERCUBE_COORD_MIN, min(_s.LASERCUBE_COORD_MAX, int(value)))

    @staticmethod
    def _clamp_color(value: int) -> int:
        """Clamp a colour channel to 0–255."""
        return max(0, min(255, int(value)))
