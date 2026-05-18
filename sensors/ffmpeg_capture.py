"""FfmpegStreamCapture — receive the GoPro webcam UDP stream and decode it.

Why this exists, and why it is shaped the way it is — all verified on a
Hero 13 (fw H24.01.02.10.00) on 2026-05-17:

* The Hero 13 delivers usable video through **webcam mode**
  (`/gopro/webcam/start`), not the `/gopro/camera/stream/start` preview.
* The webcam feed is **H.264 1920x1080 29.97fps MPEG-TS on UDP 8554**.
* A socket bound to `0.0.0.0:8554` does NOT receive it on a multi-NIC
  Windows host, and ffmpeg's own `udp://` URL handling filters by source
  address — both come up empty. A plain socket bound to the **host's IP on
  the GoPro interface** receives every packet.

So this class binds that socket itself, pipes the MPEG-TS datagrams into an
ffmpeg subprocess on stdin, and reads decoded BGR frames from its stdout. A
reader thread + bounded queue means `read()` returns the latest frame and
never blocks forever if the feed dies. The public surface matches
cv2.VideoCapture (isOpened / read / release), so it drops into GoProSensor
via the capture_factory seam.
"""
from __future__ import annotations

import logging
import queue
import socket
import subprocess
import threading
from typing import Callable, Optional, Tuple

import numpy as np

_log = logging.getLogger(__name__)

_DEFAULT_CAMERA_IP = "172.27.109.51"
_DEFAULT_PORT = 8554
_DEFAULT_WIDTH = 1920
_DEFAULT_HEIGHT = 1080
# Hold only the newest decoded frame — targeting wants the freshest possible
# image, not a backlog. A deeper queue just adds latency.
_FRAME_QUEUE_MAX = 1


def local_ip_for(camera_ip: str) -> str:
    """The host's own IPv4 on the interface that reaches `camera_ip`.

    Prefers a local address in the camera's /24 (the GoPro USB subnet),
    because a routing-table lookup can mis-pick another NIC when a VPN is
    up. Falls back to a UDP `connect()` probe (sends nothing).
    """
    prefix = camera_ip.rsplit(".", 1)[0] + "."
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None,
                                       socket.AF_INET):
            ip = info[4][0]
            if ip.startswith(prefix):
                return ip
    except socket.gaierror:
        pass
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((camera_ip, 9))         # discard port; nothing is sent
        return probe.getsockname()[0]
    except OSError:
        return "0.0.0.0"
    finally:
        probe.close()


def build_ffmpeg_command(ffmpeg_path: str, hwaccel: Optional[str]) -> list:
    """ffmpeg argv: read MPEG-TS from stdin, emit raw bgr24 on stdout.

    Tuned for low latency: a short probe, no demux/decode buffering, and
    per-packet output flushing. `hwaccel` (e.g. "cuda" for NVDEC) offloads
    decode off the CPU — worth it for sustained 1080p30 — but is off by
    default so a missing GPU build cannot break capture.
    """
    cmd = [ffmpeg_path]
    if hwaccel:
        cmd += ["-hwaccel", hwaccel]
    cmd += [
        "-probesize", "500000",         # identify the stream fast (low startup)
        "-analyzeduration", "0",
        "-fflags", "nobuffer",          # no demuxer buffering
        "-flags", "low_delay",          # no decoder reordering buffer
        "-f", "mpegts",
        "-i", "pipe:0",
        "-an",                          # ignore the audio + metadata streams
        "-f", "rawvideo",
        "-pix_fmt", "bgr24",
        "-flush_packets", "1",          # push each decoded frame out at once
        "-loglevel", "error",
        "-",
    ]
    return cmd


class FfmpegStreamCapture:
    """GoPro webcam UDP decoder with a cv2.VideoCapture-style API.

    On construction it binds the UDP socket, spawns ffmpeg, and starts the
    feeder (socket -> ffmpeg stdin) and reader (ffmpeg stdout -> frame queue)
    threads. `read()` returns the most recent decoded frame, or (False, None)
    if none arrives within `read_timeout_s`. It does NOT start the camera
    streaming — that is the control layer's job (GoProInterface webcam mode).
    """

    def __init__(self,
                 camera_ip: str = _DEFAULT_CAMERA_IP,
                 port: int = _DEFAULT_PORT,
                 width: int = _DEFAULT_WIDTH,
                 height: int = _DEFAULT_HEIGHT,
                 *,
                 bind_ip: Optional[str] = None,
                 read_timeout_s: float = 2.0,
                 ffmpeg_path: str = "ffmpeg",
                 hwaccel: Optional[str] = None,
                 popen: Optional[Callable[..., object]] = None,
                 sock: Optional[object] = None):
        self._port = port
        self._width = width
        self._height = height
        self._frame_bytes = width * height * 3            # bgr24
        self._bind_ip = bind_ip or local_ip_for(camera_ip)
        self._read_timeout = read_timeout_s
        self._cmd = build_ffmpeg_command(ffmpeg_path, hwaccel)
        self._popen = popen or subprocess.Popen
        self._proc: Optional[object] = None
        self._sock = sock
        self._stop = threading.Event()
        self._frames: "queue.Queue" = queue.Queue(maxsize=_FRAME_QUEUE_MAX)
        self._threads: list = []
        self._opened = False
        self._start()

    # ── lifecycle ────────────────────────────────────────────────────────

    def _start(self) -> None:
        if self._sock is None:
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._sock.setsockopt(socket.SOL_SOCKET,
                                      socket.SO_REUSEADDR, 1)
                self._sock.bind((self._bind_ip, self._port))
                self._sock.settimeout(0.5)
            except OSError as exc:
                _log.error("udp bind %s:%d failed: %s",
                           self._bind_ip, self._port, exc)
                self._sock = None
                return
        try:
            self._proc = self._popen(
                self._cmd, stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except (OSError, ValueError) as exc:
            _log.error("ffmpeg spawn failed (%s): %s", self._cmd[0], exc)
            self._proc = None
            return
        if getattr(self._proc, "stdout", None) is None:
            _log.error("ffmpeg started but stdout pipe is missing")
            return
        self._opened = True
        for target, name in ((self._feed, "gopro-udp-feed"),
                             (self._reader, "gopro-frame-reader"),
                             (self._drain_stderr, "ffmpeg-stderr")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        _log.info("ffmpeg webcam decoder up: udp://%s:%d -> %dx%d",
                  self._bind_ip, self._port, self._width, self._height)

    def _feed(self) -> None:
        """Pump UDP datagrams from the socket into ffmpeg's stdin."""
        stdin = self._proc.stdin                       # type: ignore[union-attr]
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65536)   # type: ignore[union-attr]
            except socket.timeout:
                continue
            except (ConnectionResetError, OSError):
                continue                               # ICMP unreachable etc.
            try:
                stdin.write(data)
                stdin.flush()
            except (OSError, ValueError):
                break                                  # ffmpeg/pipe gone

    def _reader(self) -> None:
        """Read decoded frames from ffmpeg's stdout into the frame queue,
        dropping the oldest when full so consumers always get fresh video."""
        while not self._stop.is_set():
            buf = self._read_exact(self._frame_bytes)
            if buf is None:
                break                                  # ffmpeg ended
            frame = np.frombuffer(buf, dtype=np.uint8).reshape(
                (self._height, self._width, 3)).copy()
            if self._frames.full():
                try:
                    self._frames.get_nowait()
                except queue.Empty:
                    pass
            try:
                self._frames.put_nowait(frame)
            except queue.Full:
                pass

    def _drain_stderr(self) -> None:
        stderr = getattr(self._proc, "stderr", None)
        if stderr is None:
            return
        try:
            for line in iter(stderr.readline, b""):
                _log.debug("ffmpeg: %s",
                           line.decode("utf-8", "replace").rstrip())
        except (OSError, ValueError):
            pass

    def _read_exact(self, n: int) -> Optional[bytes]:
        stdout = self._proc.stdout                     # type: ignore[union-attr]
        chunks = []
        remaining = n
        while remaining > 0:
            try:
                chunk = stdout.read(remaining)
            except (OSError, ValueError):
                return None
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    # ── cv2.VideoCapture-compatible surface ──────────────────────────────

    def isOpened(self) -> bool:                        # noqa: N802 cv2 API
        return (self._opened and self._proc is not None
                and self._proc.poll() is None)

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Return the most recent decoded frame, or (False, None) if none
        arrived within read_timeout_s (feed not up yet, or stalled)."""
        if not self._opened:
            return (False, None)
        try:
            frame = self._frames.get(timeout=self._read_timeout)
        except queue.Empty:
            return (False, None)
        return (True, frame)

    def release(self) -> None:
        """Stop the threads, close the socket, terminate ffmpeg. Idempotent."""
        self._opened = False
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    _log.warning("ffmpeg did not exit after kill()")
        for stream in (getattr(proc, "stdin", None),
                       getattr(proc, "stdout", None),
                       getattr(proc, "stderr", None)):
            if stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def bind_ip(self) -> str:
        return self._bind_ip
