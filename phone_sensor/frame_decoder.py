"""PhoneFrameDecoder — receive phone frame UDP packets and decode them.

UDP datagrams arrive carrying our protocol's `FramePacket` (binary header +
payload). The payload is either raw YUV (NV21 or I420 — depends on the phone's
camera output) or an H.264 NAL stream. Either way the public surface is the
same cv2.VideoCapture-style `isOpened` / `read` / `release` API that
`FfmpegStreamCapture` already uses, so PhoneSensor drops it in the same way
GoProSensor uses the GoPro capture.

H.264 decoding mirrors `sensors/ffmpeg_capture.py` exactly: pipe payload bytes
into `ffmpeg -i pipe:0 -f rawvideo -pix_fmt bgr24 -`, read decoded BGR frames
off stdout. Raw YUV is decoded in-process with `cv2.cvtColor`.

Reference: PHONE_SENSOR_BOOTSTRAP.md §2.2, §3.1.
"""
from __future__ import annotations

import logging
import queue
import socket
import subprocess
import threading
import time
from typing import Any, Callable, Optional, Tuple

import cv2
import numpy as np

from config import settings
from phone_sensor.protocol import FramePacket, parse_frame_packet

_log = logging.getLogger(__name__)

# Two YUV layouts Android CameraX commonly produces.
_RAW_FORMATS = {
    "nv21": cv2.COLOR_YUV2BGR_NV21,
    "i420": cv2.COLOR_YUV2BGR_I420,
    "yuv420": cv2.COLOR_YUV2BGR_I420,
}
# Codec identifiers we recognise on the wire.
_CODEC_FORMATS = {"h264", "avc", "h265", "hevc"}


def _build_ffmpeg_command(ffmpeg_path: str, codec: str,
                          hwaccel: Optional[str]) -> list:
    """Mirror the GoPro ffmpeg decode flags — low-latency, no buffering."""
    cmd = [ffmpeg_path]
    if hwaccel:
        cmd += ["-hwaccel", hwaccel]
    cmd += [
        "-probesize", "100000",
        "-analyzeduration", "0",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-avioflags", "direct",
        "-max_delay", "0",
        "-f", codec,
        "-i", "pipe:0",
        "-an",
        "-f", "rawvideo",
        "-pix_fmt", "bgr24",
        "-flush_packets", "1",
        "-loglevel", "error",
        "-",
    ]
    return cmd


class PhoneFrameDecoder:
    """UDP frame receiver with a cv2.VideoCapture-style API.

    Construction binds the UDP socket, picks a decode path from `codec`
    (raw_yuv vs h264), and spawns the worker threads. `read()` returns the
    latest decoded BGR frame, or (False, None) on `read_timeout_s` silence.
    """

    def __init__(self,
                 width: int,
                 height: int,
                 *,
                 codec: str = "h264",
                 bind_ip: str = settings.PHONE_BIND_IP,
                 port: int = settings.PHONE_FRAME_PORT,
                 read_timeout_s: float = 2.0,
                 ffmpeg_path: str = "ffmpeg",
                 hwaccel: Optional[str] = settings.PHONE_FFMPEG_HWACCEL,
                 record_path: Optional[str] = None,
                 popen: Optional[Callable[..., Any]] = None,
                 sock: Optional[Any] = None):
        self._width = width
        self._height = height
        self._codec = codec.lower()
        self._bind_ip = bind_ip
        self._port = port
        self._read_timeout = read_timeout_s
        self._ffmpeg_path = ffmpeg_path
        self._hwaccel = hwaccel
        self._record_path = record_path
        self._record_fh: Optional[Any] = None
        self._popen = popen or subprocess.Popen
        self._sock = sock
        self._proc: Optional[Any] = None
        self._stop = threading.Event()
        self._frames: "queue.Queue" = queue.Queue(
            maxsize=settings.PHONE_FRAME_QUEUE_MAX)
        self._threads: list = []
        self._opened = False
        self._latest_meta: Optional[FramePacket] = None
        self._meta_lock = threading.Lock()
        self._start()

    # ── Lifecycle ────────────────────────────────────────────────────────

    def _start(self) -> None:
        if self._sock is None:
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._sock.bind((self._bind_ip, self._port))
                self._sock.settimeout(0.5)
            except OSError as exc:
                _log.error("phone udp bind %s:%d failed: %s",
                           self._bind_ip, self._port, exc)
                self._sock = None
                return
        if self._record_path:
            try:
                self._record_fh = open(self._record_path, "wb")
                _log.info("recording phone stream -> %s", self._record_path)
            except OSError as exc:
                _log.error("could not open phone record file %s: %s",
                           self._record_path, exc)

        # Pick the decode path. Raw YUV is decoded in-process by the reader
        # thread itself; H.264 spawns ffmpeg and runs a stdout reader.
        if self._codec in _CODEC_FORMATS:
            self._proc = self._spawn_ffmpeg(self._hwaccel)
            if self._proc is not None and self._hwaccel:
                time.sleep(0.3)
                if self._proc.poll() is not None:
                    _log.warning("phone ffmpeg hwaccel=%s failed; software "
                                 "decode fallback", self._hwaccel)
                    self._proc = self._spawn_ffmpeg(None)
            if self._proc is None or getattr(self._proc, "stdout", None) is None:
                _log.error("phone ffmpeg did not start")
                return
            self._opened = True
            for target, name in (
                    (self._h264_feed, "phone-udp-feed"),
                    (self._h264_reader, "phone-frame-reader"),
                    (self._drain_stderr, "phone-ffmpeg-stderr")):
                t = threading.Thread(target=target, name=name, daemon=True)
                t.start()
                self._threads.append(t)
        elif self._codec in _RAW_FORMATS:
            self._opened = True
            t = threading.Thread(target=self._raw_yuv_loop,
                                 name="phone-yuv-reader", daemon=True)
            t.start()
            self._threads.append(t)
        else:
            _log.error("phone: unsupported codec %r", self._codec)
            return
        _log.info("phone decoder up: %s @ %dx%d on udp://%s:%d",
                  self._codec, self._width, self._height,
                  self._bind_ip, self._port)

    def _spawn_ffmpeg(self, hwaccel: Optional[str]) -> Optional[object]:
        ff_codec = "hevc" if self._codec in ("h265", "hevc") else "h264"
        cmd = _build_ffmpeg_command(self._ffmpeg_path, ff_codec, hwaccel)
        try:
            return self._popen(cmd, stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
        except (OSError, ValueError) as exc:
            _log.error("phone ffmpeg spawn failed (%s): %s", cmd[0], exc)
            return None

    # ── H.264 path — pipe payloads to ffmpeg, decode off stdout ──────────

    def _h264_feed(self) -> None:
        stdin = self._proc.stdin                       # type: ignore[union-attr]
        while not self._stop.is_set():
            packet = self._recv_packet()
            if packet is None:
                continue
            if self._record_fh is not None:
                try:
                    self._record_fh.write(packet.payload)
                except OSError:
                    pass
            try:
                stdin.write(packet.payload)
                stdin.flush()
            except (OSError, ValueError):
                break

    def _h264_reader(self) -> None:
        frame_bytes = self._width * self._height * 3
        while not self._stop.is_set():
            buf = self._read_exact(frame_bytes)
            if buf is None:
                break
            frame = np.frombuffer(buf, dtype=np.uint8).reshape(
                (self._height, self._width, 3)).copy()
            self._publish(frame)

    def _drain_stderr(self) -> None:
        stderr = getattr(self._proc, "stderr", None)
        if stderr is None:
            return
        try:
            for line in iter(stderr.readline, b""):
                _log.debug("phone ffmpeg: %s",
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

    # ── Raw YUV path — decode in-process ─────────────────────────────────

    def _raw_yuv_loop(self) -> None:
        while not self._stop.is_set():
            packet = self._recv_packet()
            if packet is None:
                continue
            conv = _RAW_FORMATS.get(packet.fmt.lower())
            if conv is None:
                # Format renegotiated on the phone — drop the packet rather
                # than misinterpret bytes as the wrong layout.
                continue
            try:
                yuv = np.frombuffer(packet.payload, dtype=np.uint8).reshape(
                    (packet.height * 3 // 2, packet.width))
            except ValueError:
                continue                        # truncated payload — skip
            frame = cv2.cvtColor(yuv, conv)
            self._publish(frame)

    # ── Shared helpers ───────────────────────────────────────────────────

    def _recv_packet(self) -> Optional[FramePacket]:
        try:
            data, _ = self._sock.recvfrom(2_000_000)   # type: ignore[union-attr]
        except socket.timeout:
            return None
        except (ConnectionResetError, OSError):
            return None
        try:
            packet = parse_frame_packet(data)
        except ValueError:
            return None
        with self._meta_lock:
            self._latest_meta = packet
        return packet

    def _publish(self, frame: np.ndarray) -> None:
        if self._frames.full():
            try:
                self._frames.get_nowait()
            except queue.Empty:
                pass
        try:
            self._frames.put_nowait(frame)
        except queue.Full:
            pass

    # ── cv2.VideoCapture-compatible surface ──────────────────────────────

    def isOpened(self) -> bool:                        # noqa: N802 cv2 API
        if not self._opened:
            return False
        if self._proc is not None and self._proc.poll() is not None:
            return False
        return True

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Latest decoded BGR frame, or (False, None) on silence."""
        if not self._opened:
            return (False, None)
        try:
            frame = self._frames.get(timeout=self._read_timeout)
        except queue.Empty:
            return (False, None)
        return (True, frame)

    def release(self) -> None:
        """Stop threads, close socket, terminate ffmpeg. Idempotent."""
        self._opened = False
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._record_fh is not None:
            try:
                self._record_fh.close()
            except OSError:
                pass
            self._record_fh = None
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
                    _log.warning("phone ffmpeg did not exit after kill()")
        for stream in (getattr(proc, "stdin", None),
                       getattr(proc, "stdout", None),
                       getattr(proc, "stderr", None)):
            if stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass

    # ── Introspection ────────────────────────────────────────────────────

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def latest_camera_id(self) -> Optional[str]:
        with self._meta_lock:
            return self._latest_meta.camera_id if self._latest_meta else None
