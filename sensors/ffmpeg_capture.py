"""FfmpegStreamCapture — decode a UDP video stream by piping ffmpeg's raw
output through a subprocess.

A cv2.VideoCapture-compatible decoder (``isOpened`` / ``read`` / ``release``)
for the GoPro Hero 13 HEVC preview. It exists because cv2's own FFmpeg backend
is unreliable at decoding the Hero 13's HEVC stream on Windows; piping a
dedicated ffmpeg process gives a deterministic, low-latency frame source.

Because the API matches cv2.VideoCapture, it drops straight into GoProSensor
via the ``capture_factory`` seam — ``GoProSensor(decoder="ffmpeg")`` selects
it, and from there frames flow through SensorManager into the targeting
pipeline like any other sensor.

Reference: HARDWARE_FINDINGS.md §3 (stream format + gotchas) and §7.2
(subprocess fallback).
"""
from __future__ import annotations

import logging
import subprocess
import threading
from typing import Callable, Optional, Tuple

import numpy as np

_log = logging.getLogger(__name__)

# The Hero 13 preview is 1920x1080 HEVC at 29.97 fps (HARDWARE_FINDINGS.md
# §3.1). ffmpeg is asked for bgr24 so frames arrive in OpenCV's native order.
_DEFAULT_URL = "udp://0.0.0.0:8554"   # explicit IPv4 — udp://@ binds IPv6 on Win
_DEFAULT_WIDTH = 1920
_DEFAULT_HEIGHT = 1080


def build_ffmpeg_command(url: str, ffmpeg_path: str,
                         hwaccel: Optional[str]) -> list:
    """The ffmpeg argv that emits raw bgr24 frames on stdout.

    Low-latency flags (nobuffer / low_delay / framedrop) are the ones verified
    in HARDWARE_FINDINGS.md §3.3. ``hwaccel`` (e.g. "cuda" for NVDEC on the
    RTX 4070 Ti) offloads HEVC decode off the CPU — recommended for sustained
    1080p30, left off by default so a missing GPU build does not break things.
    """
    cmd = [ffmpeg_path]
    if hwaccel:
        cmd += ["-hwaccel", hwaccel]
    cmd += [
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-framedrop",
        "-i", url,
        "-an",                        # drop audio + the proprietary metadata
        "-f", "rawvideo",
        "-pix_fmt", "bgr24",
        "-loglevel", "error",
        "-",
    ]
    return cmd


class FfmpegStreamCapture:
    """Subprocess-ffmpeg video decoder with a cv2.VideoCapture-style API.

    The ffmpeg process is spawned on construction. ``read()`` blocks until one
    full frame is available (like cv2.VideoCapture.read), so it is meant to be
    driven from a worker thread — which is exactly how SensorManager runs it.
    """

    def __init__(self,
                 url: str = _DEFAULT_URL,
                 width: int = _DEFAULT_WIDTH,
                 height: int = _DEFAULT_HEIGHT,
                 *,
                 ffmpeg_path: str = "ffmpeg",
                 hwaccel: Optional[str] = None,
                 popen: Optional[Callable[..., object]] = None):
        self._url = url
        self._width = width
        self._height = height
        self._frame_bytes = width * height * 3            # bgr24
        self._cmd = build_ffmpeg_command(url, ffmpeg_path, hwaccel)
        self._popen = popen or subprocess.Popen
        self._proc: Optional[object] = None
        self._stderr_thread: Optional[threading.Thread] = None
        self._opened = False
        self._start()

    # ── lifecycle ────────────────────────────────────────────────────────

    def _start(self) -> None:
        try:
            self._proc = self._popen(
                self._cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except (OSError, ValueError) as exc:
            _log.error("ffmpeg spawn failed (%s): %s", self._cmd[0], exc)
            self._proc = None
            return
        self._opened = getattr(self._proc, "stdout", None) is not None
        if not self._opened:
            _log.error("ffmpeg started but stdout pipe is missing")
            return
        if getattr(self._proc, "stderr", None) is not None:
            self._stderr_thread = threading.Thread(
                target=self._drain_stderr, name="ffmpeg-stderr", daemon=True)
            self._stderr_thread.start()
        _log.info("ffmpeg decoder started for %s (%dx%d)",
                  self._url, self._width, self._height)

    def _drain_stderr(self) -> None:
        # ffmpeg writes decode warnings here; draining stops the pipe filling
        # and deadlocking the process. Mid-GOP HEVC join errors at startup are
        # expected (HARDWARE_FINDINGS.md §3.1), hence debug-level only.
        stderr = getattr(self._proc, "stderr", None)
        if stderr is None:
            return
        try:
            for line in iter(stderr.readline, b""):
                _log.debug("ffmpeg: %s",
                           line.decode("utf-8", "replace").rstrip())
        except (OSError, ValueError):
            pass                                          # pipe closed

    # ── cv2.VideoCapture-compatible surface ──────────────────────────────

    def isOpened(self) -> bool:                           # noqa: N802 cv2 API
        """True while the ffmpeg process is alive and producing frames."""
        return (self._opened and self._proc is not None
                and self._proc.poll() is None)

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read one frame. Returns ``(True, BGR ndarray)`` or, when the stream
        has ended or broken, ``(False, None)``."""
        if self._proc is None or getattr(self._proc, "stdout", None) is None:
            return (False, None)
        buf = self._read_exact(self._frame_bytes)
        if buf is None:
            return (False, None)
        # .copy() so the array owns writable memory — np.frombuffer on the
        # immutable pipe bytes is read-only, which trips up the pipeline.
        frame = np.frombuffer(buf, dtype=np.uint8).reshape(
            (self._height, self._width, 3)).copy()
        return (True, frame)

    def _read_exact(self, n: int) -> Optional[bytes]:
        """Read exactly n bytes from ffmpeg's stdout; None on EOF / short read
        (a partial frame at stream end is discarded, not returned torn)."""
        stdout = self._proc.stdout                        # type: ignore[union-attr]
        chunks = []
        remaining = n
        while remaining > 0:
            try:
                chunk = stdout.read(remaining)
            except (OSError, ValueError):
                return None
            if not chunk:                                 # EOF
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def release(self) -> None:
        """Terminate the ffmpeg process and close its pipes. Idempotent."""
        self._opened = False
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
        for stream in (getattr(proc, "stdout", None),
                       getattr(proc, "stderr", None)):
            if stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass

    # Width/height the decoder was configured for (must match the stream).
    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height
