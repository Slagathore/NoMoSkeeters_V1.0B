"""Unit tests for FfmpegStreamCapture — the subprocess-ffmpeg GoPro decoder.

The ffmpeg process is replaced with a fake via the injectable `popen` hook,
so these run with no ffmpeg binary and no network.
"""
from __future__ import annotations

import io

import numpy as np

from sensors.ffmpeg_capture import FfmpegStreamCapture, build_ffmpeg_command

# Tiny frames keep the test fast: 4x2 bgr24 = 24 bytes per frame.
W, H = 4, 2
FRAME_BYTES = W * H * 3


def _frame(fill: int) -> bytes:
    return bytes([fill]) * FRAME_BYTES


class FakeProc:
    """Stand-in for subprocess.Popen — stdout replays the given bytes."""

    def __init__(self, stdout_data: bytes):
        self.stdout = io.BytesIO(stdout_data)
        self.stderr = io.BytesIO(b"")
        self._alive = True
        self.terminated = False
        self.killed = False

    def poll(self):
        return None if self._alive else 0

    def terminate(self):
        self.terminated = True
        self._alive = False

    def wait(self, timeout=None):
        return 0

    def kill(self):
        self.killed = True
        self._alive = False


def _capture(stdout_data: bytes) -> FfmpegStreamCapture:
    return FfmpegStreamCapture(
        "udp://0.0.0.0:8554", width=W, height=H,
        popen=lambda *a, **k: FakeProc(stdout_data))


# ── ffmpeg command ───────────────────────────────────────────────────────

def test_command_has_ipv4_url_and_low_latency_flags():
    cmd = build_ffmpeg_command("udp://0.0.0.0:8554", "ffmpeg", None)
    assert "udp://0.0.0.0:8554" in cmd
    assert "nobuffer" in cmd and "low_delay" in cmd
    assert "bgr24" in cmd
    assert cmd[-1] == "-"                       # raw video piped to stdout


def test_command_inserts_hwaccel_before_input():
    cmd = build_ffmpeg_command("udp://0.0.0.0:8554", "ffmpeg", "cuda")
    assert cmd[cmd.index("-hwaccel") + 1] == "cuda"
    assert cmd.index("-hwaccel") < cmd.index("-i")


# ── frame decoding ───────────────────────────────────────────────────────

def test_read_returns_decoded_frames_then_eof():
    cap = _capture(_frame(10) + _frame(20))
    assert cap.isOpened() is True

    ok, frame = cap.read()
    assert ok is True and frame is not None
    assert frame.shape == (H, W, 3)
    assert frame.dtype == np.uint8
    assert int(frame[0, 0, 0]) == 10

    ok, frame = cap.read()
    assert ok is True and int(frame[0, 0, 0]) == 20

    ok, frame = cap.read()                      # stdout exhausted
    assert ok is False and frame is None


def test_decoded_frame_is_writable():
    cap = _capture(_frame(5))
    ok, frame = cap.read()
    assert ok is True
    frame[0, 0, 0] = 99                         # read-only buffer would raise
    assert int(frame[0, 0, 0]) == 99


def test_torn_partial_frame_is_treated_as_eof():
    cap = _capture(_frame(7) + b"\x00\x00\x00")  # one frame + a torn tail
    ok, _ = cap.read()
    assert ok is True
    ok, frame = cap.read()                       # partial frame, not returned
    assert ok is False and frame is None


# ── lifecycle ────────────────────────────────────────────────────────────

def test_release_terminates_process_and_is_idempotent():
    box = {}

    def _popen(*a, **k):
        box["proc"] = FakeProc(_frame(1))
        return box["proc"]

    cap = FfmpegStreamCapture("udp://0.0.0.0:8554", width=W, height=H,
                              popen=_popen)
    cap.release()
    assert box["proc"].terminated is True
    assert cap.isOpened() is False
    cap.release()                                # no raise on second call


def test_spawn_failure_reports_not_opened():
    def _boom(*a, **k):
        raise OSError("ffmpeg not found")

    cap = FfmpegStreamCapture("udp://0.0.0.0:8554", width=W, height=H,
                              popen=_boom)
    assert cap.isOpened() is False
    ok, frame = cap.read()
    assert ok is False and frame is None
