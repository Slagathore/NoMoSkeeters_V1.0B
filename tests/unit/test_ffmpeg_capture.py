"""Unit tests for FfmpegStreamCapture — the GoPro webcam UDP decoder.

Both the UDP socket and the ffmpeg process are replaced with fakes via the
injectable `sock` / `popen` hooks, so these run with no ffmpeg and no network.
"""
from __future__ import annotations

import io
import socket as _socket

import numpy as np

from sensors.ffmpeg_capture import (FfmpegStreamCapture, build_ffmpeg_command,
                                    local_ip_for)

# Tiny frames keep the test fast: 4x2 bgr24 = 24 bytes per frame.
W, H = 4, 2
FRAME_BYTES = W * H * 3


def _frame(fill: int) -> bytes:
    return bytes([fill]) * FRAME_BYTES


class FakeSock:
    """Fake UDP socket: yields the given datagrams, then times out forever."""

    def __init__(self, datagrams=()):
        self._dg = list(datagrams)
        self.closed = False

    def recvfrom(self, n):
        if self._dg:
            return (self._dg.pop(0), ("172.27.109.51", 9000))
        raise _socket.timeout()

    def close(self):
        self.closed = True


class FakeProc:
    """Stand-in for subprocess.Popen — stdout replays the given frame bytes."""

    def __init__(self, stdout_data: bytes):
        self.stdin = io.BytesIO()
        self.stdout = io.BytesIO(stdout_data)
        self.stderr = io.BytesIO(b"")
        self._alive = True
        self.terminated = False

    def poll(self):
        return None if self._alive else 0

    def terminate(self):
        self.terminated = True
        self._alive = False

    def wait(self, timeout=None):
        return 0

    def kill(self):
        self._alive = False


def _capture(stdout_data: bytes, datagrams=(b"ts-bytes",)):
    return FfmpegStreamCapture(
        camera_ip="172.27.109.51", width=W, height=H,
        bind_ip="172.27.109.55", read_timeout_s=1.0,
        popen=lambda *a, **k: FakeProc(stdout_data),
        sock=FakeSock(datagrams))


# ── ffmpeg command ───────────────────────────────────────────────────────

def test_command_pipes_mpegts_from_stdin():
    cmd = build_ffmpeg_command("ffmpeg", None)
    assert "mpegts" in cmd
    assert "pipe:0" in cmd
    assert "bgr24" in cmd
    assert cmd[-1] == "-"                       # raw video piped to stdout


def test_command_inserts_hwaccel_before_input():
    cmd = build_ffmpeg_command("ffmpeg", "cuda")
    assert cmd[cmd.index("-hwaccel") + 1] == "cuda"
    assert cmd.index("-hwaccel") < cmd.index("-i")


def test_local_ip_for_returns_an_ipv4():
    ip = local_ip_for("203.0.113.51")           # TEST-NET-3, no local match
    assert isinstance(ip, str) and ip.count(".") == 3


# ── frame decoding ───────────────────────────────────────────────────────

def test_read_returns_decoded_frames():
    cap = _capture(_frame(10) + _frame(20))
    assert cap.isOpened() is True

    ok, frame = cap.read()
    assert ok is True and frame is not None
    assert frame.shape == (H, W, 3)
    assert frame.dtype == np.uint8
    assert int(frame[0, 0, 0]) == 10

    ok, frame = cap.read()
    assert ok is True and int(frame[0, 0, 0]) == 20
    cap.release()


def test_decoded_frame_is_writable():
    cap = _capture(_frame(5))
    ok, frame = cap.read()
    assert ok is True
    frame[0, 0, 0] = 99                         # read-only buffer would raise
    assert int(frame[0, 0, 0]) == 99
    cap.release()


def test_read_times_out_when_no_frames():
    cap = _capture(b"")                         # ffmpeg yields nothing
    ok, frame = cap.read()                      # waits read_timeout_s, gives up
    assert ok is False and frame is None
    cap.release()


# ── lifecycle ────────────────────────────────────────────────────────────

def test_release_terminates_process_and_is_idempotent():
    box = {}

    def _popen(*a, **k):
        box["proc"] = FakeProc(_frame(1))
        return box["proc"]

    sock = FakeSock()
    cap = FfmpegStreamCapture("172.27.109.51", width=W, height=H,
                              bind_ip="172.27.109.55", popen=_popen, sock=sock)
    cap.release()
    assert box["proc"].terminated is True
    assert sock.closed is True
    assert cap.isOpened() is False
    cap.release()                               # no raise on second call


def test_spawn_failure_reports_not_opened():
    def _boom(*a, **k):
        raise OSError("ffmpeg not found")

    cap = FfmpegStreamCapture("172.27.109.51", width=W, height=H,
                              bind_ip="172.27.109.55", popen=_boom,
                              sock=FakeSock())
    assert cap.isOpened() is False
    ok, frame = cap.read()
    assert ok is False and frame is None
