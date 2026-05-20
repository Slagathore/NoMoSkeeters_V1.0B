"""PhoneFrameDecoder — UDP receive + raw YUV decode."""
from __future__ import annotations

import queue
import socket
import time

import numpy as np

from phone_sensor.frame_decoder import PhoneFrameDecoder
from phone_sensor.protocol import FramePacket, pack_frame_packet


class _FakeUdpSock:
    """Duck-typed socket: scripted recvfrom() returns plus a close()."""

    def __init__(self, packets: list[bytes]):
        self._packets = list(packets)
        self.closed = False

    def recvfrom(self, _n):
        if not self._packets:
            raise socket.timeout
        return (self._packets.pop(0), ("127.0.0.1", 0))

    def close(self):
        self.closed = True


def _nv21_packet(width: int, height: int, *, frame_id: int = 0,
                 camera_id: str = "main", fill: int = 128) -> bytes:
    payload = bytes([fill]) * (width * height * 3 // 2)
    pkt = FramePacket(frame_id=frame_id, capture_ts_us=frame_id * 1000,
                      camera_id=camera_id, fmt="nv21",
                      width=width, height=height, payload=payload)
    return pack_frame_packet(pkt)


def _wait_for_frame(dec: PhoneFrameDecoder, timeout_s: float = 1.5):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            return dec._frames.get(timeout=0.05)
        except queue.Empty:
            continue
    return None


# ── Raw YUV path ─────────────────────────────────────────────────────────

def test_decodes_nv21_to_bgr():
    w, h = 32, 16
    sock = _FakeUdpSock([_nv21_packet(w, h, camera_id="main")])
    dec = PhoneFrameDecoder(width=w, height=h, codec="nv21", sock=sock)
    try:
        assert dec.isOpened()
        frame = _wait_for_frame(dec)
        assert frame is not None
        assert frame.shape == (h, w, 3)
        assert frame.dtype == np.uint8
        assert dec.latest_camera_id == "main"
    finally:
        dec.release()


def test_camera_id_tracks_per_packet():
    w, h = 16, 16
    sock = _FakeUdpSock([_nv21_packet(w, h, camera_id="main", frame_id=0),
                         _nv21_packet(w, h, camera_id="telephoto",
                                      frame_id=1)])
    dec = PhoneFrameDecoder(width=w, height=h, codec="nv21", sock=sock)
    try:
        # Drain both frames so the second one's metadata is in latest_meta.
        _wait_for_frame(dec)
        _wait_for_frame(dec)
        # Reader may still be running; give it a beat.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and dec.latest_camera_id != "telephoto":
            time.sleep(0.01)
        assert dec.latest_camera_id == "telephoto"
    finally:
        dec.release()


def test_bad_magic_packet_dropped_without_crashing():
    w, h = 16, 16
    valid = _nv21_packet(w, h)
    bad = b"XXXX" + valid[4:]                    # corrupt header
    sock = _FakeUdpSock([bad, valid])
    dec = PhoneFrameDecoder(width=w, height=h, codec="nv21", sock=sock)
    try:
        frame = _wait_for_frame(dec)
        assert frame is not None                  # bad packet skipped, valid one decoded
    finally:
        dec.release()


def test_unknown_yuv_format_skipped():
    w, h = 16, 16
    pkt = FramePacket(0, 0, "main", "fancy_yuv_format", w, h,
                      b"\x00" * (w * h * 3 // 2))
    sock = _FakeUdpSock([pack_frame_packet(pkt)])
    dec = PhoneFrameDecoder(width=w, height=h, codec="nv21", sock=sock)
    try:
        # No frame should arrive — unknown fmt is dropped.
        assert _wait_for_frame(dec, timeout_s=0.3) is None
    finally:
        dec.release()


def test_read_timeout_returns_false():
    dec = PhoneFrameDecoder(width=16, height=16, codec="nv21",
                            sock=_FakeUdpSock([]), read_timeout_s=0.05)
    try:
        ok, frame = dec.read()
        assert ok is False and frame is None
    finally:
        dec.release()


def test_release_is_idempotent():
    sock = _FakeUdpSock([])
    dec = PhoneFrameDecoder(width=16, height=16, codec="nv21", sock=sock)
    dec.release()
    dec.release()                                # no-op the second time
    assert dec.isOpened() is False


def test_unsupported_codec_fails_to_open():
    dec = PhoneFrameDecoder(width=16, height=16, codec="exotic",
                            sock=_FakeUdpSock([]))
    try:
        assert dec.isOpened() is False
    finally:
        dec.release()
