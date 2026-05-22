"""PhoneFrameDecoder — UDP receive + raw YUV decode."""
from __future__ import annotations

import queue
import socket
import time

import numpy as np

from phone_sensor.frame_decoder import PhoneFrameDecoder, _FrameReassembler
from phone_sensor.protocol import (MAX_CHUNK_PAYLOAD, FramePacket,
                                   pack_frame_packet)


def _chunk_datagrams(payload: bytes, *, frame_id: int, camera_id: str = "main",
                     fmt: str = "nv21", width: int = 0, height: int = 0,
                     chunk_size: int = MAX_CHUNK_PAYLOAD) -> list[bytes]:
    """Split a frame payload into the wire chunk datagrams the phone would
    emit, mirroring StreamPipeline.emit()."""
    total = len(payload)
    count = 1 if total <= chunk_size else (total + chunk_size - 1) // chunk_size
    out = []
    for i in range(count):
        slice_ = payload[i * chunk_size:(i + 1) * chunk_size]
        out.append(pack_frame_packet(FramePacket(
            frame_id=frame_id, capture_ts_us=frame_id * 1000,
            camera_id=camera_id, fmt=fmt, width=width, height=height,
            payload=slice_, chunk_index=i, chunk_count=count)))
    return out


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


def test_fragmented_nv21_frame_reassembled_and_decoded():
    # 256x192 NV21 = 73728 B > MAX_CHUNK_PAYLOAD, so it arrives as 2 chunks.
    w, h = 256, 192
    payload = bytes([128]) * (w * h * 3 // 2)
    datagrams = _chunk_datagrams(payload, frame_id=0, width=w, height=h)
    assert len(datagrams) > 1                     # genuinely fragmented
    dec = PhoneFrameDecoder(width=w, height=h, codec="nv21",
                            sock=_FakeUdpSock(datagrams))
    try:
        frame = _wait_for_frame(dec)
        assert frame is not None
        assert frame.shape == (h, w, 3)
    finally:
        dec.release()


# ── Reassembler unit ─────────────────────────────────────────────────────

def _chunk(frame_id, index, count, payload, *, fmt="h264"):
    return FramePacket(frame_id=frame_id, capture_ts_us=0, camera_id="main",
                       fmt=fmt, width=1, height=1, payload=payload,
                       chunk_index=index, chunk_count=count)


def test_reassembler_passes_single_chunk_through():
    r = _FrameReassembler()
    out = r.add(_chunk(0, 0, 1, b"whole"))
    assert out is not None and out.payload == b"whole"


def test_reassembler_joins_chunks_in_order():
    r = _FrameReassembler()
    assert r.add(_chunk(7, 0, 3, b"aaa")) is None
    assert r.add(_chunk(7, 1, 3, b"bbb")) is None
    out = r.add(_chunk(7, 2, 3, b"ccc"))
    assert out is not None
    assert out.payload == b"aaabbbccc"
    assert (out.chunk_index, out.chunk_count) == (0, 1)   # presented as whole
    assert out.frame_id == 7


def test_reassembler_handles_out_of_order_chunks():
    r = _FrameReassembler()
    assert r.add(_chunk(1, 2, 3, b"C")) is None
    assert r.add(_chunk(1, 0, 3, b"A")) is None
    out = r.add(_chunk(1, 1, 3, b"B"))
    assert out is not None and out.payload == b"ABC"


def test_reassembler_evicts_stale_partial_when_newer_completes():
    r = _FrameReassembler()
    r.add(_chunk(1, 0, 2, b"x"))                   # frame 1 left incomplete
    r.add(_chunk(2, 0, 1, b"y"))                   # frame 2 completes (single)
    # The lingering chunk of frame 1 is gone; a late chunk for it can't finish.
    assert r.add(_chunk(1, 1, 2, b"z")) is None


def test_reassembler_caps_inflight_frames():
    r = _FrameReassembler(max_inflight=2)
    r.add(_chunk(1, 0, 2, b"a"))
    r.add(_chunk(2, 0, 2, b"b"))
    r.add(_chunk(3, 0, 2, b"c"))                   # evicts frame 1 (oldest)
    assert r.add(_chunk(1, 1, 2, b"a2")) is None   # frame 1 was dropped
    out = r.add(_chunk(3, 1, 2, b"c2"))            # frame 3 still alive
    assert out is not None and out.payload == b"cc2"
