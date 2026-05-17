"""Step 5 offline tests: LaserCubeInterface socket logic without a cube.

The live-cube acceptance ("connect() returns True against your cube") is run
by hand. These tests cover the parts that DON'T need hardware — the §9.5
strict reply filter, source-IP auto-detection, msg_num sequencing, and the
send-side buffer estimate — by injecting a fake socket.
"""
from __future__ import annotations

import socket

import pytest

from laser.lasercube import LaserCubeInterface, _autodetect_src_ip
from laser.protocol import LC_CMD_GET_FULL_INFO, synth_full_info
from laser.types import LaserPoint


CUBE_IP = "10.0.0.7"
CUBE_PORT = 45457


class FakeSock:
    """Duck-typed UDP socket. recvfrom() replays a scripted list of
    (data, addr) tuples; a bare `socket.timeout` entry raises instead."""

    def __init__(self, replies=None):
        self._replies = list(replies or [])
        self.sent: list[tuple[bytes, tuple]] = []
        self.closed = False

    def settimeout(self, _t):           # noqa: D102
        pass

    def sendto(self, data, addr):       # noqa: D102
        self.sent.append((bytes(data), addr))

    def recvfrom(self, _n):             # noqa: D102
        if not self._replies:
            raise socket.timeout()
        item = self._replies.pop(0)
        if item is socket.timeout:
            raise socket.timeout()
        return item

    def close(self):                    # noqa: D102
        self.closed = True


def _iface(reply_timeout_s: float = 2.0) -> LaserCubeInterface:
    return LaserCubeInterface(ip=CUBE_IP, src_ip="0.0.0.0",
                              reply_timeout_s=reply_timeout_s)


# ── §9.5 strict reply filter ─────────────────────────────────────────────

def test_send_cmd_recv_accepts_clean_reply():
    iface = _iface()
    good = synth_full_info()
    iface._cmd_sock = FakeSock([(good, (CUBE_IP, CUBE_PORT))])
    data = iface._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO]),
                                expect_min_bytes=38,
                                expect_echo_byte=LC_CMD_GET_FULL_INFO)
    assert data == good


def test_send_cmd_recv_drops_wrong_source_ip():
    iface = _iface()
    good = synth_full_info()
    iface._cmd_sock = FakeSock([
        (good, ("10.0.0.99", CUBE_PORT)),     # impostor IP — dropped
        (good, (CUBE_IP, CUBE_PORT)),         # real cube — accepted
    ])
    data = iface._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO]),
                                expect_echo_byte=LC_CMD_GET_FULL_INFO)
    assert data == good


def test_send_cmd_recv_drops_wrong_source_port():
    iface = _iface()
    good = synth_full_info()
    iface._cmd_sock = FakeSock([
        (good, (CUBE_IP, 50000)),             # wrong port — dropped
        (good, (CUBE_IP, CUBE_PORT)),
    ])
    assert iface._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO])) == good


def test_send_cmd_recv_drops_wrong_echo_byte():
    iface = _iface()
    good = synth_full_info()
    bad = bytes([0x55]) + good[1:]            # wrong command echo
    iface._cmd_sock = FakeSock([
        (bad, (CUBE_IP, CUBE_PORT)),
        (good, (CUBE_IP, CUBE_PORT)),
    ])
    assert iface._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO])) == good


def test_send_cmd_recv_rejects_nonzero_status():
    iface = _iface()
    bad = bytearray(synth_full_info())
    bad[1] = 0x01                             # cube-reported failure
    iface._cmd_sock = FakeSock([(bytes(bad), (CUBE_IP, CUBE_PORT))])
    assert iface._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO])) is None


def test_send_cmd_recv_drops_short_reply():
    iface = _iface()
    good = synth_full_info()
    iface._cmd_sock = FakeSock([
        (b"\x77\x00\x00", (CUBE_IP, CUBE_PORT)),   # 3B < expect_min_bytes
        (good, (CUBE_IP, CUBE_PORT)),
    ])
    data = iface._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO]),
                                expect_min_bytes=38)
    assert data == good


def test_send_cmd_recv_returns_none_on_timeout():
    iface = _iface(reply_timeout_s=0.2)
    iface._cmd_sock = FakeSock([])            # nothing to receive
    assert iface._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO])) is None


def test_send_cmd_recv_returns_none_without_socket():
    iface = _iface()
    assert iface._cmd_sock is None
    assert iface._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO])) is None


# ── §13.5 source-IP auto-detection ───────────────────────────────────────

def test_autodetect_picks_matching_16_prefix(monkeypatch):
    monkeypatch.setattr(socket, "gethostname", lambda: "host")
    monkeypatch.setattr(socket, "getaddrinfo",
                        lambda *a, **k: [(0, 0, 0, "", ("192.168.1.50", 0))])
    monkeypatch.setattr(socket, "gethostbyname_ex",
                        lambda h: ("host", [], ["192.168.1.50", "169.254.99.7"]))
    assert _autodetect_src_ip("169.254.40.83") == "169.254.99.7"


def test_autodetect_falls_back_to_any(monkeypatch):
    monkeypatch.setattr(socket, "gethostname", lambda: "host")
    monkeypatch.setattr(socket, "getaddrinfo", lambda *a, **k: [])
    monkeypatch.setattr(socket, "gethostbyname_ex",
                        lambda h: ("host", [], ["192.168.1.50"]))
    assert _autodetect_src_ip("169.254.40.83") == "0.0.0.0"


# ── msg_num sequencing + send-side buffer estimate ───────────────────────

def test_next_msg_num_wraps_at_256():
    iface = _iface()
    iface._msg_num = 254
    assert [iface._next_msg_num() for _ in range(4)] == [254, 255, 0, 1]


def test_send_chunk_decrements_buffer_estimate():
    iface = _iface()
    iface._data_sock = FakeSock()
    iface._buffer.free = 6000
    pts = [LaserPoint(x=2048, y=2048) for _ in range(140)]
    assert iface.send_chunk(pts, msg_num=0, frame_num=0) is True
    assert iface._buffer.free == 6000 - 140
    assert len(iface._data_sock.sent) == 1


def test_send_frame_advances_msg_num_per_packet():
    iface = _iface()
    iface._data_sock = FakeSock()
    iface._buffer.free = 6000
    pts = [LaserPoint() for _ in range(300)]   # 3 packets @ 140 max
    assert iface.send_frame(pts, frame_num=5) is True
    msg_nums = [pkt[0][2] for pkt in iface._data_sock.sent]
    assert msg_nums == [0, 1, 2]


def test_send_chunk_without_socket_returns_false():
    iface = _iface()
    assert iface.send_chunk([LaserPoint()], 0, 0) is False
