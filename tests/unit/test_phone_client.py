"""PhoneSensorClient — TCP command channel with a fake in-process phone."""
from __future__ import annotations

import json
import socket
import threading
import time

import pytest

from phone_sensor.client import PhoneSensorClient
from phone_sensor.protocol import PROTOCOL_VERSION


class _FakePhone(threading.Thread):
    """Loopback `phone` for client tests.

    Owns one end of a socket pair, reads JSON lines, echoes a reply per
    command, and lets the test inject unsolicited events at any time.
    """

    def __init__(self, sock: socket.socket):
        super().__init__(daemon=True)
        self.sock = sock
        self.received: list = []
        self._stop = threading.Event()
        self._send_lock = threading.Lock()

    def stop(self) -> None:
        self._stop.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass

    def push_event(self, msg: dict) -> None:
        with self._send_lock:
            try:
                self.sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))
            except OSError:
                pass

    def run(self) -> None:
        buf = b""
        self.sock.settimeout(0.3)
        while not self._stop.is_set():
            try:
                chunk = self.sock.recv(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line:
                    continue
                msg = json.loads(line)
                self.received.append(msg)
                self._reply(msg)

    def _reply(self, msg: dict) -> None:
        cmd = msg["type"]
        cid = msg.get("cmd_id")
        reply = {"type": f"{cmd}_reply", "cmd_id": cid, "ok": True}
        if cmd == "connect":
            reply["capabilities"] = {
                "phone_model": "TestPhone",
                "protocol_version": PROTOCOL_VERSION,
                "cameras": [
                    {"id": "main", "fov_h_deg": 75, "supports_af": True},
                    {"id": "telephoto", "has_optical_zoom": True,
                     "optical_zoom_factor": 3.0},
                ],
            }
        elif cmd == "stream_start":
            reply["mode"] = msg.get("mode", "raw_yuv")
        with self._send_lock:
            try:
                self.sock.sendall((json.dumps(reply) + "\n").encode("utf-8"))
            except OSError:
                pass


def _paired_client() -> tuple[PhoneSensorClient, _FakePhone]:
    pc_sock, phone_sock = socket.socketpair()
    fake = _FakePhone(phone_sock)
    fake.start()

    def fake_connect(host, port, timeout_s):
        pc_sock.settimeout(None)
        return pc_sock

    client = PhoneSensorClient(connect_fn=fake_connect,
                               heartbeat_interval_s=10.0,  # tests poke manually
                               heartbeat_timeout_s=30.0)
    return client, fake


# ── Lifecycle ────────────────────────────────────────────────────────────

def test_connect_handshake_populates_capabilities():
    client, fake = _paired_client()
    try:
        assert client.connect() is True
        caps = client.capabilities
        assert caps is not None
        assert caps.phone_model == "TestPhone"
        tele = caps.camera("telephoto")
        assert tele is not None and tele.optical_zoom_factor == 3.0
        assert client.is_connected is True
        assert any(m["type"] == "connect" for m in fake.received)
    finally:
        client.disconnect()
        fake.stop()


def test_send_command_round_trip_and_id_matching():
    client, fake = _paired_client()
    try:
        assert client.connect()
        reply = client.send_command("get_status")
        assert reply is not None
        assert reply["type"] == "get_status_reply"
        # cmd_ids must match — the client routes replies by cmd_id.
        assert reply["cmd_id"] == 2          # 1 = handshake, 2 = this
    finally:
        client.disconnect()
        fake.stop()


def test_set_active_camera_wraps_send_command():
    client, fake = _paired_client()
    try:
        assert client.connect()
        assert client.set_active_camera("telephoto") is True
        seen = [m for m in fake.received if m["type"] == "set_active_camera"]
        assert seen and seen[-1]["camera_id"] == "telephoto"
    finally:
        client.disconnect()
        fake.stop()


def test_stream_start_returns_negotiated_mode():
    client, fake = _paired_client()
    try:
        assert client.connect()
        mode = client.stream_start(mode="h264_lowlat",
                                   width=1280, height=720, fps=30)
        assert mode == "h264_lowlat"
    finally:
        client.disconnect()
        fake.stop()


def test_stream_start_rejects_unknown_mode():
    client, fake = _paired_client()
    try:
        client.connect()
        with pytest.raises(ValueError):
            client.stream_start(mode="lasers_and_unicorns")
    finally:
        client.disconnect()
        fake.stop()


def test_unsolicited_events_dispatch_to_sink():
    client, fake = _paired_client()
    received: list = []
    client.set_event_sink(received.append)
    try:
        assert client.connect()
        fake.push_event({"type": "event:thermal_warning", "state": "light"})
        # Wait for the reader thread to dispatch.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not received:
            time.sleep(0.01)
        assert received and received[0]["type"] == "event:thermal_warning"
    finally:
        client.disconnect()
        fake.stop()


def test_disconnect_is_idempotent():
    client, fake = _paired_client()
    try:
        client.connect()
        client.disconnect()
        client.disconnect()                  # second call is a no-op
        assert client.is_connected is False
    finally:
        fake.stop()


def test_send_command_fails_when_not_connected():
    client = PhoneSensorClient()             # never connected
    assert client.send_command("ping") is None
