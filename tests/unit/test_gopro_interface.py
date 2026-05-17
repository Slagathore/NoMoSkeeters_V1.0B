"""Step 11: GoProInterface — Open GoPro HTTP control, offline."""
from __future__ import annotations

import time

import numpy as np

import sensors.gopro as gopro_mod
from events.schemas import GoProStatusEvent
from sensors.gopro import GoProControl, GoProSensor
from sensors.gopro_interface import _KEEPALIVE_PACKET, GoProInterface


class FakeUDP:
    def __init__(self):
        self.sent: list[tuple[bytes, tuple]] = []
        self.closed = False

    def sendto(self, data, addr):
        self.sent.append((bytes(data), addr))

    def close(self):
        self.closed = True


def _iface(http_log=None, sock=None) -> GoProInterface:
    http_log = http_log if http_log is not None else []
    return GoProInterface(
        http_get=lambda url, t: http_log.append(url) or True,
        socket_factory=lambda: sock or FakeUDP(),
    )


# ── URL + packet shape ───────────────────────────────────────────────────

def test_keepalive_packet_is_documented_gphd():
    # "_GPHD_:%u:%u:%d:%1lf\n" % (0, 0, 2, 0) — command 2 = keep alive.
    assert _KEEPALIVE_PACKET.startswith(b"_GPHD_:0:0:2:")
    assert _KEEPALIVE_PACKET.endswith(b"\n")


def test_url_targets_open_gopro_http_api():
    url = _iface()._url("/gopro/camera/stream/start")
    assert url == "http://10.5.5.9:8080/gopro/camera/stream/start"


def test_satisfies_gopro_control_protocol():
    assert isinstance(_iface(), GoProControl)


# ── start / stop ─────────────────────────────────────────────────────────

def test_start_preview_hits_start_endpoint():
    log: list[str] = []
    assert _iface(http_log=log).start_preview() is True
    assert log == ["http://10.5.5.9:8080/gopro/camera/stream/start"]


def test_stop_preview_hits_stop_endpoint_and_closes_socket():
    log: list[str] = []
    sock = FakeUDP()
    iface = _iface(http_log=log, sock=sock)
    iface.keep_alive()                       # lazily opens the UDP socket
    assert iface.stop_preview() is True
    assert log[-1] == "http://10.5.5.9:8080/gopro/camera/stream/stop"
    assert sock.closed is True


def test_start_preview_false_on_http_failure():
    iface = GoProInterface(http_get=lambda url, t: False)
    assert iface.start_preview() is False


# ── keep-alive ───────────────────────────────────────────────────────────

def test_keep_alive_sends_gphd_packet_to_stream_port():
    sock = FakeUDP()
    iface = _iface(sock=sock)
    assert iface.keep_alive() is True
    data, addr = sock.sent[0]
    assert data == _KEEPALIVE_PACKET
    assert addr == ("10.5.5.9", 8554)


# ── Integration: GoProSensor driven by a real GoProInterface ─────────────

class FakeCapture:
    def __init__(self):
        self._opened = True

    def isOpened(self):
        return self._opened

    def read(self):
        return (True, np.zeros((240, 320, 3), dtype=np.uint8))

    def release(self):
        self._opened = False


def test_gopro_sensor_uses_interface_for_lifecycle(monkeypatch):
    monkeypatch.setattr(gopro_mod, "_KEEPALIVE_INTERVAL_S", 0.01)
    http_log: list[str] = []
    sock = FakeUDP()
    iface = _iface(http_log=http_log, sock=sock)
    cam = GoProSensor(control=iface, capture_factory=FakeCapture)

    assert cam.open() is True
    assert http_log[0].endswith("/gopro/camera/stream/start")

    time.sleep(0.05)                         # let the keep-alive thread tick
    cam.close()
    assert http_log[-1].endswith("/gopro/camera/stream/stop")
    assert len(sock.sent) >= 1               # keep-alive packets went out


# ── get_state — /gopro/camera/state poll (HARDWARE_FINDINGS.md §2.4) ─────

# A representative /gopro/camera/state body; status keys are JSON strings.
_FAKE_STATE = {
    "status": {"6": 1, "8": 0, "32": 1, "70": 83, "86": 0},
    "settings": {"2": 12, "3": 100},
}


def test_get_state_parses_status_fields():
    iface = GoProInterface(http_get_json=lambda url, t: _FAKE_STATE)
    ev = iface.get_state()
    assert ev is not None
    assert ev.reachable is True
    assert ev.battery_percent == 83
    assert ev.system_hot is True
    assert ev.system_busy is False
    assert ev.preview_stream_active is True
    assert ev.thermal_throttle is False


def test_get_state_targets_the_state_endpoint():
    seen: list[str] = []
    iface = GoProInterface(
        http_get_json=lambda url, t: seen.append(url) or _FAKE_STATE)
    iface.get_state()
    assert seen == ["http://10.5.5.9:8080/gopro/camera/state"]


def test_get_state_none_when_unreachable():
    iface = GoProInterface(http_get_json=lambda url, t: None)
    assert iface.get_state() is None


def test_get_state_none_when_status_block_missing():
    iface = GoProInterface(http_get_json=lambda url, t: {"settings": {}})
    assert iface.get_state() is None


# ── GoProSensor status poll thread ───────────────────────────────────────

class FakeControlWithState:
    """GoProControl that also answers get_state — drives the status poller."""

    def __init__(self, state):
        self._state = state

    def start_preview(self): return True
    def keep_alive(self): return True
    def stop_preview(self): return True
    def get_state(self): return self._state


def test_status_poll_emits_events():
    ev = GoProStatusEvent(
        timestamp=1.0, reachable=True, battery_percent=77,
        system_hot=False, system_busy=False,
        preview_stream_active=True, thermal_throttle=False)
    received: list[GoProStatusEvent] = []
    cam = GoProSensor(
        control=FakeControlWithState(ev),
        capture_factory=FakeCapture,
        on_status=received.append,
        status_poll_interval_s=0.01)

    assert cam.open() is True
    time.sleep(0.05)
    cam.close()
    assert received and received[0].battery_percent == 77


def test_set_status_sink_enables_polling():
    """set_status_sink (used by SensorManager) arms the poller, same as the
    on_status constructor arg."""
    ev = GoProStatusEvent(
        timestamp=1.0, reachable=True, battery_percent=50,
        system_hot=False, system_busy=False,
        preview_stream_active=True, thermal_throttle=False)
    received: list[GoProStatusEvent] = []
    cam = GoProSensor(
        control=FakeControlWithState(ev),
        capture_factory=FakeCapture,
        status_poll_interval_s=0.01)
    cam.set_status_sink(received.append)       # registered before open()

    assert cam.open() is True
    time.sleep(0.05)
    cam.close()
    assert received and received[0].battery_percent == 50


def test_status_poll_emits_unreachable_when_state_is_none():
    received: list[GoProStatusEvent] = []
    cam = GoProSensor(
        control=FakeControlWithState(None),
        capture_factory=FakeCapture,
        on_status=received.append,
        status_poll_interval_s=0.01)

    assert cam.open() is True
    time.sleep(0.05)
    cam.close()
    assert received and received[0].reachable is False
