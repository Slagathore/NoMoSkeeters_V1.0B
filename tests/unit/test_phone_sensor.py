"""PhoneSensor — Sensor wrapper around client + decoder."""
from __future__ import annotations

import numpy as np

from events.schemas import (PhoneCameraChangedEvent, PhoneFocusEvent,
                            PhoneThermalEvent)
from sensors.base import SensorRole
from sensors.phone import PhoneSensor


class _FakeClient:
    """Duck-typed PhoneSensorClient."""

    def __init__(self, *, connect_ok=True, switch_ok=True,
                 stream_mode="raw_yuv"):
        self.connect_ok = connect_ok
        self.switch_ok = switch_ok
        self.stream_mode = stream_mode
        self.events: list = []
        self.event_sink = None
        self.is_connected = False
        self.set_camera_calls: list = []
        self.stream_starts: list = []
        self.stream_stops = 0
        self.disconnects = 0

    def set_event_sink(self, sink):
        self.event_sink = sink

    def connect(self):
        self.is_connected = self.connect_ok
        return self.connect_ok

    def set_active_camera(self, camera_id, **_):
        self.set_camera_calls.append(camera_id)
        return self.switch_ok

    def stream_start(self, *, mode, width, height, fps):
        self.stream_starts.append((mode, width, height, fps))
        return self.stream_mode

    def stream_stop(self):
        self.stream_stops += 1
        return True

    def disconnect(self):
        self.disconnects += 1
        self.is_connected = False

    # Test helper — fire an unsolicited event through the registered sink.
    def push(self, msg):
        if self.event_sink:
            self.event_sink(msg)


class _FakeDecoder:
    """Duck-typed PhoneFrameDecoder with scriptable frames + camera_ids."""

    def __init__(self, frames, camera_ids=None):
        self._frames = list(frames)
        self._cam_ids = list(camera_ids or [])
        self._idx = 0
        self.released = False
        self.width = 0
        self.height = 0

    def isOpened(self):
        return not self.released

    def read(self):
        if not self._frames or self._idx >= len(self._frames):
            return (False, None)
        f = self._frames[self._idx]
        self._idx += 1
        return (True, f)

    @property
    def latest_camera_id(self):
        if self._idx == 0:
            return None
        cam_idx = min(self._idx - 1, len(self._cam_ids) - 1)
        return self._cam_ids[cam_idx] if self._cam_ids else None

    def release(self):
        self.released = True


def _bgr(width: int, height: int, fill: int = 100):
    return np.full((height, width, 3), fill, dtype=np.uint8)


def _build_sensor(client, *, frames=None, camera_ids=None,
                  stream_mode="raw_yuv"):
    if frames is None:
        frames = [_bgr(16, 12)]
    decoder = _FakeDecoder(frames, camera_ids)

    def factory(*, width, height, codec):
        decoder.width = width
        decoder.height = height
        decoder.codec = codec
        return decoder

    sensor = PhoneSensor(client=client, decoder_factory=factory,
                         stream_mode=stream_mode,
                         stream_width=16, stream_height=12)
    return sensor, decoder


# ── open/close ───────────────────────────────────────────────────────────

def test_open_runs_handshake_set_camera_stream_start_and_decoder():
    client = _FakeClient()
    sensor, decoder = _build_sensor(client)
    assert sensor.open() is True
    assert client.set_camera_calls == ["main"]
    assert client.stream_starts and client.stream_starts[0][0] == "raw_yuv"
    assert decoder.codec == "nv21"           # raw_yuv → nv21 decoder
    sensor.close()
    assert decoder.released and client.disconnects == 1


def test_open_fails_when_client_connect_fails():
    client = _FakeClient(connect_ok=False)
    sensor, _ = _build_sensor(client)
    assert sensor.open() is False


def test_open_fails_when_set_camera_fails():
    client = _FakeClient(switch_ok=False)
    sensor, _ = _build_sensor(client)
    assert sensor.open() is False
    assert client.disconnects == 1            # client gets cleaned up


def test_open_picks_h264_codec_for_h264_modes():
    client = _FakeClient(stream_mode="h264_lowlat")
    sensor, decoder = _build_sensor(client)
    sensor = PhoneSensor(client=client, decoder_factory=lambda *, width, height,
                          codec: setattr(decoder, "codec", codec) or decoder,
                          stream_mode="h264_lowlat",
                          stream_width=16, stream_height=12)
    assert sensor.open() is True
    assert decoder.codec == "h264"


# ── read ─────────────────────────────────────────────────────────────────

def test_read_emits_sensor_frame_with_rgb():
    client = _FakeClient()
    sensor, _ = _build_sensor(client, frames=[_bgr(32, 24)])
    sensor.open()
    frame = sensor.read()
    assert frame is not None
    assert frame.rgb is not None and frame.rgb.shape == (24, 32, 3)
    assert (frame.width, frame.height) == (32, 24)
    assert frame.sensor_id == "phone_main"
    assert frame.sensor_role == SensorRole.TARGETING.value
    sensor.close()


def test_read_retags_sensor_id_when_camera_changes_mid_stream():
    client = _FakeClient()
    sensor, _ = _build_sensor(
        client, frames=[_bgr(16, 16), _bgr(16, 16)],
        camera_ids=["main", "telephoto"])
    sensor.open()
    f1 = sensor.read()
    f2 = sensor.read()
    assert f1.sensor_id == "phone_main"
    assert f2.sensor_id == "phone_telephoto"
    assert sensor.active_camera == "telephoto"
    sensor.close()


def test_read_returns_none_when_decoder_dry():
    client = _FakeClient()
    sensor, _ = _build_sensor(client, frames=[])
    sensor.open()
    assert sensor.read() is None
    sensor.close()


# ── switch_camera ────────────────────────────────────────────────────────

def test_switch_camera_commands_phone_and_relabels():
    client = _FakeClient()
    sensor, _ = _build_sensor(client)
    sensor.open()
    assert sensor.switch_camera("telephoto") is True
    assert "telephoto" in client.set_camera_calls
    assert sensor.sensor_id == "phone_telephoto"
    sensor.close()


def test_switch_camera_fails_when_phone_rejects():
    client = _FakeClient(switch_ok=False)
    # Manual setup: open() would fail (switch_ok=False) so skip open here.
    sensor, _ = _build_sensor(_FakeClient())
    sensor.open()
    sensor._client = client                   # swap in the rejecting client
    assert sensor.switch_camera("telephoto") is False
    sensor.close()


# ── event translation ───────────────────────────────────────────────────

def test_event_sink_translates_camera_changed():
    client = _FakeClient()
    seen: list = []
    sensor, _ = _build_sensor(client)
    sensor.set_status_sink(seen.append)
    sensor.open()
    client.push({"type": "event:camera_changed", "camera_id": "telephoto",
                 "width": 1280, "height": 720, "fov_h_deg": 23.0,
                 "has_optical_zoom": True, "optical_zoom_factor": 3.0})
    assert seen and isinstance(seen[0], PhoneCameraChangedEvent)
    ev = seen[0]
    assert ev.camera_id == "telephoto"
    assert ev.optical_zoom_factor == 3.0
    sensor.close()


def test_event_sink_translates_af_settled():
    client = _FakeClient()
    seen: list = []
    sensor, _ = _build_sensor(client)
    sensor.set_status_sink(seen.append)
    sensor.open()
    client.push({"type": "event:af_settled", "state": "settled",
                 "region": [0.4, 0.4, 0.2, 0.2]})
    assert seen and isinstance(seen[0], PhoneFocusEvent)
    assert seen[0].region == (0.4, 0.4, 0.2, 0.2)
    sensor.close()


def test_event_sink_translates_thermal_warning():
    client = _FakeClient()
    seen: list = []
    sensor, _ = _build_sensor(client)
    sensor.set_status_sink(seen.append)
    sensor.open()
    client.push({"type": "event:thermal_warning", "state": "moderate",
                 "battery_percent": 42})
    assert seen and isinstance(seen[0], PhoneThermalEvent)
    assert seen[0].state == "moderate"
    assert seen[0].battery_percent == 42
    sensor.close()


def test_unknown_event_dropped_silently():
    client = _FakeClient()
    seen: list = []
    sensor, _ = _build_sensor(client)
    sensor.set_status_sink(seen.append)
    sensor.open()
    client.push({"type": "event:lasers_engaged"})    # not a real phone event
    assert seen == []
    sensor.close()
