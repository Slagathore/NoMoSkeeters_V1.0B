"""Step 9: concrete sensors + SensorManager (offline, no hardware)."""
from __future__ import annotations

import time
from typing import Optional

import numpy as np
import pytest

import sensors.gopro as gopro_mod
import sensors.kinect_v2 as kinect_mod
from sensors.base import Sensor, SensorFrame, SensorRole
from sensors.gopro import GoProSensor
from sensors.kinect_v2 import KinectV2Sensor
from sensors.local_cam import LocalCamSensor
from sensors.sensor_manager import SensorManager


def _image() -> np.ndarray:
    return np.zeros((240, 320, 3), dtype=np.uint8)


class FakeCapture:
    """Duck-typed cv2.VideoCapture."""

    def __init__(self, frames=None, opened=True):
        self._frames = list(frames) if frames is not None else [_image()] * 50
        self._opened = opened

    def isOpened(self):
        return self._opened

    def read(self):
        if self._frames:
            f = self._frames.pop(0)
            return (f is not None, f)
        return (False, None)

    def release(self):
        self._opened = False


# ── LocalCamSensor ───────────────────────────────────────────────────────

def test_local_cam_open_and_read():
    cam = LocalCamSensor(capture_factory=lambda: FakeCapture([_image()]))
    assert cam.open() is True
    frame = cam.read()
    assert isinstance(frame, SensorFrame)
    assert frame.sensor_id == "local_cam"
    assert frame.width == 320 and frame.height == 240
    assert frame.timestamp_uncertainty_ms == 5.0
    assert frame.rgb is not None
    cam.close()


def test_local_cam_open_fails_when_capture_closed():
    cam = LocalCamSensor(capture_factory=lambda: FakeCapture(opened=False))
    assert cam.open() is False


def test_local_cam_capabilities():
    cam = LocalCamSensor()
    assert cam.has_rgb and not cam.has_depth and not cam.has_ir
    assert cam.role == SensorRole.FALLBACK


# ── GoProSensor ──────────────────────────────────────────────────────────

class FakeControl:
    def __init__(self):
        self.started = self.stopped = False
        self.keepalives = 0

    def start_preview(self):
        self.started = True
        return True

    def keep_alive(self):
        self.keepalives += 1
        return True

    def stop_preview(self):
        self.stopped = True
        return True


def test_gopro_open_read_close_with_control(monkeypatch):
    monkeypatch.setattr(gopro_mod, "_KEEPALIVE_INTERVAL_S", 0.01)
    control = FakeControl()
    cam = GoProSensor(control=control,  # type: ignore[arg-type]
                      capture_factory=lambda: FakeCapture([_image()] * 3))
    assert cam.open() is True
    assert control.started is True

    frame = cam.read()
    assert frame is not None
    assert frame.sensor_id == "gopro"
    assert frame.timestamp_uncertainty_ms == 20.0
    assert cam.role == SensorRole.TARGETING

    time.sleep(0.05)                       # let the keep-alive thread tick
    cam.close()
    assert control.stopped is True
    assert control.keepalives >= 1


def test_gopro_open_fails_when_stream_unavailable():
    cam = GoProSensor(capture_factory=lambda: FakeCapture(opened=False))
    assert cam.open() is False


# ── KinectV2Sensor ───────────────────────────────────────────────────────

def test_kinect_honest_failure_without_sdk(monkeypatch):
    # Force the no-SDK path so the guard is exercised whether or not the
    # pykinect2 binding is installed on the test host.
    monkeypatch.setattr(kinect_mod, "_HAS_PYKINECT", False)
    kinect = KinectV2Sensor()
    assert kinect.open() is False          # honest failure, no exception
    assert kinect.read() is None


def test_kinect_capabilities_and_role():
    kinect = KinectV2Sensor()
    assert kinect.has_rgb and kinect.has_depth and kinect.has_ir
    assert kinect.role == SensorRole.SAFETY


def test_kinect_world_projection():
    kinect = KinectV2Sensor()
    # A pixel at the principal point projects straight ahead at depth Z.
    pos = kinect.world_position(255, 205, depth_m=2.0)
    assert pos is not None
    x, y, z = pos
    assert abs(x) < 0.01 and abs(y) < 0.01
    assert z == pytest.approx(2.0)
    # An off-centre pixel projects off-axis.
    pos2 = kinect.world_position(455, 205, depth_m=2.0)
    assert pos2 is not None
    x2, _, _ = pos2
    assert x2 > 0.1
    assert kinect.world_position(0, 0, depth_m=0.0) is None


# ── SensorManager ────────────────────────────────────────────────────────

class FakeSensor(Sensor):
    """Emits a fixed number of frames, then None forever."""

    def __init__(self, sensor_id="fake", n_frames=5):
        self._id = sensor_id
        self._remaining = n_frames
        self.closed = False

    @property
    def sensor_id(self):
        return self._id

    @property
    def role(self):
        return SensorRole.TARGETING

    @property
    def has_rgb(self):
        return True

    @property
    def has_depth(self):
        return False

    @property
    def has_ir(self):
        return False

    def open(self):
        return True

    def close(self):
        self.closed = True

    def read(self) -> Optional[SensorFrame]:
        if self._remaining <= 0:
            return None
        self._remaining -= 1
        return SensorFrame.now(self._id)


def test_sensor_manager_forwards_frames_to_sink():
    received: list[SensorFrame] = []
    mgr = SensorManager(frame_sink=received.append)
    assert mgr.add_sensor(FakeSensor("fake", n_frames=5)) is True
    assert mgr.sensor_ids == ["fake"]

    deadline = time.monotonic() + 1.0
    while len(received) < 5 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert len(received) >= 5
    assert all(f.sensor_id == "fake" for f in received)
    assert mgr.latest_frame("fake") is not None
    mgr.stop_all()


def test_sensor_manager_rejects_duplicate_and_removes():
    mgr = SensorManager()
    s = FakeSensor("dup")
    assert mgr.add_sensor(s) is True
    assert mgr.add_sensor(FakeSensor("dup")) is False     # duplicate id
    mgr.remove_sensor("dup")
    assert mgr.sensor_ids == []
    assert s.closed is True


def test_sensor_manager_add_fails_if_sensor_wont_open():
    class DeadSensor(FakeSensor):
        def open(self):
            return False
    mgr = SensorManager()
    assert mgr.add_sensor(DeadSensor("dead")) is False


def test_sensor_manager_routes_status_events():
    """A sensor exposing set_status_sink has its status routed to the bus,
    tagged with sensor_id, and cached for latest_status()."""
    from events.schemas import GoProStatusEvent

    ev = GoProStatusEvent(
        timestamp=1.0, reachable=True, battery_percent=64,
        system_hot=False, system_busy=False,
        preview_stream_active=True, thermal_throttle=False)

    class StatusSensor(FakeSensor):
        _status_sink = None

        def set_status_sink(self, sink):
            self._status_sink = sink

        def open(self):
            if self._status_sink is not None:
                self._status_sink(ev)          # emit once on open
            return True

    received: list = []
    mgr = SensorManager(status_sink=lambda sid, e: received.append((sid, e)))
    assert mgr.add_sensor(StatusSensor("gopro")) is True
    assert received == [("gopro", ev)]         # tagged + forwarded
    assert mgr.latest_status("gopro") is ev    # cached
    mgr.remove_sensor("gopro")
    assert mgr.latest_status("gopro") is None  # cleared on removal
    mgr.stop_all()
