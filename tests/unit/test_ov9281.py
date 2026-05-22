"""OV9281Sensor — UVC config + frame read, with a fake VideoCapture."""
from __future__ import annotations

import cv2
import numpy as np

from sensors.base import SensorRole
from sensors.ov9281 import OV9281Sensor


class FakeCapture:
    """Duck-typed cv2.VideoCapture: records set() props, scripts read()."""

    def __init__(self, frames=None, opened=True):
        self._frames = list(frames or [])
        self._opened = opened
        self.props: dict[int, float] = {}
        self.released = False

    def isOpened(self):
        return self._opened

    def set(self, prop, value):
        self.props[prop] = value
        return True

    def get(self, prop):
        return self.props.get(prop, 0)

    def read(self):
        if not self._frames:
            return (False, None)
        return (True, self._frames.pop(0))

    def release(self):
        self.released = True


def _img(w=640, h=480, fill=50):
    return np.full((h, w, 3), fill, dtype=np.uint8)


def test_open_configures_mjpeg_resolution_and_manual_exposure():
    cap = FakeCapture([_img()])
    s = OV9281Sensor(capture_factory=lambda: cap,
                     width=640, height=480, fps=210,
                     auto_exposure=False, exposure=-6.0, gain=2.0)
    assert s.open() is True
    assert cap.props[cv2.CAP_PROP_FRAME_WIDTH] == 640
    assert cap.props[cv2.CAP_PROP_FRAME_HEIGHT] == 480
    assert cap.props[cv2.CAP_PROP_FPS] == 210
    assert cv2.CAP_PROP_FOURCC in cap.props          # MJPEG selected
    assert cap.props[cv2.CAP_PROP_AUTO_EXPOSURE] == 0.25   # manual
    assert cap.props[cv2.CAP_PROP_EXPOSURE] == -6.0
    assert cap.props[cv2.CAP_PROP_GAIN] == 2.0
    s.close()
    assert cap.released


def test_auto_exposure_skips_manual_props():
    cap = FakeCapture([_img()])
    s = OV9281Sensor(capture_factory=lambda: cap, auto_exposure=True)
    assert s.open() is True
    assert cap.props[cv2.CAP_PROP_AUTO_EXPOSURE] == 0.75
    assert cv2.CAP_PROP_EXPOSURE not in cap.props
    s.close()


def test_open_fails_when_capture_not_opened():
    cap = FakeCapture(opened=False)
    s = OV9281Sensor(capture_factory=lambda: cap)
    assert s.open() is False
    assert cap.released                              # cleaned up on failure


def test_read_yields_rgb_sensor_frame():
    cap = FakeCapture([_img(320, 240)])
    s = OV9281Sensor(capture_factory=lambda: cap)
    assert s.open() is True
    frame = s.read()
    assert frame is not None and frame.rgb is not None
    assert (frame.width, frame.height) == (320, 240)
    assert frame.sensor_id == "ov9281"
    assert frame.sensor_role == SensorRole.TARGETING.value
    s.close()


def test_read_returns_none_when_dry():
    cap = FakeCapture([])
    s = OV9281Sensor(capture_factory=lambda: cap)
    s.open()
    assert s.read() is None
    s.close()


def test_capabilities_and_role():
    s = OV9281Sensor(capture_factory=lambda: FakeCapture())
    assert s.sensor_id == "ov9281"
    assert s.has_rgb and not s.has_depth and not s.has_ir
    assert s.role == SensorRole.TARGETING
