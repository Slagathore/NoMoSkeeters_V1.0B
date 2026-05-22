"""KinectV2Sensor — Microsoft Kinect v2, the SAFETY-role depth sensor.

Emits RGB (1920×1080), depth (512×424, metres) and IR (512×424) merged into
one SensorFrame per read. Depth + IR give darkness operation (the 850nm IR
flood works without visible light).

The pykinect2 binding is imported behind a guard: the module imports cleanly
without the Kinect SDK installed, and open() reports an honest failure rather
than raising. Only one Kinect is allowed per process (SDK constraint).

Reference: BOOTSTRAP.md §5.3, §4 (K-WORLD), BOOTSTRAP_AMENDMENTS §5.3 (the
width/height bug fix).
"""
from __future__ import annotations

import logging
from typing import Any, Optional

import cv2
import numpy as np

from sensors.base import Sensor, SensorFrame, SensorRole

_log = logging.getLogger(__name__)

# The pykinect2 binding has no type stubs and is absent on dev machines, so the
# names are declared Any and rebound by the optional import — attribute access
# (PyKinectV2.FrameSourceTypes_*, runtime methods) then type-checks cleanly
# whether or not the SDK is installed.
PyKinectRuntime: Any = None
PyKinectV2: Any = None
try:
    from pykinect2 import PyKinectRuntime, PyKinectV2  # type: ignore[no-redef]
    _HAS_PYKINECT = True
except Exception:                                     # pragma: no cover
    _HAS_PYKINECT = False

# Nominal Kinect v2 depth-camera intrinsics (512×424). These are the standard
# published values — replace with the SDK CoordinateMapper or a per-unit
# calibration for production accuracy.
_DEPTH_FX = 365.456
_DEPTH_FY = 365.456
_DEPTH_CX = 254.878
_DEPTH_CY = 205.395

_COLOR_W, _COLOR_H = 1920, 1080
_DEPTH_W, _DEPTH_H = 512, 424

# The Kinect SDK allows exactly one runtime per process; this guards it.
_KINECT_OPEN = False


def pykinect2_available() -> bool:
    """Whether the Kinect SDK binding imported successfully."""
    return _HAS_PYKINECT


class KinectV2Sensor(Sensor):
    """Singleton Kinect v2 sensor. SAFETY role by default (§5 amendment)."""

    def __init__(self, role: SensorRole = SensorRole.SAFETY,
                 timestamp_uncertainty_ms: float = 8.0):
        self._role = role
        self._ts_uncertainty = timestamp_uncertainty_ms
        self._kinect = None
        self._width = 0
        self._height = 0

    # ── Identity / capabilities ──────────────────────────────────────────

    @property
    def sensor_id(self) -> str:
        return "kinect_v2"

    @property
    def role(self) -> SensorRole:
        return self._role

    @property
    def has_rgb(self) -> bool:
        return True

    @property
    def has_depth(self) -> bool:
        return True

    @property
    def has_ir(self) -> bool:
        return True

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    # ── Lifecycle ────────────────────────────────────────────────────────

    def open(self) -> bool:
        global _KINECT_OPEN
        if not _HAS_PYKINECT:
            _log.error("kinect_v2: pykinect2 not installed — sensor unavailable")
            return False
        if _KINECT_OPEN:
            _log.error("kinect_v2: a Kinect runtime is already open "
                       "(the SDK allows one per process)")
            return False
        try:
            # RGB + depth + IR. Body tracking (dynamic no-fire zones around
            # people/pets) is a separate safety-layer feature — not streamed
            # here, so the Body source is deliberately not requested.
            sources = (PyKinectV2.FrameSourceTypes_Color
                       | PyKinectV2.FrameSourceTypes_Depth
                       | PyKinectV2.FrameSourceTypes_Infrared)
            self._kinect = PyKinectRuntime.PyKinectRuntime(sources)
        except Exception:
            _log.exception("kinect_v2: PyKinectRuntime init failed")
            return False
        if self._kinect is None:
            return False
        _KINECT_OPEN = True
        return True

    def close(self) -> None:
        global _KINECT_OPEN
        if self._kinect is not None:
            try:
                self._kinect.close()
            except Exception:
                _log.exception("kinect_v2: close() raised")
            self._kinect = None
        _KINECT_OPEN = False

    def read(self) -> Optional[SensorFrame]:
        if self._kinect is None:
            return None
        frame = SensorFrame.now(self.sensor_id, self._role)

        if self._kinect.has_new_color_frame():
            color = self._kinect.get_last_color_frame().reshape(
                (_COLOR_H, _COLOR_W, 4))
            frame.rgb = cv2.cvtColor(color, cv2.COLOR_BGRA2BGR)
        if self._kinect.has_new_depth_frame():
            depth_mm = self._kinect.get_last_depth_frame().reshape(
                (_DEPTH_H, _DEPTH_W))
            frame.depth = depth_mm.astype(np.float32) * 0.001
        if self._kinect.has_new_infrared_frame():
            frame.ir = self._kinect.get_last_infrared_frame().reshape(
                (_DEPTH_H, _DEPTH_W))

        if frame.rgb is None and frame.depth is None and frame.ir is None:
            return None

        # §5.3 fix: width/height must be set — downstream normalizes by them.
        # Take them from whichever stream is present (IR-only is valid — the
        # depth camera can update without a new colour frame).
        ref = (frame.rgb if frame.rgb is not None else
               frame.depth if frame.depth is not None else frame.ir)
        assert ref is not None        # guaranteed: the all-None case returned above
        frame.height, frame.width = ref.shape[:2]
        self._width, self._height = frame.width, frame.height
        frame.timestamp_uncertainty_ms = self._ts_uncertainty
        return frame

    # ── Depth → world projection (K-WORLD, §4) ───────────────────────────

    def world_position(self, x_px: int, y_px: int,
                       depth_m: float) -> Optional[tuple[float, float, float]]:
        """Project a depth-camera pixel + depth into camera-relative metres
        (X right, Y up, Z forward) via the pinhole model."""
        if depth_m <= 0.0:
            return None
        x = (x_px - _DEPTH_CX) * depth_m / _DEPTH_FX
        y = -(y_px - _DEPTH_CY) * depth_m / _DEPTH_FY      # image y-down → Y-up
        return (float(x), float(y), float(depth_m))
