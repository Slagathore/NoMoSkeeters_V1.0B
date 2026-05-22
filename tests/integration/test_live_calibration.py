"""Step 11: live calibration loop, exercised offline with a coupled
fake cube + fake camera (the camera renders the dot wherever the cube was
last told to point, via a ground-truth homography)."""
from __future__ import annotations

import time

import cv2
import numpy as np

from sensors.base import SensorFrame
from targeting.calibration import apply_homography, validate_multi_depth
from targeting.calibration_controller import (
    default_truth_homography, live_validation_sweep, run_live_calibration,
)
from targeting.dot_detector import LaserDotDetector


class _BenchCube:
    """Fake transport: records the galvo coord of the last streamed frame."""

    def __init__(self, state):
        self._state = state

    def send_frame(self, points, frame_num=0):
        p = points[0]
        self._state["galvo_dac"] = (p.x, p.y)
        self._state["lit"] = bool(p.r or p.g or p.b)   # blanked => no dot
        return True

    def _next_msg_num(self):
        return 0


class _BenchCamera:
    """Fake camera: renders the laser dot where the cube is pointing."""

    def __init__(self, state, h_truth, *, size=320, blank=False):
        self._state = state
        self._h = h_truth
        self._size = size
        self._blank = blank

    def read(self):
        # A real camera paces read() at the frame interval; mimic that (fast)
        # so the capture hold-loops are paced instead of spinning the CPU.
        time.sleep(0.004)
        img = np.full((self._size, self._size, 3), 20, dtype=np.uint8)
        dac = self._state.get("galvo_dac")
        if dac is not None and self._state.get("lit") and not self._blank:
            gnorm = (dac[0] / 0xFFF, dac[1] / 0xFFF)
            s = apply_homography(self._h, [gnorm])[0]
            px = int(np.clip(s[0], 0, 1) * (self._size - 1))
            py = int(np.clip(s[1], 0, 1) * (self._size - 1))
            cv2.circle(img, (px, py), 5, (255, 255, 255), -1)
        frame = SensorFrame.now("bench_cam")
        frame.rgb = img
        frame.width, frame.height = self._size, self._size
        return frame


def _bench(blank=False):
    state: dict = {}
    h = default_truth_homography()
    return _BenchCube(state), _BenchCamera(state, h, blank=blank), h


# ── run_live_calibration ─────────────────────────────────────────────────

def test_live_calibration_fits_and_saves(tmp_path):
    cube, camera, _ = _bench()
    run = run_live_calibration(cube, camera, pattern="grid",
                               sensor_id="gopro", out_dir=tmp_path,
                               dwell_samples=1, hold_s=0.05, settle_s=0.0)
    assert run.json_path is not None and run.json_path.exists()
    assert run.mapper.n_points == 25                 # full 5×5 grid detected
    # The fitted mapper recovers the bench's ground-truth homography well.
    assert run.mapper.residual_norm < 0.01


def test_live_calibration_single_background_mode(tmp_path):
    cube, camera, _ = _bench()
    run = run_live_calibration(cube, camera, pattern="grid", out_dir=tmp_path,
                               dwell_samples=1, hold_s=0.05, settle_s=0.0,
                               background_mode="single")
    assert run.mapper.n_points == 25                 # one shared reference
    assert run.mapper.residual_norm < 0.01


def test_live_calibration_raises_when_no_dots(tmp_path):
    cube, camera, _ = _bench(blank=True)             # camera sees nothing
    import pytest
    with pytest.raises(RuntimeError):
        run_live_calibration(cube, camera, pattern="grid", out_dir=tmp_path,
                             dwell_samples=1, hold_s=0.05, settle_s=0.0)


def test_live_calibration_reports_progress(tmp_path):
    cube, camera, _ = _bench()
    seen: list[int] = []
    run_live_calibration(cube, camera, pattern="grid", out_dir=tmp_path,
                         dwell_samples=1, hold_s=0.05, settle_s=0.0,
                         on_point=lambda i, n, obs: seen.append(i))
    assert seen == list(range(25))


# ── live_validation_sweep + validate_multi_depth ─────────────────────────

def test_live_validation_sweep_passes_for_good_calibration(tmp_path):
    cube, camera, _ = _bench()
    run = run_live_calibration(cube, camera, pattern="grid",
                               out_dir=tmp_path, dwell_samples=1,
                               hold_s=0.05, settle_s=0.0)
    targets = []
    for depth in (1.0, 2.5, 4.0):
        targets.extend(live_validation_sweep(
            cube, camera, depth, pattern="grid",
            dwell_samples=1, hold_s=0.05, settle_s=0.0))
    assert len(targets) == 75                        # 25 points × 3 depths
    result = validate_multi_depth(run.mapper, targets)
    assert result.passed is True
