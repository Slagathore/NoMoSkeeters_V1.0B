"""Calibration controller — drives a LaserCubeTransport through a pattern,
collects galvo↔sensor correspondences, fits a CoordinateMapper, and writes
a calibration JSON v2.

Against DryRunTransport this runs fully offline: a SyntheticCamera stands in
for the real sensor, applying a ground-truth homography (plus noise) so the
fitted mapper can be checked against known truth and the path visualized.

Reference: BOOTSTRAP.md §8.3-§8.5, BOOTSTRAP_AMENDMENTS.md §8.5-§8.6,
Step 6 of the §17 implementation order.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from config import settings
from laser.transport import LaserCubeTransport
from laser.types import LaserPoint
from targeting.calibration import (
    CoordinateMapper, TestTarget, ValidationResult,
    apply_homography, validate_multi_depth,
)
from targeting.dot_detector import DotObservation, LaserDotDetector
from targeting.patterns import galvo_norm_to_dac, generate_pattern

_log = logging.getLogger(__name__)

Point = tuple[float, float]


# ── Synthetic camera (dry-run stand-in for a real sensor) ────────────────

def default_truth_homography() -> np.ndarray:
    """A plausible galvo-NORM → sensor-NORM homography: a modest rotation,
    scale, off-center translation and slight perspective. Used as ground
    truth for dry-run calibration so the fit can be checked against it."""
    theta = np.deg2rad(7.0)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [0.85 * c, -0.85 * s, 0.08],
        [0.80 * s,  0.80 * c, 0.06],
        [0.05,      0.03,     1.00],
    ], dtype=float)


@dataclass
class SyntheticCamera:
    """Stand-in for a real camera during dry-run calibration.

    observe() applies the ground-truth galvo→norm homography plus Gaussian
    pixel noise. depth_error_per_m injects a depth-dependent bias so
    multi-depth validation has something non-trivial to measure.
    """
    H_truth: np.ndarray
    noise_norm: float = 0.0015
    depth_error_per_m: float = 0.0
    calibration_depth_m: float = 2.5
    seed: int = 0

    def __post_init__(self) -> None:
        self._rng = np.random.default_rng(self.seed)

    def observe(self, galvo_xy: Point, depth_m: Optional[float] = None) -> Point:
        """Return the sensor-NORM coord where galvo_xy would be detected."""
        norm = apply_homography(self.H_truth, [galvo_xy])[0]
        norm = norm + self._rng.normal(0.0, self.noise_norm, size=2)
        if depth_m is not None and self.depth_error_per_m:
            # Planar homography degrades off the calibration plane.
            drift = self.depth_error_per_m * abs(depth_m - self.calibration_depth_m)
            norm = norm + self._rng.normal(0.0, drift, size=2)
        return float(norm[0]), float(norm[1])


# ── Calibration run result ───────────────────────────────────────────────

@dataclass
class CalibrationRun:
    mapper: CoordinateMapper
    galvo_pts: list[Point]
    observed_norm: list[Point]
    json_path: Optional[Path] = None

    def render_image(self, size: int = 480) -> np.ndarray:
        """Dry-fire visualization: draw observed dots on a synthetic camera
        image so the calibration path can be inspected without hardware."""
        import cv2
        img = np.zeros((size, size, 3), dtype=np.uint8)
        for (gx, gy), (nx, ny) in zip(self.galvo_pts, self.observed_norm):
            px = int(np.clip(nx, 0, 1) * (size - 1))
            py = int(np.clip(ny, 0, 1) * (size - 1))
            cv2.circle(img, (px, py), 4, (40, 230, 40), -1)
        return img


# ── Dry-run calibration ──────────────────────────────────────────────────

def run_dry_calibration(transport: LaserCubeTransport,
                        *,
                        pattern: Optional[str] = None,
                        sensor_id: str = "gopro",
                        scene: str = "dry_run",
                        mount_tilt_config: str = "synthetic",
                        camera: Optional[SyntheticCamera] = None,
                        out_dir: Optional[Path] = None,
                        validate: bool = True,
                        stream: str = "",
                        kinect_relative_pose: Optional[dict] = None) -> CalibrationRun:
    """Run a full calibration against a transport using a SyntheticCamera.

    Fires each pattern point through the transport, observes it, fits a
    CoordinateMapper, optionally runs multi-depth validation, and writes a
    calibration JSON v2. Returns the CalibrationRun.

    For a Kinect→GALVO calibration (§8.10) pass sensor_id="kinect_v2" plus
    the `stream` ("rgb"/"ir") and `kinect_relative_pose` placement hint.
    """
    pattern = pattern or settings.CALIBRATION_PATTERN
    camera = camera or SyntheticCamera(H_truth=default_truth_homography())

    if not transport.is_connected():
        transport.connect()

    galvo_path = generate_pattern(
        pattern,
        grid_rows=settings.CALIBRATION_GRID_ROWS,
        grid_cols=settings.CALIBRATION_GRID_COLS,
        n_points=settings.CALIBRATION_POINTS,
        dragline_duration_s=settings.CALIBRATION_DRAGLINE_DURATION_S,
        dac_rate=settings.LASERCUBE_DEFAULT_DAC_RATE,
        windmill_arms=settings.CALIBRATION_WINDMILL_ARMS,
        windmill_revolutions=settings.CALIBRATION_WINDMILL_REVOLUTIONS,
    )

    r, g, b = (settings.CALIBRATION_LASER_R,
               settings.CALIBRATION_LASER_G,
               settings.CALIBRATION_LASER_B)

    galvo_pts: list[Point] = []
    observed: list[Point] = []
    for gx, gy in galvo_path:
        dot = LaserPoint(x=galvo_norm_to_dac(gx), y=galvo_norm_to_dac(gy),
                         r=r, g=g, b=b)
        transport.send_frame([dot], frame_num=0)
        galvo_pts.append((gx, gy))
        observed.append(camera.observe((gx, gy)))

    mapper = CoordinateMapper.fit(
        galvo_pts, observed,
        sensor_id=sensor_id, pattern=pattern,
        mount_tilt_config=mount_tilt_config, scene=scene,
    )
    _log.info("calibration fit: %d pts, residual_norm=%.4f, residual_galvo=%.1f",
              mapper.n_points, mapper.residual_norm, mapper.residual_galvo)

    mapper.stream = stream
    mapper.kinect_relative_pose = kinect_relative_pose

    if validate:
        mapper.validation = _validate(mapper, galvo_path, camera)

    out_dir = out_dir or (settings.CALIBRATIONS_DIR / sensor_id)
    json_path = mapper.save(Path(out_dir) / f"{mount_tilt_config}_{scene}.json")

    return CalibrationRun(mapper=mapper, galvo_pts=galvo_pts,
                          observed_norm=observed, json_path=json_path)


def _validate(mapper: CoordinateMapper,
              galvo_path: list[Point],
              camera: SyntheticCamera) -> ValidationResult:
    """Sweep the pattern again at each configured depth and validate."""
    targets: list[TestTarget] = []
    for depth_m in settings.CALIBRATION_VALIDATION_DEPTHS_M:
        for g in galvo_path:
            targets.append(TestTarget(
                galvo=g,
                observed_norm=camera.observe(g, depth_m=depth_m),
                depth_m=depth_m,
            ))
    result = validate_multi_depth(mapper, targets)
    _log.info("multi-depth validation: max_residual=%.4f passed=%s",
              result.max_residual_norm, result.passed)
    return result


# ── Live calibration (Step 11 — real cube, real camera) ──────────────────

# (point_index, point_total, observation-or-None)
PointCallback = Callable[[int, int, Optional[DotObservation]], None]


def calibration_pattern_path(pattern: Optional[str] = None) -> list[Point]:
    """The galvo-NORM pattern path for a calibration run, from config."""
    pattern = pattern or settings.CALIBRATION_PATTERN
    return generate_pattern(
        pattern,
        grid_rows=settings.CALIBRATION_GRID_ROWS,
        grid_cols=settings.CALIBRATION_GRID_COLS,
        n_points=settings.CALIBRATION_POINTS,
        dragline_duration_s=settings.CALIBRATION_DRAGLINE_DURATION_S,
        dac_rate=settings.LASERCUBE_DEFAULT_DAC_RATE,
        windmill_arms=settings.CALIBRATION_WINDMILL_ARMS,
        windmill_revolutions=settings.CALIBRATION_WINDMILL_REVOLUTIONS,
    )


def _capture_background(cube, camera, *, settle_s: float, dwell_samples: int,
                        on_frame: Optional[Callable] = None,
                        n_average: int = 5) -> Optional[np.ndarray]:
    """Hold the laser blanked and capture an averaged laser-OFF frame — the
    reference for temporal-difference detection. Averaging a few frames
    knocks down sensor/compression noise.
    """
    blank = [LaserPoint(x=0, y=0, r=0, g=0, b=0)] * max(1, dwell_samples)
    start = time.monotonic()
    stack: list = []
    while True:
        cube.send_frame(blank, frame_num=0)
        frame = camera.read()
        elapsed = time.monotonic() - start
        if frame is not None and frame.rgb is not None:
            if on_frame is not None:
                on_frame(frame.rgb)
            if elapsed >= settle_s:
                stack.append(frame.rgb.astype(np.float32))
        if len(stack) >= n_average or elapsed >= settle_s + 3.0:
            break
    if not stack:
        return None
    return np.mean(stack, axis=0).astype(np.uint8)


def _capture_dot(cube, camera, detector: LaserDotDetector,
                 gx_dac: int, gy_dac: int, rgb: tuple[int, int, int],
                 *, hold_s: float, settle_s: float, dwell_samples: int,
                 background: Optional[np.ndarray] = None,
                 on_frame: Optional[Callable] = None) -> Optional[DotObservation]:
    """Hold the laser STEADY at one galvo coord, then detect the dot.

    The dwell is re-streamed continuously for `hold_s` seconds so the dot
    stays lit the whole time — a single 0.1 s flash is far shorter than the
    camera pipeline latency (several hundred ms on the USB GoPro feed), so a
    flash-then-read never sees the current point. Detection only starts
    after `settle_s`, by which time the lit dot is guaranteed to be in the
    frames; the highest-confidence observation over the window is returned.

    `on_frame`, if given, is called with every camera frame (rgb) — used by
    the step11 script's --view option.
    """
    r, g, b = rgb
    dwell = [LaserPoint(x=gx_dac, y=gy_dac, r=r, g=g, b=b)] * dwell_samples
    start = time.monotonic()
    best: Optional[DotObservation] = None
    while True:
        cube.send_frame(dwell, frame_num=0)       # keep the ringbuffer fed
        frame = camera.read()
        elapsed = time.monotonic() - start
        if frame is not None and frame.rgb is not None:
            if on_frame is not None:
                on_frame(frame.rgb)
            if elapsed >= settle_s:               # latency cleared — dot is live
                obs = (detector.detect_diff(frame.rgb, background)
                       if background is not None
                       else detector.detect(frame.rgb))
                if obs is not None and (
                        best is None or obs.confidence > best.confidence):
                    best = obs
        if elapsed >= hold_s:
            return best


def run_live_calibration(cube, camera, *,
                         pattern: Optional[str] = None,
                         sensor_id: str = "gopro",
                         scene: str = "live",
                         mount_tilt_config: str = "bench",
                         detector: Optional[LaserDotDetector] = None,
                         out_dir: Optional[Path] = None,
                         stream: str = "",
                         kinect_relative_pose: Optional[dict] = None,
                         settle_s: Optional[float] = None,
                         hold_s: Optional[float] = None,
                         dwell_samples: int = 3000,
                         laser_rgb: Optional[tuple] = None,
                         on_point: Optional[PointCallback] = None,
                         on_frame: Optional[Callable] = None) -> CalibrationRun:
    """Calibrate against the real cube + camera (Step 11).

    The caller (the step11 script) must have connected the cube and ENABLED
    OUTPUT — that path owns the operator confirmations and is not buried here.
    Multi-depth validation is interactive; run it via live_validation_sweep()
    + validate_multi_depth() after this returns.
    """
    pattern = pattern or settings.CALIBRATION_PATTERN
    detector = detector or LaserDotDetector()
    settle_s = (settings.CALIBRATION_DWELL_SETTLE_S if settle_s is None
                else settle_s)
    hold_s = (settings.CALIBRATION_DWELL_HOLD_S if hold_s is None
              else hold_s)
    rgb = laser_rgb if laser_rgb is not None else (
        settings.CALIBRATION_LASER_R, settings.CALIBRATION_LASER_G,
        settings.CALIBRATION_LASER_B)
    galvo_path = calibration_pattern_path(pattern)
    # Laser-OFF reference for temporal-difference detection.
    background = _capture_background(cube, camera, settle_s=settle_s,
                                     dwell_samples=dwell_samples,
                                     on_frame=on_frame)

    galvo_pts: list[Point] = []
    observed: list[Point] = []
    for idx, (gx, gy) in enumerate(galvo_path):
        obs = _capture_dot(cube, camera, detector,
                           galvo_norm_to_dac(gx), galvo_norm_to_dac(gy), rgb,
                           hold_s=hold_s, settle_s=settle_s,
                           dwell_samples=dwell_samples,
                           background=background,
                           on_frame=on_frame)
        if on_point is not None:
            on_point(idx, len(galvo_path), obs)
        if obs is None:
            _log.warning("live calibration: no dot for galvo point %d/%d",
                         idx + 1, len(galvo_path))
            continue
        galvo_pts.append((gx, gy))
        observed.append((obs.x_norm, obs.y_norm))

    if len(galvo_pts) < 4:
        raise RuntimeError(
            f"only {len(galvo_pts)} of {len(galvo_path)} dots detected — "
            f"need >=4 to fit a homography")

    mapper = CoordinateMapper.fit(
        galvo_pts, observed, sensor_id=sensor_id, pattern=pattern,
        mount_tilt_config=mount_tilt_config, scene=scene)
    mapper.stream = stream
    mapper.kinect_relative_pose = kinect_relative_pose
    _log.info("live calibration fit: %d/%d pts, residual_norm=%.4f",
              len(galvo_pts), len(galvo_path), mapper.residual_norm)

    out_dir = out_dir or (settings.CALIBRATIONS_DIR / sensor_id)
    json_path = mapper.save(Path(out_dir) / f"{mount_tilt_config}_{scene}.json")
    return CalibrationRun(mapper=mapper, galvo_pts=galvo_pts,
                          observed_norm=observed, json_path=json_path)


def live_validation_sweep(cube, camera, depth_m: float, *,
                          pattern: Optional[str] = None,
                          detector: Optional[LaserDotDetector] = None,
                          settle_s: Optional[float] = None,
                          hold_s: Optional[float] = None,
                          dwell_samples: int = 3000,
                          laser_rgb: Optional[tuple] = None,
                          on_point: Optional[PointCallback] = None,
                          on_frame: Optional[Callable] = None) -> list[TestTarget]:
    """Sweep the pattern once at a known depth; return TestTargets for
    validate_multi_depth(). The operator places the target plane at depth_m
    before calling this."""
    pattern = pattern or settings.CALIBRATION_PATTERN
    detector = detector or LaserDotDetector()
    settle_s = (settings.CALIBRATION_DWELL_SETTLE_S if settle_s is None
                else settle_s)
    hold_s = (settings.CALIBRATION_DWELL_HOLD_S if hold_s is None
              else hold_s)
    rgb = laser_rgb if laser_rgb is not None else (
        settings.CALIBRATION_LASER_R, settings.CALIBRATION_LASER_G,
        settings.CALIBRATION_LASER_B)
    galvo_path = calibration_pattern_path(pattern)
    # Laser-OFF reference for temporal-difference detection.
    background = _capture_background(cube, camera, settle_s=settle_s,
                                     dwell_samples=dwell_samples,
                                     on_frame=on_frame)

    targets: list[TestTarget] = []
    for idx, (gx, gy) in enumerate(galvo_path):
        obs = _capture_dot(cube, camera, detector,
                           galvo_norm_to_dac(gx), galvo_norm_to_dac(gy), rgb,
                           hold_s=hold_s, settle_s=settle_s,
                           dwell_samples=dwell_samples,
                           background=background,
                           on_frame=on_frame)
        if on_point is not None:
            on_point(idx, len(galvo_path), obs)
        if obs is not None:
            targets.append(TestTarget(galvo=(gx, gy),
                                      observed_norm=(obs.x_norm, obs.y_norm),
                                      depth_m=depth_m))
    return targets
