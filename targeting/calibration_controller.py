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


def _capture_dot(cube, camera, detector: LaserDotDetector,
                 gx_dac: int, gy_dac: int, rgb: tuple[int, int, int],
                 *, settle_frames: int, max_attempts: int,
                 dwell_samples: int,
                 on_frame: Optional[Callable] = None) -> Optional[DotObservation]:
    """Hold the laser on one galvo coord, detect the dot in the camera.

    Re-streams the dwell each attempt because the cube scans its ringbuffer
    out continuously — the dot is only lit while samples remain queued.
    `on_frame`, if given, is called with every camera frame (rgb) — used by
    the step11 script's --view option to show what the camera sees.
    """
    r, g, b = rgb
    dwell = [LaserPoint(x=gx_dac, y=gy_dac, r=r, g=g, b=b)] * dwell_samples
    for _ in range(max_attempts):
        cube.send_frame(dwell, frame_num=0)
        for _ in range(settle_frames):        # drop stale / in-flight frames
            stale = camera.read()
            if on_frame is not None and stale is not None and stale.rgb is not None:
                on_frame(stale.rgb)
        frame = camera.read()
        if frame is not None and frame.rgb is not None:
            if on_frame is not None:
                on_frame(frame.rgb)
            obs = detector.detect(frame.rgb)
            if obs is not None:
                return obs
        time.sleep(0.01)
    return None


def run_live_calibration(cube, camera, *,
                         pattern: Optional[str] = None,
                         sensor_id: str = "gopro",
                         scene: str = "live",
                         mount_tilt_config: str = "bench",
                         detector: Optional[LaserDotDetector] = None,
                         out_dir: Optional[Path] = None,
                         stream: str = "",
                         kinect_relative_pose: Optional[dict] = None,
                         settle_frames: int = 2,
                         max_attempts: int = 8,
                         dwell_samples: int = 3000,
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
    rgb = (settings.CALIBRATION_LASER_R, settings.CALIBRATION_LASER_G,
           settings.CALIBRATION_LASER_B)
    galvo_path = calibration_pattern_path(pattern)

    galvo_pts: list[Point] = []
    observed: list[Point] = []
    for idx, (gx, gy) in enumerate(galvo_path):
        obs = _capture_dot(cube, camera, detector,
                           galvo_norm_to_dac(gx), galvo_norm_to_dac(gy), rgb,
                           settle_frames=settle_frames,
                           max_attempts=max_attempts,
                           dwell_samples=dwell_samples,
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
                          settle_frames: int = 2,
                          max_attempts: int = 8,
                          dwell_samples: int = 3000,
                          on_point: Optional[PointCallback] = None,
                          on_frame: Optional[Callable] = None) -> list[TestTarget]:
    """Sweep the pattern once at a known depth; return TestTargets for
    validate_multi_depth(). The operator places the target plane at depth_m
    before calling this."""
    pattern = pattern or settings.CALIBRATION_PATTERN
    detector = detector or LaserDotDetector()
    rgb = (settings.CALIBRATION_LASER_R, settings.CALIBRATION_LASER_G,
           settings.CALIBRATION_LASER_B)
    galvo_path = calibration_pattern_path(pattern)

    targets: list[TestTarget] = []
    for idx, (gx, gy) in enumerate(galvo_path):
        obs = _capture_dot(cube, camera, detector,
                           galvo_norm_to_dac(gx), galvo_norm_to_dac(gy), rgb,
                           settle_frames=settle_frames,
                           max_attempts=max_attempts,
                           dwell_samples=dwell_samples,
                           on_frame=on_frame)
        if on_point is not None:
            on_point(idx, len(galvo_path), obs)
        if obs is not None:
            targets.append(TestTarget(galvo=(gx, gy),
                                      observed_norm=(obs.x_norm, obs.y_norm),
                                      depth_m=depth_m))
    return targets
