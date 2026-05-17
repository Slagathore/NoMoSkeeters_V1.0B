"""Cross-sensor extrinsic — projects one sensor's normalized frame into
another's.

Step 9.5 needs two calibrations for the Kinect:

  1. Kinect→GALVO — a plain CoordinateMapper (calibration.py) fit against the
     Kinect's RGB/IR stream instead of the GoPro. Use run_dry_calibration with
     sensor_id="kinect_v2" plus the `stream` and `kinect_relative_pose` args.

  2. Kinect→GoPro extrinsic — THIS module. A planar homography between the two
     cameras' normalized frames so a Kinect detection can be expressed in
     GoPro-norm space (for fusion and the §16.4 comparison harness). It is fit
     from correspondences both cameras saw — cheaply, the laser dot fired
     during the Kinect calibration, which the GoPro also images.

Reference: BOOTSTRAP_AMENDMENTS_v0.2.1.md §8.10, §16.4, Step 9.5.
"""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from targeting.calibration import apply_homography

_log = logging.getLogger(__name__)

EXTRINSIC_SCHEMA_VERSION = 2


@dataclass
class CrossSensorExtrinsic:
    """Bidirectional normalized-frame homography between two sensors."""
    H_src_to_dst: np.ndarray
    H_dst_to_src: np.ndarray
    src_sensor: str
    dst_sensor: str
    n_points: int = 0
    residual_norm: float = 0.0
    scene: str = ""

    # ── Projection ───────────────────────────────────────────────────────

    def project(self, x: float, y: float) -> np.ndarray:
        """Project a src-sensor norm coord into dst-sensor norm space."""
        return apply_homography(self.H_src_to_dst, [(x, y)])[0]

    def project_inverse(self, x: float, y: float) -> np.ndarray:
        """Project a dst-sensor norm coord back into src-sensor norm space."""
        return apply_homography(self.H_dst_to_src, [(x, y)])[0]

    # ── Fitting ──────────────────────────────────────────────────────────

    @classmethod
    def fit(cls,
            src_pts: list[tuple[float, float]],
            dst_pts: list[tuple[float, float]],
            *,
            src_sensor: str,
            dst_sensor: str,
            scene: str = "") -> "CrossSensorExtrinsic":
        """Fit from shared correspondences — points both cameras observed."""
        src = np.asarray(src_pts, dtype=float).reshape(-1, 2)
        dst = np.asarray(dst_pts, dtype=float).reshape(-1, 2)
        if len(src) < 4 or len(src) != len(dst):
            raise ValueError(
                f"need ≥4 matched correspondences, got {len(src)}/{len(dst)}")

        H_src_to_dst, _ = cv2.findHomography(src, dst, method=0)
        if H_src_to_dst is None:
            raise ValueError("findHomography failed — degenerate points?")
        H_dst_to_src = np.linalg.inv(H_src_to_dst)

        pred = apply_homography(H_src_to_dst, src)
        residual = float(np.mean(np.linalg.norm(pred - dst, axis=1)))
        _log.info("extrinsic %s→%s fit: %d pts, residual_norm=%.4f",
                  src_sensor, dst_sensor, len(src), residual)
        return cls(H_src_to_dst=H_src_to_dst, H_dst_to_src=H_dst_to_src,
                   src_sensor=src_sensor, dst_sensor=dst_sensor,
                   n_points=len(src), residual_norm=residual, scene=scene)

    # ── Serialization ────────────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            "schema_version": EXTRINSIC_SCHEMA_VERSION,
            "type": "cross_sensor_extrinsic",
            "src_sensor": self.src_sensor,
            "dst_sensor": self.dst_sensor,
            "scene": self.scene,
            "n_points": self.n_points,
            "residual_norm": self.residual_norm,
            "H_src_to_dst": self.H_src_to_dst.tolist(),
            "H_dst_to_src": self.H_dst_to_src.tolist(),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CrossSensorExtrinsic":
        if d.get("schema_version") != EXTRINSIC_SCHEMA_VERSION:
            raise ValueError(
                f"extrinsic schema_version {d.get('schema_version')}, "
                f"expected {EXTRINSIC_SCHEMA_VERSION}")
        return cls(
            H_src_to_dst=np.asarray(d["H_src_to_dst"], dtype=float),
            H_dst_to_src=np.asarray(d["H_dst_to_src"], dtype=float),
            src_sensor=d["src_sensor"],
            dst_sensor=d["dst_sensor"],
            n_points=d.get("n_points", 0),
            residual_norm=d.get("residual_norm", 0.0),
            scene=d.get("scene", ""),
        )

    def save(self, path: Path | str) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2))
        _log.info("extrinsic saved: %s", path)
        return path

    @classmethod
    def load(cls, path: Path | str) -> "CrossSensorExtrinsic":
        return cls.from_dict(json.loads(Path(path).read_text()))
