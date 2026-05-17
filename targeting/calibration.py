"""CoordinateMapper — the NORM↔GALVO homography learned by calibration.

Calibration observes laser dots: fire a known galvo coord, the camera detects
a sensor pixel. After normalization that is GALVO→NORM — the inverse of what
the targeting pipeline needs. We fit one direction and invert ONCE here, then
store BOTH matrices so the hot path never inverts (§8.5).

Reference: BOOTSTRAP.md §8.4-§8.5, BOOTSTRAP_AMENDMENTS.md §8.5-§8.6.
"""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from config import settings
from laser.types import COORD_MAX

_log = logging.getLogger(__name__)

CALIBRATION_SCHEMA_VERSION = 2


# ── Homography helpers ───────────────────────────────────────────────────

def apply_homography(H: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply a 3×3 homography to an (N,2) array of points; returns (N,2)."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    homog = np.hstack([pts, np.ones((len(pts), 1))])
    out = (H @ homog.T).T
    return out[:, :2] / out[:, 2:3]


# ── Multi-depth validation (§8.6) ────────────────────────────────────────

@dataclass(frozen=True)
class TestTarget:
    """A validation observation: a commanded galvo coord, the sensor-norm
    coord it was detected at, and the depth it was captured at."""
    __test__ = False   # not a pytest test class despite the name (§8.6)
    galvo: tuple[float, float]
    observed_norm: tuple[float, float]
    depth_m: float


@dataclass(frozen=True)
class ValidationResult:
    per_depth_residual_norm: dict[float, float]
    max_residual_norm: float
    passed: bool

    def to_dict(self) -> dict:
        depths = sorted(self.per_depth_residual_norm)
        return {
            "depths_m": depths,
            "residuals_norm": [self.per_depth_residual_norm[d] for d in depths],
            "passed": self.passed,
        }


def group_by_depth_bin(targets: list[TestTarget],
                       bin_size_m: float = 0.5) -> dict[float, list[TestTarget]]:
    """Bucket validation targets by depth, rounded to bin_size_m."""
    bins: dict[float, list[TestTarget]] = {}
    for t in targets:
        key = round(t.depth_m / bin_size_m) * bin_size_m
        bins.setdefault(key, []).append(t)
    return bins


def validate_multi_depth(mapper: "CoordinateMapper",
                         test_targets: list[TestTarget]) -> ValidationResult:
    """Validate a fitted mapper at multiple depths inside the operating
    volume. A planar homography that fails here cannot cover the volume —
    escalate to a depth-aware mapper (v0.3)."""
    by_depth = group_by_depth_bin(test_targets, bin_size_m=0.5)
    residuals: dict[float, float] = {}
    for depth_m, group in by_depth.items():
        errors = [
            float(np.linalg.norm(
                mapper.galvo_to_norm(*t.galvo) - np.asarray(t.observed_norm)))
            for t in group
        ]
        residuals[depth_m] = float(np.mean(errors)) if errors else 0.0
    max_residual = max(residuals.values()) if residuals else 0.0
    return ValidationResult(
        per_depth_residual_norm=residuals,
        max_residual_norm=max_residual,
        passed=max_residual < settings.CALIBRATION_MAX_RESIDUAL_NORM,
    )


# ── CoordinateMapper ─────────────────────────────────────────────────────

@dataclass
class CoordinateMapper:
    """Bidirectional NORM↔GALVO mapper. Both homographies precomputed.

    Build with CoordinateMapper.fit() from calibration correspondences, or
    CoordinateMapper.load() from a saved calibration JSON v2.
    """
    H_norm_to_galvo: np.ndarray
    H_galvo_to_norm: np.ndarray
    sensor_id: str = ""
    pattern: str = ""
    n_points: int = 0
    residual_norm: float = 0.0
    residual_galvo: float = 0.0
    mount_tilt_config: str = ""
    scene: str = ""
    validation: Optional[ValidationResult] = None
    # Kinect-only extras (§8.10): which stream was calibrated, and an
    # informational pose hint for re-placing a moved Kinect.
    stream: str = ""
    kinect_relative_pose: Optional[dict] = None

    # ── Hot-path conversions ─────────────────────────────────────────────

    def norm_to_galvo(self, x: float, y: float) -> np.ndarray:
        """Map a sensor-NORM coord to galvo-NORM space [0,1]². Hot path —
        no inversion, just one matrix-vector apply."""
        return apply_homography(self.H_norm_to_galvo, [(x, y)])[0]

    def galvo_to_norm(self, x: float, y: float) -> np.ndarray:
        """Map a galvo-NORM coord back to sensor-NORM space [0,1]²."""
        return apply_homography(self.H_galvo_to_norm, [(x, y)])[0]

    # ── Fitting ──────────────────────────────────────────────────────────

    @classmethod
    def fit(cls,
            galvo_pts: list[tuple[float, float]],
            norm_pts: list[tuple[float, float]],
            *,
            sensor_id: str = "",
            pattern: str = "",
            mount_tilt_config: str = "",
            scene: str = "") -> "CoordinateMapper":
        """Fit from observed correspondences. Option A (§8.5): fit the
        physically-observed GALVO→NORM direction, invert once for the inverse.
        cv2.findHomography needs ≥ 4 non-degenerate correspondences.
        """
        galvo = np.asarray(galvo_pts, dtype=float).reshape(-1, 2)
        norm = np.asarray(norm_pts, dtype=float).reshape(-1, 2)
        if len(galvo) < 4 or len(galvo) != len(norm):
            raise ValueError(
                f"need ≥4 matched correspondences, got {len(galvo)}/{len(norm)}")

        H_galvo_to_norm, _ = cv2.findHomography(galvo, norm, method=0)
        if H_galvo_to_norm is None:
            raise ValueError("findHomography failed — degenerate points?")
        H_norm_to_galvo = np.linalg.inv(H_galvo_to_norm)

        mapper = cls(
            H_norm_to_galvo=H_norm_to_galvo,
            H_galvo_to_norm=H_galvo_to_norm,
            sensor_id=sensor_id, pattern=pattern, n_points=len(galvo),
            mount_tilt_config=mount_tilt_config, scene=scene,
        )
        mapper._recompute_residuals(galvo, norm)
        return mapper

    def _recompute_residuals(self, galvo: np.ndarray, norm: np.ndarray) -> None:
        """Mean reprojection error, both directions. residual_norm is in
        normalized units; residual_galvo is scaled to 12-bit DAC units to
        match the calibration-JSON convention."""
        pred_norm = apply_homography(self.H_galvo_to_norm, galvo)
        pred_galvo = apply_homography(self.H_norm_to_galvo, norm)
        self.residual_norm = float(
            np.mean(np.linalg.norm(pred_norm - norm, axis=1)))
        self.residual_galvo = float(
            np.mean(np.linalg.norm(pred_galvo - galvo, axis=1)) * COORD_MAX)

    # ── Serialization — calibration JSON v2 (§8.5) ───────────────────────

    def to_dict(self) -> dict:
        d = {
            "schema_version": CALIBRATION_SCHEMA_VERSION,
            "sensor_id": self.sensor_id,
            "mount_tilt_config": self.mount_tilt_config,
            "scene": self.scene,
            "n_points": self.n_points,
            "pattern": self.pattern,
            "H_norm_to_galvo": self.H_norm_to_galvo.tolist(),
            "H_galvo_to_norm": self.H_galvo_to_norm.tolist(),
            "residual_norm": self.residual_norm,
            "residual_galvo": self.residual_galvo,
        }
        if self.validation is not None:
            d["validation"] = self.validation.to_dict()
        if self.stream:
            d["stream"] = self.stream
        if self.kinect_relative_pose is not None:
            d["kinect_relative_pose"] = self.kinect_relative_pose
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "CoordinateMapper":
        ver = d.get("schema_version")
        if ver != CALIBRATION_SCHEMA_VERSION:
            raise ValueError(
                f"calibration schema_version {ver}, expected "
                f"{CALIBRATION_SCHEMA_VERSION}")
        validation = None
        if "validation" in d:
            v = d["validation"]
            per_depth = dict(zip(v["depths_m"], v["residuals_norm"]))
            validation = ValidationResult(
                per_depth_residual_norm=per_depth,
                max_residual_norm=max(v["residuals_norm"]) if v["residuals_norm"] else 0.0,
                passed=v["passed"],
            )
        return cls(
            H_norm_to_galvo=np.asarray(d["H_norm_to_galvo"], dtype=float),
            H_galvo_to_norm=np.asarray(d["H_galvo_to_norm"], dtype=float),
            sensor_id=d.get("sensor_id", ""),
            pattern=d.get("pattern", ""),
            n_points=d.get("n_points", 0),
            residual_norm=d.get("residual_norm", 0.0),
            residual_galvo=d.get("residual_galvo", 0.0),
            mount_tilt_config=d.get("mount_tilt_config", ""),
            scene=d.get("scene", ""),
            validation=validation,
            stream=d.get("stream", ""),
            kinect_relative_pose=d.get("kinect_relative_pose"),
        )

    def save(self, path: Path | str) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2))
        _log.info("calibration saved: %s (residual_norm=%.4f)",
                  path, self.residual_norm)
        return path

    @classmethod
    def load(cls, path: Path | str) -> "CoordinateMapper":
        return cls.from_dict(json.loads(Path(path).read_text()))
