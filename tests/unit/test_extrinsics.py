"""Step 9.5: Kinect→GALVO calibration extras + Kinect→GoPro extrinsic."""
from __future__ import annotations

import numpy as np
import pytest

from laser.transports.dry_run import DryRunTransport
from targeting.calibration import CoordinateMapper, apply_homography
from targeting.calibration_controller import run_dry_calibration
from targeting.extrinsics import CrossSensorExtrinsic
from targeting.patterns import halton_points


# ── Kinect→GALVO calibration (§8.10) ─────────────────────────────────────

def test_kinect_galvo_calibration_carries_stream_and_pose(tmp_path):
    pose = {"approx_origin_xyz_m": [0.5, 1.2, 0.0],
            "operator_note": "corner shelf"}
    run = run_dry_calibration(
        DryRunTransport(), pattern="grid", sensor_id="kinect_v2",
        out_dir=tmp_path, stream="rgb", kinect_relative_pose=pose)

    doc = run.mapper.to_dict()
    assert doc["sensor_id"] == "kinect_v2"
    assert doc["stream"] == "rgb"
    assert doc["kinect_relative_pose"]["operator_note"] == "corner shelf"

    reloaded = CoordinateMapper.load(run.json_path)
    assert reloaded.stream == "rgb"
    assert reloaded.kinect_relative_pose["approx_origin_xyz_m"] == [0.5, 1.2, 0.0]


# ── Kinect→GoPro extrinsic ───────────────────────────────────────────────

def _truth_extrinsic() -> np.ndarray:
    """A plausible kinect-norm → gopro-norm homography."""
    return np.array([
        [0.92, 0.04, 0.05],
        [-0.03, 0.95, 0.02],
        [0.02, 0.01, 1.00],
    ], dtype=float)


def _shared_points(noise: float = 0.0, seed: int = 3):
    rng = np.random.default_rng(seed)
    kinect = halton_points(25)
    H = _truth_extrinsic()
    gopro = apply_homography(H, kinect)
    if noise:
        gopro = gopro + rng.normal(0, noise, size=gopro.shape)
    return kinect, [tuple(p) for p in gopro], H


def test_extrinsic_fit_recovers_projection():
    kinect, gopro, _ = _shared_points(noise=0.0)
    ext = CrossSensorExtrinsic.fit(kinect, gopro,
                                   src_sensor="kinect_v2", dst_sensor="gopro")
    assert ext.n_points == 25
    assert ext.residual_norm < 1e-6
    # Projecting a Kinect detection lands on the GoPro point.
    proj = ext.project(*kinect[0])
    assert np.allclose(proj, gopro[0], atol=1e-6)


def test_extrinsic_project_inverse_round_trips():
    kinect, gopro, _ = _shared_points(noise=0.001)
    ext = CrossSensorExtrinsic.fit(kinect, gopro,
                                   src_sensor="kinect_v2", dst_sensor="gopro")
    fwd = ext.project(0.42, 0.58)
    back = ext.project_inverse(*fwd)
    assert np.allclose(back, [0.42, 0.58], atol=1e-9)


def test_extrinsic_rejects_too_few_points():
    with pytest.raises(ValueError):
        CrossSensorExtrinsic.fit([(0, 0), (1, 0), (0, 1)],
                                 [(0, 0), (1, 0), (0, 1)],
                                 src_sensor="kinect_v2", dst_sensor="gopro")


def test_extrinsic_json_roundtrip(tmp_path):
    kinect, gopro, _ = _shared_points(noise=0.002)
    ext = CrossSensorExtrinsic.fit(kinect, gopro, src_sensor="kinect_v2",
                                   dst_sensor="gopro", scene="garage")
    path = ext.save(tmp_path / "kinect_to_gopro" / "garage.json")
    assert path.exists()
    loaded = CrossSensorExtrinsic.load(path)
    assert loaded.src_sensor == "kinect_v2" and loaded.dst_sensor == "gopro"
    assert np.allclose(loaded.H_src_to_dst, ext.H_src_to_dst)
    assert np.allclose(loaded.project(0.3, 0.3), ext.project(0.3, 0.3))
