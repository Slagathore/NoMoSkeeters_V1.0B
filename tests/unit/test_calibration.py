"""Step 6: CoordinateMapper fit, both-matrix storage, JSON v2, validation."""
from __future__ import annotations

import numpy as np
import pytest

from targeting.calibration import (
    CoordinateMapper, TestTarget, ValidationResult,
    apply_homography, group_by_depth_bin, validate_multi_depth,
)
from targeting.calibration_controller import default_truth_homography
from targeting.patterns import halton_points


def _correspondences(noise: float = 0.0, seed: int = 1):
    """Build galvo↔norm correspondences from a known truth homography."""
    rng = np.random.default_rng(seed)
    galvo = halton_points(25)
    H = default_truth_homography()
    norm = apply_homography(H, galvo)
    if noise:
        norm = norm + rng.normal(0, noise, size=norm.shape)
    return galvo, [tuple(p) for p in norm], H


# ── Fitting ──────────────────────────────────────────────────────────────

def test_fit_recovers_known_homography_noise_free():
    galvo, norm, _ = _correspondences(noise=0.0)
    mapper = CoordinateMapper.fit(galvo, norm, sensor_id="gopro",
                                  pattern="halton")
    assert mapper.n_points == 25
    assert mapper.residual_norm < 1e-6        # exact fit, no noise


def test_fit_residual_tracks_noise():
    galvo, norm, _ = _correspondences(noise=0.003)
    mapper = CoordinateMapper.fit(galvo, norm)
    # Residual should be on the order of the injected noise.
    assert 0.0005 < mapper.residual_norm < 0.02
    assert mapper.residual_galvo > 0.0


def test_fit_rejects_too_few_points():
    with pytest.raises(ValueError):
        CoordinateMapper.fit([(0, 0), (1, 0), (0, 1)],
                             [(0, 0), (1, 0), (0, 1)])


# ── Both matrices / directionality (§8.5) ────────────────────────────────

def test_both_matrices_are_mutual_inverses():
    galvo, norm, _ = _correspondences()
    mapper = CoordinateMapper.fit(galvo, norm)
    identity = mapper.H_norm_to_galvo @ mapper.H_galvo_to_norm
    identity = identity / identity[2, 2]
    assert np.allclose(identity, np.eye(3), atol=1e-9)


def test_norm_to_galvo_round_trips():
    galvo, norm, _ = _correspondences()
    mapper = CoordinateMapper.fit(galvo, norm)
    g = mapper.norm_to_galvo(0.4, 0.55)
    back = mapper.galvo_to_norm(*g)
    assert np.allclose(back, [0.4, 0.55], atol=1e-9)


# ── JSON v2 serialization ────────────────────────────────────────────────

def test_calibration_json_roundtrip(tmp_path):
    galvo, norm, _ = _correspondences(noise=0.002)
    mapper = CoordinateMapper.fit(galvo, norm, sensor_id="gopro",
                                  pattern="halton", scene="garage")
    path = mapper.save(tmp_path / "gopro" / "synthetic_garage.json")
    assert path.exists()

    loaded = CoordinateMapper.load(path)
    assert loaded.sensor_id == "gopro"
    assert loaded.pattern == "halton"
    assert loaded.n_points == 25
    assert np.allclose(loaded.H_norm_to_galvo, mapper.H_norm_to_galvo)
    assert np.allclose(loaded.H_galvo_to_norm, mapper.H_galvo_to_norm)


def test_to_dict_has_v2_schema_and_both_matrices():
    galvo, norm, _ = _correspondences()
    d = CoordinateMapper.fit(galvo, norm).to_dict()
    assert d["schema_version"] == 2
    assert "H_norm_to_galvo" in d and "H_galvo_to_norm" in d
    assert np.array(d["H_norm_to_galvo"]).shape == (3, 3)


def test_from_dict_rejects_wrong_schema():
    with pytest.raises(ValueError):
        CoordinateMapper.from_dict({"schema_version": 1})


# ── Multi-depth validation (§8.6) ────────────────────────────────────────

def test_group_by_depth_bin():
    targets = [
        TestTarget((0, 0), (0, 0), 1.0),
        TestTarget((0, 0), (0, 0), 1.1),
        TestTarget((0, 0), (0, 0), 2.5),
    ]
    bins = group_by_depth_bin(targets, bin_size_m=0.5)
    assert sorted(bins) == [1.0, 2.5]
    assert len(bins[1.0]) == 2


def test_validate_multi_depth_passes_clean_data():
    galvo, norm, H = _correspondences(noise=0.0)
    mapper = CoordinateMapper.fit(galvo, norm)
    targets = [TestTarget(g, tuple(apply_homography(H, [g])[0]), depth)
               for depth in (1.0, 2.5, 4.0) for g in galvo]
    result = validate_multi_depth(mapper, targets)
    assert isinstance(result, ValidationResult)
    assert result.passed
    assert result.max_residual_norm < 1e-6


def test_validate_multi_depth_fails_when_off_plane():
    galvo, norm, H = _correspondences(noise=0.0)
    mapper = CoordinateMapper.fit(galvo, norm)
    # Inject large per-depth error → planar homography insufficient.
    targets = [TestTarget(g, (0.5, 0.5), depth)
               for depth in (1.0, 2.5, 4.0) for g in galvo]
    result = validate_multi_depth(mapper, targets)
    assert not result.passed
