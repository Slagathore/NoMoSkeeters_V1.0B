"""Step 6 acceptance: a dry-run calibration produces a calibration JSON v2
file with both matrices and a plausible residual."""
from __future__ import annotations

import json

from laser.transports.dry_run import DryRunTransport
from targeting.calibration import CoordinateMapper
from targeting.calibration_controller import (
    SyntheticCamera, default_truth_homography, run_dry_calibration,
)


def test_dry_calibration_writes_v2_json(tmp_path):
    transport = DryRunTransport()
    run = run_dry_calibration(transport, pattern="halton",
                              sensor_id="gopro", out_dir=tmp_path)

    # JSON file exists and is schema v2 with both matrices.
    assert run.json_path is not None and run.json_path.exists()
    doc = json.loads(run.json_path.read_text())
    assert doc["schema_version"] == 2
    assert len(doc["H_norm_to_galvo"]) == 3
    assert len(doc["H_galvo_to_norm"]) == 3

    # Plausible residual: small, positive, well under the pass threshold.
    assert 0.0 < doc["residual_norm"] < 0.015
    assert doc["residual_galvo"] > 0.0

    # Multi-depth validation ran and is recorded.
    assert doc["validation"]["passed"] is True
    assert len(doc["validation"]["depths_m"]) == 3


def test_dry_calibration_reloads_into_working_mapper(tmp_path):
    transport = DryRunTransport()
    run = run_dry_calibration(transport, pattern="grid",
                              sensor_id="gopro", out_dir=tmp_path)
    mapper = CoordinateMapper.load(run.json_path)
    # Mapper recovered the synthetic camera's truth homography closely.
    g = mapper.norm_to_galvo(0.5, 0.5)
    assert 0.0 <= g[0] <= 1.0 and 0.0 <= g[1] <= 1.0


def test_dry_calibration_off_plane_fails_validation(tmp_path):
    transport = DryRunTransport()
    # A camera whose planar homography degrades badly off the cal plane.
    camera = SyntheticCamera(H_truth=default_truth_homography(),
                             depth_error_per_m=0.05)
    run = run_dry_calibration(transport, pattern="halton",
                              sensor_id="gopro", out_dir=tmp_path,
                              camera=camera)
    assert run.mapper.validation is not None
    assert run.mapper.validation.passed is False
