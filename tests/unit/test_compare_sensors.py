"""Step 9.7: sensor comparison harness."""
from __future__ import annotations

import json

from targeting.extrinsics import CrossSensorExtrinsic
from targeting.calibration import apply_homography
from targeting.patterns import halton_points
from tools.compare_sensors import (
    DetectionRecord, compare, load_detections, main, write_csv,
)


def _rec(ts, sid, x, y):
    return DetectionRecord(ts=ts, sensor_id=sid, x_norm=x, y_norm=y)


# ── compare() binning ────────────────────────────────────────────────────

def test_compare_bins_both_gopro_only_kinect_only():
    dets = [
        _rec(1.00, "gopro", 0.50, 0.50),       # matched by the kinect_ir below
        _rec(1.02, "kinect_ir", 0.50, 0.50),   # → Both
        _rec(5.00, "gopro", 0.10, 0.10),       # → GoPro only
        _rec(9.00, "kinect_rgb", 0.90, 0.90),  # → Kinect only
    ]
    report = compare(dets)
    assert report.raw_counts == {"gopro": 2, "kinect_ir": 1, "kinect_rgb": 1}
    assert report.both_per_stream == {"kinect_ir": 1}
    assert report.gopro_only == 1
    assert report.kinect_only == 1
    assert report.n_gopro == 2


def test_compare_respects_time_window():
    dets = [
        _rec(1.0, "gopro", 0.5, 0.5),
        _rec(2.0, "kinect_ir", 0.5, 0.5),      # 1000ms apart > 100ms window
    ]
    report = compare(dets, assoc_dt_ms=100.0)
    assert report.both_per_stream == {}
    assert report.gopro_only == 1
    assert report.kinect_only == 1


def test_compare_respects_distance_window():
    dets = [
        _rec(1.0, "gopro", 0.10, 0.10),
        _rec(1.0, "kinect_ir", 0.90, 0.90),    # far apart
    ]
    report = compare(dets, assoc_dist_norm=0.05)
    assert report.gopro_only == 1
    assert report.kinect_only == 1


def test_compare_applies_extrinsic_projection():
    # Kinect coords offset from GoPro; only the extrinsic makes them match.
    kinect_pts = halton_points(25)
    H = __import__("numpy").array([[1.0, 0.0, 0.2],
                                   [0.0, 1.0, 0.15],
                                   [0.0, 0.0, 1.0]])
    gopro_pts = apply_homography(H, kinect_pts)
    ext = CrossSensorExtrinsic.fit(kinect_pts, [tuple(p) for p in gopro_pts],
                                   src_sensor="kinect_v2", dst_sensor="gopro")
    dets = [
        _rec(1.0, "gopro", float(gopro_pts[0][0]), float(gopro_pts[0][1])),
        _rec(1.0, "kinect_rgb", kinect_pts[0][0], kinect_pts[0][1]),
    ]
    assert compare(dets).both_per_stream == {}           # raw coords miss
    assert compare(dets, extrinsic=ext).both_per_stream == {"kinect_rgb": 1}


# ── JSONL loading ────────────────────────────────────────────────────────

def test_load_detections_filters_event_type(tmp_path):
    path = tmp_path / "session.jsonl"
    path.write_text("\n".join([
        json.dumps({"ts": 0.0, "type": "session_start"}),
        json.dumps({"ts": 1.0, "type": "detection", "sensor_id": "gopro",
                    "x_norm": 0.5, "y_norm": 0.5}),
        json.dumps({"ts": 1.1, "type": "heartbeat_status"}),
        json.dumps({"ts": 2.0, "type": "detection", "sensor_id": "kinect_ir",
                    "x_norm": 0.3, "y_norm": 0.3}),
        "",                                              # blank line tolerated
    ]))
    dets = load_detections(path)
    assert len(dets) == 2
    assert dets[0].sensor_id == "gopro"
    assert dets[1].is_kinect


# ── CLI end-to-end ───────────────────────────────────────────────────────

def test_main_runs_and_reports(tmp_path, capsys):
    path = tmp_path / "rec.jsonl"
    path.write_text("\n".join(
        json.dumps({"ts": float(i), "type": "detection",
                    "sensor_id": "gopro" if i % 2 else "kinect_ir",
                    "x_norm": 0.5, "y_norm": 0.5})
        for i in range(6)))
    rc = main([str(path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "SENSOR COMPARISON" in out
    assert "Detection categorization" in out


def test_write_csv(tmp_path):
    dets = [_rec(1.0, "gopro", 0.5, 0.5), _rec(2.0, "kinect_ir", 0.3, 0.3)]
    csv = write_csv(dets, tmp_path / "out.csv")
    lines = csv.read_text().splitlines()
    assert lines[0] == "ts,sensor_id,x_norm,y_norm"
    assert len(lines) == 3
