"""Step 8: detection↔track assignment (Hungarian + greedy, gate, IoU)."""
from __future__ import annotations

import numpy as np

from events.schemas import DetectionEvent
from tracking import assignment
from tracking.assignment import _greedy_assign, assign, build_cost_matrix
from tracking.kalman_track import MODE_IOU_ONLY, MODE_KALMAN_NORM_DT, new_track


def _det(x, y, did=0, bbox=(0, 0, 10, 10)) -> DetectionEvent:
    return DetectionEvent(
        timestamp=0.0, sensor_id="gopro", detection_id=did,
        x_norm=x, y_norm=y, x_px=int(x * 320), y_px=int(y * 240),
        area_pixels=100.0, bbox=bbox,
        classifier_label="mosquito", classifier_confidence=0.9)


def _track(x, y, bbox=(0, 0, 10, 10)):
    return new_track(1, _det(x, y, bbox=bbox), MODE_KALMAN_NORM_DT, 0.0)


# ── Distance assignment ──────────────────────────────────────────────────

def test_assign_pairs_nearest():
    tracks = [_track(0.20, 0.20), _track(0.80, 0.80)]
    dets = [_det(0.81, 0.81, did=0), _det(0.21, 0.21, did=1)]
    matches = assign(tracks, dets, MODE_KALMAN_NORM_DT)
    assert sorted(matches) == [(0, 1), (1, 0)]


def test_assign_rejects_beyond_gate():
    tracks = [_track(0.2, 0.2)]
    dets = [_det(0.9, 0.9)]              # well beyond TRACKER_MAX_DISTANCE_NORM
    assert assign(tracks, dets, MODE_KALMAN_NORM_DT) == []


def test_assign_empty_inputs():
    assert assign([], [_det(0.5, 0.5)], MODE_KALMAN_NORM_DT) == []
    assert assign([_track(0.5, 0.5)], [], MODE_KALMAN_NORM_DT) == []


# ── IoU assignment (iou_only mode) ───────────────────────────────────────

def test_iou_matches_overlapping_boxes():
    tracks = [_track(0.5, 0.5, bbox=(100, 100, 20, 20))]
    dets = [_det(0.5, 0.5, bbox=(105, 105, 20, 20))]   # overlaps
    assert assign(tracks, dets, MODE_IOU_ONLY) == [(0, 0)]


def test_iou_rejects_disjoint_boxes():
    tracks = [_track(0.5, 0.5, bbox=(0, 0, 10, 10))]
    dets = [_det(0.5, 0.5, bbox=(200, 200, 10, 10))]   # no overlap
    assert assign(tracks, dets, MODE_IOU_ONLY) == []


# ── Greedy fallback parity ───────────────────────────────────────────────

def test_greedy_fallback_matches_hungarian(monkeypatch):
    tracks = [_track(0.2, 0.2), _track(0.6, 0.6)]
    dets = [_det(0.21, 0.21, did=0), _det(0.61, 0.61, did=1)]
    cost = build_cost_matrix(tracks, dets, MODE_KALMAN_NORM_DT)
    assert sorted(_greedy_assign(cost)) == [(0, 0), (1, 1)]
