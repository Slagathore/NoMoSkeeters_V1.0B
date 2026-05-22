"""§5.7 — cross-sensor track-level fusion."""
from __future__ import annotations

import numpy as np

from events.schemas import DetectionEvent
from tracking.cross_sensor_fusion import (CrossSensorFusion, FusedTrack,
                                          SensorLocalState, _passthrough)
from collections import deque


def _det(sensor_id: str, t: float, x: float, y: float,
         det_id: int = 0) -> DetectionEvent:
    return DetectionEvent(
        timestamp=t, sensor_id=sensor_id, detection_id=det_id,
        x_norm=x, y_norm=y, x_px=int(x * 1920), y_px=int(y * 1080),
        area_pixels=12.0, bbox=(0, 0, 4, 4),
        classifier_label="mosquito", classifier_confidence=0.9)


# ── SensorLocalState ─────────────────────────────────────────────────────

def test_project_to_extrapolates_with_velocity():
    s = SensorLocalState("gopro", deque([(0.0, 0.10, 0.20),
                                         (0.1, 0.20, 0.20),
                                         (0.2, 0.30, 0.20)], maxlen=8))
    s.update_velocity(min_samples=3)
    assert s.vx_norm_per_s == 0.0 or abs(s.vx_norm_per_s - 1.0) < 1e-6
    # ~1.0 norm/s in x → +0.05 over the next 50 ms.
    x, y = s.project_to(0.25)
    assert abs(x - 0.35) < 1e-6
    assert abs(y - 0.20) < 1e-6


def test_project_to_empty_history_is_nan():
    s = SensorLocalState("gopro", deque(maxlen=8))
    x, y = s.project_to(1.0)
    assert np.isnan(x) and np.isnan(y)


def test_update_velocity_ignores_degenerate_window():
    s = SensorLocalState("gopro", deque([(5.0, 0.1, 0.1)] * 4, maxlen=8))
    s.update_velocity(min_samples=3)
    assert s.vx_norm_per_s == 0.0 and s.vy_norm_per_s == 0.0


# ── ingest_detection — per-sensor mini-tracker ───────────────────────────

def test_ingest_groups_nearby_detections_into_one_subtrack():
    fus = CrossSensorFusion()
    for i in range(4):
        fus.ingest_detection(_det("gopro", 0.02 * i, 0.50 + 0.001 * i, 0.50))
    assert fus.sub_track_count() == 1


def test_ingest_splits_far_detections_into_separate_subtracks():
    fus = CrossSensorFusion()
    fus.ingest_detection(_det("gopro", 0.00, 0.20, 0.20))
    fus.ingest_detection(_det("gopro", 0.01, 0.80, 0.80))
    assert fus.sub_track_count() == 2


def test_ingest_prunes_stale_subtracks():
    fus = CrossSensorFusion(projection_max_dt_ms=100.0)
    fus.ingest_detection(_det("gopro", 0.0, 0.30, 0.30))
    # 0.5 s later, far beyond the 100 ms window — the old sub-track is pruned.
    fus.ingest_detection(_det("gopro", 0.5, 0.31, 0.31))
    assert fus.sub_track_count() == 1


def test_ingest_history_is_bounded():
    fus = CrossSensorFusion(history_length=4)
    for i in range(10):
        fus.ingest_detection(_det("gopro", 0.005 * i, 0.5, 0.5))
    state = next(iter(fus._sensor_states.values()))
    assert len(state.history) == 4


# ── fuse ─────────────────────────────────────────────────────────────────

def test_fuse_single_sensor_passes_through():
    fus = CrossSensorFusion()
    for i in range(3):
        fus.ingest_detection(_det("gopro", 0.02 * i, 0.5, 0.5))
    tracks = fus.fuse()
    assert len(tracks) == 1
    assert tracks[0].contributing_sensors == ["gopro"]
    assert not tracks[0].fused


def test_fuse_associates_two_sensors_on_same_object():
    fus = CrossSensorFusion()
    for i in range(3):
        t = 0.02 * i
        fus.ingest_detection(_det("gopro", t, 0.50, 0.50))
        fus.ingest_detection(_det("kinect_v2", t, 0.51, 0.50))
    tracks = fus.fuse()
    assert len(tracks) == 1
    track = tracks[0]
    assert track.fused
    assert set(track.contributing_sensors) == {"gopro", "kinect_v2"}
    assert track.assoc_distance_norm < 0.05
    assert abs(track.position_norm[0] - 0.505) < 1e-3


def test_fuse_keeps_distinct_objects_solo():
    fus = CrossSensorFusion()
    for i in range(3):
        t = 0.02 * i
        fus.ingest_detection(_det("gopro", t, 0.20, 0.20))
        fus.ingest_detection(_det("kinect_v2", t, 0.80, 0.80))
    tracks = fus.fuse()
    assert len(tracks) == 2
    assert all(not tr.fused for tr in tracks)


def test_fuse_default_t_ref_is_latest_timestamp():
    fus = CrossSensorFusion()
    fus.ingest_detection(_det("gopro", 0.10, 0.5, 0.5))
    fus.ingest_detection(_det("kinect_v2", 0.14, 0.5, 0.5))
    tracks = fus.fuse()
    assert tracks[0].t_ref == 0.14


# ── extrinsic remap ──────────────────────────────────────────────────────

class _FakeExtrinsic:
    """A fixed +0.10/-0.05 norm shift, kinect_v2 → gopro."""
    src_sensor = "kinect_v2"
    dst_sensor = "gopro"

    def project(self, x, y):
        return np.array([x + 0.10, y - 0.05])


def test_extrinsic_remaps_src_sensor_into_shared_frame():
    fus = CrossSensorFusion(extrinsic=_FakeExtrinsic())  # type: ignore[arg-type]
    for i in range(3):
        t = 0.02 * i
        fus.ingest_detection(_det("gopro", t, 0.50, 0.50))
        # Kinect sees the object 0.10 left / 0.05 up in its own frame; the
        # extrinsic shift lands it on the GoPro point.
        fus.ingest_detection(_det("kinect_v2", t, 0.40, 0.55))
    tracks = fus.fuse()
    assert len(tracks) == 1
    assert tracks[0].fused
    assert tracks[0].assoc_distance_norm < 1e-6


# ── helpers ──────────────────────────────────────────────────────────────

def test_passthrough_emits_one_track_per_subtrack():
    states = {
        ("gopro", 0): SensorLocalState("gopro", deque([(0.0, 0.1, 0.1)])),
        ("gopro", 1): SensorLocalState("gopro", deque([(0.0, 0.9, 0.9)])),
        ("gopro", 2): SensorLocalState("gopro", deque(maxlen=4)),  # empty
    }
    tracks = _passthrough(states, t_ref=0.0)
    assert len(tracks) == 2
    assert all(isinstance(tr, FusedTrack) for tr in tracks)


def test_clear_drops_subtracks():
    fus = CrossSensorFusion()
    fus.ingest_detection(_det("gopro", 0.0, 0.5, 0.5))
    fus.clear()
    assert fus.sub_track_count() == 0
    assert fus.fuse() == []
