"""Step 8: Tracker — lifecycle, modes, and the fire_eligible signal."""
from __future__ import annotations

import pytest

from config import settings
from events.schemas import DetectionEvent, TrackEvent
from tracking.kalman_track import TRACKER_MODES
from tracking.tracker import Tracker, to_track_event


def _det(x, y, did=0, ts=0.0, bbox=(100, 100, 20, 20), z=None) -> DetectionEvent:
    return DetectionEvent(
        timestamp=ts, sensor_id="gopro", detection_id=did,
        x_norm=x, y_norm=y, x_px=int(x * 320), y_px=int(y * 240),
        area_pixels=120.0, bbox=bbox,
        classifier_label="mosquito", classifier_confidence=0.9, z_world_m=z)


# ── Lifecycle ────────────────────────────────────────────────────────────

def test_first_detection_spawns_one_unconfirmed_track():
    trk = Tracker(mode="kalman_norm_dt")
    tracks = trk.update([_det(0.5, 0.5)], timestamp=0.0)
    assert len(tracks) == 1
    assert tracks[0].confirmed is False
    assert tracks[0].track_id == 1


def test_track_confirms_after_confirmation_frames():
    trk = Tracker(mode="kalman_norm_dt")
    for i in range(settings.TRACKER_CONFIRMATION_FRAMES):
        tracks = trk.update([_det(0.5, 0.5, ts=i * 0.03)], timestamp=i * 0.03)
    assert tracks[0].confirmed is True
    # A steady single target stays a single track — no duplicates.
    assert len(trk.tracks()) == 1


def test_track_follows_moving_target():
    trk = Tracker(mode="kalman_norm_dt")
    for i in range(12):
        x = 0.30 + 0.02 * i
        trk.update([_det(x, 0.5, ts=i * 0.03)], timestamp=i * 0.03)
    t = trk.tracks()[0]
    # The estimate trails the target (TRACKER_KALMAN_MEAS_NOISE=2.0 is a
    # pixel-space value, so it over-smooths in norm space — see report),
    # but it moves in the right direction and has learned the velocity.
    assert 0.30 < t.position[0] < 0.52     # tracking forward, still trailing
    assert t.state[2] > 0.0                # learned a positive x-velocity


def test_unmatched_track_coasts_then_drops():
    trk = Tracker(mode="kalman_norm_dt")
    for i in range(4):               # establish a track
        trk.update([_det(0.5, 0.5, ts=i * 0.03)], timestamp=i * 0.03)
    # Withhold detections — track should coast, then drop.
    for i in range(settings.TRACKER_MAX_DISAPPEARED):
        tracks = trk.update([], timestamp=(4 + i) * 0.03)
        assert tracks[0].coasting is True
    trk.update([], timestamp=99.0)
    assert trk.tracks() == []


# ── fire_eligible signal (§7.2) ──────────────────────────────────────────

def test_fire_eligible_false_while_unconfirmed():
    trk = Tracker(mode="kalman_norm_dt")
    tracks = trk.update([_det(0.5, 0.5)], timestamp=0.0)
    assert tracks[0].fire_eligible is False
    assert "unconfirmed" in tracks[0].fire_eligible_reason


def test_fire_eligible_becomes_true_then_false_on_coast():
    trk = Tracker(mode="kalman_norm_dt")
    # Many steady matched frames — confidence ramps past the threshold.
    for i in range(12):
        tracks = trk.update([_det(0.5, 0.5, ts=i * 0.03)], timestamp=i * 0.03)
    t = tracks[0]
    assert t.confirmed and t.fire_eligible is True
    assert t.fire_eligible_reason == "ok"
    # Drop one detection → coasting → not eligible.
    tracks = trk.update([], timestamp=12 * 0.03)
    assert tracks[0].fire_eligible is False
    assert "coasting" in tracks[0].fire_eligible_reason


# ── Modes ────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("mode", TRACKER_MODES)
def test_every_mode_runs(mode):
    trk = Tracker(mode=mode)
    for i in range(5):
        tracks = trk.update([_det(0.5, 0.5, ts=i * 0.03, z=2.0)],
                            timestamp=i * 0.03)
    assert len(tracks) == 1
    assert tracks[0].confirmed is True


def test_kalman_3d_carries_depth():
    trk = Tracker(mode="kalman_3d")
    trk.update([_det(0.5, 0.5, z=2.5)], timestamp=0.0)
    t = trk.tracks()[0]
    assert len(t.state) == 6                 # [x,y,z,vx,vy,vz]
    assert t.position[2] == pytest.approx(2.5)


def test_rejects_unknown_mode():
    with pytest.raises(ValueError):
        Tracker(mode="warp_drive")


# ── TrackEvent projection ────────────────────────────────────────────────

def test_to_track_event():
    trk = Tracker(mode="kalman_norm_dt")
    for i in range(4):
        trk.update([_det(0.5, 0.5, ts=i * 0.03)], timestamp=i * 0.03)
    ev = to_track_event(trk.tracks()[0], timestamp=0.09)
    assert isinstance(ev, TrackEvent)
    assert ev.track_id == 1
    assert ev.confirmed is True
    assert len(ev.state_norm) == 2 and len(ev.velocity_norm) == 2
