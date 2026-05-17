"""Step 10 acceptance: against DryRunTransport, a target generates a logged
ShotPattern with correct sample count and centered geometry."""
from __future__ import annotations

from laser.laser_manager import LaserManager
from laser.shot_patterns import DotRepeat, MicroCircle
from laser.transports.dry_run import DryRunTransport
from targeting.calibration import CoordinateMapper
from targeting.calibration_controller import (
    default_truth_homography, SyntheticCamera, run_dry_calibration,
)


def test_shoot_streams_pattern_and_logs():
    events: list[dict] = []
    transport = DryRunTransport()
    transport.connect()
    mgr = LaserManager(transport, pattern=MicroCircle(radius_galvo=30),
                       event_sink=events.append)

    cmd = mgr.shoot(0.5, 0.5, track_id=7)

    # Centered geometry: norm 0.5 → DAC 2048.
    assert cmd.target_x_galvo == 2048 and cmd.target_y_galvo == 2048
    assert cmd.pattern_id == "micro_circle"
    assert cmd.track_id == 7

    # Correct sample count: 80 ms × 30 kpps = 2400 samples.
    assert len(events) == 1
    assert events[0]["n_samples"] == 2400
    assert events[0]["sent"] is True
    # 2400 samples / 140 per packet → 18 UDP packets streamed.
    assert transport.packet_count == 18


def test_shoot_applies_lead_aim():
    transport = DryRunTransport()
    transport.connect()
    # 100 ms of software lag, target moving +0.2 norm/s in x.
    mgr = LaserManager(transport, pattern=DotRepeat(), software_lag_ms=100.0)

    still = mgr.shoot(0.5, 0.5)
    moving = mgr.shoot(0.5, 0.5, vx_norm=0.2)

    # Lead-aim pushed the aim point forward: 0.5 + 0.2×0.1 = 0.52 → DAC 2129.
    assert still.target_x_galvo == 2048
    assert moving.target_x_galvo == 2129


def test_engage_track_uses_track_state():
    from events.schemas import DetectionEvent
    from tracking.tracker import Tracker

    transport = DryRunTransport()
    transport.connect()
    mgr = LaserManager(transport, pattern=DotRepeat())

    trk = Tracker(mode="kalman_norm_dt")
    for i in range(4):
        d = DetectionEvent(timestamp=i * 0.03, sensor_id="gopro",
                           detection_id=0, x_norm=0.4, y_norm=0.6,
                           x_px=128, y_px=144, area_pixels=100.0,
                           bbox=(0, 0, 10, 10), classifier_label="mosquito",
                           classifier_confidence=0.9)
        trk.update([d], timestamp=i * 0.03)

    cmd = mgr.engage_track(trk.tracks()[0])
    assert cmd.track_id == 1
    # Aim lands near the track's estimated position (in DAC space).
    assert 1400 < cmd.target_x_galvo < 1900       # ~0.4 norm, KF-smoothed
    assert 2100 < cmd.target_y_galvo < 2600       # ~0.6 norm, KF-smoothed


def test_shoot_through_calibrated_mapper(tmp_path):
    transport = DryRunTransport()
    transport.connect()
    # A real fitted mapper from a dry calibration.
    camera = SyntheticCamera(H_truth=default_truth_homography())
    run = run_dry_calibration(transport, pattern="halton",
                              sensor_id="gopro", out_dir=tmp_path,
                              camera=camera)
    mgr = LaserManager(transport, mapper=run.mapper, pattern=DotRepeat())

    cmd = mgr.shoot(0.5, 0.5)
    # The mapper sends NORM 0.5,0.5 somewhere valid in 12-bit galvo space.
    assert 0 <= cmd.target_x_galvo <= 0xFFF
    assert 0 <= cmd.target_y_galvo <= 0xFFF
