"""§9.11 — cone-collapse firing sequence + LaserManager.engage_cone driver."""
from __future__ import annotations

from types import SimpleNamespace
from typing import Any

import pytest

from config import settings
from laser.laser_manager import LaserManager
from laser.shot_patterns import (ConeCollapseConfig, ConeCollapsePattern,
                                 Phase, TrackerSnapshot, simulate_to_csv)
from laser.transports.dry_run import DryRunTransport


def _fast_cfg(**over) -> ConeCollapseConfig:
    """A short pattern so tests run in a handful of chunks per phase."""
    base: dict[str, Any] = dict(shrink_duration_s=0.05, line_duration_s=0.02,
                                dark_duration_s=0.02, bzzt_duration_s=0.02,
                                dac_rate=30000, chunk_size=140)
    base.update(over)
    return ConeCollapseConfig(**base)


def _still():
    return TrackerSnapshot(2048, 2048, 0.0, 0.0, 1.0)


# ── ConeCollapsePattern — phases ─────────────────────────────────────────

def test_pattern_walks_every_phase_in_order():
    pat = ConeCollapsePattern(_fast_cfg(), _still())
    seen = []
    while not pat.is_done:
        chunk = pat.next_chunk(_still())
        if not seen or seen[-1] != chunk.phase:
            seen.append(chunk.phase)
    # The final next_chunk() crosses into DONE and returns an empty chunk.
    assert seen == [Phase.SHRINK, Phase.LINE, Phase.DARK, Phase.BZZT,
                    Phase.DONE]
    assert pat.is_done


def test_pattern_chunk_has_one_packet_of_samples():
    cfg = _fast_cfg()
    pat = ConeCollapsePattern(cfg, _still())
    chunk = pat.next_chunk(_still())
    assert len(chunk.points) == cfg.chunk_size


def test_shrink_radius_collapses_toward_r_min():
    cfg = _fast_cfg(r_start_galvo=80, r_min_galvo=6)
    pat = ConeCollapsePattern(cfg, _still())
    first = pat.next_chunk(_still())
    radii = [first.current_radius_galvo]
    while not pat.is_done:
        c = pat.next_chunk(_still())
        if c.phase == Phase.SHRINK:
            radii.append(c.current_radius_galvo)
    assert radii[0] > radii[-1]            # monotonically shrinking
    assert radii[0] <= 80 and radii[-1] >= 6


def test_dark_phase_is_blanked():
    pat = ConeCollapsePattern(_fast_cfg(), _still())
    dark_pts = []
    while not pat.is_done:
        c = pat.next_chunk(_still())
        if c.phase == Phase.DARK:
            dark_pts.extend(c.points)
    assert dark_pts
    assert all(p.r == 0 and p.g == 0 and p.b == 0 for p in dark_pts)


def test_bzzt_phase_is_full_power():
    cfg = _fast_cfg()
    pat = ConeCollapsePattern(cfg, _still())
    bzzt_pts = []
    while not pat.is_done:
        c = pat.next_chunk(_still())
        if c.phase == Phase.BZZT:
            bzzt_pts.extend(c.points)
    assert bzzt_pts
    assert all((p.r, p.g, p.b) == cfg.color_bzzt for p in bzzt_pts)


# ── Tracking — breach + stretch ──────────────────────────────────────────

def test_breach_when_target_leaves_the_cone():
    cfg = _fast_cfg(r_start_galvo=80, breach_multiplier=1.5)
    pat = ConeCollapsePattern(cfg, _still())
    # Target far outside the initial cone (80 * 1.5 = 120 galvo grace).
    chunk = pat.next_chunk(TrackerSnapshot(2048 + 500, 2048, 0, 0, 1.0))
    assert chunk.breach_detected


def test_no_breach_when_target_stays_inside():
    pat = ConeCollapsePattern(_fast_cfg(r_start_galvo=80), _still())
    chunk = pat.next_chunk(TrackerSnapshot(2048 + 10, 2048, 0, 0, 1.0))
    assert not chunk.breach_detected


def test_cone_stretches_along_velocity():
    cfg = _fast_cfg(stretch_max=1.8, stretch_velocity_scale=1500.0)
    pat = ConeCollapsePattern(cfg, _still())
    moving = TrackerSnapshot(2048, 2048, 1500.0, 0.0, 1.0)
    chunk = pat.next_chunk(moving)
    assert chunk.stretch_factor > 1.0


# ── Config ───────────────────────────────────────────────────────────────

def test_config_from_settings_matches_settings():
    cfg = ConeCollapseConfig.from_settings()
    assert cfg.shrink_duration_s == settings.CONE_SHRINK_DURATION_S
    assert cfg.r_start_galvo == settings.CONE_R_START_GALVO
    assert cfg.lead_factor == settings.CONE_LEAD_FACTOR
    assert cfg.dac_rate == settings.LASERCUBE_DEFAULT_DAC_RATE
    assert cfg.chunk_size == settings.LASERCUBE_MAX_SAMPLES_PER_PACKET


# ── simulate_to_csv ──────────────────────────────────────────────────────

def test_simulate_to_csv_writes_samples(tmp_path):
    path = tmp_path / "cone.csv"
    simulate_to_csv(str(path), config=_fast_cfg())
    lines = path.read_text().splitlines()
    assert lines[0].startswith("t_s,x_galvo,y_galvo")
    assert len(lines) > 1                  # header + sample rows
    assert "bzzt" in path.read_text()      # the kill phase was reached


def test_simulate_to_csv_terminates_on_non_converging_target(tmp_path):
    # A target that teleports far every chunk breaches forever — the
    # max_restarts cap must stop the run rather than loop unbounded.
    def runaway(t_s):
        return TrackerSnapshot(2048 + t_s * 100000, 2048, 0, 0, 1.0)

    path = tmp_path / "runaway.csv"
    simulate_to_csv(str(path), config=_fast_cfg(), mosquito_trajectory=runaway,
                    max_restarts=3)
    rows = len(path.read_text().splitlines())
    # 3 restarts of SHRINK only — bounded, and far short of a full pattern.
    assert 1 < rows < 50 * _fast_cfg().chunk_size


# ── LaserManager.engage_cone ─────────────────────────────────────────────

def _no_sleep(_s):
    pass


def test_engage_cone_fires_on_a_still_target():
    mgr = LaserManager(DryRunTransport())
    summary = mgr.engage_cone(_still, config=_fast_cfg(), sleep_fn=_no_sleep)
    assert summary["fired"] is True
    assert summary["reason"] == "complete"
    assert summary["chunks"] > 0
    assert summary["reacquires"] == 0


def test_engage_cone_aborts_when_no_initial_track():
    mgr = LaserManager(DryRunTransport())
    summary = mgr.engage_cone(lambda: None, config=_fast_cfg(),
                              sleep_fn=_no_sleep)
    assert summary["fired"] is False
    assert summary["reason"] == "no_initial_track"


def test_engage_cone_aborts_when_track_lost_mid_shot():
    calls = {"n": 0}

    def snapshot_fn():
        calls["n"] += 1
        return _still() if calls["n"] <= 3 else None

    mgr = LaserManager(DryRunTransport())
    summary = mgr.engage_cone(snapshot_fn, config=_fast_cfg(),
                              sleep_fn=_no_sleep)
    assert summary["reason"] == "track_lost"
    assert summary["fired"] is False


def test_engage_cone_reacquires_then_aborts_on_repeated_breach():
    # Each poll the target has jumped 500 galvo units further away — every
    # fresh pattern immediately breaches again.
    calls = {"n": 0}

    def snapshot_fn():
        x = 2048 + calls["n"] * 500
        calls["n"] += 1
        return TrackerSnapshot(x, 2048, 0, 0, 1.0)

    mgr = LaserManager(DryRunTransport())
    summary = mgr.engage_cone(snapshot_fn, config=_fast_cfg(),
                              max_reacquires=2, sleep_fn=_no_sleep)
    assert summary["reason"] == "breach"
    assert summary["reacquires"] == 2
    assert summary["fired"] is False


def test_engage_cone_no_reacquire_aborts_on_first_breach():
    calls = {"n": 0}

    def snapshot_fn():
        x = 2048 + calls["n"] * 500
        calls["n"] += 1
        return TrackerSnapshot(x, 2048, 0, 0, 1.0)

    mgr = LaserManager(DryRunTransport())
    summary = mgr.engage_cone(snapshot_fn, config=_fast_cfg(),
                              allow_reacquire=False, sleep_fn=_no_sleep)
    assert summary["reason"] == "breach"
    assert summary["reacquires"] == 0


def test_engage_cone_paces_one_sleep_per_chunk():
    slept = []
    mgr = LaserManager(DryRunTransport())
    summary = mgr.engage_cone(_still, config=_fast_cfg(),
                              sleep_fn=slept.append)
    # The driver paces one chunk-duration after every streamed chunk; the
    # trailing empty DONE chunk streams nothing and is not slept on.
    assert len(slept) == summary["chunks"]
    cfg = _fast_cfg()
    assert all(abs(s - cfg.chunk_size / cfg.dac_rate) < 1e-9 for s in slept)


# ── track_to_cone_snapshot ───────────────────────────────────────────────

def test_track_to_cone_snapshot_maps_norm_to_galvo():
    mgr = LaserManager(DryRunTransport())            # no mapper → norm == galvo-norm
    track = SimpleNamespace(state=(0.5, 0.5, 0.1, 0.0), confidence=0.8)
    snap = mgr.track_to_cone_snapshot(track)
    # 0.5 norm → mid galvo range.
    assert abs(snap.target_x_galvo - 2048) <= 1
    assert abs(snap.target_y_galvo - 2048) <= 1
    # +0.1 norm/s in x → positive galvo velocity; y still.
    assert snap.target_vx_galvo_per_s > 0
    assert abs(snap.target_vy_galvo_per_s) < 1e-6
    assert snap.confidence == 0.8
