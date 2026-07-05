"""SAFETY_DWELL_LIMIT enforcement in LaserManager (BOOTSTRAP §10.2).

The pattern knobs say how long a shot WANTS the target; the safety limit
says how long it MAY. LaserManager clamps at the seam.
"""
from __future__ import annotations

import pytest

from config import settings
from laser.laser_manager import LaserManager
from laser.shot_patterns import ConeCollapseConfig, TrackerSnapshot
from laser.transports.dry_run import DryRunTransport


def _transport() -> DryRunTransport:
    t = DryRunTransport()
    t.connect()
    return t


def test_shot_pattern_dwell_clamped(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(settings, "SAFETY_DWELL_LIMIT_ENABLED", True)
    monkeypatch.setattr(settings, "SAFETY_DWELL_LIMIT_MS", 80)
    mgr = LaserManager(_transport(), dwell_ms=200)
    assert mgr._dwell_ms == 80


def test_dwell_within_limit_untouched(monkeypatch) -> None:
    monkeypatch.setattr(settings, "SAFETY_DWELL_LIMIT_ENABLED", True)
    monkeypatch.setattr(settings, "SAFETY_DWELL_LIMIT_MS", 80)
    mgr = LaserManager(_transport(), dwell_ms=50)
    assert mgr._dwell_ms == 50


def test_dwell_limit_disabled_passes_through(monkeypatch) -> None:
    monkeypatch.setattr(settings, "SAFETY_DWELL_LIMIT_ENABLED", False)
    mgr = LaserManager(_transport(), dwell_ms=200)
    assert mgr._dwell_ms == 200


def test_cone_bzzt_clamped_and_reported(monkeypatch) -> None:
    monkeypatch.setattr(settings, "SAFETY_DWELL_LIMIT_ENABLED", True)
    monkeypatch.setattr(settings, "SAFETY_DWELL_LIMIT_MS", 80)

    events: list[dict] = []
    mgr = LaserManager(_transport(), event_sink=events.append)

    cfg = ConeCollapseConfig.from_settings()
    cfg.shrink_duration_s = 0.05
    cfg.line_duration_s = 0.02
    cfg.dark_duration_s = 0.02
    cfg.bzzt_duration_s = 0.12          # wants 120ms > 80ms limit

    snap = TrackerSnapshot(2048.0, 2048.0, 0.0, 0.0, 1.0)
    summary = mgr.engage_cone(lambda: snap, config=cfg,
                              sleep_fn=lambda s: None)
    assert summary["fired"]
    clamps = [e for e in events if e.get("op") == "dwell_clamped"]
    assert len(clamps) == 1
    assert clamps[0]["capped_ms"] == pytest.approx(80.0)
    # Caller's config object must not be mutated by the clamp.
    assert cfg.bzzt_duration_s == 0.12
