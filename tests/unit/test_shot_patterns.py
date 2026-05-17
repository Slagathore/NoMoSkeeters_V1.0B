"""Step 10: ShotPattern generators — counts, geometry, power."""
from __future__ import annotations

import pytest

from laser.shot_patterns import (
    SHOT_PATTERNS, DotRepeat, FigureEight, MicroCircle,
    _rgb_level, _sample_count, get_shot_pattern,
)


def _center(points):
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    return sum(xs) / len(xs), sum(ys) / len(ys)


# ── Sample counts ────────────────────────────────────────────────────────

def test_sample_count_scales_with_dwell_and_rate():
    assert _sample_count(80, 30000) == 2400
    assert _sample_count(0, 30000) == 1            # never zero samples


def test_rgb_level_maps_power():
    assert _rgb_level(100) == 0xFFF
    assert _rgb_level(0) == 0
    assert _rgb_level(50) == 0xFFF // 2


# ── DotRepeat ────────────────────────────────────────────────────────────

def test_dot_repeat_parks_on_target():
    pts = DotRepeat().generate(2048, 1900, dwell_ms=80, dac_rate=30000,
                               power_pct=100)
    assert len(pts) == 2400
    assert all(p.x == 2048 and p.y == 1900 for p in pts)
    assert all(p.r == 0xFFF for p in pts)


# ── MicroCircle ──────────────────────────────────────────────────────────

def test_micro_circle_is_centered_and_bounded():
    radius = 30
    pts = MicroCircle(radius_galvo=radius).generate(
        2048, 2048, dwell_ms=80, dac_rate=30000, power_pct=100)
    assert len(pts) == 2400
    cx, cy = _center(pts)
    assert abs(cx - 2048) < 1.0 and abs(cy - 2048) < 1.0
    # Every sample sits on the orbit radius (±1 from integer rounding).
    for p in pts:
        r = ((p.x - 2048) ** 2 + (p.y - 2048) ** 2) ** 0.5
        assert abs(r - radius) <= 1.5


# ── FigureEight ──────────────────────────────────────────────────────────

def test_figure_eight_centered_and_sized():
    scale = 40
    pts = FigureEight(scale_galvo=scale).generate(
        1000, 1000, dwell_ms=80, dac_rate=30000, power_pct=100)
    assert len(pts) == 2400
    cx, cy = _center(pts)
    assert abs(cx - 1000) < 2.0 and abs(cy - 1000) < 2.0
    assert max(abs(p.x - 1000) for p in pts) <= scale + 1


# ── Registry ─────────────────────────────────────────────────────────────

def test_registry_and_dispatch():
    assert set(SHOT_PATTERNS) == {"dot_repeat", "micro_circle", "figure_eight"}
    assert get_shot_pattern("dot_repeat").pattern_id == "dot_repeat"
    assert get_shot_pattern().pattern_id == "micro_circle"   # config default


def test_get_shot_pattern_rejects_unknown():
    with pytest.raises(ValueError):
        get_shot_pattern("death_ray")
