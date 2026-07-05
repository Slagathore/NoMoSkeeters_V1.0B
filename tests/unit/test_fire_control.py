"""FireGate decision chain + object-size guard (BOOTSTRAP §10.3).

The gate is pure decision logic, so these tests drive it with fake tracks
and a fake moderator — the exact scenarios that used to live untested
inside scripts/live_fire_session.py's hot loop.
"""
from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from config import settings
from safety.object_size_guard import (check_target_size,
                                      pixel_area_to_world_mm2)
from targeting import fire_control
from targeting.fire_control import FireGate


class FakeModerator:
    def __init__(self, safe: bool = True) -> None:
        self.safe = safe
        self.polls = 0

    def is_safe_to_fire(self) -> bool:
        self.polls += 1
        return self.safe


def _track(tid=1, x=0.5, y=0.5, eligible=True, det=None):
    return SimpleNamespace(track_id=tid, state=[x, y, 0.0, 0.0],
                           fire_eligible=eligible, last_detection=det)


def _det(area_px=4.0, z=None):
    return SimpleNamespace(area_pixels=area_px, z_world_m=z)


def _gate(mod=None, **kw):
    defaults = dict(moderator=mod or FakeModerator(),
                    fov_margin=0.05, per_track_cooldown_s=0.8,
                    max_rate_hz=8.0, frame_width_px=640)
    defaults.update(kw)
    return FireGate(**defaults)


def test_clean_track_fires() -> None:
    d = _gate().evaluate(_track(), 100.0, safe_hint=True)
    assert d.fire and d.reason == fire_control.FIRE


def test_not_eligible_blocks() -> None:
    d = _gate().evaluate(_track(eligible=False), 100.0, safe_hint=True)
    assert not d.fire and d.reason == fire_control.NOT_ELIGIBLE


def test_outside_fov_blocks() -> None:
    d = _gate().evaluate(_track(x=0.01), 100.0, safe_hint=True)
    assert not d.fire and d.reason == fire_control.OUTSIDE_FOV


def test_unsafe_hint_blocks_without_polling() -> None:
    mod = FakeModerator(safe=True)
    d = _gate(mod).evaluate(_track(), 100.0, safe_hint=False)
    assert not d.fire and d.reason == fire_control.UNSAFE
    assert mod.polls == 0            # cheap rejection — no fresh poll


def test_final_poll_guards_the_shot() -> None:
    # Frame-level hint says safe but the moderator flipped since —
    # the fresh pre-shot poll must catch it.
    mod = FakeModerator(safe=False)
    d = _gate(mod).evaluate(_track(), 100.0, safe_hint=True)
    assert not d.fire and d.reason == fire_control.UNSAFE
    assert mod.polls == 1


def test_per_track_cooldown() -> None:
    g = _gate()
    assert g.evaluate(_track(tid=7), 100.0, safe_hint=True).fire
    d = g.evaluate(_track(tid=7), 100.5, safe_hint=True)
    assert not d.fire and d.reason == fire_control.COOLDOWN_TRACK
    assert g.evaluate(_track(tid=7), 100.9, safe_hint=True).fire


def test_global_rate_cap_across_tracks() -> None:
    g = _gate()                      # 8 Hz → 0.125s min interval
    assert g.evaluate(_track(tid=1), 100.0, safe_hint=True).fire
    d = g.evaluate(_track(tid=2), 100.05, safe_hint=True)
    assert not d.fire and d.reason == fire_control.COOLDOWN_GLOBAL
    assert g.evaluate(_track(tid=2), 100.2, safe_hint=True).fire


def test_blocked_decision_does_not_consume_cooldown() -> None:
    mod = FakeModerator(safe=False)
    g = _gate(mod)
    g.evaluate(_track(tid=3), 100.0, safe_hint=True)   # blocked at poll
    mod.safe = True
    assert g.evaluate(_track(tid=3), 100.01, safe_hint=True).fire


def test_oversize_target_blocks(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(settings, "SAFETY_OBJECT_SIZE_GUARD_ENABLED", True)
    monkeypatch.setattr(settings, "SAFETY_MAX_TARGET_AREA_MM2", 10000.0)
    # 1000 px² at 2 m through a 70° / 640 px camera ≈ 19,150 mm² — a hand,
    # not a mosquito.
    big = _det(area_px=1000.0, z=2.0)
    d = _gate().evaluate(_track(det=big), 100.0, safe_hint=True)
    assert not d.fire and d.reason == fire_control.OVERSIZE
    assert d.target_area_mm2 == pytest.approx(19151.0, rel=0.01)


def test_small_target_passes_size_guard(monkeypatch) -> None:
    monkeypatch.setattr(settings, "SAFETY_OBJECT_SIZE_GUARD_ENABLED", True)
    small = _det(area_px=4.0, z=2.0)     # ~77 mm² — plausibly insect-sized
    d = _gate().evaluate(_track(det=small), 100.0, safe_hint=True)
    assert d.fire
    assert d.target_area_mm2 is not None and d.target_area_mm2 < 100.0


def test_no_depth_fails_open() -> None:
    # §10.3: "no depth info; don't gate" — the 2D pipeline must not be
    # blocked by the guard.
    d = _gate().evaluate(_track(det=_det(area_px=1e6, z=None)), 100.0,
                         safe_hint=True)
    assert d.fire and d.target_area_mm2 is None


def test_guard_disabled_setting(monkeypatch) -> None:
    monkeypatch.setattr(settings, "SAFETY_OBJECT_SIZE_GUARD_ENABLED", False)
    ok, area = check_target_size(_det(area_px=1e6, z=2.0),
                                 frame_width_px=640)
    assert ok and area is None


def test_pixel_area_to_world_mm2_math() -> None:
    # One pixel at 2 m, 70° HFoV, 640 px wide → ~4.376 mm per pixel side.
    mm2 = pixel_area_to_world_mm2(1.0, 2.0, fov_h_rad=math.radians(70),
                                  frame_width_px=640)
    assert mm2 == pytest.approx(4.376 ** 2, rel=0.01)
