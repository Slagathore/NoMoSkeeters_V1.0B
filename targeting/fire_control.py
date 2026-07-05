"""FireGate — the per-track fire decision chain for a live session.

Extracted from scripts/live_fire_session.py so the highest-stakes logic in
the repo (safety re-poll ordering, cooldowns, FOV gate, object-size guard)
is unit-testable with a fake moderator. Pure decision logic — no laser
I/O, no cv2, no sensors.

Decision order per candidate track:
  1. fire_eligible signal        (tracker's surfaced eligibility)
  2. FOV margin                  (stay inside the calibrated frame)
  3. frame-level safety hint     (cheap: verdict cached once per frame)
  4. per-track cooldown          (don't spam one coasting track)
  5. global rate cap             (MAX_SHOT_RATE across all tracks)
  6. object-size guard           (BOOTSTRAP §10.3 — needs depth; fail-open)
  7. fresh is_safe_to_fire poll  (moderator contract: poll between shots)

Only a "fire" outcome mutates the gate's cooldown state.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from safety.object_size_guard import check_target_size

# Decision reasons (also used as counter keys by callers).
FIRE = "fire"
NOT_ELIGIBLE = "not_eligible"
OUTSIDE_FOV = "outside_fov"
UNSAFE = "unsafe"
COOLDOWN_TRACK = "cooldown_track"
COOLDOWN_GLOBAL = "cooldown_global"
OVERSIZE = "oversize"


@dataclass(frozen=True)
class FireDecision:
    fire: bool
    reason: str
    target_area_mm2: Optional[float] = None   # set when the size guard ran


class FireGate:
    """Stateful decision gate. One instance per session."""

    def __init__(self, *,
                 moderator,
                 fov_margin: float,
                 per_track_cooldown_s: float,
                 max_rate_hz: float,
                 frame_width_px: Optional[int] = None):
        self._moderator = moderator
        self._fov_margin = float(fov_margin)
        self._per_track_cooldown_s = float(per_track_cooldown_s)
        self._min_fire_interval_s = (1.0 / max_rate_hz if max_rate_hz > 0.0
                                     else 0.0)
        # frame_width_px enables the object-size guard's area estimate;
        # None skips the guard (equivalent to no depth — fail-open).
        self._frame_width_px = frame_width_px
        self._last_track_fire: dict[int, float] = {}
        self._last_any_fire = 0.0

    def evaluate(self, track, now: float, *,
                 safe_hint: bool) -> FireDecision:
        """Run the decision chain for one track at time `now`.

        `safe_hint` is the frame-level verdict (cheap, possibly stale) —
        a fresh moderator poll still guards the actual shot."""
        if not track.fire_eligible:
            return FireDecision(False, NOT_ELIGIBLE)

        x_n, y_n = float(track.state[0]), float(track.state[1])
        m = self._fov_margin
        if not (m <= x_n <= 1.0 - m and m <= y_n <= 1.0 - m):
            return FireDecision(False, OUTSIDE_FOV)

        if not safe_hint:
            return FireDecision(False, UNSAFE)

        last = self._last_track_fire.get(track.track_id, 0.0)
        if now - last < self._per_track_cooldown_s:
            return FireDecision(False, COOLDOWN_TRACK)
        if now - self._last_any_fire < self._min_fire_interval_s:
            return FireDecision(False, COOLDOWN_GLOBAL)

        area_mm2: Optional[float] = None
        det = getattr(track, "last_detection", None)
        if det is not None and self._frame_width_px is not None:
            ok, area_mm2 = check_target_size(
                det, frame_width_px=self._frame_width_px)
            if not ok:
                return FireDecision(False, OVERSIZE,
                                    target_area_mm2=area_mm2)

        # Fresh poll immediately before the shot — the moderator may have
        # flipped since the frame-level hint was taken.
        if not self._moderator.is_safe_to_fire():
            return FireDecision(False, UNSAFE, target_area_mm2=area_mm2)

        self._last_track_fire[track.track_id] = now
        self._last_any_fire = now
        return FireDecision(True, FIRE, target_area_mm2=area_mm2)
