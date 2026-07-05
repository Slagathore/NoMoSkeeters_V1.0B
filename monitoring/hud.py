"""Shared cv2 HUD for the live orchestrator scripts.

One overlay used by both `scripts/spotter_mode.py` and
`scripts/live_fire_session.py` (previously duplicated in each). Renders:

  - translucent banner: counters line + safety verdict line
  - right-aligned loop FPS and configured pipeline lag
  - FOV-margin rectangle
  - per-track marker, label, and a short velocity vector when the
    tracker state carries one
  - a fading ring at the last shot position (live-fire feedback)

Everything is drawn with LINE_AA. The banner is blended, not painted,
so the FOV rectangle and scene stay visible beneath it.
"""
from __future__ import annotations

import time
from typing import Optional, Sequence

import cv2

BANNER_H = 60
FLASH_DURATION_S = 0.35
_FONT = cv2.FONT_HERSHEY_SIMPLEX
# Velocity vectors preview this many seconds of travel at current speed.
_VEL_PREVIEW_S = 0.15


class FpsMeter:
    """EMA loop-rate meter. Call `tick()` once per rendered frame."""

    def __init__(self, alpha: float = 0.1):
        self._alpha = alpha
        self._last: Optional[float] = None
        self._fps: Optional[float] = None

    def tick(self, now: Optional[float] = None) -> Optional[float]:
        now = time.monotonic() if now is None else now
        if self._last is not None:
            dt = now - self._last
            if dt > 0.0:
                inst = 1.0 / dt
                self._fps = (inst if self._fps is None
                             else self._alpha * inst
                             + (1.0 - self._alpha) * self._fps)
        self._last = now
        return self._fps


def draw_hud(img,
             *,
             tracks,
             verdict,
             fov_margin: float,
             stats: Sequence[tuple[str, object]],
             fps: Optional[float] = None,
             lag_ms: Optional[float] = None,
             flash: Optional[tuple[float, tuple[int, int]]] = None) -> None:
    """Draw the session HUD onto `img` (BGR, mutated in place).

    `stats` is an ordered list of (label, value) pairs for the banner —
    each script passes its own counters (FIRED vs VIABLE etc.).
    `flash` is (monotonic_ts_of_shot, (cx_px, cy_px)); a ring is drawn
    for FLASH_DURATION_S after the shot.
    """
    h, w = img.shape[:2]

    # Translucent banner (blend, don't paint — the scene and the FOV
    # rectangle stay legible underneath).
    banner = img[:BANNER_H].copy()
    banner[:] = (24, 24, 24)
    cv2.addWeighted(banner, 0.72, img[:BANNER_H], 0.28, 0.0,
                    dst=img[:BANNER_H])

    # FOV margin rectangle.
    x0 = int(fov_margin * w)
    y0 = int(fov_margin * h)
    x1 = int((1.0 - fov_margin) * w)
    y1 = int((1.0 - fov_margin) * h)
    cv2.rectangle(img, (x0, y0), (x1, y1), (80, 200, 80), 1, cv2.LINE_AA)

    # Tracks: marker + label + velocity vector when the state carries one.
    for t in tracks:
        x_n, y_n = float(t.state[0]), float(t.state[1])
        cx, cy = int(x_n * (w - 1)), int(y_n * (h - 1))
        col = (0, 255, 0) if t.fire_eligible else (0, 165, 255)
        cv2.circle(img, (cx, cy), 6, col, 1, cv2.LINE_AA)
        if len(t.state) >= 4:
            vx_n, vy_n = float(t.state[2]), float(t.state[3])
            ex = int(cx + vx_n * _VEL_PREVIEW_S * (w - 1))
            ey = int(cy + vy_n * _VEL_PREVIEW_S * (h - 1))
            if (ex, ey) != (cx, cy):
                cv2.line(img, (cx, cy), (ex, ey), col, 1, cv2.LINE_AA)
        cv2.putText(img, f"t{t.track_id}", (cx + 8, cy - 6),
                    _FONT, 0.4, col, 1, cv2.LINE_AA)

    # Shot flash: expanding, fading ring at the last shot position.
    if flash is not None:
        age = time.monotonic() - flash[0]
        if 0.0 <= age < FLASH_DURATION_S:
            u = age / FLASH_DURATION_S
            radius = int(8 + 26 * u)
            level = int(255 * (1.0 - u))
            cv2.circle(img, flash[1], radius, (0, level // 3, level), 2,
                       cv2.LINE_AA)

    # Banner text: counters left, FPS/lag right, safety verdict below.
    line = "   ".join(f"{k} {v}" for k, v in stats)
    cv2.putText(img, line, (10, 22), _FONT, 0.55, (255, 255, 255), 1,
                cv2.LINE_AA)

    right = []
    if fps is not None:
        right.append(f"{fps:.0f} FPS")
    if lag_ms is not None:
        right.append(f"LAG {lag_ms:.0f}ms")
    if right:
        text = "  ".join(right)
        (tw, _), _ = cv2.getTextSize(text, _FONT, 0.5, 1)
        cv2.putText(img, text, (w - tw - 10, 22), _FONT, 0.5,
                    (180, 180, 180), 1, cv2.LINE_AA)

    safe_col = (0, 200, 0) if verdict.safe else (0, 0, 255)
    state = verdict.state.value.upper()
    reasons = "; ".join(verdict.reasons[:2])
    text = f"SAFETY: {state}   {reasons}"
    while (len(text) > 12
           and cv2.getTextSize(text, _FONT, 0.45, 1)[0][0] > w - 20):
        text = text[:-8] + "..."
    cv2.putText(img, text, (10, 48), _FONT, 0.45, safe_col, 1, cv2.LINE_AA)
