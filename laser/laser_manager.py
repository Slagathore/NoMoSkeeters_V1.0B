"""LaserManager — turns a target into a streamed ShotPattern.

Per engaged target: lead-aim the predicted position forward by the measured
software lag, map NORM→GALVO via the CoordinateMapper, generate the configured
ShotPattern, and stream it through the transport.

LaserManager never calls enable_output() — emitting laser light is the safety
layer's job (Step 12). This only streams sample data via send_frame(); on a
DryRunTransport that is photon-free.

Reference: BOOTSTRAP_AMENDMENTS.md §9.11, §8.8 (lead-aim), Step 10.
"""
from __future__ import annotations

import logging
import time
from typing import Callable, Optional

from config import settings
from events.schemas import TargetCommandEvent
from laser.shot_patterns import ShotPattern, get_shot_pattern
from laser.transport import LaserCubeTransport
from laser.types import COORD_MAX, COORD_MIN

_log = logging.getLogger(__name__)

EventSink = Callable[[dict], None]


def _clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def _galvo_norm_to_dac(v: float) -> int:
    """Normalized galvo coord [0,1] → 12-bit DAC value."""
    return max(COORD_MIN, min(COORD_MAX, int(round(v * COORD_MAX))))


class LaserManager:
    """Streams ShotPatterns at targets through a LaserCubeTransport."""

    def __init__(self,
                 transport: LaserCubeTransport,
                 mapper=None,                       # CoordinateMapper or None
                 pattern: Optional[ShotPattern] = None,
                 *,
                 dwell_ms: Optional[int] = None,
                 power_pct: Optional[int] = None,
                 dac_rate: Optional[int] = None,
                 software_lag_ms: Optional[float] = None,
                 event_sink: Optional[EventSink] = None):
        self._transport = transport
        self._mapper = mapper                       # None → NORM is GALVO-norm
        self._pattern = pattern or get_shot_pattern()
        self._dwell_ms = (dwell_ms if dwell_ms is not None
                          else settings.SHOT_PATTERN_DWELL_MS)
        self._power_pct = (power_pct if power_pct is not None
                           else settings.SHOT_PATTERN_POWER_PCT)
        self._dac_rate = (dac_rate if dac_rate is not None
                          else settings.LASERCUBE_DEFAULT_DAC_RATE)
        self._lag_ms = (software_lag_ms if software_lag_ms is not None
                        else settings.LATENCY_SOFTWARE_LAG_MS)
        self._sink = event_sink

    @property
    def pattern(self) -> ShotPattern:
        return self._pattern

    def set_pattern(self, pattern: ShotPattern | str) -> None:
        self._pattern = (pattern if isinstance(pattern, ShotPattern)
                         else get_shot_pattern(pattern))

    # ── Engagement ───────────────────────────────────────────────────────

    def shoot(self, x_norm: float, y_norm: float,
              *, vx_norm: float = 0.0, vy_norm: float = 0.0,
              track_id: int = -1) -> TargetCommandEvent:
        """Aim at a NORM-space target (with optional velocity for lead-aim),
        generate the ShotPattern, and stream it. Returns the bus command."""
        # Lead-aim: predict where the target will be when the laser arrives.
        lag_s = self._lag_ms / 1000.0
        px = x_norm + vx_norm * lag_s
        py = y_norm + vy_norm * lag_s

        # NORM → GALVO-norm → 12-bit DAC.
        if self._mapper is not None:
            g = self._mapper.norm_to_galvo(px, py)
            gx_norm, gy_norm = float(g[0]), float(g[1])
        else:
            gx_norm, gy_norm = px, py
        gx = _galvo_norm_to_dac(_clamp01(gx_norm))
        gy = _galvo_norm_to_dac(_clamp01(gy_norm))

        points = self._pattern.generate(gx, gy, self._dwell_ms,
                                        self._dac_rate, self._power_pct)
        ok = self._transport.send_frame(points)
        if not ok:
            _log.error("laser_manager: send_frame failed for track %d", track_id)

        cmd = TargetCommandEvent(
            timestamp=time.monotonic(),
            track_id=track_id,
            target_x_galvo=gx,
            target_y_galvo=gy,
            pattern_id=self._pattern.pattern_id,
            dwell_ms=self._dwell_ms,
        )
        self._emit({
            "op": "shoot", "track_id": track_id,
            "pattern": self._pattern.pattern_id,
            "target_galvo": (gx, gy),
            "n_samples": len(points),
            "lead_applied": (vx_norm != 0.0 or vy_norm != 0.0),
            "sent": ok,
        })
        return cmd

    def engage_track(self, track) -> TargetCommandEvent:
        """Engage a TrackedTarget — pulls its position and velocity for
        lead-aim."""
        half = len(track.state) // 2
        return self.shoot(
            track.state[0], track.state[1],
            vx_norm=track.state[half], vy_norm=track.state[half + 1],
            track_id=track.track_id,
        )

    # ── Internal ─────────────────────────────────────────────────────────

    def _emit(self, payload: dict) -> None:
        _log.debug("laser_manager %s", payload)
        if self._sink is not None:
            try:
                self._sink({"ts": time.monotonic(), **payload})
            except Exception:
                _log.exception("laser_manager event sink raised; suppressing")
