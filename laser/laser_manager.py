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

import dataclasses
import logging
import time
from typing import Callable, Optional

from config import settings
from events.schemas import TargetCommandEvent
from laser.shot_patterns import (ConeCollapseConfig, ConeCollapsePattern,
                                 Phase, ShotPattern, TrackerSnapshot,
                                 get_shot_pattern)
from laser.transport import LaserCubeTransport
from laser.types import COORD_MAX, COORD_MIN

_log = logging.getLogger(__name__)

EventSink = Callable[[dict], None]


def _clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def _galvo_norm_to_dac(v: float) -> int:
    """Normalized galvo coord [0,1] → 12-bit DAC value."""
    return max(COORD_MIN, min(COORD_MAX, int(round(v * COORD_MAX))))


def _dwell_cap_ms(requested_ms: float, label: str) -> float:
    """Enforce the §10.2 safety dwell ceiling. Pattern knobs
    (SHOT_PATTERN_DWELL_MS, CONE_BZZT_DURATION_S, ...) say how long a
    shot WANTS to hold the target; SAFETY_DWELL_LIMIT_MS says how long
    it MAY. Returns the capped value, warning loudly when it bites —
    raise the limit in settings if the clamp is unwanted, don't bypass
    it here."""
    if not settings.SAFETY_DWELL_LIMIT_ENABLED:
        return requested_ms
    limit = float(settings.SAFETY_DWELL_LIMIT_MS)
    if requested_ms > limit:
        _log.warning("%s wants %.0fms on-target but SAFETY_DWELL_LIMIT_MS "
                     "is %.0fms — clamping", label, requested_ms, limit)
        return limit
    return requested_ms


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
        self._dwell_ms = int(_dwell_cap_ms(
            dwell_ms if dwell_ms is not None
            else settings.SHOT_PATTERN_DWELL_MS, "shot pattern dwell"))
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

    # ── NORM → GALVO ─────────────────────────────────────────────────────

    def _norm_to_galvo_dac(self, px: float, py: float) -> tuple[int, int]:
        """NORM coord → 12-bit GALVO DAC pair, via the CoordinateMapper if
        one is wired in (else NORM is treated as GALVO-norm directly)."""
        if self._mapper is not None:
            g = self._mapper.norm_to_galvo(px, py)
            gx_norm, gy_norm = float(g[0]), float(g[1])
        else:
            gx_norm, gy_norm = px, py
        return (_galvo_norm_to_dac(_clamp01(gx_norm)),
                _galvo_norm_to_dac(_clamp01(gy_norm)))

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

        gx, gy = self._norm_to_galvo_dac(px, py)

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

    # ── Cone-collapse firing sequence (§9.11) ────────────────────────────

    def track_to_cone_snapshot(self, track) -> TrackerSnapshot:
        """Convert a norm-space TrackedTarget into a galvo-space
        TrackerSnapshot for the cone-collapse pattern.

        Velocity is finite-differenced through the NORM→GALVO mapping so a
        non-linear homography is handled correctly (the Jacobian, not a flat
        scale)."""
        half = len(track.state) // 2
        x_norm, y_norm = track.state[0], track.state[1]
        vx_norm, vy_norm = track.state[half], track.state[half + 1]
        gx, gy = self._norm_to_galvo_dac(x_norm, y_norm)
        probe_dt = 0.05
        gx2, gy2 = self._norm_to_galvo_dac(x_norm + vx_norm * probe_dt,
                                           y_norm + vy_norm * probe_dt)
        return TrackerSnapshot(
            target_x_galvo=float(gx), target_y_galvo=float(gy),
            target_vx_galvo_per_s=(gx2 - gx) / probe_dt,
            target_vy_galvo_per_s=(gy2 - gy) / probe_dt,
            confidence=float(getattr(track, "confidence", 1.0)))

    def engage_cone(self,
                    snapshot_fn: Callable[[], Optional[TrackerSnapshot]],
                    *,
                    config: Optional[ConeCollapseConfig] = None,
                    on_chunk: Optional[Callable] = None,
                    allow_reacquire: bool = True,
                    max_reacquires: Optional[int] = None,
                    sleep_fn: Callable[[float], None] = time.sleep) -> dict:
        """Run one full cone-collapse shot — the live "bzzt" firing sequence.

        `snapshot_fn` is polled once per chunk; it returns the freshest
        galvo-space TrackerSnapshot (use `track_to_cone_snapshot`), or None to
        signal the track is lost — which aborts the shot. Each chunk is
        streamed to the cube and the driver sleeps one chunk-duration so the
        cube ringbuffer stays roughly one chunk deep (no precomputed-shot
        overrun). On a breach the pattern restarts from the new position up to
        `max_reacquires` times, then aborts.

        Returns a summary dict: fired, reason, chunks, reacquires.
        """
        cfg = config or ConeCollapseConfig.from_settings()
        # The bzzt pulse is the shot's on-target dwell — the §10.2 safety
        # ceiling applies to it like any other dwell.
        capped_s = _dwell_cap_ms(cfg.bzzt_duration_s * 1000.0,
                                 "cone bzzt pulse") / 1000.0
        if capped_s != cfg.bzzt_duration_s:
            cfg = dataclasses.replace(cfg, bzzt_duration_s=capped_s)
            self._emit({"op": "dwell_clamped", "phase": "bzzt",
                        "capped_ms": capped_s * 1000.0})
        if max_reacquires is None:
            max_reacquires = settings.CONE_MAX_REACQUIRES
        chunk_dt = cfg.chunk_size / cfg.dac_rate

        snap = snapshot_fn()
        if snap is None:
            self._emit({"op": "cone_abort", "reason": "no_initial_track"})
            return {"fired": False, "reason": "no_initial_track",
                    "chunks": 0, "reacquires": 0}

        pat = ConeCollapsePattern(cfg, snap)
        reacquires = 0
        chunks_sent = 0
        fired = False

        while not pat.is_done:
            snap = snapshot_fn()
            if snap is None:
                self._emit({"op": "cone_abort", "reason": "track_lost",
                            "chunks": chunks_sent})
                return {"fired": fired, "reason": "track_lost",
                        "chunks": chunks_sent, "reacquires": reacquires}

            chunk = pat.next_chunk(snap)

            if chunk.breach_detected:
                if allow_reacquire and reacquires < max_reacquires:
                    reacquires += 1
                    pat = ConeCollapsePattern(cfg, snap)
                    self._emit({"op": "cone_reacquire", "n": reacquires})
                    continue
                self._emit({"op": "cone_abort", "reason": "breach",
                            "chunks": chunks_sent, "reacquires": reacquires})
                return {"fired": fired, "reason": "breach",
                        "chunks": chunks_sent, "reacquires": reacquires}

            if chunk.points:
                if not self._transport.send_frame(chunk.points):
                    _log.error("engage_cone: send_frame failed mid-shot")
                chunks_sent += 1
            if chunk.phase == Phase.BZZT:
                fired = True
            if on_chunk is not None:
                on_chunk(chunk)
            if not pat.is_done:
                sleep_fn(chunk_dt)

        self._emit({"op": "cone_done", "fired": fired,
                    "chunks": chunks_sent, "reacquires": reacquires})
        return {"fired": fired, "reason": "complete",
                "chunks": chunks_sent, "reacquires": reacquires}

    # ── Internal ─────────────────────────────────────────────────────────

    def _emit(self, payload: dict) -> None:
        _log.debug("laser_manager %s", payload)
        if self._sink is not None:
            try:
                self._sink({"ts": time.monotonic(), **payload})
            except Exception:
                _log.exception("laser_manager event sink raised; suppressing")
