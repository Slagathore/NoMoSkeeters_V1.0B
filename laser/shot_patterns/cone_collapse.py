"""Cone-collapse ShotPattern — the "bzzt" firing sequence.

A wide cone of light tracks the target, shrinking continuously until it
becomes a tight line, blanks for a beat, then fires a single bright pulse at
full power.

Why this design:
  1. Visually obvious to a human operator what's happening: wide cone =
     "acquiring," shrinking = "homing," line = "locked," blink = "trigger
     pull moment," bzzt = "fired."
  2. The shrinking cone is *de facto* settling time for the galvos. By the
     time it's "a line" the mechanical PID has locked on the target position.
  3. The wide-then-tight progression doubles as a tracking-confidence
     visualizer — if the target leaves the cone during shrink, the pattern
     aborts and restarts from the new position with a fresh wide cone.
  4. When the target is moving, the cone stretches into an oval along the
     velocity vector — the laser dwells more on where the target IS GOING
     than where it HAS BEEN. Standard lead-aim, baked into pattern geometry.

CRITICAL: this generator emits ONE chunk (~140 samples) at a time, NOT the
full pattern in advance. The caller (LaserManager.engage_cone) is expected to:
  1. Call `next_chunk()` repeatedly to get the next chunk.
  2. Pass current tracker state at each call (position, velocity, confidence).
  3. The generator uses the LIVE tracker state to update the cone center,
     stretch, and breach detection ON EACH CHUNK.
  4. If a chunk reports `breach_detected`, the caller decides whether to abort
     the shot or restart the pattern from the new position.

This way the pattern follows the target in real time. We do NOT precompute the
full ~33,000-sample shot in advance — that would make the cone blind to where
the target actually went during the pattern.

Coordinate space: all input/output in 12-bit GALVO coordinates (0-4095).

Reference: BOOTSTRAP_AMENDMENTS.md §9.11 (shot patterns).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from laser.types import LaserPoint


# ── Configuration ────────────────────────────────────────────────────────

class Phase(str, Enum):
    SHRINK = "shrink"      # wide cone collapsing to a point
    LINE = "line"          # tight micro-sweep across target
    DARK = "dark"          # blank — the "trigger pull" moment
    BZZT = "bzzt"          # full-power kill pulse
    DONE = "done"


@dataclass
class ConeCollapseConfig:
    """All timing and geometry knobs for the pattern. Operator-tunable —
    `from_settings()` builds one from config/settings.py (CONE_* section)."""

    # Timing — durations of each phase in seconds.
    # Default total dwell = shrink + line + dark + bzzt = ~1.1 s
    shrink_duration_s: float = 0.80
    line_duration_s: float = 0.08
    dark_duration_s: float = 0.10
    bzzt_duration_s: float = 0.12

    # Cone geometry in galvo units. r_start = initial wide-cone radius,
    # r_min = the "line"/bzzt tight radius. 12-bit galvo space is 0-4095; a
    # radius of 80 = ~2% of full scan range (~3 cm of cone at 2 m).
    r_start_galvo: int = 80
    r_min_galvo: int = 6

    # Power level per phase, as a 12-bit DAC value (0-4095). NOTE: the sample
    # generators currently drive brightness from the color_* tuples below;
    # these fields are reserved for an explicit power-scaling pass.
    power_shrink: int = 0x500    # ~30%
    power_line: int = 0x600      # ~37%
    power_bzzt: int = 0xFFF      # 100%

    # Color channel mixing — (r, g, b), each 0-4095. Default: red shrink
    # (visible, lower energy), green-tinted bzzt (matches the laser's
    # strongest channel for peak photon-per-watt visibility).
    color_shrink: tuple = (0xCC0, 0x000, 0x000)   # red
    color_line: tuple = (0xCC0, 0x400, 0x000)     # orange-red
    color_bzzt: tuple = (0xFFF, 0xFFF, 0xFFF)     # white (all channels max)

    # Tracking behavior. lead_factor: each chunk, the cone center moves this
    # fraction of the way toward the target's current position. Higher = more
    # aggressive tracking, more glitchy. Lower = smoother, lags more.
    lead_factor: float = 0.20

    # Breach threshold: if the target is more than `breach_multiplier` times
    # the current cone radius from the cone center, declare a breach. 1.5 =
    # the cone gets 1.5x grace before a restart.
    breach_multiplier: float = 1.5

    # Stretch behavior: when the target moves, the cone becomes an oval.
    # stretch_max: maximum long-axis / short-axis ratio. stretch_velocity_scale:
    # the target speed (galvo units/s) at which max stretch is reached; below
    # it, stretch interpolates linearly from 1.0 (circle).
    stretch_max: float = 1.8
    stretch_velocity_scale: float = 1500.0

    # Sample rate of the LaserCube DAC.
    dac_rate: int = 30000

    # Samples per chunk (each chunk fits in one UDP packet).
    chunk_size: int = 140

    @classmethod
    def from_settings(cls) -> "ConeCollapseConfig":
        """Build a config from config/settings.py — the operator-tunable
        CONE_* knobs plus the cube's DAC rate and packet size."""
        from config import settings
        return cls(
            shrink_duration_s=settings.CONE_SHRINK_DURATION_S,
            line_duration_s=settings.CONE_LINE_DURATION_S,
            dark_duration_s=settings.CONE_DARK_DURATION_S,
            bzzt_duration_s=settings.CONE_BZZT_DURATION_S,
            r_start_galvo=settings.CONE_R_START_GALVO,
            r_min_galvo=settings.CONE_R_MIN_GALVO,
            power_shrink=settings.CONE_POWER_SHRINK,
            power_line=settings.CONE_POWER_LINE,
            power_bzzt=settings.CONE_POWER_BZZT,
            lead_factor=settings.CONE_LEAD_FACTOR,
            breach_multiplier=settings.CONE_BREACH_MULTIPLIER,
            stretch_max=settings.CONE_STRETCH_MAX,
            stretch_velocity_scale=settings.CONE_STRETCH_VELOCITY_SCALE,
            dac_rate=settings.LASERCUBE_DEFAULT_DAC_RATE,
            chunk_size=settings.LASERCUBE_MAX_SAMPLES_PER_PACKET,
        )


@dataclass
class TrackerSnapshot:
    """One moment of tracker state, passed in at each next_chunk() call.
    All coordinates in GALVO space."""
    target_x_galvo: float
    target_y_galvo: float
    target_vx_galvo_per_s: float = 0.0
    target_vy_galvo_per_s: float = 0.0
    confidence: float = 1.0          # 0-1; reserved for cone-size modulation


@dataclass
class ChunkResult:
    """What next_chunk() returns. The caller streams `points` to the cube."""
    points: list[LaserPoint]
    phase: Phase
    phase_progress: float            # 0-1, within the current phase
    current_radius_galvo: float
    cone_center_x: float
    cone_center_y: float
    stretch_factor: float
    stretch_angle_rad: float
    elapsed_s: float                 # since pattern start
    is_done: bool = False
    breach_detected: bool = False    # target left the cone — caller must
    #                                  decide whether to abort or restart


# ── The generator ────────────────────────────────────────────────────────

class ConeCollapsePattern:
    """Stateful pattern generator. One instance per shot.

    Usage::

        pat = ConeCollapsePattern(config, initial_tracker_state)
        while not pat.is_done:
            chunk = pat.next_chunk(current_tracker_state)
            if chunk.breach_detected:
                break          # caller: restart with a fresh instance, or abort
            transport.send_frame(chunk.points)
    """

    def __init__(self, config: ConeCollapseConfig, initial: TrackerSnapshot):
        self.cfg = config
        # Cone center starts AT the target — phase 1 begins with the cone
        # locked onto the initial estimate.
        self.cone_x = initial.target_x_galvo
        self.cone_y = initial.target_y_galvo
        self.stretch_factor = 1.0
        self.stretch_angle = 0.0
        # theta tracks the sweep angle around the cone — incremented each
        # sample so we trace a continuous arc instead of teleporting.
        self.theta = 0.0
        self.phase = Phase.SHRINK
        self.phase_elapsed_s = 0.0
        self.total_elapsed_s = 0.0
        self._phase_dur = {
            Phase.SHRINK: config.shrink_duration_s,
            Phase.LINE: config.line_duration_s,
            Phase.DARK: config.dark_duration_s,
            Phase.BZZT: config.bzzt_duration_s,
        }

    @property
    def is_done(self) -> bool:
        return self.phase == Phase.DONE

    def _done_chunk(self) -> ChunkResult:
        return ChunkResult(
            points=[], phase=Phase.DONE, phase_progress=1.0,
            current_radius_galvo=0,
            cone_center_x=self.cone_x, cone_center_y=self.cone_y,
            stretch_factor=1.0, stretch_angle_rad=0.0,
            elapsed_s=self.total_elapsed_s, is_done=True,
        )

    def next_chunk(self, tracker: TrackerSnapshot) -> ChunkResult:
        """Generate the next chunk of samples, using the LIVE tracker snapshot
        to update cone position, stretch, and detect breach."""
        if self.is_done:
            return self._done_chunk()

        # How long does this chunk represent in wall-clock time?
        chunk_dt_s = self.cfg.chunk_size / self.cfg.dac_rate

        # Phase transition — we don't split chunks across phases; advance at
        # the start of the chunk if past the boundary. The error (< chunk_dt)
        # is invisible.
        if self.phase_elapsed_s >= self._phase_dur.get(self.phase, 0.0):
            self._advance_phase()
            if self.is_done:
                return self._done_chunk()

        # Track-and-stretch (only meaningful in the SHRINK phase).
        breach_detected = False
        if self.phase == Phase.SHRINK:
            breach_detected = self._update_tracking(tracker)

        # Generate samples for this chunk based on the current phase.
        if self.phase == Phase.SHRINK:
            points = self._generate_shrink_samples(chunk_dt_s)
        elif self.phase == Phase.LINE:
            points = self._generate_line_samples(chunk_dt_s)
        elif self.phase == Phase.DARK:
            # Dark phase — all samples blanked. The galvos still need to scan
            # somewhere (parking hard stresses the mirrors), so they trace a
            # tiny invisible circle.
            points = self._generate_dark_samples(chunk_dt_s)
        elif self.phase == Phase.BZZT:
            points = self._generate_bzzt_samples(chunk_dt_s)
        else:
            points = []

        # Advance clocks.
        self.phase_elapsed_s += chunk_dt_s
        self.total_elapsed_s += chunk_dt_s

        cur_r = self._current_radius()
        progress = self.phase_elapsed_s / max(
            0.001, self._phase_dur.get(self.phase, 1.0))
        return ChunkResult(
            points=points,
            phase=self.phase,
            phase_progress=min(1.0, progress),
            current_radius_galvo=cur_r,
            cone_center_x=self.cone_x,
            cone_center_y=self.cone_y,
            stretch_factor=self.stretch_factor,
            stretch_angle_rad=self.stretch_angle,
            elapsed_s=self.total_elapsed_s,
            is_done=False,
            breach_detected=breach_detected,
        )

    def _advance_phase(self) -> None:
        """Move to the next phase in the sequence and reset its clock."""
        order = [Phase.SHRINK, Phase.LINE, Phase.DARK, Phase.BZZT, Phase.DONE]
        idx = order.index(self.phase)
        self.phase = order[idx + 1] if idx < len(order) - 1 else Phase.DONE
        self.phase_elapsed_s = 0.0

    def _current_radius(self) -> float:
        """Cone radius for the current phase + progress."""
        if self.phase == Phase.SHRINK:
            # Linear interp from r_start to r_min over the shrink duration.
            t = self.phase_elapsed_s / max(0.001, self.cfg.shrink_duration_s)
            t = min(1.0, max(0.0, t))
            return (self.cfg.r_start_galvo
                    + (self.cfg.r_min_galvo - self.cfg.r_start_galvo) * t)
        if self.phase == Phase.LINE:
            return self.cfg.r_min_galvo
        if self.phase == Phase.DARK:
            # Tiny circle, invisible (zero power), just to keep galvos moving.
            return self.cfg.r_min_galvo * 0.5
        if self.phase == Phase.BZZT:
            return self.cfg.r_min_galvo
        return 0.0

    def _update_tracking(self, tracker: TrackerSnapshot) -> bool:
        """Slide the cone center toward the target, update stretch, detect
        breach. Returns True if a breach was detected (target outside cone)."""
        dx = tracker.target_x_galvo - self.cone_x
        dy = tracker.target_y_galvo - self.cone_y
        dist = math.sqrt(dx * dx + dy * dy)
        cur_r = self._current_radius()

        # Breach test BEFORE sliding the cone — if the mosquito is already
        # well outside, flag it rather than silently chasing.
        if dist > cur_r * self.cfg.breach_multiplier:
            return True

        # Slide the cone center toward the target. lead_factor 0.2 = move 20%
        # of the way per chunk; at chunk_dt ~4.7 ms that is a smooth pursuit
        # response with time constant ~20 ms.
        self.cone_x += dx * self.cfg.lead_factor
        self.cone_y += dy * self.cfg.lead_factor

        # Stretch along the velocity vector — the cone elongates in the
        # direction of travel so the laser dwells on where the target is going.
        speed = math.sqrt(tracker.target_vx_galvo_per_s ** 2
                          + tracker.target_vy_galvo_per_s ** 2)
        if speed > 10.0:
            t = min(1.0, speed / self.cfg.stretch_velocity_scale)
            self.stretch_factor = 1.0 + (self.cfg.stretch_max - 1.0) * t
            self.stretch_angle = math.atan2(tracker.target_vy_galvo_per_s,
                                            tracker.target_vx_galvo_per_s)
        else:
            # Decay back toward a circle when the target slows.
            self.stretch_factor = self.stretch_factor * 0.9 + 1.0 * 0.1

        return False

    def _generate_shrink_samples(self, dt_s: float) -> list[LaserPoint]:
        """Trace the (stretched) oval around the cone center, sweeping theta."""
        n = self.cfg.chunk_size
        pts = []
        # Advance theta ~2 full revolutions per chunk — visually continuous,
        # slow enough that galvo mechanics are not stressed.
        theta_per_sample = (2 * math.pi * 2) / n
        r = self._current_radius()
        cos_a = math.cos(self.stretch_angle)
        sin_a = math.sin(self.stretch_angle)
        sf = self.stretch_factor
        # Oval long axis = r * sf, short axis = r / sqrt(sf). Compute (u, v)
        # in oval-local space then rotate into galvo space.
        for i in range(n):
            theta = self.theta + i * theta_per_sample
            u = r * sf * math.cos(theta)
            v = r * (1.0 / math.sqrt(sf)) * math.sin(theta)
            gx = self.cone_x + u * cos_a - v * sin_a
            gy = self.cone_y + u * sin_a + v * cos_a
            pts.append(LaserPoint(
                x=int(_clamp_galvo(gx)), y=int(_clamp_galvo(gy)),
                r=self.cfg.color_shrink[0], g=self.cfg.color_shrink[1],
                b=self.cfg.color_shrink[2]))
        self.theta += n * theta_per_sample
        return pts

    def _generate_line_samples(self, dt_s: float) -> list[LaserPoint]:
        """Tight micro-sweep — a small line across the target, oriented along
        the velocity vector if stretched. The 'locked' phase: galvos are
        essentially parked, with minimal motion to keep the mirrors moving."""
        n = self.cfg.chunk_size
        pts = []
        line_half = self.cfg.r_min_galvo
        cos_a = math.cos(self.stretch_angle)
        sin_a = math.sin(self.stretch_angle)
        for i in range(n):
            phase = (i / n) * 2 * math.pi
            u = line_half * math.sin(phase)
            v = 0.0
            gx = self.cone_x + u * cos_a - v * sin_a
            gy = self.cone_y + u * sin_a + v * cos_a
            pts.append(LaserPoint(
                x=int(_clamp_galvo(gx)), y=int(_clamp_galvo(gy)),
                r=self.cfg.color_line[0], g=self.cfg.color_line[1],
                b=self.cfg.color_line[2]))
        return pts

    def _generate_dark_samples(self, dt_s: float) -> list[LaserPoint]:
        """Blank phase — color channels at zero, galvos trace a tiny invisible
        circle so the mirrors are not parked hard. ~100 ms of darkness is the
        'trigger pull' beat before the kill pulse."""
        n = self.cfg.chunk_size
        pts = []
        r = self.cfg.r_min_galvo * 0.5
        theta_per_sample = (2 * math.pi) / n
        for i in range(n):
            theta = self.theta + i * theta_per_sample
            gx = self.cone_x + r * math.cos(theta)
            gy = self.cone_y + r * math.sin(theta)
            pts.append(LaserPoint(
                x=int(_clamp_galvo(gx)), y=int(_clamp_galvo(gy)),
                r=0, g=0, b=0))                # all channels off — no output
        self.theta += n * theta_per_sample
        return pts

    def _generate_bzzt_samples(self, dt_s: float) -> list[LaserPoint]:
        """The kill pulse — full power, tight micro-circle around the target.
        This is where the energy actually goes. Color is white (all channels
        max) for maximum brightness."""
        n = self.cfg.chunk_size
        pts = []
        r = self.cfg.r_min_galvo
        # Sweep faster during bzzt so the energy is spread but the visual
        # effect is still one bright dot.
        theta_per_sample = (2 * math.pi * 3) / n
        for i in range(n):
            theta = self.theta + i * theta_per_sample
            gx = self.cone_x + r * math.cos(theta)
            gy = self.cone_y + r * math.sin(theta)
            pts.append(LaserPoint(
                x=int(_clamp_galvo(gx)), y=int(_clamp_galvo(gy)),
                r=self.cfg.color_bzzt[0], g=self.cfg.color_bzzt[1],
                b=self.cfg.color_bzzt[2]))
        self.theta += n * theta_per_sample
        return pts


def _clamp_galvo(v: float) -> float:
    """Clamp to 12-bit galvo range [0, 4095]."""
    return max(0, min(4095, v))


# ── Demo / tuning — simulate offline, no cube required ───────────────────

def simulate_to_csv(output_path: str,
                    config: Optional[ConeCollapseConfig] = None,
                    mosquito_trajectory=None,
                    max_restarts: int = 8) -> str:
    """Run a complete pattern against a simulated mosquito trajectory and
    write a per-sample CSV — (t_s, x_galvo, y_galvo, r, g, b, phase, radius,
    stretch, breach). Plot it to inspect what the cube would scan. No
    hardware involved.

    On a breach the simulator restarts the SHRINK phase from the target's
    new position (mirroring a real reacquire). A target that never settles —
    one whose pursuit lag exceeds the shrunk cone — would restart forever, so
    the run stops after `max_restarts` breaches, exactly as the live
    `LaserManager.engage_cone` aborts after `max_reacquires`."""
    import csv
    if config is None:
        config = ConeCollapseConfig()
    if mosquito_trajectory is None:
        def _stationary_traj(t_s):              # stationary, galvo center
            return TrackerSnapshot(2048, 2048, 0, 0, 1.0)
        mosquito_trajectory = _stationary_traj

    pat = ConeCollapsePattern(config, mosquito_trajectory(0.0))
    chunk_dt = config.chunk_size / config.dac_rate
    restarts = 0

    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "x_galvo", "y_galvo", "r", "g", "b",
                    "phase", "radius", "stretch", "breach"])
        while not pat.is_done:
            tracker = mosquito_trajectory(pat.total_elapsed_s)
            chunk = pat.next_chunk(tracker)
            if chunk.breach_detected:
                if restarts >= max_restarts:
                    break          # never converged — stop, like engage_cone
                restarts += 1
                # Restart SHRINK from the target's new position.
                pat.cone_x = tracker.target_x_galvo
                pat.cone_y = tracker.target_y_galvo
                pat.phase = Phase.SHRINK
                pat.phase_elapsed_s = 0.0
            t = pat.total_elapsed_s - chunk_dt
            for i, p in enumerate(chunk.points):
                t_sample = t + (i / config.dac_rate)
                w.writerow([f"{t_sample:.6f}", p.x, p.y, p.r, p.g, p.b,
                            chunk.phase.value,
                            f"{chunk.current_radius_galvo:.1f}",
                            f"{chunk.stretch_factor:.2f}",
                            int(chunk.breach_detected)])
    return output_path
