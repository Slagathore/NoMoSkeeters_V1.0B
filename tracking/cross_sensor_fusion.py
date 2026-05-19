"""Cross-sensor track-level fusion.

When two sensors run with the `targeting` role, their detections must be
associated to the same physical object across time-of-arrival differences.
Neither sensor reports hardware timestamps (we tag with `time.monotonic()` at
receipt) and each carries its own USB/network jitter, so raw timestamp
comparison is unreliable.

The fix (BOOTSTRAP_AMENDMENTS_v0.2.1 §5.7): **alignment happens at the track
level, not the detection level.** Each sensor's detections feed a lightweight
per-sensor sub-tracker. To check whether two sensors see the same object, each
sub-track is projected to a shared reference timestamp using its own velocity
estimate, then positions are compared in a shared coordinate space.

The shared coordinate space is the destination sensor's normalized frame: if a
`CrossSensorExtrinsic` is supplied, detections from its `src_sensor` are
projected into `dst_sensor` norm space at ingest, so `fuse()` compares
apples-to-apples (the §5.7 sketch's `fuse()` compares raw norm positions).

The crucial property: fusion only ever combines tracks from *different*
sensors. Same-sensor tracks are handled by that sensor's local mini-tracker
(and the system tracker downstream). This avoids the degenerate case where one
sensor's noise self-fuses into a phantom track.

Reference: BOOTSTRAP_AMENDMENTS_v0.2.1.md §5.7.
"""
from __future__ import annotations

import logging
import math
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

import numpy as np

from config import settings
from events.schemas import DetectionEvent

if TYPE_CHECKING:                                          # avoid import cost
    from targeting.extrinsics import CrossSensorExtrinsic

_log = logging.getLogger(__name__)


@dataclass
class SensorLocalState:
    """One sensor's view of one tracked object — minimal Kalman or even just
    last 2-3 detections with linear extrapolation."""
    sensor_id: str
    history: deque   # of (timestamp, x_norm, y_norm) — bounded length
    # Velocity estimated from the last N entries:
    vx_norm_per_s: float = 0.0
    vy_norm_per_s: float = 0.0

    def project_to(self, t_ref: float) -> tuple[float, float]:
        """Linear extrapolation/interpolation to t_ref using last-known
        position and current velocity estimate. Cheap and good enough for
        ±50ms windows."""
        if not self.history:
            return (float("nan"), float("nan"))
        t_last, x_last, y_last = self.history[-1]
        dt = t_ref - t_last
        return (x_last + self.vx_norm_per_s * dt,
                y_last + self.vy_norm_per_s * dt)

    def update_velocity(self, min_samples: int = 3) -> None:
        """Refit velocity from the last min_samples points using linear
        regression. Cheap; runs once per detection."""
        if len(self.history) < min_samples:
            return
        ts = np.array([h[0] for h in self.history])
        xs = np.array([h[1] for h in self.history])
        ys = np.array([h[2] for h in self.history])
        # np.polyfit degree 1 → slope is velocity. A degenerate window (every
        # sample at the same timestamp) makes the fit singular — keep the
        # previous estimate rather than emitting a NaN velocity.
        if np.ptp(ts) <= 0.0:
            return
        self.vx_norm_per_s = float(np.polyfit(ts, xs, 1)[0])
        self.vy_norm_per_s = float(np.polyfit(ts, ys, 1)[0])


@dataclass
class FusedTrack:
    """One fused observation at a shared reference timestamp.

    `contributing_sensors` has two entries when two sensors' sub-tracks were
    associated, one entry for a solo (single-sensor) track. `position_norm` is
    in the shared (destination-sensor) normalized frame.
    """
    t_ref: float
    position_norm: tuple[float, float]
    contributing_sensors: list[str]
    assoc_distance_norm: float = 0.0

    @property
    def fused(self) -> bool:
        """True when two sensors contributed — i.e. cross-sensor confirmed."""
        return len(self.contributing_sensors) >= 2

    @classmethod
    def solo(cls, state: SensorLocalState, t_ref: float) -> "FusedTrack":
        """A FusedTrack carrying a single sensor's sub-track, unmatched."""
        return cls(t_ref=t_ref,
                   position_norm=state.project_to(t_ref),
                   contributing_sensors=[state.sensor_id],
                   assoc_distance_norm=0.0)


def _passthrough(sensor_states: dict[tuple[str, int], SensorLocalState],
                 t_ref: float) -> list[FusedTrack]:
    """Emit every sub-track solo — used when fewer than two sensors are
    present, so no cross-sensor association is possible."""
    return [FusedTrack.solo(s, t_ref)
            for s in sensor_states.values() if s.history]


class CrossSensorFusion:
    """Maintains per-sensor sub-tracks and fuses them across sensors at
    track level. Emits a unified FusedTrack stream to the rest of the
    system."""

    def __init__(self,
                 max_assoc_distance_norm: float
                 = settings.FUSION_MAX_ASSOC_DISTANCE_NORM,
                 history_length: int = settings.FUSION_HISTORY_LENGTH,
                 *,
                 extrinsic: Optional["CrossSensorExtrinsic"] = None,
                 min_velocity_samples: int
                 = settings.FUSION_MIN_VELOCITY_SAMPLES,
                 projection_max_dt_ms: float
                 = settings.FUSION_PROJECTION_MAX_DT_MS):
        self._sensor_states: dict[tuple[str, int], SensorLocalState] = {}
        self.max_assoc_distance_norm = max_assoc_distance_norm
        self.history_length = history_length
        self._extrinsic = extrinsic
        self._min_velocity_samples = min_velocity_samples
        self._max_dt_s = projection_max_dt_ms / 1000.0
        # Monotonically increasing local sub-track id, per sensor.
        self._next_local_id: dict[str, int] = defaultdict(int)

    # ── Ingest ───────────────────────────────────────────────────────────

    def _to_shared_frame(self, det: DetectionEvent) -> tuple[float, float]:
        """Detection norm coords mapped into the shared (dst-sensor) frame.

        Only the extrinsic's `src_sensor` is remapped; the `dst_sensor` (and
        any sensor with no extrinsic) is already in the shared frame."""
        if (self._extrinsic is not None
                and det.sensor_id == self._extrinsic.src_sensor):
            px, py = self._extrinsic.project(det.x_norm, det.y_norm)
            return (float(px), float(py))
        return (float(det.x_norm), float(det.y_norm))

    def ingest_detection(self, det: DetectionEvent) -> None:
        """Append a detection to its sensor-local sub-track.

        The per-sensor mini-tracker is a distance matcher: the detection joins
        the nearest existing sub-track for its sensor (projected to the
        detection's timestamp) when within `max_assoc_distance_norm`, else a
        fresh sub-track id is allocated. Cross-sensor association happens in
        `fuse()` — never here.
        """
        t = det.timestamp
        x, y = self._to_shared_frame(det)
        self._prune(now=t)

        best_state: Optional[SensorLocalState] = None
        best_dist = float("inf")
        for (sid, _local), state in self._sensor_states.items():
            if sid != det.sensor_id or not state.history:
                continue
            px, py = state.project_to(t)
            d = math.hypot(px - x, py - y)
            if d < best_dist:
                best_dist = d
                best_state = state

        if best_state is not None and best_dist <= self.max_assoc_distance_norm:
            state = best_state
        else:
            local_id = self._next_local_id[det.sensor_id]
            self._next_local_id[det.sensor_id] += 1
            state = SensorLocalState(
                sensor_id=det.sensor_id,
                history=deque(maxlen=self.history_length))
            self._sensor_states[(det.sensor_id, local_id)] = state

        state.history.append((t, x, y))
        state.update_velocity(self._min_velocity_samples)

    def _prune(self, now: float) -> None:
        """Drop sub-tracks whose last detection is older than the projection
        window — extrapolating past it is not trustworthy (§5.7 settings)."""
        stale = [key for key, s in self._sensor_states.items()
                 if not s.history or now - s.history[-1][0] > self._max_dt_s]
        for key in stale:
            del self._sensor_states[key]

    # ── Fuse ─────────────────────────────────────────────────────────────

    def fuse(self, t_ref: Optional[float] = None) -> list[FusedTrack]:
        """For every pair of sensor sub-tracks (across different sensors),
        project each to t_ref and check whether their projected positions
        are within max_assoc_distance_norm.

        If yes — emit one FusedTrack with attributes from both sensors.
        If no  — each sub-track emits its own FusedTrack solo.

        t_ref defaults to "the most recent timestamp across all sensors."
        """
        if t_ref is None:
            t_ref = max(
                (s.history[-1][0] for s in self._sensor_states.values()
                 if s.history),
                default=0.0,
            )

        # Group by sensor
        by_sensor: dict[str, list[SensorLocalState]] = defaultdict(list)
        for state in self._sensor_states.values():
            by_sensor[state.sensor_id].append(state)

        fused = []
        sensors = list(by_sensor.keys())
        if len(sensors) < 2:
            # Only one sensor — no cross-fusion possible. Pass through.
            return _passthrough(self._sensor_states, t_ref)

        # All-pairs association across sensors. For 2 sensors with N tracks
        # each it's O(N^2) — fine for mosquito counts in single digits.
        used: set = set()
        for sa in by_sensor[sensors[0]]:
            best_match = None
            best_dist = float("inf")
            xa, ya = sa.project_to(t_ref)
            for sb in by_sensor[sensors[1]]:
                if id(sb) in used:
                    continue
                xb, yb = sb.project_to(t_ref)
                d = ((xa - xb) ** 2 + (ya - yb) ** 2) ** 0.5
                if d < best_dist and d < self.max_assoc_distance_norm:
                    best_dist = d
                    best_match = sb
            if best_match is not None:
                used.add(id(best_match))
                fused.append(FusedTrack(
                    t_ref=t_ref,
                    position_norm=((xa + best_match.project_to(t_ref)[0]) / 2,
                                   (ya + best_match.project_to(t_ref)[1]) / 2),
                    contributing_sensors=[sa.sensor_id, best_match.sensor_id],
                    assoc_distance_norm=best_dist,
                ))
            else:
                fused.append(FusedTrack.solo(sa, t_ref))
        # Add unmatched tracks from the second sensor
        for sb in by_sensor[sensors[1]]:
            if id(sb) not in used:
                fused.append(FusedTrack.solo(sb, t_ref))
        return fused

    # ── Introspection ────────────────────────────────────────────────────

    def sub_track_count(self) -> int:
        """Number of live per-sensor sub-tracks (after the last ingest)."""
        return len(self._sensor_states)

    def clear(self) -> None:
        """Drop all sub-tracks. Local id counters are kept so a re-ingested
        object does not collide with a just-pruned sub-track's id."""
        self._sensor_states.clear()
