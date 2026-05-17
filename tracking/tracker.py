"""Tracker — multi-modal detection-to-track association and lifecycle.

Per frame: predict every track, assign detections (Hungarian), update matched
tracks, coast unmatched ones, spawn tracks for unmatched detections, drop dead
tracks, then recompute fire_eligible for the survivors.

fire_eligible is a computed SIGNAL surfaced to the operator — never an
enforced gate (§7.2 amendment, Cole's "feature, not lockout" directive).

Modes (§7.1): kalman_pixel (fixed dt), kalman_norm_dt (real dt, default),
kalman_3d (adds Z), iou_only (no Kalman, bbox-IoU matching). All modes track
in normalized space so the rest of the pipeline is mode-agnostic; kalman_pixel
differs from kalman_norm_dt only in using a fixed dt.

Reference: BOOTSTRAP.md §7, BOOTSTRAP_AMENDMENTS.md §7.2.
"""
from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from config import settings
from events.schemas import DetectionEvent, TrackEvent
from tracking.assignment import assign
from tracking.kalman_track import (
    MODE_IOU_ONLY, MODE_KALMAN_3D, MODE_KALMAN_PIXEL, TRACKER_MODES,
    TrackedTarget, kalman_predict, kalman_update, measurement_dim, new_track,
)

_log = logging.getLogger(__name__)

_FIXED_DT = 1.0          # kalman_pixel uses a fixed dt
_MAX_DT = 1.0            # clamp real dt so a stalled frame can't blow up predict


class Tracker:
    """Stateful multi-target tracker. One Tracker per tracked sensor stream."""

    def __init__(self, mode: Optional[str] = None):
        self._mode = mode or settings.TRACKER_MODE
        if self._mode not in TRACKER_MODES:
            raise ValueError(f"unknown tracker mode: {self._mode!r}")
        self._tracks: list[TrackedTarget] = []
        self._next_id = 1
        self._last_ts: Optional[float] = None

    @property
    def mode(self) -> str:
        return self._mode

    def tracks(self) -> list[TrackedTarget]:
        return list(self._tracks)

    # ── Per-frame update ─────────────────────────────────────────────────

    def update(self, detections: list[DetectionEvent],
               timestamp: float) -> list[TrackedTarget]:
        """Advance the tracker one frame and return the live tracks."""
        dt = self._frame_dt(timestamp)

        # 1. Predict every existing track forward.
        for t in self._tracks:
            if self._mode != MODE_IOU_ONLY:
                kalman_predict(t, dt, settings.TRACKER_KALMAN_PROC_NOISE)
            t.age_frames += 1

        # 2. Associate detections to tracks.
        matches = assign(self._tracks, detections, self._mode)
        matched_tracks = {ti for ti, _ in matches}
        matched_dets = {di for _, di in matches}

        # 3. Update matched tracks.
        for ti, di in matches:
            self._apply_match(self._tracks[ti], detections[di], timestamp, dt)

        # 4. Coast unmatched tracks.
        for i, t in enumerate(self._tracks):
            if i not in matched_tracks:
                self._apply_coast(t)

        # 5. Spawn tracks for unmatched detections.
        for j, d in enumerate(detections):
            if j not in matched_dets:
                self._tracks.append(new_track(
                    self._next_id, d, self._mode, timestamp,
                    history_len=settings.TRACKER_HISTORY_LENGTH))
                self._next_id += 1

        # 6. Drop dead tracks.
        self._tracks = [t for t in self._tracks if not self._is_dead(t)]

        # 7. Recompute the fire-eligibility signal.
        for t in self._tracks:
            t.fire_eligible, t.fire_eligible_reason = \
                self._compute_fire_eligibility(t, timestamp)

        self._last_ts = timestamp
        return self._tracks

    # ── Frame timing ─────────────────────────────────────────────────────

    def _frame_dt(self, timestamp: float) -> float:
        if self._mode == MODE_KALMAN_PIXEL:
            return _FIXED_DT
        if self._last_ts is None:
            return _FIXED_DT
        return float(np.clip(timestamp - self._last_ts, 1e-3, _MAX_DT))

    # ── Match / coast ────────────────────────────────────────────────────

    def _apply_match(self, t: TrackedTarget, d: DetectionEvent,
                     timestamp: float, dt: float) -> None:
        half = measurement_dim(self._mode)
        meas = [d.x_norm, d.y_norm]
        if half == 3:
            meas.append(d.z_world_m if d.z_world_m is not None else t.state[2])

        if self._mode == MODE_IOU_ONLY:
            # No Kalman: snap position, derive velocity by finite difference.
            prev = t.state[:half].copy()
            t.state[:half] = meas
            t.state[half:] = (np.asarray(meas) - prev) / max(dt, 1e-3)
        else:
            kalman_update(t, np.asarray(meas, dtype=float),
                          settings.TRACKER_KALMAN_MEAS_NOISE)

        t.hits += 1
        t.disappeared_frames = 0
        t.coast_frames = 0
        t.confidence = min(1.0, t.confidence * settings.TRACKER_CONFIDENCE_BOOST)
        t.last_detection = d
        t.last_update_ts = timestamp
        if t.hits >= settings.TRACKER_CONFIRMATION_FRAMES:
            t.confirmed = True
        t.history.append(tuple(t.position))

    def _apply_coast(self, t: TrackedTarget) -> None:
        t.disappeared_frames += 1
        t.coast_frames += 1
        t.confidence *= settings.TRACKER_CONFIDENCE_DECAY
        t.history.append(tuple(t.position))

    def _is_dead(self, t: TrackedTarget) -> bool:
        return (t.disappeared_frames > settings.TRACKER_MAX_DISAPPEARED
                or t.coast_frames > settings.TRACKER_MAX_COAST_FRAMES)

    # ── Fire eligibility — §7.2 amendment ────────────────────────────────

    @staticmethod
    def _compute_fire_eligibility(t: TrackedTarget,
                                  now: float) -> tuple[bool, str]:
        """Pure function: track state → (eligible, human-readable reason).
        Computed and surfaced; the operator decides what to do with it."""
        age_ms = (now - t.last_update_ts) * 1000.0
        if not t.confirmed:
            return False, f"unconfirmed ({t.age_frames} frames)"
        if t.coast_frames > 0:
            return False, f"coasting {t.coast_frames} frames"
        if age_ms > settings.FIRE_ELIGIBILITY_MAX_AGE_MS:
            return False, f"stale {age_ms:.0f}ms"
        if t.confidence < settings.FIRE_ELIGIBILITY_MIN_CONFIDENCE:
            return False, f"conf={t.confidence:.2f}"
        return True, "ok"


def to_track_event(t: TrackedTarget, timestamp: float) -> TrackEvent:
    """Project a TrackedTarget onto the bus TrackEvent schema."""
    half = len(t.state) // 2
    return TrackEvent(
        timestamp=timestamp,
        track_id=t.track_id,
        sensor_id=t.sensor_id,
        state_norm=tuple(t.state[:half]),
        velocity_norm=tuple(t.state[half:]),
        age_frames=t.age_frames,
        confirmed=t.confirmed,
        coasting=t.coasting,
        confidence=t.confidence,
        fire_eligible=t.fire_eligible,
        last_detection_age_ms=(timestamp - t.last_update_ts) * 1000.0,
    )
