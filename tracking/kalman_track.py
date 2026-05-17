"""TrackedTarget + the constant-velocity Kalman filter math.

State is [x, y, vx, vy] (4-D) or, in kalman_3d mode, [x, y, z, vx, vy, vz]
(6-D). The first half is position, the second half velocity. predict/update
are plain functions over a TrackedTarget so the tracker stays declarative.

Reference: BOOTSTRAP.md §7.2, BOOTSTRAP_AMENDMENTS.md §7.2 (fire_eligible).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from events.schemas import DetectionEvent

# Tracking modes (§7.1).
MODE_KALMAN_PIXEL = "kalman_pixel"
MODE_KALMAN_NORM_DT = "kalman_norm_dt"
MODE_KALMAN_3D = "kalman_3d"
MODE_IOU_ONLY = "iou_only"
TRACKER_MODES = (MODE_KALMAN_PIXEL, MODE_KALMAN_NORM_DT,
                 MODE_KALMAN_3D, MODE_IOU_ONLY)


@dataclass
class TrackedTarget:
    """One tracked object. state/covariance are the Kalman state; the
    lifecycle + confidence fields drive promotion, coasting and dropping."""
    track_id: int
    sensor_id: str
    state: np.ndarray                 # [x,y,vx,vy] or [x,y,z,vx,vy,vz]
    covariance: np.ndarray

    # Lifecycle
    age_frames: int = 0
    hits: int = 0                     # total frames matched to a detection
    confirmed: bool = False
    disappeared_frames: int = 0
    coast_frames: int = 0

    # Confidence — rises on match, decays while coasting.
    confidence: float = 0.5

    # Trajectory history (for visualization).
    history: deque = field(default_factory=lambda: deque(maxlen=64))

    # Last detection + update time (for safety reasoning + freshness).
    last_detection: Optional[DetectionEvent] = None
    last_update_ts: float = 0.0

    # §7.2 amendment — fire eligibility is a surfaced SIGNAL, not a gate.
    fire_eligible: bool = False
    fire_eligible_reason: str = ""

    @property
    def coasting(self) -> bool:
        return self.coast_frames > 0

    @property
    def position(self) -> tuple[float, ...]:
        """Position component of the state (2 or 3 values)."""
        return tuple(self.state[:self._half])

    @property
    def _half(self) -> int:
        return len(self.state) // 2


# ── Construction ─────────────────────────────────────────────────────────

def measurement_dim(mode: str) -> int:
    """Number of measured position components for a mode (2-D or 3-D)."""
    return 3 if mode == MODE_KALMAN_3D else 2


def new_track(track_id: int, det: DetectionEvent, mode: str,
              timestamp: float, history_len: int = 64) -> TrackedTarget:
    """Spawn a fresh track from an unmatched detection."""
    half = measurement_dim(mode)
    state = np.zeros(2 * half, dtype=float)
    state[0], state[1] = det.x_norm, det.y_norm
    if half == 3:
        state[2] = det.z_world_m if det.z_world_m is not None else 0.0
    # Generous initial covariance — the first updates should move the estimate.
    cov = np.eye(2 * half, dtype=float)
    cov[half:, half:] *= 10.0          # velocity is very uncertain at birth
    t = TrackedTarget(
        track_id=track_id, sensor_id=det.sensor_id,
        state=state, covariance=cov,
        history=deque(maxlen=history_len),
        last_detection=det, last_update_ts=timestamp,
        age_frames=1, hits=1,
    )
    t.history.append(tuple(t.position))
    return t


# ── Kalman filter math ───────────────────────────────────────────────────

def _transition(dim: int, dt: float) -> np.ndarray:
    """Constant-velocity state-transition matrix for a 2*half state."""
    F = np.eye(dim, dtype=float)
    half = dim // 2
    for i in range(half):
        F[i, half + i] = dt
    return F


def kalman_predict(t: TrackedTarget, dt: float, proc_noise: float) -> None:
    """Advance the track's state by dt with the constant-velocity model."""
    dim = len(t.state)
    F = _transition(dim, dt)
    Q = np.eye(dim, dtype=float) * proc_noise
    t.state = F @ t.state
    t.covariance = F @ t.covariance @ F.T + Q


def kalman_update(t: TrackedTarget, measurement: np.ndarray,
                  meas_noise: float) -> None:
    """Correct the predicted state with a position measurement."""
    dim = len(t.state)
    half = dim // 2
    H = np.zeros((half, dim), dtype=float)
    for i in range(half):
        H[i, i] = 1.0
    R = np.eye(half, dtype=float) * meas_noise
    z = np.asarray(measurement, dtype=float)[:half]

    innovation = z - H @ t.state
    S = H @ t.covariance @ H.T + R
    K = t.covariance @ H.T @ np.linalg.inv(S)
    t.state = t.state + K @ innovation
    t.covariance = (np.eye(dim) - K @ H) @ t.covariance
