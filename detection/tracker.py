"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Tracker Module
--------------
Provides multi-object tracking with Kalman filters and Hungarian algorithm
assignment.  Converts the raw per-frame detection list into a set of persistent
tracks with IDs, velocity estimates, and future position predictions.

Algorithm overview:
  1. Receive new frame's list of Detection objects.
  2. Predict each existing track's new position with its Kalman filter.
  3. Build a cost matrix: Euclidean distance between predicted-track centroids
     and new detection centroids.
  4. Use scipy.optimize.linear_sum_assignment (Hungarian algorithm) to find
     the minimum-cost assignment.
  5. Matched pairs update their track's Kalman filter with the measurement.
  6. Unmatched detections spawn new tracks.
  7. Unmatched tracks increment a "disappeared" counter; when it exceeds
     TRACKER_MAX_DISAPPEARED the track is removed.
  8. Each alive track exposes a predicted_position() for targeting.

Kalman filter state vector: [cx, cy, vx, vy]
  cx, cy — centroid in camera pixels
  vx, vy — velocity in pixels/frame

Modules imported:
  numpy               — matrices, linear algebra
  scipy.optimize      — Hungarian algorithm assignment
  dataclasses         — TrackedTarget dataclass
  config.settings     — TRACKER_* constants
  detection.detector  — Detection namedtuple
  utils.logging_utils — get_logger

Classes:
  TrackedTarget       — Dataclass holding a single track's state.
  MosquitoTracker     — Stateful tracker that manages all live tracks.

Methods (MosquitoTracker):
  update(detections)  — Feed new detections; returns updated list of live tracks.
  get_best_target()   — Return the track with the highest "priority" for targeting.
  reset()             — Clear all tracks.

Variables (TrackedTarget):
  track_id      (int):   Unique ID assigned at creation.
  cx, cy        (float): Current estimated centroid (from Kalman state).
  vx, vy        (float): Estimated velocity (px/frame).
  disappeared   (int):   Frames since last measurement match.
  age           (int):   Total frames since track was created.
  last_detection(Detection | None): Most recent matched Detection.
  _kf_state     (np.ndarray): Kalman [cx,cy,vx,vy] state vector.
  _kf_P         (np.ndarray): Kalman error covariance 4×4 matrix.

#todo Add 3-D localisation using two cameras (stereo) for vertical prediction.
#todo Classify track movement signature to improve mosquito vs. not-mosquito.
#todo Expose track history (trajectory polyline) to the GUI overlay.
#todo Add a confidence score per track based on detection consistency.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

import config.settings as _s
from detection.detector import Detection
from utils.logging_utils import get_logger

logger = get_logger(__name__)

# Try scipy for Hungarian; fall back to a greedy nearest-neighbour assignment
try:
    from scipy.optimize import linear_sum_assignment as _hungarian
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False
    logger.warning("scipy not available — using greedy assignment fallback.")


# ─── Kalman filter helpers ────────────────────────────────────────────────────

def _init_kalman(cx: float, cy: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Initialise Kalman filter state and covariance for a new track.

    State vector: [cx, cy, vx, vy]

    Args:
        cx: Initial centroid X in pixels.
        cy: Initial centroid Y in pixels.

    Returns:
        Tuple of (state_vector [4,], covariance_matrix [4,4]).
    """
    state = np.array([cx, cy, 0.0, 0.0], dtype=np.float64)
    P = np.eye(4, dtype=np.float64) * 500.0  # high initial uncertainty
    return state, P


def _kalman_predict(
    state: np.ndarray,
    P: np.ndarray,
    dt: float = 1.0,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Kalman prediction step (constant-velocity model).

    Args:
        state: [cx, cy, vx, vy] current state.
        P:     4×4 error covariance matrix.
        dt:    Time step in frames (normally 1).

    Returns:
        (predicted_state, predicted_P)
    """
    Q = np.eye(4) * _s.TRACKER_KALMAN_PROC_NOISE  # process noise

    # State transition: position += velocity * dt
    F = np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1,  0],
        [0, 0, 0,  1],
    ], dtype=np.float64)

    state_pred = F @ state
    P_pred = F @ P @ F.T + Q
    return state_pred, P_pred


def _kalman_update(
    state_pred: np.ndarray,
    P_pred: np.ndarray,
    measurement: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Kalman update step given a new cx,cy measurement.

    Args:
        state_pred:   Predicted state [cx, cy, vx, vy].
        P_pred:       Predicted covariance 4×4.
        measurement:  Observed [cx, cy] array.

    Returns:
        (updated_state, updated_P)
    """
    R = np.eye(2) * _s.TRACKER_KALMAN_MEAS_NOISE  # measurement noise

    # Observation matrix: only cx,cy are observed
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
    ], dtype=np.float64)

    y = measurement - H @ state_pred           # innovation
    S = H @ P_pred @ H.T + R                   # innovation covariance
    K = P_pred @ H.T @ np.linalg.inv(S)        # Kalman gain
    state_updated = state_pred + K @ y
    P_updated = (np.eye(4) - K @ H) @ P_pred
    return state_updated, P_updated


# ─── TrackedTarget dataclass ──────────────────────────────────────────────────

@dataclass
class TrackedTarget:
    """
    Represents a single mosquito track over time.

    Attributes:
        track_id       (int):   Globally unique track identifier.
        cx             (float): Current estimated centroid X (pixels).
        cy             (float): Current estimated centroid Y (pixels).
        vx             (float): Estimated velocity X (px/frame).
        vy             (float): Estimated velocity Y (px/frame).
        disappeared    (int):   Consecutive frames with no matched detection.
        age            (int):   Total frame count since this track was created.
        last_detection (Detection | None): Most recently matched Detection.
        created_at     (float): time.monotonic() when track was created.
        _kf_state      (np.ndarray): Kalman state [cx, cy, vx, vy].
        _kf_P          (np.ndarray): Kalman 4×4 error covariance.
    """
    track_id: int
    cx: float
    cy: float
    vx: float = 0.0
    vy: float = 0.0
    disappeared: int = 0
    age: int = 0
    last_detection: Optional[Detection] = None
    created_at: float = field(default_factory=time.monotonic)
    _kf_state: np.ndarray = field(init=False, repr=False)
    _kf_P: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._kf_state, self._kf_P = _init_kalman(self.cx, self.cy)

    def predict(self, dt: float = 1.0) -> tuple[float, float]:
        """
        Advance the Kalman filter by one step and return the predicted centroid.

        Args:
            dt: Time step multiplier (use 1.0 for per-frame prediction).

        Returns:
            (predicted_cx, predicted_cy) in camera pixels.
        """
        self._kf_state, self._kf_P = _kalman_predict(self._kf_state, self._kf_P, dt)
        self.cx = float(self._kf_state[0])
        self.cy = float(self._kf_state[1])
        self.vx = float(self._kf_state[2])
        self.vy = float(self._kf_state[3])
        return self.cx, self.cy

    def update(self, detection: Detection) -> None:
        """
        Update the Kalman filter with a matched detection measurement.

        Args:
            detection: The Detection that was matched to this track.
        """
        meas = np.array([detection.cx, detection.cy], dtype=np.float64)
        self._kf_state, self._kf_P = _kalman_update(
            self._kf_state, self._kf_P, meas
        )
        self.cx = float(self._kf_state[0])
        self.cy = float(self._kf_state[1])
        self.vx = float(self._kf_state[2])
        self.vy = float(self._kf_state[3])
        self.disappeared = 0
        self.last_detection = detection
        self.age += 1

    def predicted_position(self, predict_ms: int = _s.TARGETING_PREDICT_MS) -> tuple[float, float]:
        """
        Return where this target will be in *predict_ms* milliseconds.

        Assumes a constant frame rate of DETECTION_FPS_TARGET.

        Args:
            predict_ms: Milliseconds to look ahead.

        Returns:
            (predicted_cx, predicted_cy) in camera pixels.
        """
        frames_ahead = (predict_ms / 1000.0) * _s.DETECTION_FPS_TARGET
        pred_cx = self.cx + self.vx * frames_ahead
        pred_cy = self.cy + self.vy * frames_ahead
        return pred_cx, pred_cy


# ─── Tracker ──────────────────────────────────────────────────────────────────

class MosquitoTracker:
    """
    Stateful multi-object tracker using Kalman filters and Hungarian assignment.

    Call update(detections) once per frame to advance the tracker.

    Attributes:
        _tracks    (dict[int, TrackedTarget]): Active tracks keyed by track_id.
        _next_id   (int): Auto-incrementing ID counter for new tracks.
    """

    def __init__(self) -> None:
        self._tracks: dict[int, TrackedTarget] = {}
        self._next_id: int = _s.TRACKER_NEXT_ID_START

    # ──────────────────────────────────────────────────────────────────────────
    # Main update
    # ──────────────────────────────────────────────────────────────────────────

    def update(self, detections: list[Detection]) -> list[TrackedTarget]:
        """
        Advance the tracker by one frame.

        Steps:
          1. Predict all existing tracks forward.
          2. Assign detections to tracks via Hungarian algorithm.
          3. Update matched tracks; create new tracks for unmatched detections.
          4. Increment disappeared counter for unmatched tracks; prune expired.
          5. Return list of all currently alive tracks.

        Args:
            detections: Detections from current frame (may be empty list).

        Returns:
            List of TrackedTarget objects representing all live tracks.
        """
        # 1. Predict all tracks forward
        for track in self._tracks.values():
            track.predict()

        if not detections:
            # No detections this frame — just age all tracks
            to_delete = []
            for tid, track in self._tracks.items():
                track.disappeared += 1
                if track.disappeared > _s.TRACKER_MAX_DISAPPEARED:
                    to_delete.append(tid)
            for tid in to_delete:
                logger.debug("Track #%d pruned (disappeared).", tid)
                del self._tracks[tid]
            return list(self._tracks.values())

        if not self._tracks:
            # No existing tracks — create one for every detection
            for det in detections:
                self._create_track(det)
            return list(self._tracks.values())

        # 2. Build cost matrix (Euclidean distance: predicted tracks × detections)
        track_ids = list(self._tracks.keys())
        track_list = [self._tracks[tid] for tid in track_ids]

        cost = np.zeros((len(track_list), len(detections)), dtype=np.float64)
        for i, track in enumerate(track_list):
            for j, det in enumerate(detections):
                dx = track.cx - det.cx
                dy = track.cy - det.cy
                cost[i, j] = np.sqrt(dx * dx + dy * dy)

        # 3. Assignment
        matched_track_idx, matched_det_idx = self._assign(cost)

        matched_track_ids: set[int] = set()
        matched_det_indices: set[int] = set()

        for ti, di in zip(matched_track_idx, matched_det_idx):
            if cost[ti, di] > _s.TRACKER_MAX_DISTANCE:
                continue  # too far — treat as unmatched
            track = track_list[ti]
            track.update(detections[di])
            matched_track_ids.add(track_ids[ti])
            matched_det_indices.add(di)

        # 4. Unmatched tracks — increment disappeared
        to_delete = []
        for tid in track_ids:
            if tid not in matched_track_ids:
                self._tracks[tid].disappeared += 1
                if self._tracks[tid].disappeared > _s.TRACKER_MAX_DISAPPEARED:
                    to_delete.append(tid)
        for tid in to_delete:
            logger.debug("Track #%d expired.", tid)
            del self._tracks[tid]

        # 5. Unmatched detections — spawn new tracks
        for di, det in enumerate(detections):
            if di not in matched_det_indices:
                self._create_track(det)

        return list(self._tracks.values())

    # ──────────────────────────────────────────────────────────────────────────
    # Best target selection
    # ──────────────────────────────────────────────────────────────────────────

    def get_best_target(self) -> TrackedTarget | None:
        """
        Return the single highest-priority live track for laser targeting.

        Priority heuristic: oldest track (most confident, not just a noise blip)
        that has not disappeared recently.

        Returns:
            TrackedTarget or None if no tracks exist.
        """
        candidates = [
            t for t in self._tracks.values()
            if t.disappeared == 0 and t.age >= 2
        ]
        if not candidates:
            return None
        # Prefer oldest track (most confirmed)
        return max(candidates, key=lambda t: t.age)

    def reset(self) -> None:
        """Clear all active tracks."""
        self._tracks.clear()
        self._next_id = _s.TRACKER_NEXT_ID_START
        logger.info("Tracker reset — all tracks cleared.")

    def live_track_count(self) -> int:
        """Return the number of currently active tracks."""
        return len(self._tracks)

    # ──────────────────────────────────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _create_track(self, detection: Detection) -> TrackedTarget:
        """
        Spawn a new TrackedTarget from a new unmatched detection.

        Args:
            detection: Unmatched Detection from the current frame.

        Returns:
            The newly created TrackedTarget.
        """
        track = TrackedTarget(
            track_id=self._next_id,
            cx=detection.cx,
            cy=detection.cy,
        )
        track.update(detection)
        self._tracks[self._next_id] = track
        logger.debug("New track #%d at (%.0f, %.0f).", self._next_id, detection.cx, detection.cy)
        self._next_id += 1
        return track

    @staticmethod
    def _assign(
        cost: np.ndarray,
    ) -> tuple[list[int], list[int]]:
        """
        Solve the assignment problem on the cost matrix.

        Uses scipy.optimize.linear_sum_assignment (Hungarian) if available,
        otherwise falls back to a greedy nearest-neighbour approach.

        Args:
            cost: (num_tracks, num_detections) cost matrix.

        Returns:
            Tuple of (track_indices, detection_indices) matched pairs.
        """
        if _HAS_SCIPY:
            row_ind, col_ind = _hungarian(cost)
            return list(row_ind), list(col_ind)

        # Greedy fallback: match closest pairs iteratively
        rows, cols = [], []
        used_cols: set[int] = set()
        flat = list(np.ndindex(cost.shape))
        flat.sort(key=lambda idx: cost[idx])
        used_rows: set[int] = set()
        for r, c in flat:
            if r in used_rows or c in used_cols:
                continue
            rows.append(r)
            cols.append(c)
            used_rows.add(r)
            used_cols.add(c)
        return rows, cols
