"""Detection↔track assignment — Hungarian with a greedy fallback.

Hungarian (scipy.optimize.linear_sum_assignment) is optimal; the greedy
nearest-cost fallback runs if scipy is unavailable. Invalid pairs (beyond the
gate) carry a large finite cost rather than inf so the solver stays feasible,
then are filtered out of the result.

Reference: BOOTSTRAP.md §7.4.
"""
from __future__ import annotations

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
    HAS_SCIPY = True
except ImportError:                                   # pragma: no cover
    HAS_SCIPY = False

from config import settings
from events.schemas import DetectionEvent
from tracking.kalman_track import MODE_IOU_ONLY, TrackedTarget

# Sentinel cost for a pair outside the gate — large but finite.
_BLOCKED = 1.0e6


def _bbox_iou(a: tuple, b: tuple) -> float:
    """Intersection-over-union of two (x, y, w, h) boxes."""
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix0, iy0 = max(ax, bx), max(ay, by)
    ix1, iy1 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    iw, ih = max(0, ix1 - ix0), max(0, iy1 - iy0)
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def build_cost_matrix(tracks: list[TrackedTarget],
                      detections: list[DetectionEvent],
                      mode: str) -> np.ndarray:
    """Cost matrix (n_tracks × n_detections). iou_only uses 1−IoU on the
    bounding boxes; every other mode uses normalized-space distance."""
    cost = np.full((len(tracks), len(detections)), _BLOCKED, dtype=float)
    for i, t in enumerate(tracks):
        for j, d in enumerate(detections):
            if mode == MODE_IOU_ONLY:
                if t.last_detection is None:
                    continue
                iou = _bbox_iou(t.last_detection.bbox, d.bbox)
                if iou > 0.0:
                    cost[i, j] = 1.0 - iou
            else:
                dist = float(np.hypot(t.state[0] - d.x_norm,
                                      t.state[1] - d.y_norm))
                if dist < settings.TRACKER_MAX_DISTANCE_NORM:
                    cost[i, j] = dist
    return cost


def _greedy_assign(cost: np.ndarray) -> list[tuple[int, int]]:
    """Repeatedly take the global minimum valid cell."""
    matches: list[tuple[int, int]] = []
    used_rows: set[int] = set()
    used_cols: set[int] = set()
    work = cost.copy()
    while True:
        idx = int(np.argmin(work))
        r, c = divmod(idx, work.shape[1])
        if work[r, c] >= _BLOCKED:
            break
        matches.append((r, c))
        used_rows.add(r)
        used_cols.add(c)
        work[r, :] = _BLOCKED
        work[:, c] = _BLOCKED
        if len(used_rows) == cost.shape[0] or len(used_cols) == cost.shape[1]:
            break
    return matches


def assign(tracks: list[TrackedTarget],
           detections: list[DetectionEvent],
           mode: str) -> list[tuple[int, int]]:
    """Return (track_idx, detection_idx) matches within the gate."""
    if not tracks or not detections:
        return []
    cost = build_cost_matrix(tracks, detections, mode)
    if HAS_SCIPY:
        rows, cols = linear_sum_assignment(cost)
        return [(int(r), int(c)) for r, c in zip(rows, cols)
                if cost[r, c] < _BLOCKED]
    return _greedy_assign(cost)
