"""Kinect-depth-based human-presence check for the safety gate.

Per BOOTSTRAP.md §5, the Kinect's SAFETY role is to find people/pets in
or near the targeting volume. The Kinect's 512×424 depth plane is the
right tool: a person is a roughly-vertical, ~1.5-2 m tall blob with a
foreground depth (typically 0.5-4 m from the sensor); reflections,
ceiling fans, and other dynamic visible-light noise that confuse an
RGB detector don't show up in depth.

This module turns a Kinect depth frame into a list of `DepthBlob` records
flagged as `is_human_sized`. The SafetyModerator inhibits firing whenever
any such blob is present and (optionally) inside a user-configured no-fire
volume.

Reference: BOOTSTRAP.md §5.3 (KinectV2Sensor), §12 (safety).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

_log = logging.getLogger(__name__)


@dataclass
class DepthBlob:
    """A connected component in the Kinect depth plane."""
    x_px: int
    y_px: int
    w_px: int
    h_px: int
    area_px: int
    median_depth_m: float
    is_human_sized: bool

    @property
    def center_px(self) -> tuple[int, int]:
        return (self.x_px + self.w_px // 2, self.y_px + self.h_px // 2)


# Kinect v2 depth intrinsics (matches sensors/kinect_v2.py).
_FX = 365.456
_FY = 365.456


def _pixel_to_metres(pixels: float, depth_m: float, focal: float) -> float:
    """Approximate the metric extent corresponding to `pixels` at depth_m."""
    if focal <= 0.0:
        return 0.0
    return pixels * depth_m / focal


class KinectSafetyCheck:
    """Depth-plane person detector for the safety gate.

    Parameters
    ----------
    min_depth_m, max_depth_m:
        Depth range to consider. The Kinect's working range is ~0.5-4.5 m;
        narrowing this to the room you're defending cuts false positives
        (e.g. you don't care about the person on the other side of a wall).
    min_height_m, max_height_m:
        Metric height range to flag as "human-sized". Defaults bracket
        kids-through-tall-adults.
    min_aspect_h_over_w:
        Persons are roughly taller than wide. Eliminates floor/wall blobs.
    """

    def __init__(self,
                 min_depth_m: float = 0.5,
                 max_depth_m: float = 4.5,
                 min_height_m: float = 0.9,
                 max_height_m: float = 2.2,
                 min_aspect_h_over_w: float = 1.2,
                 min_area_px: int = 1500):
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.min_height_m = min_height_m
        self.max_height_m = max_height_m
        self.min_aspect = min_aspect_h_over_w
        self.min_area_px = min_area_px

    def analyze(self, depth_m: np.ndarray) -> list[DepthBlob]:
        """Return all depth blobs found in the frame, flagged for human-size."""
        if depth_m is None or depth_m.size == 0:
            return []
        if depth_m.ndim != 2:
            _log.warning("kinect_safety_check: expected 2D depth, got shape %s",
                         depth_m.shape)
            return []

        mask = ((depth_m >= self.min_depth_m)
                & (depth_m <= self.max_depth_m)).astype(np.uint8)
        if mask.sum() == 0:
            return []

        # Morphological close to merge speckle-broken silhouettes into one
        # blob; opening to drop salt-and-pepper noise. Kernel 5px is a
        # compromise across the Kinect's 512×424 depth resolution.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        out: list[DepthBlob] = []
        for i in range(1, num):
            x, y, w, h, area = stats[i]
            if area < self.min_area_px:
                continue
            # Median depth inside the blob — robust to a few stray pixels at
            # foreground/background depths inside the silhouette.
            blob_mask = labels[y:y + h, x:x + w] == i
            inside = depth_m[y:y + h, x:x + w][blob_mask]
            inside = inside[inside > 0]
            if inside.size == 0:
                continue
            median_d = float(np.median(inside))

            metric_h = _pixel_to_metres(h, median_d, _FY)
            metric_w = _pixel_to_metres(w, median_d, _FX)
            aspect = (metric_h / metric_w) if metric_w > 1e-6 else float("inf")

            is_human = (self.min_height_m <= metric_h <= self.max_height_m
                        and aspect >= self.min_aspect)
            out.append(DepthBlob(int(x), int(y), int(w), int(h), int(area),
                                 median_d, is_human))
        return out

    def is_person_present(self, depth_m: np.ndarray) -> bool:
        """Convenience: True iff at least one human-sized blob is detected."""
        return any(b.is_human_sized for b in self.analyze(depth_m))

    def person_blobs(self, depth_m: np.ndarray) -> list[DepthBlob]:
        return [b for b in self.analyze(depth_m) if b.is_human_sized]
