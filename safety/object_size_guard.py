"""Range-aware object-size guard — BOOTSTRAP §10.3.

For each candidate target, estimate its world-space area from pixel area +
depth + sensor field of view. If the target is larger than
`SAFETY_MAX_TARGET_AREA_MM2`, refuse to fire — a hand, a face, or a pet
mistracked as a target is far bigger than any mosquito.

Fail-open without depth, per the spec ("no depth info; don't gate"): the
guard adds safety when depth exists, it never blocks the 2D pipeline.
The spec's reference field of view is the Kinect v2 RGB (~70° horizontal);
callers with a different sensor pass their own `fov_h_rad`.
"""
from __future__ import annotations

import math
from typing import Optional

from config import settings

# Kinect v2 RGB horizontal FoV — the spec's reference sensor (§10.3).
KINECT_RGB_FOV_H_RAD = math.radians(70.0)


def pixel_area_to_world_mm2(pixel_area: float, depth_m: float, *,
                            fov_h_rad: float,
                            frame_width_px: int) -> float:
    """Approximate world-space area (mm²) of a blob of `pixel_area` px²
    at `depth_m`, assuming square pixels and a pinhole camera with
    horizontal FoV `fov_h_rad` across `frame_width_px` pixels."""
    m_per_px = (2.0 * depth_m * math.tan(fov_h_rad / 2.0)) / frame_width_px
    return float(pixel_area) * (m_per_px * 1000.0) ** 2


def check_target_size(detection, *, frame_width_px: int,
                      fov_h_rad: float = KINECT_RGB_FOV_H_RAD,
                      ) -> tuple[bool, Optional[float]]:
    """Returns (ok, estimated_area_mm2).

    ok=True → safe to fire at this target (small enough, or no depth to
    judge by, or the guard is disabled). estimated_area_mm2 is None when
    no estimate was made."""
    if not settings.SAFETY_OBJECT_SIZE_GUARD_ENABLED:
        return True, None
    depth_m = getattr(detection, "z_world_m", None)
    if depth_m is None or depth_m <= 0.0:
        return True, None            # no depth info; don't gate (§10.3)
    area_mm2 = pixel_area_to_world_mm2(
        detection.area_pixels, float(depth_m),
        fov_h_rad=fov_h_rad, frame_width_px=frame_width_px)
    return area_mm2 < settings.SAFETY_MAX_TARGET_AREA_MM2, area_mm2
