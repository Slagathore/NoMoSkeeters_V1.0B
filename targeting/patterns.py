"""Calibration sweep pattern generators + drag-line streaming.

All generators return points in NORMALIZED GALVO space — floats in [0, 1]²,
where (0,0) and (1,1) are the galvo DAC extremes. Scale to 12-bit wire
coordinates with galvo_norm_to_dac() before sending to the cube.

Reference: BOOTSTRAP.md §8.2, BOOTSTRAP_AMENDMENTS.md §8.7.
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Iterator

from laser.protocol import MAX_SAMPLES_PER_PACKET
from laser.transport import LaserCubeTransport
from laser.types import COORD_MAX, COORD_MIN, LaserPoint


_log = logging.getLogger(__name__)

Point = tuple[float, float]


# ── §8.2 — Pattern generators ────────────────────────────────────────────

def grid_points(rows: int, cols: int, margin: float = 0.2) -> list[Point]:
    """Returns normalized galvo coords for an N×N grid with margin."""
    return [
        (margin + (1 - 2 * margin) * c / max(1, cols - 1),
         margin + (1 - 2 * margin) * r / max(1, rows - 1))
        for r in range(rows) for c in range(cols)
    ]


def halton_points(n: int, base_x: int = 2, base_y: int = 3,
                  margin: float = 0.2) -> list[Point]:
    """N Halton-distributed points in [margin, 1-margin]² of galvo space."""
    def halton(idx: int, base: int) -> float:
        f, r = 1.0, 0.0
        while idx > 0:
            f /= base
            r += f * (idx % base)
            idx //= base
        return r
    span = 1 - 2 * margin
    return [(margin + span * halton(i + 1, base_x),
             margin + span * halton(i + 1, base_y)) for i in range(n)]


def dragline_path(start: Point, end: Point,
                  duration_s: float, dac_rate: int) -> list[Point]:
    """Returns N points along start→end such that scanning at dac_rate
    takes duration_s seconds."""
    n_points = int(duration_s * dac_rate)
    return [(start[0] + (end[0] - start[0]) * i / max(1, n_points - 1),
             start[1] + (end[1] - start[1]) * i / max(1, n_points - 1))
            for i in range(n_points)]


def windmill_path(n_arms: int = 4, n_revolutions: float = 2.0,
                   r_inner: float = 0.05, r_outer: float = 0.45,
                   points_per_arm: int = 50,
                   center: Point = (0.5, 0.5)) -> list[Point]:
    """Twisting windmill calibration sweep (Cole's pattern)."""
    cx, cy = center
    path: list[Point] = []
    total_angle = n_revolutions * 2 * math.pi
    arm_angles = [i * 2 * math.pi / n_arms for i in range(n_arms)]
    for arm_offset in arm_angles:
        for i in range(points_per_arm):
            t = i / max(1, points_per_arm - 1)
            angle = arm_offset + total_angle * t
            r = r_inner + (r_outer - r_inner) * t
            path.append((cx + r * math.cos(angle), cy + r * math.sin(angle)))
    return path


def generate_pattern(name: str, *,
                     grid_rows: int = 5, grid_cols: int = 5,
                     n_points: int = 25,
                     dragline_duration_s: float = 2.0, dac_rate: int = 30000,
                     windmill_arms: int = 4,
                     windmill_revolutions: float = 2.0,
                     margin: float = 0.2) -> list[Point]:
    """Dispatch a pattern name (config CALIBRATION_PATTERN) to its generator.

    'grid' / 'halton' return discrete fire-and-capture points; 'dragline' /
    'windmill' return a continuous path meant for streaming.
    """
    if name == "grid":
        return grid_points(grid_rows, grid_cols, margin)
    if name == "halton":
        return halton_points(n_points, margin=margin)
    if name == "dragline":
        return dragline_path((margin, margin), (1 - margin, 1 - margin),
                             dragline_duration_s, dac_rate)
    if name == "windmill":
        return windmill_path(n_arms=windmill_arms,
                             n_revolutions=windmill_revolutions)
    raise ValueError(f"unknown calibration pattern: {name!r}")


# ── Galvo-space ↔ 12-bit DAC conversion ──────────────────────────────────

def galvo_norm_to_dac(v: float) -> int:
    """Map a normalized galvo coord in [0,1] to a 12-bit DAC value, clamped."""
    return max(COORD_MIN, min(COORD_MAX, int(round(v * COORD_MAX))))


def path_to_laser_points(path: list[Point],
                         r: int, g: int, b: int) -> list[LaserPoint]:
    """Convert a normalized galvo path to wire-ready LaserPoints with a
    fixed RGB (calibration fires at low/operator-set power)."""
    return [LaserPoint(x=galvo_norm_to_dac(px), y=galvo_norm_to_dac(py),
                       r=r, g=g, b=b)
            for px, py in path]


# ── §8.7 — Drag-line streaming with backpressure ─────────────────────────

@dataclass(frozen=True)
class StreamProgress:
    """One streamed chunk. scan_out_est_at lets the calibration controller
    match commanded galvo coords to observed camera pixels by time."""
    sent_at: float
    scan_out_est_at: float
    chunk_start_idx: int
    chunk_size: int


def stream_dragline(transport: LaserCubeTransport,
                    path: list[Point],
                    dac_rate: int,
                    *,
                    r: int = COORD_MAX, g: int = COORD_MAX, b: int = COORD_MAX,
                    target_buffer_free: int = 5000) -> Iterator[StreamProgress]:
    """Stream a long path to the cube as 140-sample chunks, respecting
    ringbuffer backpressure (§8.7).

    The naive dragline_path() can return 60k points for a 2s/30k-pps sweep;
    the cube's ringbuffer holds only 6000. This blocks on get_ringbuf_empty()
    until there is room, then sends one MAX_SAMPLES_PER_PACKET chunk.
    """
    chunk_size = MAX_SAMPLES_PER_PACKET
    points = path_to_laser_points(path, r, g, b)
    i = 0
    while i < len(points):
        free = transport.get_ringbuf_empty()
        if free is not None and free < target_buffer_free:
            # Cube is full enough; wait until ~one chunk worth scans out.
            time.sleep(chunk_size / dac_rate)
            continue
        chunk = points[i:i + chunk_size]
        send_ts = time.monotonic()
        msg_num = transport._next_msg_num()
        if not transport.send_chunk(chunk, msg_num, frame_num=0):
            _log.error("stream_dragline: send_chunk failed at idx %d", i)
            return
        # Samples in this chunk leave the ringbuffer within
        # (chunk_size / dac_rate) seconds of being added.
        yield StreamProgress(
            sent_at=send_ts,
            scan_out_est_at=send_ts + (chunk_size / dac_rate),
            chunk_start_idx=i,
            chunk_size=len(chunk),
        )
        i += chunk_size
