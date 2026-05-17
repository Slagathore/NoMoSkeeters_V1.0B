"""Step 6: calibration pattern generators + drag-line streaming."""
from __future__ import annotations

from laser.transports.dry_run import DryRunTransport
from targeting.patterns import (
    dragline_path, generate_pattern, galvo_norm_to_dac, grid_points,
    halton_points, path_to_laser_points, stream_dragline, windmill_path,
)


# ── Generators — dimensional sanity ──────────────────────────────────────

def test_grid_point_count_and_bounds():
    pts = grid_points(5, 5, margin=0.2)
    assert len(pts) == 25
    assert all(0.2 <= x <= 0.8 and 0.2 <= y <= 0.8 for x, y in pts)
    assert pts[0] == (0.2, 0.2)
    assert pts[-1] == (0.8, 0.8)


def test_grid_handles_single_row_col():
    assert len(grid_points(1, 1)) == 1


def test_halton_count_and_bounds():
    pts = halton_points(25, margin=0.2)
    assert len(pts) == 25
    assert all(0.2 <= x <= 0.8 and 0.2 <= y <= 0.8 for x, y in pts)
    # Quasi-random: not a regular grid — points should be distinct.
    assert len(set(pts)) == 25


def test_dragline_count_and_endpoints():
    pts = dragline_path((0.1, 0.1), (0.9, 0.9), duration_s=0.01, dac_rate=30000)
    assert len(pts) == int(0.01 * 30000)
    assert pts[0] == (0.1, 0.1)
    assert abs(pts[-1][0] - 0.9) < 1e-9 and abs(pts[-1][1] - 0.9) < 1e-9


def test_windmill_count():
    pts = windmill_path(n_arms=4, points_per_arm=50)
    assert len(pts) == 200


def test_generate_pattern_dispatch():
    assert len(generate_pattern("grid", grid_rows=3, grid_cols=3)) == 9
    assert len(generate_pattern("halton", n_points=16)) == 16
    assert len(generate_pattern("windmill", windmill_arms=3)) == 150
    assert len(generate_pattern("dragline", dragline_duration_s=0.01,
                                dac_rate=10000)) == 100


def test_generate_pattern_rejects_unknown():
    import pytest
    with pytest.raises(ValueError):
        generate_pattern("spiral")


# ── DAC conversion ───────────────────────────────────────────────────────

def test_galvo_norm_to_dac_clamps():
    assert galvo_norm_to_dac(0.0) == 0
    assert galvo_norm_to_dac(1.0) == 0xFFF
    assert galvo_norm_to_dac(0.5) == round(0.5 * 0xFFF)
    assert galvo_norm_to_dac(-0.5) == 0           # clamp low
    assert galvo_norm_to_dac(2.0) == 0xFFF        # clamp high


def test_path_to_laser_points_carries_rgb():
    pts = path_to_laser_points([(0.0, 0.0), (1.0, 1.0)], r=0xFFF, g=0, b=0)
    assert pts[0].x == 0 and pts[0].y == 0
    assert pts[1].x == 0xFFF and pts[1].y == 0xFFF
    assert all(p.r == 0xFFF and p.g == 0 and p.b == 0 for p in pts)


# ── §8.7 drag-line streaming ─────────────────────────────────────────────

def test_stream_dragline_chunks_path_and_yields_progress():
    transport = DryRunTransport()
    transport.connect()
    path = [(i / 300.0, 0.5) for i in range(300)]   # 3 packets @ 140
    progress = list(stream_dragline(transport, path, dac_rate=30000))
    assert len(progress) == 3
    assert [p.chunk_size for p in progress] == [140, 140, 20]
    assert [p.chunk_start_idx for p in progress] == [0, 140, 280]
    # scan_out estimate is after the send.
    assert all(p.scan_out_est_at > p.sent_at for p in progress)
    assert transport.packet_count == 3
