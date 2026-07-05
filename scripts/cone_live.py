#!/usr/bin/env python3
"""cone_live.py — fire the cone-collapse pattern on the real LaserCube.

This is the live-hardware counterpart to scripts/cone_demo.py. It uses the
same LaserManager.engage_cone() path, but with LaserCubeInterface instead of
DryRunTransport.

The target is synthetic and expressed directly in galvo space: still, moving,
or jinking. Use --mosquito-5s for a longer piecewise path that changes
direction several times before the final shot. This script is for visually
inspecting the pattern on a safe matte target surface before the tracker is
allowed to drive it.

>>> Class 3B laser. Eye protection + safety lens REQUIRED. <<<

    python scripts/cone_live.py --power-pct 5
    python scripts/cone_live.py --trajectory moving --power-pct 5
    python scripts/cone_live.py --mosquito-5s --power-pct 5
    python scripts/cone_live.py --mosquito-5s --mosquito-speed 1.5 --power-pct 5
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from config import settings                                        # noqa: E402
from laser.laser_manager import LaserManager                       # noqa: E402
from laser.lasercube import LaserCubeInterface                     # noqa: E402
from laser.shot_patterns import ConeCollapseConfig, TrackerSnapshot  # noqa: E402

_COORD_MAX = 0xFFF


def _clamp_galvo(v: float) -> float:
    return max(0.0, min(float(_COORD_MAX), v))


def _norm_to_galvo(v: float) -> float:
    return _clamp_galvo(float(v) * _COORD_MAX)


def _scale_color(rgb: tuple[int, int, int], power_pct: float) -> tuple[int, int, int]:
    scale = max(0.0, min(100.0, power_pct)) / 100.0
    scaled = [max(0, min(_COORD_MAX, int(round(c * scale)))) for c in rgb]
    return (scaled[0], scaled[1], scaled[2])


def _apply_power_scale(cfg: ConeCollapseConfig,
                       acquire_power_pct: float,
                       bzzt_power_pct: float) -> ConeCollapseConfig:
    cfg.color_shrink = _scale_color(cfg.color_shrink, acquire_power_pct)
    cfg.color_line = _scale_color(cfg.color_line, acquire_power_pct)
    cfg.color_bzzt = _scale_color(cfg.color_bzzt, bzzt_power_pct)
    cfg.power_shrink = max(cfg.color_shrink)
    cfg.power_line = max(cfg.color_line)
    cfg.power_bzzt = max(cfg.color_bzzt)
    return cfg


def _preflight(cube: LaserCubeInterface) -> bool:
    info = cube.get_full_info()
    if info is None:
        print("FAIL: no GET_FULL_INFO response")
        return False
    print(f"  firmware     : {info.fw_major}.{info.fw_minor}")
    print(f"  interlock    : {info.interlock}")
    print(f"  output now   : {info.output_enabled}")
    print(f"  over_temp    : {info.over_temp}")
    print(f"  temp_warn    : {info.temp_warn}")
    print(f"  temperature  : {info.temperature_c} degC")
    print(f"  buffer free  : {info.buffer_free}/{info.buffer_size}")
    if not info.interlock:
        print("FAIL: interlock not satisfied")
        return False
    if info.over_temp:
        print("FAIL: cube reports over-temperature")
        return False
    return True


def _confirm(prompt: str) -> bool:
    return input(f"{prompt} [type 'yes' to proceed] ").strip().lower() == "yes"


def _snapshot_factory(trajectory: str, x0: float, y0: float, extent: float,
                      mosquito_speed: float):
    start = time.monotonic()

    def still(_: float) -> TrackerSnapshot:
        return TrackerSnapshot(x0, y0, 0.0, 0.0, 1.0)

    def moving(t_s: float) -> TrackerSnapshot:
        vx = 300.0
        return TrackerSnapshot(
            _clamp_galvo(x0 - extent * 0.4 + vx * t_s),
            y0, vx, 0.0, 1.0)

    def jinking(t_s: float) -> TrackerSnapshot:
        wx = 8.0
        wy = 6.0
        x = x0 + extent * math.sin(t_s * wx)
        y = y0 + extent * 0.65 * math.cos(t_s * wy)
        vx = extent * wx * math.cos(t_s * wx)
        vy = -extent * 0.65 * wy * math.sin(t_s * wy)
        return TrackerSnapshot(_clamp_galvo(x), _clamp_galvo(y), vx, vy, 1.0)

    def mosquito5(t_s: float) -> TrackerSnapshot:
        # Five-second burst path: short straight-line darts, abrupt turns,
        # and a final settle near center so line/dark/bzzt land visibly.
        #
        # At mosquito_speed=1.0 and extent=260, most bursts are roughly
        # 900-1600 galvo units/s. Increase --mosquito-speed to compress the
        # same path in time, producing faster apparent flight and more
        # breach/reacquire behavior.
        waypoints = [
            (0.00, -0.85, -0.15),
            (0.34,  0.15, -0.75),
            (0.72,  0.90, -0.05),
            (1.10, -0.15,  0.70),
            (1.48, -0.95,  0.05),
            (1.86,  0.35, -0.60),
            (2.24,  0.95,  0.50),
            (2.68, -0.55,  0.65),
            (3.10,  0.65, -0.30),
            (3.55, -0.25, -0.80),
            (4.05,  0.80,  0.10),
            (4.55, -0.35,  0.35),
            (5.00,  0.00,  0.00),
        ]
        t_s *= max(0.1, mosquito_speed)
        if t_s <= waypoints[0][0]:
            _, px, py = waypoints[0]
            return TrackerSnapshot(
                _clamp_galvo(x0 + px * extent),
                _clamp_galvo(y0 + py * extent),
                0.0, 0.0, 1.0)
        for (t0, x_a, y_a), (t1, x_b, y_b) in zip(waypoints, waypoints[1:]):
            if t_s <= t1:
                span = max(0.001, t1 - t0)
                u = max(0.0, min(1.0, (t_s - t0) / span))
                px = x_a + (x_b - x_a) * u
                py = y_a + (y_b - y_a) * u
                vx = (x_b - x_a) * extent / span * mosquito_speed
                vy = (y_b - y_a) * extent / span * mosquito_speed
                return TrackerSnapshot(
                    _clamp_galvo(x0 + px * extent),
                    _clamp_galvo(y0 + py * extent),
                    vx, vy, 1.0)
        return TrackerSnapshot(x0, y0, 0.0, 0.0, 1.0)

    trajectories = {
        "still": still,
        "moving": moving,
        "jinking": jinking,
        "mosquito5": mosquito5,
    }
    fn = trajectories[trajectory]

    def snapshot():
        return fn(time.monotonic() - start)

    return snapshot


def _build_config(args) -> ConeCollapseConfig:
    cfg = ConeCollapseConfig.from_settings()
    if args.mosquito_5s:
        if args.shrink is None:
            cfg.shrink_duration_s = 5.0
        if args.r_start is None:
            cfg.r_start_galvo = 260
        if args.lead is None:
            cfg.lead_factor = 0.42
        if args.breach is None:
            cfg.breach_multiplier = 3.5
        if args.stretch_max is None:
            cfg.stretch_max = 3.2
    if args.shrink is not None:
        cfg.shrink_duration_s = args.shrink
    if args.line is not None:
        cfg.line_duration_s = args.line
    if args.dark is not None:
        cfg.dark_duration_s = args.dark
    if args.bzzt is not None:
        cfg.bzzt_duration_s = args.bzzt
    if args.r_start is not None:
        cfg.r_start_galvo = args.r_start
    if args.r_min is not None:
        cfg.r_min_galvo = args.r_min
    if args.lead is not None:
        cfg.lead_factor = args.lead
    if args.breach is not None:
        cfg.breach_multiplier = args.breach
    if args.stretch_max is not None:
        cfg.stretch_max = args.stretch_max
    return _apply_power_scale(cfg, args.power_pct, args.bzzt_power_pct)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Live LaserCube cone-collapse pattern runner.")
    parser.add_argument("--ip", default="169.254.40.83")
    parser.add_argument("--src-ip", default="auto")
    parser.add_argument("--trajectory", choices=["still", "moving", "jinking"],
                        default="still")
    parser.add_argument("--mosquito-5s", action="store_true",
                        help="track a five-second mosquito-like path with "
                             "several direction changes before firing")
    parser.add_argument("--mosquito-speed", type=float, default=1.0,
                        help="--mosquito-5s speed multiplier")
    parser.add_argument("--x", type=float, default=0.5,
                        help="center x, normalized galvo coord 0..1")
    parser.add_argument("--y", type=float, default=0.5,
                        help="center y, normalized galvo coord 0..1")
    parser.add_argument("--extent", type=float, default=260.0,
                        help="moving/jinking excursion in galvo units")
    parser.add_argument("--power-pct", type=float, default=5.0,
                        help="scales shrink/line acquisition colors")
    parser.add_argument("--bzzt-power-pct", type=float, default=100.0,
                        help="scales the final bzzt pulse")
    parser.add_argument("--max-reacquires", type=int, default=2)
    parser.add_argument("--no-reacquire", action="store_true")
    parser.add_argument("--shrink", type=float, help="shrink phase duration (s)")
    parser.add_argument("--line", type=float, help="line phase duration (s)")
    parser.add_argument("--dark", type=float, help="dark phase duration (s)")
    parser.add_argument("--bzzt", type=float, help="bzzt phase duration (s)")
    parser.add_argument("--r-start", type=int, help="initial cone radius")
    parser.add_argument("--r-min", type=int, help="tight line/bzzt radius")
    parser.add_argument("--lead", type=float, help="cone-center pursuit gain")
    parser.add_argument("--breach", type=float, help="breach radius multiplier")
    parser.add_argument("--stretch-max", type=float, help="max oval stretch")
    args = parser.parse_args(argv)
    if args.mosquito_5s:
        args.trajectory = "mosquito5"

    # Hard ceiling from settings — operator args cannot exceed it.
    max_pct = float(settings.SAFETY_MAX_POWER_PCT)
    for name in ("power_pct", "bzzt_power_pct"):
        if getattr(args, name) > max_pct:
            print(f"note: --{name.replace('_', '-')} "
                  f"{getattr(args, name):.0f}% clamped to "
                  f"SAFETY_MAX_POWER_PCT={max_pct:.0f}%")
            setattr(args, name, max_pct)

    cfg = _build_config(args)
    x0 = _norm_to_galvo(args.x)
    y0 = _norm_to_galvo(args.y)

    print("=" * 60)
    print("  LIVE cone-collapse LaserCube runner")
    print("  Class 3B laser. Eye protection + safety lens REQUIRED.")
    print("=" * 60)
    print(f"trajectory : {args.trajectory}")
    print(f"center     : ({x0:.0f}, {y0:.0f}) galvo")
    print(f"power      : acquire={args.power_pct:.1f}% "
          f"bzzt={args.bzzt_power_pct:.1f}%")
    print(f"timing     : shrink={cfg.shrink_duration_s}s "
          f"line={cfg.line_duration_s}s dark={cfg.dark_duration_s}s "
          f"bzzt={cfg.bzzt_duration_s}s")
    print(f"geometry   : r_start={cfg.r_start_galvo} "
          f"r_min={cfg.r_min_galvo} lead={cfg.lead_factor} "
          f"breach={cfg.breach_multiplier}x")

    cube = LaserCubeInterface(ip=args.ip, src_ip=args.src_ip)
    if not cube.connect():
        print("FAIL: could not connect to the cube")
        return 1
    print(f"\nconnected (src_ip={cube.src_ip})\n")

    try:
        print("Pre-flight:")
        if not _preflight(cube):
            return 1
        if not _confirm("Fire one live cone-collapse pattern now?"):
            print("aborted by operator")
            return 0
        if args.bzzt_power_pct >= 50.0:
            if not _confirm(f"Final BZZT is set to "
                            f"{args.bzzt_power_pct:.0f}% power. Confirm?"):
                print("aborted by operator")
                return 0

        cube.set_dac_rate(cfg.dac_rate)
        cube.clear_ringbuffer()
        mgr = LaserManager(cube)
        snapshot_fn = _snapshot_factory(args.trajectory, x0, y0, args.extent,
                                        args.mosquito_speed)

        seen_phases: set[str] = set()

        def on_chunk(chunk):
            if chunk.phase.value not in seen_phases:
                seen_phases.add(chunk.phase.value)
                print(f"  t={chunk.elapsed_s:6.3f}s -> "
                      f"{chunk.phase.value.upper():6s} "
                      f"radius={chunk.current_radius_galvo:5.1f} "
                      f"stretch={chunk.stretch_factor:.2f}")

        print("\nLASER ON — streaming cone-collapse pattern")
        cube.enable_output()
        summary = mgr.engage_cone(
            snapshot_fn, config=cfg, on_chunk=on_chunk,
            allow_reacquire=not args.no_reacquire,
            max_reacquires=args.max_reacquires)
        print(f"\nresult : {summary['reason']}")
        print(f"fired  : {summary['fired']}")
        print(f"chunks : {summary['chunks']}")
        print(f"reacqs : {summary['reacquires']}")
        return 0 if summary["fired"] else 1
    finally:
        cube.disable_output()
        cube.clear_ringbuffer()
        print("LASER OFF")
        cube.disconnect()
        print("disconnected")


if __name__ == "__main__":
    raise SystemExit(main())
