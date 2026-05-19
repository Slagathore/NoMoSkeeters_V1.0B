"""Cone-collapse demo + tuning runner — no hardware, no photons.

Two modes:

  csv  — simulate the full pattern against still / moving / jinking targets
         and write a per-sample CSV for each. Plot the CSVs to see what the
         cube would scan. (laser/shot_patterns/cone_collapse.py::simulate_to_csv)

  fire — run the pattern through LaserManager.engage_cone against a
         DryRunTransport, printing each phase transition. This exercises the
         real firing driver (breach/reacquire, chunk streaming) end to end.

Every ConeCollapseConfig knob is exposed as a flag, so this doubles as a
tuning bench: change a value, re-run, inspect.

    python scripts/cone_demo.py                         # csv, all trajectories
    python scripts/cone_demo.py --mode fire --trajectory jinking
    python scripts/cone_demo.py --mode fire --lead 0.35 --breach 1.2 --realtime
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from laser.laser_manager import LaserManager                       # noqa: E402
from laser.shot_patterns import (ConeCollapseConfig,               # noqa: E402
                                 TrackerSnapshot, simulate_to_csv)
from laser.transports.dry_run import DryRunTransport               # noqa: E402


# ── Simulated mosquito trajectories (all in 12-bit GALVO space) ──────────

def _still(t_s: float) -> TrackerSnapshot:
    return TrackerSnapshot(2048, 2048, 0.0, 0.0, 1.0)


def _moving(t_s: float) -> TrackerSnapshot:
    # Linear drift to the right at 300 galvo units/s — slow enough that the
    # cone's pursuit lag stays inside the shrunk cone, so the shot converges
    # and shows a cleanly stretched, tracked oval.
    return TrackerSnapshot(1500 + 300 * t_s, 2048, 300.0, 0.0, 1.0)


def _jinking(t_s: float) -> TrackerSnapshot:
    # Sinusoidal jinking — direction keeps changing, provokes breaches.
    x = 2048 + 300 * math.sin(t_s * 8)
    y = 2048 + 200 * math.cos(t_s * 6)
    vx = 300 * 8 * math.cos(t_s * 8)
    vy = -200 * 6 * math.sin(t_s * 6)
    return TrackerSnapshot(x, y, vx, vy, 1.0)


_TRAJECTORIES = {"still": _still, "moving": _moving, "jinking": _jinking}


# ── Config from CLI ──────────────────────────────────────────────────────

def _build_config(args) -> ConeCollapseConfig:
    """Start from the settings.py defaults, override with any CLI knobs."""
    cfg = ConeCollapseConfig.from_settings()
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
    return cfg


# ── Modes ────────────────────────────────────────────────────────────────

def _run_csv(cfg: ConeCollapseConfig, out_dir: Path) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, traj in _TRAJECTORIES.items():
        path = out_dir / f"cone_{name}.csv"
        simulate_to_csv(str(path), config=cfg, mosquito_trajectory=traj)
        rows = sum(1 for _ in path.open()) - 1
        print(f"  wrote {path}  ({rows} samples)")
    print(f"\n{len(_TRAJECTORIES)} CSVs in {out_dir} - plot x_galvo/y_galvo "
          "coloured by phase to see the cone collapse.")
    return 0


def _run_fire(cfg: ConeCollapseConfig, trajectory: str, realtime: bool) -> int:
    traj = _TRAJECTORIES[trajectory]
    transport = DryRunTransport()
    transport.connect()
    mgr = LaserManager(transport)

    # engage_cone polls snapshot_fn once per chunk; advance a local clock by
    # one chunk-duration each call so the trajectory plays forward.
    chunk_dt = cfg.chunk_size / cfg.dac_rate
    clock = {"t": 0.0}

    def snapshot_fn():
        snap = traj(clock["t"])
        clock["t"] += chunk_dt
        return snap

    seen_phases: set = set()

    def on_chunk(chunk):
        if chunk.phase not in seen_phases:
            seen_phases.add(chunk.phase)
            print(f"  t={chunk.elapsed_s:6.3f}s  -> {chunk.phase.value.upper()}"
                  f"  radius={chunk.current_radius_galvo:5.1f}"
                  f"  stretch={chunk.stretch_factor:.2f}")

    print(f"firing cone-collapse at a '{trajectory}' target "
          f"(transport: dry-run)\n")
    summary = mgr.engage_cone(
        snapshot_fn, config=cfg, on_chunk=on_chunk,
        sleep_fn=__import__("time").sleep if realtime else (lambda _s: None))

    print(f"\n  result : {summary['reason']}")
    print(f"  fired  : {summary['fired']}")
    print(f"  chunks : {summary['chunks']}  ({transport.packet_count} UDP "
          f"packets to the dry-run cube)")
    print(f"  reacqs : {summary['reacquires']}")
    transport.disconnect()
    return 0 if summary["fired"] else 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Cone-collapse demo / tuning bench.")
    p.add_argument("--mode", choices=["csv", "fire"], default="csv")
    p.add_argument("--trajectory", choices=list(_TRAJECTORIES),
                   default="jinking", help="fire mode: which target to chase")
    p.add_argument("--out-dir", default="cone_demo_out",
                   help="csv mode: directory for the CSV files")
    p.add_argument("--realtime", action="store_true",
                   help="fire mode: pace at true wall-clock speed (~1.1 s) "
                        "instead of running flat out")
    # Tunable ConeCollapseConfig knobs — unset flags fall back to settings.py.
    p.add_argument("--shrink", type=float, help="shrink phase duration (s)")
    p.add_argument("--line", type=float, help="line phase duration (s)")
    p.add_argument("--dark", type=float, help="dark phase duration (s)")
    p.add_argument("--bzzt", type=float, help="bzzt phase duration (s)")
    p.add_argument("--r-start", type=int, help="initial cone radius (galvo)")
    p.add_argument("--r-min", type=int, help="tight line/bzzt radius (galvo)")
    p.add_argument("--lead", type=float, help="cone-center pursuit gain")
    p.add_argument("--breach", type=float, help="breach radius multiplier")
    p.add_argument("--stretch-max", type=float, help="max oval stretch ratio")
    args = p.parse_args(argv)

    cfg = _build_config(args)
    print("=" * 60)
    print(f"  Cone-collapse demo - mode={args.mode}")
    print(f"  shrink={cfg.shrink_duration_s}s line={cfg.line_duration_s}s "
          f"dark={cfg.dark_duration_s}s bzzt={cfg.bzzt_duration_s}s")
    print(f"  r_start={cfg.r_start_galvo} r_min={cfg.r_min_galvo} "
          f"lead={cfg.lead_factor} breach={cfg.breach_multiplier}x")
    print("=" * 60)

    if args.mode == "csv":
        return _run_csv(cfg, Path(args.out_dir))
    return _run_fire(cfg, args.trajectory, args.realtime)


if __name__ == "__main__":
    raise SystemExit(main())
