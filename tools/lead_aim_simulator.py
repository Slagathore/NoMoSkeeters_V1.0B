"""lead_aim_simulator.py — offline miss-distance simulator.

Given the measured system lag (the `LATENCY_SOFTWARE_LAG_MS` setting) and a
target velocity, compute the predicted miss distance with and without
lead-aim. Use to:
  • Decide whether lead-aim is worth turning on for a given target class.
  • Set realistic expectations before a session.
  • Spot-check that the LATENCY_SOFTWARE_LAG_MS in settings.py matches the
    most recent latency probe.

Pure simulation — no laser, no camera, no network.

    python tools/lead_aim_simulator.py
    python tools/lead_aim_simulator.py --lag-ms 24 --velocity 0.3 0.5 1.0 2.0
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from config import settings                                       # noqa: E402


def simulate(lag_ms: float, velocity_norm_per_s: float,
             scene_width_m: float) -> dict:
    """Return miss distance (norm + metres) with and without lead-aim.

    Lead-aim adds `velocity * lag` to the aimed position; the laser still
    takes `lag` to arrive, so it lands `velocity * lag` ahead of the
    target-at-detect — i.e. on top of the target-at-fire. Without lead, the
    laser lands at the target-at-detect — and the target has moved
    `velocity * lag` since."""
    lag_s = lag_ms / 1000.0
    miss_norm = velocity_norm_per_s * lag_s
    miss_m = miss_norm * scene_width_m
    return {
        "lag_ms": lag_ms,
        "velocity_norm_per_s": velocity_norm_per_s,
        "velocity_m_per_s": velocity_norm_per_s * scene_width_m,
        "miss_no_lead_norm": miss_norm,
        "miss_no_lead_mm": miss_m * 1000.0,
        "miss_with_lead_norm": 0.0,    # idealised: assumes velocity is known
        "miss_with_lead_mm": 0.0,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lag-ms", type=float,
                    default=settings.LATENCY_SOFTWARE_LAG_MS,
                    help="system lag in ms (default = LATENCY_SOFTWARE_LAG_MS)")
    ap.add_argument("--velocity", type=float, nargs="+",
                    default=[0.1, 0.3, 0.5, 1.0, 2.0],
                    help="target velocities to simulate, in norm/s "
                         "(default 0.1 ... 2.0)")
    ap.add_argument("--scene-width-m", type=float, default=2.0,
                    help="metres spanned by norm-x=[0,1] at the target plane "
                         "(default 2 m — adjust for your room geometry)")
    args = ap.parse_args(argv)

    print("=" * 60)
    print("  Lead-aim miss-distance simulator")
    print(f"  system lag           : {args.lag_ms:.0f} ms")
    print(f"  scene width @ target : {args.scene_width_m:.2f} m"
          f"   (norm·width = metres)")
    print("=" * 60)
    print(f"{'v(norm/s)':>10}  {'v(m/s)':>9}  "
          f"{'miss noLead(norm)':>17}  {'miss noLead(mm)':>17}  "
          f"{'miss lead(mm)':>14}")
    for v in args.velocity:
        r = simulate(args.lag_ms, v, args.scene_width_m)
        print(f"{r['velocity_norm_per_s']:>10.3f}  "
              f"{r['velocity_m_per_s']:>9.3f}  "
              f"{r['miss_no_lead_norm']:>17.4f}  "
              f"{r['miss_no_lead_mm']:>17.1f}  "
              f"{r['miss_with_lead_mm']:>14.1f}")
    print("\nNote: lead-aim assumes velocity is correctly estimated. Real-world")
    print("miss with lead-aim is dominated by velocity-estimate noise (see")
    print("TRACKER_KALMAN_* settings) rather than the lag, so a Kalman that")
    print("smooths velocity over 5+ frames is essential when v > 0.3 m/s.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
