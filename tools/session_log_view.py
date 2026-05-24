"""session_log_view.py — summarise an events.jsonl from a live session.

Reads a SessionRecorder log and prints a human-readable summary:
  • event counts by op
  • safety verdict timeline (state changes only)
  • shot tally with per-track stats
  • basic latency between detection and shoot, when matchable

Use after a live session to audit what happened:

    python tools/session_log_view.py user_data/sessions/live_fire_20260524T..../
    python tools/session_log_view.py --json file.jsonl   # any JSONL log
"""
from __future__ import annotations

import argparse
import collections
import json
import sys
from pathlib import Path
from typing import Iterable


def _iter_records(path: Path) -> Iterable[dict]:
    """Yield records from a session dir OR a single JSONL file."""
    if path.is_dir():
        path = path / "events.jsonl"
    if not path.is_file():
        raise SystemExit(f"no events.jsonl at {path}")
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def summarise(records: list[dict]) -> dict:
    counts = collections.Counter(r.get("op", "?") for r in records)

    verdicts = [r for r in records if r.get("op") == "safety_verdict"]
    transitions = []
    last_state = None
    for v in verdicts:
        s = v["data"].get("state")
        if s != last_state:
            transitions.append((v.get("t_mono", 0.0), last_state, s,
                                v["data"].get("reasons", [])))
            last_state = s

    shoots = [r for r in records
              if r.get("op") == "laser_event"
              and r["data"].get("op") == "shoot"]
    per_track: dict = collections.Counter(
        s["data"].get("track_id", "?") for s in shoots)

    # Detection → shoot deltas, when both reference the same track id.
    det_times: dict = {}
    deltas: list[float] = []
    for r in records:
        if r.get("op") == "detection":
            tid = r["data"].get("detection_id")
            if tid is not None:
                det_times.setdefault(tid, []).append(r.get("t_mono", 0.0))
        if r.get("op") == "laser_event" and r["data"].get("op") == "shoot":
            tid = r["data"].get("track_id")
            if tid in det_times and det_times[tid]:
                deltas.append(r.get("t_mono", 0.0) - det_times[tid].pop(0))

    return {
        "n_records": len(records),
        "duration_s": records[-1].get("t_mono", 0.0) if records else 0.0,
        "counts": dict(counts),
        "transitions": transitions,
        "shoots": len(shoots),
        "per_track": dict(per_track),
        "det_to_shoot_deltas_s": deltas,
    }


def _print_report(report: dict) -> None:
    print("=" * 60)
    print(f"  records      : {report['n_records']}")
    print(f"  duration     : {report['duration_s']:.1f} s")
    print()
    print("  event counts:")
    for op, n in sorted(report["counts"].items(), key=lambda kv: -kv[1]):
        print(f"    {op:<20} {n}")
    print()
    if report["transitions"]:
        print("  safety state transitions:")
        for t, src, dst, reasons in report["transitions"]:
            r = "; ".join(reasons[:2])
            print(f"    {t:6.2f}s  {str(src):>10} -> {dst:<10}  {r}")
        print()
    print(f"  shots fired  : {report['shoots']}")
    if report["per_track"]:
        for tid, n in sorted(report["per_track"].items(),
                              key=lambda kv: -kv[1]):
            print(f"    track {tid}: {n} shot(s)")
    if report["det_to_shoot_deltas_s"]:
        deltas = report["det_to_shoot_deltas_s"]
        mean = sum(deltas) / len(deltas) * 1000.0
        print(f"  det->shoot   : mean {mean:.0f} ms over {len(deltas)} pairs")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path",
                    help="session dir or path to events.jsonl")
    ap.add_argument("--json", action="store_true",
                    help="emit the report as JSON instead of a text summary")
    args = ap.parse_args(argv)

    records = list(_iter_records(Path(args.path)))
    report = summarise(records)
    if args.json:
        json.dump(report, sys.stdout, indent=2, default=repr)
        sys.stdout.write("\n")
    else:
        _print_report(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
