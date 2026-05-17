"""compare_sensors — does the Kinect detect mosquitoes the GoPro misses?

Replays the detection events from a recorded session JSONL, cross-associates
GoPro detections with Kinect (RGB/IR) detections within a time+space window,
and bins them: Both / GoPro-only / Kinect-only. The "Kinect only" set is the
one that matters — if it is large, Cole's IR hypothesis holds.

    python tools/compare_sensors.py recording.jsonl [--extrinsic FILE]

Kinect detections are projected into GoPro-norm space via the Kinect→GoPro
extrinsic (§9.5) when one is supplied; without it, raw norm coords are
compared and the report says so.

Reference: BOOTSTRAP_AMENDMENTS_v0.2.1.md §16.4, Step 9.7.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import settings                                    # noqa: E402
from targeting.extrinsics import CrossSensorExtrinsic           # noqa: E402


@dataclass
class DetectionRecord:
    ts: float
    sensor_id: str
    x_norm: float
    y_norm: float

    @property
    def is_gopro(self) -> bool:
        return self.sensor_id == "gopro"

    @property
    def is_kinect(self) -> bool:
        return self.sensor_id.startswith("kinect")


@dataclass
class ComparisonReport:
    source: str
    duration_s: float
    raw_counts: dict[str, int]
    both_per_stream: dict[str, int]          # kinect stream -> # gopro matches
    gopro_only: int
    kinect_only: int
    projected: bool                          # was an extrinsic applied?
    assoc_dt_ms: float = field(default=0.0)
    assoc_dist_norm: float = field(default=0.0)

    @property
    def n_gopro(self) -> int:
        return self.raw_counts.get("gopro", 0)

    def _per_minute(self, n: int) -> float:
        minutes = self.duration_s / 60.0
        return n / minutes if minutes > 0 else 0.0

    def format(self) -> str:
        lines = [
            f"SENSOR COMPARISON — {self.source}",
            f"Duration: {self.duration_s:.0f} seconds",
            "",
            "Total detections (raw):",
        ]
        for sid in sorted(self.raw_counts):
            lines.append(f"  {sid:<14}{self.raw_counts[sid]:>6}")
        lines += [
            "",
            "Cross-sensor association window:",
            f"  +/-{self.assoc_dt_ms:.0f}ms, "
            f"+/-{self.assoc_dist_norm * 100:.1f}% normalized",
            f"  Kinect->GoPro projection: "
            f"{'extrinsic applied' if self.projected else 'NONE (raw coords)'}",
            "",
            "Detection categorization:",
        ]
        gp = max(1, self.n_gopro)
        for stream, n in sorted(self.both_per_stream.items()):
            lines.append(f"  Both (gopro + {stream}):{'':<6}{n:>6}  "
                         f"({100.0 * n / gp:.1f}% of gopro)")
        lines += [
            f"  GoPro only (no kinect at all):{'':>2}{self.gopro_only:>6}  "
            f"({100.0 * self.gopro_only / gp:.1f}% of gopro)",
            f"  Kinect only (any stream, no gopro): {self.kinect_only:>6}  "
            f"(NEW from kinect)",
            "",
            "Detection rate per minute:",
        ]
        for sid in sorted(self.raw_counts):
            lines.append(f"  {sid:<14}{self._per_minute(self.raw_counts[sid]):>8.1f}/min")
        return "\n".join(lines)


# ── Loading ──────────────────────────────────────────────────────────────

def load_detections(jsonl_path: Path | str) -> list[DetectionRecord]:
    """Parse `type == "detection"` events out of a session JSONL recording."""
    records: list[DetectionRecord] = []
    for line in Path(jsonl_path).read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            continue
        if ev.get("type") != "detection":
            continue
        records.append(DetectionRecord(
            ts=float(ev.get("ts", ev.get("timestamp", 0.0))),
            sensor_id=str(ev["sensor_id"]),
            x_norm=float(ev["x_norm"]),
            y_norm=float(ev["y_norm"]),
        ))
    return records


# ── Comparison ───────────────────────────────────────────────────────────

def _project(d: DetectionRecord,
             extrinsic: Optional[CrossSensorExtrinsic]) -> tuple[float, float]:
    """Kinect detection in GoPro-norm space (raw coords if no extrinsic)."""
    if extrinsic is not None and d.is_kinect:
        p = extrinsic.project(d.x_norm, d.y_norm)
        return float(p[0]), float(p[1])
    return d.x_norm, d.y_norm


def compare(detections: list[DetectionRecord],
            *,
            extrinsic: Optional[CrossSensorExtrinsic] = None,
            assoc_dt_ms: Optional[float] = None,
            assoc_dist_norm: Optional[float] = None,
            source: str = "<memory>") -> ComparisonReport:
    """Cross-associate GoPro and Kinect detections into Both/only bins."""
    assoc_dt_ms = (assoc_dt_ms if assoc_dt_ms is not None
                   else settings.COMPARE_TOOL_ASSOC_DT_MS)
    assoc_dist_norm = (assoc_dist_norm if assoc_dist_norm is not None
                       else settings.COMPARE_TOOL_ASSOC_DISTANCE_NORM)
    dt_s = assoc_dt_ms / 1000.0

    gopro = [d for d in detections if d.is_gopro]
    kinect = [d for d in detections if d.is_kinect]

    raw_counts: dict[str, int] = {}
    for d in detections:
        raw_counts[d.sensor_id] = raw_counts.get(d.sensor_id, 0) + 1

    def matches(a: DetectionRecord, b: DetectionRecord) -> bool:
        if abs(a.ts - b.ts) > dt_s:
            return False
        ax, ay = _project(a, extrinsic)
        bx, by = _project(b, extrinsic)
        return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5 <= assoc_dist_norm

    both_per_stream: dict[str, int] = {}
    gopro_only = 0
    for g in gopro:
        hit_streams = {k.sensor_id for k in kinect if matches(g, k)}
        if hit_streams:
            for s in hit_streams:
                both_per_stream[s] = both_per_stream.get(s, 0) + 1
        else:
            gopro_only += 1

    kinect_only = sum(
        1 for k in kinect if not any(matches(k, g) for g in gopro))

    timestamps = [d.ts for d in detections]
    duration = (max(timestamps) - min(timestamps)) if timestamps else 0.0

    return ComparisonReport(
        source=source, duration_s=duration, raw_counts=raw_counts,
        both_per_stream=both_per_stream, gopro_only=gopro_only,
        kinect_only=kinect_only, projected=extrinsic is not None,
        assoc_dt_ms=assoc_dt_ms, assoc_dist_norm=assoc_dist_norm,
    )


def write_csv(detections: list[DetectionRecord], path: Path | str) -> Path:
    """Per-detection dump for downstream labeling / heatmapping."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = ["ts,sensor_id,x_norm,y_norm"]
    rows += [f"{d.ts},{d.sensor_id},{d.x_norm},{d.y_norm}" for d in detections]
    path.write_text("\n".join(rows) + "\n")
    return path


# ── CLI ──────────────────────────────────────────────────────────────────

def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Compare GoPro vs Kinect "
                                     "detections from a session recording.")
    parser.add_argument("recording", help="session JSONL recording")
    parser.add_argument("--extrinsic", metavar="FILE", default=None,
                        help="Kinect→GoPro extrinsic JSON (§9.5)")
    parser.add_argument("--csv", metavar="FILE", default=None,
                        help="also write a per-detection CSV")
    args = parser.parse_args(argv)

    detections = load_detections(args.recording)
    extrinsic = (CrossSensorExtrinsic.load(args.extrinsic)
                 if args.extrinsic else None)
    report = compare(detections, extrinsic=extrinsic,
                     source=Path(args.recording).name)
    print(report.format())
    if args.csv:
        print(f"\nPer-detection CSV: {write_csv(detections, args.csv)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
