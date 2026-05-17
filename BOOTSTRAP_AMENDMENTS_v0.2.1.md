# BOOTSTRAP — AMENDMENTS v0.2.1

**An addendum to `BOOTSTRAP_AMENDMENTS.md` v0.2.** Adds selectable Kinect-as-targeting modes, cross-sensor track-level interpolation for fusion, and a comparison harness for empirically testing whether Kinect outperforms GoPro at mosquito detection.

> Status: Drafted on operator request — Cole hypothesizes the Kinect (IR + 1080p RGB) may detect mosquitoes the GoPro misses, and wants the architecture to support testing this empirically. Not just to test, but to actually use Kinect as the targeting sensor if the data supports it.
> Author: Claude.
> Last updated: 2026-05-10

---

## What changes from v0.2

The v0.2 amendments locked in:
- GoPro = TARGETING (rigid mount, calibrated to GALVO)
- Kinect = SAFETY only (mobile, depth/world reasoning)

This addendum **softens that to the default**, not a hard rule. Both sensors can be assigned `SensorRole.TARGETING` independently. The architecture supports three modes:

| Mode | GoPro role | Kinect role | Notes |
|---|---|---|---|
| **A — `gopro_only`** (default) | TARGETING | SAFETY | What v0.2 designed. |
| **B — `kinect_only`** (experimental) | OFF or SAFETY | TARGETING | Tests Cole's hypothesis solo. Requires Kinect→GALVO calibration and re-cal per session. |
| **C — `fused_interp`** (Cole's preferred) | TARGETING | TARGETING + SAFETY | Both feed detections; tracks merge across sensors via timestamp-aligned interpolation. |

Mode is set in config; runtime-toggleable with appropriate calibration availability checks.

---

## §5 — Sensor layer (AMEND, additions)

### §5.6.1 (NEW) — Selectable sensor roles

`SensorRole` is a config knob, not a static binding. Selection happens in `config/settings.py`:

```python
# config/settings.py — sensor roles section
TARGETING_MODE: str = "gopro_only"   # "gopro_only" | "kinect_only" | "fused_interp"

# Per-sensor role assignments (computed from TARGETING_MODE at boot,
# or set explicitly to override).
SENSOR_ROLES: dict = {
    "gopro":     "auto",   # "auto" | "targeting" | "safety" | "fallback" | "off"
    "kinect_v2": "auto",
    "local_cam": "auto",
}
# When "auto", the role is derived from TARGETING_MODE:
#   gopro_only:    gopro=targeting,  kinect=safety,    local=fallback
#   kinect_only:   gopro=safety,     kinect=targeting, local=off
#   fused_interp:  gopro=targeting,  kinect=targeting, local=off
# Explicit values override.
```

Validation at startup: a sensor with `role=targeting` must have a valid GALVO calibration profile loaded. If missing, the system refuses to start in that mode and prints which calibration is missing.

### §5.6.2 (NEW) — Kinect intrinsic considerations for targeting

If you put Kinect in TARGETING role, three things matter that didn't before:

1. **Resolution.** Kinect RGB is 1920×1080 (better than the GoPro at the same wide field of view, depending on GoPro mode), but Kinect's RGB has a wider lens distortion and warmer color cast. For small targets, the depth/IR streams (512×424) are *lower* resolution than the GoPro RGB — but they may pick up signal the RGB misses entirely.
2. **IR illumination.** The Kinect floods the scene with 850nm IR. Mosquitoes scatter IR like any small object; a tiny moving heat signature shows up well against a static IR-bright background. This is genuinely a different sensor modality than RGB and may catch what the GoPro can't.
3. **Frame rate.** Kinect runs at 30 fps for all streams. That's the same as GoPro live preview, so no temporal advantage either way. A fast mosquito flying 30 cm/frame at 30 fps moves ~30 px between frames in the Kinect RGB — challenging but tractable for a Kalman tracker with reasonable process noise.

Detection pipeline per sensor stream when Kinect is in targeting role:

| Stream | Pipeline |
|---|---|
| Kinect RGB 1920×1080 | MOG2 background subtraction, classifier, normalize |
| Kinect IR 512×424 | High-pass filter for moving warm spots, classifier, normalize |
| Kinect depth 512×424 | Used to attribute world coords to detections from the other two streams (not detection itself; depth is too noisy for mosquito-scale targets) |

Each stream emits its own `DetectionEvent` on the bus with `sensor_id` reflecting the stream (`"kinect_rgb"`, `"kinect_ir"`). The tracker can be configured to pull from one or both.

### §5.6.3 (NEW) — Per-stream role assignment (fine-grained)

For the fused_interp mode in particular, you may want to be more granular than per-sensor:

```python
SENSOR_STREAM_ROLES: dict = {
    "gopro":      "targeting",
    "kinect_rgb": "targeting",
    "kinect_ir":  "targeting",   # both Kinect streams contribute
    "kinect_depth": "world_attribution",  # depth never detects, only attributes
    "local_cam":  "off",
}
```

This gets surfaced as a section in the GUI control panel when in fused_interp mode.

---

## §5.7 (NEW) — Cross-sensor temporal alignment (interpolation)

When two or more sensors are running with `targeting` role, their detections need to be associated to the same physical object across time-of-arrival differences. Since neither sensor reports hardware timestamps (we tag with `time.monotonic()` at receipt), and there's USB/network jitter on each, raw timestamp comparison is unreliable.

The solution: **alignment happens at the track level, not the detection level.** Each sensor's detections feed into its own per-sensor sub-tracker (small, lightweight, not the full system tracker). When a detection arrives, append to its sensor-local track. To check whether two sensors are seeing the same object, project each sub-track to a shared reference timestamp using its own velocity estimate, then compare positions in a shared coordinate space.

```python
# tracking/cross_sensor_fusion.py
from dataclasses import dataclass
from collections import defaultdict, deque
from typing import Optional
import numpy as np


@dataclass
class SensorLocalState:
    """One sensor's view of one tracked object — minimal Kalman or even just
    last 2-3 detections with linear extrapolation."""
    sensor_id: str
    history: deque   # of (timestamp, x_norm, y_norm) — bounded length
    # Velocity estimated from the last N entries:
    vx_norm_per_s: float = 0.0
    vy_norm_per_s: float = 0.0

    def project_to(self, t_ref: float) -> tuple[float, float]:
        """Linear extrapolation/interpolation to t_ref using last-known
        position and current velocity estimate. Cheap and good enough for
        ±50ms windows."""
        if not self.history:
            return (float("nan"), float("nan"))
        t_last, x_last, y_last = self.history[-1]
        dt = t_ref - t_last
        return (x_last + self.vx_norm_per_s * dt,
                y_last + self.vy_norm_per_s * dt)

    def update_velocity(self, min_samples: int = 3) -> None:
        """Refit velocity from the last min_samples points using linear
        regression. Cheap; runs once per detection."""
        if len(self.history) < min_samples:
            return
        ts = np.array([h[0] for h in self.history])
        xs = np.array([h[1] for h in self.history])
        ys = np.array([h[2] for h in self.history])
        # np.polyfit degree 1 → slope is velocity
        self.vx_norm_per_s = float(np.polyfit(ts, xs, 1)[0])
        self.vy_norm_per_s = float(np.polyfit(ts, ys, 1)[0])


class CrossSensorFusion:
    """Maintains per-sensor sub-tracks and fuses them across sensors at
    track level. Emits a unified TrackEvent stream to the rest of the system."""

    def __init__(self,
                 max_assoc_distance_norm: float = 0.05,
                 history_length: int = 8):
        self._sensor_states: dict[tuple[str, int], SensorLocalState] = {}
        self.max_assoc_distance_norm = max_assoc_distance_norm
        self.history_length = history_length

    def ingest_detection(self, det: "DetectionEvent") -> None:
        """Append detection to its sensor-local sub-track. New sub-track ID
        is allocated locally per sensor; cross-sensor association happens
        in fuse()."""
        # ... per-sensor mini-tracker logic; could be a tiny IoU/distance
        #     matcher or a 1D Kalman per dimension. Out of scope for this
        #     code sketch but the public interface is what matters.
        pass

    def fuse(self, t_ref: Optional[float] = None) -> list["FusedTrack"]:
        """For every pair of sensor sub-tracks (across different sensors),
        project each to t_ref and check whether their projected positions
        are within max_assoc_distance_norm.

        If yes — emit one FusedTrack with attributes from both sensors.
        If no  — each sub-track emits its own FusedTrack solo.

        t_ref defaults to "the most recent timestamp across all sensors."
        """
        if t_ref is None:
            t_ref = max(
                (s.history[-1][0] for s in self._sensor_states.values()
                 if s.history),
                default=0.0,
            )

        # Group by sensor
        by_sensor: dict[str, list[SensorLocalState]] = defaultdict(list)
        for state in self._sensor_states.values():
            by_sensor[state.sensor_id].append(state)

        fused = []
        sensors = list(by_sensor.keys())
        if len(sensors) < 2:
            # Only one sensor — no cross-fusion possible. Pass through.
            return _passthrough(self._sensor_states, t_ref)

        # All-pairs association across sensors. For 2 sensors with N tracks
        # each it's O(N^2) — fine for mosquito counts in single digits.
        used: set = set()
        for sa in by_sensor[sensors[0]]:
            best_match = None
            best_dist = float("inf")
            xa, ya = sa.project_to(t_ref)
            for sb in by_sensor[sensors[1]]:
                if id(sb) in used:
                    continue
                xb, yb = sb.project_to(t_ref)
                d = ((xa - xb) ** 2 + (ya - yb) ** 2) ** 0.5
                if d < best_dist and d < self.max_assoc_distance_norm:
                    best_dist = d
                    best_match = sb
            if best_match is not None:
                used.add(id(best_match))
                fused.append(FusedTrack(
                    t_ref=t_ref,
                    position_norm=((xa + best_match.project_to(t_ref)[0]) / 2,
                                   (ya + best_match.project_to(t_ref)[1]) / 2),
                    contributing_sensors=[sa.sensor_id, best_match.sensor_id],
                    assoc_distance_norm=best_dist,
                ))
            else:
                fused.append(FusedTrack.solo(sa, t_ref))
        # Add unmatched tracks from the second sensor
        for sb in by_sensor[sensors[1]]:
            if id(sb) not in used:
                fused.append(FusedTrack.solo(sb, t_ref))
        return fused
```

Settings:

```python
# config/settings.py — fusion section
FUSION_MAX_ASSOC_DISTANCE_NORM: float = 0.05   # ~5% of frame
FUSION_HISTORY_LENGTH: int = 8                  # samples per sub-track
FUSION_MIN_VELOCITY_SAMPLES: int = 3            # refit velocity after N detections
FUSION_PROJECTION_MAX_DT_MS: float = 100.0     # don't extrapolate past this
```

The crucial property: fusion only ever combines tracks from *different* sensors. Same-sensor tracks are handled by that sensor's local mini-tracker (and by the system tracker downstream). This avoids the degenerate case where one sensor's noise gets self-fused into a phantom track.

---

## §8.10 (NEW) — Kinect calibration to GALVO space

Required when Kinect has TARGETING role. Same calibration patterns as the GoPro (grid, halton, dragline, windmill — see v0.1 §8.2), observed by Kinect's RGB stream rather than the GoPro.

**Critical difference**: Kinect mobility. The calibration is invalid the moment the Kinect is bumped. Two strategies:

### Strategy A — Recalibrate per session

The simplest. Every time you start the system in `kinect_only` or `fused_interp` mode:
1. Place the Kinect.
2. Run a 25-point grid calibration.
3. Verify multi-depth (§8.6).
4. Persist the calibration to `user_data/calibrations/kinect_v2/session_<timestamp>.json`.
5. Don't move the Kinect for the rest of the session.

Cost: 30-60 seconds of setup time per session. Probably acceptable.

### Strategy B — Pose-tracked recalibration (deferred)

Use ArUco markers or known scene features to detect when the Kinect has moved and trigger a quick recalibration. More work; defer to v0.4 unless Strategy A becomes annoying.

### Calibration JSON schema for Kinect

Same v2 schema as GoPro (§8.5), with one extra field:

```json
{
  "schema_version": 2,
  "sensor_id": "kinect_v2",
  "stream": "rgb",                    // or "ir" — calibrate per stream
  "scene": "garage_default",
  "session_timestamp": "2026-05-10T14:23:45",
  "kinect_relative_pose": {           // NEW: best-effort recovery hint
    "approx_origin_xyz_m": [0.5, 1.2, 0.0],
    "approx_yaw_pitch_roll_deg": [0, -10, 0],
    "operator_note": "On corner shelf, pointed at wall"
  },
  "n_points": 25,
  "pattern": "halton",
  "H_norm_to_galvo": [[...], [...], [...]],
  "H_galvo_to_norm": [[...], [...], [...]],
  "residual_norm": 0.0058,
  "residual_galvo": 19.7,
  "validation": {
    "depths_m": [1.0, 2.5, 4.0],
    "residuals_norm": [0.012, 0.005, 0.007],
    "passed": true
  }
}
```

The `kinect_relative_pose` is informational (helps the operator place the Kinect roughly where it was last time), not used by code.

### Settings

```python
# config/settings.py — Kinect calibration
KINECT_CALIBRATION_REQUIRED_FOR_TARGETING: bool = True
KINECT_CALIBRATION_PROMPT_ON_STARTUP: bool = True   # ask each session
KINECT_CALIBRATION_PER_STREAM: bool = False          # if True, calibrate RGB and IR separately
KINECT_CALIBRATION_REUSE_TOLERANCE_HOURS: float = 6  # auto-reuse if recent
```

---

## §16.4 (NEW) — Sensor comparison harness

This is the tool that actually answers the empirical question — does Kinect detect mosquitoes the GoPro misses? Lives at `tools/compare_sensors.py`. Run after a recorded session with both sensors active and `--dry-fire`.

### What it does

1. Loads a session JSONL recording.
2. Replays detection events from each sensor.
3. For each detection, checks for a corresponding detection in the other sensor within ±FUSION_PROJECTION_MAX_DT_MS and within max_assoc_distance_norm in shared coordinates.
4. Bins detections into:
   - **Both** — detected by both sensors at roughly the same place and time
   - **GoPro only** — present in GoPro stream but no nearby Kinect detection
   - **Kinect only (RGB)** — present in Kinect RGB but no nearby GoPro detection
   - **Kinect only (IR)** — present in Kinect IR but no nearby GoPro detection
   - **All Kinect, no GoPro** — the interesting set if Cole's hypothesis is right
5. Writes a comparison report.

### Expected report shape

```
SENSOR COMPARISON — session_2026-05-10_140532.jsonl
Duration: 312 seconds
Total detections (raw):
  gopro:      1284
  kinect_rgb:  892
  kinect_ir:  1117

Cross-sensor association window:
  ±100ms, ±5% normalized

Detection categorization:
  Both (gopro + kinect_rgb):           754  (58.7% of gopro)
  Both (gopro + kinect_ir):            612  (47.7% of gopro)
  GoPro only (no kinect at all):       456  (35.5% of gopro)
  Kinect only (any stream, no gopro):  389  (NEW from kinect)

Detection rate per minute:
  gopro:        247/min
  kinect_rgb:   172/min
  kinect_ir:    215/min
  union:        323/min

Per-stream true-positive rate (post-classifier):
  [requires labeled ground truth — populate with --labels FILE]

Spatial heatmap saved to: reports/heatmap_2026-05-10_140532.png
Per-detection details:    reports/detections_2026-05-10_140532.csv
```

### Key implementation notes

- **Kinect→GoPro projection.** To compare positions, you need to project Kinect detections into GoPro's normalized frame. This requires the Kinect→GoPro extrinsic calibration (separate from Kinect→GALVO; see §4.3 in v0.2). Run that calibration once per Kinect placement.
- **The "no nearby" set is what matters most.** If "Kinect only" is large and dominated by IR, Cole's hypothesis is supported and the architecture decision swings toward Mode C (fused_interp).
- **False positive contamination.** Without labeled ground truth, you can't say "Kinect saw 389 mosquitoes the GoPro missed" — only "Kinect saw 389 *things classified as mosquito candidates* that the GoPro didn't classify." For a clean answer, you also need a manual-labeling pass over a session subset (operator clicks "real" or "false" on each candidate via a review GUI).
- **Run-time visualization.** Optional: a live mode that overlays both sensors' detections on a single frame in different colors so you can eyeball the disagreement during operation.

### Settings

```python
COMPARE_TOOL_ASSOC_DT_MS: float = 100.0
COMPARE_TOOL_ASSOC_DISTANCE_NORM: float = 0.05
COMPARE_TOOL_OUTPUT_DIR: Path = BASE_DIR / "reports"
```

---

## §17 — Implementation order (AMEND, addition)

Insert two steps into the v0.2 amended order:

### Step 9 — Sensor implementations (REVISED)

(Existing v0.2 step.) Each sensor produces valid `SensorFrame` events with proper width/height/timestamp_uncertainty. **Plus**: each sensor exposes its `role` property based on `SENSOR_ROLES` config. The startup validator refuses to start if a sensor has `role=targeting` but no calibration loaded.

### Step 9.5 (NEW) — Kinect calibration to GALVO + cross-sensor extrinsic

Required if `TARGETING_MODE != "gopro_only"`. Two sub-calibrations:

1. **Kinect→GALVO** (per-stream if `KINECT_CALIBRATION_PER_STREAM`): same patterns as GoPro, observed by Kinect.
2. **Kinect→GoPro extrinsic**: for fusion mode and the comparison harness. Use either a shared marker visible to both, or correspondence via the laser dot fired during the Kinect calibration (since it's the same dot in the same physical location, the GoPro sees it too — cheap re-use of calibration data).

### Step 9.7 (NEW) — Sensor comparison harness validation run

Before going further, run a dry-fire session with both sensors active. Generate the comparison report. Three possible outcomes:

| Outcome | Action |
|---|---|
| GoPro detects strictly more than Kinect (no Kinect-unique detections) | Stay in `gopro_only` mode. v0.2 design wins. |
| Kinect catches significant unique detections in IR | Switch default to `fused_interp`. Validate worth the latency budget impact. |
| Kinect catches significantly more in RGB *and* IR than GoPro | Consider `kinect_only` for at least some scenarios. Validate against operator-labeled ground truth. |

This step is the empirical decision point. The architecture supports all three; the data picks.

### Step 10 — ShotPattern + targeting layer (UNCHANGED)

(Existing v0.2 step. ShotPattern is sensor-agnostic; aim coordinates come from the unified track stream regardless of source.)

### Step 11+ — All subsequent steps unchanged from v0.2.

---

## §18 — Open questions (AMEND, additions)

Add to the v0.2 list:

12. **Does Kinect IR actually see mosquitoes better than GoPro RGB?** The hypothesis under test. The comparison harness (§16.4) answers this empirically. Predicted answer: yes for some lighting conditions (dim rooms, against busy backgrounds), no for others (bright daylight, plain ceilings). If the answer is consistent enough, Mode C becomes default.
13. **Is Kinect's 30 fps fast enough for fast-moving mosquitoes?** Mosquitoes are reported at 1-2 m/s peak speed, ~300 wing-beats/sec. At 30 fps that's 30-60 mm of motion per frame, which is several Kinect-IR pixels at typical range. Tractable for a well-tuned tracker. Validation: run the comparison harness against fast-flying targets specifically.
14. **Does Kinect IR+depth give "free" depth attribution to GoPro detections, or is the cross-sensor extrinsic too noisy?** If GoPro detection has its world-position pinned by associating with a Kinect detection within fusion window, you get depth-aware safety reasoning for free. But association noise could attribute the wrong depth to a target. Validate against a target at known depth.
15. **What's the latency budget impact of fused_interp mode?** Adding sensor association adds compute cost per frame. Acceptable if it stays under ~5ms total. Measure with `latency_sample` events while running fused_interp vs gopro_only.

---

## Appendix S — Quick-start for testing the hypothesis

The path from "I want to know if Kinect is better" to actually knowing:

1. **Confirm the architecture works for both modes.** Run the SHA204 cold test (`sha204_cold_test.py`) first. If that fails, the whole thing is on hold.
2. **Get GoPro working in TARGETING role.** v0.2 step order. End state: GoPro→GALVO calibration validated multi-depth.
3. **Get Kinect working in SAFETY role.** v0.2 step order. End state: Kinect publishes RGB+depth+IR frames on the bus, depth attribution works.
4. **Add Kinect→GALVO calibration code** (this addendum, §8.10). End state: per-session Kinect calibration produces a valid v2 JSON.
5. **Add Kinect→GoPro extrinsic calibration** (this addendum, §17 step 9.5). End state: a Kinect detection can be projected into GoPro normalized space.
6. **Add the comparison harness** (this addendum, §16.4). End state: `tools/compare_sensors.py recording.jsonl` produces a report.
7. **Run a real session in dry-fire with both sensors.** 5-10 minutes of recording, with at least some real or simulated mosquito-like targets in scene (a small black dot on a stick waved around works for development).
8. **Read the report.** Decide which mode wins.
9. **(Optional) Label a session subset** for ground-truth validation. This converts "candidate count" into "true positive count" and removes noise contamination from the answer.

The honest version: steps 1-3 are weeks of work. Steps 4-6 are days. Step 7 is fast. Step 8 is the moment you actually learn something.

---

## End

Apply this v0.2.1 delta on top of the v0.2 amendments. Keep the file structure parallel — these additions don't conflict with any v0.2 sections; they extend them.

If you read this and decide your hypothesis can't be tested cleanly without ground-truth labeling, that's worth knowing before you build all of it. The labeling tool isn't in this doc — it's a small GUI that walks you through session detections and lets you click "real" or "false" — but it's the missing piece for going from "Kinect found more candidates" to "Kinect actually performs better."
