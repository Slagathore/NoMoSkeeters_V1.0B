# BOOTSTRAP — AMENDMENTS v0.2

**A delta document against `BOOTSTRAP.md` v0.1.** Apply these in place to that doc, or live alongside it as the canonical "v0.2 changes" reference until merged.

> Status: Drafted from review feedback (laserdesign_Review_Feedback.pdf) + Cole's clarified hardware geometry (GoPro rigidly mounted to LaserCube body, Kinect mobile/unmounted).
> Author: Claude, with Cole's directives.
> Last updated: 2026-05-10
> Convention: section numbers below mirror BOOTSTRAP.md v0.1. **NEW** sections are insertions; **AMEND** sections replace existing prose.

---

## 0. Read this first — the architectural shift

The single most consequential update is geometric:

- **The GoPro Hero 13 is rigidly bolted to the LaserCube body.** Optical axis and laser aperture share a baseline of roughly 5–10 cm. The only mutable degree of freedom is GoPro vertical tilt (bolt loosening).
- **The Kinect v2 is *not* rigidly mounted.** Operator places it relationally per session, but the relative pose between Kinect and laser is not preserved across sessions and may drift mid-session.

This forces an asymmetric sensor-role architecture that v0.1 didn't commit to:

| Sensor | Role | Calibration cadence |
|---|---|---|
| **GoPro** | TARGETING — drives laser aim | Per (mount-tilt-config, scene). Persists across sessions. Recalibrate when GoPro tilt bolt is touched, or scene changes substantially. |
| **Kinect** | SAFETY + WORLD-REASONING — depth, body tracking, 3D exclusion zones | Per-session. Stored as Kinect-internal world coords. Does NOT need a homography to galvo space because it does not drive aim. |
| **Local cam** | TARGETING fallback | Per (lens position, scene). |

This means: **multi-sensor fusion is no longer symmetric.** The Tracker does not promote a Kinect-only detection to a fire-able track. Kinect's contribution is to attach depth/world attributes to GoPro tracks via spatial association in the GoPro frame, after Kinect→GoPro extrinsic alignment (a separate per-session step).

If Kinect moves mid-session, its alignment is invalid; targets get marked "depth-unknown." The system continues to operate on GoPro alone with reduced safety context.

> Note: This collapses several open architectural ambiguities in v0.1 — particularly the symmetric "fused mode" in section 5.6 and the ambiguous "K-DEPTH-PX → NORM" arrow in section 4.2.

---

## A. Punch list (scannable)

If you do nothing else, do these. Listed by section number.

### Code-level bugs (fix immediately, low risk, low effort)
- **§5.1** `Sensor.normalize()` references `self.width/self.height`, which the ABC doesn't define. Fix per §5.1 amendment below.
- **§5.3** `KinectV2Sensor.read()` doesn't set `frame.width/frame.height`. Downstream divides by zero.
- **§6.1** `_build_detection(frame, c, label, conf)` is called with `label`/`conf` undefined when classifier is None. Set defaults before the branch.
- **§9.5** `_send_cmd_recv()` accepts the first datagram off the socket. Filter by `(src_ip, src_port)` and reject responses with bad command echo or `status != 0`.
- **§9.5** `disconnect()` only sends `SET_OUTPUT(0)` if `self._connected`. Should attempt it best-effort if the cmd socket exists, regardless of connected state.
- **§9.5** `_buffer_free` decrement-on-send goes stale on dropped UDP replies. Treat as estimate; never gate on it without a recent ground-truth refresh.

### Architectural changes (real work, real value)
- **§3.1** Add formal `events/schemas.py` module — explicit dataclasses for every bus payload (`FrameEvent`, `DetectionEvent`, `TrackEvent`, `TargetCommandEvent`, `LaserStatusEvent`, `LatencySample`).
- **§4** Add 7th coordinate space concept: **LASER-RAY** (the laser commands a galvo angle/ray, not a world point). Document the rigid-mount assumption explicitly.
- **§5** Add `SensorRole` enum and reframe Kinect role.
- **§5** Add `timestamp_uncertainty_ms` to `SensorFrame` for fusion alignment.
- **§6** Make Hu moments optional — they are noise on sub-5px blobs; validate empirically.
- **§7** Add separate `fire_eligible` boolean on `TrackedTarget`, computed from confirmation/freshness/non-coasting/confidence. Surface it in the GUI as its own state, not as gating. Operator-visible, operator-controllable.
- **§8** Add multi-depth calibration validation step. **This is non-optional given small parallax baseline.**
- **§8** Drag-line must stream chunked with backpressure, not be flushed as a single frame.
- **§8** Drag-line at multiple speeds to decompose galvo dynamics from software lag.
- **§9** Add **heartbeat daemon thread** that runs independent of main thread. Critical for debugging without losing the cube to the 4-second comms timer.
- **§9** Add **ShotPattern abstraction** — converts `(target_galvo, dwell_ms)` into a stream of micro-frame samples. The cube does not "park"; it scans.
- **§9** Add cold-test of `SET_OUTPUT` before committing further engineering — establishes whether SHA204 handshake is required.

### Implementation order reorg
Move dry-fire transport, heartbeat, golden fixtures, and event schemas before any live LaserCube method exposure. See §17 amendment.

### New tests required
- Golden binary fixtures: `golden_full_info_64b.bin`, `golden_ringbuf_empty_4b.bin`, `golden_alive_2b.bin`.
- `test_full_info_parser_golden_fixture.py`
- `test_lasercube_transport_dry_run.py`
- `test_calibration_roundtrip.py` (norm → galvo → norm should be identity within tolerance)
- `test_latency_budget.py` (synthetic detection through full pipeline, asserts on max ms)
- `test_heartbeat_continues_during_main_block.py` (deliberately block main thread, verify cube stays alive)

---

## §2 — Glossary (AMEND, additions)

Add the following entries; keep existing ones intact (yes, including uWu mode):

| Term | Meaning |
|---|---|
| **SensorRole** | The architectural job a sensor does: TARGETING (drives aim), SAFETY (drives no-fire reasoning), or FALLBACK (replaces TARGETING when primary unavailable). Set per-sensor in config. |
| **LASER-RAY** | The galvo-angle direction commanded to the cube. The laser does not address a world point; it addresses a ray. Two cameras at different positions seeing the "same" world point will see it on different rays. |
| **Mount-tilt-config** | A specific GoPro vertical-tilt setting on the laser-body bracket. Calibration is valid for one mount-tilt-config and must be redone if the bolt is loosened/re-tightened to a different angle. |
| **ShotPattern** | The micro-frame of samples sent to the cube to "hit" a target. The galvo always scans; we do not park it. Patterns include `dot_repeat`, `micro_circle`, `figure_eight`, `dot_blank_dot`. Pattern shape is empirical and tunable per situation. |
| **Heartbeat** | Periodic `GET_FULL_INFO` sent to the cube to keep its 4-second internal comms timer alive. Runs in a daemon thread that does not block on main-thread breakpoints. |
| **Fire eligibility** | A computed boolean on a TrackedTarget — whether it currently meets the criteria for laser authorization (recently updated, confirmed, non-coasting, above confidence threshold). Surfaced in the GUI as state, not enforced as a hidden gate. |
| **Golden fixture** | A captured raw byte response from real hardware (e.g. `golden_full_info_64b.bin`) checked into the repo for parser regression tests. The test loads the bytes from disk and asserts the parser produces the known-correct dataclass. |
| **Galvo dynamics** | The mechanical lag between commanding a new galvo position and the mirrors physically arriving. Small steps: tens of microseconds. Large steps: milliseconds. Conflated with software latency in naive drag-line measurement. |

---

## §3.1 — The bus pattern (AMEND, addition)

Add a subsection **§3.1.1 Event schemas** with the following content:

The bus exists conceptually in v0.1 but the payloads are scattered. Define them once in `events/schemas.py`:

```python
# events/schemas.py
from dataclasses import dataclass, field
from typing import Optional
import numpy as np


@dataclass(frozen=True)
class FrameEvent:
    """Raw sensor frame ready for downstream processing."""
    timestamp: float                  # monotonic, set at capture
    sensor_id: str                    # "gopro", "kinect_v2", "local_cam"
    sensor_role: str                  # SensorRole.TARGETING etc.
    rgb: Optional[np.ndarray]
    depth: Optional[np.ndarray]
    ir: Optional[np.ndarray]
    width: int
    height: int
    timestamp_uncertainty_ms: float = 5.0  # see §5 amendment


@dataclass(frozen=True)
class DetectionEvent:
    """A single candidate detection from a frame."""
    timestamp: float                  # frame timestamp it came from
    sensor_id: str
    detection_id: int                 # local-to-frame id
    x_norm: float                     # NORM space, see §4
    y_norm: float
    x_px: int                         # for overlay rendering
    y_px: int
    area_pixels: float
    bbox: tuple                       # (x, y, w, h) px
    classifier_label: str             # "candidate" if no classifier ran
    classifier_confidence: float      # 0.5 default if no classifier ran
    # Depth attributes — populated by Kinect-as-safety association if available
    z_world_m: Optional[float] = None
    x_world_m: Optional[float] = None
    y_world_m: Optional[float] = None


@dataclass
class TrackEvent:
    """Updated track state after the tracker runs on a frame."""
    timestamp: float
    track_id: int
    sensor_id: str                    # which sensor track originated from
    state_norm: tuple                 # (x_norm, y_norm) and optional z
    velocity_norm: tuple              # (vx_norm, vy_norm) per second
    age_frames: int
    confirmed: bool
    coasting: bool
    confidence: float
    fire_eligible: bool               # see §7 amendment
    last_detection_age_ms: float


@dataclass(frozen=True)
class TargetCommandEvent:
    """Computed laser aim point for a track."""
    timestamp: float
    track_id: int
    target_x_galvo: int
    target_y_galvo: int
    pattern_id: str                   # which ShotPattern to use
    dwell_ms: int


@dataclass(frozen=True)
class LaserStatusEvent:
    """Periodic status from the cube."""
    timestamp: float
    output_enabled: bool
    interlock_ok: bool
    over_temp: bool
    buffer_free: int                  # last refresh
    buffer_free_age_ms: float         # how stale is this number
    dac_rate: int
    packet_errors_since_last: int


@dataclass(frozen=True)
class LatencySample:
    """One end-to-end pipeline latency measurement."""
    timestamp: float
    detection_to_target_command_ms: float
    capture_to_detection_ms: float
    target_command_to_send_ms: float
    total_ms: float
```

Every bus emitter constructs and emits one of these. Every consumer accepts the dataclass — no `dict`-shaped events, no positional-arg drift.

---

## §4 — Coordinate systems contract (AMEND)

### §4.1 — Defined coordinate systems (AMEND)

Add a 7th conceptual space (not always materialized in code, but documented for reasoning):

| ID | Space | Range | Units | Where it lives |
|---|---|---|---|---|
| **L-RAY** | Laser-ray angle/direction | x, y in galvo units | unitless 12-bit | What the cube *actually* commands. A point in L-RAY corresponds to a ray in world space, not a world point. |

### §4.2 — Conversion flow (AMEND, full replacement)

```
GoPro (rigidly mounted, fixed extrinsics to laser)
   GP-PX ──► NORM ──► CoordinateMapper.fwd(NORM) ──► GALVO ≈ L-RAY direction
                          │
                          ▼
                   per-(mount-tilt, scene) homography H_norm_to_galvo

Kinect (mobile, world-reasoning role)
   K-RGB-PX ──► (NORM) ──► used for visualization overlays only, NOT aim
   K-DEPTH-PX ──► K-WORLD ──► consumed by safety reasoning (no-fire zones, range guard)
                  │
                  ▼
   K-WORLD coords are associated to GoPro tracks via Kinect→GoPro extrinsic
   (separate per-session calibration). Result: GoPro track gains
   (z_world, x_world, y_world) attributes from the nearest Kinect detection.
```

### §4.3 — Conversion rules (AMEND, additions)

Add:
- **GP-PX → GALVO**: composition of GP-PX → NORM → GALVO via the per-(mount-tilt, scene) homography. **Validity assumption: parallax baseline (camera lens to laser aperture) is small relative to target depth.** Validate per §8 multi-depth check.
- **K-WORLD → GoPro track attribute**: requires per-session Kinect→GoPro extrinsic calibration. If absent, Kinect contributes no depth attribution, only standalone safety zones in K-WORLD.
- **NEW** Reverse conversions (`H_galvo_to_norm`, `H_galvo_to_px`) MUST be precomputed and stored in the calibration JSON — never invert at runtime in the hot path.

### §4.6 (NEW) — Geometric assumption documentation

> **V1 assumption**: The GoPro lens and laser aperture are rigidly co-mounted with a baseline of approximately 5–10 cm. For targets at depth ≥ 1 m, this baseline produces angular parallax error of ≤ 6° (worst case at 1 m). At ≥ 2 m operating depth, error is ≤ 3°. The homography from GP-NORM to GALVO is treated as valid across the operating volume *iff* multi-depth validation (§8.6) shows reprojection error within the configured `CALIBRATION_MAX_RESIDUAL_NORM` threshold at three depths spanning the operating range.
>
> If multi-depth validation fails, escalate to a depth-aware mapper (deferred to v0.3, requires Kinect alignment to GoPro frame and a `(norm_x, norm_y, depth) → galvo` regression).

---

## §5 — Sensor layer (AMEND)

### §5.1 — Sensor ABC (AMEND, replace `normalize`)

```python
@dataclass
class SensorFrame:
    timestamp: float
    sensor_id: str
    sensor_role: str                  # NEW: SensorRole.TARGETING etc.
    rgb: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None
    ir: Optional[np.ndarray] = None
    width: int = 0
    height: int = 0
    timestamp_uncertainty_ms: float = 5.0   # NEW: for cross-sensor fusion alignment


class Sensor(ABC):
    @property
    @abstractmethod
    def sensor_id(self) -> str: ...

    @property
    @abstractmethod
    def role(self) -> "SensorRole": ...
    
    # ... existing abstract methods ...

    # FIXED: take frame as the source of truth for dims, not self.
    @staticmethod
    def normalize(frame: SensorFrame, x_px: float, y_px: float) -> tuple[float, float]:
        """Convert pixel coords to normalized [0,1]² coords using the frame's
        actual dimensions, not the sensor's nominal dimensions (which may
        not match if the sensor renegotiated resolution mid-stream)."""
        return (
            x_px / max(1, frame.width - 1),
            y_px / max(1, frame.height - 1),
        )
```

Add the role enum:

```python
# sensors/base.py — top of file
from enum import Enum

class SensorRole(str, Enum):
    TARGETING = "targeting"   # drives aim
    SAFETY    = "safety"      # drives no-fire/world reasoning
    FALLBACK  = "fallback"    # used if TARGETING is unavailable
```

### §5.3 — Kinect implementation (AMEND, replace `read()`)

```python
def read(self) -> Optional[SensorFrame]:
    frame = SensorFrame(
        timestamp=time.monotonic(),
        sensor_id=self.sensor_id_str,
        sensor_role=SensorRole.SAFETY,    # NEW: see §0 architectural shift
    )
    
    if self._kinect.has_new_color_frame():
        color = self._kinect.get_last_color_frame().reshape((1080, 1920, 4))
        frame.rgb = cv2.cvtColor(color, cv2.COLOR_BGRA2BGR)
        # FIXED: Kinect can present multiple resolutions across streams.
        # The "primary" dimensions are RGB-aligned because that's the most
        # commonly addressed stream. Depth dimensions are tracked separately
        # if needed (see frame.depth.shape).
        frame.width, frame.height = 1920, 1080
    
    if self._kinect.has_new_depth_frame():
        depth_mm = self._kinect.get_last_depth_frame().reshape((424, 512))
        frame.depth = depth_mm.astype(np.float32) * 0.001
        if frame.width == 0:
            # No RGB this tick; fall back to depth dims so downstream
            # normalization doesn't divide by zero.
            frame.width, frame.height = 512, 424
    
    if self._kinect.has_new_infrared_frame():
        frame.ir = self._kinect.get_last_infrared_frame().reshape((424, 512))
    
    # USB/SDK jitter is ~5-10ms; document explicitly so fusion knows.
    frame.timestamp_uncertainty_ms = 8.0
    
    if frame.rgb is None and frame.depth is None and frame.ir is None:
        return None
    return frame
```

### §5.6 — Multi-sensor fusion strategy (AMEND, full replacement)

Three modes, asymmetric:

- **Single TARGETING sensor**: GoPro (or local cam fallback) drives detection → tracker → aim. No Kinect.
- **TARGETING + SAFETY**: GoPro drives aim. Kinect runs in parallel emitting K-WORLD detections that are *associated* (not merged) into GoPro tracks via Kinect→GoPro extrinsic transform. Result: a GoPro track gains a `z_world_m` attribute when Kinect saw it too. Used by safety reasoning.
- **Parallel diagnostic**: each sensor maintains independent tracks for debugging UI. No fusion. Only one stream is fire-eligible.

**Removed from v0.1**: the symmetric "Fused mode where each sensor's detections merge into shared tracks." This was wrong. Kinect detections do not fire the laser.

Multi-sensor temporal alignment, when association is needed:
- Both sensors timestamp frames at receipt with `time.monotonic()`.
- Each frame carries `timestamp_uncertainty_ms`.
- The Tracker does not associate Kinect detections with GoPro tracks across windows wider than `(GP_uncertainty + K_uncertainty)`. Default rejection threshold: 25 ms.

---

## §6 — Detection layer (AMEND)

### §6.1 — Pipeline (AMEND, fix the `label`/`conf` bug)

```python
def detect_bgsub(self, frame: np.ndarray) -> list[Detection]:
    # ... grayscale, blur, MOG2, threshold, morph, contours ...
    
    detections = []
    for c in contours:
        area = cv2.contourArea(c)
        if not (settings.DETECTION_MIN_AREA <= area <= settings.DETECTION_MAX_AREA):
            continue
        
        # FIXED: defaults set BEFORE the optional classifier branch so that
        # _build_detection() always has valid label/conf even when classifier
        # is disabled or absent.
        label = "candidate"
        conf = 0.5
        
        if self._classifier is not None:
            label, conf = self._classifier.classify(frame, c)
            if label != "mosquito":
                continue
        
        detections.append(self._build_detection(frame, c, label, conf))
    return detections
```

### §6.2 — The classifier (AMEND, Hu moments demoted to optional)

The 10-feature vector in v0.1 includes 4 Hu moments. Empirical concern: at typical mosquito range (2–6 m on the GoPro), targets are 2–5 px blobs; Hu moments computed on a contour of fewer than ~20 px are dominated by sub-pixel quantization noise and contribute mostly variance rather than signal.

Make Hu features opt-in:

```python
# config/settings.py — classifier section
CLASSIFIER_USE_HU_MOMENTS: bool = False     # Default OFF until empirically validated
CLASSIFIER_USE_AREA_RATIO: bool = True
CLASSIFIER_USE_ASPECT: bool = True
CLASSIFIER_USE_SOLIDITY: bool = True
CLASSIFIER_USE_EXTENT: bool = True
CLASSIFIER_USE_EQUIV_DIAM: bool = True
CLASSIFIER_USE_PERIM_SQ: bool = True
```

Training pipeline addendum (§6.4): when retraining with collected session data, validate cross-validation accuracy with `CLASSIFIER_USE_HU_MOMENTS=True` vs `False`. Pick whichever yields higher held-out accuracy on small-blob samples (area < 50 px²). Document the decision in `models/classifier_meta.json` next to the pickle.

---

## §7 — Tracking layer (AMEND)

### §7.2 — Per-track state (AMEND, add `fire_eligible`)

```python
@dataclass
class TrackedTarget:
    # ... existing fields ...
    
    # NEW: surfaced as a separate visible state, not as enforcement.
    # The operator sees track state AND fire eligibility independently.
    # This is cosmetic/diagnostic by default; the operator decides what
    # to do with it.
    fire_eligible: bool = False
    fire_eligible_reason: str = ""   # human-readable, for GUI tooltip
```

Compute on every track update:

```python
def _compute_fire_eligibility(self, t: TrackedTarget, now: float) -> tuple[bool, str]:
    """Pure function. Inputs: track state. Outputs: bool + reason string.
    Operator can override or ignore in their own logic; this is computed,
    not enforced."""
    age_ms = (now - t.last_update_ts) * 1000.0
    if not t.confirmed:
        return False, f"unconfirmed ({t.age_frames} frames)"
    if t.coast_frames > 0:
        return False, f"coasting {t.coast_frames} frames"
    if age_ms > settings.FIRE_ELIGIBILITY_MAX_AGE_MS:
        return False, f"stale {age_ms:.0f}ms"
    if t.confidence < settings.FIRE_ELIGIBILITY_MIN_CONFIDENCE:
        return False, f"conf={t.confidence:.2f}"
    return True, "ok"
```

Add config:

```python
# config/settings.py — tracking section
FIRE_ELIGIBILITY_MAX_AGE_MS: float = 50.0       # track update freshness
FIRE_ELIGIBILITY_MIN_CONFIDENCE: float = 0.7    # tunable
```

Note explicitly: this is a *signal*, not a *gate*. The GUI surfaces it. The operator decides whether to wire it into a manual-fire button enable, or ignore it. Per Cole's directive — feature, not lockout.

---

## §8 — Calibration (AMEND, multiple additions)

### §8.5 (NEW) — Calibration directionality

Calibration physically observes laser dots in camera pixels: you fire a known galvo coord, the camera detects a pixel, giving you a `(galvo_x, galvo_y) → (px_x, px_y)` correspondence. After normalization that's `GALVO → NORM`, the *inverse* of what the targeting pipeline needs.

Two valid implementations:

**Option A — fit forward, invert at calibration time:**
```python
H_galvo_to_norm, _ = cv2.findHomography(galvo_pts, norm_pts)
H_norm_to_galvo = np.linalg.inv(H_galvo_to_norm)
```

**Option B — fit inverse directly:**
```python
H_norm_to_galvo, _ = cv2.findHomography(norm_pts, galvo_pts)
H_galvo_to_norm = np.linalg.inv(H_norm_to_galvo)
```

Both work; Option B is fewer lines but Option A is numerically more stable when calibration points are clustered in galvo space (which Halton/grid patterns ensure they aren't, but it's a sane default).

**Store BOTH matrices in the calibration JSON.** Never invert at runtime in the hot path.

```json
{
  "schema_version": 2,
  "sensor_id": "gopro",
  "mount_tilt_config": "tripod_garage_lvl",
  "scene": "garage_default",
  "n_points": 25,
  "pattern": "halton",
  "H_norm_to_galvo": [[...], [...], [...]],
  "H_galvo_to_norm": [[...], [...], [...]],
  "residual_norm": 0.0042,
  "residual_galvo": 14.3,
  "validation": {
    "depths_m": [1.0, 2.5, 4.0],
    "residuals_norm": [0.0091, 0.0040, 0.0058],
    "passed": true
  }
}
```

### §8.6 (NEW) — Multi-depth validation

After fitting at one depth (the calibration target plane), validate at additional depths inside the operating volume:

```python
# targeting/calibration.py
def validate_multi_depth(mapper: CoordinateMapper,
                         test_targets: list[TestTarget]) -> ValidationResult:
    """test_targets is a list of (galvo_xy, observed_px_xy, depth_m) tuples,
    captured at multiple distances during the validation sweep.
    
    Returns per-depth residuals and a pass/fail boolean."""
    by_depth = group_by_depth_bin(test_targets, bin_size_m=0.5)
    residuals = {}
    for depth_m, group in by_depth.items():
        errors = [
            np.linalg.norm(mapper.galvo_to_norm(*t.galvo) - t.observed_norm)
            for t in group
        ]
        residuals[depth_m] = np.mean(errors)
    max_residual = max(residuals.values())
    return ValidationResult(
        per_depth_residual_norm=residuals,
        max_residual_norm=max_residual,
        passed=max_residual < settings.CALIBRATION_MAX_RESIDUAL_NORM,
    )
```

Settings:
```python
CALIBRATION_MAX_RESIDUAL_NORM: float = 0.015   # ~1.5% normalized = roughly 1cm @ 2m
CALIBRATION_VALIDATION_DEPTHS_M: list = [1.0, 2.5, 4.0]
```

Failure of multi-depth validation indicates the planar homography is insufficient for the operating volume — escalate to a depth-aware mapper (v0.3 work).

### §8.7 (NEW) — Drag-line streaming with backpressure

The naive `dragline_path()` from v0.1 returns `duration_s × dac_rate` points (e.g. 60,000 for 2s at 30k pps). The cube's ringbuffer is 6,000 samples. You cannot send the path as one frame.

```python
def stream_dragline(cube: LaserCubeInterface,
                    path: list[tuple[float, float]],
                    dac_rate: int,
                    target_buffer_free: int = 5000) -> Iterator[StreamProgress]:
    """Stream a long path to the cube as 140-sample chunks, respecting
    ringbuffer backpressure. Yields per-chunk progress with timestamps
    so the calibration controller can match commanded-vs-observed by time."""
    chunk_size = MAX_SAMPLES_PER_PACKET   # 140
    i = 0
    while i < len(path):
        # Block until cube has room. get_ringbuf_empty is cheap UDP roundtrip.
        free = cube.get_ringbuf_empty()
        if free is not None and free < target_buffer_free:
            # Cube is full enough; sleep until ~one chunk worth scans out.
            time.sleep(chunk_size / dac_rate)
            continue
        chunk = path[i:i + chunk_size]
        send_ts = time.monotonic()
        ok = cube.send_chunk_raw(chunk)
        # Estimated scan-out time of this chunk:
        # samples up to and including this chunk will leave the ringbuffer
        # within (chunk_size / dac_rate) seconds of being added.
        scan_out_est_ts = send_ts + (chunk_size / dac_rate)
        yield StreamProgress(
            sent_at=send_ts,
            scan_out_est_at=scan_out_est_ts,
            chunk_start_idx=i,
            chunk_size=len(chunk),
        )
        i += chunk_size
```

The calibration controller uses `scan_out_est_at` to match each commanded chunk to the observed laser dot in the camera frame — that's the correspondence.

### §8.8 (NEW) — Galvo dynamics in latency estimation

Drag-line at *one* speed conflates software lag and galvo settling time. Run drag-line at multiple speeds:

```python
DRAGLINE_SWEEP_SPEEDS_NORM_PER_SEC = [0.05, 0.20, 0.50, 1.00]  # slow → fast
```

For each speed `v`, measure `total_lag(v)` from commanded-time to observed-pixel-time. Fit:

```
total_lag(v) = software_lag + alpha * v
```

`software_lag` (intercept) is the pipeline lag we want to compensate for in real-time aim. `alpha` (slope) is the galvo dynamic constant — useful diagnostic but not directly used in aim correction (galvo dynamics are inside the cube's PID loop, we can't drive them). 

Settings:
```python
CALIBRATION_DRAGLINE_MULTI_SPEED: bool = True
LATENCY_SOFTWARE_LAG_MS: float = 0.0  # populated by calibration result
```

Lead-aim in the targeting layer uses `LATENCY_SOFTWARE_LAG_MS` to predict where the target *will be* by the time the laser arrives.

### §8.9 (NEW) — Per-sensor calibration storage strategy

Calibrations live at `user_data/calibrations/<sensor_id>/<profile_name>.json`.

| Sensor | Profile naming | When to recalibrate |
|---|---|---|
| **GoPro** | `<mount-tilt-config>_<scene>.json` e.g. `tripod_lvl_garage.json` | Bolt loosened/re-tightened, scene changed (furniture moved enough to alter useful operating volume), new room |
| **Kinect** | `<scene>_session_<timestamp>.json` (or none if Kinect→GoPro extrinsic only) | Every session; Kinect physically moved |
| **Local cam** | `<lens-position>_<scene>.json` | Lens twisted, scene changed |

Kinect→GoPro extrinsic alignment is a separate calibration step (an aruco/checkerboard or laser-dot-shared correspondence between the two views). Stored at `user_data/calibrations/kinect_to_gopro/<session>.json`. If absent, Kinect contributes only K-WORLD safety zones, no track-attribute association.

---

## §9 — LaserCube protocol (AMEND)

### §9.5 — Reference implementation (AMEND, multiple fixes)

Replace `_send_cmd_recv()` with a strict-validation version:

```python
def _send_cmd_recv(self,
                   payload: bytes,
                   expect_min_bytes: int = 1,
                   expect_echo_byte: Optional[int] = None) -> Optional[bytes]:
    """Send command on CMD_PORT and wait for a reply matching:
      - source IP == self.ip
      - source port == self.cmd_port
      - len(reply) >= expect_min_bytes
      - reply[0] == expect_echo_byte (if provided)
      - reply[1] == 0x00 (success status, if reply has >= 2 bytes)
    Loops until match or timeout. Drops mismatched datagrams (could be
    LaserOS or stale crud on the same socket)."""
    if expect_echo_byte is None:
        expect_echo_byte = payload[0]
    deadline = time.monotonic() + self.reply_timeout_s
    
    with self._lock:
        if self._cmd_sock is None:
            return None
        try:
            self._cmd_sock.sendto(payload, (self.ip, self.cmd_port))
        except OSError:
            return None
    
    while time.monotonic() < deadline:
        remaining = max(0.01, deadline - time.monotonic())
        try:
            self._cmd_sock.settimeout(remaining)
            data, addr = self._cmd_sock.recvfrom(4096)
        except socket.timeout:
            return None
        except OSError:
            return None
        
        # Source filter
        if addr[0] != self.ip or addr[1] != self.cmd_port:
            continue   # not from our cube; drop
        # Length filter
        if len(data) < expect_min_bytes:
            continue
        # Command echo filter
        if expect_echo_byte is not None and data[0] != expect_echo_byte:
            continue
        # Status filter (if reply has a status byte)
        if len(data) >= 2 and data[1] != 0x00:
            # cube reports failure; surface to caller as None
            return None
        return data
    return None
```

Replace `disconnect()` with best-effort:

```python
def disconnect(self) -> None:
    """Disable output (best-effort, even on partial state) and close sockets."""
    with self._lock:
        # Try to disable output if the cmd socket exists at all.
        # We don't trust _connected — could be stale.
        if self._cmd_sock is not None:
            try:
                # repeat=2 in case of UDP loss
                self._send_cmd_no_reply(
                    bytes([LC_CMD_SET_OUTPUT, 0x00]), repeat=2)
            except Exception:
                pass   # failure is logged; we're going down anyway
        self._close_sockets_unlocked()
        self._connected = False
```

Add buffer-state staleness tracking:

```python
@dataclass
class BufferEstimate:
    """Tracks ringbuffer fullness as an *estimate* with a freshness clock."""
    free: int = 0
    size: int = 6000
    last_refresh_ts: float = 0.0
    
    def age_ms(self) -> float:
        return (time.monotonic() - self.last_refresh_ts) * 1000.0
    
    def is_fresh(self, max_age_ms: float = 100.0) -> bool:
        return self.age_ms() <= max_age_ms

# In LaserCubeInterface:
self._buffer = BufferEstimate(free=0, size=6000)

def buffer_estimate(self) -> BufferEstimate:
    """Returns the LAST KNOWN buffer state. May be stale.
    Always call get_ringbuf_empty() before any decision that gates on buffer."""
    return self._buffer
```

Code that depends on buffer state must consult `buffer_estimate().is_fresh()` and refresh if stale.

### §9.10 (NEW) — Heartbeat thread architecture

The cube's 4-second comms timeout means a paused main thread (debugger breakpoint, Qt event loop hiccup, GC stutter) can lose the cube.

```python
# laser/heartbeat.py
import threading
import time

class CubeHeartbeat:
    """Independent daemon thread that sends GET_FULL_INFO at a fixed cadence
    no matter what the main thread is doing. Survives breakpoints in GUI code
    because it's a separate OS thread.
    
    Note: we are NOT a daemon process, just a daemon thread. If you set a
    breakpoint INSIDE the heartbeat thread itself, it will of course pause.
    Don't do that; or run heartbeat in a subprocess if your debugging
    requires it (overkill for V1)."""
    
    def __init__(self,
                 cube: "LaserCubeInterface",
                 interval_s: float = 1.5,
                 on_status: Optional[Callable[[LaserInfo], None]] = None):
        self._cube = cube
        self._interval = interval_s
        self._stop = threading.Event()
        self._on_status = on_status
        self._thread = threading.Thread(
            target=self._run, name="CubeHeartbeat", daemon=True)
    
    def start(self) -> None:
        self._thread.start()
    
    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop.set()
        self._thread.join(timeout=timeout_s)
    
    def _run(self) -> None:
        while not self._stop.wait(self._interval):
            try:
                info = self._cube.get_full_info()
                if info is not None and self._on_status:
                    self._on_status(info)
            except Exception:
                # Heartbeat NEVER raises. If the cube dies, we just
                # stop receiving info; the next get_full_info() call will
                # try again. Logging happens at the cube interface level.
                pass
```

Wire it up:

```python
# main.py or laser_manager.py during init
cube = LaserCubeInterface(...)
cube.connect()
heartbeat = CubeHeartbeat(cube, interval_s=1.5, on_status=publish_to_bus)
heartbeat.start()

# At shutdown:
heartbeat.stop()
cube.disconnect()
```

### §9.11 (NEW) — ShotPattern abstraction

The cube scans continuously; we never "park" the galvo. To "hit" a target at `(galvo_x, galvo_y)` for `dwell_ms`, generate a stream of samples that center on the target while keeping the galvos moving in a controlled pattern.

```python
# laser/shot_patterns.py
from abc import ABC, abstractmethod

class ShotPattern(ABC):
    """Generates a stream of LaserPoints centered on a target."""
    
    @abstractmethod
    def generate(self,
                 target_x_galvo: int,
                 target_y_galvo: int,
                 dwell_ms: int,
                 dac_rate: int,
                 power_pct: int) -> list[LaserPoint]: ...


class DotRepeat(ShotPattern):
    """Simplest: park galvo at one spot. Bad for galvo health but maximum
    energy density. Use for short bursts only. Default OFF in favor of
    MicroCircle."""
    def generate(self, x, y, dwell_ms, dac_rate, power_pct):
        n = int((dwell_ms / 1000.0) * dac_rate)
        rgb = (power_pct * 0xFFF) // 100
        return [LaserPoint(x=x, y=y, r=rgb, g=rgb, b=rgb) for _ in range(n)]


class MicroCircle(ShotPattern):
    """Tight circle around target. Keeps galvos moving, spreads optical
    energy over ~5-10mm radius depending on tuning. Default."""
    def __init__(self, radius_galvo: int = 30):
        self.radius = radius_galvo
    
    def generate(self, x, y, dwell_ms, dac_rate, power_pct):
        import math
        n = int((dwell_ms / 1000.0) * dac_rate)
        rgb = (power_pct * 0xFFF) // 100
        return [
            LaserPoint(
                x=int(x + self.radius * math.cos(2*math.pi*i/n)),
                y=int(y + self.radius * math.sin(2*math.pi*i/n)),
                r=rgb, g=rgb, b=rgb,
            )
            for i in range(n)
        ]


class FigureEight(ShotPattern):
    """Lemniscate around target. Spreads energy more than a circle, useful
    against evading targets — beam covers a wider area in the same dwell."""
    def __init__(self, scale_galvo: int = 40):
        self.scale = scale_galvo
    
    def generate(self, x, y, dwell_ms, dac_rate, power_pct):
        import math
        n = int((dwell_ms / 1000.0) * dac_rate)
        rgb = (power_pct * 0xFFF) // 100
        out = []
        for i in range(n):
            t = 2 * math.pi * i / n
            # Lemniscate of Bernoulli
            denom = 1 + math.sin(t)**2
            dx = self.scale * math.cos(t) / denom
            dy = self.scale * math.sin(t) * math.cos(t) / denom
            out.append(LaserPoint(
                x=int(x + dx), y=int(y + dy),
                r=rgb, g=rgb, b=rgb,
            ))
        return out


# Registry — config selects which to use
SHOT_PATTERNS: dict[str, ShotPattern] = {
    "dot_repeat":   DotRepeat(),
    "micro_circle": MicroCircle(radius_galvo=30),
    "figure_eight": FigureEight(scale_galvo=40),
}
```

Settings:
```python
SHOT_PATTERN_DEFAULT: str = "micro_circle"
SHOT_PATTERN_DWELL_MS: int = 80                 # operator-tunable
SHOT_PATTERN_POWER_PCT: int = 100               # 0-100, operator-tunable
```

### §9.12 (NEW) — Pre-flight: SHA204 handshake test

Before sinking weeks into the rewrite, run a single throwaway test to determine whether your specific cube + FW 0.23 requires the SHA204 handshake before `SET_OUTPUT` does anything.

Procedure:
1. Connect via probe.
2. Send `SET_OUTPUT(0x01)` cold (no SHA204 exchange).
3. Send a short test frame (single bright dot, low power, safety lens or aimed at a wall).
4. Observe whether the laser actually emits.

If yes → no handshake needed; proceed with the rewrite as designed.
If no → handshake required. Halt rewrite. Capture LaserOS's authentication exchange with Wireshark on `udp.port == 45457` while LaserOS connects, reverse the SHA204 challenge/response. This is a real project on its own — likely 1-2 weeks of work — and changes the implementation order.

This single test is the highest-leverage thing you can do before committing further. It either confirms your timeline or saves you from discovering a brick wall in week 4.

---

## §10 — Safety system (no amendment)

Per Cole's directive — keep operator-controllable defaults. The 3D no-fire zones in K-WORLD (§10.4) remain useful since that's now Kinect's actual job. No changes.

---

## §14 — Logging and diagnostics (AMEND, additions)

Add these JSONL event types beyond v0.1's list:

```python
EVENT_TYPES_ADDED = [
    "latency_sample",            # one entry per processed frame
    "buffer_state",              # cube buffer fresh-read
    "calibration_started",       # already in v0.1 — reaffirmed
    "calibration_point_observed", # NEW: per-point during calibration
    "calibration_point_rejected", # NEW: per-point if auto-detect failed
    "calibration_validation",    # NEW: multi-depth validation result
    "heartbeat_status",          # cube_alive, output_enabled, temp_c, etc.
    "kinect_pose_change_detected", # if you implement extrinsic-drift detection
    "shot_pattern_emitted",       # which pattern, target, dwell, power
]
```

This makes session replay informative enough to reconstruct exactly what happened during a calibration run or an engagement.

---

## §16 — Testing (AMEND, additions)

### §16.2 — Test suite layout (AMEND, add)

Add to the structure:

```
tests/
├── unit/
│   ├── test_full_info_parser.py            # uses golden fixtures
│   ├── test_laser_packet_packing.py        # 12-bit clamp, byte order
│   ├── test_calibration_roundtrip.py       # H_norm_to_galvo @ H_galvo_to_norm = I
│   ├── test_calibration_multidepth.py      # synthetic multi-depth validation
│   ├── test_shot_patterns.py               # dot, circle, figure-8 dimensional sanity
│   ├── test_buffer_estimate_staleness.py   # is_fresh() vs age_ms()
│   ├── test_heartbeat_isolated.py          # main thread blocked, heartbeat continues
│   └── ...
├── integration/
│   ├── test_dry_fire_pipeline.py
│   ├── test_lasercube_dry_run_transport.py # LaserCubeTransport ABC w/ no-photon impl
│   ├── test_session_replay.py
│   └── test_latency_budget.py              # synthetic full pipeline ≤ N ms
└── fixtures/
    ├── golden_full_info_64b.bin            # captured from probe
    ├── golden_ringbuf_empty_4b.bin
    ├── golden_alive_2b.bin
    └── golden_calibration_v2.json          # known-good calibration JSON
```

### §16.3 — Golden fixture capture procedure (NEW)

Before you write any unit tests for the parser, run the probe and dump the raw bytes to disk:

```bash
python scripts/lasercube_protocol_probe.py --src-ip 169.254.25.216 --save-raw tests/fixtures/
```

(Add `--save-raw` to the probe — currently it saves parsed results; you want the raw `data` from `socket.recvfrom()` written to disk for each command.)

Each subsequent parser-related test loads from disk:

```python
def test_parse_full_info_against_real_hardware():
    raw = (Path(__file__).parent / "fixtures/golden_full_info_64b.bin").read_bytes()
    info = LaserCubeInterface._parse_full_info(raw)
    assert info is not None
    assert info.model_name == "Wifi LaserCube 2.5W"
    assert info.fw_major == 0
    assert info.fw_minor == 23
    assert info.serial_number == "c4:5b:be:88:53:24"
    assert info.buffer_size == 6000
    assert info.dac_rate == 30000
    assert info.interlock is True
```

This regression test runs in milliseconds and catches any future parser drift instantly — without needing the cube physically connected.

---

## §17 — Bootstrap implementation order (AMEND, full replacement)

The reorder. Goals: get a fully testable bus running with no hardware required by Step 5; live cube methods don't appear in code until safety + dry-fire transport exist.

### Step 0 — Repo skeleton, dataclasses, schemas
- Create directory layout per §3.3.
- Implement `events/schemas.py` per §3.1.1 amendment.
- Implement `sensors/base.py` with `Sensor` ABC, `SensorFrame`, `SensorRole` enum.
- Implement `LaserPoint`, `LaserInfo`, `BufferEstimate` dataclasses in `laser/types.py`.

**Acceptance**: imports work, `pytest` finds the empty test tree, no behavior yet.

### Step 1 — ConfigManager + settings
- Implement `config/settings.py` with all tunables from this document.
- Implement `config/config_manager.py` for JSON overrides.
- Document runtime-vs-restart settings.

**Acceptance**: `ConfigManager().get("LASERCUBE_DEFAULT_IP")` returns the value, with env/JSON override pathways tested.

### Step 2 — LaserCubeProtocol parser utilities + golden fixtures
- Port the probe parsers (`parse_full_info`, `parse_alive`, `parse_ringbuf_empty`) into `laser/protocol.py` as pure functions taking bytes and returning dataclasses.
- Capture golden fixtures from your real cube via the probe.
- Write `tests/unit/test_full_info_parser.py` etc. against the golden fixtures.

**Acceptance**: parser tests pass with cube physically disconnected. Drift in the byte layout breaks the test immediately.

### Step 3 — LaserCubeTransport ABC + DryRunTransport
- Define `LaserCubeTransport(ABC)` with `connect`, `disconnect`, `send_chunk`, `enable_output`, `disable_output`, `get_full_info`, `get_ringbuf_empty`.
- Implement `DryRunTransport` that logs every call and never touches the network. Returns canned successful responses.
- Implement `_parse_full_info` on the dry-run by reading the golden fixture.

**Acceptance**: `DryRunTransport().connect(); transport.send_chunk([...])` runs without hardware, logs to session JSONL, no UDP packets emitted.

### Step 4 — Heartbeat thread
- Implement `laser/heartbeat.py` per §9.10.
- Wire it up to a transport (works against `DryRunTransport`).
- Test: deliberately block the main thread for 10 seconds, verify heartbeat ticks continued via session log.

**Acceptance**: `test_heartbeat_continues_during_main_block` passes against `DryRunTransport`.

### Step 5 — Real LaserCubeInterface (live transport)
- Implement `LaserCubeInterface` per §9.5 amendment with strict validation, `BufferEstimate`, best-effort disconnect.
- Make it a `LaserCubeTransport` subclass.
- Implement source-IP auto-detection per §13.5 (existing in v0.1, keep).
- **DO NOT call `enable_output()` yet from any production path.** It exists, but only the test fixture `test_set_output_cold.py` invokes it — that test is the §9.12 SHA204 pre-flight.

**Acceptance**: `LaserCubeInterface(...).connect()` returns True against your cube; `get_full_info()` matches the golden fixture; heartbeat survives a 30s session.

### Step 5.5 — SHA204 pre-flight (§9.12)
**This is a gating step.** If `SET_OUTPUT(0x01)` does not produce visible laser output, halt and reverse the handshake before continuing. If it does, proceed.

### Step 6 — Calibration patterns + dry-fire visualization
- Implement `targeting/patterns.py` (grid, halton, dragline, windmill).
- Implement `stream_dragline()` per §8.7.
- Implement `CoordinateMapper` storing both `H_norm_to_galvo` and `H_galvo_to_norm` per §8.5.
- Calibration controller drives `LaserCubeTransport.send_chunk()`. Against `DryRunTransport`, you can visualize the path on a synthetic camera image.

**Acceptance**: dry-run calibration produces a calibration JSON v2 file with both matrices and a plausible residual.

### Step 7 — Detector + classifier port
- Port classifier per §6 amendment, with Hu features behind a flag.
- Detector with the `label`/`conf` default fix.
- Unit tests for feature extraction, heuristic classify, fail-mode.

**Acceptance**: detector runs on saved test video; classifier toggleable.

### Step 8 — Tracker
- Multi-modal tracker with `fire_eligible` computed per-track per §7 amendment.
- Hungarian + greedy fallback.

**Acceptance**: existing tracking modes still work, `fire_eligible` updates correctly under coast/confirm transitions.

### Step 9 — Sensor implementations
- GoPro Hero 13 with `SensorRole.TARGETING`.
- Kinect v2 with `SensorRole.SAFETY`. RGB+depth+IR streams; world-coord mapping via Kinect SDK.
- Local cam fallback.

**Acceptance**: each sensor produces valid `SensorFrame` with `width/height/timestamp_uncertainty_ms` populated.

### Step 10 — ShotPattern + targeting layer
- Implement `laser/shot_patterns.py` per §9.11.
- LaserManager pulls a target from the bus, applies the configured ShotPattern, sends via transport.
- Lead-aim using `LATENCY_SOFTWARE_LAG_MS` from §8.8.

**Acceptance**: against `DryRunTransport`, a target generates a logged ShotPattern with correct sample count and centered geometry.

### Step 11 — Live calibration + multi-depth validation
- First time the cube actually emits laser light (low power, safety lens, eye protection).
- Run calibration, then validate at 3 depths per §8.6.
- Persist calibration v2 JSON.

**Acceptance**: calibration validation passes at all three depths within tolerance.

### Step 12 — Live targeting (the actual scary step)
- Wire detector → tracker → targeting → safety → live transport.
- Test against still targets on a wall first; then slow-moving dummies; then mosquitoes if they show up.
- Log every shot to session recorder.

**Acceptance**: against a still small target at 2m range, hit rate ≥ some-threshold-you-pick.

### Step 13 — Web monitor, GUI controls, CLI completion, session recording, audio stub, polish
(All of v0.1's existing later steps, in roughly the same order.)

---

## §18 — Open questions (AMEND, additions)

Add to the existing list:

6. **Does FW 0.23 require SHA204 before SET_OUTPUT?** Test cold per §9.12. **Highest-priority open question.**
7. **What is the actual reprojection error of the GoPro homography at 1m, 2.5m, 4m?** Multi-depth validation (§8.6) answers this empirically. Threshold: 1.5% normalized.
8. **What is the galvo dynamic constant for our cube?** Drag-line at multiple speeds (§8.8) decomposes it. Used for diagnosis, not aim correction.
9. **What ShotPattern produces the highest hit rate?** Empirical question. Run dot_repeat vs micro_circle vs figure_eight against still and moving targets; log to session; compare. Likely depends on dwell and target speed.
10. **How long does the laser need to dwell to mark a mosquito wing at 2m / 4m / 6m?** Don't know without tests. Set expectations honestly: V1 may be "tag with green dot," not "vaporize."
11. **What is the practical maximum useful operating depth?** Bounded by GoPro's ability to resolve a sub-arcminute target plus calibration accuracy at depth.

---

## Appendix Q — Code-level bug punch list (one-pass fixes)

Apply these as a single PR before any architectural changes:

| File | Fix | LOC |
|---|---|---|
| `sensors/base.py` | `Sensor.normalize()` takes `frame` arg, uses `frame.width/height` | ~5 |
| `sensors/kinect_v2.py` | `read()` sets `frame.width/height` from primary stream | ~6 |
| `detection/detector.py` | Set `label="candidate", conf=0.5` defaults before classifier branch | ~3 |
| `laser/lasercube.py` | `_send_cmd_recv()` filters by source IP/port + echo + status | ~25 |
| `laser/lasercube.py` | `disconnect()` attempts SET_OUTPUT(0) on partial state | ~5 |
| `laser/lasercube.py` | Replace `_buffer_free` with `BufferEstimate(free, size, last_refresh_ts)` | ~15 |
| `scripts/lasercube_protocol_probe.py` | Add `--save-raw <dir>` flag for golden fixture capture | ~10 |
| `scripts/lasercube_protocol_probe.py` | Filter `recvfrom` by expected source IP/port | ~10 |

Total: roughly 80 LOC of fixes, an afternoon of focused work.

---

## End

Apply this delta into BOOTSTRAP.md when you're ready, or carry it as a v0.2 reference until the rewrite is in flight. Highest-leverage next moves, in order:

1. **Run the SHA204 cold test (§9.12).** Confirms timeline before any more engineering.
2. **Capture golden fixtures (§16.3).** 5 minutes of work, unlocks parser regression tests forever.
3. **Build DryRunTransport + heartbeat (Steps 3-4).** ~150 LOC, lets you build the whole bus offline.
4. **Apply Appendix Q punch list.** ~80 LOC, removes a dozen latent bugs.
5. **Set up multi-depth calibration validation (§8.6).** This is the one that decides whether your homography model survives contact with reality, or whether you need to escalate to a depth-aware mapper.

Build it like the protocol probe: verify, document, then ship.
