# NoMoSkeeters — Final Rewrite Bootstrap

**A complete reference for the rewrite, consolidating every architectural decision, verified protocol value, and implementation pattern from the design phase.**

> Status: Ready for implementation. All protocol values verified against hardware.
> Hardware: WiFi LaserCube 2.5W, FW 0.23, serial `c4:5b:be:88:53:24`
> Author: Cole + Claude (design phase, completed prior to rewrite)
> Last verified: protocol probe successful via direct ethernet APIPA on 2026-05-09

---

## Table of Contents

0. [How to use this document](#0-how-to-use-this-document)
1. [Mission and scope](#1-mission-and-scope)
2. [Glossary](#2-glossary)
3. [Architecture](#3-architecture)
4. [Coordinate systems contract](#4-coordinate-systems-contract)
5. [Sensor layer](#5-sensor-layer)
6. [Detection layer](#6-detection-layer)
7. [Tracking layer](#7-tracking-layer)
8. [Calibration](#8-calibration)
9. [LaserCube protocol — verified reference](#9-lasercube-protocol--verified-reference)
10. [Safety system](#10-safety-system)
11. [GUI (PySide6)](#11-gui-pyside6)
12. [Web monitoring server](#12-web-monitoring-server)
13. [Configuration](#13-configuration)
14. [Logging and diagnostics](#14-logging-and-diagnostics)
15. [Entry point and CLI](#15-entry-point-and-cli)
16. [Testing and verification](#16-testing-and-verification)
17. [Bootstrap implementation order](#17-bootstrap-implementation-order)
18. [Open questions and future work](#18-open-questions-and-future-work)
19. [Appendix A — Verified hardware specs](#19-appendix-a--verified-hardware-specs)
20. [Appendix B — Network setup specifics](#20-appendix-b--network-setup-specifics)
21. [Appendix C — Useful diagnostic commands](#21-appendix-c--useful-diagnostic-commands)

---

## 0. How to use this document

This is the canonical reference for the NoMoSkeeters rewrite. It is intended to be:

- **Quoted from**, not paraphrased. When section 9 says `LASERCUBE_BUFFER_SIZE = 6000`, that's the hardware value, not a guess.
- **Read in order on first pass**, then jumped into by section as a reference during implementation.
- **Updated in place** as decisions evolve. If a value here is wrong, fix it here and reference the new value going forward.

Sections 3, 4, and 9 are the load-bearing structural parts. Sections 5–14 are domain implementations that hang off them. Section 17 is the actual implementation plan.

When implementing, lean on the **principle of least narrative drift**: copy code patterns from this document directly into the codebase rather than rewriting from memory. Memory is where the original repo's fictional protocol came from.

---

## 1. Mission and scope

**Mission**: Detect mosquitoes in the room using vision sensors, track them in 3D, and neutralize them with a laser beam. Operator-supervised, safety-gated, runs as a desktop application.

**In scope for the rewrite**:
- Multi-sensor input: GoPro Hero 13 Black (RGB), Kinect v2 (RGB + depth + IR + audio), local webcam fallback.
- Detection: background subtraction, optional YOLO, classifier-gated.
- Tracking: Kalman + Hungarian assignment, multiple selectable modes.
- Targeting: homography-based mapping from sensor space to LaserCube galvo space.
- Calibration: configurable patterns (grid, drag-line, twisting windmill), automatic + manual fallback.
- LaserCube control: full protocol implementation matching libLaserdockCore.
- Safety: arm/disarm, latched e-stop, no-fire zones, dwell limit, idle blank, range-aware object-size guard.
- Operator GUI: live feed, overlays, calibration wizard, control panel.
- Web monitoring: browser dashboard with MJPEG, status API, event log.
- Practice/dry-fire mode and forensic session recording.

**Out of scope (for this rewrite, but planned)**:
- Audio-based wingbeat detection (Kinect mic array stub only).
- ML-based species classification.
- Multi-laser coordination.
- Outdoor operation (Kinect IR doesn't survive sunlight).
- Cube link networking between multiple cubes.

**Hard non-goals**:
- Replacing the safety architecture — the current `SafetySystem` is correct in shape and stays.
- Cross-platform on day one — Windows-first, Linux later via Kinect libfreenect2 path.
- USB LaserCube support — network only.

---

## 2. Glossary

| Term | Meaning |
|---|---|
| **APIPA** | Automatic Private IP Addressing. 169.254.x.x range Windows assigns when no DHCP. The cube uses this when direct-attached without a router. |
| **Galvo coords** | The LaserCube's native coordinate space. 12-bit unsigned per axis, range 0x000–0xFFF (0–4095). |
| **DAC rate** | Points per second the laser projector outputs. Scan rate. Ceiling for the 2.5W cube is 30,000 pps. |
| **PPS** | Points per second — synonymous with DAC rate / scan rate. |
| **ILDA** | International Laser Display Association. Standard interface for laser projectors. |
| **MOG2** | Mixture-of-Gaussians background subtraction algorithm in OpenCV. |
| **Hungarian algorithm** | Optimal bipartite assignment algorithm. Used to match detections to existing tracks. |
| **APIPA cube address** | The cube self-assigns 169.254.40.83 on direct-attached ethernet. PC self-assigns its own 169.254.x.x. |
| **Multi-NIC routing problem** | Windows picking the wrong network adapter to send UDP out, when multiple adapters have routes to the destination. The reason our first probe attempts failed. |
| **Source IP binding** | Forcing a UDP socket to use a specific local IP as source, overriding Windows' adapter selection. |
| **Bus pattern** | The architecture: subsystems communicate by emitting events (Qt signals), not by calling each other directly. |
| **uWu mode** | A user preference, not a software mode. |

---

## 3. Architecture

### 3.1 The bus pattern

The current repo's modular separation is correct. The rewrite preserves it. Each subsystem is a thread-isolated worker that emits **events on Qt signals**; downstream consumers connect to those signals. Subsystems do not call each other directly.

```
                ┌─────────────────────────────────────────────┐
                │              Sensor Layer                    │
                │  ┌─────────┐  ┌──────────┐  ┌────────────┐ │
                │  │ GoPro   │  │ Kinect v2│  │ LocalCam   │ │
                │  │ Sensor  │  │ Sensor   │  │ Sensor     │ │
                │  └────┬────┘  └────┬─────┘  └─────┬──────┘ │
                │       │ frame_ready signals       │         │
                └───────┴──────────────┴────────────┴─────────┘
                                   │
                                   ▼
                ┌─────────────────────────────────────────────┐
                │            Detection Layer                   │
                │  • Background subtraction (MOG2)            │
                │  • Optional YOLO                            │
                │  • Classifier (10-feature shape filter)     │
                │       │ detection_ready(Detection)          │
                └───────┴─────────────────────────────────────┘
                                   │
                                   ▼
                ┌─────────────────────────────────────────────┐
                │             Tracking Layer                   │
                │  • Kalman filter per track                  │
                │  • Hungarian assignment (scipy fallback)    │
                │  • Multi-modal: dt-based, confidence,       │
                │    coasting, history                        │
                │       │ track_updated(TrackedTarget)        │
                └───────┴─────────────────────────────────────┘
                                   │
                                   ▼
                ┌─────────────────────────────────────────────┐
                │            Targeting Layer                   │
                │  • CoordinateMapper (sensor → galvo space)  │
                │  • Predictive lead-aim                      │
                │       │ target_command(LaserCmd)            │
                └───────┴─────────────────────────────────────┘
                                   │
                                   ▼
                ┌─────────────────────────────────────────────┐
                │            Safety Gate                       │
                │  • Arm/disarm state                         │
                │  • Latched e-stop                           │
                │  • No-fire zones (2D + 3D)                  │
                │  • Range-aware object-size guard            │
                │  • Dwell limit                              │
                │       │ authorized_command(LaserCmd)        │
                └───────┴─────────────────────────────────────┘
                                   │
                                   ▼
                ┌─────────────────────────────────────────────┐
                │           LaserCube Interface                │
                │  • UDP protocol (libLaserdockCore)          │
                │  • Backpressure tracking                    │
                │  • Periodic GET_FULL_INFO heartbeat         │
                └─────────────────────────────────────────────┘
```

Side-channel observers on the bus:

- **GUI** subscribes to all signals to render overlays, stats, state.
- **Web monitor** subscribes to all signals to push browser updates.
- **Session recorder** subscribes to all signals to write JSONL forensic logs.

### 3.2 Threading model

Each subsystem owns its work on a dedicated `QThread`. Signals cross thread boundaries via Qt's queued connection (default for cross-thread). No subsystem ever calls into another's internal state directly.

```
┌─────────────────────────────────────────────────────────────┐
│  Qt Main Thread (GUI event loop, signal dispatch)           │
└──────┬───────────┬───────────┬───────────┬───────────┬──────┘
       │           │           │           │           │
   ┌───▼───┐   ┌───▼────┐  ┌───▼────┐  ┌──▼─────┐  ┌──▼─────┐
   │Sensor │   │Detector│  │Tracker │  │Targeter│  │ Laser  │
   │Thread │   │Thread  │  │Thread  │  │Thread  │  │Thread  │
   └───────┘   └────────┘  └────────┘  └────────┘  └────────┘
   
                  (Web monitor + Session recorder run in their
                   own threads as additional subscribers)
```

Why this matters: video capture, classifier inference, and laser streaming all have independent timing constraints. Pinning them to the GUI thread would make the GUI stutter and frame-drop sensors. With per-subsystem threads, the GUI stays smooth even under load.

### 3.3 Module map (filesystem layout)

```
NoMoSkeeters/
├── main.py                    # Minimal entry point. Parses CLI, starts app.
├── config/
│   ├── settings.py            # All tunables. Source of truth.
│   ├── config_manager.py      # JSON user-overrides on disk.
│   └── coordinate_systems.md  # Coordinate contract (see section 4).
├── sensors/                   # ← NEW (was camera/)
│   ├── base.py                # Sensor ABC + SensorFrame dataclass.
│   ├── gopro.py               # GoPro Hero 13 implementation.
│   ├── kinect_v2.py           # Kinect v2 implementation.
│   ├── local_cam.py           # Local webcam fallback.
│   ├── sensor_manager.py      # Multiplexes active sensors.
│   └── gopro_interface.py     # Low-level GoPro Open API client.
├── detection/
│   ├── detector.py            # BG subtraction + optional YOLO.
│   ├── classifier.py          # 10-feature shape classifier (NEW, ported).
│   └── training_data.py       # Sample collection + RF model training.
├── tracking/                  # ← split from detection/
│   ├── tracker.py             # Multi-modal Kalman tracker.
│   ├── kalman_track.py        # Per-target Kalman filter.
│   └── assignment.py          # Hungarian + greedy fallback.
├── targeting/
│   ├── coordinate_mapper.py   # Homography pixel→galvo.
│   ├── calibration.py         # Calibration controller.
│   └── patterns.py            # Grid/dragline/windmill pattern generators.
├── laser/
│   ├── lasercube.py           # COMPLETE REWRITE. UDP protocol.
│   ├── laser_manager.py       # Targeting queue, dwell limit, safety wiring.
│   └── frames.py              # Test patterns (calibration, idle, spiral).
├── utils/
│   ├── safety.py              # SafetySystem (arm/disarm/zones/etc).
│   ├── logging_utils.py       # Rotating file logger.
│   └── session_recorder.py    # JSONL forensic event log (NEW).
├── monitoring/                # ← NEW
│   ├── web_server.py          # Stdlib HTTP + MJPEG server.
│   └── dashboard.html         # Inline dark-themed dashboard.
├── gui/
│   ├── main_window.py         # MainWindow, wires everything.
│   ├── control_panel.py       # All operator controls.
│   ├── camera_view.py         # Live overlays.
│   ├── calibration_dialog.py  # Calibration wizard.
│   └── settings_dialog.py     # NEW. Tunables editor.
├── scripts/
│   └── lasercube_protocol_probe.py  # Read-only diagnostic.
├── docs/
│   ├── BOOTSTRAP.md           # This document.
│   ├── PROTOCOL.md            # LaserCube protocol quick-ref.
│   ├── COORDINATES.md         # Coordinate contract.
│   └── MODULE_DOCS.md         # Existing per-module docs.
├── user_data/                 # Generated at runtime.
│   ├── config.json
│   ├── calibrations/
│   │   ├── gopro_<scene>.json
│   │   └── kinect_<scene>.json
│   └── sessions/
│       └── 2026-MM-DD-HHMMSS.jsonl
├── models/                    # Optional ML.
│   └── classifier.pkl         # Trained RandomForest if collected.
├── logs/
│   └── iron_dome.log          # Rotating.
└── requirements.txt
```

---

## 4. Coordinate systems contract

The system has **six** distinct coordinate systems. Most bugs in multi-sensor laser systems come from confusing them. This is the canonical contract.

### 4.1 Defined coordinate systems

| ID | Space | Range | Units | Where it lives |
|---|---|---|---|---|
| **GP-PX** | GoPro pixel | x: 0..W-1, y: 0..H-1 | pixels | Frames out of `GoProSensor.read()` |
| **K-RGB-PX** | Kinect RGB pixel | x: 0..1919, y: 0..1079 | pixels | Frames out of `KinectV2Sensor.read()` (RGB stream) |
| **K-DEPTH-PX** | Kinect depth pixel | x: 0..511, y: 0..423 | pixels | Frames out of `KinectV2Sensor.read()` (depth stream) |
| **K-WORLD** | Kinect 3D world | X: ~-2..+2, Y: ~-1.5..+1.5, Z: 0.5..4.5 | meters | Camera-relative. X right, Y up, Z forward. |
| **NORM** | Sensor-normalized | x, y: 0.0..1.0 | unitless | Internal canonical. All sensors normalize to this on output. |
| **GALVO** | LaserCube native | x, y: 0..0xFFF (0..4095) | unitless 12-bit | Final stage before UDP. |

### 4.2 Conversion flow

```
                  ┌─── per-sensor calibration (homography) ───┐
                  │                                            │
   GP-PX  ──────► NORM ────┐                                  │
                           │                                  ▼
   K-RGB-PX ─────► NORM ───┼──► CoordinateMapper.fwd(NORM) ──► GALVO
                           │                                  ▲
   K-DEPTH-PX ──► NORM ────┘                                  │
              └──► K-WORLD ───► (used for 3D no-fire and      │
                                  range-aware size guard)     │
                                                              │
                                       optional pre-mapping   │
                                       lead-aim adjustment ───┘
```

### 4.3 Conversion rules

- **Pixel → Norm**: divide by sensor dimensions. `x_norm = x_px / (W - 1)`.
- **Norm → Pixel**: multiply by sensor dimensions. Used for drawing overlays.
- **Norm → Galvo**: homography matrix learned per (sensor, scene) during calibration. Stored in JSON.
- **Pixel → World** (Kinect only): use the Kinect SDK's `CoordinateMapper` (different than ours, same name) to project pixel + depth into the camera-relative 3D frame.
- **World → Norm** is NOT a defined direct conversion. World coords are for safety checks and scene reasoning; they don't directly drive the laser. The laser is driven by NORM coords from a calibrated sensor.

### 4.4 Naming conventions

In code, every coordinate-bearing variable carries its space as a suffix:

```python
target_x_norm: float    # ✓ obvious
target_x_px: int        # ✓ obvious
target_x_galvo: int     # ✓ obvious
target_x: float         # ✗ AMBIGUOUS. Reject in code review.
```

Detections carry both pixel and normalized coords (the pixel coords for overlay drawing, normalized for downstream consumption).

### 4.5 Why this matters

The current repo's `targeting/coordinate_mapper.py` works in pixel space. The `claude_upgrades` work in normalized space. Without this contract, any port of one into the other introduces silent failures where, e.g., a tracker passes pixel coords into a function expecting normalized — and the laser fires at coordinates 1000× larger than intended. This document is the contract that prevents that.

---

## 5. Sensor layer

### 5.1 The Sensor abstract base class

```python
# sensors/base.py
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Tuple
import numpy as np
import time


@dataclass
class SensorFrame:
    """One frame from a sensor. Always carries timestamp."""
    timestamp: float                    # seconds (monotonic), set at capture
    sensor_id: str                      # e.g. "gopro", "kinect_rgb", "kinect_ir"
    rgb: Optional[np.ndarray] = None    # HxWx3 BGR uint8
    depth: Optional[np.ndarray] = None  # HxW float32, meters
    ir: Optional[np.ndarray] = None     # HxW uint16
    width: int = 0
    height: int = 0
    
    @classmethod
    def now(cls, sensor_id: str) -> "SensorFrame":
        return cls(timestamp=time.monotonic(), sensor_id=sensor_id)


class Sensor(ABC):
    """Abstract base for all sensors. Implementations live in sensors/<name>.py"""
    
    @property
    @abstractmethod
    def sensor_id(self) -> str:
        """Stable identifier used in logs, calibration filenames, event streams."""
    
    @property
    @abstractmethod
    def has_rgb(self) -> bool: ...
    
    @property
    @abstractmethod
    def has_depth(self) -> bool: ...
    
    @property
    @abstractmethod
    def has_ir(self) -> bool: ...
    
    @abstractmethod
    def open(self) -> bool:
        """Connect/initialize the sensor. Return True on success."""
    
    @abstractmethod
    def close(self) -> None:
        """Disconnect. Idempotent."""
    
    @abstractmethod
    def read(self) -> Optional[SensorFrame]:
        """Non-blocking. Return next frame or None if not ready."""
    
    def normalize(self, x_px: float, y_px: float) -> Tuple[float, float]:
        """Default: scale by frame dims. Override for non-rectangular sensors."""
        return (x_px / max(1, self.width - 1), y_px / max(1, self.height - 1))
    
    def world_position(
        self, x_px: int, y_px: int, depth_m: float
    ) -> Optional[Tuple[float, float, float]]:
        """
        For depth-capable sensors: convert (pixel, depth) to (X, Y, Z) meters
        in sensor-relative camera space. Default implementation returns None.
        """
        return None
```

### 5.2 GoPro Hero 13 implementation

The Hero 13 Black exposes a TCP HTTP API (the **Open GoPro API**) for control plus a UDP video preview stream. The cube is at `10.5.5.9` when in WiFi AP mode (Hero is the AP); other modes vary.

Key facts:
- Live preview stream is capped at 30 fps (sometimes 60 in specific modes). High-fps modes (240 fps) are *capture-only*, not streaming.
- Keep-alive ping required every ~2.5 seconds or the stream dies.
- Stream URL: `udp://0.0.0.0:8554` after starting preview via HTTP API.

Use `cv2.VideoCapture` with the UDP URL. On Windows you'll want `CAP_FFMPEG` backend explicitly to get reliable UDP decoding:

```python
self.cap = cv2.VideoCapture("udp://0.0.0.0:8554", cv2.CAP_FFMPEG)
```

For the live targeting pipeline, accept 30 fps as the working frame rate. For "forensic high-fps capture" of a confirmed target, trigger an internal SD-card recording via the API in a high-fps mode — those frames don't reach the PC live but are downloadable later for review and classifier training.

**TODO**: Audio Hero13 supports stereo audio via the API. Could be a wingbeat detection input alongside Kinect mic.

### 5.3 Kinect v2 implementation

Microsoft Kinect SDK 2.0 + `pykinect2` Python binding. Already verified working in earlier conversation.

Streams emitted (one frame each per cycle; merged into one `SensorFrame` per read):
- **RGB**: 1920×1080 BGRA → convert to BGR.
- **Depth**: 512×424 uint16 in millimeters → convert to float32 meters.
- **IR**: 512×424 uint16, full dynamic range.
- **Body tracking** (optional): up to 6 skeletons, 25 joints each.
- **Audio**: 4-element mic array, 16 kHz mono mixed-down or per-element raw.

Key implementation details:
- The SDK only allows ONE Kinect per process. Singleton sensor.
- USB 3.0 controller compatibility is real; document the working controllers in operator notes.
- The depth-to-RGB pixel mapping requires the SDK's `CoordinateMapper.MapDepthFrameToColorSpace()`. Without that, depth pixels and RGB pixels are not aligned.
- IR + depth streams give us "darkness operation" — Kinect's 850nm IR floods the room and depth works without visible-light illumination.

```python
# sensors/kinect_v2.py — sketch
from pykinect2 import PyKinectV2, PyKinectRuntime

class KinectV2Sensor(Sensor):
    sensor_id_str = "kinect_v2"
    
    def __init__(self):
        self._kinect = None
    
    def open(self) -> bool:
        sources = (
            PyKinectV2.FrameSourceTypes_Color
            | PyKinectV2.FrameSourceTypes_Depth
            | PyKinectV2.FrameSourceTypes_Infrared
            | PyKinectV2.FrameSourceTypes_Body
        )
        self._kinect = PyKinectRuntime.PyKinectRuntime(sources)
        return self._kinect is not None
    
    def read(self) -> Optional[SensorFrame]:
        frame = SensorFrame.now(self.sensor_id_str)
        # Color: 1920×1080 BGRA
        if self._kinect.has_new_color_frame():
            color = self._kinect.get_last_color_frame().reshape((1080, 1920, 4))
            frame.rgb = cv2.cvtColor(color, cv2.COLOR_BGRA2BGR)
        # Depth: 512×424 uint16 mm
        if self._kinect.has_new_depth_frame():
            depth_mm = self._kinect.get_last_depth_frame().reshape((424, 512))
            frame.depth = depth_mm.astype(np.float32) * 0.001
        # IR
        if self._kinect.has_new_infrared_frame():
            frame.ir = self._kinect.get_last_infrared_frame().reshape((424, 512))
        return frame if (frame.rgb is not None or frame.depth is not None) else None
```

### 5.4 Local camera fallback

Plain `cv2.VideoCapture(index)`. Used for development, testing, and when GoPro isn't connected. Implementation is trivial. Index configurable; defaults to 0.

### 5.5 SensorManager

Multiplexes multiple active sensors. Single point where the rest of the app subscribes for frames. Each Sensor implementation runs in its own QThread; the manager forwards their `frame_ready` signals onto the bus, tagged with sensor_id.

```python
class SensorManager(QObject):
    frame_ready = Signal(SensorFrame)  # bus signal
    
    def __init__(self):
        super().__init__()
        self._sensors: dict[str, Sensor] = {}
        self._workers: dict[str, SensorWorker] = {}  # QThread per sensor
    
    def add_sensor(self, sensor: Sensor) -> bool:
        if not sensor.open():
            return False
        worker = SensorWorker(sensor)
        worker.frame_ready.connect(self.frame_ready)  # forward to bus
        worker.start()
        self._sensors[sensor.sensor_id] = sensor
        self._workers[sensor.sensor_id] = worker
        return True
```

### 5.6 Multi-sensor fusion strategy

Detections from each sensor flow as separate event streams onto the bus. The Tracker is the fusion point: it maintains tracks in normalized coordinate space, and a track gets "promoted" to 3D-aware status when a Kinect detection (with depth) joins/matches it.

Three operational modes, selectable in settings:
- **Single sensor**: only one sensor's detections drive tracks. Simplest.
- **Parallel**: each sensor maintains its own tracks. UI shows them side-by-side. Useful for debugging.
- **Fused**: detections from multiple sensors merge into shared tracks via spatial proximity. Kinect provides depth dimension when available.

---

## 6. Detection layer

### 6.1 Background-subtraction pipeline

Existing pipeline is correct in shape. Refactor to use config tunables instead of magic numbers.

```python
# detection/detector.py — pipeline outline
def detect_bgsub(self, frame: np.ndarray) -> list[Detection]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if settings.DETECTION_BLUR_KERNEL > 1:
        gray = cv2.GaussianBlur(gray,
                                 (settings.DETECTION_BLUR_KERNEL,) * 2, 0)
    fg_mask = self._mog2.apply(gray)
    _, thresh = cv2.threshold(fg_mask, settings.DETECTION_THRESHOLD,
                               255, cv2.THRESH_BINARY)
    # Morph cleanup
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                   (settings.DETECTION_MORPH_KERNEL,) * 2)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, k)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, k)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    for c in contours:
        # Geometric pre-filter (cheap)
        area = cv2.contourArea(c)
        if not (settings.DETECTION_MIN_AREA <= area <= settings.DETECTION_MAX_AREA):
            continue
        # Classifier post-filter (expensive, optional)
        if self._classifier is not None:
            label, conf = self._classifier.classify(frame, c)
            if label != "mosquito":
                continue
        detections.append(self._build_detection(frame, c, label, conf))
    return detections
```

### 6.2 The classifier

Ported from `claude_upgrades/classifier.py`. Lives at `detection/classifier.py`.

10-feature shape vector per contour:
1. `aspect_ratio` — `w / h` of bounding rect
2. `solidity` — `area / convex_hull_area` (compactness)
3. `extent` — `area / bounding_rect_area` (fill ratio)
4. `equiv_diameter` — `sqrt(4 * area / π)`
5. `perimeter_squared`
6. `area_frame_ratio` — `area / (frame_w * frame_h)`
7-10. First 4 log-scaled Hu moments (rotation-invariant shape descriptors)

Two operating modes:
- **Heuristic** (always available): hand-tuned thresholds in config. Default if no model.
- **ML** (optional, requires sklearn): loads `models/classifier.pkl` (RandomForest) trained on labeled samples.

```python
# detection/classifier.py — public API
class DetectionClassifier:
    def __init__(self, model_path: Optional[str] = None,
                 heuristic_thresholds: Optional[dict] = None):
        self._model = self._try_load_model(model_path)
        self._thresholds = heuristic_thresholds or settings.CLASSIFIER_HEURISTIC_DEFAULTS
    
    def classify(self, frame: np.ndarray, contour: np.ndarray) -> tuple[str, float]:
        """Returns (label, confidence). Never raises in the hot path."""
        try:
            features = self._extract_features(frame, contour)
            if self._model is not None:
                return self._classify_ml(features)
            return self._classify_heuristic(features, cv2.contourArea(contour))
        except Exception:
            # Fail-open behavior is configurable
            if settings.CLASSIFIER_FAIL_MODE == "open":
                return ("mosquito", 0.5)
            return ("false_positive", 0.5)
```

### 6.3 Configurable everything

All thresholds in `config/settings.py`. None hardcoded:

```python
# Pre-classifier geometric filters
DETECTION_MIN_AREA: int = 5
DETECTION_MAX_AREA: int = 800
DETECTION_MIN_ASPECT: float = 0.15
DETECTION_MAX_ASPECT: float = 8.0
# ...

# Classifier heuristic thresholds (from claude_upgrades, configurable)
CLASSIFIER_HEURISTIC_DEFAULTS = {
    "min_solidity": 0.30,
    "min_aspect": 0.20,
    "max_aspect": 5.00,
    "min_area_px": 20,
    "max_area_px": 5000,
}
CLASSIFIER_MODEL_PATH: str = "models/classifier.pkl"
CLASSIFIER_FAIL_MODE: str = "open"   # "open" | "closed" | "neutral"
CLASSIFIER_ENABLED: bool = True       # GUI toggleable
```

### 6.4 Training pipeline

`detection/training_data.py` runs offline, not during operation. Collects labeled samples (operator clicks "real mosquito" / "false positive" on detection events recorded during sessions) and trains a RandomForest.

Output: `models/classifier.pkl`. Loaded automatically on next run if present.

---

## 7. Tracking layer

### 7.1 Multi-modal design

Multiple tracking modes selectable in settings, all sharing the bus protocol. Switching is runtime-toggleable in the GUI.

| Mode | Description | When to use |
|---|---|---|
| `kalman_pixel` | Existing pixel-space Kalman, fixed dt. | Stable conditions, single sensor. |
| `kalman_norm_dt` | Normalized space, real dt from frame timestamps. | Default. Variable frame rate. |
| `kalman_3d` | Adds Z axis when Kinect depth available. | Fused mode. |
| `iou_only` | No Kalman, simple IoU matching frame to frame. | Diagnostic, very fast targets. |

### 7.2 Per-track state

```python
# tracking/kalman_track.py
@dataclass
class TrackedTarget:
    track_id: int
    sensor_id: str                   # which sensor this track originated from
    state: np.ndarray                # [x, y, vx, vy] or [x,y,z,vx,vy,vz] in 3D mode
    covariance: np.ndarray
    
    # Lifecycle
    age_frames: int = 0              # total frames since track creation
    confirmed: bool = False           # promoted after N confirmed matches
    disappeared_frames: int = 0       # consecutive frames without a match
    coast_frames: int = 0             # frames spent coasting (predicting w/o update)
    
    # Confidence
    confidence: float = 0.5           # rises on match, decays while coasting
    
    # History (for trajectory visualization)
    history: deque = field(default_factory=lambda: deque(maxlen=64))
    
    # Last detection (for safety reasoning)
    last_detection: Optional[Detection] = None
    last_update_ts: float = 0.0
```

### 7.3 Tunables

```python
# config/settings.py — tracking section
TRACKER_MODE: str = "kalman_norm_dt"       # see modes above
TRACKER_MAX_DISAPPEARED: int = 8           # frames before track dropped
TRACKER_MAX_COAST_FRAMES: int = 30          # frames of coast before drop
TRACKER_MAX_DISTANCE_NORM: float = 0.06    # max norm-distance for assignment match
TRACKER_KALMAN_PROC_NOISE: float = 0.1
TRACKER_KALMAN_MEAS_NOISE: float = 2.0
TRACKER_CONFIRMATION_FRAMES: int = 3        # frames before track is "confirmed"
TRACKER_CONFIDENCE_DECAY: float = 0.9       # multiplied per coast frame
TRACKER_CONFIDENCE_BOOST: float = 1.05      # multiplied per match
TRACKER_HISTORY_LENGTH: int = 64            # for trajectory drawing
```

### 7.4 Assignment

Hungarian algorithm via `scipy.optimize.linear_sum_assignment` if available, greedy nearest-neighbor fallback if not. The scipy dependency is optional but strongly recommended.

```python
def assign(detections: list[Detection], tracks: list[TrackedTarget]
          ) -> list[tuple[int, int]]:
    """Returns list of (track_idx, det_idx) matches."""
    if not detections or not tracks:
        return []
    
    # Build cost matrix (norm distance)
    cost = np.full((len(tracks), len(detections)), np.inf)
    for i, t in enumerate(tracks):
        tx, ty = t.state[0], t.state[1]
        for j, d in enumerate(detections):
            dist = np.hypot(tx - d.x_norm, ty - d.y_norm)
            if dist < settings.TRACKER_MAX_DISTANCE_NORM:
                cost[i, j] = dist
    
    if HAS_SCIPY:
        from scipy.optimize import linear_sum_assignment
        row, col = linear_sum_assignment(cost)
        return [(r, c) for r, c in zip(row, col) if cost[r, c] < np.inf]
    else:
        return _greedy_assign(cost)
```

---

## 8. Calibration

### 8.1 Why calibration matters

The CoordinateMapper learns a homography matrix that maps NORM coords to GALVO coords. Without it, the laser fires at wrong locations. Quality of calibration directly determines targeting accuracy.

For mosquitoes specifically: target size is sub-pixel at 2-6m range. Even 1° of calibration error puts the beam off target by tens of millimeters at range. **Calibration accuracy > all other concerns.**

### 8.2 Calibration patterns

Configurable, multiple supported. All draw with the laser at low power.

#### 8.2.1 Grid (default, simplest)

`N×N` grid of points across galvo space. Fire each, capture, detect, record correspondence.

```python
# targeting/patterns.py
def grid_points(rows: int, cols: int,
                margin: float = 0.2) -> list[tuple[float, float]]:
    """Returns normalized galvo coords for an N×N grid with margin."""
    return [
        (margin + (1 - 2 * margin) * c / max(1, cols - 1),
         margin + (1 - 2 * margin) * r / max(1, rows - 1))
        for r in range(rows) for c in range(cols)
    ]
```

Configurable: `CALIBRATION_GRID_ROWS`, `CALIBRATION_GRID_COLS`. Default `5×5` (25 points). Minimum sane is `3×3` (9). Max useful is `9×9` (81).

#### 8.2.2 Halton sequence (better coverage with fewer points)

Quasi-random low-discrepancy sequence. Better spatial coverage than a regular grid for the same point count.

```python
def halton_points(n: int, base_x: int = 2, base_y: int = 3,
                  margin: float = 0.2) -> list[tuple[float, float]]:
    """N Halton-distributed points in [margin, 1-margin]² of galvo space."""
    def halton(idx, base):
        f, r = 1.0, 0.0
        while idx > 0:
            f /= base
            r += f * (idx % base)
            idx //= base
        return r
    span = 1 - 2 * margin
    return [(margin + span * halton(i + 1, base_x),
             margin + span * halton(i + 1, base_y)) for i in range(n)]
```

#### 8.2.3 Drag-line (lag-aware)

Laser sweeps a slow continuous line across galvo space. Camera tracks the bright dot's trajectory frame-by-frame. Hundreds to thousands of correspondences per second.

The killer feature: by comparing the *expected* galvo position at time `t` with the *observed* camera-pixel position at time `t`, we recover both:
- The homography (galvo → pixel mapping)
- The system pipeline lag in milliseconds

This is critical because real-time targeting *must* compensate for lag (network + capture + detection + mapping + laser-stream queue).

```python
def dragline_path(start: tuple[float, float], end: tuple[float, float],
                  duration_s: float, dac_rate: int) -> list[tuple[float, float]]:
    """Returns N points along start→end such that scanning at dac_rate
    takes duration_s seconds."""
    n_points = int(duration_s * dac_rate)
    return [(start[0] + (end[0] - start[0]) * i / max(1, n_points - 1),
             start[1] + (end[1] - start[1]) * i / max(1, n_points - 1))
            for i in range(n_points)]
```

#### 8.2.4 Twisting windmill (Cole's pattern)

Rotating radial spokes from center, sweeping through angles. Looks like a windmill spinning. Hits all angular regions of galvo space and provides rotational symmetry samples that pure grids don't.

```python
def windmill_path(n_arms: int = 4, n_revolutions: float = 2.0,
                   r_inner: float = 0.05, r_outer: float = 0.45,
                   points_per_arm: int = 50,
                   center: tuple[float, float] = (0.5, 0.5)
                   ) -> list[tuple[float, float]]:
    """Twisting windmill calibration sweep."""
    cx, cy = center
    path = []
    total_angle = n_revolutions * 2 * math.pi
    arm_angles = [i * 2 * math.pi / n_arms for i in range(n_arms)]
    for arm_offset in arm_angles:
        for i in range(points_per_arm):
            t = i / max(1, points_per_arm - 1)
            angle = arm_offset + total_angle * t
            r = r_inner + (r_outer - r_inner) * t
            path.append((cx + r * math.cos(angle), cy + r * math.sin(angle)))
    return path
```

### 8.3 Detection modes

Per-point: how do we find the laser dot in the camera image?

- **Auto** (default): background subtraction + bright-channel threshold + centroid. Works in dim rooms.
- **Manual**: operator clicks on the dot in the camera view. Always available as a fallback if auto-detect fails.
- **Hybrid**: auto runs first, manual override if confidence is low.

### 8.4 Calibration profiles

Calibrations are scene-specific. If the GoPro moves 6 inches, recalibrate.

Storage: `user_data/calibrations/<sensor_id>_<profile_name>.json`.

```json
{
  "schema": 1,
  "sensor_id": "gopro",
  "profile_name": "garage_default",
  "created_ts": 1715292345.123,
  "pattern": "halton_25",
  "method": "auto",
  "homography": [[1.234, 0.012, -100.5], ...],
  "reprojection_error_px": 1.7,
  "lag_estimate_ms": 48.0,
  "correspondences": [
    {"galvo": [0.2, 0.2], "pixel": [320.5, 240.1], "confidence": 0.95},
    ...
  ]
}
```

Multiple profiles per sensor. Switch between in GUI.

### 8.5 Quality metrics in UI

After calibration:
- Mean reprojection error (in pixels and millimeters at known range).
- Per-point error highlighted on a heatmap.
- "Rerun bad points" option for points with > 2σ error.
- Estimated pipeline lag if drag-line was used.

### 8.6 Tunables

```python
# config/settings.py — calibration section
CALIBRATION_PATTERN: str = "halton"      # "grid" | "halton" | "dragline" | "windmill"
CALIBRATION_POINTS: int = 25              # used by halton; grid uses ROWS×COLS
CALIBRATION_GRID_ROWS: int = 5
CALIBRATION_GRID_COLS: int = 5
CALIBRATION_DETECTION_MODE: str = "auto"  # "auto" | "manual" | "hybrid"
CALIBRATION_DOT_THRESHOLD: int = 200
CALIBRATION_LASER_R: int = 0xFFF
CALIBRATION_LASER_G: int = 0xFFF
CALIBRATION_LASER_B: int = 0xFFF
CALIBRATION_DRAGLINE_DURATION_S: float = 2.0
CALIBRATION_WINDMILL_REVOLUTIONS: float = 2.0
CALIBRATION_WINDMILL_ARMS: int = 4
```

---

## 9. LaserCube protocol — verified reference

**This is the authoritative reference for the rewrite. All values verified against `Wickedlasers/libLaserdockCore` (manufacturer source) AND tested against physical hardware.**

The current repo's `laser/lasercube_interface.py` is **fictional from end to end** — its command bytes do not exist in any official source, and its sample format is wrong. **Discard it entirely. Write the new implementation from this document.**

### 9.1 Network layout

| Port | Role | Direction | Notes |
|---|---|---|---|
| **45456** | ALIVE | broadcast in, unicast out | Discovery only. Accepts `0x27 GET_ALIVE`, replies `[0x27, 0x00]`. |
| **45457** | CMD | bidirectional | All commands and most replies. |
| **45458** | DATA | client→cube | Sample data. Cube may also reply here with buffer status if enabled. |

**Critical binding rule:** the official client binds its command socket to `("0.0.0.0", 45457)` — using port 45457 as both source AND destination. The cube replies to source port. If you don't bind to 45457, replies arrive at a port nothing's listening on and are dropped silently.

### 9.2 Command bytes (full reference from official source)

```python
# laser/lasercube.py — protocol constants

# Read-only (safe to send anytime)
LC_CMD_GET_ALIVE                  = 0x27   # ALIVE_PORT only
LC_CMD_GET_FULL_INFO              = 0x77   # CMD_PORT
LC_CMD_GET_RINGBUF_EMPTY_COUNT    = 0x8A   # CMD_PORT

# State-modifying
LC_CMD_ENABLE_BUFFER_RESPONSE     = 0x78   # CMD_PORT, payload [enable: 0|1]
LC_CMD_SET_OUTPUT                 = 0x80   # CMD_PORT, payload [enable: 0|1] -- FIRES THE LASER
LC_CMD_SET_ILDA_RATE              = 0x82   # CMD_PORT, payload [u32 LE rate]
LC_CMD_CLEAR_RINGBUFFER           = 0x8D   # CMD_PORT
LC_CMD_SET_NV_MODEL_INFO          = 0x97   # CMD_PORT, programs NV memory (don't touch)
LC_CMD_SET_DAC_BUF_THOLD_LVL      = 0xA0   # CMD_PORT, payload [u32 LE threshold]
LC_CMD_SECURITY_REQUEST           = 0xB0   # CMD_PORT, SHA204 challenge
LC_CMD_SECURITY_RESPONSE          = 0xB1   # CMD_PORT, SHA204 response

# Sample-data packet IDs (sent on DATA_PORT, NOT CMD_PORT)
LC_DATA_SAMPLE_ID                 = 0xA9   # uncompressed, what we use
LC_DATA_SAMPLE_COMPRESSED_ID      = 0x9A   # compressed; not used by our impl
```

### 9.3 GET_FULL_INFO response — 64-byte layout

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | command echo | always `0x77` |
| 1 | 1 | result/status | `0x00` = success |
| 2 | 1 | payload version | currently `0` |
| 3 | 1 | fw_major | uint8 |
| 4 | 1 | fw_minor | uint8 |
| 5 | 1 | flags | bit 0 output_enabled, bit 1 interlock, bit 2 temp_warn, bit 3 over_temp, bits 4–7 packet_errors (FW ≥ 0.13) |
| 6–9 | 4 | reserved | |
| 10 | 4 | dac_rate | uint32 LE, points/sec |
| 14 | 4 | max_dac_rate | uint32 LE, ceiling |
| 18 | 1 | reserved | |
| 19 | 2 | rx_buffer_free | uint16 LE, samples |
| 21 | 2 | rx_buffer_size | uint16 LE, samples |
| 23 | 1 | battery_percent | uint8, `0xFF` = AC/no battery |
| 24 | 1 | temperature | int8, °C |
| 25 | 1 | connection_type | enum (see below) |
| 26 | 6 | serial_number | 6 raw bytes, format MAC-style |
| 32 | 4 | ip_address | 4 bytes |
| 36 | 1 | reserved | |
| 37 | 1 | model_number | uint8 |
| 38+ | var | model_name | null-terminated UTF-8 |

Connection types (raw byte at offset 25, +1 in the official source for the public enum):
- `0` → UNKNOWN
- `1` → USB
- `2` → WIFI_SERVER
- `3` → WIFI_CLIENT
- `4` → ETHERNET_SERVER
- `5` → ETHERNET_CLIENT

> **Note on the ETHERNET vs WIFI_CLIENT inconsistency:** Cole's hardware reports `WIFI_CLIENT` even when LaserOS displays "Ethernet Client." This is consistent with the WiFi LaserCube using one underlying "client" path internally regardless of physical link layer, OR the +1 offset being inverted. Verify on first connect; treat as cosmetic for now.

### 9.4 Sample data packet format

This is what the current repo gets WRONG. Verified format:

**Packet structure:**
```
[0xA9] [0x00] [msg_num] [frame_num] [sample_0] [sample_1] ... [sample_N-1]
  1B     1B      1B         1B           10B       10B           10B
```

**Per-sample (10 bytes):**
```
[x_lo] [x_hi] [y_lo] [y_hi] [r_lo] [r_hi] [g_lo] [g_hi] [b_lo] [b_hi]
```

Each value is **uint16 little-endian**, but the actual range is **12-bit unsigned (0x000–0xFFF, decimal 0–4095)**. The high 4 bits of each uint16 are zero/ignored.

**Sequence numbers:**
- `msg_num` increments per UDP packet sent (wraps at 0xFF)
- `frame_num` increments per logical frame (wraps at 0xFF)
- A frame split across multiple UDP packets keeps the same `frame_num`, increments `msg_num`

**Maximum samples per UDP packet: 140**
- 140 × 10 + 4 = 1404 bytes. Just under 1500 MTU.

### 9.5 Reference implementation

```python
# laser/lasercube.py — core protocol implementation

from __future__ import annotations
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Iterable

# ── Protocol constants (see section 9.2) ────────────────────────────────
ALIVE_PORT = 45456
CMD_PORT   = 45457
DATA_PORT  = 45458

LC_CMD_GET_ALIVE               = 0x27
LC_CMD_GET_FULL_INFO           = 0x77
LC_CMD_ENABLE_BUFFER_RESPONSE  = 0x78
LC_CMD_SET_OUTPUT              = 0x80
LC_CMD_SET_ILDA_RATE           = 0x82
LC_CMD_GET_RINGBUF_EMPTY       = 0x8A
LC_CMD_CLEAR_RINGBUFFER        = 0x8D
LC_DATA_SAMPLE_ID              = 0xA9

MAX_SAMPLES_PER_PACKET = 140
COORD_MIN = 0
COORD_MAX = 0xFFF


@dataclass
class LaserPoint:
    """One galvo position + RGB. Values clamped to 12-bit unsigned."""
    x: int = 0
    y: int = 0
    r: int = 0
    g: int = 0
    b: int = 0
    
    def packed(self) -> bytes:
        """Pack into the 10-byte wire format."""
        return struct.pack(
            "<HHHHH",
            max(COORD_MIN, min(COORD_MAX, self.x)),
            max(COORD_MIN, min(COORD_MAX, self.y)),
            max(COORD_MIN, min(COORD_MAX, self.r)),
            max(COORD_MIN, min(COORD_MAX, self.g)),
            max(COORD_MIN, min(COORD_MAX, self.b)),
        )


@dataclass
class LaserInfo:
    """Parsed GET_FULL_INFO response."""
    model_name: str
    model_number: int
    fw_major: int
    fw_minor: int
    output_enabled: bool
    interlock: bool
    temp_warn: bool
    over_temp: bool
    packet_errors: int
    dac_rate: int
    max_dac_rate: int
    buffer_free: int
    buffer_size: int
    battery_percent: int  # 0xFF = AC powered
    temperature_c: int
    connection_type_raw: int
    serial_number: str   # "c4:5b:be:88:53:24" format
    reported_ip: str


class LaserCubeInterface:
    """
    Network LaserCube driver. Implements the libLaserdockCore protocol
    correctly. Thread-safe.

    Usage:
        cube = LaserCubeInterface(ip="169.254.40.83", src_ip="169.254.25.216")
        cube.connect()
        info = cube.get_full_info()
        # ... build a frame ...
        cube.send_frame(points, frame_num=0)
        cube.disconnect()
    """
    
    def __init__(self,
                 ip: str,
                 src_ip: str = "0.0.0.0",
                 cmd_port: int = CMD_PORT,
                 data_port: int = DATA_PORT,
                 alive_port: int = ALIVE_PORT,
                 reply_timeout_s: float = 1.5):
        self.ip = ip
        self.src_ip = src_ip
        self.cmd_port = cmd_port
        self.data_port = data_port
        self.alive_port = alive_port
        self.reply_timeout_s = reply_timeout_s
        
        self._cmd_sock: Optional[socket.socket] = None
        self._data_sock: Optional[socket.socket] = None
        self._lock = threading.RLock()
        self._connected = False
        
        # Sequencing
        self._msg_num = 0
        
        # Backpressure tracking (mirrors libLaserdockCore approach)
        self._buffer_free = 0
        self._buffer_size = 6000   # default; updated from GET_FULL_INFO
    
    # ── Connection management ────────────────────────────────────────
    
    def connect(self) -> bool:
        """Open UDP sockets bound per the official binding pattern."""
        with self._lock:
            try:
                # CMD socket: bound to (src_ip, CMD_PORT) for both send and recv
                self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 5_250_000)
                self._cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 5_250_000)
                self._cmd_sock.bind((self.src_ip, self.cmd_port))
                self._cmd_sock.settimeout(self.reply_timeout_s)
                
                # DATA socket: bound to (src_ip, DATA_PORT)
                self._data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 5_250_000)
                self._data_sock.bind((self.src_ip, self.data_port))
                
                # Verify with a GET_FULL_INFO
                info = self._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO]),
                                            expect_min_bytes=38)
                if info is None:
                    self._close_sockets_unlocked()
                    return False
                
                self._connected = True
                # Cache buffer size from the info response
                if len(info) >= 23:
                    self._buffer_size = struct.unpack_from("<H", info, 21)[0]
                    self._buffer_free = struct.unpack_from("<H", info, 19)[0]
                return True
            except OSError:
                self._close_sockets_unlocked()
                return False
    
    def disconnect(self) -> None:
        """Disable output and close sockets."""
        with self._lock:
            if self._connected:
                # Send SET_OUTPUT(0) twice for reliability (UDP)
                self._send_cmd_no_reply(bytes([LC_CMD_SET_OUTPUT, 0x00]), repeat=2)
            self._close_sockets_unlocked()
            self._connected = False
    
    def is_connected(self) -> bool:
        return self._connected
    
    # ── Read-only commands ───────────────────────────────────────────
    
    def get_full_info(self) -> Optional[LaserInfo]:
        """Send GET_FULL_INFO, parse 64-byte reply."""
        data = self._send_cmd_recv(bytes([LC_CMD_GET_FULL_INFO]),
                                    expect_min_bytes=38)
        if data is None:
            return None
        return self._parse_full_info(data)
    
    def get_ringbuf_empty(self) -> Optional[int]:
        """Returns free sample count in ringbuffer."""
        data = self._send_cmd_recv(bytes([LC_CMD_GET_RINGBUF_EMPTY]),
                                    expect_min_bytes=4)
        if data is None or len(data) < 4:
            return None
        return struct.unpack_from("<H", data, 2)[0]
    
    # ── State-modifying commands (DANGER) ────────────────────────────
    
    def enable_output(self) -> bool:
        """Enable laser output. CALLER IS RESPONSIBLE FOR SAFETY GATE."""
        return self._send_cmd_no_reply(
            bytes([LC_CMD_SET_OUTPUT, 0x01]), repeat=2)
    
    def disable_output(self) -> bool:
        """Disable laser output. Idempotent."""
        return self._send_cmd_no_reply(
            bytes([LC_CMD_SET_OUTPUT, 0x00]), repeat=2)
    
    def set_dac_rate(self, rate: int) -> bool:
        """Set scan rate in points/second. Capped at max_dac_rate."""
        rate = max(1000, min(30_000, int(rate)))
        payload = bytes([LC_CMD_SET_ILDA_RATE]) + struct.pack("<I", rate)
        return self._send_cmd_no_reply(payload, repeat=2)
    
    def clear_ringbuffer(self) -> bool:
        """Drop all queued samples. Useful on e-stop."""
        return self._send_cmd_no_reply(bytes([LC_CMD_CLEAR_RINGBUFFER]), repeat=2)
    
    # ── Sample data streaming ────────────────────────────────────────
    
    def send_frame(self, points: list[LaserPoint], frame_num: int = 0) -> bool:
        """
        Send a frame of points, splitting across multiple UDP packets if
        needed to stay under MTU. Tracks msg_num and frame_num correctly.
        Returns True if all packets sent without OS error.
        """
        if not points:
            return True
        with self._lock:
            if self._data_sock is None:
                return False
            ok = True
            i = 0
            while i < len(points):
                chunk = points[i:i + MAX_SAMPLES_PER_PACKET]
                packet = bytes([
                    LC_DATA_SAMPLE_ID,
                    0x00,
                    self._msg_num & 0xFF,
                    frame_num & 0xFF,
                ])
                for p in chunk:
                    packet += p.packed()
                try:
                    self._data_sock.sendto(packet, (self.ip, self.data_port))
                except OSError:
                    ok = False
                    break
                self._msg_num = (self._msg_num + 1) & 0xFF
                self._buffer_free = max(0, self._buffer_free - len(chunk))
                i += MAX_SAMPLES_PER_PACKET
            return ok
    
    def buffer_free(self) -> int:
        """Last-known free buffer space. Update via get_full_info() periodically."""
        return self._buffer_free
    
    # ── Internal helpers ─────────────────────────────────────────────
    
    def _send_cmd_recv(self, payload: bytes,
                        expect_min_bytes: int = 1) -> Optional[bytes]:
        """Send a command on CMD_PORT and wait for reply."""
        with self._lock:
            if self._cmd_sock is None:
                return None
            try:
                self._cmd_sock.sendto(payload, (self.ip, self.cmd_port))
                data, _addr = self._cmd_sock.recvfrom(4096)
                if len(data) < expect_min_bytes:
                    return None
                return data
            except (socket.timeout, OSError):
                return None
    
    def _send_cmd_no_reply(self, payload: bytes, repeat: int = 1) -> bool:
        """Send a command, do not wait for reply. UDP retransmission for reliability."""
        with self._lock:
            if self._cmd_sock is None:
                return False
            try:
                for _ in range(max(1, repeat)):
                    self._cmd_sock.sendto(payload, (self.ip, self.cmd_port))
                return True
            except OSError:
                return False
    
    def _close_sockets_unlocked(self) -> None:
        for attr in ("_cmd_sock", "_data_sock"):
            sock = getattr(self, attr, None)
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
                setattr(self, attr, None)
    
    @staticmethod
    def _parse_full_info(data: bytes) -> Optional[LaserInfo]:
        """Parse 64-byte GET_FULL_INFO reply."""
        if len(data) < 38 or data[0] != LC_CMD_GET_FULL_INFO:
            return None
        buf = data if len(data) >= 64 else data + b"\x00" * (64 - len(data))
        flags = buf[5]
        serial = ":".join(f"{b:02x}" for b in buf[26:32])
        ip_bytes = buf[32:36]
        ip_str = ".".join(str(b) for b in ip_bytes)
        name = data[38:].split(b"\x00", 1)[0].decode("utf-8", errors="replace")
        return LaserInfo(
            model_name=name,
            model_number=buf[37],
            fw_major=buf[3],
            fw_minor=buf[4],
            output_enabled=bool(flags & 0x01),
            interlock=bool(flags & 0x02),
            temp_warn=bool(flags & 0x04),
            over_temp=bool(flags & 0x08),
            packet_errors=(flags >> 4) & 0x0F,
            dac_rate=struct.unpack_from("<I", buf, 10)[0],
            max_dac_rate=struct.unpack_from("<I", buf, 14)[0],
            buffer_free=struct.unpack_from("<H", buf, 19)[0],
            buffer_size=struct.unpack_from("<H", buf, 21)[0],
            battery_percent=buf[23],
            temperature_c=struct.unpack_from("<b", buf, 24)[0],
            connection_type_raw=buf[25],
            serial_number=serial,
            reported_ip=ip_str,
        )
```

### 9.6 Backpressure handling

The cube's ringbuffer is 6000 samples. If you push faster than the cube scans, the buffer fills and packets are dropped or queued at unbounded latency. The official strategy:

```python
# Target: keep at most ~1000 samples in flight
TARGET_BUFFER_FREE = 5000

# Streaming loop
while running:
    info = cube.get_ringbuf_empty()  # cheap, ~0ms locally
    if info is not None and info < TARGET_BUFFER_FREE:
        # cube is full enough; sleep until ~100 samples scan out
        time.sleep(100 / dac_rate)
        continue
    points = next_frame()
    cube.send_frame(points, frame_num=frame_counter)
    frame_counter += 1
```

Periodic `GET_FULL_INFO` heartbeats also keep the cube's "comms timer" alive (4-second timeout in firmware).

### 9.7 Security handshake (optional, future)

The cube has an SHA204 challenge/response (`0xB0`/`0xB1`). In libLaserdockCore source, the calling code is **commented out** — meaning the public protocol does not require it for basic operation including SET_OUTPUT.

We may discover that 2.5W cubes with FW 0.23 do require it for SET_OUTPUT. If so, we'll need to:
1. Capture LaserOS's challenge/response with Wireshark.
2. Reverse-engineer the SHA204 key (or find it documented).
3. Implement the handshake before SET_OUTPUT calls.

Stub it out as a TODO and only implement when we hit the brick wall. As of this design phase, GET_FULL_INFO works without it, which means we have full diagnostic capability today.

### 9.8 Probe results from Cole's hardware (verification)

Successful probe response (verified 2026-05-09):

```
GET_FULL_INFO (0x77) -> 64B from 169.254.40.83:45457
  model_name      : 'Wifi LaserCube 2.5W'
  model_number    : 1
  firmware        : 0.23
  connection_type : WIFI_CLIENT (raw=2)
  output_enabled  : False
  interlock       : True
  temp_warn       : False
  over_temp       : False
  packet_errors   : 0
  dac_rate        : 30000 pps
  max_dac_rate    : 30000 pps
  rx_buffer       : 6000 free / 6000 total
  battery         : 255% (= AC powered, sentinel value)
  temperature     : 51 degC
  serial          : c4:5b:be:88:53:24
  reported_ip     : 169.254.40.83

GET_RINGBUF_EMPTY (0x8a) -> 4B
  empty_sample_count: 6000  (full buffer free, idle)

GET_ALIVE (0x27 on port 45456) -> 2B
  [0x27, 0x00] (canonical)
```

These values populate the defaults in `config/settings.py`.

---

## 10. Safety system

### 10.1 Architecture (keep, don't rebuild)

The current `utils/safety.py` `SafetySystem` is well-shaped:
- Latched arm/disarm state.
- Latched e-stop (requires explicit reset, not automatic).
- No-fire zones (rectangles in pixel space).
- `check_point()` gate function for laser fire.

Keep all of this. The architecture is correct.

### 10.2 What changes: defaults and tunables

Per Cole's directive — strip artificial gating, keep real safety:

```python
# config/settings.py — safety section
SAFETY_ARM_REQUIRED: bool = True            # operator must press Arm once
SAFETY_ARM_STICKY: bool = True              # stays armed across sessions until disarm
SAFETY_ESTOP_LATCH: bool = True             # e-stop requires explicit reset (KEEP)
SAFETY_NO_FIRE_ZONES_ENABLED: bool = True   # KEEP
SAFETY_DWELL_LIMIT_ENABLED: bool = True     # toggle (Cole's preference)
SAFETY_DWELL_LIMIT_MS: int = 80             # tunable
SAFETY_IDLE_BLANK_ENABLED: bool = True      # toggle (Cole's preference)
SAFETY_MANUAL_FIRE_ALLOWED: bool = True     # fire on operator command without target lock
SAFETY_MAX_POWER_PCT: int = 100             # GUI slider, percentage
SAFETY_DEV_MODE: bool = False               # CLI --dev: bypass arm-required, etc.
SAFETY_DRY_FIRE_MODE: bool = False          # CLI --dry-fire: log commands, don't send
```

### 10.3 New safety feature: range-aware object-size guard

Toggleable. Uses Kinect depth.

For each candidate target, compute its world-space size from pixel-area + depth + sensor intrinsics. If the target is larger than a configurable threshold (default: ~human fist, or ~10cm at any range), refuse to fire.

```python
# utils/safety.py — new method
def check_target_size(self, detection: Detection) -> bool:
    """Returns True if it's safe to fire at this target (small enough)."""
    if not settings.SAFETY_OBJECT_SIZE_GUARD_ENABLED:
        return True
    if detection.x_world is None:
        return True  # no depth info; don't gate
    # Estimate world-space size from pixel area + depth
    depth_m = detection.z_world
    pixel_area = detection.area_pixels
    # Approx world area in mm² from intrinsics; Kinect v2 RGB ~ 70° HFoV
    world_area_mm2 = self._pixel_area_to_world_mm2(
        pixel_area, depth_m, fov_h_rad=math.radians(70))
    # Compare to threshold (configurable)
    max_mm2 = settings.SAFETY_MAX_TARGET_AREA_MM2  # e.g. 10000 = ~10cm × 10cm
    return world_area_mm2 < max_mm2
```

Tunable threshold defaults to "larger than human fist at measured range" but is fully configurable.

### 10.4 New safety feature: 3D no-fire zones

Existing 2D no-fire zones (rectangles in pixel space) are kept and remain useful. Adding a 3D variant only valid when Kinect depth is available.

```python
@dataclass
class NoFireZone3D:
    """Axis-aligned box in Kinect world coordinates (meters)."""
    min_x: float
    min_y: float
    min_z: float
    max_x: float
    max_y: float
    max_z: float

# Example uses:
# - "don't fire below 1m altitude" (head-and-up): max_y = 1.0
# - "don't fire within 2m of sensor": min_z = 2.0
# - "don't fire at this chair": specific bounding box
```

### 10.5 Dev mode and dry-fire mode

```python
# Dev mode (--dev flag): for development convenience
if settings.SAFETY_DEV_MODE:
    # Skip arm requirement
    # Allow manual fire from CLI
    # Single-key keyboard shortcuts in GUI for quick test fires
    pass

# Dry-fire mode (--dry-fire flag): full pipeline without photons
if settings.SAFETY_DRY_FIRE_MODE:
    # LaserManager logs what would have been sent
    # LaserCubeInterface.send_frame() returns success without sending
    # GUI overlays still show targeting cross/spiral
    # Useful for full system validation with cube physically off
    pass
```

---

## 11. GUI (PySide6)

Keep current architecture. Add the missing controls.

### 11.1 New controls in ControlPanel

- **Classifier toggle + status** (Enabled/Disabled, Heuristic/ML)
- **Laser power slider** (10–100%)
- **Tracker mode dropdown** (kalman_pixel, kalman_norm_dt, kalman_3d, iou_only)
- **Dwell limit toggle + ms input**
- **Idle blank toggle**
- **Object-size guard toggle + max-area input**
- **Active sensor selector** (GoPro, Kinect, Local cam, Multiple)
- **Calibration profile selector** (per active sensor)
- **Web monitor toggle** (start/stop server, show URL)
- **Test fire button** (with safety dialog)
- **Dry-fire mode toggle**

### 11.2 New dialog: SettingsDialog

Full editor for `config/settings.py` overrides. Per-section tabs. Saves to `user_data/config.json` via ConfigManager. Applied on next start, with subset of "live-applicable" settings (sensitivity, power, dwell) applied immediately.

### 11.3 Camera view enhancements

- Per-sensor view tabs (or split view) when multiple sensors active.
- Trajectory history overlay (configurable history length).
- Calibration quality heatmap overlay (after calibration).
- Depth visualization overlay (Kinect mode).
- IR visualization overlay (Kinect mode).
- 3D no-fire zone visualization (projected into camera view).

### 11.4 Calibration dialog enhancements

- Pattern selector (grid, halton, dragline, windmill).
- Point count slider.
- Auto/manual/hybrid detection mode.
- Live error visualization.
- "Rerun bad points" button.
- Profile name field for saving.

---

## 12. Web monitoring server

### 12.1 Architecture

Stdlib-only HTTP server (no Flask/FastAPI) running in its own thread. Subscribes to bus signals, pushes state to the browser via:

- `GET /` — dashboard HTML (single inline page, dark theme)
- `GET /stream` — MJPEG live camera feed
- `GET /api/status` — JSON system state (snapshot)
- `GET /api/events?since=<ts>` — JSON event log incremental
- `GET /api/sensors` — JSON list of active sensors with status
- `GET /api/tracks` — JSON live tracks
- `POST /api/disarm` — emergency disarm via web (configurable, off by default)

### 12.2 Event types

```python
EVENT_TYPES = [
    "track_acquired",
    "track_confirmed",
    "track_lost",
    "laser_fire",
    "laser_cooldown",
    "calibration_started",
    "calibration_complete",
    "emergency_stop",
    "sensor_connected",
    "sensor_disconnected",
    "config_changed",
    "system_armed",
    "system_disarmed",
    "shot_fired",
    "shot_blocked_safety",
]
```

### 12.3 Configuration

```python
# config/settings.py — web monitor section
WEB_MONITOR_ENABLED: bool = False           # off by default
WEB_MONITOR_BIND_HOST: str = "127.0.0.1"    # localhost only by default
WEB_MONITOR_PORT: int = 8765
WEB_MONITOR_MJPEG_FPS: int = 10             # downsampled
WEB_MONITOR_MJPEG_QUALITY: int = 70
WEB_MONITOR_EVENT_LOG_SIZE: int = 1000
WEB_MONITOR_REQUIRE_TOKEN: bool = False     # simple shared-secret optional
WEB_MONITOR_TOKEN: str = ""
```

**Default-localhost is deliberate.** Exposing on LAN requires explicit config change. Even then, no real auth — flag as private/local-only.

### 12.4 Console-safe printing

The Codex review noted that `claude_upgrades/monitor.py` printed unicode arrows that fail under Windows cp1252. Use ASCII-only console output, OR force UTF-8:

```python
# At server start
import sys
sys.stdout.reconfigure(encoding='utf-8', errors='replace')  # Python 3.7+
```

Avoid characters like `→`, `█`, `■` in print statements regardless.

---

## 13. Configuration

### 13.1 Single source of truth: `config/settings.py`

All tunables live here. **No magic numbers anywhere else in the codebase.** If you find one, lift it to settings.py before anything else.

### 13.2 User overrides: `user_data/config.json`

ConfigManager reads `settings.py` defaults, overlays JSON overrides on top. Never edit `settings.py` to change a value for a session — change it via GUI or directly in `config.json`.

### 13.3 Runtime-changeable vs restart-required

Mark each setting:

```python
# In settings.py
DETECTION_THRESHOLD: int = 20
DETECTION_THRESHOLD__RUNTIME = True         # changeable while app runs

LASERCUBE_DEFAULT_IP: str = "169.254.40.83"
LASERCUBE_DEFAULT_IP__RUNTIME = False       # requires reconnect
```

Or use a more structured approach with annotations. Pick a pattern and document it in module docstring.

### 13.4 Calibration storage

Per profile (sensor + scene combo):

```
user_data/calibrations/
  ├── gopro_garage_default.json
  ├── gopro_living_room.json
  ├── kinect_v2_garage_default.json
  └── kinect_v2_living_room.json
```

JSON schema in section 8.4.

### 13.5 Network configuration profile

Critical for the multi-NIC fix. Stored separately because it's per-machine, not per-user-preference:

```python
# config/settings.py — network section
NETWORK_PROFILE: str = "auto"              # "auto" | "lan_client" | "wifi_server" | "manual"
LASERCUBE_DEFAULT_IP: str = "169.254.40.83"
LASERCUBE_BIND_SRC_IP: str = "auto"        # "auto" detects APIPA, or specify "169.254.25.216"
```

`"auto"` for `LASERCUBE_BIND_SRC_IP` triggers a startup probe: enumerate local interfaces, pick the one with a 169.254.x.x address on the same /16 as the cube IP. Falls back to `0.0.0.0` if not found.

---

## 14. Logging and diagnostics

### 14.1 Existing rotating file logger

Keep. `utils/logging_utils.py` already handles this well. Logs to `logs/iron_dome.log`, 10MB rotation, 5 backups.

### 14.2 Session recorder (NEW)

`utils/session_recorder.py`. Subscribes to all bus signals, writes append-only JSONL to `user_data/sessions/<timestamp>.jsonl`.

```jsonl
{"ts": 1715292345.123, "type": "session_start", "version": "0.2.0"}
{"ts": 1715292345.456, "type": "sensor_connected", "sensor_id": "kinect_v2", "specs": {...}}
{"ts": 1715292345.789, "type": "system_armed", "operator": "cole"}
{"ts": 1715292346.012, "type": "detection", "sensor_id": "gopro", "x_norm": 0.234, "y_norm": 0.567, ...}
{"ts": 1715292346.045, "type": "track_acquired", "track_id": 1, "first_detection": {...}}
{"ts": 1715292346.123, "type": "shot_fired", "track_id": 1, "galvo_xy": [2048, 1900], "duration_ms": 50}
...
```

Use cases:
- Replay sessions for debugging.
- Mine for false-positive vs true-positive detections to train the classifier.
- Forensic record of what the system did.
- Support evidence ("the system fired at this thing because...").

### 14.3 Configuration

```python
# config/settings.py — logging section
LOG_LEVEL: str = "INFO"
SESSION_RECORDING_ENABLED: bool = True
SESSION_RECORDING_DIR: Path = BASE_DIR / "user_data" / "sessions"
SESSION_RECORDING_MAX_FILES: int = 100      # auto-prune
SESSION_RECORDING_MAX_TOTAL_MB: int = 1000  # cap total size
```

---

## 15. Entry point and CLI

`main.py` stays minimal (existing pattern is correct). All CLI flags route through ConfigManager overlays:

```python
# main.py — CLI argument layout
parser.add_argument("--log-level", default=None,
                    choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"])
parser.add_argument("--config", default=None, metavar="PATH",
                    help="Custom config JSON file location")
parser.add_argument("--headless", action="store_true",
                    help="Run without GUI (web monitor + bus only)")
parser.add_argument("--debug", action="store_true",
                    help="Enable debug logging + extra diagnostics")
parser.add_argument("--dev", action="store_true",
                    help="Skip arm-required, enable manual fire shortcuts")
parser.add_argument("--dry-fire", action="store_true",
                    help="Run full pipeline without sending laser commands")
parser.add_argument("--no-laser", action="store_true",
                    help="Don't even connect to the laser (camera-only test)")
parser.add_argument("--record", default=None, metavar="PATH",
                    help="Auto-start session recording to specified path")
parser.add_argument("--camera", default=None,
                    choices=["gopro", "kinect", "local", "all"],
                    help="Active sensor(s) for this run")
parser.add_argument("--monitor-port", type=int, default=None,
                    help="Override web monitor port (enables monitor)")
parser.add_argument("--no-monitor", action="store_true",
                    help="Disable web monitor")
parser.add_argument("--fullscreen", action="store_true",
                    help="Launch GUI fullscreen (kiosk mode)")
parser.add_argument("--cube-ip", default=None,
                    help="Override LaserCube IP address")
parser.add_argument("--bind-ip", default=None,
                    help="Override LaserCube source bind IP (multi-NIC fix)")
```

Mode interactions:
- `--headless` is incompatible with `--fullscreen`.
- `--dev` implies `SAFETY_ARM_REQUIRED = False`.
- `--dry-fire` overrides `--no-laser` (dry-fire still loads the interface for completeness, just doesn't send).
- `--no-laser` is mutually exclusive with `--dry-fire`.

---

## 16. Testing and verification

### 16.1 What's been verified (design phase)

| Component | Verified by | Result |
|---|---|---|
| LaserCube reachability | `ping 169.254.40.83` | <1ms RTT |
| LaserCube web admin | Browser to `http://169.254.40.83/` | Login page loads |
| Multi-NIC routing problem | Ping output showing wrong-gateway responses | Confirmed |
| Source-IP binding fix | `python -c "..."` one-liner | Worked |
| GET_FULL_INFO bytes | Probe script `--src-ip 169.254.25.216 --verbose` | All fields parsed cleanly |
| GET_RINGBUF_EMPTY bytes | Same probe | Returns 6000 (idle) |
| GET_ALIVE bytes | Same probe | Returns `[0x27, 0x00]` |
| Protocol values match LaserOS | Cross-reference probe vs LaserOS UI | All values agree |

### 16.2 What needs verification (during/post implementation)

| Component | Test | Acceptance |
|---|---|---|
| LaserCubeInterface.connect() | Unit test with real cube | Returns True, populates info |
| Sample data packet wire format | Wireshark capture during draw | 10-byte samples, sequence nums increment |
| SET_OUTPUT enables laser | Manual test, low power, safety lens | Visible beam |
| Backpressure behavior | Stream more than 6000 samples without get_ringbuf | Verify no underrun/overrun artifacts |
| Calibration accuracy | Aim laser at known camera-pixel target | <2px reprojection error |
| Lag estimate from drag-line | Compare to synthetic injected lag | Within ~10ms |
| Kinect+GoPro fusion | Plant target visible to both | Single track in fused mode |
| Web monitor MJPEG | View in browser while detecting | Smooth at 10fps |
| Session recording fidelity | Replay a session, compare visual output | Identical decisions |
| Object-size guard | Walk into frame | System refuses to fire |
| 3D no-fire zone | Configure floor-zone, kneel into it | Refuses |
| Practice mode | Run with `--dry-fire`, cube off | Full pipeline runs, no errors |

### 16.3 Test fixtures

Build a `tests/` directory with:

```
tests/
├── unit/
│   ├── test_classifier.py          # feature vector, heuristic rules
│   ├── test_lasercube_packing.py   # point packing, sequence numbers
│   ├── test_lasercube_parsing.py   # full info parse round-trip
│   ├── test_coordinate_mapper.py   # homography forward/inverse
│   ├── test_safety_system.py       # arm, e-stop, zones
│   ├── test_tracker.py             # Kalman, assignment, coasting
│   └── test_patterns.py            # grid, halton, dragline, windmill
├── integration/
│   ├── test_dry_fire_pipeline.py   # full bus, no hardware
│   └── test_session_replay.py      # JSONL → reconstructed events
├── fixtures/
│   ├── golden_full_info_64b.bin    # captured cube response
│   ├── golden_session.jsonl        # replayable session
│   └── test_video.mp4              # sample mosquito footage
└── manual/
    ├── README.md                    # hardware-required test procedures
    └── test_calibration.md          # step-by-step calibration validation
```

---

## 17. Bootstrap implementation order

The actual implementation plan, in dependency order. Each step builds on the previous; nothing reaches forward.

### Step 1 — Foundation (no behavior change yet)

**Goal**: Make the rewrite possible without breaking things.

- Create `docs/BOOTSTRAP.md` (this document).
- Create `docs/PROTOCOL.md` (LaserCube quick-ref extracted from section 9).
- Create `docs/COORDINATES.md` (coordinate contract from section 4).
- Add `timestamp: float` field to `Detection` dataclass.
- Define `Sensor` ABC and `SensorFrame` dataclass at `sensors/base.py`.
- Move `camera/` → `sensors/`, refactor `CameraManager` to be `SensorManager` with backwards-compatible API.

**Acceptance**: existing app still launches and functions identically.

### Step 2 — Configuration cleanup

- Audit `config/settings.py` for missing tunables. Add all from sections 6, 7, 8, 9, 10, 12, 13, 14, 15.
- Remove every magic number found elsewhere; replace with `settings.X` references.
- Add `__RUNTIME` annotations.
- Ensure ConfigManager handles all new keys.

**Acceptance**: `git grep -E "= [0-9]+" -- '*.py' ':!config/'` returns very few lines, all justified.

### Step 3 — Classifier port

- Port `claude_upgrades/classifier.py` → `detection/classifier.py`.
- Make heuristic thresholds configurable via settings.
- Wire into `MosquitoDetector._detect_bgsub()` with toggle.
- Add unit tests for feature vector + heuristic rules.

**Acceptance**: classifier toggleable in GUI; integration verified on saved test video.

### Step 4 — Tracker upgrades

- Split `detection/tracker.py` → `tracking/tracker.py` + `tracking/kalman_track.py` + `tracking/assignment.py`.
- Implement multi-modal tracker selectable in settings.
- Add `dt`-based Kalman, confidence decay, coasting, history.
- Migrate existing pixel-space mode under `kalman_pixel`.
- Add `kalman_norm_dt` (new default) and `iou_only`.

**Acceptance**: existing tracking still works; new modes pass unit tests.

### Step 5 — Calibration upgrades

- Add `targeting/patterns.py` with grid, halton, dragline, windmill generators.
- Extend `targeting/calibration.py`:
  - Configurable patterns
  - Auto/manual/hybrid detection modes
  - Quality metrics
  - Per-sensor profile storage
- Update `gui/calibration_dialog.py` with new controls.

**Acceptance**: can run calibration in all four pattern modes; profiles save/load correctly; reprojection error displayed.

### Step 6 — LaserCube interface rewrite

This is the big one. **Discard `laser/lasercube_interface.py` entirely.** Replace with new `laser/lasercube.py` per section 9.5.

- Implement `LaserCubeInterface` per the reference code.
- Implement `LaserPoint` and `LaserInfo` dataclasses.
- Implement `_parse_full_info()` per the byte layout in section 9.3.
- Wire backpressure tracking and periodic GET_FULL_INFO heartbeat.
- Update `laser/laser_manager.py` to use new interface.
- Discover/configure source IP automatically (per section 13.5).

**Acceptance**: Test on Cole's hardware. `LaserCubeInterface.connect()` returns True. `get_full_info()` returns matching values to LaserOS UI. **Do NOT call `enable_output()` until safety-gate integration is complete in Step 7.**

### Step 7 — Safety integration with new interface

- Wire `LaserManager` through `SafetySystem` for the new interface.
- Implement sticky arm, dev mode, dry-fire mode flags.
- Add object-size guard (using Kinect depth, optional).
- Add 3D no-fire zones (Kinect-aware, optional).
- Add manual fire button with confirmation dialog.
- Test with cube on, safety lens on, low power.

**Acceptance**: cube draws calibration cross when armed and operator clicks "test fire." Safety gate refuses to fire when disarmed. E-stop latches and requires reset.

### Step 8 — GoPro Hero 13 integration

- Implement `sensors/gopro.py` with Hero 13 Open API.
- Connect via WiFi or USB tether.
- Live preview at 30 fps.
- Optional: trigger forensic high-fps SD-card recording on engagement.

**Acceptance**: GoPro feed visible in GUI; full pipeline runs through GoPro path.

### Step 9 — Kinect v2 integration

- Implement `sensors/kinect_v2.py`.
- RGB + depth + IR streams.
- World-coordinate mapping via Kinect SDK.
- Body tracking → 3D no-fire zone candidates.

**Acceptance**: Kinect feed visible in GUI; depth + IR streams available; world coordinates flowing into safety system.

### Step 10 — Multi-sensor fusion

- Implement Tracker fusion mode.
- Detections from multiple sensors merge by spatial proximity.
- 3D promotion when Kinect-with-depth detection joins a 2D track.

**Acceptance**: Plant target visible to both sensors; single fused track in `kalman_norm_dt` mode with depth.

### Step 11 — Web monitoring

- Implement `monitoring/web_server.py`.
- Dashboard, MJPEG stream, status API, event log API.
- Wire into bus.
- Configurable via settings, off by default, localhost by default.

**Acceptance**: browser to `http://localhost:8765/` shows live state; events appear; MJPEG smooth.

### Step 12 — CLI completion

- Implement all CLI flags from section 15.
- Headless mode (no GUI).
- Dry-fire mode end-to-end test.

**Acceptance**: `python main.py --headless --dry-fire --camera kinect` runs full pipeline without GUI or laser.

### Step 13 — Session recording

- Implement `utils/session_recorder.py`.
- Subscribe to all bus signals.
- JSONL output, size-capped, prunable.
- Replay tool (`tools/replay_session.py`).

**Acceptance**: Run a session; replay reconstructs the event stream identically.

### Step 14 — Audio sensor stub

- Stub `sensors/kinect_audio.py` with the SDK audio API.
- Capture 4-channel raw + beamformed mono.
- Placeholder wingbeat detector.
- Wire into bus as a separate sensor stream.

**Acceptance**: audio captured to disk; placeholder detector emits diagnostic events. Real wingbeat detection deferred to future.

### Step 15 — Polish

- README and operator quick-start.
- Module docs updated.
- Integration test suite passing.
- Manual hardware test checklist.

**Acceptance**: someone other than Cole could clone the repo, follow README, get to working calibration on their cube within 30 minutes.

---

## 18. Open questions and future work

### 18.1 Open questions (resolve during implementation)

1. **Connection type enum offset**: GET_FULL_INFO byte 25 reports `2` (= raw value), source code does `+1` to map to enum. We mapped `3` → `WIFI_CLIENT`. But Cole's cube is in ETHERNET_CLIENT mode physically. Either the cube reports its WiFi state regardless of physical link, or the +1 is in the wrong direction. Verify by powering the cube via WiFi-only and re-reading.

2. **Security handshake requirement**: Does FW 0.23 require `0xB0`/`0xB1` before `SET_OUTPUT`? Test by trying SET_OUTPUT cold and observing. If it fails, capture LaserOS's handshake with Wireshark.

3. **Compressed sample format (`0x9A`)**: Worth implementing for bandwidth reduction at 30k pps? Or stick with uncompressed (`0xA9`)? Likely not needed at our packet rates.

4. **GoPro Hero 13 max preview fps**: API docs say 30 fps; some report 60 fps in specific modes. Test on actual hardware to determine ceiling.

5. **Kinect SDK on Windows 11**: Microsoft support officially ended. May require driver workarounds on newest Windows.

### 18.2 Deferred to post-V1

- **Audio wingbeat detection**: real implementation, not stub. Requires Kinect mic array characterization for mosquito species.
- **ML-based species classification**: train classifier on real labeled data once enough sessions recorded.
- **Predictive lead-aim**: Kalman state already has velocity; use it for time-of-flight compensation. Trivial once tracking is solid.
- **Pan/tilt rig**: motorize the GoPro/Kinect for wider coverage. Mechanical engineering project.
- **Linux support**: replace pykinect2 with libfreenect2; LaserCube already cross-platform via UDP.
- **Multi-cube support**: cube link or LAN-coordinated targeting. Not a useful feature for one room.
- **Outdoor mode**: Kinect IR fails in sunlight. Daylight mode would need different sensor strategy (high-fps RGB only).

---

## 19. Appendix A — Verified hardware specs

### A.1 Cole's LaserCube

```
Model: WiFi LaserCube 2.5W
Model number: 1
Firmware: 0.23
Serial: c4:5b:be:88:53:24
DAC rate (default & max): 30,000 pps
Buffer size: 6,000 samples
Output: Class 3B, 2.5W (upper bound — treat as Class 4)
Power: AC via charger (battery_percent reports 0xFF = "AC powered")
Idle temperature: 45–51°C (varies)
Interlock: hardware-enabled
Connection: LAN client, direct ethernet to PC via APIPA
IP: 169.254.40.83
Web admin: http://169.254.40.83/ (HOME, CONFIG, UPDATE tabs)
```

### A.2 Cole's PC

```
Default ethernet APIPA: 169.254.25.216 (on ifIndex 13)
Default WiFi: 192.168.1.x with gateway 192.168.1.125
VPN: Mullvad daemon active (capture potential)
Other UDP-busy processes: Plex, qBittorrent, Steam, Opera (port collision risk for monitor)
```

### A.3 Cole's GoPro

```
Model: Hero 13 Black
Connection options: WiFi (Hero AP mode at 10.5.5.9), USB-C tether, smartphone bridge
Live preview: 30–60 fps depending on mode
Slow-mo (240+ fps): SD-card capture only, NOT live-streamable
```

### A.4 Cole's Kinect

```
Model: v2 (Xbox One Kinect)
SDK: Microsoft Kinect SDK 2.0
Python binding: pykinect2
Streams: RGB 1920×1080 BGRA / Depth 512×424 uint16 mm / IR 512×424 uint16 / Body 6×25 joints / Audio 4-element 16kHz
Notable: Verified working ("Kinect Studio works just fine")
USB 3.0 controller compatibility: confirmed working
```

---

## 20. Appendix B — Network setup specifics

### B.1 The multi-NIC routing problem (root cause)

When Cole's PC has multiple active network interfaces:
- WiFi: `192.168.1.x` with default gateway `192.168.1.125`
- Direct ethernet: APIPA `169.254.25.216` on ifIndex 13 (route to 169.254.0.0/16 on-link)
- VPN: Mullvad TAP interface

…and a UDP socket binds to `("0.0.0.0", 45457)` and sends to `169.254.40.83:45457`, Windows picks the source IP based on its routing logic. **Sometimes it picks the WiFi interface.** The packet then has source `192.168.1.x:45457`, the cube responds back to `192.168.1.x:45457`, and the response is dropped because that IP isn't on the network the cube is connected to.

ICMP recovers from this via auto-retry. UDP doesn't.

### B.2 The fix (and how it's wired into the rewrite)

Bind UDP sockets to a **specific local IP**, forcing Windows to use that interface for both send and receive.

Cole's working command from probe verification:

```
python scripts\lasercube_protocol_probe.py --src-ip 169.254.25.216 --verbose
```

In `LaserCubeInterface`:

```python
self._cmd_sock.bind(("169.254.25.216", 45457))
```

Auto-detection logic at startup:

```python
def detect_apipa_for_cube(cube_ip: str) -> Optional[str]:
    """Find a local interface IP on the same /16 as the cube."""
    if not cube_ip.startswith("169.254."):
        return None
    cube_prefix = ".".join(cube_ip.split(".")[:2])  # "169.254"
    # Enumerate interfaces (psutil or netifaces; or use ipconfig parse)
    for iface in get_local_ipv4_addresses():
        if iface.startswith(cube_prefix):
            return iface
    return None
```

Stored as `LASERCUBE_BIND_SRC_IP = "auto"` by default in settings; resolved on first connect; cached for subsequent runs.

### B.3 Windows-specific firewall notes

Inbound UDP to port 45457 may need a firewall exception:

```powershell
# As admin — one-time setup
New-NetFirewallRule -DisplayName "NoMoSkeeters LaserCube CMD" `
    -Direction Inbound -Protocol UDP -LocalPort 45457 -Action Allow
New-NetFirewallRule -DisplayName "NoMoSkeeters LaserCube DATA" `
    -Direction Inbound -Protocol UDP -LocalPort 45458 -Action Allow
New-NetFirewallRule -DisplayName "NoMoSkeeters LaserCube ALIVE" `
    -Direction Inbound -Protocol UDP -LocalPort 45456 -Action Allow
```

The first time the app runs, Windows may pop up a firewall dialog for `python.exe`. If the user clicks Cancel, all UDP fails. App should detect this case and instruct.

### B.4 LaserCube modes (reference)

Per the manual:

| Mode | Cube role | Indicator | Notes |
|---|---|---|---|
| WiFi Server | DHCP server, AP at 192.168.1.1 | (per manual) | Default. Connect with phone/PC directly. |
| WiFi Client | Joins existing WiFi via WPS or creds | Solid blue when connected | Useful but stuttering risk. |
| LAN Server | DHCP server on ethernet | Solid yellow when client connects | For USB→LAN adapters. |
| **LAN Client** | Joins existing LAN, gets DHCP, or APIPA if direct | (Cole's mode) | **Recommended for low-latency targeting.** |

---

## 21. Appendix C — Useful diagnostic commands

```powershell
# Verify cube is reachable
ping 169.254.40.83

# Inspect local routing for APIPA range
Get-NetRoute -DestinationPrefix '169.254.0.0/16' | Format-Table -AutoSize

# Find your APIPA address
ipconfig | Select-String '169.254'

# See who has the LaserCube ports bound (LaserOS or our app)
Get-NetUDPEndpoint | Where-Object { $_.LocalPort -in 45456,45457,45458 } | ForEach-Object {
    $proc = Get-Process -Id $_.OwningProcess -ErrorAction SilentlyContinue
    [PSCustomObject]@{
        LocalPort = $_.LocalPort
        Process   = $proc.ProcessName
        PID       = $_.OwningProcess
    }
} | Format-Table -AutoSize

# Probe protocol (read-only, safe to run anytime)
python scripts\lasercube_protocol_probe.py --src-ip 169.254.25.216 --verbose

# Probe with broadcast discovery
python scripts\lasercube_protocol_probe.py --src-ip 169.254.25.216 --discover --verbose

# Capture LaserCube traffic with Wireshark display filter:
#   udp.port == 45457 or udp.port == 45456 or udp.port == 45458

# Enumerate USB devices (for Kinect troubleshooting)
Get-PnpDevice -PresentOnly | Where-Object { $_.FriendlyName -like "*Kinect*" }

# Check Windows Firewall state
Get-NetFirewallProfile | Format-Table Name, Enabled

# One-liner protocol test (no script needed)
python -c "import socket; s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); s.bind(('169.254.25.216', 45457)); s.settimeout(2.0); s.sendto(bytes([0x77]), ('169.254.40.83', 45457)); data, addr = s.recvfrom(4096); print(f'GOT {len(data)} bytes from {addr}'); print(data.hex())"
```

---

## End

This document is the source of truth for the rewrite. When the implementation diverges from a value here, fix the implementation OR fix the document, never both at once. Anything you find missing — a TODO, a value, a constraint — add a section rather than letting it stay in your head.

Build it like the protocol probe: verify, document, then ship.
