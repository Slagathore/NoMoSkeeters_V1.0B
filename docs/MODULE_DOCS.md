# Iron Dome Anti-Mosquito System — Module Documentation

> **Mission Statement**
> The Iron Dome Anti-Mosquito System is a Python application that fuses live video
> from a GoPro action camera with precision galvo targeting via a LaserCube ILDA
> laser projector to autonomously detect and neutralise mosquitos in real-time.
> The system achieves this through a layered pipeline: background-subtraction (or
> YOLO) detection → Kalman-filter multi-object tracking → homography coordinate
> mapping → safety-gated laser firing, wrapped in a PySide6 GUI with live video
> overlay and operator controls.
>
> **Safety is non-negotiable.** The system defaults to disarmed. It will not fire
> unless the operator explicitly arms it, all safety checks pass, and the target
> falls outside all configured no-fire zones.

---

## Table of Contents

1. [Project Structure](#project-structure)
2. [Setup & Requirements](#setup--requirements)
3. [Hardware Configuration](#hardware-configuration)
4. [Quick-Start Guide](#quick-start-guide)
5. [Calibration Walkthrough](#calibration-walkthrough)
6. [Safety System Overview](#safety-system-overview)
7. [Module Reference](#module-reference)
   - [main.py](#mainpy)
   - [config/settings.py](#configsettingspy)
   - [config/config_manager.py](#configconfig_managerpy)
   - [utils/logging_utils.py](#utilslogging_utilspy)
   - [utils/safety.py](#utilssafetypy)
   - [camera/gopro_interface.py](#cameragopro_interfacepy)
   - [camera/camera_manager.py](#cameracamera_managerpy)
   - [detection/detector.py](#detectiondetectorpy)
   - [detection/tracker.py](#detectiontrackerpy)
   - [laser/lasercube_interface.py](#laserlasercube_interfacepy)
   - [laser/laser_manager.py](#laserlaser_managerpy)
   - [targeting/coordinate_mapper.py](#targetingcoordinate_mapperpy)
   - [targeting/calibration.py](#targetingcalibrationpy)
   - [gui/camera_view.py](#guicamera_viewpy)
   - [gui/control_panel.py](#guicontrol_panelpy)
   - [gui/calibration_dialog.py](#guicalibration_dialogpy)
   - [gui/main_window.py](#guimain_windowpy)
8. [Data Flow Diagram](#data-flow-diagram)
9. [Troubleshooting](#troubleshooting)

---

## Project Structure

```
example/
├── main.py                      Entry point — parse CLI, init logging, launch GUI.
├── requirements.txt             All Python dependencies.
├── docs/
│   └── MODULE_DOCS.md           This file.
├── config/
│   ├── __init__.py
│   ├── settings.py              All application constants and defaults.
│   └── config_manager.py        Persistent JSON-backed override layer.
├── utils/
│   ├── __init__.py
│   ├── logging_utils.py         Rotating file + console log setup.
│   └── safety.py                Arm/disarm/e-stop/no-fire-zone gatekeeper.
├── camera/
│   ├── __init__.py
│   ├── gopro_interface.py       GoPro Open API HTTP wrapper + stream control.
│   └── camera_manager.py        QThread frame-capture loop; OpenCV VideoCapture.
├── detection/
│   ├── __init__.py
│   ├── detector.py              MOG2 / YOLO detection QThread worker.
│   └── tracker.py               Kalman + Hungarian multi-object tracker.
├── laser/
│   ├── __init__.py
│   ├── lasercube_interface.py   Raw UDP socket driver for LaserCube hardware.
│   └── laser_manager.py         QThread targeting loop; safety-gated firing.
├── targeting/
│   ├── __init__.py
│   ├── coordinate_mapper.py     Homography — camera pixels ↔ laser coordinates.
│   └── calibration.py           Point-pair collection & homography computation.
└── gui/
    ├── __init__.py
    ├── camera_view.py           OpenCV-frame-to-QPixmap widget with overlays.
    ├── control_panel.py         Right-dock panel (connect, safety, stats).
    ├── calibration_dialog.py    Step-by-step calibration wizard dialog.
    └── main_window.py           Top-level QMainWindow; wires all subsystems.
```

---

## Setup & Requirements

**Python version:** 3.10 or newer (uses `match` and `X | Y` type union syntax).

### Install dependencies

```bash
pip install -r requirements.txt
```

Key packages:
| Package | Purpose |
|---|---|
| `PySide6` | Qt6 GUI framework |
| `opencv-python` | Frame capture, MOG2 background subtraction, homography |
| `numpy` | Array operations, Kalman matrices |
| `scipy` | `linear_sum_assignment` (Hungarian algorithm) |
| `requests` | GoPro Open API HTTP calls |
| `ultralytics` _(optional)_ | YOLO mosquito detection model |

---

## Hardware Configuration

### GoPro Hero (OpenAPI mode)

1. Enable **GoPro App** → **Connect** → **GoPro Quik (WiFi Direct AP)** on the camera.
2. On your PC, connect to the GoPro WiFi network (SSID: `GP...`).
3. Default camera IP: **`10.5.5.9`** (port `8080` for HTTP API, UDP stream arrives at `0.0.0.0:8554`).
4. These defaults are in `config/settings.py` as `GOPRO_DEFAULT_IP`, `GOPRO_API_PORT`, `GOPRO_STREAM_UDP_PORT`.

### LaserCube

1. Power on the LaserCube and connect your PC to its WiFi AP (SSID: `LaserCube-...`).
2. Default device IP: **`192.168.1.1`**.
3. Data port: **`45456`** — sends ILDA galvo point frames.
4. Command port: **`45457`** — bidirectional control channel.
5. These defaults are in `config/settings.py` as `LASERCUBE_DEFAULT_IP`, `LASERCUBE_DATA_PORT`, `LASERCUBE_CMD_PORT`.

> ⚠ **Critical:** The GoPro and LaserCube each run their own WiFi AP. Your PC cannot be joined
> to both simultaneously on most standard WiFi adapters. Use a second USB WiFi adapter for one
> of the two devices, or use the LaserCube tethered over USB when its SDK supports it.

---

## Quick-Start Guide

```bash
# Launch with defaults
python main.py

# Launch with debug logging
python main.py --log-level DEBUG

# Launch with a custom config file
python main.py --config path/to/custom_config.json
```

**In the GUI:**

1. **Connect GoPro** — Click "Connect GoPro" in the Control Panel. The camera feed appears in the central view.
2. **Connect Laser** — Click "Connect LaserCube". Status dot turns green on success.
3. **Calibrate** — Click "Calibrate…" (Tools menu or Control Panel). Follow the wizard: the system fires each calibration point, detects the dot with the camera, and builds a homography transform.
4. **Add no-fire zones** — Click "Add Zone" and click two opposing corners on the camera view to mark regions the laser must avoid.
5. **Arm** — Click "ARM". Confirm the target area is clear.
6. **Monitor** — Active tracks and the laser crosshair appear in the camera overlay.
7. **Emergency Stop** — Press **Escape** or click "E-STOP" at any time. The laser blanks immediately.

---

## Calibration Walkthrough

Calibration maps camera pixel coordinates to LaserCube galvo coordinates via a homography matrix.

1. Ensure the LaserCube is connected and the GoPro feed is live.
2. Open **Tools → Calibrate Laser ↔ Camera…** (or the "Calibrate…" button in the Control Panel).
3. The wizard fires the laser at a grid of `CALIBRATION_GRID_ROWS × CALIBRATION_GRID_COLS` points (default: 3×3 = 9 points).
4. For each point, the system:
   - Fires the laser at the known grid coordinate.
   - Waits ~150 ms for the galvo to settle.
   - Captures a frame from the camera.
   - Detects the bright dot using background subtraction.
   - Records the (camera pixel, laser coord) pair.
5. After all points are collected, `finish()` computes the homography via `cv2.findHomography` and saves it to `CALIBRATION_FILE` (default: `~/.iron_dome/calibration.json`).
6. Calibration can be re-run at any time — new data overwrites the old mapping.

**Tips:**

- Run calibration in the same lighting conditions you'll use for hunting.
- Ensure the laser dot is clearly visible on the camera. Use a plain flat surface behind the target zone.
- The more point pairs collected, the more accurate the homography (consider increasing `CALIBRATION_GRID_COLS/ROWS` in settings).

---

## Safety System Overview

All laser firing is gated through `SafetySystem` (`utils/safety.py`). Firing is only allowed when **all** of the following are true:

| Condition                           | How to satisfy                                         |
| ----------------------------------- | ------------------------------------------------------ |
| `armed == True`                     | Operator clicked "ARM"                                 |
| `estop == False`                    | No emergency stop has been triggered (or it was reset) |
| `check_point(cam_x, cam_y) == True` | Target is outside all configured no-fire zones         |

The safety system is **fail-safe by default**: it starts disarmed and requires explicit operator action to arm. The e-stop latch can only be cleared by calling `disarm()` after an emergency stop.

**No-fire zones** are rectangles defined in camera pixel space. Any target whose predicted position falls inside a zone is silently skipped — the laser will not fire.

---

## Module Reference

---

### `main.py`

**Entry point.** Minimal intentionally.

| Symbol   | Kind     | Description                                                                                                                                                                                |
| -------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `main()` | function | Parses CLI args (`--log-level`, `--config`), calls `setup_logging()`, creates `ConfigManager`, creates `QApplication` and `MainWindow`, runs the Qt event loop, and returns the exit code. |

**Called modules:** `argparse`, `sys`, `PySide6.QtWidgets.QApplication`, `PySide6.QtCore.Qt`, `utils.logging_utils.setup_logging`, `config.config_manager.ConfigManager`, `gui.main_window.MainWindow`

---

### `config/settings.py`

**All application constants.** No classes or logic — only module-level variables.

| Variable                         | Type  | Description                                       |
| -------------------------------- | ----- | ------------------------------------------------- |
| `PROJECT_NAME`                   | str   | Human-readable application name.                  |
| `VERSION`                        | str   | Semantic version, e.g. `"1.0.0"`.                 |
| `BASE_DIR`                       | Path  | Project root directory (derived from `__file__`). |
| `CONFIG_DIR`                     | Path  | User config storage directory (`~/.iron_dome/`).  |
| `CONFIG_FILE`                    | Path  | JSON config file path (`CONFIG_DIR/config.json`). |
| `GOPRO_DEFAULT_IP`               | str   | `"10.5.5.9"` — GoPro AP IP.                       |
| `GOPRO_API_PORT`                 | int   | `8080` — GoPro HTTP API port.                     |
| `GOPRO_STREAM_UDP_PORT`          | int   | `8554` — GoPro live preview UDP port on host.     |
| `GOPRO_TIMEOUT_S`                | float | HTTP request timeout in seconds.                  |
| `GOPRO_PREVIEW_STREAM_URL`       | str   | OpenCV VideoCapture URL for GoPro UDP stream.     |
| `GOPRO_KEEP_ALIVE_INTERVAL_S`    | int   | Seconds between keep-alive pings.                 |
| `LASERCUBE_DEFAULT_IP`           | str   | `"192.168.1.1"` — LaserCube AP IP.                |
| `LASERCUBE_DATA_PORT`            | int   | `45456` — UDP data port.                          |
| `LASERCUBE_CMD_PORT`             | int   | `45457` — UDP command port.                       |
| `LASERCUBE_SCAN_RATE`            | int   | Galvo scan rate (points/sec).                     |
| `LASERCUBE_MAX_POINTS_PER_FRAME` | int   | Max points per UDP frame packet.                  |
| `LASERCUBE_COORD_MIN`            | int   | `-32767` — minimum galvo coordinate.              |
| `LASERCUBE_COORD_MAX`            | int   | `32767` — maximum galvo coordinate.               |
| `LASERCUBE_HEARTBEAT_INTERVAL_S` | float | Seconds between laser heartbeats.                 |
| `DETECTION_MIN_AREA`             | int   | Min contour area (px²) for mosquito candidate.    |
| `DETECTION_MAX_AREA`             | int   | Max contour area (px²) for mosquito candidate.    |
| `DETECTION_MIN_ASPECT`           | float | Min bounding-box aspect ratio.                    |
| `DETECTION_MAX_ASPECT`           | float | Max bounding-box aspect ratio.                    |
| `DETECTION_BLUR_KERNEL`          | int   | Gaussian blur kernel size before subtraction.     |
| `DETECTION_THRESHOLD`            | int   | Binary threshold value after subtraction.         |
| `DETECTION_MOG2_HISTORY`         | int   | MOG2 background history frame count.              |
| `DETECTION_MOG2_VARTH`           | float | MOG2 variance threshold.                          |
| `DETECTION_FPS_TARGET`           | int   | Target FPS for the detector thread.               |
| `DETECTION_YOLO_CONF`            | float | YOLO confidence threshold (0–1).                  |
| `DETECTION_YOLO_MODEL`           | str   | YOLO weights path or model name.                  |
| `DETECTION_MORPH_KERNEL`         | int   | Morphological open/close kernel size.             |
| `TRACKER_MAX_DISAPPEARED`        | int   | Frames before a track is dropped.                 |
| `TRACKER_MAX_DISTANCE`           | float | Max pixel distance for Hungarian match.           |
| `TRACKER_KALMAN_PROC_NOISE`      | float | Kalman process noise scalar.                      |
| `TRACKER_KALMAN_MEAS_NOISE`      | float | Kalman measurement noise scalar.                  |
| `TRACKER_NEXT_ID_START`          | int   | Starting value for track ID counter.              |
| `LASER_MAX_DWELL_MS`             | int   | Max laser dwell time on a single point (ms).      |
| `LASER_SAFE_MODE`                | bool  | Require explicit arming before firing.            |
| `LASER_ARM_REQUIRED`             | bool  | Enforce arm-required policy.                      |
| `LASER_IDLE_BLANK`               | bool  | Send blank frame when no target is locked.        |
| `LASER_BURST_REPEAT`             | int   | Times to repeat a point in one frame.             |
| `LASER_DEFAULT_POWER_R/G/B`      | int   | Default RGB colour for targeting dot.             |
| `TARGETING_PREDICT_MS`           | int   | Ms ahead to predict mosquito position.            |
| `CALIBRATION_GRID_COLS`          | int   | Calibration grid columns.                         |
| `CALIBRATION_GRID_ROWS`          | int   | Calibration grid rows.                            |
| `CALIBRATION_LASER_R/G/B`        | int   | RGB colour of calibration laser dot.              |
| `CALIBRATION_FILE`               | Path  | Where calibration JSON is saved.                  |
| `GUI_WINDOW_TITLE`               | str   | Main window title bar text.                       |
| `GUI_DEFAULT_WIDTH/HEIGHT`       | int   | Default window size in pixels.                    |
| `GUI_CAMERA_FPS_DISPLAY`         | int   | Max rate (Hz) for camera frame refresh.           |
| `GUI_OVERLAY_BOX_COLOR`          | tuple | BGR for detection bounding boxes.                 |
| `GUI_OVERLAY_CROSS_COLOR`        | tuple | BGR for targeting crosshair.                      |
| `LOG_LEVEL`                      | str   | Default logging level (`"INFO"`).                 |
| `LOG_FILE`                       | Path  | Path to rotating log file.                        |
| `LOG_MAX_BYTES`                  | int   | Max log file size before rotation.                |
| `LOG_BACKUP_COUNT`               | int   | Number of rotated backup files.                   |

---

### `config/config_manager.py`

**Persistent user configuration** — JSON-backed override layer on top of `settings.py`.

#### Class `ConfigManager`

| Member                  | Kind   | Description                                                          |
| ----------------------- | ------ | -------------------------------------------------------------------- |
| `_config_path`          | Path   | Path to the JSON config file on disk.                                |
| `_overrides`            | dict   | User override values loaded from disk.                               |
| `_defaults`             | dict   | Default values extracted from `settings.py`.                         |
| `__init__(config_path)` | method | Loads config file; falls back gracefully if file does not exist.     |
| `load()`                | method | Read the JSON override file from disk. Returns the loaded dict.      |
| `save()`                | method | Write current overrides to disk as JSON.                             |
| `get(key)`              | method | Return value: user override if set, otherwise `settings.py` default. |
| `set(key, value)`       | method | Set a user override in memory and persist to disk.                   |
| `reset(key)`            | method | Remove a user override, reverting to `settings.py` default.          |
| `reset_all()`           | method | Clear all overrides and persist.                                     |
| `as_dict()`             | method | Return merged view of defaults + overrides.                          |

---

### `utils/logging_utils.py`

**Application-wide logging setup.**

| Symbol                                                    | Kind     | Description                                                                                                                                                                                  |
| --------------------------------------------------------- | -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `setup_logging(level, log_file, max_bytes, backup_count)` | function | Configures root logger with a `StreamHandler` (console) and a `RotatingFileHandler`. Must be called once before any other module creates loggers. All args have defaults from `settings.py`. |
| `get_logger(name)`                                        | function | Returns a named child logger. Used by every module: `logger = get_logger(__name__)`.                                                                                                         |

---

### `utils/safety.py`

**Central safety gatekeeper.** All laser firing must pass through this object.

#### `NoFireZone` (dataclass)

| Field     | Type | Description                                |
| --------- | ---- | ------------------------------------------ |
| `zone_id` | int  | Unique auto-assigned identifier.           |
| `x1, y1`  | int  | Top-left corner in camera pixel space.     |
| `x2, y2`  | int  | Bottom-right corner in camera pixel space. |
| `label`   | str  | Human-readable name shown in the UI.       |

#### Class `SafetySystem`

| Member                                    | Kind             | Description                                                                                                                                                         |
| ----------------------------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `_armed`                                  | bool             | Arm latch — true only when operator has explicitly armed.                                                                                                           |
| `_estop`                                  | bool             | E-stop latch — set by `emergency_stop()`; requires `disarm()` to reset.                                                                                             |
| `_zones`                                  | list[NoFireZone] | Registered no-fire zones.                                                                                                                                           |
| `_callbacks`                              | list[Callable]   | Zero-arg callbacks notified on any state change.                                                                                                                    |
| `_lock`                                   | threading.Lock   | Protects all state from concurrent access.                                                                                                                          |
| `_next_zone_id`                           | int              | Auto-increment counter for zone IDs.                                                                                                                                |
| `arm()`                                   | method           | Set `_armed = True`. Returns `False` if e-stop is active; `True` on success.                                                                                        |
| `disarm()`                                | method           | Clear `_armed` and reset `_estop` latch.                                                                                                                            |
| `emergency_stop()`                        | method           | Set `_estop = True` and `_armed = False`. Immediate.                                                                                                                |
| `reset_estop()`                           | method           | Clear `_estop` latch (does not re-arm).                                                                                                                             |
| `is_armed()`                              | method           | Return current arm state.                                                                                                                                           |
| `is_estop()`                              | method           | Return current e-stop latch state.                                                                                                                                  |
| `check_point(x, y)`                       | method           | Returns `True` if `(x, y)` is safe to fire (armed, not estop, outside all zones).                                                                                   |
| `add_no_fire_zone(x1, y1, x2, y2, label)` | method           | Register a new no-fire rectangle. Returns `zone_id` (int).                                                                                                          |
| `remove_no_fire_zone(zone_id)`            | method           | Remove zone by ID. Returns `True` if found and removed.                                                                                                             |
| `get_zones()`                             | method           | Return a copy of the current zone list.                                                                                                                             |
| `status_dict()`                           | method           | Return `{"armed": bool, "estop": bool, "zones": list}` snapshot.                                                                                                    |
| `register_callback(fn)`                   | method           | Register a **zero-argument** callable called on any state change (arm/disarm/estop/zone add/remove). Callbacks are invoked while holding the lock — keep them fast. |
| `_notify()`                               | method           | Internal — call all registered callbacks.                                                                                                                           |

---

### `camera/gopro_interface.py`

**GoPro Open API HTTP wrapper.** Handles camera control and stream management.

#### Class `GoProInterface`

| Member                       | Kind             | Description                                                                |
| ---------------------------- | ---------------- | -------------------------------------------------------------------------- |
| `_ip`                        | str              | GoPro AP IP address.                                                       |
| `_api_port`                  | int              | HTTP API port (default 8080).                                              |
| `_session`                   | requests.Session | Persistent HTTP session for keep-alive.                                    |
| `__init__(ip, api_port)`     | method           | Store connection params. Does not connect.                                 |
| `is_reachable()`             | method           | Quick HTTP GET to check if GoPro responds. Returns bool.                   |
| `enable_wired_usb_control()` | method           | PUT to enable USB/WiFi API control mode.                                   |
| `start_preview_stream()`     | method           | POST to start the UDP preview stream to host port `GOPRO_STREAM_UDP_PORT`. |
| `stop_preview_stream()`      | method           | POST to stop the UDP preview stream.                                       |
| `keep_alive()`               | method           | GET keep-alive endpoint to prevent GoPro Wi-Fi timeout.                    |
| `get_status()`               | method           | GET full camera status dict from GoPro API.                                |
| `close()`                    | method           | Close the persistent HTTP session.                                         |

---

### `camera/camera_manager.py`

**Frame capture QThread.** Wraps `GoProInterface` and `cv2.VideoCapture` in a background thread. Emits frames as Qt signals.

#### Class `CameraManager(QThread)`

| Member                    | Kind                   | Description                                                                     |
| ------------------------- | ---------------------- | ------------------------------------------------------------------------------- |
| `_source`                 | str or int             | VideoCapture source — UDP URL string or local device index.                     |
| `_gopro`                  | GoProInterface or None | Set when source is GoPro; None for local webcam.                                |
| `_running`                | bool                   | Loop control flag (stop via `stop()`).                                          |
| `frame_ready`             | Signal(np.ndarray)     | Emitted each captured frame (BGR array).                                        |
| `fps_updated`             | Signal(float)          | Emitted with rolling average FPS roughly every 30 frames.                       |
| `connection_changed`      | Signal(bool)           | Emitted when VideoCapture opens or closes.                                      |
| `error_occurred`          | Signal(str)            | Emitted on capture errors.                                                      |
| `set_source_gopro(ip)`    | method                 | Configure to capture from GoPro at `ip`. Starts preview stream.                 |
| `set_source_local(index)` | method                 | Configure to capture from local camera at device `index`.                       |
| `snapshot()`              | method                 | Return a single frame as `np.ndarray`, or `None` if not running. Thread-safe.   |
| `stop()`                  | method                 | Signal the capture loop to stop and `wait(5000)` for the thread to finish.      |
| `run()`                   | method (QThread)       | Internal capture loop — opens `VideoCapture`, emits frames, manages keep-alive. |

---

### `detection/detector.py`

**MOG2 / YOLO detection QThread.** Receives frames via a queue and emits detection results.

#### `DetectionMode` (Enum)

| Value                    | Description                                                              |
| ------------------------ | ------------------------------------------------------------------------ |
| `BACKGROUND_SUBTRACTION` | Use OpenCV MOG2 background model (default, no GPU required).             |
| `YOLO`                   | Use Ultralytics YOLO model (requires `ultralytics` package and weights). |

#### `Detection` (NamedTuple)

| Field        | Type  | Description                                                              |
| ------------ | ----- | ------------------------------------------------------------------------ |
| `cx, cy`     | float | Centroid of detected object in camera pixel space.                       |
| `w, h`       | float | Bounding box width and height in pixels.                                 |
| `area`       | float | Contour area in px².                                                     |
| `confidence` | float | Detection confidence (1.0 for MOG2 matches, YOLO model score otherwise). |

#### Class `MosquitoDetector(QThread)`

| Member                   | Kind                         | Description                                                                  |
| ------------------------ | ---------------------------- | ---------------------------------------------------------------------------- |
| `_queue`                 | Queue                        | Frame input buffer (bounded).                                                |
| `_mode`                  | DetectionMode                | Current detection mode.                                                      |
| `_sensitivity`           | float                        | Sensitivity scalar 0.0–1.0 (scales area thresholds).                         |
| `_running`               | bool                         | Loop control flag.                                                           |
| `_mog2`                  | cv2.BackgroundSubtractorMOG2 | Lazy-initialised background model.                                           |
| `_yolo`                  | YOLO or None                 | Lazy-loaded YOLO model (only when mode is YOLO).                             |
| `detections_ready`       | Signal(list)                 | Emitted with list of `Detection` each frame that has detections.             |
| `fps_updated`            | Signal(float)                | Emitted with rolling detection FPS.                                          |
| `push_frame(frame)`      | method                       | Enqueue a BGR `np.ndarray` for processing. Drops oldest if full.             |
| `set_mode(mode)`         | method                       | Switch between `BACKGROUND_SUBTRACTION` and `YOLO`. Resets background model. |
| `set_sensitivity(value)` | method                       | Set sensitivity 0.0–1.0; adjusts area/threshold parameters.                  |
| `reset_background()`     | method                       | Reinitialise the MOG2 background model (call after scene changes).           |
| `stop()`                 | method                       | Signal the detection loop to stop.                                           |
| `run()`                  | method (QThread)             | Internal detection loop.                                                     |

---

### `detection/tracker.py`

**Kalman + Hungarian multi-object tracker.**

#### `TrackedTarget`

| Member                           | Kind   | Description                                                                                    |
| -------------------------------- | ------ | ---------------------------------------------------------------------------------------------- |
| `track_id`                       | int    | Unique monotonically-increasing identifier.                                                    |
| `cx, cy`                         | float  | Last measured centroid in camera pixel space.                                                  |
| `vx, vy`                         | float  | Current velocity estimate (px/frame) from Kalman filter.                                       |
| `age`                            | int    | Frames this track has been continuously observed.                                              |
| `disappeared`                    | int    | Consecutive frames since last match.                                                           |
| `predicted_position(predict_ms)` | method | Return `(x, y)` extrapolated `predict_ms` milliseconds into the future using current velocity. |

#### Class `MosquitoTracker`

| Member               | Kind                     | Description                                                                                                                                                               |
| -------------------- | ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `_tracks`            | dict[int, TrackedTarget] | All live track objects keyed by ID.                                                                                                                                       |
| `_next_id`           | int                      | Next track ID to assign.                                                                                                                                                  |
| `update(detections)` | method                   | Run one tick: predict all Kalman states, run Hungarian assignment against new detections, create/update/age-out tracks. Returns `list[TrackedTarget]` of all live tracks. |
| `get_best_target()`  | method                   | Return the single highest-priority target (oldest confirmed track with `disappeared==0` and `age>=2`), or `None`.                                                         |
| `live_track_count()` | method                   | Return count of currently active tracks (not disappeared).                                                                                                                |
| `reset()`            | method                   | Clear all tracks.                                                                                                                                                         |

---

### `laser/lasercube_interface.py`

**Raw UDP socket driver for LaserCube hardware.**

#### Class `LaserCubeInterface`

| Member                              | Kind           | Description                                                                                                                     |
| ----------------------------------- | -------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `_ip`                               | str            | LaserCube AP IP.                                                                                                                |
| `_data_port`                        | int            | UDP data port (45456).                                                                                                          |
| `_cmd_port`                         | int            | UDP command port (45457).                                                                                                       |
| `_data_sock`                        | socket or None | UDP socket for sending point frames.                                                                                            |
| `_cmd_sock`                         | socket or None | UDP socket for control commands.                                                                                                |
| `_connected`                        | bool           | Connection state.                                                                                                               |
| `__init__(ip, data_port, cmd_port)` | method         | Store params. Does not open sockets.                                                                                            |
| `connect()`                         | method         | Open both UDP sockets and bind the command socket for responses. Returns bool.                                                  |
| `disconnect()`                      | method         | Close both sockets.                                                                                                             |
| `is_connected()`                    | method         | Return current connection state.                                                                                                |
| `send_frame(points)`                | method         | Pack and send a list of `(x, y, r, g, b)` points as a LaserCube UDP frame. Coordinates are signed int16 in range −32767…+32767. |
| `send_blank()`                      | method         | Send a single dark point frame to blank the laser output.                                                                       |
| `send_command(cmd_bytes)`           | method         | Send a raw command byte sequence to the cmd port.                                                                               |
| `heartbeat()`                       | method         | Send keep-alive command to prevent the hardware from timing out.                                                                |

**Wire format:** Each point is packed as `struct("<hhBBB")` — two signed 16-bit integers (x, y) followed by three unsigned bytes (r, g, b) = 7 bytes/point.

---

### `laser/laser_manager.py`

**Safety-gated laser targeting QThread.** Does not interact with hardware directly — delegates to `LaserCubeInterface`.

#### Class `LaserManager(QThread)`

| Member                                                 | Kind                       | Description                                                                                      |
| ------------------------------------------------------ | -------------------------- | ------------------------------------------------------------------------------------------------ |
| `_interface`                                           | LaserCubeInterface or None | Set via `set_laser_interface()`.                                                                 |
| `_safety`                                              | SafetySystem or None       | Set via `set_safety_system()`.                                                                   |
| `_queue`                                               | Queue                      | Bounded target job queue.                                                                        |
| `_running`                                             | bool                       | Loop control flag.                                                                               |
| `laser_fired`                                          | Signal(int, int)           | Emitted each time the laser fires with `(laser_x, laser_y)`.                                     |
| `connection_changed`                                   | Signal(bool)               | Emitted when laser connects/disconnects.                                                         |
| `error_occurred`                                       | Signal(str)                | Emitted on hardware or safety errors.                                                            |
| `output_changed`                                       | Signal(bool)               | Emitted when laser output state changes (on/off).                                                |
| `set_laser_interface(iface)`                           | method                     | Inject the `LaserCubeInterface` instance. Must be called before `connect_laser()`.               |
| `set_safety_system(safety)`                            | method                     | Inject the `SafetySystem` instance. Must be called before `arm()`.                               |
| `connect_laser()`                                      | method                     | Call `interface.connect()`. Returns bool.                                                        |
| `disconnect_laser()`                                   | method                     | Blank laser and call `interface.disconnect()`.                                                   |
| `arm()`                                                | method                     | Delegate to `SafetySystem.arm()`. Returns bool.                                                  |
| `disarm()`                                             | method                     | Delegate to `SafetySystem.disarm()`.                                                             |
| `emergency_stop()`                                     | method                     | Delegate to `SafetySystem.emergency_stop()` and blank laser immediately.                         |
| `push_target(laser_x, laser_y, cam_x, cam_y, r, g, b)` | method                     | Enqueue a targeting job. Default RGB values from `settings.py`.                                  |
| `stop()`                                               | method                     | Signal the loop to stop.                                                                         |
| `run()`                                                | method (QThread)           | Target delivery loop: dequeues jobs, checks `safety.check_point(cam_x, cam_y)`, fires or blanks. |

---

### `targeting/coordinate_mapper.py`

**Homography transform — camera pixels ↔ LaserCube galvo coordinates.**

#### Class `CoordinateMapper`

| Member                       | Kind               | Description                                                                                             |
| ---------------------------- | ------------------ | ------------------------------------------------------------------------------------------------------- |
| `_H`                         | np.ndarray or None | 3×3 homography matrix (camera → laser).                                                                 |
| `_H_inv`                     | np.ndarray or None | Inverse homography (laser → camera).                                                                    |
| `_frame_w, _frame_h`         | int                | Reference frame dimensions for normalisation.                                                           |
| `__init__(frame_w, frame_h)` | method             | Initialise with optional frame dimensions (default 1920×1080).                                          |
| `set_homography(H)`          | method             | Store a pre-computed homography matrix.                                                                 |
| `camera_to_laser(cx, cy)`    | method             | Map a camera pixel `(cx, cy)` to `(laser_x, laser_y)` as int. Returns `(0, 0)` if no homography loaded. |
| `laser_to_camera(lx, ly)`    | method             | Inverse map — laser coords → camera pixels. Returns `(0.0, 0.0)` if no inverse loaded.                  |
| `is_calibrated()`            | method             | Return `True` if a homography matrix has been loaded.                                                   |
| `save(path)`                 | method             | Serialise homography to JSON at `path`.                                                                 |
| `load(path)`                 | method             | Load homography from JSON. Returns bool success.                                                        |

---

### `targeting/calibration.py`

**Point-pair collection and homography computation.** Drives the laser to a grid of known points, observes the camera dot position, and computes the mapping.

#### Class `CalibrationController`

| Member                       | Kind               | Description                                                                                                                            |
| ---------------------------- | ------------------ | -------------------------------------------------------------------------------------------------------------------------------------- |
| `_mapper`                    | CoordinateMapper   | Receives the computed homography on `finish()`.                                                                                        |
| `_points`                    | list[tuple]        | Grid of laser coordinates to visit (col-major).                                                                                        |
| `_pairs`                     | list[tuple]        | Collected `(camera_pt, laser_pt)` pairs.                                                                                               |
| `_bg_frame`                  | np.ndarray or None | Background frame captured at `start()`.                                                                                                |
| `_frame_fn`                  | Callable           | Function to call to capture a frame (e.g. `CameraManager.snapshot`).                                                                   |
| `_laser_mgr`                 | LaserManager       | Used to fire calibration points.                                                                                                       |
| `_current_idx`               | int                | Index of the next point to visit.                                                                                                      |
| `start(frame_fn, laser_mgr)` | method             | Capture a background frame and reset state.                                                                                            |
| `next_point()`               | method             | **Blocking.** Fire laser at next grid point, wait ~150 ms, detect dot, record pair. Returns `(success: bool, message: str)`.           |
| `current_step()`             | method             | Return `(current_idx, total_steps)` — safe to call from any thread.                                                                    |
| `is_complete()`              | method             | Return `True` when `current_idx >= total_steps`.                                                                                       |
| `finish()`                   | method             | Compute homography from collected pairs via `cv2.findHomography`, push to `CoordinateMapper`, save to disk. Returns `True` on success. |
| `cancel()`                   | method             | Reset state without saving.                                                                                                            |

---

### `gui/camera_view.py`

**Central camera display widget.** Converts OpenCV BGR `np.ndarray` frames to `QPixmap` and renders all overlays (bounding boxes, tracks, crosshair, zones, FPS counter).

#### Class `CameraView(QWidget)`

| Member                    | Kind                  | Description                                                                                   |
| ------------------------- | --------------------- | --------------------------------------------------------------------------------------------- |
| `_pixmap`                 | QPixmap or None       | Most recent camera frame as QPixmap.                                                          |
| `_detections`             | list                  | Latest `Detection` objects for bounding box overlay.                                          |
| `_tracks`                 | list                  | Latest `TrackedTarget` objects for track path overlay.                                        |
| `_target`                 | TrackedTarget or None | Current best target for crosshair rendering.                                                  |
| `_zones`                  | list[NoFireZone]      | No-fire zones to render as translucent red rectangles.                                        |
| `_fps`                    | float                 | FPS text for HUD.                                                                             |
| `_show_fps`               | bool                  | Toggle FPS overlay (default True).                                                            |
| `_show_tracks`            | bool                  | Toggle track overlay (default True).                                                          |
| `_show_zones`             | bool                  | Toggle zone overlay (default True).                                                           |
| `pixel_clicked`           | Signal(float, float)  | Emitted with image-space `(x, y)` on mouse press. Corrects for letterboxing.                  |
| `update_frame(frame)`     | method                | Accept a BGR `np.ndarray`, convert to QPixmap, schedule repaint.                              |
| `update_detections(dets)` | method                | Store detection list; triggers repaint.                                                       |
| `update_tracks(tracks)`   | method                | Store track list; triggers repaint.                                                           |
| `set_target(target)`      | method                | Set the crosshair target (`TrackedTarget` or `None` to clear).                                |
| `set_fps(fps)`            | method                | Update FPS HUD value.                                                                         |
| `set_zones(zones)`        | method                | Update rendered no-fire zones.                                                                |
| `clear_overlays()`        | method                | Clear all overlay state.                                                                      |
| `set_show_fps(b)`         | method                | Show/hide FPS counter.                                                                        |
| `set_show_tracks(b)`      | method                | Show/hide track overlays.                                                                     |
| `set_show_zones(b)`       | method                | Show/hide no-fire zone overlays.                                                              |
| `paintEvent(event)`       | method                | Qt paint handler — scales pixmap to widget size with letterboxing, then renders all overlays. |
| `mousePressEvent(event)`  | method                | Convert click to image-space coords and emit `pixel_clicked`.                                 |

---

### `gui/control_panel.py`

**Right-dock control panel widget.** Groups all operator controls and system status indicators.

#### Class `StatusLight(QLabel)`

A small coloured circle indicator used to show connected/disconnected/armed/error state.

| Member               | Kind   | Description                                                        |
| -------------------- | ------ | ------------------------------------------------------------------ |
| `set_status(ok)`     | method | Set colour: green (`ok=True`) or red (`ok=False`).                 |
| `set_color(hex_str)` | method | Set colour to any hex string, e.g. `"#f9e2af"` for warning yellow. |

#### Class `ControlPanel(QWidget)`

All signals are emitted by user interaction; the main window slots handle all logic.

| Signal                       | Args | Description                                          |
| ---------------------------- | ---- | ---------------------------------------------------- |
| `gopro_connect_requested`    | —    | User clicked "Connect GoPro".                        |
| `gopro_disconnect_requested` | —    | User clicked "Disconnect GoPro".                     |
| `laser_connect_requested`    | —    | User clicked "Connect LaserCube".                    |
| `laser_disconnect_requested` | —    | User clicked "Disconnect LaserCube".                 |
| `arm_requested`              | —    | User clicked "ARM".                                  |
| `disarm_requested`           | —    | User clicked "DISARM".                               |
| `estop_requested`            | —    | User clicked "E-STOP".                               |
| `detection_mode_changed`     | str  | Mode combo changed; value is `"mog2"` or `"yolo"`.   |
| `sensitivity_changed`        | int  | Sensitivity slider moved; value is 0–100.            |
| `reset_background_requested` | —    | User clicked "Reset Background".                     |
| `calibrate_requested`        | —    | User clicked "Calibrate…".                           |
| `add_zone_requested`         | —    | User clicked "Add Zone".                             |
| `remove_zone_requested`      | int  | User clicked "Remove" on a zone; value is `zone_id`. |

| Method                                          | Description                                        |
| ----------------------------------------------- | -------------------------------------------------- |
| `set_gopro_status(connected)`                   | Update GoPro status light and button states.       |
| `set_laser_status(connected)`                   | Update laser status light and button states.       |
| `set_armed_state(armed, estop)`                 | Update ARM/DISARM button states and arm indicator. |
| `update_stats(cam_fps, det_fps, tracks, shots)` | Refresh the statistics group labels.               |
| `refresh_zones(zones)`                          | Rebuild the no-fire zone list widget.              |

---

### `gui/calibration_dialog.py`

**Step-by-step calibration wizard dialog.**

#### Class `_StepWorker(QThread)`

Internal helper that runs `CalibrationController.next_point()` off the UI thread.

| Member      | Kind              | Description                                                        |
| ----------- | ----------------- | ------------------------------------------------------------------ |
| `step_done` | Signal(bool, str) | Emitted when `next_point()` returns; carries `(success, message)`. |
| `run()`     | method            | Call `controller.next_point()` and emit `step_done`.               |

#### Class `CalibrationDialog(QDialog)`

| Member                       | Kind         | Description                                                                        |
| ---------------------------- | ------------ | ---------------------------------------------------------------------------------- |
| `calibration_complete`       | Signal(bool) | Emitted when the dialog closes; `True` = success, `False` = cancelled.             |
| `start(frame_fn, laser_mgr)` | method       | Call `controller.start(frame_fn, laser_mgr)` and show the first step UI.           |
| `_advance()`                 | method       | Spawn a `_StepWorker` to run the next calibration point.                           |
| `_on_step_done(ok, msg)`     | method       | Handle step result: update progress, log message, auto-advance or prompt user.     |
| `_run_finish()`              | method       | Call `controller.finish()`, emit `calibration_complete`, and accept/reject dialog. |

---

### `gui/main_window.py`

**Top-level QMainWindow.** Owns all subsystem lifetimes and wires all signals.

#### Class `MainWindow(QMainWindow)`

**Construction order:**

1. Create `SafetySystem`, `CoordinateMapper`, `MosquitoTracker`, `CalibrationController`.
2. Create `CameraManager`, `MosquitoDetector`, `LaserManager` (QThread workers).
3. Inject `SafetySystem` into `LaserManager` via `set_safety_system()`.
4. Build GUI: `CameraView` (central), `ControlPanel` (right dock), menu bar, status bar.
5. Wire all signals via `_wire_signals()`.
6. Register `_on_safety_state_changed` (zero-arg) with `SafetySystem.register_callback()`.

| Key Method                      | Description                                                                                                                           |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| `_wire_signals()`               | Connect all Qt signals from workers and the control panel to slots.                                                                   |
| `_start_threads()`              | Start detector and laser threads; camera starts only after "Connect GoPro".                                                           |
| `_stop_threads()`               | Call `.stop()` on all three QThread workers then `wait()`.                                                                            |
| `_on_frame_ready(frame)`        | Push frame to `CameraView.update_frame()` and `MosquitoDetector.push_frame()`.                                                        |
| `_on_detections_ready(dets)`    | Run tracker, update view, compute best target, push to laser via `push_target()`.                                                     |
| `_on_laser_fired(lx, ly)`       | Increment `_shots_fired` counter.                                                                                                     |
| `_on_gopro_connect_requested()` | Call `camera.set_source_gopro(ip)` and start thread.                                                                                  |
| `_on_laser_connect_requested()` | Create `LaserCubeInterface(ip, data, cmd)`, inject into laser manager, connect.                                                       |
| `_on_arm_requested()`           | Call `laser.arm()`; show warning dialog on failure.                                                                                   |
| `_on_estop_requested()`         | Call `laser.emergency_stop()`; pin status bar message.                                                                                |
| `_on_add_zone_requested()`      | Begin two-click zone-drawing flow (state: `_zone_first_pt`).                                                                          |
| `_on_camera_clicked(x, y)`      | On first click, store corner. On second click, call `safety.add_no_fire_zone()` and refresh UI.                                       |
| `_on_safety_state_changed()`    | **Zero-arg** safety callback. Reads `safety.status_dict()` and schedules a `QTimer.singleShot(0)` GUI update to ensure thread safety. |
| `_refresh_stats()`              | Called every 500 ms via `_stats_timer`; pushes FPS/tracks/shots to `ControlPanel`.                                                    |
| `closeEvent(event)`             | Fire e-stop, stop stats timer, stop all threads, accept close.                                                                        |

---

## Data Flow Diagram

```
GoPro Camera
    │  (UDP stream)
    ▼
CameraManager (QThread)
    │  frame_ready(np.ndarray) ──────────────────────────┐
    ▼                                                     │
MosquitoDetector (QThread)                          CameraView
    │  detections_ready(list[Detection])           (Qt widget)
    ▼
MosquitoTracker
    │  list[TrackedTarget]
    ▼
get_best_target()
    │  TrackedTarget
    ▼
predicted_position(predict_ms)
    │  (px, py)
    ▼
CoordinateMapper.camera_to_laser(px, py)
    │  (laser_x, laser_y)
    ▼
LaserManager.push_target(lx, ly, px, py)  ←── SafetySystem.check_point(px, py)
    │
    ▼
LaserCubeInterface.send_frame([(lx,ly,r,g,b)])
    │
    ▼
LaserCube Hardware
```

---

## Troubleshooting

| Symptom                           | Likely Cause                               | Fix                                                          |
| --------------------------------- | ------------------------------------------ | ------------------------------------------------------------ |
| "Connect GoPro" does nothing      | PC not on GoPro WiFi network               | Connect to GoPro AP in OS network settings                   |
| Camera feed shows black frame     | GoPro stream not started                   | Check `GOPRO_STREAM_UDP_PORT` in settings; restart GoPro     |
| "Connect LaserCube" fails         | PC not on LaserCube WiFi, or wrong IP/port | Check `LASERCUBE_DEFAULT_IP` in settings; verify AP SSID     |
| Laser status stays disconnected   | UDP sockets blocked by firewall            | Allow inbound/outbound UDP on ports 45456–45457              |
| "Cannot Arm" dialog every time    | E-stop latch is active                     | Click DISARM first to reset the latch, then ARM              |
| Detections but laser doesn't fire | System is disarmed                         | Click ARM in the Control Panel                               |
| Laser fires in wrong direction    | Calibration is off                         | Re-run calibration (Tools → Calibrate…)                      |
| No detections at all              | Background model too fresh                 | Wait 10–15 seconds for MOG2 to stabilise, or use YOLO mode   |
| YOLO mode shows import error      | `ultralytics` not installed                | `pip install ultralytics`                                    |
| Log file not written              | `CONFIG_DIR` not writable                  | Check `~/.iron_dome/` directory permissions                  |
| Calibration fails at every point  | Laser dot not visible to camera            | Ensure laser dot falls in camera frame; dim room lights help |
