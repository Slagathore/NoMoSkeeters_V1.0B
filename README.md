# NoMoSkeeters 🦟⚡

**NoMoSkeeters** is a real-time mosquito detection and laser neutralisation system built in Python. It uses a GoPro camera, computer vision, and a LaserCube RGB galvo projector to detect and engage mosquitoes autonomously — with comprehensive safety controls.

> ⚠️ **Safety Warning**: This system uses a high-powered laser. **Always** follow laser safety guidelines, configure no-fire zones, and never leave the armed system unattended. Eye exposure is permanently injurious. Comply with all local laser safety and emissions regulations.

---

## Features

- **Real-time detection** via MOG2 background subtraction or YOLO (optional)
- **Multi-object Kalman filter tracking** with Hungarian algorithm assignment
- **GoPro WiFi API** integration for live camera feed
- **LaserCube galvo projector** control via UDP protocol
- **Homography-based coordinate mapping** from camera pixels → laser coordinates
- **Calibration wizard** — automated multi-point grid calibration
- **No-fire zone management** — draw exclusion zones on the camera view
- **Safety interlocks** — arm/disarm/emergency-stop with callback hooks
- **Catppuccin-themed PySide6 GUI** with live overlays (tracks, FPS, zones, crosshair)
- **Persistent configuration** — JSON-backed override layer on top of defaults

---

## Hardware Requirements

| Component | Details |
|-----------|---------|
| **Camera** | GoPro (any model supporting GoPro Open API WiFi) or USB webcam |
| **Laser projector** | LaserCube by Wicked Lasers (192.168.1.1, UDP ports 45456/45457) |
| **Host PC** | Windows 10/11, Python 3.10+, Wi-Fi NIC |

### Network Setup

- GoPro creates a Wi-Fi AP at `10.5.5.9:8080` (HTTP) and streams RTSP at `udp://10.5.5.9:8554`
- LaserCube connects at `192.168.1.1` (UDP data: 45456, commands: 45457)

---

## Software Requirements

Python 3.10 or newer is required (uses `X | Y` union syntax and `match` statements).

```bash
pip install -r requirements.txt
```

| Package | Purpose |
|---------|---------|
| `PySide6 >= 6.5.0` | Qt6 GUI framework |
| `opencv-python >= 4.8.0` | Camera capture and image processing |
| `numpy >= 1.24.0` | Array math and Kalman filter matrices |
| `requests >= 2.31.0` | GoPro Open API HTTP client |
| `scipy >= 1.11.0` | Hungarian algorithm for tracker assignment |
| `ultralytics >= 8.0.0` | Optional YOLO detection (falls back to BGSub if absent) |
| `filterpy >= 1.4.5` | Optional dedicated Kalman library (falls back to NumPy impl) |

---

## Installation

```bash
git clone https://github.com/<your-username>/NoMoSkeeters_V1.0B.git
cd NoMoSkeeters_V1.0B
python -m venv .venv
.venv\Scripts\activate        # Windows
pip install -r requirements.txt
python main.py
```

### CLI Options

```
python main.py [--log-level {DEBUG,INFO,WARNING,ERROR}] [--config <path>]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--log-level` | `INFO` | Console and file log verbosity |
| `--config` | `user_data/config.json` | Path to JSON config override file |

---

## Usage

### 1. Connect GoPro
Click **Connect GoPro** in the Control Panel. The system connects via the GoPro WiFi AP and starts the UDP preview stream.

### 2. Connect LaserCube
Click **Connect Laser**. The system sends a handshake to the LaserCube and enables output.

### 3. Calibrate (first run)
Go to **Tools → Calibrate Laser**. The calibration wizard will:
1. Fire the laser at a grid of points
2. Detect the laser dot in the camera feed via background subtraction
3. Compute a perspective homography matrix
4. Save the calibration to `user_data/calibration.json`

Calibration is loaded automatically on subsequent launches.

### 4. Configure No-Fire Zones
Click **Add Zone** in the Control Panel, then click two corners on the camera view. Any target inside a no-fire zone will not be engaged.

### 5. Arm and Engage
Click **ARM** to enable laser firing. The system will automatically track detected mosquitoes and fire the laser at the predicted intercept position.

Use **Emergency Stop** or **Disarm** at any time to halt laser output.

---

## Project Structure

```
NoMoSkeeters_V1.0B/
├── main.py                  # Entry point — CLI args, QApplication, MainWindow
├── requirements.txt
├── camera/
│   ├── camera_manager.py    # QThread capture loop (GoPro UDP / local webcam)
│   └── gopro_interface.py   # GoPro Open API HTTP client
├── config/
│   ├── settings.py          # All application constants (no magic numbers)
│   └── config_manager.py    # JSON-backed persistent config override layer
├── detection/
│   ├── detector.py          # QThread detection worker (MOG2 / YOLO)
│   └── tracker.py           # Kalman filter + Hungarian multi-object tracker
├── gui/
│   ├── main_window.py       # Top-level QMainWindow wiring all subsystems
│   ├── control_panel.py     # Dock widget with all controls
│   ├── camera_view.py       # Live camera feed widget with overlays
│   └── calibration_dialog.py# Step-by-step calibration wizard QDialog
├── laser/
│   ├── laser_manager.py     # QThread laser controller with safety-gated loop
│   └── lasercube_interface.py# Low-level UDP socket driver for LaserCube hardware
├── targeting/
│   ├── calibration.py       # Multi-point calibration procedure orchestrator
│   └── coordinate_mapper.py # Perspective homography camera↔laser coordinate mapper
├── utils/
│   ├── logging_utils.py     # Rotating file + coloured console log setup
│   └── safety.py            # Safety gatekeeper — arm/disarm/e-stop/no-fire zones
└── docs/
    └── MODULE_DOCS.md       # Comprehensive per-module/class/function documentation
```

---

## Architecture

The system is a multi-QThread pipeline connected via Qt signals/slots:

```
CameraManager (QThread)
       │ frame_ready(ndarray)
       ▼
MosquitoDetector (QThread)
       │ detections_ready(list[Detection])
       ▼
MosquitoTracker (in-thread, sync)
       │ TrackedTarget list
       ▼
CoordinateMapper → LaserManager (QThread)
                         │
                   LaserCubeInterface (UDP)
```

All GUI updates are marshalled to the main thread via `QTimer.singleShot(0, ...)`.

---

## Configuration

All constants live in `config/settings.py`. User overrides are persisted to `user_data/config.json` via `ConfigManager`. Key tuning parameters:

| Constant | Default | Description |
|----------|---------|-------------|
| `DETECTION_MOG2_HISTORY` | 200 | MOG2 background model history length |
| `DETECTION_MOG2_VARTH` | 30 | MOG2 variance threshold |
| `TRACKER_MAX_DISAPPEARED` | 10 | Frames before a track is pruned |
| `TRACKER_MAX_DISTANCE` | 80 | Max pixel distance for assignment |
| `LASER_MAX_DWELL_MS` | 500 | Max time to fire at a single target (ms) |
| `TARGETING_PREDICT_MS` | 50 | Milliseconds ahead to predict target position |
| `CALIBRATION_GRID_COLS` | 4 | Calibration grid columns |
| `CALIBRATION_GRID_ROWS` | 4 | Calibration grid rows |

---

## Safety System

The `SafetySystem` class (`utils/safety.py`) enforces:
- **Arm/disarm** with explicit user action required
- **Emergency stop** — immediately blanks the laser and blocks re-arm
- **No-fire zones** — rectangular exclusion zones in camera pixel space; any target inside is not engaged
- **Thread-safe callbacks** — registered listeners notified on any state change

---

## Contributing

This project is a personal/research system. Issues and pull requests are welcome. When contributing, please:
- Keep all laser safety interlocks intact
- Follow the existing code style (PySide6 Qt patterns, type annotations, docstrings)
- Run `python -m py_compile` on all modified files before submitting

---

## License

MIT License — see [LICENSE](LICENSE) for details.

---

## Disclaimer

This system is for **research and recreational use only**. The authors are not responsible for any injury, property damage, or legal violations arising from use of this software or the associated hardware. Always operate within applicable laws and laser safety regulations.
