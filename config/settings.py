"""
Iron Dome Anti-Mosquito System
================================
Mission: Automatically detect and neutralize mosquitos in real-time using a
         GoPro camera for vision and a LaserCube for precision targeting.

Settings Module
---------------
Central repository for ALL application constants and default configuration values.
Every tunable parameter lives here so that no magic numbers are scattered through
the codebase.  Users override values via ConfigManager (config_manager.py) which
writes overrides to a JSON file — these settings represent the hard defaults.

Variables:
  PROJECT_NAME             — Human-readable application name string.
  VERSION                  — Semantic version string (major.minor.patch).
  BASE_DIR                 — pathlib.Path to the project root directory.
  CONFIG_DIR               — pathlib.Path to directory where user config is stored.
  CONFIG_FILE              — pathlib.Path to the user config JSON file.

  GOPRO_DEFAULT_IP         — IPv4 address of GoPro when in AP (hotspot) mode.
  GOPRO_API_PORT           — HTTP port for GoPro Open API.
  GOPRO_STREAM_UDP_PORT    — UDP port on host that GoPro preview stream is sent to.
  GOPRO_TIMEOUT_S          — Seconds before an HTTP request to GoPro times out.
  GOPRO_PREVIEW_STREAM_URL — OpenCV-compatible URL for the GoPro preview UDP stream.
  GOPRO_KEEP_ALIVE_INTERVAL_S — Seconds between keep-alive pings to GoPro.

  LASERCUBE_DEFAULT_IP     — IPv4 address of LaserCube WiFi AP.
  LASERCUBE_DATA_PORT      — UDP port for streaming point frame data to LaserCube.
  LASERCUBE_CMD_PORT       — UDP port for control commands to/from LaserCube.
  LASERCUBE_SCAN_RATE      — Galvo scan rate in points-per-second.
  LASERCUBE_MAX_POINTS_PER_FRAME — Maximum points in a single UDP frame packet.
  LASERCUBE_COORD_MIN      — Minimum signed-integer coordinate value (left/bottom).
  LASERCUBE_COORD_MAX      — Maximum signed-integer coordinate value (right/top).
  LASERCUBE_HEARTBEAT_INTERVAL_S — Seconds between laser keep-alive heartbeats.

  DETECTION_MIN_AREA       — Minimum contour area (px²) to consider as mosquito.
  DETECTION_MAX_AREA       — Maximum contour area (px²) to consider as mosquito.
  DETECTION_MIN_ASPECT     — Minimum bounding-box aspect ratio for candidate blobs.
  DETECTION_MAX_ASPECT     — Maximum bounding-box aspect ratio for candidate blobs.
  DETECTION_BLUR_KERNEL    — Gaussian blur kernel size applied before subtraction.
  DETECTION_THRESHOLD      — Binary threshold value after subtraction.
  DETECTION_MOG2_HISTORY   — Number of frames used to build background model.
  DETECTION_MOG2_VARTH     — Variance threshold for MOG2 foreground/background split.
  DETECTION_FPS_TARGET     — Target processing frame-rate for the detector thread.
  DETECTION_YOLO_CONF      — Confidence threshold when running YOLO (0–1).
  DETECTION_YOLO_MODEL     — Path or model name for YOLO weights file.
  DETECTION_MORPH_KERNEL   — Morphological open/close kernel size for noise removal.

  TRACKER_MAX_DISAPPEARED  — Frames a track can go unmatched before being dropped.
  TRACKER_MAX_DISTANCE     — Max pixel distance for Hungarian assignment match.
  TRACKER_KALMAN_PROC_NOISE  — Kalman filter process noise covariance scalar.
  TRACKER_KALMAN_MEAS_NOISE  — Kalman filter measurement noise covariance scalar.
  TRACKER_NEXT_ID_START    — Initial track ID counter value.

  LASER_MAX_DWELL_MS       — Maximum milliseconds laser can dwell on a single point.
  LASER_SAFE_MODE          — If True, laser requires explicit arming before firing.
  LASER_ARM_REQUIRED       — If True, user must press Arm button before any firing.
  LASER_IDLE_BLANK         — If True, send a dark/blank frame when no target locked.
  LASER_BURST_REPEAT       — Number of times to repeat a target point in one frame.
  LASER_DEFAULT_POWER_R    — Default red channel value (0–255) for target dot.
  LASER_DEFAULT_POWER_G    — Default green channel value (0–255) for target dot.
  LASER_DEFAULT_POWER_B    — Default blue channel value (0–255) for target dot.

  TARGETING_PREDICT_MS     — Milliseconds ahead to predict mosquito position.
  CALIBRATION_GRID_COLS    — Number of calibration grid columns.
  CALIBRATION_GRID_ROWS    — Number of calibration grid rows.
  CALIBRATION_LASER_R      — Red value of calibration laser dot.
  CALIBRATION_LASER_G      — Green value of calibration laser dot.
  CALIBRATION_LASER_B      — Blue value of calibration laser dot.
  CALIBRATION_FILE         — pathlib.Path where calibration data is saved.

  GUI_WINDOW_TITLE         — Main window title bar text.
  GUI_DEFAULT_WIDTH        — Default main window width in pixels.
  GUI_DEFAULT_HEIGHT       — Default main window height in pixels.
  GUI_CAMERA_FPS_DISPLAY   — Maximum rate (Hz) at which GUI refreshes the camera frame.
  GUI_OVERLAY_BOX_COLOR    — BGR tuple for detection bounding box overlay.
  GUI_OVERLAY_CROSS_COLOR  — BGR tuple for targeting crosshair overlay.

  LOG_LEVEL                — Default logging level string (e.g. "INFO").
  LOG_FILE                 — pathlib.Path for rotating file log output.
  LOG_MAX_BYTES            — Maximum log file size in bytes before rotation.
  LOG_BACKUP_COUNT         — Number of rotated log file backups to keep.
"""

from pathlib import Path

# ─── Project ───────────────────────────────────────────────────────────────────
PROJECT_NAME = "Iron Dome Anti-Mosquito System"
VERSION = "0.1.0"
BASE_DIR: Path = Path(__file__).parent.parent  # project root
CONFIG_DIR: Path = BASE_DIR / "user_data"
CONFIG_FILE: Path = CONFIG_DIR / "config.json"

# ─── GoPro ─────────────────────────────────────────────────────────────────────
# GoPro connects over WiFi in AP mode; its default IP is always 10.5.5.9
GOPRO_DEFAULT_IP: str = "10.5.5.9"
GOPRO_API_PORT: int = 8080
GOPRO_STREAM_UDP_PORT: int = 8554          # Port on THIS machine GoPro streams to
GOPRO_TIMEOUT_S: float = 5.0
GOPRO_PREVIEW_STREAM_URL: str = "udp://0.0.0.0:8554"  # OpenCV capture URL
GOPRO_KEEP_ALIVE_INTERVAL_S: float = 2.5  # GoPro disconnects after ~5s without ping

# ─── LaserCube ─────────────────────────────────────────────────────────────────
# LaserCube WiFi AP: SSID "LaserCube-XXXXXX", IP 192.168.1.1
LASERCUBE_DEFAULT_IP: str = "192.168.1.1"
LASERCUBE_DATA_PORT: int = 45456          # UDP port for streaming point frames
LASERCUBE_CMD_PORT: int = 45457           # UDP port for commands / status
LASERCUBE_SCAN_RATE: int = 30_000         # Galvo points per second (max)
LASERCUBE_MAX_POINTS_PER_FRAME: int = 1_000
LASERCUBE_COORD_MIN: int = -32767         # Leftmost / Bottom-most laser position
LASERCUBE_COORD_MAX: int = 32767          # Rightmost / Top-most laser position
LASERCUBE_HEARTBEAT_INTERVAL_S: float = 0.5

# LaserCube command byte constants (based on LaserOS SDK protocol)
LC_CMD_GET_ALIVE: int = 0x40
LC_CMD_SET_ILDA_RATE: int = 0x41
LC_CMD_GET_ILDA_RATE: int = 0x42
LC_CMD_GET_RINGBUFFER_EMPTY: int = 0x43
LC_CMD_GET_LAST_STATUS: int = 0x44
LC_CMD_SET_OUTPUT: int = 0x45
LC_CMD_WRITE_DAC: int = 0x46

# ─── Detection ─────────────────────────────────────────────────────────────────
# These are tuned for a mosquito (wingspan ~10mm) at 1–3m distance
# with a GoPro at 1080p resolution.
DETECTION_MIN_AREA: int = 5        # px² — smaller than this = noise
DETECTION_MAX_AREA: int = 800      # px² — larger than this = not a mosquito
DETECTION_MIN_ASPECT: float = 0.15 # bounding box w/h ratio floor
DETECTION_MAX_ASPECT: float = 8.0  # bounding box w/h ratio ceiling
DETECTION_BLUR_KERNEL: int = 5     # must be odd
DETECTION_THRESHOLD: int = 20      # foreground/background binary split
DETECTION_MOG2_HISTORY: int = 300  # frames used to build background model
DETECTION_MOG2_VARTH: float = 16.0 # lower = more sensitive
DETECTION_FPS_TARGET: int = 30     # detection thread target FPS
DETECTION_YOLO_CONF: float = 0.45  # YOLO confidence threshold (0–1)
DETECTION_YOLO_MODEL: str = "yolov8n.pt"  # default lightweight YOLO weights
DETECTION_MORPH_KERNEL: int = 3    # morphological cleanup kernel size

# ─── Tracking ──────────────────────────────────────────────────────────────────
TRACKER_MAX_DISAPPEARED: int = 8   # frames before a track is considered lost
TRACKER_MAX_DISTANCE: float = 80.0 # px — max dist for Hungarian assignment
TRACKER_KALMAN_PROC_NOISE: float = 0.1   # Q matrix scalar
TRACKER_KALMAN_MEAS_NOISE: float = 2.0   # R matrix scalar
TRACKER_NEXT_ID_START: int = 1

# ─── Laser Safety ──────────────────────────────────────────────────────────────
# SAFETY WARNING: A laser can cause serious eye injury or fire.
# Never point at people, animals, or flammable materials.
LASER_MAX_DWELL_MS: int = 80          # Hard cap on dwell time per target point
LASER_SAFE_MODE: bool = True          # Require explicit Arm before firing
LASER_ARM_REQUIRED: bool = True       # GUI Arm button must be pressed
LASER_IDLE_BLANK: bool = True         # Send blank frame when not targeting
LASER_BURST_REPEAT: int = 100         # Repeat target point N times in one frame
LASER_DEFAULT_POWER_R: int = 0        # Red channel for target dot (0–255)
LASER_DEFAULT_POWER_G: int = 255      # Green channel for target dot
LASER_DEFAULT_POWER_B: int = 0        # Blue channel for target dot

# ─── Targeting ─────────────────────────────────────────────────────────────────
TARGETING_PREDICT_MS: int = 50        # Predict position this far ahead (ms)

# ─── Calibration ───────────────────────────────────────────────────────────────
CALIBRATION_GRID_COLS: int = 3
CALIBRATION_GRID_ROWS: int = 3
CALIBRATION_LASER_R: int = 255        # Bright white-ish for calibration dot
CALIBRATION_LASER_G: int = 255
CALIBRATION_LASER_B: int = 255
CALIBRATION_FILE: Path = CONFIG_DIR / "calibration.json"
CALIBRATION_DOT_DETECT_THRESHOLD: int = 200  # Grayscale threshold for dot detection

# ─── GUI ───────────────────────────────────────────────────────────────────────
GUI_WINDOW_TITLE: str = f"{PROJECT_NAME} v{VERSION}"
GUI_DEFAULT_WIDTH: int = 1280
GUI_DEFAULT_HEIGHT: int = 820
GUI_CAMERA_FPS_DISPLAY: int = 30       # Max Hz to repaint the camera feed widget
GUI_OVERLAY_BOX_COLOR: tuple = (0, 255, 0)    # BGR green bounding boxes
GUI_OVERLAY_CROSS_COLOR: tuple = (0, 0, 255)  # BGR red crosshair
GUI_OVERLAY_PRED_COLOR: tuple = (255, 165, 0) # BGR orange predicted position
GUI_ZONE_COLOR: tuple = (0, 0, 180)           # BGR dark-red no-fire zones

# ─── Logging ───────────────────────────────────────────────────────────────────
LOG_LEVEL: str = "INFO"
LOG_FILE: Path = BASE_DIR / "logs" / "iron_dome.log"
LOG_MAX_BYTES: int = 10 * 1024 * 1024  # 10 MB per log file
LOG_BACKUP_COUNT: int = 5
