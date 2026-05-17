"""Single source of truth for every tunable in the system.

Default values are constants at module level. User-supplied JSON overrides
(via ConfigManager) overlay on top. No magic numbers anywhere else in the
codebase — if you find one, lift it here before doing anything else.

Runtime-mutability convention: by default, settings are runtime-mutable.
Settings listed in `RESTART_REQUIRED` (bottom of this file) need an app
restart to take effect.

Sections mirror BOOTSTRAP.md §§5-15 + BOOTSTRAP_AMENDMENTS §§5-9.
"""
from __future__ import annotations

from pathlib import Path


# ── Paths ────────────────────────────────────────────────────────────────
BASE_DIR: Path = Path(__file__).resolve().parent.parent
USER_DATA_DIR: Path = BASE_DIR / "user_data"
CALIBRATIONS_DIR: Path = USER_DATA_DIR / "calibrations"
SESSIONS_DIR: Path = USER_DATA_DIR / "sessions"
MODELS_DIR: Path = BASE_DIR / "models"
LOGS_DIR: Path = BASE_DIR / "logs"
USER_CONFIG_PATH: Path = USER_DATA_DIR / "config.json"


# ── §3.3 / §5 — Sensor roles + targeting mode (v0.2.1) ───────────────────
TARGETING_MODE: str = "gopro_only"   # "gopro_only" | "kinect_only" | "fused_interp"

SENSOR_ROLES: dict = {
    "gopro":     "auto",
    "kinect_v2": "auto",
    "local_cam": "auto",
}

SENSOR_STREAM_ROLES: dict = {
    "gopro":        "targeting",
    "kinect_rgb":   "targeting",
    "kinect_ir":    "targeting",
    "kinect_depth": "world_attribution",
    "local_cam":    "off",
}


# ── §5.7 (v0.2.1) — Cross-sensor fusion ──────────────────────────────────
FUSION_MAX_ASSOC_DISTANCE_NORM: float = 0.05
FUSION_HISTORY_LENGTH: int = 8
FUSION_MIN_VELOCITY_SAMPLES: int = 3
FUSION_PROJECTION_MAX_DT_MS: float = 100.0


# ── §6 — Detection ───────────────────────────────────────────────────────
DETECTION_BLUR_KERNEL: int = 3
DETECTION_THRESHOLD: int = 20
DETECTION_MORPH_KERNEL: int = 3
DETECTION_MIN_AREA: int = 5
DETECTION_MAX_AREA: int = 800
DETECTION_MIN_ASPECT: float = 0.15
DETECTION_MAX_ASPECT: float = 8.0


# ── §6.2 — Classifier ────────────────────────────────────────────────────
CLASSIFIER_ENABLED: bool = True
CLASSIFIER_FAIL_MODE: str = "open"   # "open" | "closed" | "neutral"
CLASSIFIER_MODEL_PATH: str = "models/classifier.pkl"

CLASSIFIER_HEURISTIC_DEFAULTS: dict = {
    "min_solidity": 0.30,
    "min_aspect":   0.20,
    "max_aspect":   5.00,
    "min_area_px":  20,
    "max_area_px":  5000,
}

# Per-feature toggles (BOOTSTRAP_AMENDMENTS §6.2 — Hu moments off by default).
CLASSIFIER_USE_HU_MOMENTS: bool = False
CLASSIFIER_USE_AREA_RATIO: bool = True
CLASSIFIER_USE_ASPECT:     bool = True
CLASSIFIER_USE_SOLIDITY:   bool = True
CLASSIFIER_USE_EXTENT:     bool = True
CLASSIFIER_USE_EQUIV_DIAM: bool = True
CLASSIFIER_USE_PERIM_SQ:   bool = True


# ── §7 — Tracking ────────────────────────────────────────────────────────
TRACKER_MODE: str = "kalman_norm_dt"   # kalman_pixel | kalman_norm_dt | kalman_3d | iou_only
TRACKER_MAX_DISAPPEARED: int = 8
TRACKER_MAX_COAST_FRAMES: int = 30
TRACKER_MAX_DISTANCE_NORM: float = 0.06
# Kalman noise — variances in NORM space (0..1 coords); the tracker tracks
# in norm space for every mode. Both were pixel-scaled (0.1 / 2.0), which
# made R ≫ P and over-smoothed every track. Norm-space derivation:
#   PROC : unmodeled mosquito accel per step, ~0.003 norm → variance ~1e-5
#   MEAS : detection-centroid jitter, ~2-3 px on 1080p → ~0.0015 norm → ~2e-6
# Starting points — verify against live tracker behaviour during Step 11.
TRACKER_KALMAN_PROC_NOISE: float = 1e-5
TRACKER_KALMAN_MEAS_NOISE: float = 2e-6
TRACKER_CONFIRMATION_FRAMES: int = 3
TRACKER_CONFIDENCE_DECAY: float = 0.9
TRACKER_CONFIDENCE_BOOST: float = 1.05
TRACKER_HISTORY_LENGTH: int = 64

# Fire eligibility (computed signal, not a hidden gate — see amendment §7).
FIRE_ELIGIBILITY_MAX_AGE_MS: float = 50.0
FIRE_ELIGIBILITY_MIN_CONFIDENCE: float = 0.7


# ── §8 — Calibration ─────────────────────────────────────────────────────
CALIBRATION_PATTERN: str = "halton"   # "grid" | "halton" | "dragline" | "windmill"
CALIBRATION_POINTS: int = 25
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

# Multi-depth validation (amendment §8.6).
CALIBRATION_MAX_RESIDUAL_NORM: float = 0.015

# Operator-facing validation depths. US tape measures read feet, so depths
# are specified in CALIBRATION_DEPTH_UNITS ("ft" or "m"). The pipeline works
# in metres internally — CALIBRATION_VALIDATION_DEPTHS_M below is DERIVED from
# these; edit CALIBRATION_VALIDATION_DEPTHS, not the _M list.
FT_TO_M: float = 0.3048
CALIBRATION_DEPTH_UNITS: str = "ft"                       # "ft" | "m"
CALIBRATION_VALIDATION_DEPTHS: list = [3.0, 8.0, 13.0]    # in CALIBRATION_DEPTH_UNITS
CALIBRATION_VALIDATION_DEPTHS_M: list = [
    round(d * FT_TO_M, 4) if CALIBRATION_DEPTH_UNITS == "ft" else float(d)
    for d in CALIBRATION_VALIDATION_DEPTHS
]
CALIBRATION_DRAGLINE_MULTI_SPEED: bool = True
CALIBRATION_DRAGLINE_SPEEDS: list = [0.05, 0.20, 0.50, 1.00]
LATENCY_SOFTWARE_LAG_MS: float = 0.0   # populated by drag-line calibration

# Kinect calibration (v0.2.1 §8.10).
KINECT_CALIBRATION_REQUIRED_FOR_TARGETING: bool = True
KINECT_CALIBRATION_PROMPT_ON_STARTUP: bool = True
KINECT_CALIBRATION_PER_STREAM: bool = False
KINECT_CALIBRATION_REUSE_TOLERANCE_HOURS: float = 6.0


# ── §9 — LaserCube protocol + network (verified hardware values) ─────────
LASERCUBE_DEFAULT_IP: str = "169.254.40.83"
LASERCUBE_BIND_SRC_IP: str = "auto"   # "auto" | "0.0.0.0" | explicit "169.254.25.216"
NETWORK_PROFILE: str = "auto"          # "auto" | "lan_client" | "wifi_server" | "manual"

LASERCUBE_CMD_PORT: int = 45457
LASERCUBE_DATA_PORT: int = 45458
LASERCUBE_ALIVE_PORT: int = 45456

LASERCUBE_BUFFER_SIZE: int = 6000      # ringbuffer total samples
LASERCUBE_TARGET_BUFFER_FREE: int = 5000
LASERCUBE_DEFAULT_DAC_RATE: int = 30000
LASERCUBE_MAX_DAC_RATE: int = 30000
LASERCUBE_REPLY_TIMEOUT_S: float = 1.5
LASERCUBE_HEARTBEAT_INTERVAL_S: float = 1.5   # cube comms timer is 4s
LASERCUBE_MAX_SAMPLES_PER_PACKET: int = 140

# Shot patterns (amendment §9.11).
SHOT_PATTERN_DEFAULT: str = "micro_circle"   # "dot_repeat" | "micro_circle" | "figure_eight"
SHOT_PATTERN_DWELL_MS: int = 80
SHOT_PATTERN_POWER_PCT: int = 100
SHOT_PATTERN_CIRCLE_RADIUS_GALVO: int = 30
SHOT_PATTERN_FIGURE8_SCALE_GALVO: int = 40


# ── §10 — Safety ─────────────────────────────────────────────────────────
SAFETY_ARM_REQUIRED: bool = True
SAFETY_ARM_STICKY: bool = True
SAFETY_ESTOP_LATCH: bool = True
SAFETY_NO_FIRE_ZONES_ENABLED: bool = True
SAFETY_DWELL_LIMIT_ENABLED: bool = True
SAFETY_DWELL_LIMIT_MS: int = 80
SAFETY_IDLE_BLANK_ENABLED: bool = True
SAFETY_MANUAL_FIRE_ALLOWED: bool = True
SAFETY_MAX_POWER_PCT: int = 100
SAFETY_DEV_MODE: bool = False
SAFETY_DRY_FIRE_MODE: bool = False
SAFETY_OBJECT_SIZE_GUARD_ENABLED: bool = True
SAFETY_MAX_TARGET_AREA_MM2: float = 10000.0   # ~10cm × 10cm


# ── §12 — Web monitor ────────────────────────────────────────────────────
WEB_MONITOR_ENABLED: bool = False
WEB_MONITOR_BIND_HOST: str = "127.0.0.1"
WEB_MONITOR_PORT: int = 8765
WEB_MONITOR_MJPEG_FPS: int = 10
WEB_MONITOR_MJPEG_QUALITY: int = 70
WEB_MONITOR_EVENT_LOG_SIZE: int = 1000
WEB_MONITOR_REQUIRE_TOKEN: bool = False
WEB_MONITOR_TOKEN: str = ""


# ── §14 — Logging + session recording ────────────────────────────────────
LOG_LEVEL: str = "INFO"
SESSION_RECORDING_ENABLED: bool = True
SESSION_RECORDING_DIR: Path = SESSIONS_DIR
SESSION_RECORDING_MAX_FILES: int = 100
SESSION_RECORDING_MAX_TOTAL_MB: int = 1000


# ── §16.4 (v0.2.1) — Sensor comparison harness ───────────────────────────
COMPARE_TOOL_ASSOC_DT_MS: float = 100.0
COMPARE_TOOL_ASSOC_DISTANCE_NORM: float = 0.05
COMPARE_TOOL_OUTPUT_DIR: Path = BASE_DIR / "reports"


# ── Restart-required settings (§13.3) ────────────────────────────────────
# Anything not in this set is runtime-mutable.
RESTART_REQUIRED: set = {
    "LASERCUBE_DEFAULT_IP",
    "LASERCUBE_BIND_SRC_IP",
    "LASERCUBE_CMD_PORT",
    "LASERCUBE_DATA_PORT",
    "LASERCUBE_ALIVE_PORT",
    "NETWORK_PROFILE",
    "WEB_MONITOR_BIND_HOST",
    "WEB_MONITOR_PORT",
    "WEB_MONITOR_REQUIRE_TOKEN",
    "WEB_MONITOR_TOKEN",
    "BASE_DIR",
    "USER_DATA_DIR",
    "CALIBRATIONS_DIR",
    "SESSIONS_DIR",
    "USER_CONFIG_PATH",
}
