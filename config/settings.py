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
from typing import Optional


# ── Paths ────────────────────────────────────────────────────────────────
BASE_DIR: Path = Path(__file__).resolve().parent.parent
USER_DATA_DIR: Path = BASE_DIR / "user_data"
CALIBRATIONS_DIR: Path = USER_DATA_DIR / "calibrations"
SESSIONS_DIR: Path = USER_DATA_DIR / "sessions"
MODELS_DIR: Path = BASE_DIR / "models"
LOGS_DIR: Path = BASE_DIR / "logs"
USER_CONFIG_PATH: Path = USER_DATA_DIR / "config.json"


# ── §3.3 / §5 — Sensor roles + targeting mode (v0.2.1) ───────────────────
# TARGETING_MODE is a named preset consumed by sensors.factory.build_sensor_manager().
# Solo modes run one targeting sensor; fusion modes run several and let the
# cross-sensor fusion layer combine them. "config" means "ignore the preset and
# use SENSOR_ROLES below verbatim". See sensors/factory.py MODE_PRESETS.
TARGETING_MODE: str = "config"
#   solo:   "gopro_only" | "kinect_only" | "ov9281_only" | "phone_only" | "local_only"
#   fusion: "ov9281_kinect" | "phone_kinect" | "gopro_kinect" | "fused_interp"
#   "config" → honour SENSOR_ROLES exactly.

# Per-sensor selection the factory honours when TARGETING_MODE == "config".
# Value is the SensorRole ("targeting" | "safety" | "fallback" | "off") or
# "auto" (build with the sensor's natural default role; skip if it won't open).
SENSOR_ROLES: dict = {
    "gopro":     "off",       # default witness/kill-cam now, not targeting
    "kinect_v2": "auto",
    "ov9281":    "auto",      # V1 ground-truth targeting camera
    "phone":     "off",       # needs the companion app running
    "local_cam": "off",
}

SENSOR_STREAM_ROLES: dict = {
    "gopro":        "targeting",
    "kinect_rgb":   "targeting",
    "kinect_ir":    "targeting",
    "kinect_depth": "world_attribution",
    "ov9281":       "targeting",
    "phone":        "targeting",
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
# Threshold for temporal-difference detection (laser-on minus laser-off).
# The diff image is near-black except the dot, so this is low — it only
# has to clear sensor/compression noise.
CALIBRATION_DOT_DIFF_THRESHOLD: int = 40

# Live-calibration dwell timing. The laser is held STEADY at each galvo
# point for HOLD_S seconds — a single 0.1 s flash is far shorter than the
# USB GoPro feed's pipeline latency, so detection must wait SETTLE_S for
# the lit dot to actually appear in the frames before reading it.
CALIBRATION_DWELL_SETTLE_S: float = 1.0
CALIBRATION_DWELL_HOLD_S: float = 1.8
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
LATENCY_SOFTWARE_LAG_MS: float = 194.0  # phone h264_lowlat, measured 2026-05-22
                                        # via tools/phone_latency.py (mean 194,
                                        # min 187, max 204, stdev 8 over 10
                                        # trials). GoPro Hero 13 USB-webcam
                                        # baseline was 810 ms — pass explicitly
                                        # to LaserManager(software_lag_ms=...)
                                        # if you switch back to the GoPro.

# Phone-as-sensor (PHONE_SENSOR_BOOTSTRAP.md). Replaces the GoPro as default
# targeting camera; speaks NoMoSkeeters Sensor Protocol v1 over TCP+UDP.
PHONE_PROTOCOL_VERSION: int = 1
PHONE_IP: str = "192.168.1.158"          # phone's reachable address (USB-NCM
                                          # or wifi); override per session. This
                                          # is a DHCP lease — if the probe can't
                                          # reach it, re-check the app's top line
                                          # or pin a DHCP reservation.
PHONE_CMD_PORT: int = 45470              # TCP — commands + events + heartbeat
PHONE_FRAME_PORT: int = 45471            # UDP — frame packets
PHONE_BIND_IP: str = "0.0.0.0"           # PC's local bind for the UDP frame
                                          # socket. Mirrors the GoPro lesson:
                                          # set explicit on a multi-NIC host.
PHONE_DEFAULT_CAMERA: str = "main"       # "ultrawide"|"main"|"telephoto"
PHONE_DEFAULT_STREAM_MODE: str = "h264_lowlat"   # raw_yuv|h264_lowlat|h264_quality
PHONE_DEFAULT_STREAM_WIDTH: int = 1920
PHONE_DEFAULT_STREAM_HEIGHT: int = 1080
PHONE_DEFAULT_STREAM_FPS: int = 60
PHONE_HEARTBEAT_INTERVAL_S: float = 1.0  # ping cadence; 3 misses = safe state
PHONE_HEARTBEAT_TIMEOUT_S: float = 3.5   # mark unhealthy after this much silence
PHONE_RECONNECT_BACKOFF_S: float = 1.0   # TCP reconnect delay (capped at 8s)
PHONE_FRAME_QUEUE_MAX: int = 1           # latest-only, like the GoPro decoder
PHONE_FRAME_REASSEMBLY_MAX: int = 8      # in-flight fragmented frames kept while
                                          # waiting for chunks; oldest dropped past
                                          # this (a lost chunk can't leak memory)
PHONE_FFMPEG_HWACCEL: Optional[str] = "cuda"   # NVDEC for h264; None = software

# ── OV9281 global-shutter UVC camera — V1 ground-truth targeting sensor ──
# OmniVision OV9281, monochrome global shutter, MJPEG over USB 2.0 (UVC, no
# driver). Global shutter = no rolling-shutter smear on fast targets; IR-
# sensitive so it pairs with the Kinect IR flood for darkness ops. The lens is
# a manual 5-50mm CS varifocal. Frame is delivered in the SensorFrame.rgb slot
# (cv2 hands back 3-channel BGR even for a mono MJPEG source).
OV9281_DEVICE_INDEX: int = 0           # cv2.VideoCapture index (find via probe)
OV9281_BACKEND: str = "auto"           # "auto"|"dshow"|"msmf"|"v4l2"|"any"
                                       #   auto → DSHOW on Windows (best UVC
                                       #   MJPEG/high-fps support), else ANY.
OV9281_FOURCC: str = "MJPG"            # the camera streams MJPEG
OV9281_WIDTH: int = 640
OV9281_HEIGHT: int = 480
OV9281_FPS: int = 210                  # 640x480 @ 210fps mode (datasheet)
                                       #   1280x800@120 / 320x240@420 / 160x120@640
OV9281_AUTO_EXPOSURE: bool = False     # lock exposure for crisp fast frames
OV9281_EXPOSURE: float = -6.0          # backend-dependent: DSHOW≈log2(sec)
                                       #   (-6 ≈ 1/64s); v4l2≈100µs units. For
                                       #   210fps the exposure MUST be < ~4.5ms;
                                       #   tune per rig + lighting.
OV9281_GAIN: float = 0.0               # raise if the locked-exposure image is dark
OV9281_TIMESTAMP_UNCERTAINTY_MS: float = 5.0   # ~OV9281 25ms latency floor / fast

# ── GoPro slow-mo kill-cam (witness recorder, NOT a targeting sensor) ────
# On each laser shot the kill-cam triggers the GoPro to capture/mark a slow-mo
# clip of the engagement. The GoPro can't stream-as-webcam and record at once,
# so this uses it as a dedicated witness camera. Open GoPro HTTP; preset ids are
# firmware-specific (HARDWARE_FINDINGS §2.4) — verify on the bench.
KILLCAM_ENABLED: bool = False
KILLCAM_GOPRO_IP: str = "172.27.109.51"        # USB-tethered Hero 13 (.51)
KILLCAM_MODE: str = "per_shot"                 # "per_shot" | "continuous_hilight"
KILLCAM_SLOMO_PRESET_ID: int = 0xE503          # GoPro slo-mo preset (verify per model)
KILLCAM_RECORD_SECONDS: float = 3.0            # per_shot: clip length after a fire
KILLCAM_PREROLL_SECONDS: float = 0.0           # reserved (continuous mode lead-in)
KILLCAM_DEBOUNCE_SECONDS: float = 2.0          # ignore re-triggers within this window
KILLCAM_TRIGGER_ON: str = "fired"              # "fired" (only real kill pulses) |
                                               #   "any_shot" (every shoot/cone)

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

# Cone-collapse firing sequence (the "bzzt" pattern — wide cone homes in,
# collapses to a line, blanks, fires). All operator-tunable; consumed by
# ConeCollapseConfig.from_settings(). See laser/shot_patterns/cone_collapse.py.
CONE_SHRINK_DURATION_S: float = 0.80   # wide cone collapsing to a point
CONE_LINE_DURATION_S: float = 0.08     # tight "locked" micro-sweep
CONE_DARK_DURATION_S: float = 0.10     # blank — the "trigger pull" beat
CONE_BZZT_DURATION_S: float = 0.12     # full-power kill pulse
CONE_R_START_GALVO: int = 80           # initial cone radius (12-bit galvo units)
CONE_R_MIN_GALVO: int = 6              # tight line/bzzt radius
CONE_POWER_SHRINK: int = 0x500         # reserved — generators use color tuples
CONE_POWER_LINE: int = 0x600           # reserved
CONE_POWER_BZZT: int = 0xFFF           # reserved
CONE_LEAD_FACTOR: float = 0.20         # cone-center pursuit gain per chunk
CONE_BREACH_MULTIPLIER: float = 1.5    # breach when target > this * radius out
CONE_STRETCH_MAX: float = 1.8          # max oval long/short-axis ratio
CONE_STRETCH_VELOCITY_SCALE: float = 1500.0   # galvo units/s for max stretch
CONE_MAX_REACQUIRES: int = 2           # breach restarts before LaserManager aborts


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

# ── Step 12 — Safety moderator (gates cube.enable_output) ─────────────────
SAFETY_MODERATOR_STALE_AFTER_S: float = 0.5   # SAFETY sensor goes stale →
                                              # gate closes (fail-safe).
SAFETY_MODERATOR_COOLDOWN_S: float = 1.0      # re-arm delay after unsafe → safe
SAFETY_VOICE_WARNINGS_ENABLED: bool = True
SAFETY_VOICE_COOLDOWN_S: float = 4.0          # per-message
SAFETY_VOICE_RATE: int = 0                    # System.Speech rate, [-10, 10]
SAFETY_PERSON_DETECT_ENABLED: bool = True     # HOG check on SAFETY RGB sensors
SAFETY_PERSON_HOG_HIT_THRESHOLD: float = 0.0  # SVM margin floor; > 0 stricter
SAFETY_KINECT_DEPTH_CHECK_ENABLED: bool = True
SAFETY_KINECT_DEPTH_MIN_M: float = 0.5
SAFETY_KINECT_DEPTH_MAX_M: float = 4.5
SAFETY_KINECT_PERSON_MIN_HEIGHT_M: float = 0.9
SAFETY_KINECT_PERSON_MAX_HEIGHT_M: float = 2.2


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
