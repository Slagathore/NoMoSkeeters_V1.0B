"""Bus event payloads. One dataclass per event type.

Every emitter constructs and publishes one of these. Every consumer accepts
the dataclass directly. No dict-shaped events, no positional-arg drift.

Reference: BOOTSTRAP_AMENDMENTS.md §3.1.1.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class FrameEvent:
    """Raw sensor frame ready for downstream processing."""
    timestamp: float
    sensor_id: str
    sensor_role: str
    rgb: Optional[np.ndarray]
    depth: Optional[np.ndarray]
    ir: Optional[np.ndarray]
    width: int
    height: int
    timestamp_uncertainty_ms: float = 5.0


@dataclass(frozen=True)
class DetectionEvent:
    """One candidate detection from a frame."""
    timestamp: float
    sensor_id: str
    detection_id: int
    x_norm: float
    y_norm: float
    x_px: int
    y_px: int
    area_pixels: float
    bbox: Tuple[int, int, int, int]
    classifier_label: str
    classifier_confidence: float
    z_world_m: Optional[float] = None
    x_world_m: Optional[float] = None
    y_world_m: Optional[float] = None


@dataclass
class TrackEvent:
    """Updated track state after the tracker runs on a frame."""
    timestamp: float
    track_id: int
    sensor_id: str
    state_norm: Tuple[float, ...]
    velocity_norm: Tuple[float, ...]
    age_frames: int
    confirmed: bool
    coasting: bool
    confidence: float
    fire_eligible: bool
    last_detection_age_ms: float


@dataclass(frozen=True)
class TargetCommandEvent:
    """Computed laser aim point for a track."""
    timestamp: float
    track_id: int
    target_x_galvo: int
    target_y_galvo: int
    pattern_id: str
    dwell_ms: int


@dataclass(frozen=True)
class LaserStatusEvent:
    """Periodic status from the cube."""
    timestamp: float
    output_enabled: bool
    interlock_ok: bool
    over_temp: bool
    buffer_free: int
    buffer_free_age_ms: float
    dac_rate: int
    packet_errors_since_last: int


@dataclass(frozen=True)
class GoProStatusEvent:
    """Periodic /gopro/camera/state poll from the GoPro Hero 13.

    Fields map to Open GoPro status codes (HARDWARE_FINDINGS.md §2.4). Safety
    subscribes to this: `system_hot` / `thermal_throttle` gate streaming, and
    `preview_stream_active` falling to False unexpectedly flags a dead feed.
    `reachable` is False when the state poll itself failed (camera offline /
    USB unplugged); the other fields are then meaningless.
    """
    timestamp: float
    reachable: bool
    battery_percent: int          # status 70
    system_hot: bool              # status 6  — overheating; stop streaming
    system_busy: bool             # status 8  — busy with another task
    preview_stream_active: bool   # status 32 — UDP preview push running
    thermal_throttle: bool        # status 86 — thermal throttling state


@dataclass(frozen=True)
class PhoneCameraChangedEvent:
    """Emitted when the phone-as-sensor switches active lens (wide ↔ main ↔
    telephoto). Carries the new camera_id and the per-camera intrinsics so
    the tracker can invalidate norm-space tracks across the discontinuity
    (PHONE_SENSOR_BOOTSTRAP.md §3.3, §3.5)."""
    timestamp: float
    camera_id: str
    width: int
    height: int
    fov_h_deg: float = 0.0
    has_optical_zoom: bool = False
    optical_zoom_factor: float = 1.0


@dataclass(frozen=True)
class PhoneFocusEvent:
    """Phone AF state change. Recorded for replay; rarely used for live
    decisions."""
    timestamp: float
    camera_id: str
    state: str           # "settled" | "searching" | "locked" | "unlocked"
    region: Optional[Tuple[float, float, float, float]] = None  # (x,y,w,h) norm


@dataclass(frozen=True)
class PhoneThermalEvent:
    """Phone thermal state. Safety may gate firing when severity rises
    (PHONE_SENSOR_BOOTSTRAP.md §4.5)."""
    timestamp: float
    camera_id: str
    state: str           # "none" | "light" | "moderate" | "severe" | "critical"
    battery_percent: int = -1
    reachable: bool = True


@dataclass(frozen=True)
class SensorHealthEvent:
    """Worker-level sensor liveness from SensorManager: a sensor whose
    read() stops producing frames is closed and reopened with backoff.
    Distinct from camera-side status (GoProStatusEvent / PhoneThermalEvent)
    — this covers the PC side dying under us: dead USB device, crashed
    ffmpeg decoder, wedged SDK."""
    timestamp: float
    sensor_id: str
    state: str            # "stalled" | "reopening" | "recovered" | "reopen_failed"
    detail: str = ""
    stalled_for_s: float = 0.0


@dataclass(frozen=True)
class LatencySample:
    """One end-to-end pipeline latency measurement."""
    timestamp: float
    detection_to_target_command_ms: float
    capture_to_detection_ms: float
    target_command_to_send_ms: float
    total_ms: float
