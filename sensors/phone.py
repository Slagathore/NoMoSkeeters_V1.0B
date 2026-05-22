"""PhoneSensor — the phone-as-remote-smart-camera, primary TARGETING.

Replaces the GoPro in the targeting role. The PC is the brain; the phone is
a dumb-on-purpose smart sensor that accepts commands (switch lens, set AF
region, start streaming) and ships frames. PhoneSensor wires the two halves —
`phone_sensor.PhoneSensorClient` for the TCP command channel and
`phone_sensor.PhoneFrameDecoder` for the UDP frame channel — into a `Sensor`
that drops into `SensorManager` exactly like any other.

`sensor_id` is `phone_<camera>` (e.g. `phone_main`, `phone_telephoto`) and
re-tags when the active lens changes mid-session, so downstream consumers see
the discontinuity. The lens-switch dance itself (§3.5) is `switch_camera`.

Reference: PHONE_SENSOR_BOOTSTRAP.md §2-§3.
"""
from __future__ import annotations

import logging
import time
from typing import Any, Callable, Optional

from config import settings
from events.schemas import (PhoneCameraChangedEvent, PhoneFocusEvent,
                            PhoneThermalEvent)
from phone_sensor.client import PhoneSensorClient
from phone_sensor.frame_decoder import PhoneFrameDecoder
from sensors.base import Sensor, SensorFrame, SensorRole

_log = logging.getLogger(__name__)

StatusCallback = Callable[[object], None]


def _codec_for_mode(mode: str) -> str:
    """Map a negotiated stream mode to the decoder's codec name.

    raw_yuv defaults to NV21 (Android CameraX standard); h264 modes both
    decode via ffmpeg. The per-packet `fmt` header drives the actual YUV
    conversion, so a phone that ships I420 still decodes correctly."""
    if mode.startswith("h264"):
        return "h264"
    return "nv21"


class PhoneSensor(Sensor):
    """Streams phone-camera frames as SensorFrames over the standard bus."""

    def __init__(self,
                 client: Optional[PhoneSensorClient] = None,
                 *,
                 role: SensorRole = SensorRole.TARGETING,
                 active_camera: str = settings.PHONE_DEFAULT_CAMERA,
                 stream_mode: str = settings.PHONE_DEFAULT_STREAM_MODE,
                 stream_width: int = settings.PHONE_DEFAULT_STREAM_WIDTH,
                 stream_height: int = settings.PHONE_DEFAULT_STREAM_HEIGHT,
                 stream_fps: int = settings.PHONE_DEFAULT_STREAM_FPS,
                 timestamp_uncertainty_ms: float = 60.0,
                 decoder_factory: Optional[Callable[..., Any]] = None,
                 record_path: Optional[str] = None,
                 on_status: Optional[StatusCallback] = None):
        self._client = client or PhoneSensorClient()
        self._role = role
        self._active_camera = active_camera
        self._stream_mode = stream_mode
        self._stream_width = stream_width
        self._stream_height = stream_height
        self._stream_fps = stream_fps
        self._ts_uncertainty = timestamp_uncertainty_ms
        self._decoder_factory = decoder_factory or self._default_decoder
        self._record_path = record_path
        self._on_status = on_status
        self._cap: Optional[Any] = None
        self._width = 0
        self._height = 0
        self._sensor_id = f"phone_{self._active_camera}"

    def _default_decoder(self, *, width: int, height: int,
                         codec: str) -> object:
        return PhoneFrameDecoder(
            width=width, height=height, codec=codec,
            record_path=self._record_path)

    def set_status_sink(self, on_status: Optional[StatusCallback]) -> None:
        """Register (or clear) the bus sink for phone status events. Must be
        called BEFORE open() — open() wires the client's event channel into
        this. SensorManager calls it automatically."""
        self._on_status = on_status

    # ── Identity / capabilities ──────────────────────────────────────────

    @property
    def sensor_id(self) -> str:
        return self._sensor_id

    @property
    def role(self) -> SensorRole:
        return self._role

    @property
    def has_rgb(self) -> bool:
        return True

    @property
    def has_depth(self) -> bool:
        return False

    @property
    def has_ir(self) -> bool:
        # Phone IR-pass is an open question per §8; assume colour-only until
        # the phone reports an IR-capable camera in its manifest.
        return False

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def active_camera(self) -> str:
        return self._active_camera

    @property
    def client(self) -> PhoneSensorClient:
        """The TCP control channel — exposed for PC orchestration code that
        needs to drive lens / focus / exposure beyond the Sensor surface."""
        return self._client

    # ── Lifecycle ────────────────────────────────────────────────────────

    def open(self) -> bool:
        # Wire client events into our bus-event translator BEFORE connect()
        # so handshake-time events (e.g. capabilities) are not dropped.
        self._client.set_event_sink(self._on_client_event)
        if not self._client.connect():
            _log.error("phone: client.connect() failed")
            return False
        if not self._client.set_active_camera(self._active_camera):
            _log.error("phone: set_active_camera(%s) failed",
                       self._active_camera)
            self._client.disconnect()
            return False
        negotiated = self._client.stream_start(
            mode=self._stream_mode, width=self._stream_width,
            height=self._stream_height, fps=self._stream_fps)
        if negotiated is None:
            _log.error("phone: stream_start failed")
            self._client.disconnect()
            return False
        if negotiated != self._stream_mode:
            _log.info("phone: stream mode downgraded %s -> %s",
                      self._stream_mode, negotiated)
            self._stream_mode = negotiated
        self._cap = self._decoder_factory(
            width=self._stream_width, height=self._stream_height,
            codec=_codec_for_mode(negotiated))
        if self._cap is None or not self._cap.isOpened():
            _log.error("phone: frame decoder did not start")
            if self._cap is not None:
                self._cap.release()
                self._cap = None
            self._client.disconnect()
            return False
        _log.info("phone: streaming %s @ %dx%d (camera=%s, mode=%s)",
                  self._sensor_id, self._stream_width, self._stream_height,
                  self._active_camera, negotiated)
        return True

    def close(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                _log.exception("phone: decoder release raised; ignoring")
            self._cap = None
        if self._client.is_connected:
            try:
                self._client.stream_stop()
            except Exception:
                _log.exception("phone: stream_stop raised; closing anyway")
            self._client.disconnect()

    def read(self) -> Optional[SensorFrame]:
        if self._cap is None:
            return None
        ok, image = self._cap.read()
        if not ok or image is None:
            return None
        # Per-packet camera_id can change mid-stream (lens switch). Re-tag
        # frames so detector/tracker see the discontinuity without a
        # separate `set_active_camera` round-trip.
        latest_cam = getattr(self._cap, "latest_camera_id", None)
        if latest_cam and latest_cam != self._active_camera:
            self._active_camera = latest_cam
            self._sensor_id = f"phone_{latest_cam}"
        self._height, self._width = image.shape[:2]
        frame = SensorFrame.now(self._sensor_id, self._role)
        frame.rgb = image
        frame.width = self._width
        frame.height = self._height
        frame.timestamp_uncertainty_ms = self._ts_uncertainty
        return frame

    # ── PC-side orchestration vocabulary (§3.5 lens-switch dance) ────────

    def switch_camera(self, camera_id: str) -> bool:
        """Command the phone to a new lens. Caller is responsible for
        invalidating the tracker — this just commands + relabels; the per-
        packet camera_id in the UDP stream will pivot when the phone has
        the new lens producing frames."""
        if not self._client.set_active_camera(camera_id):
            return False
        self._active_camera = camera_id
        self._sensor_id = f"phone_{camera_id}"
        return True

    # ── Client event translation → bus events ────────────────────────────

    def _on_client_event(self, msg: dict) -> None:
        """Convert one raw protocol event into a typed bus event, then push
        to the registered status sink. Unknown events are dropped."""
        if self._on_status is None:
            return
        kind = msg.get("type", "")
        ts = time.monotonic()
        event = None
        if kind == "event:camera_changed":
            event = PhoneCameraChangedEvent(
                timestamp=ts,
                camera_id=msg.get("camera_id", self._active_camera),
                width=int(msg.get("width", self._stream_width)),
                height=int(msg.get("height", self._stream_height)),
                fov_h_deg=float(msg.get("fov_h_deg", 0.0)),
                has_optical_zoom=bool(msg.get("has_optical_zoom", False)),
                optical_zoom_factor=float(msg.get("optical_zoom_factor", 1.0)),
            )
        elif kind == "event:af_settled":
            region = msg.get("region")
            event = PhoneFocusEvent(
                timestamp=ts,
                camera_id=msg.get("camera_id", self._active_camera),
                state=msg.get("state", "settled"),
                region=tuple(region) if region else None,
            )
        elif kind == "event:thermal_warning":
            event = PhoneThermalEvent(
                timestamp=ts,
                camera_id=msg.get("camera_id", self._active_camera),
                state=msg.get("state", "light"),
                battery_percent=int(msg.get("battery_percent", -1)),
                reachable=True,
            )
        elif kind == "event:battery_low":
            event = PhoneThermalEvent(
                timestamp=ts,
                camera_id=self._active_camera,
                state="battery_low",
                battery_percent=int(msg.get("battery_percent", -1)),
                reachable=True,
            )
        if event is None:
            return
        try:
            self._on_status(event)
        except Exception:
            _log.exception("phone: status sink raised; suppressing")
