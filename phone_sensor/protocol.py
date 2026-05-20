"""NoMoSkeeters Sensor Protocol v1 — wire format.

Two channels between PC and phone:

  - **TCP, command channel** (PHONE_CMD_PORT). One line-delimited JSON object
    per message. Bidirectional: the PC sends commands; the phone replies and
    pushes unsolicited status events. Cheap to parse, easy to debug with
    `nc`, and the bandwidth is tiny.
  - **UDP, frame channel** (PHONE_FRAME_PORT). One packet per video frame
    (one frame fits in one MTU for raw YUV at modest resolutions; for H.264
    keyframes a packet can be ~10s of KB). Compact binary header so the per-
    frame overhead is single-digit bytes.

The protocol is intentionally line-oriented JSON rather than protobuf — the
phone-side spec mentions both; JSON is what this PC implementation expects.
Every command carries a monotonic `cmd_id`; the phone echoes it in the reply.

Reference: PHONE_SENSOR_BOOTSTRAP.md §2.
"""
from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

PROTOCOL_VERSION = 1


# ── Command channel — JSON ──────────────────────────────────────────────

class StreamMode(str, Enum):
    RAW_YUV = "raw_yuv"
    H264_LOWLAT = "h264_lowlat"
    H264_QUALITY = "h264_quality"


# Commands the phone accepts (PHONE_SENSOR_BOOTSTRAP §2.1). Listed so callers
# typo-check at one site; the wire is still free-form JSON `{type, ...}`.
COMMANDS = frozenset({
    "connect", "disconnect", "ping",
    "set_active_camera", "get_camera_capabilities",
    "set_exposure_mode", "set_exposure_value",
    "set_af_mode", "set_af_region", "lock_focus", "unlock_focus",
    "stream_start", "stream_stop",
    "stream_set_resolution", "stream_set_target_bitrate",
    "stream_set_target_fps",
    "get_status", "get_intrinsics",
    "recording_start", "recording_stop",
    "recording_list", "recording_transfer",
})

# Unsolicited events the phone pushes (PHONE_SENSOR_BOOTSTRAP §2.2). Mapped
# 1:1 onto our bus events by PhoneSensor.
EVENTS = frozenset({
    "event:af_settled", "event:exposure_changed", "event:thermal_warning",
    "event:battery_low", "event:camera_unavailable", "event:camera_changed",
    "event:stream_started", "event:stream_stopped",
})


@dataclass(frozen=True)
class PhoneCameraSpec:
    """One physical lens the phone exposes. From the capabilities manifest."""
    id: str
    fov_h_deg: float = 0.0
    fov_v_deg: float = 0.0
    max_resolution: tuple = (0, 0)
    preferred_streaming_resolution: tuple = (0, 0)
    max_fps_at_streaming_res: int = 0
    has_optical_zoom: bool = False
    optical_zoom_factor: float = 1.0
    supports_af: bool = False
    supports_locked_focus: bool = False
    supports_hdr: bool = False

    @classmethod
    def from_dict(cls, d: dict) -> "PhoneCameraSpec":
        return cls(
            id=d["id"],
            fov_h_deg=float(d.get("fov_h_deg", 0.0)),
            fov_v_deg=float(d.get("fov_v_deg", 0.0)),
            max_resolution=tuple(d.get("max_resolution", (0, 0))),
            preferred_streaming_resolution=tuple(
                d.get("preferred_streaming_resolution", (0, 0))),
            max_fps_at_streaming_res=int(d.get("max_fps_at_streaming_res", 0)),
            has_optical_zoom=bool(d.get("has_optical_zoom", False)),
            optical_zoom_factor=float(d.get("optical_zoom_factor", 1.0)),
            supports_af=bool(d.get("supports_af", False)),
            supports_locked_focus=bool(d.get("supports_locked_focus", False)),
            supports_hdr=bool(d.get("supports_hdr", False)),
        )


@dataclass(frozen=True)
class PhoneCapabilities:
    """The phone's manifest returned on `connect`."""
    phone_model: str
    protocol_version: int
    cameras: tuple = ()       # tuple of PhoneCameraSpec

    def camera(self, camera_id: str) -> Optional[PhoneCameraSpec]:
        for c in self.cameras:
            if c.id == camera_id:
                return c
        return None

    @classmethod
    def from_dict(cls, d: dict) -> "PhoneCapabilities":
        return cls(
            phone_model=d.get("phone_model", "unknown"),
            protocol_version=int(d.get("protocol_version", 0)),
            cameras=tuple(PhoneCameraSpec.from_dict(c)
                          for c in d.get("cameras", ())),
        )


def pack_command(cmd: str, cmd_id: int, **params: Any) -> bytes:
    """Encode one command as a JSON line (trailing newline included).

    `cmd` is the message type (`set_active_camera`, `ping`, …); `cmd_id` is a
    monotonic id the phone echoes in its reply; `params` is the per-command
    payload."""
    if cmd not in COMMANDS:
        raise ValueError(f"unknown phone command: {cmd!r}")
    msg = {"type": cmd, "cmd_id": int(cmd_id), **params}
    return (json.dumps(msg, separators=(",", ":")) + "\n").encode("utf-8")


def parse_message(line: bytes) -> dict:
    """Decode one JSON line from the phone — a reply or an unsolicited event.

    Raises ValueError on malformed input; the caller decides whether to drop
    the message or close the link."""
    try:
        obj = json.loads(line.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ValueError(f"bad phone message: {exc}") from None
    if not isinstance(obj, dict) or "type" not in obj:
        raise ValueError("phone message missing 'type' field")
    return obj


# ── Frame channel — compact binary header ───────────────────────────────
#
# Header (little-endian):
#
#   uint32 magic       = b"NMS1"  — sanity / version-id
#   uint64 frame_id    — monotonic, lets the PC detect drops
#   uint64 capture_ts  — microseconds, phone monotonic clock
#   uint16 cam_id_len  — bytes of camera_id string that follow
#   uint16 fmt_len     — bytes of format string that follow (e.g. "nv21")
#   uint16 width
#   uint16 height
#   uint32 payload_len — bytes of pixel/encoded payload that follow
#   <camera_id bytes>  — utf-8
#   <format bytes>     — utf-8
#   <payload bytes>    — raw YUV, H.264 NAL, etc. (see StreamMode)

_FRAME_MAGIC = b"NMS1"
_HEADER_FMT = "<4sQQHHHHI"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)


@dataclass
class FramePacket:
    """One decoded frame packet off the UDP socket — before pixel decoding."""
    frame_id: int
    capture_ts_us: int
    camera_id: str
    fmt: str                 # "nv21", "i420", "h264", …
    width: int
    height: int
    payload: bytes


def pack_frame_packet(p: FramePacket) -> bytes:
    cid = p.camera_id.encode("utf-8")
    fmt = p.fmt.encode("utf-8")
    if len(cid) > 0xFFFF or len(fmt) > 0xFFFF:
        raise ValueError("camera_id or format too long for the header")
    return struct.pack(_HEADER_FMT, _FRAME_MAGIC,
                       p.frame_id, p.capture_ts_us,
                       len(cid), len(fmt), p.width, p.height,
                       len(p.payload)) + cid + fmt + p.payload


def parse_frame_packet(buf: bytes) -> FramePacket:
    """Parse one UDP datagram into a FramePacket. Raises ValueError on a bad
    magic, short read, or length mismatch — caller drops the packet."""
    if len(buf) < _HEADER_SIZE:
        raise ValueError("frame packet shorter than header")
    (magic, frame_id, capture_ts_us, cid_len, fmt_len, width, height,
     payload_len) = struct.unpack(_HEADER_FMT, buf[:_HEADER_SIZE])
    if magic != _FRAME_MAGIC:
        raise ValueError(f"bad frame magic {magic!r}")
    end_cid = _HEADER_SIZE + cid_len
    end_fmt = end_cid + fmt_len
    end_pay = end_fmt + payload_len
    if len(buf) < end_pay:
        raise ValueError(
            f"frame packet truncated: have {len(buf)}, need {end_pay}")
    return FramePacket(
        frame_id=frame_id,
        capture_ts_us=capture_ts_us,
        camera_id=buf[_HEADER_SIZE:end_cid].decode("utf-8"),
        fmt=buf[end_cid:end_fmt].decode("utf-8"),
        width=width, height=height,
        payload=bytes(buf[end_fmt:end_pay]),
    )
