"""NoMoSkeeters Sensor Protocol v1 — wire format.

Two channels between PC and phone:

  - **TCP, command channel** (PHONE_CMD_PORT). One line-delimited JSON object
    per message. Bidirectional: the PC sends commands; the phone replies and
    pushes unsolicited status events. Cheap to parse, easy to debug with
    `nc`, and the bandwidth is tiny.
  - **UDP, frame channel** (PHONE_FRAME_PORT). One video frame per *chunk
    sequence* — a frame is split into one or more datagrams (chunk_index /
    chunk_count) so an H.264 IDR or a raw-YUV frame larger than the UDP ceiling
    still gets through; the PC reassembles by frame_id. Compact binary header
    so the per-chunk overhead is single-digit bytes.

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
# A single video frame is split into one or more *chunks*, each its own UDP
# datagram, so a frame larger than the IPv4 UDP payload ceiling (an H.264 IDR,
# or any raw-YUV frame) still gets through. All chunks of a frame share its
# `frame_id`; the PC reassembles them in `chunk_index` order (see
# `phone_sensor.frame_decoder._FrameReassembler`). An unfragmented frame is
# just `chunk_count == 1`.
#
# Header (little-endian):
#
#   uint32 magic        = b"NMS2"  — sanity / version-id
#   uint64 frame_id     — monotonic; identical across a frame's chunks
#   uint64 capture_ts   — microseconds, phone monotonic clock
#   uint16 cam_id_len   — bytes of camera_id string that follow
#   uint16 fmt_len      — bytes of format string that follow (e.g. "nv21")
#   uint16 width
#   uint16 height
#   uint32 payload_len  — bytes of payload in THIS chunk that follow
#   uint16 chunk_index  — 0-based position of this chunk within the frame
#   uint16 chunk_count  — total chunks for the frame (>= 1)
#   <camera_id bytes>   — utf-8 (repeated in every chunk)
#   <format bytes>      — utf-8 (repeated in every chunk)
#   <payload bytes>     — this chunk's slice of the frame
#
# `MAX_CHUNK_PAYLOAD` keeps each packed datagram (header + strings + chunk)
# under the 65507-byte ceiling with room to spare; the phone splits on it.

_FRAME_MAGIC = b"NMS2"
_HEADER_FMT = "<4sQQHHHHIHH"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)

#: 65535 − 8 (UDP) − 20 (IPv4) = 65507 datagram ceiling; leave headroom for the
#: header and the (tiny) camera_id/format strings.
MAX_UDP_PAYLOAD = 65507
MAX_CHUNK_PAYLOAD = 60000


@dataclass
class FramePacket:
    """One frame packet off the UDP socket — before pixel decoding.

    On the wire this is a single *chunk*; `chunk_index`/`chunk_count` describe
    its place in the frame. After reassembly the decoder hands consumers a
    `FramePacket` with the full payload and `chunk_index=0, chunk_count=1`.
    """
    frame_id: int
    capture_ts_us: int
    camera_id: str
    fmt: str                 # "nv21", "i420", "h264", …
    width: int
    height: int
    payload: bytes
    chunk_index: int = 0
    chunk_count: int = 1


def pack_frame_packet(p: FramePacket) -> bytes:
    cid = p.camera_id.encode("utf-8")
    fmt = p.fmt.encode("utf-8")
    if len(cid) > 0xFFFF or len(fmt) > 0xFFFF:
        raise ValueError("camera_id or format too long for the header")
    if not (1 <= p.chunk_count <= 0xFFFF):
        raise ValueError(f"chunk_count out of range: {p.chunk_count}")
    if not (0 <= p.chunk_index < p.chunk_count):
        raise ValueError(
            f"chunk_index {p.chunk_index} out of range for count {p.chunk_count}")
    return struct.pack(_HEADER_FMT, _FRAME_MAGIC,
                       p.frame_id, p.capture_ts_us,
                       len(cid), len(fmt), p.width, p.height,
                       len(p.payload), p.chunk_index, p.chunk_count) \
        + cid + fmt + p.payload


def parse_frame_packet(buf: bytes) -> FramePacket:
    """Parse one UDP datagram (one chunk) into a FramePacket. Raises ValueError
    on a bad magic, short read, length mismatch, or out-of-range chunk fields —
    caller drops the packet."""
    if len(buf) < _HEADER_SIZE:
        raise ValueError("frame packet shorter than header")
    (magic, frame_id, capture_ts_us, cid_len, fmt_len, width, height,
     payload_len, chunk_index, chunk_count) = struct.unpack(
        _HEADER_FMT, buf[:_HEADER_SIZE])
    if magic != _FRAME_MAGIC:
        raise ValueError(f"bad frame magic {magic!r}")
    if chunk_count < 1 or chunk_index >= chunk_count:
        raise ValueError(
            f"bad chunk fields: index {chunk_index} of count {chunk_count}")
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
        chunk_index=chunk_index, chunk_count=chunk_count,
    )
