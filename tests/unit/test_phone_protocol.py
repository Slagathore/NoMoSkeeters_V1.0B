"""Phone-as-sensor protocol — JSON commands + binary frame packets."""
from __future__ import annotations

import json

import pytest

from phone_sensor.protocol import (PROTOCOL_VERSION, FramePacket,
                                    PhoneCapabilities, pack_command,
                                    pack_frame_packet, parse_frame_packet,
                                    parse_message)


# ── Command channel ──────────────────────────────────────────────────────

def test_pack_command_round_trip():
    wire = pack_command("set_active_camera", 42, camera_id="telephoto")
    assert wire.endswith(b"\n")
    msg = parse_message(wire.rstrip(b"\n"))
    assert msg == {"type": "set_active_camera", "cmd_id": 42,
                   "camera_id": "telephoto"}


def test_pack_command_rejects_unknown():
    with pytest.raises(ValueError):
        pack_command("brew_coffee", 1)


def test_parse_message_requires_type():
    with pytest.raises(ValueError):
        parse_message(b'{"cmd_id": 1, "ok": true}')


def test_parse_message_rejects_malformed_json():
    with pytest.raises(ValueError):
        parse_message(b'{not json}')


def test_capabilities_from_dict():
    caps = PhoneCapabilities.from_dict({
        "phone_model": "OnePlus 15",
        "protocol_version": PROTOCOL_VERSION,
        "cameras": [
            {"id": "main", "fov_h_deg": 75, "supports_af": True},
            {"id": "telephoto", "has_optical_zoom": True,
             "optical_zoom_factor": 3.0},
        ],
    })
    assert caps.phone_model == "OnePlus 15"
    assert len(caps.cameras) == 2
    tele = caps.camera("telephoto")
    assert tele is not None and tele.optical_zoom_factor == 3.0
    assert caps.camera("missing") is None


# ── Frame channel ────────────────────────────────────────────────────────

def test_frame_packet_round_trip():
    pkt = FramePacket(frame_id=7, capture_ts_us=123_456_789,
                      camera_id="main", fmt="nv21",
                      width=320, height=240,
                      payload=b"\x00" * (320 * 240 * 3 // 2))
    wire = pack_frame_packet(pkt)
    back = parse_frame_packet(wire)
    assert back.frame_id == 7
    assert back.capture_ts_us == 123_456_789
    assert back.camera_id == "main"
    assert back.fmt == "nv21"
    assert (back.width, back.height) == (320, 240)
    assert len(back.payload) == 320 * 240 * 3 // 2


def test_frame_packet_rejects_bad_magic():
    pkt = FramePacket(0, 0, "main", "nv21", 4, 4, b"\x00" * 24)
    wire = bytearray(pack_frame_packet(pkt))
    wire[:4] = b"XXXX"
    with pytest.raises(ValueError):
        parse_frame_packet(bytes(wire))


def test_frame_packet_rejects_truncated_payload():
    pkt = FramePacket(0, 0, "main", "nv21", 4, 4, b"\x00" * 24)
    wire = pack_frame_packet(pkt)
    with pytest.raises(ValueError):
        parse_frame_packet(wire[:-5])           # chop the payload tail


def test_frame_packet_defaults_to_single_chunk():
    pkt = FramePacket(1, 2, "main", "h264", 8, 8, b"\x01" * 10)
    back = parse_frame_packet(pack_frame_packet(pkt))
    assert (back.chunk_index, back.chunk_count) == (0, 1)


def test_frame_packet_chunk_fields_round_trip():
    pkt = FramePacket(5, 9, "telephoto", "h264", 1920, 1080,
                      b"\xab" * 100, chunk_index=2, chunk_count=4)
    back = parse_frame_packet(pack_frame_packet(pkt))
    assert back.frame_id == 5
    assert back.chunk_index == 2
    assert back.chunk_count == 4
    assert back.payload == b"\xab" * 100


def test_pack_rejects_index_past_count():
    with pytest.raises(ValueError):
        pack_frame_packet(FramePacket(0, 0, "main", "h264", 4, 4, b"",
                                      chunk_index=3, chunk_count=3))


def test_pack_rejects_zero_count():
    with pytest.raises(ValueError):
        pack_frame_packet(FramePacket(0, 0, "main", "h264", 4, 4, b"",
                                      chunk_index=0, chunk_count=0))


def test_parse_rejects_bad_chunk_fields():
    # Hand-craft a header with chunk_count=0 to exercise the parse-side guard.
    import struct
    from phone_sensor.protocol import _FRAME_MAGIC, _HEADER_FMT
    bad = struct.pack(_HEADER_FMT, _FRAME_MAGIC, 0, 0, 4, 4, 4, 4, 0, 0, 0) \
        + b"main" + b"nv21"
    with pytest.raises(ValueError):
        parse_frame_packet(bad)
