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
