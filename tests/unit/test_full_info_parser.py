"""Step 2 acceptance: parser tests pass with the cube physically disconnected.

The `golden_*.bin` fixtures are real datagrams captured from Cole's cube via
`lasercube_protocol_probe.py --save-raw tests/fixtures/` (2026-05-16). Any
drift in the byte layout breaks this test immediately.

The `synth_*()` builder functions (still imported below) generate parametrized
payloads for the rejection / round-trip tests; only the three canonical-fixture
tests load real bytes off disk.
"""
from pathlib import Path

import pytest

from laser.protocol import (
    parse_full_info, parse_ringbuf_empty, parse_alive,
    connection_type_name,
    synth_full_info, synth_ringbuf_empty,
    LC_CMD_GET_FULL_INFO, LC_CMD_GET_RINGBUF_EMPTY_COUNT,
)


FIXTURES = Path(__file__).resolve().parent.parent / "fixtures"


# ── GET_FULL_INFO ────────────────────────────────────────────────────────

def test_full_info_parses_canonical_fixture():
    raw = (FIXTURES / "golden_full_info_64b.bin").read_bytes()
    assert len(raw) == 64
    info = parse_full_info(raw)
    assert info is not None
    assert info.model_name == "Wifi LaserCube 2.5W"
    assert info.model_number == 1
    assert info.fw_major == 0
    assert info.fw_minor == 23
    assert info.serial_number == "c4:5b:be:88:53:24"
    assert info.reported_ip == "169.254.40.83"
    assert info.dac_rate == 30000
    assert info.max_dac_rate == 30000
    assert info.buffer_free == 6000
    assert info.buffer_size == 6000
    assert info.battery_percent == 0xFF      # AC powered sentinel
    assert info.temperature_c == 44          # live reading at capture time
    assert info.interlock is True
    assert info.output_enabled is False
    assert info.over_temp is False
    assert info.connection_type_raw == 2


def test_full_info_rejects_short_payload():
    assert parse_full_info(b"") is None
    assert parse_full_info(b"\x77") is None
    assert parse_full_info(b"\x77\x00" + b"\x00" * 30) is None    # 32B < 38


def test_full_info_rejects_wrong_cmd_echo():
    bad = bytearray(synth_full_info())
    bad[0] = 0x55
    assert parse_full_info(bytes(bad)) is None


def test_full_info_rejects_nonzero_status():
    bad = bytearray(synth_full_info())
    bad[1] = 0x01
    assert parse_full_info(bytes(bad)) is None


def test_full_info_round_trip_preserves_temperature_negative():
    """Byte 24 is int8 — make sure the parser handles negative values."""
    raw = synth_full_info(temperature_c=-7)
    info = parse_full_info(raw)
    assert info is not None
    assert info.temperature_c == -7


def test_full_info_packet_errors_field():
    raw = synth_full_info(packet_errors=0xA)
    info = parse_full_info(raw)
    assert info is not None
    assert info.packet_errors == 0xA


@pytest.mark.parametrize("raw_byte,expected", [
    (2, "WIFI_CLIENT"),     # Cole's hardware reports 2 → WIFI_CLIENT (after +1)
    (4, "ETHERNET_CLIENT"),
    (0, "USB"),
])
def test_connection_type_name(raw_byte, expected):
    assert connection_type_name(raw_byte) == expected


# ── GET_RINGBUF_EMPTY_COUNT ──────────────────────────────────────────────

def test_ringbuf_empty_parses_canonical_fixture():
    raw = (FIXTURES / "golden_ringbuf_empty_4b.bin").read_bytes()
    assert len(raw) == 4
    assert parse_ringbuf_empty(raw) == 6000


def test_ringbuf_empty_rejects_short():
    assert parse_ringbuf_empty(b"\x8a\x00\x00") is None


def test_ringbuf_empty_rejects_wrong_echo():
    assert parse_ringbuf_empty(b"\x55\x00\x70\x17") is None


def test_ringbuf_empty_rejects_nonzero_status():
    bad = bytearray(synth_ringbuf_empty(empty_count=4096))
    bad[1] = 0xFF
    assert parse_ringbuf_empty(bytes(bad)) is None


def test_ringbuf_empty_value_roundtrip():
    for v in (0, 1, 1500, 4096, 6000, 0xFFFF):
        assert parse_ringbuf_empty(synth_ringbuf_empty(empty_count=v)) == v


# ── GET_ALIVE ────────────────────────────────────────────────────────────

def test_alive_parses_canonical_fixture():
    raw = (FIXTURES / "golden_alive_2b.bin").read_bytes()
    assert raw == bytes([0x27, 0x00])
    assert parse_alive(raw) is True


def test_alive_rejects_garbage():
    assert parse_alive(b"") is False
    assert parse_alive(b"\x27") is False
    assert parse_alive(b"\x27\x01") is False
    assert parse_alive(b"\x55\x00") is False


# ── Constants sanity ─────────────────────────────────────────────────────

def test_protocol_command_bytes():
    assert LC_CMD_GET_FULL_INFO == 0x77
    assert LC_CMD_GET_RINGBUF_EMPTY_COUNT == 0x8A
