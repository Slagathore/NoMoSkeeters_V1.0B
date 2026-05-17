"""Step 3 acceptance: DryRunTransport.connect(); send_chunk([...]) runs without
hardware, logs to an event sink, no UDP packets emitted."""
import time

from laser.transports.dry_run import DryRunTransport
from laser.types import LaserPoint


def test_connect_and_get_full_info_returns_canonical_info():
    events = []
    t = DryRunTransport(event_sink=events.append)
    assert t.connect() is True
    assert t.is_connected() is True
    info = t.get_full_info()
    assert info is not None
    assert info.model_name == "Wifi LaserCube 2.5W"
    assert info.fw_major == 0 and info.fw_minor == 23
    assert info.buffer_size == 6000
    assert info.serial_number == "c4:5b:be:88:53:24"


def test_send_chunk_advances_msg_num_and_decrements_buffer():
    t = DryRunTransport()
    t.connect()
    free_before = t.buffer_estimate().free
    pts = [LaserPoint(x=0x800, y=0x800, r=0xFFF, g=0, b=0)] * 140
    assert t.send_chunk(pts, msg_num=0, frame_num=0) is True
    assert t.packet_count == 1
    assert t.buffer_estimate().free == free_before - 140


def test_send_frame_splits_long_path_into_chunks():
    t = DryRunTransport()
    t.connect()
    pts = [LaserPoint(x=i, y=i) for i in range(350)]   # 3 chunks of 140/140/70
    assert t.send_frame(pts, frame_num=7) is True
    assert t.packet_count == 3


def test_enable_disable_output_reflected_in_get_full_info():
    t = DryRunTransport()
    t.connect()
    info = t.get_full_info()
    assert info is not None and info.output_enabled is False
    t.enable_output()
    info = t.get_full_info()
    assert info is not None and info.output_enabled is True
    t.disable_output()
    info = t.get_full_info()
    assert info is not None and info.output_enabled is False


def test_disconnect_disables_output_best_effort():
    t = DryRunTransport()
    t.connect()
    t.enable_output()
    assert t.output_enabled is True
    t.disconnect()
    assert t.output_enabled is False
    assert t.is_connected() is False


def test_event_sink_captures_operation_stream():
    events = []
    t = DryRunTransport(event_sink=events.append)
    t.connect()
    t.enable_output()
    t.send_chunk([LaserPoint()], msg_num=0, frame_num=0)
    t.disable_output()
    t.disconnect()
    ops = [e["op"] for e in events]
    # connect, enable_output, send_chunk, disable_output, disconnect (with the
    # disconnect-time disable_output also emitted because output flips during cleanup).
    assert ops[0] == "connect"
    assert "enable_output" in ops
    assert "send_chunk" in ops
    assert "disable_output" in ops
    assert ops[-1] == "disconnect"


def test_clear_ringbuffer_resets_estimate():
    t = DryRunTransport()
    t.connect()
    pts = [LaserPoint()] * 1000
    t.send_chunk(pts[:140], msg_num=0, frame_num=0)
    assert t.buffer_estimate().free < t.buffer_estimate().size
    t.clear_ringbuffer()
    assert t.buffer_estimate().free == t.buffer_estimate().size


def test_get_ringbuf_empty_returns_simulated_free_count():
    t = DryRunTransport()
    t.connect()
    free = t.get_ringbuf_empty()
    assert free == 6000
    t.send_chunk([LaserPoint()] * 100, msg_num=0, frame_num=0)
    assert t.get_ringbuf_empty() == 5900


def test_set_dac_rate_clamps_and_logs():
    events = []
    t = DryRunTransport(event_sink=events.append)
    t.connect()
    assert t.set_dac_rate(99999) is True   # clamped to 30000
    info = t.get_full_info()
    assert info is not None and info.dac_rate == 30000
    assert any(e["op"] == "set_dac_rate" for e in events)


def test_buffer_estimate_freshness_advances():
    t = DryRunTransport()
    t.connect()
    be1 = t.buffer_estimate()
    time.sleep(0.01)
    t.get_ringbuf_empty()
    be2 = t.buffer_estimate()
    assert be2.last_refresh_ts >= be1.last_refresh_ts
