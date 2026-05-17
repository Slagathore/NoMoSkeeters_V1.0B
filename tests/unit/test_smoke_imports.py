"""Step 0 acceptance: imports work, pytest finds the empty test tree."""

def test_laser_types_import():
    from laser.types import LaserPoint, LaserInfo, BufferEstimate, COORD_MIN, COORD_MAX
    assert COORD_MIN == 0
    assert COORD_MAX == 0xFFF


def test_laser_point_packs_to_10_bytes():
    from laser.types import LaserPoint
    pkt = LaserPoint(x=0x800, y=0x800, r=0xFFF, g=0, b=0).packed()
    assert len(pkt) == 10


def test_laser_point_clamps_to_12_bit():
    from laser.types import LaserPoint
    # Out-of-range values clamp to 0xFFF on the wire.
    pkt = LaserPoint(x=99999, y=-5, r=0xFFFF, g=0, b=0).packed()
    import struct
    x, y, r, g, b = struct.unpack("<HHHHH", pkt)
    assert x == 0xFFF
    assert y == 0
    assert r == 0xFFF


def test_buffer_estimate_freshness():
    from laser.types import BufferEstimate
    import time
    be = BufferEstimate(free=6000, size=6000, last_refresh_ts=time.monotonic())
    assert be.is_fresh(max_age_ms=100.0)
    be.last_refresh_ts = time.monotonic() - 1.0  # 1s old
    assert not be.is_fresh(max_age_ms=100.0)


def test_event_schemas_import():
    # Skip if numpy not installed locally; the schemas module imports it.
    import importlib.util
    if importlib.util.find_spec("numpy") is None:
        import pytest
        pytest.skip("numpy not installed")
    from events.schemas import (
        FrameEvent, DetectionEvent, TrackEvent,
        TargetCommandEvent, LaserStatusEvent, LatencySample,
    )
    assert all([
        FrameEvent, DetectionEvent, TrackEvent,
        TargetCommandEvent, LaserStatusEvent, LatencySample,
    ])


def test_sensor_base_import_and_role_enum():
    import importlib.util
    if importlib.util.find_spec("numpy") is None:
        import pytest
        pytest.skip("numpy not installed")
    from sensors.base import Sensor, SensorFrame, SensorRole
    assert SensorRole.TARGETING.value == "targeting"
    assert SensorRole.SAFETY.value == "safety"


def test_sensor_normalize_uses_frame_dims():
    import importlib.util
    if importlib.util.find_spec("numpy") is None:
        import pytest
        pytest.skip("numpy not installed")
    from sensors.base import Sensor, SensorFrame
    f = SensorFrame(timestamp=0.0, sensor_id="x", width=1920, height=1080)
    nx, ny = Sensor.normalize(f, 960.0, 540.0)
    # 960/(1920-1) = 0.50026..., 540/(1080-1) = 0.50046...
    assert 0.4 < nx < 0.6
    assert 0.4 < ny < 0.6
