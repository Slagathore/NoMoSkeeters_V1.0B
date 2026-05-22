"""sensors.factory — mode/role resolution and manager assembly, no hardware."""
from __future__ import annotations

from typing import Optional

from sensors import factory
from sensors.base import Sensor, SensorFrame, SensorRole


class FakeSensor(Sensor):
    def __init__(self, sid: str, role: SensorRole, open_ok: bool = True):
        self._sid = sid
        self._role = role
        self._open_ok = open_ok
        self.opened = False
        self.closed = False

    @property
    def sensor_id(self) -> str: return self._sid
    @property
    def role(self) -> SensorRole: return self._role
    @property
    def has_rgb(self) -> bool: return True
    @property
    def has_depth(self) -> bool: return False
    @property
    def has_ir(self) -> bool: return False

    def open(self) -> bool:
        self.opened = True
        return self._open_ok

    def close(self) -> None:
        self.closed = True

    def read(self) -> Optional[SensorFrame]:
        return None                       # worker thread just naps


def _make_builder(name: str, open_ok: bool):
    def build(role: SensorRole) -> Sensor:
        return FakeSensor(name, role, open_ok)
    return build


def _builders(**open_ok) -> dict:
    names = ["ov9281", "kinect_v2", "gopro", "phone", "local_cam"]
    return {n: _make_builder(n, open_ok.get(n, True)) for n in names}


# ── resolve_selection ──────────────────────────────────────────────────────

def test_resolve_solo_preset():
    assert factory.resolve_selection("ov9281_only") == {
        "ov9281": SensorRole.TARGETING}


def test_resolve_fusion_preset_assigns_natural_roles():
    sel = factory.resolve_selection("ov9281_kinect")
    assert set(sel) == {"ov9281", "kinect_v2"}
    assert sel["ov9281"] == SensorRole.TARGETING
    assert sel["kinect_v2"] == SensorRole.SAFETY      # Kinect is the safety sensor


def test_resolve_config_uses_roles_and_skips_off():
    sel = factory.resolve_selection(
        "config",
        roles={"ov9281": "targeting", "gopro": "off", "kinect_v2": "auto"})
    assert "gopro" not in sel
    assert sel["ov9281"] == SensorRole.TARGETING
    assert sel["kinect_v2"] == SensorRole.SAFETY      # auto → natural default


def test_unknown_mode_falls_back_to_config():
    sel = factory.resolve_selection("nonsense", roles={"ov9281": "targeting"})
    assert sel == {"ov9281": SensorRole.TARGETING}


# ── build_sensors / build_sensor_manager ─────────────────────────────────

def test_build_sensors_uses_injected_builders():
    sensors = factory.build_sensors("ov9281_kinect", builders=_builders())
    assert sorted(s.sensor_id for s in sensors) == ["kinect_v2", "ov9281"]


def test_build_manager_adds_sensors():
    mgr = factory.build_sensor_manager("ov9281_only", builders=_builders())
    try:
        assert mgr.sensor_ids == ["ov9281"]
    finally:
        mgr.stop_all()


def test_build_manager_skips_sensor_that_fails_to_open():
    mgr = factory.build_sensor_manager("ov9281_kinect",
                                       builders=_builders(kinect_v2=False))
    try:
        assert mgr.sensor_ids == ["ov9281"]           # kinect dropped gracefully
    finally:
        mgr.stop_all()


def test_builder_exception_is_skipped_not_fatal():
    def boom(role: SensorRole) -> Sensor:
        raise RuntimeError("no camera")
    builders = _builders()
    builders["ov9281"] = boom
    sensors = factory.build_sensors("ov9281_kinect", builders=builders)
    assert [s.sensor_id for s in sensors] == ["kinect_v2"]
