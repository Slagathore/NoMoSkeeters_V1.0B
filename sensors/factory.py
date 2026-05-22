"""Sensor factory — build the active sensor set from config.

This is the wiring between `settings.TARGETING_MODE` / `settings.SENSOR_ROLES`
and a live `SensorManager`. It is the one place that knows how to construct each
sensor type, so callers (tools, the future orchestrator) just ask for a mode.

- **Solo modes** run a single targeting sensor (`ov9281_only`, `kinect_only`,
  `gopro_only`, `phone_only`, `local_only`).
- **Fusion modes** run several (`ov9281_kinect`, `phone_kinect`, …); the cross-
  sensor fusion layer (targeting/) combines their detections via the calibrated
  extrinsics. The factory just makes the streams available on the bus.
- **"config"** ignores the preset and builds exactly what `SENSOR_ROLES` says.

Sensor construction is lazy (imported only when selected) so a missing optional
dependency — pykinect2, the phone app being offline — never breaks the factory,
and injectable (the `builders` arg) so tests run without any hardware.

Reference: settings §3.3/§5, sensors/sensor_manager.py.
"""
from __future__ import annotations

import logging
from typing import Callable, Dict, List, Optional

from config import settings
from sensors.base import Sensor, SensorRole
from sensors.sensor_manager import FrameSink, SensorManager, StatusSink

_log = logging.getLogger(__name__)

SensorBuilder = Callable[[SensorRole], Sensor]

# Named presets → the set of sensors to run. Order matters only for logging.
MODE_PRESETS: Dict[str, List[str]] = {
    # Solo targeting.
    "gopro_only":   ["gopro"],
    "kinect_only":  ["kinect_v2"],
    "ov9281_only":  ["ov9281"],
    "phone_only":   ["phone"],
    "local_only":   ["local_cam"],
    # Fusion (targeting camera + Kinect depth/IR safety & world-attribution).
    "ov9281_kinect": ["ov9281", "kinect_v2"],
    "phone_kinect":  ["phone", "kinect_v2"],
    "gopro_kinect":  ["gopro", "kinect_v2"],
    "fused_interp":  ["gopro", "kinect_v2"],   # legacy v0.2.1 name
}

# Each sensor's natural role when a preset (not an explicit role) selects it.
_DEFAULT_ROLE: Dict[str, SensorRole] = {
    "gopro":     SensorRole.TARGETING,
    "ov9281":    SensorRole.TARGETING,
    "phone":     SensorRole.TARGETING,
    "local_cam": SensorRole.FALLBACK,
    "kinect_v2": SensorRole.SAFETY,
}


# ── Lazy real builders ─────────────────────────────────────────────────────

def _build_ov9281(role: SensorRole) -> Sensor:
    from sensors.ov9281 import OV9281Sensor
    return OV9281Sensor(role=role)


def _build_kinect(role: SensorRole) -> Sensor:
    from sensors.kinect_v2 import KinectV2Sensor
    return KinectV2Sensor()                 # role is fixed (SAFETY) internally


def _build_gopro(role: SensorRole) -> Sensor:
    from sensors.gopro import GoProSensor
    from sensors.gopro_interface import GoProInterface
    return GoProSensor(control=GoProInterface(), role=role)


def _build_phone(role: SensorRole) -> Sensor:
    from sensors.phone import PhoneSensor
    return PhoneSensor(role=role)


def _build_local(role: SensorRole) -> Sensor:
    from sensors.local_cam import LocalCamSensor
    return LocalCamSensor(role=role)


def default_builders() -> Dict[str, SensorBuilder]:
    return {
        "ov9281":    _build_ov9281,
        "kinect_v2": _build_kinect,
        "gopro":     _build_gopro,
        "phone":     _build_phone,
        "local_cam": _build_local,
    }


# ── Selection resolution ───────────────────────────────────────────────────

def _role_for(value: str, name: str) -> SensorRole:
    """A SENSOR_ROLES value → SensorRole. 'auto' / unknown → the sensor's
    natural default."""
    value = (value or "auto").lower()
    for r in SensorRole:
        if value == r.value:
            return r
    return _DEFAULT_ROLE.get(name, SensorRole.TARGETING)


def resolve_selection(mode: Optional[str] = None,
                      roles: Optional[Dict[str, str]] = None
                      ) -> Dict[str, SensorRole]:
    """Resolve a mode (or the config role map) into {sensor_name: SensorRole}.

    A preset wins if given; otherwise `roles` (defaults to settings.SENSOR_ROLES)
    is used, building every sensor not set to "off"."""
    mode = mode or settings.TARGETING_MODE
    if mode in MODE_PRESETS:
        return {name: _DEFAULT_ROLE.get(name, SensorRole.TARGETING)
                for name in MODE_PRESETS[mode]}
    if mode != "config":
        _log.warning("unknown TARGETING_MODE %r; falling back to SENSOR_ROLES", mode)
    role_map = roles if roles is not None else settings.SENSOR_ROLES
    return {name: _role_for(value, name)
            for name, value in role_map.items()
            if (value or "").lower() != "off"}


# ── Build ────────────────────────────────────────────────────────────────

def build_sensors(mode: Optional[str] = None,
                  *,
                  roles: Optional[Dict[str, str]] = None,
                  builders: Optional[Dict[str, SensorBuilder]] = None
                  ) -> List[Sensor]:
    """Construct (but do not open) the selected sensors. Unknown names and
    builder failures are logged and skipped."""
    builders = builders or default_builders()
    selection = resolve_selection(mode, roles)
    out: List[Sensor] = []
    for name, role in selection.items():
        builder = builders.get(name)
        if builder is None:
            _log.warning("no builder for sensor %r; skipping", name)
            continue
        try:
            out.append(builder(role))
        except Exception:
            _log.exception("building sensor %r failed; skipping", name)
    return out


def build_sensor_manager(mode: Optional[str] = None,
                         *,
                         roles: Optional[Dict[str, str]] = None,
                         builders: Optional[Dict[str, SensorBuilder]] = None,
                         frame_sink: Optional[FrameSink] = None,
                         status_sink: Optional[StatusSink] = None,
                         manager: Optional[SensorManager] = None
                         ) -> SensorManager:
    """Build the selected sensors and add each to a SensorManager (opening it).
    Sensors that fail to open are skipped — a 'fusion' run degrades gracefully
    to whatever actually came up. Returns the manager (possibly empty)."""
    mgr = manager or SensorManager(frame_sink=frame_sink, status_sink=status_sink)
    sensors = build_sensors(mode, roles=roles, builders=builders)
    added = 0
    for sensor in sensors:
        if mgr.add_sensor(sensor):
            added += 1
        else:
            _log.warning("sensor %s did not come up; skipping", sensor.sensor_id)
    _log.info("sensor manager: %d/%d sensors active (mode=%s) -> %s",
              added, len(sensors), mode or settings.TARGETING_MODE, mgr.sensor_ids)
    return mgr
