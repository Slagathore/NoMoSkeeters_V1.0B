"""Sensor ABC + SensorFrame dataclass + SensorRole enum.

Reference: BOOTSTRAP.md §5.1, BOOTSTRAP_AMENDMENTS.md §5.1, §5.6.
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import numpy as np


class SensorRole(str, Enum):
    """Architectural job. Set per-sensor in config; drives gating + fusion."""
    TARGETING = "targeting"
    SAFETY    = "safety"
    FALLBACK  = "fallback"
    OFF       = "off"


@dataclass
class SensorFrame:
    """One frame from a sensor. Carries the timestamp and the frame's actual
    dimensions (which may not match the sensor's nominal dims if the sensor
    renegotiated resolution mid-stream)."""
    timestamp: float
    sensor_id: str
    sensor_role: str = SensorRole.TARGETING.value
    rgb: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None
    ir: Optional[np.ndarray] = None
    width: int = 0
    height: int = 0
    timestamp_uncertainty_ms: float = 5.0

    @classmethod
    def now(cls, sensor_id: str, role: SensorRole = SensorRole.TARGETING) -> "SensorFrame":
        return cls(timestamp=time.monotonic(), sensor_id=sensor_id, sensor_role=role.value)


class Sensor(ABC):
    """Abstract base for all sensors."""

    @property
    @abstractmethod
    def sensor_id(self) -> str: ...

    @property
    @abstractmethod
    def role(self) -> SensorRole: ...

    @property
    @abstractmethod
    def has_rgb(self) -> bool: ...

    @property
    @abstractmethod
    def has_depth(self) -> bool: ...

    @property
    @abstractmethod
    def has_ir(self) -> bool: ...

    @abstractmethod
    def open(self) -> bool: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def read(self) -> Optional[SensorFrame]: ...

    @staticmethod
    def normalize(frame: SensorFrame, x_px: float, y_px: float) -> Tuple[float, float]:
        return (
            x_px / max(1, frame.width - 1),
            y_px / max(1, frame.height - 1),
        )

    def world_position(
        self, x_px: int, y_px: int, depth_m: float
    ) -> Optional[Tuple[float, float, float]]:
        return None
