"""ShotPattern — generates the LaserPoint stream that "hits" a target.

The cube scans continuously; we never park the galvo. To dwell on a target
at (galvo_x, galvo_y) we emit a stream of samples centered on it while
keeping the galvos moving in a controlled figure.

Reference: BOOTSTRAP_AMENDMENTS.md §9.11.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod

from config import settings
from laser.types import LaserPoint


def _sample_count(dwell_ms: int, dac_rate: int) -> int:
    """Number of samples for a dwell — at least 1."""
    return max(1, int((dwell_ms / 1000.0) * dac_rate))


def _rgb_level(power_pct: int) -> int:
    """0-100 power → 12-bit RGB level."""
    return max(0, min(0xFFF, (power_pct * 0xFFF) // 100))


class ShotPattern(ABC):
    """Generates a stream of LaserPoints centered on a target."""

    pattern_id: str = "abstract"

    @abstractmethod
    def generate(self, target_x_galvo: int, target_y_galvo: int,
                 dwell_ms: int, dac_rate: int,
                 power_pct: int) -> list[LaserPoint]: ...


class DotRepeat(ShotPattern):
    """Park the galvo on one spot — maximum energy density, bad for galvo
    health. Short bursts only; default OFF in favor of MicroCircle."""

    pattern_id = "dot_repeat"

    def generate(self, x, y, dwell_ms, dac_rate, power_pct):
        n = _sample_count(dwell_ms, dac_rate)
        rgb = _rgb_level(power_pct)
        return [LaserPoint(x=x, y=y, r=rgb, g=rgb, b=rgb) for _ in range(n)]


class MicroCircle(ShotPattern):
    """Tight circle around the target — keeps the galvos moving and spreads
    optical energy over a small radius. Default."""

    pattern_id = "micro_circle"

    def __init__(self, radius_galvo: int = 30):
        self.radius = radius_galvo

    def generate(self, x, y, dwell_ms, dac_rate, power_pct):
        n = _sample_count(dwell_ms, dac_rate)
        rgb = _rgb_level(power_pct)
        return [
            LaserPoint(
                x=int(x + self.radius * math.cos(2 * math.pi * i / n)),
                y=int(y + self.radius * math.sin(2 * math.pi * i / n)),
                r=rgb, g=rgb, b=rgb,
            )
            for i in range(n)
        ]


class FigureEight(ShotPattern):
    """Lemniscate around the target — spreads energy wider than a circle,
    useful against an evading target."""

    pattern_id = "figure_eight"

    def __init__(self, scale_galvo: int = 40):
        self.scale = scale_galvo

    def generate(self, x, y, dwell_ms, dac_rate, power_pct):
        n = _sample_count(dwell_ms, dac_rate)
        rgb = _rgb_level(power_pct)
        out = []
        for i in range(n):
            t = 2 * math.pi * i / n
            denom = 1 + math.sin(t) ** 2          # Lemniscate of Bernoulli
            dx = self.scale * math.cos(t) / denom
            dy = self.scale * math.sin(t) * math.cos(t) / denom
            out.append(LaserPoint(x=int(x + dx), y=int(y + dy),
                                  r=rgb, g=rgb, b=rgb))
        return out


# Registry — config selects which to use.
SHOT_PATTERNS: dict[str, ShotPattern] = {
    "dot_repeat":   DotRepeat(),
    "micro_circle": MicroCircle(radius_galvo=settings.SHOT_PATTERN_CIRCLE_RADIUS_GALVO),
    "figure_eight": FigureEight(scale_galvo=settings.SHOT_PATTERN_FIGURE8_SCALE_GALVO),
}


def get_shot_pattern(name: str | None = None) -> ShotPattern:
    """Resolve a pattern name (config SHOT_PATTERN_DEFAULT) to its instance."""
    name = name or settings.SHOT_PATTERN_DEFAULT
    try:
        return SHOT_PATTERNS[name]
    except KeyError:
        raise ValueError(f"unknown shot pattern: {name!r}") from None
