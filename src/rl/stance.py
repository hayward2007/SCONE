"""Lightweight standing-pose definitions shared by RL launchers and environments."""

from __future__ import annotations

import math
from collections.abc import Sequence


UPPER_STANDING_DEGREES = (135.0, 135.0, 180.0, 180.0, 225.0, 225.0)
SPORT_STANDING_DEGREES = UPPER_STANDING_DEGREES + (170.0,) * 6 + (195.0,) * 6
STANDARD_STANDING_DEGREES = UPPER_STANDING_DEGREES + (240.0,) * 6 + (255.0,) * 6

STANCE_PRESETS = {
    "standard": STANDARD_STANDING_DEGREES,
    "sport": SPORT_STANDING_DEGREES,
}


def validate_standing_pose(value: Sequence[float]) -> tuple[float, ...]:
    """Validate and normalize actuator IDs 1..18 expressed in motor degrees."""

    degrees = tuple(float(item) for item in value)
    if len(degrees) != 18:
        raise ValueError("standing pose must contain actuator degrees for IDs 1..18")
    if not all(math.isfinite(item) for item in degrees):
        raise ValueError("standing pose degrees must be finite")
    if not all(0.0 <= item <= 360.0 for item in degrees):
        raise ValueError("standing pose degrees must be between 0 and 360")
    return degrees


__all__ = [
    "SPORT_STANDING_DEGREES",
    "STANDARD_STANDING_DEGREES",
    "STANCE_PRESETS",
    "UPPER_STANDING_DEGREES",
    "validate_standing_pose",
]
