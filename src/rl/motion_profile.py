"""Adapt a selected RL standing pose to the legacy mode state machine."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import replace

from src.locomotion.profile import MotionProfile, SPORT, STANDARD

from .stance import (
    SPORT_STANDING_DEGREES,
    STANDARD_STANDING_DEGREES,
    validate_standing_pose,
)


def motion_profile_for_standing_pose(value: Sequence[float]) -> MotionProfile:
    """Build the nearest legacy motion profile around an RL standing pose."""

    degrees = validate_standing_pose(value)
    candidates = (
        (STANDARD, STANDARD_STANDING_DEGREES),
        (SPORT, SPORT_STANDING_DEGREES),
    )
    base, _ = min(
        candidates,
        key=lambda item: sum(
            (actual - preset) ** 2
            for actual, preset in zip(degrees, item[1], strict=True)
        ),
    )
    return replace(
        base,
        name=f"rl-{base.name}",
        upper_initial_position=tuple(degrees[:6]),
        middle_initial_position=sum(degrees[6:12]) / 6.0,
        lower_initial_position=sum(degrees[12:18]) / 6.0,
    )


__all__ = ["motion_profile_for_standing_pose"]
