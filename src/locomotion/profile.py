"""Named, immutable posture/speed profiles for SCONE."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionProfile:
    name: str
    upper_initial_position: tuple[float, ...]
    middle_initial_position: float
    lower_initial_position: float
    boost_speed: int
    safety_speed: int
    walking_speed: int
    driving_speed: int
    climbing_speed: int


STANDARD = MotionProfile(
    name="standard",
    upper_initial_position=(135, 135, 180, 180, 225, 225),
    middle_initial_position=240,
    lower_initial_position=255,
    boost_speed=150,
    safety_speed=50,
    walking_speed=100,
    driving_speed=150,
    climbing_speed=200,
)

SPORT = MotionProfile(
    name="sport",
    upper_initial_position=(135, 135, 180, 180, 225, 225),
    middle_initial_position=170,
    lower_initial_position=195,
    boost_speed=150,
    safety_speed=50,
    walking_speed=100,
    driving_speed=150,
    climbing_speed=100,
)

PROFILES = {profile.name: profile for profile in (STANDARD, SPORT)}


def get_profile(name: str) -> MotionProfile:
    try:
        return PROFILES[name.lower()]
    except KeyError as error:
        raise ValueError(f"unknown SCONE profile: {name!r}") from error


__all__ = ["MotionProfile", "PROFILES", "SPORT", "STANDARD", "get_profile"]
