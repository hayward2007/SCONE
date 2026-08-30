"""Typed terrain definitions independent of MuJoCo model loading."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class TerrainType(str, Enum):
    FLAT = "flat"
    UNEVEN = "uneven"
    STAIRS_1 = "stairs-1"
    STAIRS_2 = "stairs-2"
    STAIRS_3 = "stairs-3"
    SLOPE_1 = "slope-1"
    SLOPE_2 = "slope-2"
    SLOPE_3 = "slope-3"
    MIXED = "mixed"

    @classmethod
    def parse(cls, value: "TerrainType | str") -> "TerrainType":
        if isinstance(value, cls):
            return value
        try:
            return cls(value.lower())
        except ValueError as error:
            choices = ", ".join(item.value for item in cls)
            raise ValueError(f"unknown terrain {value!r}; choose from {choices}") from error


@dataclass(frozen=True)
class StairProfile:
    """Per-step dimensions for an ascending staircase.

    Each tuple entry describes one physical step. ``rises`` are incremental
    height increases, not absolute top heights. This allows every step to use
    a different rise, tread depth, and width.
    """

    rises: tuple[float, ...]
    tread_depths: tuple[float, ...]
    widths: tuple[float, ...]
    landing_length: float = 0.60

    def __post_init__(self) -> None:
        count = len(self.rises)
        if count == 0 or len(self.tread_depths) != count or len(self.widths) != count:
            raise ValueError("stair rises, tread_depths, and widths must have equal length")
        if min(self.rises + self.tread_depths + self.widths) <= 0.0:
            raise ValueError("all stair dimensions must be positive")
        if self.landing_length <= 0.0:
            raise ValueError("stair landing_length must be positive")

    @property
    def total_height(self) -> float:
        return float(sum(self.rises))


@dataclass(frozen=True)
class SlopeProfile:
    angle_degrees: float
    length: float
    width: float
    landing_length: float = 0.60
    thickness: float = 0.06

    def __post_init__(self) -> None:
        if not 0.0 < self.angle_degrees < 45.0:
            raise ValueError("slope angle must be between 0 and 45 degrees")
        if min(self.length, self.width, self.landing_length, self.thickness) <= 0.0:
            raise ValueError("all slope dimensions must be positive")


@dataclass(frozen=True)
class TerrainBuildResult:
    terrain: TerrainType
    geom_names: tuple[str, ...]
    start_y: float
    end_y: float
    max_height: float


__all__ = [
    "SlopeProfile",
    "StairProfile",
    "TerrainBuildResult",
    "TerrainType",
]
