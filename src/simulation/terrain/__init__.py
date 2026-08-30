"""Procedural terrain API for SCONE MuJoCo simulations."""

from .generator import TerrainGenerator, add_terrain
from .presets import SLOPE_PRESETS, STAIR_PRESETS, TERRAIN_LABELS
from .types import SlopeProfile, StairProfile, TerrainBuildResult, TerrainType


TERRAIN_CHOICES = tuple(item.value for item in TerrainType)


__all__ = [
    "SLOPE_PRESETS",
    "STAIR_PRESETS",
    "TERRAIN_CHOICES",
    "TERRAIN_LABELS",
    "SlopeProfile",
    "StairProfile",
    "TerrainBuildResult",
    "TerrainGenerator",
    "TerrainType",
    "add_terrain",
]
