"""Public MuJoCo API; implementations live under ``core`` and ``terrain``."""

from .core import DEFAULT_MODEL_PATH, MuJoCoController, build_terrain_xml, load_model
from .terrain import TERRAIN_CHOICES, TerrainType

__all__ = [
    "DEFAULT_MODEL_PATH",
    "MuJoCoController",
    "TERRAIN_CHOICES",
    "TerrainType",
    "build_terrain_xml",
    "load_model",
]
