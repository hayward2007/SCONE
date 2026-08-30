"""Compatibility import; implementation moved to :mod:`simulation.core`."""

from .core.model import DEFAULT_MODEL_PATH, build_terrain_xml, load_model


__all__ = ["DEFAULT_MODEL_PATH", "build_terrain_xml", "load_model"]
