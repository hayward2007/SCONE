"""Compatibility import; implementation moved to :mod:`simulation.core`."""

from .core.cli_bridge import SimulationControl, run


__all__ = ["SimulationControl", "run"]
