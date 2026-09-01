"""Compatibility import; implementation moved to :mod:`simulation.core`."""

from .core.simulator_cli import (
    build_parser,
    main,
    select_rl_checkpoint,
    select_simulation_control,
    select_stair_demo_strategy,
    select_stair_terrain,
    select_terrain,
)


__all__ = [
    "build_parser",
    "main",
    "select_rl_checkpoint",
    "select_simulation_control",
    "select_stair_demo_strategy",
    "select_stair_terrain",
    "select_terrain",
]
