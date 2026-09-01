"""Compatibility imports for the former ``non_rl`` gait module.

New code should import :class:`TripodGait` from :mod:`src.locomotion` or
:mod:`src.locomotion.tripod_gait`. This module remains so saved scripts and
external imports do not break during the naming migration.
"""

from .tripod_gait import (
    GaitConfig,
    GaitSample,
    NonRLWalk,
    PhoenixTripodGait,
    TripodGait,
    VelocityCommand,
)

__all__ = [
    "GaitConfig",
    "GaitSample",
    "NonRLWalk",
    "PhoenixTripodGait",
    "TripodGait",
    "VelocityCommand",
]
