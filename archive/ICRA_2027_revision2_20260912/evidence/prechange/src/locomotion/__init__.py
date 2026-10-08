"""Backend-independent SCONE locomotion state machine."""

from .climb import Climb
from .drive import Drive
from .legacy_velocity import LegacyVelocityAdapter, legacy_movement_for
from .mode import Mode
from .tripod_gait import (
    GaitConfig,
    GaitSample,
    NonRLWalk,
    PhoenixTripodGait,
    TripodGait,
    VelocityCommand,
)
from .scone_gait import SconeGait, SconeGaitConfig
from .stair_geometry import (
    ArcWheelGeometry,
    SCONE_V2_ARC_WHEEL,
    legged_wheel_opening_ratio,
    quasi_static_horizontal_push,
    quasi_static_pivot_torque,
    required_friction_coefficient,
    stair_slope,
    support_polygon_margin,
    wheel_edge_offset,
)
from .profile import MotionProfile, SPORT, STANDARD, get_profile
from .walk import Walk

__all__ = [
    "Climb",
    "Drive",
    "ArcWheelGeometry",
    "Mode",
    "MotionProfile",
    "GaitConfig",
    "GaitSample",
    "LegacyVelocityAdapter",
    "NonRLWalk",
    "PhoenixTripodGait",
    "SconeGait",
    "SconeGaitConfig",
    "SCONE_V2_ARC_WHEEL",
    "SPORT",
    "STANDARD",
    "VelocityCommand",
    "TripodGait",
    "Walk",
    "get_profile",
    "legged_wheel_opening_ratio",
    "legacy_movement_for",
    "quasi_static_horizontal_push",
    "quasi_static_pivot_torque",
    "required_friction_coefficient",
    "stair_slope",
    "support_polygon_margin",
    "wheel_edge_offset",
]
