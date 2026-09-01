"""SCONE implementation package."""

from .main import RobotCommand, RobotStatus, SCONE, UnsupportedCommandError

__all__ = [
    "JointAngles",
    "GaitConfig",
    "GaitSample",
    "LegKinematics",
    "NonRLWalk",
    "PhoenixTripodGait",
    "SconeGait",
    "SconeGaitConfig",
    "TripodGait",
    "RobotCommand",
    "RobotKinematics",
    "RobotStatus",
    "SCONE",
    "SCONEKinematics",
    "UnsupportedCommandError",
    "VelocityCommand",
]


def __getattr__(name: str):
    if name in {
        "GaitConfig",
        "GaitSample",
        "NonRLWalk",
        "PhoenixTripodGait",
        "SconeGait",
        "SconeGaitConfig",
        "TripodGait",
        "VelocityCommand",
    }:
        from .locomotion import (
            GaitConfig,
            GaitSample,
            NonRLWalk,
            PhoenixTripodGait,
            SconeGait,
            SconeGaitConfig,
            TripodGait,
            VelocityCommand,
        )

        return {
            "GaitConfig": GaitConfig,
            "GaitSample": GaitSample,
            "NonRLWalk": NonRLWalk,
            "PhoenixTripodGait": PhoenixTripodGait,
            "SconeGait": SconeGait,
            "SconeGaitConfig": SconeGaitConfig,
            "TripodGait": TripodGait,
            "VelocityCommand": VelocityCommand,
        }[name]
    if name in {
        "JointAngles",
        "LegKinematics",
        "RobotKinematics",
        "SCONEKinematics",
    }:
        from .kinematics import (
            JointAngles,
            LegKinematics,
            RobotKinematics,
            SCONEKinematics,
        )

        return {
            "JointAngles": JointAngles,
            "LegKinematics": LegKinematics,
            "RobotKinematics": RobotKinematics,
            "SCONEKinematics": SCONEKinematics,
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
