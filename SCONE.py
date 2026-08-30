"""Stable public module: ``import SCONE``."""

from src.main import RobotCommand, RobotStatus, SCONE, UnsupportedCommandError

__all__ = [
    "JointAngles",
    "GaitConfig",
    "GaitSample",
    "LegKinematics",
    "NonRLWalk",
    "PhoenixTripodGait",
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
        "VelocityCommand",
    }:
        from src.locomotion import (
            GaitConfig,
            GaitSample,
            NonRLWalk,
            PhoenixTripodGait,
            VelocityCommand,
        )

        return {
            "GaitConfig": GaitConfig,
            "GaitSample": GaitSample,
            "NonRLWalk": NonRLWalk,
            "PhoenixTripodGait": PhoenixTripodGait,
            "VelocityCommand": VelocityCommand,
        }[name]
    if name in {
        "JointAngles",
        "LegKinematics",
        "RobotKinematics",
        "SCONEKinematics",
    }:
        from src.kinematics import (
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


if __name__ == "__main__":
    from src.cli import main

    raise SystemExit(main())
