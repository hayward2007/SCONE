"""DYNAMIXEL model metadata used by the SCONE hardware adapter.

Only registers used by this project are defined here. Keeping address and
byte width together prevents model-specific magic numbers from spreading.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


@dataclass(frozen=True)
class Register:
    address: int
    size: int


@dataclass(frozen=True)
class ControlTable:
    torque_enable: Register
    goal_position: Register
    present_position: Register
    moving_speed: Register | None = None
    operating_mode: Register | None = None
    goal_velocity: Register | None = None
    present_velocity: Register | None = None
    profile_velocity: Register | None = None
    profile_acceleration: Register | None = None


@dataclass(frozen=True)
class ActuatorModel:
    name: str
    model_number: int
    protocol_version: float
    position_resolution: int
    table: ControlTable


class OperatingMode(IntEnum):
    VELOCITY = 1
    POSITION = 3
    EXTENDED_POSITION = 4


MX28_AT = ActuatorModel(
    name="MX-28AT",
    model_number=29,
    protocol_version=1.0,
    position_resolution=4096,
    table=ControlTable(
        torque_enable=Register(24, 1),
        goal_position=Register(30, 2),
        moving_speed=Register(32, 2),
        present_position=Register(36, 2),
    ),
)

_XM430_TABLE = ControlTable(
    operating_mode=Register(11, 1),
    torque_enable=Register(64, 1),
    goal_velocity=Register(104, 4),
    profile_acceleration=Register(108, 4),
    profile_velocity=Register(112, 4),
    goal_position=Register(116, 4),
    present_velocity=Register(128, 4),
    present_position=Register(132, 4),
)

XM430_W350T = ActuatorModel(
    name="XM430-W350-T",
    model_number=1020,
    protocol_version=2.0,
    position_resolution=4096,
    table=_XM430_TABLE,
)

XM430_W210T = ActuatorModel(
    name="XM430-W210-T",
    model_number=1030,
    protocol_version=2.0,
    position_resolution=4096,
    table=_XM430_TABLE,
)


__all__ = [
    "ActuatorModel",
    "ControlTable",
    "MX28_AT",
    "OperatingMode",
    "Register",
    "XM430_W210T",
    "XM430_W350T",
]
