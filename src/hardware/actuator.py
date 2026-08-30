"""Single source of truth for SCONE actuator metadata."""

from __future__ import annotations

from .actuator_control_table import (
    MX28_AT,
    XM430_W210T,
    XM430_W350T,
    ActuatorModel,
    OperatingMode,
)
from .actuator_index import ActuatorIndex


class Torque:
    OFF = 0
    ON = 1


class Position:
    START = 0
    CENTER = 2048
    END = 4096


class Model:
    MX28_AT = MX28_AT
    XM430_W350T = XM430_W350T
    XM430_W210T = XM430_W210T


def model_for_id(motor_id: int) -> ActuatorModel:
    if motor_id in ActuatorIndex.UPPER:
        return MX28_AT
    if motor_id in ActuatorIndex.MIDDLE:
        return XM430_W350T
    if motor_id in ActuatorIndex.LOWER:
        return XM430_W210T
    raise ValueError(f"actuator ID must be between 1 and 18, got {motor_id}")


class Actuator:
    Index = ActuatorIndex
    Model = Model
    OperatingMode = OperatingMode
    Position = Position
    Torque = Torque


__all__ = [
    "Actuator",
    "ActuatorIndex",
    "Model",
    "OperatingMode",
    "Position",
    "Torque",
    "model_for_id",
]
