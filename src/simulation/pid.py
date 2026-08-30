"""Compatibility import; implementation moved to :mod:`simulation.core`."""

from .core.pid import (
    DCMotorPID,
    DCMotorSpec,
    MX28AT,
    XM430_W210,
    XM430_W350,
    default_gains_for_motor_id,
    spec_for_motor_id,
)


__all__ = [
    "DCMotorPID",
    "DCMotorSpec",
    "MX28AT",
    "XM430_W210",
    "XM430_W350",
    "default_gains_for_motor_id",
    "spec_for_motor_id",
]
