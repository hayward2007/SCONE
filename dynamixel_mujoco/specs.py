"""DYNAMIXEL actuator specifications and the MuJoCo constants derived from them.

Every number in :data:`CATALOG` is copied from the ROBOTIS e-Manual. Everything
else on :class:`DynamixelSpec` is derived, so the two can never drift apart.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def rpm(value: float) -> float:
    """Revolutions per minute to radians per second."""
    return value * 2.0 * math.pi / 60.0


def arcmin(value: float) -> float:
    """Arcminutes to radians."""
    return math.radians(value / 60.0)


# A coreless rotor of the size these actuators use. ROBOTIS does not publish
# rotor inertia, so this is an estimate: it puts the mechanical time constant
# in the 10-25 ms band that such motors occupy. Identify it per actuator by
# fitting a no-load step response, which needs no knowledge of this number.
DEFAULT_ROTOR_INERTIA = 1.4e-7


@dataclass(frozen=True)
class DynamixelSpec:
    """One actuator at one supply voltage, plus the MuJoCo constants it implies.

    The e-Manual figures are measured at the **output shaft of the assembled
    actuator**, so gearbox and motor loss is already inside them. That is the
    single most important fact for modelling: see :mod:`dynamixel_mujoco.mjcf`.
    """

    name: str
    volts: float
    stall_torque: float          # N m, at ``volts``
    no_load_speed: float         # rad/s, at ``volts``
    stall_current: float         # A, at ``volts``
    gear_ratio: float
    backlash: float              # rad of total output-shaft play
    mass: float                  # kg
    rotor_inertia: float = DEFAULT_ROTOR_INERTIA

    # -- constants MuJoCo needs ------------------------------------------

    @property
    def torque_constant(self) -> float:
        """K in N m/A, output referred. Also the back-EMF constant in V s/rad."""
        return self.volts / self.no_load_speed

    @property
    def resistance(self) -> float:
        """R in ohms, as MuJoCo's ``dcmotor nominal`` shortcut derives it."""
        return self.torque_constant * self.volts / self.stall_torque

    @property
    def armature(self) -> float:
        """Reflected rotor inertia, in kg m^2. NOT contained in the e-Manual."""
        return self.rotor_inertia * self.gear_ratio ** 2

    @property
    def mechanical_time_constant(self) -> float:
        """tau_m = J R / K^2, in seconds. Invariant under gearing."""
        return self.armature * self.resistance / self.torque_constant ** 2

    # -- derived diagnostics ---------------------------------------------

    @property
    def modelled_stall_current(self) -> float:
        """What V/R predicts. Lower than the sheet: the sheet torque is post-gearbox."""
        return self.volts / self.resistance

    @property
    def gear_efficiency(self) -> float:
        """stall_torque / (K * stall_current): the loss the sheet already absorbs."""
        return self.stall_torque / (self.torque_constant * self.stall_current)

    def critical_damping(self, kp: float, link_inertia: float) -> float:
        """kd for a critically damped torque-space PD, armature included."""
        return 2.0 * math.sqrt(kp * (link_inertia + self.armature))

    def nominal(self) -> str:
        """The ``nominal`` attribute value for a MuJoCo ``dcmotor``."""
        return f"{self.volts:g} {self.stall_torque:g} {self.no_load_speed!r}"


#: ROBOTIS e-Manual, 12 V column. Add an entry only from the manual itself.
CATALOG: dict[str, DynamixelSpec] = {
    "MX-28AT": DynamixelSpec(
        name="MX-28AT", volts=12.0, stall_torque=2.5, no_load_speed=rpm(55.0),
        stall_current=1.4, gear_ratio=193.0, backlash=arcmin(20.0), mass=0.077,
    ),
    "XM430-W350-T": DynamixelSpec(
        name="XM430-W350-T", volts=12.0, stall_torque=4.1, no_load_speed=rpm(46.0),
        stall_current=2.3, gear_ratio=353.5, backlash=arcmin(15.0), mass=0.082,
    ),
    "XM430-W210-T": DynamixelSpec(
        name="XM430-W210-T", volts=12.0, stall_torque=3.0, no_load_speed=rpm(77.0),
        stall_current=2.3, gear_ratio=212.6, backlash=arcmin(15.0), mass=0.082,
    ),
}


def spec(name: str) -> DynamixelSpec:
    try:
        return CATALOG[name]
    except KeyError:
        raise KeyError(
            f"unknown actuator {name!r}; known: {sorted(CATALOG)}"
        ) from None


__all__ = [
    "CATALOG", "DEFAULT_ROTOR_INERTIA", "DynamixelSpec",
    "arcmin", "rpm", "spec",
]
