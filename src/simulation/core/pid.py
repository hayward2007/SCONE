"""Outer position/velocity PID loop for the MJCF ``<dcmotor>`` actuators.

``model.xml`` drives its 18 actuators with ``input="voltage"`` (the MuJoCo
default for ``<dcmotor>``), so ``data.ctrl`` is a raw terminal voltage rather
than a target angle. This module supplies the missing outer control loop in
Python instead of using dcmotor's built-in ``input="pos vel ff"`` controller,
so the DYNAMIXEL-style position gains stay visible and tunable here rather
than hidden inside the XML.

The control law mirrors exactly what MuJoCo's own dcmotor `controller`
attribute documents for its built-in mode (see the "actuator/dcmotor"
section of https://mujoco.readthedocs.io/en/stable/XMLreference.html and the
referenced DC motor technical note):

    torque  = kp*(target_pos - pos) + kd*(target_vel - vel) + ki*integral + ff
    voltage = (R / K) * torque + K * vel

The second term of the voltage equation compensates back-EMF, so a correctly
tuned loop delivers close to the commanded torque until the supply voltage
or continuous-torque limit is reached -- matching how a DYNAMIXEL's internal
position loop drives its own DC motor.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DCMotorSpec:
    """Electromechanical constants for one DYNAMIXEL model, at rated voltage.

    Mirrors a ``<dcmotor nominal="voltage stall_torque no_load_speed">``
    default class in ``model.xml`` -- keep the two definitions in sync.
    ``K`` and ``R`` are derived with the same formula MuJoCo's compiler uses
    internally for the ``nominal`` shortcut.
    """

    voltage: float           # V, rated/nominal supply voltage
    stall_torque: float      # N*m, at `voltage`
    no_load_speed: float     # rad/s, at `voltage`
    continuous_torque: float  # N*m, saturation torque limit (matches model.xml)

    @property
    def K(self) -> float:
        """Combined torque / back-EMF constant (N*m/A, equivalently V*s/rad)."""
        return self.voltage / self.no_load_speed

    @property
    def R(self) -> float:
        """Terminal resistance (Ohm)."""
        return self.K * self.voltage / self.stall_torque


def _rpm_to_radians_per_second(rpm: float) -> float:
    return rpm * 2.0 * math.pi / 60.0


# Measured/datasheet values at 12 V. Must match model.xml's motor_mx28at /
# motor_xm430_w350 / motor_xm430_w210 <dcmotor nominal="..."> attributes and
# saturation torque limits.
MX28AT = DCMotorSpec(
    voltage=12.0,
    stall_torque=2.5,
    no_load_speed=_rpm_to_radians_per_second(55.0),
    continuous_torque=2.5,
)
XM430_W350 = DCMotorSpec(
    voltage=12.0,
    stall_torque=4.1,
    no_load_speed=_rpm_to_radians_per_second(46.0),
    continuous_torque=4.1,
)
XM430_W210 = DCMotorSpec(
    voltage=12.0,
    stall_torque=3.0,
    no_load_speed=_rpm_to_radians_per_second(77.0),
    continuous_torque=3.0,
)

# Torque-space (kp, kd) pairs, in N*m/rad and N*m*s/rad. kp was originally
# set so a since-superseded, conservative torque ceiling was reached at a 5
# degree position error; kd is set to critical damping for this model's
# per-stage effective inertia. Now that saturation matches stall torque
# (see model.xml), the same kp only asks for that much torque at a much
# larger position error, which is exactly what's needed to recover from a
# large transient error -- see SCONE_RL.md for the full derivation. ki
# defaults to 0 (start with a PD loop; add integral action only if a
# persistent steady-state error shows up).
# 2026-09-02: model.xml gained per-joint ``armature`` (reflected rotor inertia),
# which raised the effective inertia these gains damp by 21, 105 and 250 percent.
# The old kd values then left 0, 4.6 and 13.6 percent step overshoot instead of
# the critical damping this table is meant to provide, so kd was recomputed as
# 2*sqrt(kp*(J_link + armature)) with J_link recovered from the previous kd.
# The pre-armature table, kept rather than discarded so the change is legible
# and reversible. These were critical for the link inertia alone; with the
# reflected rotor inertia present they leave 0, 4.6 and 13.6 percent step
# overshoot, and cost 37 percent more cost of transport on a flat walking trial
# (0.655 -> 0.900 against 0.695 for the recomputed values, five gait phases).
_LINK_ONLY_GAINS: dict[str, tuple[float, float]] = {
    "mx28at": (5.73, 0.752),
    "xm430_w350": (9.40, 0.792),
    "xm430_w210": (6.88, 0.264),
}

_DEFAULT_GAINS: dict[str, tuple[float, float]] = {
    "mx28at": (5.73, 0.828),
    "xm430_w350": (9.40, 1.134),
    "xm430_w210": (6.88, 0.494),
}


def spec_for_motor_id(motor_id: int) -> DCMotorSpec:
    """Return the DCMotorSpec for a hardware motor ID (1-18)."""

    if motor_id <= 6:
        return MX28AT
    if motor_id <= 12:
        return XM430_W350
    return XM430_W210


def default_gains_for_motor_id(motor_id: int) -> tuple[float, float]:
    """Return the default (kp, kd) pair for a hardware motor ID (1-18)."""

    if motor_id <= 6:
        return _DEFAULT_GAINS["mx28at"]
    if motor_id <= 12:
        return _DEFAULT_GAINS["xm430_w350"]
    return _DEFAULT_GAINS["xm430_w210"]


class DCMotorPID:
    """Position(+velocity)-setpoint PID that drives a voltage-input dcmotor.

    One instance controls one joint. Call :meth:`step` once per physics
    step with the joint's current position/velocity and the profile
    generator's current setpoint; it returns the voltage to write into
    ``data.ctrl`` for that actuator.
    """

    def __init__(
        self,
        spec: DCMotorSpec,
        kp: float,
        kd: float = 0.0,
        ki: float = 0.0,
        integral_limit: float | None = None,
        slew_max: float = math.inf,
    ) -> None:
        self.spec = spec
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.integral_limit = (
            spec.continuous_torque if integral_limit is None else integral_limit
        )
        self.slew_max = slew_max
        self._integral = 0.0
        self._slewed_target = 0.0

    def reset(self, position: float) -> None:
        """Clear integral windup and re-seed the slew-limited target.

        Call this whenever the joint is re-enabled or its setpoint is reset
        out-of-band, so the next :meth:`step` does not see a stale error.
        """

        self._integral = 0.0
        self._slewed_target = position

    def step(
        self,
        dt: float,
        position: float,
        velocity: float,
        target_position: float,
        target_velocity: float = 0.0,
        feedforward_torque: float = 0.0,
    ) -> float:
        """Advance one control step and return the commanded voltage (V)."""

        if self.slew_max != math.inf:
            max_step = self.slew_max * dt
            delta = target_position - self._slewed_target
            if delta > max_step:
                delta = max_step
            elif delta < -max_step:
                delta = -max_step
            self._slewed_target += delta
        else:
            self._slewed_target = target_position

        position_error = self._slewed_target - position
        velocity_error = target_velocity - velocity

        if self.ki != 0.0:
            self._integral += position_error * dt
            if self._integral > self.integral_limit:
                self._integral = self.integral_limit
            elif self._integral < -self.integral_limit:
                self._integral = -self.integral_limit

        torque = (
            self.kp * position_error
            + self.kd * velocity_error
            + self.ki * self._integral
            + feedforward_torque
        )

        voltage = (self.spec.R / self.spec.K) * torque + self.spec.K * velocity
        if voltage > self.spec.voltage:
            voltage = self.spec.voltage
        elif voltage < -self.spec.voltage:
            voltage = -self.spec.voltage
        return voltage
