"""Simulation-only SCONE gait driven by continuous distal-frame rotation.

The analytical :class:`src.locomotion.SconeGait` remains a bounded 18-position
reference for residual-RL checkpoint compatibility.  This controller uses it
only to generate the small upper/middle stabilising motion and steering
solution.  Motors 13..18 are switched to velocity mode and continuously roll
the sector-shaped end frames, which is the motion the position reference
cannot represent.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

import numpy as np

from ...hardware import Actuator
from ...locomotion import (
    GaitSample,
    MotionProfile,
    SconeGait,
    SconeGaitConfig,
    VelocityCommand,
)
from ...main import SCONE
from .controller import MuJoCoController


@dataclass(frozen=True)
class SconeRollingGaitConfig:
    """Validated flat-ground parameters for continuous sector rolling.

    Values are simulation measurements, not physical-robot safety limits.
    The 72-degree offset de-synchronises the three open sectors in tripod B;
    otherwise all six openings can reach the floor together and drop the body.
    """

    control_frequency: float = 50.0
    roll_velocity: int = 175
    support_velocity_ratio: float = 0.80
    tripod_b_phase_offset_degrees: float = 72.0
    velocity_time_constant: float = 0.10
    profile_velocity: int = 160
    profile_acceleration: int = 50
    middle_stiffness_multiplier: float = 2.0
    cycle_frequency: float = 0.8
    duty_factor: float = 0.64
    step_height: float = 0.004
    max_stride: float = 0.025
    max_lateral_stride: float = 0.025
    max_vx: float = 0.18
    max_vy: float = 0.12
    max_yaw_rate: float = 0.9
    max_steering_degrees: float = 45.0
    steering_blend: float = 0.20
    ik_tolerance: float = 1e-3
    ik_stride_backoff_attempts: int = 4

    def __post_init__(self) -> None:
        if self.control_frequency <= 0.0:
            raise ValueError("control_frequency must be positive")
        if self.roll_velocity <= 0:
            raise ValueError("roll_velocity must be positive")
        if not 0.0 < self.support_velocity_ratio <= 1.0:
            raise ValueError("support_velocity_ratio must be in (0, 1]")
        if not 0.0 <= self.tripod_b_phase_offset_degrees <= 120.0:
            raise ValueError("tripod B phase offset must be in [0, 120] degrees")
        if self.velocity_time_constant < 0.0:
            raise ValueError("velocity_time_constant cannot be negative")
        if self.profile_velocity <= 0 or self.profile_acceleration <= 0:
            raise ValueError("profile velocity and acceleration must be positive")
        if not 0.5 <= self.middle_stiffness_multiplier <= 4.0:
            raise ValueError("middle stiffness multiplier must be in [0.5, 4.0]")
        # Reuse the complete GaitConfig/SconeGaitConfig validation instead of
        # maintaining a second, drift-prone copy here.
        self.planner_config()

    def planner_config(self) -> SconeGaitConfig:
        """Build the bounded IK stabiliser configuration."""

        return SconeGaitConfig(
            control_frequency=self.control_frequency,
            cycle_frequency=self.cycle_frequency,
            duty_factor=self.duty_factor,
            step_height=self.step_height,
            max_stride=self.max_stride,
            max_lateral_stride=self.max_lateral_stride,
            max_vx=self.max_vx,
            max_vy=self.max_vy,
            max_yaw_rate=self.max_yaw_rate,
            command_time_constant=self.velocity_time_constant,
            max_steering_degrees=self.max_steering_degrees,
            steering_blend=self.steering_blend,
            ik_tolerance=self.ik_tolerance,
            ik_stride_backoff_attempts=self.ik_stride_backoff_attempts,
        )


@dataclass(frozen=True)
class SconeRollingSample:
    """One continuous-roll command frame and its analytical planner sample."""

    planner_sample: GaitSample
    lower_velocities: tuple[int, ...]


class SconeRollingGait:
    """Coordinate small tripod body motion with continuously rolling sectors."""

    def __init__(
        self,
        controller: MuJoCoController,
        profile: str | MotionProfile,
        *,
        config: SconeRollingGaitConfig | None = None,
    ) -> None:
        if not isinstance(controller, MuJoCoController):
            raise TypeError("SconeRollingGait requires the MuJoCo controller")
        self.controller = controller
        self.config = config or SconeRollingGaitConfig()
        self.planner = SconeGait(
            controller,
            profile,
            config=self.config.planner_config(),
        )
        self._filtered_lower_velocity = np.zeros(6, dtype=np.float64)
        self._prepared = False
        self._active = False

    def prepare(self) -> dict[int, int]:
        """Tune the model gait path and stagger tripod-B sector openings.

        Returns raw targets so a real-time caller can wait for the MuJoCo
        stepping thread to reach the pose before switching to velocity mode.
        """

        phase_positions = {
            motor_id: float(
                self.planner.profile.lower_initial_position
                + (
                    self.config.tripod_b_phase_offset_degrees
                    if motor_id - 12 in self.planner.TRIPOD_B
                    else 0.0
                )
            )
            for motor_id in Actuator.Index.LOWER
        }
        if any(not 0.0 <= degrees <= 360.0 for degrees in phase_positions.values()):
            raise ValueError(
                "tripod B phase offset exceeds the selected profile's 0..360 "
                "degree position range"
            )
        self.controller.set_all_speed(self.config.profile_velocity)
        self.controller.set_accelerations(
            {
                motor_id: self.config.profile_acceleration
                for motor_id in Actuator.Index.XM
            }
        )
        self.controller.set_gait_position_stiffness(
            self.config.middle_stiffness_multiplier
        )
        self.controller.set_positions(phase_positions)
        self._prepared = True
        return {
            motor_id: self.controller.degrees_to_raw(motor_id, degrees)
            for motor_id, degrees in phase_positions.items()
        }

    def activate(self) -> None:
        """Switch all six sector joints to continuous velocity mode."""

        if not self._prepared:
            raise RuntimeError("prepare SconeRollingGait before activating it")
        self._filtered_lower_velocity.fill(0.0)
        self.controller.set_all_mode(Actuator.OperatingMode.VELOCITY)
        self.controller.set_velocities(
            {motor_id: 0 for motor_id in Actuator.Index.LOWER}
        )
        self._active = True

    def update(
        self,
        command: VelocityCommand,
        dt: float | None = None,
    ) -> SconeRollingSample:
        """Send one stabilising position frame and six rolling velocities."""

        if not self._active:
            raise RuntimeError("prepare and activate SconeRollingGait first")
        if dt is None:
            dt = 1.0 / self.config.control_frequency
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        sample = self.planner.step(command, dt)
        if not sample.converged:
            raise RuntimeError(
                "continuous-roll stabiliser IK failed for legs "
                f"{sample.failed_legs}"
            )
        self.controller.set_positions(
            {
                motor_id: float(sample.motor_degrees[motor_id - 1])
                for motor_id in (*Actuator.Index.UPPER, *Actuator.Index.MIDDLE)
            }
        )

        activity = self.planner._activity(sample.command.as_array())
        target = np.zeros(6, dtype=np.float64)
        for leg in range(1, 7):
            _steering, polarity, alignment = self.planner.steering_solution(
                leg,
                sample.command,
            )
            phase_ratio = (
                self.config.support_velocity_ratio
                if leg in sample.stance_legs
                else 1.0
            )
            target[leg - 1] = (
                -polarity
                * self.config.roll_velocity
                * activity
                * alignment
                * phase_ratio
            )

        tau = self.config.velocity_time_constant
        alpha = 1.0 if tau == 0.0 else 1.0 - math.exp(-dt / tau)
        self._filtered_lower_velocity += alpha * (
            target - self._filtered_lower_velocity
        )
        lower_velocities = tuple(
            int(round(value)) for value in self._filtered_lower_velocity
        )
        self.controller.set_velocities(
            {
                motor_id: lower_velocities[motor_id - 13]
                for motor_id in Actuator.Index.LOWER
            }
        )
        return SconeRollingSample(sample, lower_velocities)

    def stop(self) -> None:
        """Stop the sector wheels without changing other robot state."""

        self._filtered_lower_velocity.fill(0.0)
        self.controller.set_velocities(
            {motor_id: 0 for motor_id in Actuator.Index.LOWER}
        )
        self._active = False


def run_scone_rolling_gait_joystick_cli(
    robot: SCONE,
    *,
    stop_event: threading.Event | None = None,
    config: SconeRollingGaitConfig | None = None,
) -> None:
    """Drive the simulation-only continuous-roll gait from the shared joystick."""

    from ...cli import run_velocity_joystick_cli

    if robot.profile_name == "sport":
        print(
            "[SCONE] continuous-roll scone-gait는 Standard에서 검증됐습니다. "
            "Sport는 접지 여유와 phase 안정성이 검증되지 않았습니다."
        )
    gait = SconeRollingGait(robot.controller, robot.profile, config=config)
    targets = gait.prepare()
    if not gait.controller.wait_until_raw_positions(
        targets,
        tolerance=96,
        timeout=4.0,
    ):
        raise RuntimeError("sector phase staggering did not reach its start pose")
    gait.activate()
    try:
        run_velocity_joystick_cli(
            limits=gait.planner.config,
            apply_command=gait.update,
            profile_name=robot.profile_name,
            control_name="scone-gait/continuous-roll",
            control_hint="13..18번 말단 프레임은 위치 왕복이 아니라 연속 회전합니다.",
            stop_event=stop_event,
        )
    finally:
        gait.stop()


__all__ = [
    "SconeRollingGait",
    "SconeRollingGaitConfig",
    "SconeRollingSample",
    "run_scone_rolling_gait_joystick_cli",
]
