"""Controller adapters used by the paper benchmarks."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Protocol

import numpy as np

from src.hardware import Actuator
from src.locomotion import (
    GaitConfig,
    SconeGait,
    SconeGaitV2,
    SconeGaitV2Config,
    TripodGait,
    VelocityCommand,
)
from src.simulation.core.cli_bridge import (
    SCONE_GAIT_SIMULATION_CONFIG,
    SCONE_GAIT_V2_SIMULATION_CONFIG,
    TRIPOD_GAIT_SIMULATION_CONFIG,
    configure_model_gait_controller,
)
from src.simulation.core.scone_rolling_gait import RollGait, RollGaitConfig

from .common import MetricsRecorder, SimulationTrial


CONTROLLER_CHOICES = (
    "articulated-walk",
    "distal-only-roll",
    "full-roll",
    "bounded-scone",
    "role-split-scone",
    "rear-pair-roll",
    "front-pair-roll",
    "no-roll-baseline",
    "fast-articulated-walk",
    "matched-articulated",
    "matched-distal-only",
    "matched-coordinated",
)

MATCHED_CONTROLLER_CHOICES = (
    "matched-articulated",
    "matched-distal-only",
    "matched-coordinated",
)
MATCHED_ROLL_CONFIG = RollGaitConfig()


@dataclass(frozen=True)
class ControlDiagnostics:
    converged: bool = True
    stride_clip_fraction: float = 0.0
    ik_backoff_scale: float = 1.0


class BenchmarkController(Protocol):
    name: str

    def prepare(
        self,
        trial: SimulationTrial,
        *,
        recorder: MetricsRecorder | None = None,
    ) -> None: ...

    def update(self, command: VelocityCommand, dt: float) -> ControlDiagnostics: ...

    def stop(self) -> None: ...


def _diagnostics(sample) -> ControlDiagnostics:
    return ControlDiagnostics(
        converged=bool(sample.converged),
        stride_clip_fraction=float(sample.stride_clip_fraction),
        ik_backoff_scale=float(sample.ik_backoff_scale),
    )


def _phase_pose(planner: SconeGait, config: RollGaitConfig) -> np.ndarray:
    pose = planner.nominal_motor_degrees
    for leg in planner.TRIPOD_B:
        pose[11 + leg] += config.tripod_b_phase_offset_degrees
    return pose


def _configure_matched_position_path(
    trial: SimulationTrial,
    config: RollGaitConfig,
) -> None:
    trial.controller.set_all_speed(config.profile_velocity)
    trial.controller.set_accelerations(
        {motor_id: config.profile_acceleration for motor_id in Actuator.Index.XM}
    )
    trial.controller.set_gait_position_stiffness(
        config.middle_stiffness_multiplier
    )


def _acquire_phase_pose(
    trial: SimulationTrial,
    planner: SconeGait,
    config: RollGaitConfig,
    *,
    recorder: MetricsRecorder | None,
) -> np.ndarray:
    pose = _phase_pose(planner, config)
    _configure_matched_position_path(trial, config)
    trial.controller.set_positions(
        {motor_id: float(pose[motor_id - 1]) for motor_id in Actuator.Index.ALL}
    )
    raw_targets = {
        motor_id: trial.controller.degrees_to_raw(motor_id, float(pose[motor_id - 1]))
        for motor_id in Actuator.Index.ALL
    }
    if not trial.wait_until_raw_positions(
        raw_targets,
        tolerance=96,
        timeout=4.0,
        recorder=recorder,
    ):
        raise RuntimeError("matched controller phase pose did not settle")
    return pose


class ArticulatedWalkController:
    name = "articulated-walk"

    def __init__(
        self,
        trial: SimulationTrial,
        *,
        phase: float = 0.0,
        config: GaitConfig | None = None,
        name: str | None = None,
    ) -> None:
        if name is not None:
            self.name = name
        self.trial = trial
        self.phase = phase
        self.config = config or TRIPOD_GAIT_SIMULATION_CONFIG
        self.gait: TripodGait | None = None

    def prepare(
        self,
        trial: SimulationTrial,
        *,
        recorder: MetricsRecorder | None = None,
    ) -> None:
        del recorder
        configure_model_gait_controller(trial.controller)
        self.gait = TripodGait(
            trial.controller,
            trial.robot.profile,
            config=self.config,
        )
        self.gait.reset(phase=self.phase)

    def update(self, command: VelocityCommand, dt: float) -> ControlDiagnostics:
        assert self.gait is not None
        sample = self.gait.update(command, dt=dt, send=True)
        return _diagnostics(sample)

    def stop(self) -> None:
        if self.gait is not None:
            self.gait.update(VelocityCommand(), dt=0.02, send=True)


class DistalOnlyRollController:
    """Lock proximal targets while only the six C-frames rotate."""

    name = "distal-only-roll"

    def __init__(
        self,
        trial: SimulationTrial,
        *,
        phase: float = 0.0,
        config: RollGaitConfig | None = None,
        recalibrate_phase_pose: bool = False,
    ) -> None:
        self.trial = trial
        self.phase = phase
        self.config = config or RollGaitConfig()
        self.recalibrate_phase_pose = recalibrate_phase_pose
        self.planner = SconeGait(
            trial.controller,
            trial.robot.profile,
            config=self.config.planner_config(),
        )
        self.planner.reset(phase=phase)
        self._filtered_velocity = np.zeros(6, dtype=np.float64)
        self._active = False

    def prepare(
        self,
        trial: SimulationTrial,
        *,
        recorder: MetricsRecorder | None = None,
    ) -> None:
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
        trial.controller.set_all_speed(self.config.profile_velocity)
        trial.controller.set_accelerations(
            {
                motor_id: self.config.profile_acceleration
                for motor_id in Actuator.Index.XM
            }
        )
        trial.controller.set_gait_position_stiffness(
            self.config.middle_stiffness_multiplier
        )
        trial.controller.set_positions(phase_positions)
        raw_targets = {
            motor_id: trial.controller.degrees_to_raw(motor_id, degrees)
            for motor_id, degrees in phase_positions.items()
        }
        if not trial.wait_until_raw_positions(
            raw_targets,
            tolerance=96,
            timeout=4.0,
            recorder=recorder,
        ):
            raise RuntimeError("distal-only phase staggering did not settle")
        if self.recalibrate_phase_pose:
            pose = self.planner.nominal_motor_degrees
            for leg in self.planner.TRIPOD_B:
                pose[11 + leg] += self.config.tripod_b_phase_offset_degrees
            self.planner.reset(phase=self.phase, motor_degrees=pose)
        trial.controller.set_all_mode(Actuator.OperatingMode.VELOCITY)
        trial.controller.set_velocities(
            {motor_id: 0 for motor_id in Actuator.Index.LOWER}
        )
        self._filtered_velocity.fill(0.0)
        self._active = True

    def update(self, command: VelocityCommand, dt: float) -> ControlDiagnostics:
        if not self._active:
            raise RuntimeError("prepare distal-only controller first")
        sample = self.planner.step(command, dt)
        activity = max(
            abs(sample.command.vx) / self.config.max_vx,
            abs(sample.command.vy) / self.config.max_vy,
            abs(sample.command.yaw_rate) / self.config.max_yaw_rate,
        )
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
        self._filtered_velocity += alpha * (target - self._filtered_velocity)
        trial_velocities = tuple(int(round(value)) for value in self._filtered_velocity)
        self.trial.controller.set_velocities(
            {
                motor_id: trial_velocities[motor_id - 13]
                for motor_id in Actuator.Index.LOWER
            }
        )
        return _diagnostics(sample)

    def stop(self) -> None:
        self._filtered_velocity.fill(0.0)
        self.trial.controller.set_velocities(
            {motor_id: 0 for motor_id in Actuator.Index.LOWER}
        )
        self._active = False


class FullRollController:
    name = "full-roll"

    def __init__(
        self,
        trial: SimulationTrial,
        *,
        phase: float = 0.0,
        config: RollGaitConfig | None = None,
        recalibrate_phase_pose: bool = False,
    ) -> None:
        self.trial = trial
        self.phase = phase
        self.recalibrate_phase_pose = recalibrate_phase_pose
        self.gait = RollGait(
            trial.controller,
            trial.robot.profile,
            config=config,
        )
        self.gait.planner.reset(phase=phase)

    def prepare(
        self,
        trial: SimulationTrial,
        *,
        recorder: MetricsRecorder | None = None,
    ) -> None:
        targets = self.gait.prepare()
        if not trial.wait_until_raw_positions(
            targets,
            tolerance=96,
            timeout=4.0,
            recorder=recorder,
        ):
            raise RuntimeError("full-roll phase staggering did not settle")
        if self.recalibrate_phase_pose:
            pose = _phase_pose(self.gait.planner, self.gait.config)
            self.gait.planner.reset(phase=self.phase, motor_degrees=pose)
        self.gait.activate()

    def update(self, command: VelocityCommand, dt: float) -> ControlDiagnostics:
        sample = self.gait.update(command, dt)
        return _diagnostics(sample.planner_sample)

    def stop(self) -> None:
        self.gait.stop()


class BoundedSconeController:
    name = "bounded-scone"

    def __init__(self, trial: SimulationTrial, *, phase: float = 0.0) -> None:
        self.gait = SconeGait(
            trial.controller,
            trial.robot.profile,
            config=SCONE_GAIT_SIMULATION_CONFIG,
        )
        self.gait.reset(phase=phase)

    def prepare(
        self,
        trial: SimulationTrial,
        *,
        recorder: MetricsRecorder | None = None,
    ) -> None:
        del trial, recorder

    def update(self, command: VelocityCommand, dt: float) -> ControlDiagnostics:
        sample = self.gait.update(command, dt=dt, send=True)
        return _diagnostics(sample)

    def stop(self) -> None:
        self.gait.update(VelocityCommand(), dt=0.02, send=True)


# Two-leg drives: only the named pair rolls and the other four walk.  Legs 1
# and 2 trail a forward command and already supply most of the push when the
# robot only walks; legs 5 and 6 lead it and are the pair that brakes.
REAR_PAIR_ROLL_CONFIG = replace(SCONE_GAIT_V2_SIMULATION_CONFIG, roll_legs=(1, 2))
FRONT_PAIR_ROLL_CONFIG = replace(SCONE_GAIT_V2_SIMULATION_CONFIG, roll_legs=(5, 6))
# The control for every rolling claim: the same scheduler, the same stride
# budget, the same cadence, with no leg allowed to roll.  Any difference
# against it is the rolling and nothing else.
NO_ROLL_CONFIG = replace(SCONE_GAIT_V2_SIMULATION_CONFIG, roll_legs=())
# The same budget given to the plain tripod walker, which is a different
# code path and therefore a second, independent control.
FAST_WALK_CONFIG = GaitConfig(
    cycle_frequency=SCONE_GAIT_V2_SIMULATION_CONFIG.cycle_frequency,
    duty_factor=SCONE_GAIT_V2_SIMULATION_CONFIG.duty_factor,
    step_height=SCONE_GAIT_V2_SIMULATION_CONFIG.step_height,
    max_stride=SCONE_GAIT_V2_SIMULATION_CONFIG.max_stride,
    max_lateral_stride=SCONE_GAIT_V2_SIMULATION_CONFIG.max_lateral_stride,
    max_vx=SCONE_GAIT_V2_SIMULATION_CONFIG.max_vx,
    max_vy=SCONE_GAIT_V2_SIMULATION_CONFIG.max_vy,
    ik_tolerance=1e-3,
    ik_stride_backoff_attempts=4,
)


class RoleSplitSconeController:
    """scone-gait-v2: corner legs roll on steered sectors, middle legs walk."""

    name = "role-split-scone"

    def __init__(
        self,
        trial: SimulationTrial,
        *,
        phase: float = 0.0,
        config: "SconeGaitV2Config | None" = None,
        name: str | None = None,
    ) -> None:
        if name is not None:
            self.name = name
        self.gait = SconeGaitV2(
            trial.controller,
            trial.robot.profile,
            config=config or SCONE_GAIT_V2_SIMULATION_CONFIG,
        )
        self.gait.reset(phase=phase)

    def prepare(
        self,
        trial: SimulationTrial,
        *,
        recorder: MetricsRecorder | None = None,
    ) -> None:
        # The gait stands with every sector at the centre of its tread, which
        # is about 88 degrees from where the profile parks it.  Rolling there
        # at the unlimited gait speed would drive the robot half a body length
        # sideways before the first command frame, so acquire the pose under a
        # profile limit first.
        pose = self.gait.nominal_motor_degrees
        trial.controller.set_all_speed(60)
        trial.controller.set_accelerations(
            {motor_id: 20 for motor_id in Actuator.Index.XM}
        )
        trial.controller.set_positions(
            {motor_id: float(pose[motor_id - 1]) for motor_id in Actuator.Index.ALL}
        )
        raw_targets = {
            motor_id: trial.controller.degrees_to_raw(
                motor_id, float(pose[motor_id - 1])
            )
            for motor_id in Actuator.Index.ALL
        }
        if not trial.wait_until_raw_positions(
            raw_targets,
            tolerance=96,
            timeout=6.0,
            recorder=recorder,
        ):
            raise RuntimeError("role-split sector stance pose did not settle")
        configure_model_gait_controller(trial.controller)

    def update(self, command: VelocityCommand, dt: float) -> ControlDiagnostics:
        sample = self.gait.update(command, dt=dt, send=True)
        return _diagnostics(sample)

    def stop(self) -> None:
        self.gait.update(VelocityCommand(), dt=0.02, send=True)


class MatchedArticulatedController:
    """Common-gait reference with continuous distal rotation disabled."""

    name = "matched-articulated"

    def __init__(self, trial: SimulationTrial, *, phase: float = 0.0) -> None:
        self.trial = trial
        self.phase = phase
        self.config = MATCHED_ROLL_CONFIG
        self.gait = SconeGait(
            trial.controller,
            trial.robot.profile,
            config=self.config.planner_config(),
        )
        self.gait.reset(phase=phase)

    def prepare(
        self,
        trial: SimulationTrial,
        *,
        recorder: MetricsRecorder | None = None,
    ) -> None:
        pose = _acquire_phase_pose(
            trial,
            self.gait,
            self.config,
            recorder=recorder,
        )
        self.gait.reset(phase=self.phase, motor_degrees=pose)

    def update(self, command: VelocityCommand, dt: float) -> ControlDiagnostics:
        sample = self.gait.update(command, dt=dt, send=True)
        return _diagnostics(sample)

    def stop(self) -> None:
        self.gait.update(VelocityCommand(), dt=0.02, send=True)


def make_controller(
    name: str,
    trial: SimulationTrial,
    *,
    phase: float = 0.0,
) -> BenchmarkController:
    if name == "articulated-walk":
        return ArticulatedWalkController(trial, phase=phase)
    if name == "distal-only-roll":
        return DistalOnlyRollController(trial, phase=phase)
    if name == "full-roll":
        return FullRollController(trial, phase=phase)
    if name == "bounded-scone":
        return BoundedSconeController(trial, phase=phase)
    if name == "role-split-scone":
        return RoleSplitSconeController(trial, phase=phase)
    if name == "rear-pair-roll":
        return RoleSplitSconeController(
            trial,
            phase=phase,
            config=REAR_PAIR_ROLL_CONFIG,
            name="rear-pair-roll",
        )
    if name == "no-roll-baseline":
        return RoleSplitSconeController(
            trial,
            phase=phase,
            config=NO_ROLL_CONFIG,
            name="no-roll-baseline",
        )
    if name == "fast-articulated-walk":
        return ArticulatedWalkController(
            trial,
            phase=phase,
            config=FAST_WALK_CONFIG,
            name="fast-articulated-walk",
        )
    if name == "front-pair-roll":
        return RoleSplitSconeController(
            trial,
            phase=phase,
            config=FRONT_PAIR_ROLL_CONFIG,
            name="front-pair-roll",
        )
    if name == "matched-articulated":
        return MatchedArticulatedController(trial, phase=phase)
    if name == "matched-distal-only":
        return DistalOnlyRollController(
            trial,
            phase=phase,
            config=MATCHED_ROLL_CONFIG,
            recalibrate_phase_pose=True,
        )
    if name == "matched-coordinated":
        return FullRollController(
            trial,
            phase=phase,
            config=MATCHED_ROLL_CONFIG,
            recalibrate_phase_pose=True,
        )
    raise ValueError(f"unknown benchmark controller {name!r}")


__all__ = [
    "BenchmarkController",
    "CONTROLLER_CHOICES",
    "MATCHED_CONTROLLER_CHOICES",
    "FAST_WALK_CONFIG",
    "FRONT_PAIR_ROLL_CONFIG",
    "NO_ROLL_CONFIG",
    "MATCHED_ROLL_CONFIG",
    "REAR_PAIR_ROLL_CONFIG",
    "ControlDiagnostics",
    "make_controller",
]
