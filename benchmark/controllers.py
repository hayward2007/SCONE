"""Controller adapters used by the paper benchmarks."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from src.hardware import Actuator
from src.locomotion import SconeGait, TripodGait, VelocityCommand
from src.simulation.core.cli_bridge import (
    SCONE_GAIT_SIMULATION_CONFIG,
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

    def __init__(self, trial: SimulationTrial, *, phase: float = 0.0) -> None:
        self.trial = trial
        self.phase = phase
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
            config=TRIPOD_GAIT_SIMULATION_CONFIG,
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
    "MATCHED_ROLL_CONFIG",
    "ControlDiagnostics",
    "make_controller",
]
