"""Reproduce the SCONE stair-control hypothesis comparison in headless MuJoCo.

This is an engineering benchmark, not evidence that the same motion is safe on
physical hardware.  It deliberately uses the current deterministic model and
procedural stair presets so controller changes can be compared under the same
initial pose, top-of-stair criterion, and metrics.
"""

from __future__ import annotations

import argparse
import json
import math
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from typing import Iterator
from unittest.mock import patch

import mujoco
import numpy as np

from ..hardware import Actuator
from ..locomotion import VelocityCommand
from ..main import SCONE
from .core.controller import MuJoCoController
from .core.model import load_model
from .core.stair_demo import HardcodedStairRoller
from .core.stair_climber import (
    SconeStairClimber,
    prepare_scone_stair_pose,
    synchronized_phase_spread_degrees,
)
from .terrain import STAIR_PRESETS, TerrainType


HYPOTHESES = (
    "pure-rolling",
    "synchronized-open-loop",
    "legacy-climb",
    "tripod-hook",
    "hybrid",
    "adaptive",
)
TUNING_VARIANTS = {
    "hybrid-base": (250.0, 165.0, -105, -185, 0.75, 0.0),
    "hybrid-soft": (240.0, 170.0, -80, -170, 0.75, 0.0),
    "hybrid-balanced": (240.0, 170.0, -100, -170, 0.65, 0.0),
    "hybrid-fast": (245.0, 170.0, -120, -210, 0.60, 0.0),
    "hybrid-ramped": (250.0, 165.0, -105, -185, 0.75, 0.18),
}


@dataclass(frozen=True)
class StairBenchmarkResult:
    strategy: str
    terrain: str
    top_reached: bool
    time_to_top_s: float | None
    work_to_top_j: float | None
    minimum_upright_to_top: float | None
    peak_contact_force_to_top_n: float | None
    elapsed_s: float
    total_absolute_work_j: float
    minimum_upright: float
    peak_contact_force_n: float
    final_y: float
    final_z: float
    assist_entries_to_top: int | None = None
    front_stage1_degrees: float | None = None
    front_stage1_actual_degrees: float | None = None
    front_stage1_sync_entries_to_top: int | None = None
    phase_sync_entries_to_top: int | None = None
    maximum_phase_spread_to_top_degrees: float | None = None
    phase_spread_at_top_degrees: float | None = None


class _Trial:
    def __init__(self, terrain: TerrainType) -> None:
        self.terrain = terrain
        self.model = load_model(floating_base=True, terrain=terrain)
        self.data = mujoco.MjData(self.model)
        self.controller = MuJoCoController(self.model, self.data, verbose=False)
        self.robot = SCONE(self.controller, profile="standard")
        self.root_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "UPPER_BODY_1",
        )
        self.elapsed = 0.0
        self.work = 0.0
        self.peak_force = 0.0
        self.minimum_upright = 1.0
        self.measurement_start_time: float | None = None
        self.measurement_start_work = 0.0
        self.top_y = math.inf
        self.top_z = math.inf
        self.time_to_top: float | None = None
        self.work_to_top: float | None = None
        self.peak_force_to_top: float | None = None
        self.minimum_upright_to_top: float | None = None

    def close(self) -> None:
        self.controller.close()

    def advance(self, seconds: float) -> None:
        steps = max(1, int(np.ceil(seconds / self.model.opt.timestep)))
        dt = float(self.model.opt.timestep)
        contact_force = np.zeros(6, dtype=np.float64)
        for _ in range(steps):
            self.controller.update(dt)
            mujoco.mj_step(self.model, self.data)
            for actuator_id in range(self.model.nu):
                joint_id = int(self.model.actuator_trnid[actuator_id, 0])
                dof_id = int(self.model.jnt_dofadr[joint_id])
                self.work += abs(
                    float(self.data.actuator_force[actuator_id])
                    * float(self.data.qvel[dof_id])
                ) * dt
            for contact_id in range(self.data.ncon):
                mujoco.mj_contactForce(
                    self.model,
                    self.data,
                    contact_id,
                    contact_force,
                )
                self.peak_force = max(
                    self.peak_force,
                    float(np.linalg.norm(contact_force[:3])),
                )
            rotation = self.data.xmat[self.root_id].reshape(3, 3)
            self.minimum_upright = min(
                self.minimum_upright,
                float(rotation[2, 2]),
            )
            if (
                self.measurement_start_time is not None
                and self.time_to_top is None
                and float(self.data.xpos[self.root_id, 1]) >= self.top_y
                and float(self.data.xpos[self.root_id, 2]) >= self.top_z
            ):
                self.time_to_top = self.elapsed + dt - self.measurement_start_time
                self.work_to_top = self.work - self.measurement_start_work
                self.peak_force_to_top = self.peak_force
                self.minimum_upright_to_top = self.minimum_upright
            self.elapsed += dt

    @contextmanager
    def simulated_sleep(self) -> Iterator[None]:
        with patch("time.sleep", side_effect=self.advance):
            yield

    def prepare_side_on(self) -> np.ndarray:
        with self.simulated_sleep():
            self.robot.initialize()
            prepare_scone_stair_pose(self.robot)
            self.advance(0.5)
        return self.data.xpos[self.root_id].copy()

    def prepare_after_approach(self) -> np.ndarray:
        self.prepare_side_on()
        with self.simulated_sleep():
            self.robot.left()
        return self.data.xpos[self.root_id].copy()

    def begin_measurement(self, start: np.ndarray) -> None:
        profile = STAIR_PRESETS[self.terrain]
        self.measurement_start_time = self.elapsed
        self.measurement_start_work = self.work
        self.minimum_upright = 1.0
        self.peak_force = 0.0
        self.top_y = (
            0.35
            + sum(profile.tread_depths[:-1])
            + 0.4 * profile.tread_depths[-1]
        )
        self.top_z = float(start[2]) + 0.70 * profile.total_height

    def result(
        self,
        strategy: str,
        start_time: float,
        start_work: float,
        *,
        assist_entries_to_top: int | None = None,
        front_stage1_degrees: float | None = None,
        front_stage1_actual_degrees: float | None = None,
        front_stage1_sync_entries_to_top: int | None = None,
        phase_sync_entries_to_top: int | None = None,
        maximum_phase_spread_to_top_degrees: float | None = None,
        phase_spread_at_top_degrees: float | None = None,
    ) -> StairBenchmarkResult:
        final = self.data.xpos[self.root_id]
        return StairBenchmarkResult(
            strategy=strategy,
            terrain=self.terrain.value,
            top_reached=self.time_to_top is not None,
            time_to_top_s=self.time_to_top,
            work_to_top_j=self.work_to_top,
            minimum_upright_to_top=self.minimum_upright_to_top,
            peak_contact_force_to_top_n=self.peak_force_to_top,
            elapsed_s=self.elapsed - start_time,
            total_absolute_work_j=self.work - start_work,
            minimum_upright=self.minimum_upright,
            peak_contact_force_n=self.peak_force,
            final_y=float(final[1]),
            final_z=float(final[2]),
            assist_entries_to_top=assist_entries_to_top,
            front_stage1_degrees=front_stage1_degrees,
            front_stage1_actual_degrees=front_stage1_actual_degrees,
            front_stage1_sync_entries_to_top=front_stage1_sync_entries_to_top,
            phase_sync_entries_to_top=phase_sync_entries_to_top,
            maximum_phase_spread_to_top_degrees=(
                maximum_phase_spread_to_top_degrees
            ),
            phase_spread_at_top_degrees=phase_spread_at_top_degrees,
        )


def _set_group_velocity(
    trial: _Trial,
    group: tuple[int, ...],
    raw_velocity: int,
) -> None:
    mapped = trial.controller.arc_wheel_velocities(raw_velocity)
    trial.controller.set_velocities(
        {motor_id: mapped[motor_id] for motor_id in group}
    )


def _run_alternating_tripods(
    trial: _Trial,
    *,
    support_angle: float,
    swing_angle: float,
    support_speed: int,
    swing_speed: int,
    phase_seconds: float,
    phase_count: int,
    transition_seconds: float = 0.0,
) -> None:
    tripod_a_middle = Actuator.Index.MIDDLE_DIAGONAL_RIGHT
    tripod_b_middle = Actuator.Index.MIDDLE_DIAGONAL_LEFT
    tripod_a_lower = Actuator.Index.LOWER_DIAGONAL_RIGHT
    tripod_b_lower = Actuator.Index.LOWER_DIAGONAL_LEFT
    prior_middle = {motor_id: 180.0 for motor_id in Actuator.Index.MIDDLE}
    prior_velocity = {motor_id: 0.0 for motor_id in Actuator.Index.LOWER}

    for phase in range(phase_count):
        support_middle, swing_middle = (
            (tripod_a_middle, tripod_b_middle)
            if phase % 2 == 0
            else (tripod_b_middle, tripod_a_middle)
        )
        support_lower, swing_lower = (
            (tripod_a_lower, tripod_b_lower)
            if phase % 2 == 0
            else (tripod_b_lower, tripod_a_lower)
        )
        middle_target = {
            **{motor_id: support_angle for motor_id in support_middle},
            **{motor_id: swing_angle for motor_id in swing_middle},
        }
        raw_target = {
            **{motor_id: support_speed for motor_id in support_lower},
            **{motor_id: swing_speed for motor_id in swing_lower},
        }
        mapped_target = {
            motor_id: trial.controller.arc_wheel_velocities(raw)[motor_id]
            for motor_id, raw in raw_target.items()
        }
        if transition_seconds <= 0.0:
            trial.controller.set_positions(middle_target)
            trial.controller.set_velocities(mapped_target)
            trial.advance(phase_seconds)
        else:
            step_dt = 0.02
            steps = int(round(phase_seconds / step_dt))
            for index in range(1, steps + 1):
                u = min(1.0, index * step_dt / transition_seconds)
                blend = u * u * (3.0 - 2.0 * u)
                trial.controller.set_positions(
                    {
                        motor_id: prior_middle[motor_id]
                        + blend
                        * (middle_target[motor_id] - prior_middle[motor_id])
                        for motor_id in Actuator.Index.MIDDLE
                    }
                )
                trial.controller.set_velocities(
                    {
                        motor_id: int(
                            round(
                                prior_velocity[motor_id]
                                + blend
                                * (
                                    mapped_target[motor_id]
                                    - prior_velocity[motor_id]
                                )
                            )
                        )
                        for motor_id in Actuator.Index.LOWER
                    }
                )
                trial.advance(step_dt)
        prior_middle = middle_target
        prior_velocity = mapped_target


def run_hypothesis(
    terrain: TerrainType | str,
    strategy: str,
) -> StairBenchmarkResult:
    """Run one historical or synchronized comparison from a side-on state."""

    parsed_terrain = TerrainType.parse(terrain)
    if parsed_terrain not in STAIR_PRESETS:
        raise ValueError("stair benchmark requires stairs-1, stairs-2, or stairs-3")
    if strategy not in HYPOTHESES:
        raise ValueError(f"unknown stair strategy: {strategy}")
    trial = _Trial(parsed_terrain)
    phase_sync_entries_to_top: int | None = None
    front_stage1_degrees: float | None = None
    front_stage1_actual_degrees: float | None = None
    front_stage1_sync_entries_to_top: int | None = None
    maximum_phase_spread_to_top_degrees: float | None = None
    phase_spread_at_top_degrees: float | None = None
    try:
        start = trial.prepare_side_on()

        if strategy in ("adaptive", "synchronized-open-loop"):
            # Phase acquisition is a setup operation, not ascent.  The fixed
            # baseline uses 60 degrees on every preset; the improved controller
            # may select its measured tall-stair phase before measurement.
            if strategy == "adaptive":
                phase_controller: SconeStairClimber | HardcodedStairRoller = (
                    SconeStairClimber(trial.controller, terrain=parsed_terrain)
                )
            else:
                phase_controller = HardcodedStairRoller(trial.controller)
            front_targets = phase_controller.prepare_front_stage1()
            trial.advance(1.5)
            for motor_id, target in front_targets.items():
                actual = trial.controller.get_position(motor_id)
                if (
                    abs(actual - target)
                    > phase_controller.config.front_stage1_tolerance_raw
                ):
                    raise RuntimeError(
                        f"ID {motor_id} failed front stage-1 brace acquisition: "
                        f"target={target}, actual={actual}"
                    )
            raw_targets = phase_controller.prepare()
            trial.advance(1.5)
            for motor_id, target in raw_targets.items():
                actual = trial.controller.get_position(motor_id)
                if abs(actual - target) > phase_controller.config.phase_tolerance_raw:
                    raise RuntimeError(
                        f"ID {motor_id} failed common-phase acquisition: "
                        f"target={target}, actual={actual}"
                    )
            phase_controller.activate()
            front_stage1_degrees = phase_controller.front_stage1_degrees
            front_stage1_actual_degrees = float(
                np.mean(
                    [
                        trial.controller.get_position(motor_id) / 4096.0 * 360.0
                        for motor_id in Actuator.Index.MIDDLE_RIGHT
                    ]
                )
            )
            start = trial.data.xpos[trial.root_id].copy()

        trial.begin_measurement(start)
        start_time = trial.elapsed
        start_work = trial.work

        if strategy == "adaptive":
            assert isinstance(phase_controller, SconeStairClimber)
            # Reuse the prepared controller instead of resetting the acquired
            # phase and position-mode setpoints.
            climber = phase_controller
            for _ in range(800):
                climber.update(
                    VelocityCommand(vy=climber.config.max_vy),
                    0.02,
                )
                trial.advance(0.02)
                if (
                    trial.time_to_top is not None
                    and maximum_phase_spread_to_top_degrees is None
                ):
                    phase_sync_entries_to_top = climber.phase_sync_entries
                    front_stage1_sync_entries_to_top = (
                        climber.front_stage1_sync_entries
                    )
                    maximum_phase_spread_to_top_degrees = (
                        climber.maximum_phase_spread_degrees
                    )
                    phase_spread_at_top_degrees = (
                        synchronized_phase_spread_degrees(trial.controller)
                    )
                    break
            climber.stop()
            if trial.time_to_top is None:
                maximum_phase_spread_to_top_degrees = (
                    climber.maximum_phase_spread_degrees
                )
        elif strategy == "synchronized-open-loop":
            assert isinstance(phase_controller, HardcodedStairRoller)
            for _ in range(800):
                phase_controller.update()
                trial.advance(0.02)
                if (
                    trial.time_to_top is not None
                    and maximum_phase_spread_to_top_degrees is None
                ):
                    phase_sync_entries_to_top = phase_controller.phase_sync_entries
                    front_stage1_sync_entries_to_top = (
                        phase_controller.front_stage1_sync_entries
                    )
                    maximum_phase_spread_to_top_degrees = (
                        phase_controller.maximum_phase_spread_degrees
                    )
                    phase_spread_at_top_degrees = (
                        synchronized_phase_spread_degrees(trial.controller)
                    )
                    break
            phase_controller.stop()
            if trial.time_to_top is None:
                maximum_phase_spread_to_top_degrees = (
                    phase_controller.maximum_phase_spread_degrees
                )
        elif strategy == "pure-rolling":
            trial.controller.set_all_mode(Actuator.OperatingMode.VELOCITY)
            trial.controller.set_velocities(
                trial.controller.arc_wheel_velocities(-150)
            )
            trial.advance(16.0)
        elif strategy == "legacy-climb":
            with trial.simulated_sleep():
                trial.robot.left()
                trial.robot.change_mode()
                for _ in range(3):
                    trial.robot.right()
        else:
            trial.controller.set_all_mode(Actuator.OperatingMode.VELOCITY)
            trial.controller.set_velocities(
                trial.controller.arc_wheel_velocities(-150)
            )
            trial.advance(1.0)
            trial.controller.set_velocities(
                {motor_id: 0 for motor_id in Actuator.Index.LOWER}
            )
            is_tripod_hook = strategy == "tripod-hook"
            _run_alternating_tripods(
                trial,
                support_angle=250.0,
                swing_angle=165.0,
                support_speed=-45 if is_tripod_hook else -105,
                swing_speed=-210 if is_tripod_hook else -185,
                phase_seconds=0.75,
                phase_count=12,
            )
            trial.advance(1.0)
        return trial.result(
            strategy,
            start_time,
            start_work,
            front_stage1_degrees=front_stage1_degrees,
            front_stage1_actual_degrees=front_stage1_actual_degrees,
            front_stage1_sync_entries_to_top=front_stage1_sync_entries_to_top,
            phase_sync_entries_to_top=phase_sync_entries_to_top,
            maximum_phase_spread_to_top_degrees=(
                maximum_phase_spread_to_top_degrees
            ),
            phase_spread_at_top_degrees=phase_spread_at_top_degrees,
        )
    finally:
        trial.close()


def run_tuning_variant(strategy: str) -> StairBenchmarkResult:
    """Run one H3 parameter variant after the shared stairs-3 approach."""

    if strategy not in TUNING_VARIANTS:
        raise ValueError(f"unknown tuning variant: {strategy}")
    trial = _Trial(TerrainType.STAIRS_3)
    try:
        start = trial.prepare_after_approach()
        trial.begin_measurement(start)
        start_time = trial.elapsed
        start_work = trial.work
        trial.controller.set_all_mode(Actuator.OperatingMode.VELOCITY)
        values = TUNING_VARIANTS[strategy]
        _run_alternating_tripods(
            trial,
            support_angle=values[0],
            swing_angle=values[1],
            support_speed=values[2],
            swing_speed=values[3],
            phase_seconds=values[4],
            phase_count=int(math.ceil(9.0 / values[4])),
            transition_seconds=values[5],
        )
        trial.controller.set_velocities(
            {motor_id: 0 for motor_id in Actuator.Index.LOWER}
        )
        trial.controller.set_positions(
            {motor_id: 180.0 for motor_id in Actuator.Index.MIDDLE}
        )
        trial.advance(1.0)
        return trial.result(strategy, start_time, start_work)
    finally:
        trial.close()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run headless SCONE stair-control comparisons",
    )
    parser.add_argument(
        "--terrain",
        action="append",
        choices=("stairs-1", "stairs-2", "stairs-3"),
        help="stair preset; repeat for multiple presets",
    )
    parser.add_argument(
        "--strategy",
        action="append",
        choices=HYPOTHESES,
        help="historical or synchronized strategy; repeat for multiple strategies",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="run every strategy on all three stair presets",
    )
    parser.add_argument(
        "--tuning",
        action="store_true",
        help="run all H3 parameter variants on stairs-3 after approach",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    if not args.all and not args.tuning and not args.strategy:
        raise SystemExit("choose --all, --tuning, or at least one --strategy")
    if args.all or args.strategy:
        terrains = args.terrain or (
            "stairs-1",
            "stairs-2",
            "stairs-3",
        )
        strategies = HYPOTHESES if args.all else args.strategy
        for terrain in terrains:
            for strategy in strategies:
                print(json.dumps(asdict(run_hypothesis(terrain, strategy))))
    if args.tuning:
        for strategy in TUNING_VARIANTS:
            print(json.dumps(asdict(run_tuning_variant(strategy))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
