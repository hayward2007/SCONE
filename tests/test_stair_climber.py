from __future__ import annotations

import unittest
from unittest.mock import Mock, call, patch

import mujoco
import numpy as np

from src.hardware import Actuator
from src.locomotion import VelocityCommand
from src.main import SCONE
from src.simulation import MuJoCoController, load_model
from src.simulation.core.stair_climber import (
    SconeStairClimber,
    SconeStairConfig,
    StairControlState,
    prepare_scone_stair_pose,
    synchronized_lower_degrees,
)
from src.simulation.terrain import STAIR_PRESETS, TerrainType


class SconeStairClimberTests(unittest.TestCase):
    def test_stair_preparation_runs_drive_then_climb_transitions(self) -> None:
        robot = Mock()
        robot.mode_name = "walk"
        robot.change_mode.side_effect = ("drive", "climb")

        prepare_scone_stair_pose(robot)

        self.assertEqual(robot.left.call_count, 4)
        self.assertEqual(robot.change_mode.call_args_list, [call(), call()])

    @staticmethod
    def _run_ascent(
        terrain: TerrainType,
    ) -> tuple[SconeStairClimber, float, float, bool]:
        model = load_model(floating_base=True, terrain=terrain)
        data = mujoco.MjData(model)
        controller = MuJoCoController(model, data, verbose=False)
        robot = SCONE(controller, profile="standard")
        root_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "UPPER_BODY_1",
        )
        minimum_upright = 1.0

        def advance(seconds: float) -> None:
            nonlocal minimum_upright
            steps = max(1, int(np.ceil(seconds / model.opt.timestep)))
            for _ in range(steps):
                controller.update(model.opt.timestep)
                mujoco.mj_step(model, data)
                minimum_upright = min(
                    minimum_upright,
                    float(data.xmat[root_id].reshape(3, 3)[2, 2]),
                )

        try:
            with patch("time.sleep", side_effect=advance):
                robot.initialize()
                prepare_scone_stair_pose(robot)

            climber = SconeStairClimber(controller, terrain=terrain)
            front_targets = climber.prepare_front_stage1()
            advance(climber.config.front_stage1_sync_timeout)
            for motor_id, target in front_targets.items():
                actual = controller.get_position(motor_id)
                if abs(actual - target) > climber.config.front_stage1_tolerance_raw:
                    raise AssertionError(
                        f"ID {motor_id} front brace did not synchronize: "
                        f"target={target}, actual={actual}"
                    )
            raw_targets = climber.prepare()
            advance(climber.config.phase_sync_timeout)
            for motor_id, target in raw_targets.items():
                actual = controller.get_position(motor_id)
                if abs(actual - target) > climber.config.phase_tolerance_raw:
                    raise AssertionError(
                        f"ID {motor_id} phase did not synchronize: "
                        f"target={target}, actual={actual}"
                    )
            climber.activate()

            # Synchronization is a setup motion, so ascent timing/height starts
            # only after all six C-frames have acquired the common phase.
            start_z = float(data.xpos[root_id, 2])
            minimum_upright = 1.0
            profile = STAIR_PRESETS[terrain]
            top_y = (
                0.35
                + sum(profile.tread_depths[:-1])
                + 0.4 * profile.tread_depths[-1]
            )
            top_z = start_z + 0.70 * profile.total_height
            reached_top = False
            for _ in range(900):
                climber.update(
                    VelocityCommand(vy=climber.config.max_vy),
                    0.02,
                )
                advance(0.02)
                if (
                    float(data.xpos[root_id, 1]) >= top_y
                    and float(data.xpos[root_id, 2]) >= top_z
                ):
                    reached_top = True
                    break
            spread = climber.maximum_phase_spread_degrees
            climber.stop()
            return climber, minimum_upright, spread, reached_top
        finally:
            controller.close()

    def test_geometric_phase_mirrors_even_model_axes(self) -> None:
        targets = synchronized_lower_degrees(60.0)
        self.assertEqual(
            targets,
            {13: 60.0, 14: 300.0, 15: 60.0, 16: 300.0, 17: 60.0, 18: 300.0},
        )
        unwrapped = synchronized_lower_degrees(-10.0)
        self.assertEqual(unwrapped[13], -10.0)
        self.assertEqual(unwrapped[14], 370.0)

    def test_all_stair_heights_climb_with_one_common_phase(self) -> None:
        for terrain in (
            TerrainType.STAIRS_1,
            TerrainType.STAIRS_2,
            TerrainType.STAIRS_3,
        ):
            with self.subTest(terrain=terrain.value):
                climber, minimum_upright, spread, reached_top = self._run_ascent(
                    terrain
                )
                self.assertTrue(reached_top)
                self.assertEqual(climber.phase_sync_entries, 1)
                self.assertEqual(climber.front_stage1_sync_entries, 1)
                self.assertEqual(
                    climber.initial_phase_degrees,
                    90.0 if terrain is TerrainType.STAIRS_3 else 60.0,
                )
                self.assertEqual(
                    climber.selected_phase_velocity,
                    250.0 if terrain is TerrainType.STAIRS_1 else 200.0,
                )
                self.assertEqual(
                    climber.front_stage1_degrees,
                    {
                        TerrainType.STAIRS_1: 180.0,
                        TerrainType.STAIRS_2: 184.0,
                        TerrainType.STAIRS_3: 195.0,
                    }[terrain],
                )
                # Commands are exactly phase-identical.  Loaded joints may
                # physically lag while hooked on an edge; keep that transient
                # below the 135-degree C-frame opening rather than pretending
                # the rigid contact can maintain zero measured error.
                self.assertLess(spread, 100.0)
                self.assertIs(climber.state, StairControlState.IDLE)
                self.assertGreater(minimum_upright, 0.70)
                self.assertTrue(
                    all(
                        climber.controller.get_mode(motor_id)
                        == Actuator.OperatingMode.EXTENDED_POSITION
                        for motor_id in Actuator.Index.LOWER
                    )
                )

    def test_configuration_rejects_invalid_phase_and_rate(self) -> None:
        with self.assertRaises(ValueError):
            SconeStairConfig(synchronized_phase_degrees=360.0)
        with self.assertRaises(ValueError):
            SconeStairConfig(phase_velocity=0.0)
        with self.assertRaises(ValueError):
            SconeStairConfig(tall_front_stage1_degrees=361.0)


if __name__ == "__main__":
    unittest.main()
