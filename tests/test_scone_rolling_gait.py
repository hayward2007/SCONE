from __future__ import annotations

import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from src.hardware import Actuator
from src.locomotion import VelocityCommand
from src.main import SCONE
from src.simulation import (
    MuJoCoController,
    RollGait,
    RollGaitConfig,
    load_model,
)


class RollGaitTests(unittest.TestCase):
    def test_config_rejects_unstable_or_unbounded_values(self) -> None:
        with self.assertRaises(ValueError):
            RollGaitConfig(roll_velocity=0)
        with self.assertRaises(ValueError):
            RollGaitConfig(support_velocity_ratio=1.1)
        with self.assertRaises(ValueError):
            RollGaitConfig(tripod_b_phase_offset_degrees=121.0)
        with self.assertRaises(ValueError):
            RollGaitConfig(cycle_frequency=-1.0)
        with self.assertRaises(ValueError):
            RollGaitConfig(basic_lower_motion_blend=1.1)
        with self.assertRaises(ValueError):
            RollGaitConfig(max_basic_lower_velocity=0.0)
        with self.assertRaises(ValueError):
            RollGaitConfig(basic_velocity_time_constant=-0.1)

    def test_continuous_sector_rotation_is_fast_and_stays_supported(self) -> None:
        model = load_model(floating_base=True, terrain="flat")
        data = mujoco.MjData(model)
        controller = MuJoCoController(model, data, verbose=False)
        robot = SCONE(controller, profile="standard")
        root_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "UPPER_BODY_1",
        )

        def advance(seconds: float) -> None:
            for _ in range(max(1, int(np.ceil(seconds / model.opt.timestep)))):
                controller.update(model.opt.timestep)
                mujoco.mj_step(model, data)

        try:
            with patch("time.sleep", side_effect=advance):
                robot.initialize()
            gait = RollGait(controller, robot.profile)
            raw_targets = gait.prepare()
            offset_raw = controller.degrees_to_raw(13, 60.0)
            self.assertAlmostEqual(raw_targets[14] - raw_targets[13], offset_raw, delta=1)
            self.assertAlmostEqual(raw_targets[15] - raw_targets[13], offset_raw, delta=1)
            self.assertAlmostEqual(raw_targets[18] - raw_targets[13], offset_raw, delta=1)
            advance(2.5)
            gait.activate()

            start = data.xpos[root_id].copy()
            start_rotation = data.xmat[root_id].reshape(3, 3).copy()
            start_lower = np.array(
                [controller._joint_position(i) for i in Actuator.Index.LOWER]
            )
            minimum_z = float(start[2])
            minimum_upright = 1.0
            maximum_upper_offset = 0.0
            maximum_middle_offset = 0.0
            maximum_basic_velocity = 0
            for _ in range(300):
                sample = gait.update(VelocityCommand(vx=0.18), 0.02)
                self.assertTrue(sample.planner_sample.converged)
                for combined, rolling, basic in zip(
                    sample.lower_velocities,
                    sample.rolling_velocities,
                    sample.basic_velocities,
                    strict=True,
                ):
                    self.assertLessEqual(abs(combined - rolling - basic), 1)
                nominal = gait.planner.nominal_motor_degrees
                maximum_upper_offset = max(
                    maximum_upper_offset,
                    float(
                        np.max(
                            np.abs(
                                sample.planner_sample.motor_degrees[:6]
                                - nominal[:6]
                            )
                        )
                    ),
                )
                maximum_middle_offset = max(
                    maximum_middle_offset,
                    float(
                        np.max(
                            np.abs(
                                sample.planner_sample.motor_degrees[6:12]
                                - nominal[6:12]
                            )
                        )
                    ),
                )
                maximum_basic_velocity = max(
                    maximum_basic_velocity,
                    max(abs(value) for value in sample.basic_velocities),
                )
                advance(0.02)
                minimum_z = min(minimum_z, float(data.xpos[root_id, 2]))
                minimum_upright = min(
                    minimum_upright,
                    float(data.xmat[root_id].reshape(3, 3)[2, 2]),
                )

            displacement = start_rotation.T @ (data.xpos[root_id] - start)
            end_lower = np.array(
                [controller._joint_position(i) for i in Actuator.Index.LOWER]
            )
            mean_revolutions = float(
                np.mean(np.abs(end_lower - start_lower)) / (2.0 * np.pi)
            )
            self.assertGreater(float(displacement[0]), 0.80)
            self.assertLess(abs(float(displacement[1])), 0.06)
            self.assertGreater(minimum_z - float(start[2]), -0.025)
            self.assertGreater(minimum_upright, 0.98)
            self.assertGreater(mean_revolutions, 2.5)
            self.assertGreater(maximum_upper_offset, 10.0)
            self.assertGreater(maximum_middle_offset, 5.0)
            self.assertGreater(maximum_basic_velocity, 5)
        finally:
            controller.close()


if __name__ == "__main__":
    unittest.main()
