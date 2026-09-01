from __future__ import annotations

import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from src.locomotion import VelocityCommand
from src.main import SCONE
from src.simulation import MuJoCoController, load_model
from src.simulation.core.stair_climber import (
    SconeStairClimber,
    StairControlState,
    prepare_scone_stair_pose,
)
from src.simulation.terrain import STAIR_PRESETS, TerrainType


class SconeStairClimberTests(unittest.TestCase):
    @staticmethod
    def _run_ascent(terrain: TerrainType) -> tuple[SconeStairClimber, float]:
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
            start_z = float(data.xpos[root_id, 2])
            profile = STAIR_PRESETS[terrain]
            top_y = (
                0.35
                + sum(profile.tread_depths[:-1])
                + 0.4 * profile.tread_depths[-1]
            )
            top_z = start_z + 0.70 * profile.total_height
            climber = SconeStairClimber(controller, terrain=terrain)
            reached_top = False
            for _ in range(400):
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
            climber.stop()
            if not reached_top:
                raise AssertionError(
                    f"{terrain.value} top not reached: "
                    f"root={data.xpos[root_id].tolist()}"
                )
            return climber, minimum_upright
        finally:
            controller.close()

    def test_easy_stairs_keep_roll_only_path(self) -> None:
        climber, minimum_upright = self._run_ascent(TerrainType.STAIRS_1)

        self.assertFalse(climber.tall_stair)
        self.assertEqual(climber.assist_entries, 0)
        self.assertGreater(minimum_upright, 0.97)

    def test_tall_stairs_use_tripod_assist_without_falling(self) -> None:
        climber, minimum_upright = self._run_ascent(TerrainType.STAIRS_3)

        self.assertTrue(climber.tall_stair)
        self.assertGreaterEqual(climber.assist_entries, 1)
        self.assertIs(climber.state, StairControlState.IDLE)
        self.assertGreater(minimum_upright, 0.84)


if __name__ == "__main__":
    unittest.main()
