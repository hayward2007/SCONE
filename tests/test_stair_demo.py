from __future__ import annotations

import unittest

import mujoco

from src.hardware import Actuator
from src.simulation import MuJoCoController, load_model
from src.simulation.core.stair_demo import (
    HardcodedStairRoller,
    StairDemoStrategy,
    run_automatic_stair_demo,
)
from src.simulation.core.simulator_cli import main


class StairDemoTests(unittest.TestCase):
    def test_strategy_parser_and_terrain_boundary(self) -> None:
        self.assertIs(
            StairDemoStrategy.parse("compare"),
            StairDemoStrategy.COMPARE,
        )
        with self.assertRaises(ValueError):
            StairDemoStrategy.parse("manual")
        with self.assertRaises(ValueError):
            run_automatic_stair_demo("improved", terrain="flat")

    def test_hardcoded_baseline_synchronizes_then_uses_open_loop_velocity(self) -> None:
        model = load_model(floating_base=True, terrain="stairs-1")
        controller = MuJoCoController(model, mujoco.MjData(model), verbose=False)
        try:
            baseline = HardcodedStairRoller(controller, velocity=150)
            front_targets = baseline.prepare_front_stage1()
            self.assertEqual(front_targets, {7: 3072, 9: 3072, 11: 3072})
            targets = baseline.prepare()
            self.assertEqual(
                targets,
                {13: 682, 14: 3413, 15: 682, 16: 3413, 17: 682, 18: 3413},
            )
            baseline.activate()
            baseline.update()
            self.assertTrue(
                all(
                    controller.get_mode(motor_id)
                    == Actuator.OperatingMode.VELOCITY
                    for motor_id in Actuator.Index.LOWER
                )
            )
            expected = controller.arc_wheel_velocities(-150)
            self.assertEqual(
                tuple(
                    int(round(controller._velocity_command[motor_id]))
                    for motor_id in Actuator.Index.LOWER
                ),
                tuple(
                    int(round(controller._speed_to_radians_per_second(motor_id, expected[motor_id])))
                    for motor_id in Actuator.Index.LOWER
                ),
            )
            baseline.stop()
            self.assertEqual(baseline.front_stage1_degrees, 270.0)
            self.assertEqual(baseline.front_stage1_sync_entries, 1)
            self.assertEqual(baseline.phase_sync_entries, 1)
        finally:
            controller.close()

    def test_direct_cli_routes_demo_and_defaults_to_stairs_two(self) -> None:
        from unittest.mock import patch

        with patch(
            "src.simulation.core.simulator_cli.run_automatic_stair_demo"
        ) as demo:
            self.assertEqual(main(["--demo", "compare"]), 0)
        demo.assert_called_once()
        self.assertEqual(demo.call_args.args[0], "compare")
        self.assertEqual(demo.call_args.kwargs["terrain"].value, "stairs-2")

    def test_compare_waits_for_macos_viewer_teardown(self) -> None:
        from unittest.mock import call, patch

        with (
            patch(
                "src.simulation.core.stair_demo._run_single_demo",
                side_effect=("hardcoded-result", "improved-result"),
            ) as run_single,
            patch("src.simulation.core.stair_demo.time.sleep") as sleep,
            patch("src.simulation.core.stair_demo.sys.platform", "darwin"),
        ):
            results = run_automatic_stair_demo("compare", terrain="stairs-2")

        self.assertEqual(results, ("hardcoded-result", "improved-result"))
        self.assertEqual(run_single.call_count, 2)
        self.assertEqual(sleep.call_args_list, [call(1.0)])


if __name__ == "__main__":
    unittest.main()
