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

    def test_hardcoded_baseline_has_no_feedback_state(self) -> None:
        model = load_model(floating_base=True, terrain="stairs-1")
        controller = MuJoCoController(model, mujoco.MjData(model), verbose=False)
        try:
            baseline = HardcodedStairRoller(controller, velocity=150)
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


if __name__ == "__main__":
    unittest.main()
