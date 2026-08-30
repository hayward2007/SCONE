from __future__ import annotations

import unittest
from unittest.mock import patch

import mujoco

from src.hardware import ControllerProtocol
from src.simulation import DEFAULT_MODEL_PATH, MuJoCoController, load_model
from src.simulation.core.cli_bridge import SimulationControl, run


class SimulationBackendTests(unittest.TestCase):
    def test_rl_control_routes_to_policy_runtime_without_legacy_viewer(self) -> None:
        with patch("src.rl.joystick_control.run_rl_joystick") as runner:
            run(
                control=SimulationControl.RL,
                checkpoint="policy.zip",
                terrain="uneven",
                terrain_seed=11,
            )

        runner.assert_called_once()
        arguments = runner.call_args.kwargs
        self.assertEqual(arguments["terrain"].value, "uneven")
        self.assertEqual(arguments["terrain_seed"], 11)

    def test_model_maps_all_actuators_to_controller_contract(self) -> None:
        model = load_model(DEFAULT_MODEL_PATH)
        controller = MuJoCoController(model, mujoco.MjData(model), verbose=False)
        try:
            self.assertIsInstance(controller, ControllerProtocol)
            self.assertEqual(model.nu, 18)
        finally:
            controller.close()


if __name__ == "__main__":
    unittest.main()
