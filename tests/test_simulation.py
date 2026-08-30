from __future__ import annotations

import unittest

import mujoco

from src.hardware import ControllerProtocol
from src.simulation import DEFAULT_MODEL_PATH, MuJoCoController, load_model


class SimulationBackendTests(unittest.TestCase):
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
