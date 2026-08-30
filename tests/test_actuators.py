from __future__ import annotations

import unittest

from src.hardware import Actuator, model_for_id


class ActuatorMetadataTests(unittest.TestCase):
    def test_each_leg_maps_to_three_stages(self) -> None:
        self.assertEqual(Actuator.Index.for_leg(1), (1, 7, 13))
        self.assertEqual(Actuator.Index.for_leg(6), (6, 12, 18))

    def test_model_assignment(self) -> None:
        self.assertEqual(model_for_id(1).name, "MX-28AT")
        self.assertEqual(model_for_id(7).name, "XM430-W350-T")
        self.assertEqual(model_for_id(13).name, "XM430-W210-T")
        self.assertTrue(
            all(
                model_for_id(motor_id).position_resolution == 4096
                for motor_id in Actuator.Index.ALL
            )
        )


if __name__ == "__main__":
    unittest.main()
