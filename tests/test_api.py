from __future__ import annotations

import unittest
from collections.abc import Iterable, Mapping
from unittest.mock import patch

from SCONE import SCONE
from src.cli import run_control_cli
from src.hardware import Actuator, ControllerProtocol


class FakeController:
    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.positions = {
            motor_id: Actuator.Position.CENTER for motor_id in Actuator.Index.ALL
        }

    def _call(self, *args) -> None:
        self.calls.append(args)

    def set_mode(self, motor_id: int, mode: int) -> None:
        self._call("set_mode", motor_id, mode)

    def set_all_mode(self, mode: int) -> None:
        self._call("set_all_mode", mode)

    def get_mode(self, motor_id: int) -> int:
        return int(Actuator.OperatingMode.POSITION)

    def set_speed(self, motor_id: int, speed: int) -> None:
        self._call("set_speed", motor_id, speed)

    def set_all_speed(self, speed: int) -> None:
        self._call("set_all_speed", speed)

    def set_speeds(self, speeds: Mapping[int, int]) -> None:
        self._call("set_speeds", dict(speeds))

    def set_velocity(self, motor_id: int, velocity: int) -> None:
        self._call("set_velocity", motor_id, velocity)

    def set_velocities(self, velocities: Mapping[int, int]) -> None:
        self._call("set_velocities", dict(velocities))

    def set_acceleration(self, motor_id: int, acceleration: int) -> None:
        self._call("set_acceleration", motor_id, acceleration)

    def set_accelerations(self, accelerations: Mapping[int, int]) -> None:
        self._call("set_accelerations", dict(accelerations))

    def set_torque(self, motor_id: int, torque: int) -> None:
        self._call("set_torque", motor_id, torque)

    def set_torques(self, motor_ids: Iterable[int], torque: int) -> None:
        self._call("set_torques", tuple(motor_ids), torque)

    def enable_torque(self) -> None:
        self._call("enable_torque")

    def disable_torque(self) -> None:
        self._call("disable_torque")

    def set_position(self, motor_id: int, position: float) -> None:
        self.positions[motor_id] = position
        self._call("set_position", motor_id, position)

    def set_positions(self, positions: Mapping[int, float]) -> None:
        self.positions.update(positions)
        self._call("set_positions", dict(positions))

    def set_raw_position(self, motor_id: int, position: int) -> None:
        self.positions[motor_id] = position
        self._call("set_raw_position", motor_id, position)

    def set_raw_positions(self, positions: Mapping[int, int]) -> None:
        self.positions.update(positions)
        self._call("set_raw_positions", dict(positions))

    def get_position(self, motor_id: int) -> int:
        return int(self.positions[motor_id])

    def close(self) -> None:
        self._call("close")


class PublicApiTests(unittest.TestCase):
    def test_fake_controller_satisfies_contract(self) -> None:
        self.assertIsInstance(FakeController(), ControllerProtocol)

    @patch("src.main.time.sleep", return_value=None)
    def test_initialize_uses_profile_and_group_api(self, _sleep) -> None:
        controller = FakeController()
        robot = SCONE(controller, profile="sport")
        robot.initialize()

        self.assertTrue(robot.initialized)
        self.assertEqual(robot.profile_name, "sport")
        self.assertEqual(robot.mode_name, "walk")
        self.assertTrue(any(call[0] == "set_positions" for call in controller.calls))
        self.assertEqual(controller.positions[7], 170)
        self.assertEqual(controller.positions[13], 195)

    @patch("src.locomotion.walk.time.sleep", return_value=None)
    @patch("src.main.time.sleep", return_value=None)
    def test_common_cli_dispatches_to_robot_api(self, _main_sleep, _walk_sleep) -> None:
        controller = FakeController()
        robot = SCONE(controller)
        robot.initialize()
        keys = iter(("w", "q"))

        run_control_cli(robot, key_reader=lambda: next(keys))

        upper_batches = [
            call for call in controller.calls if call[0] == "set_positions" and 1 in call[1]
        ]
        self.assertTrue(upper_batches)


if __name__ == "__main__":
    unittest.main()
