from __future__ import annotations

import unittest
from collections.abc import Iterable, Mapping
from types import SimpleNamespace
from unittest.mock import patch

from SCONE import SCONE
from src.cli import (
    JoystickState,
    KeyboardJoystick,
    _select_profile,
    main as run_launcher,
    render_joystick_ui,
    run_control_cli,
    run_joystick_cli,
)
from src.hardware import Actuator, ControllerProtocol, HardwareProbe
from src.locomotion import GaitConfig, VelocityCommand, legacy_movement_for
from src.simulation.core.cli_bridge import SimulationControl
from src.simulation.terrain import TerrainType


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


class InquirerLauncherTests(unittest.TestCase):
    @staticmethod
    def _choice(*, value, name):
        return SimpleNamespace(value=value, name=name)

    def test_profile_picker_uses_inquirer_selection(self) -> None:
        prompt = SimpleNamespace(execute=lambda: "sport")
        inquirer = SimpleNamespace(select=lambda **_arguments: prompt)

        with patch("src.cli._inquirer", return_value=(inquirer, self._choice)):
            self.assertEqual(_select_profile(), "sport")

    def test_root_launcher_quits_from_inquirer_menu(self) -> None:
        prompt = SimpleNamespace(execute=lambda: "quit")
        inquirer = SimpleNamespace(select=lambda **_arguments: prompt)

        with (
            patch("src.cli._inquirer", return_value=(inquirer, self._choice)),
            patch(
                "src.cli.discover_hardware",
                return_value=HardwareProbe(False, detail="not connected"),
            ),
        ):
            self.assertEqual(run_launcher(), 0)

    def test_root_launcher_routes_rl_simulation_selection(self) -> None:
        actions = iter(("simulation", "quit"))
        prompt = SimpleNamespace(execute=lambda: next(actions))
        inquirer = SimpleNamespace(select=lambda **_arguments: prompt)
        checkpoint = SimpleNamespace()

        with (
            patch("src.cli._inquirer", return_value=(inquirer, self._choice)),
            patch(
                "src.cli.discover_hardware",
                return_value=HardwareProbe(False, detail="not connected"),
            ),
            patch(
                "src.simulation.core.simulator_cli.select_simulation_control",
                return_value=SimulationControl.RL,
            ),
            patch(
                "src.simulation.core.simulator_cli.select_rl_checkpoint",
                return_value=checkpoint,
            ),
            patch(
                "src.simulation.core.simulator_cli.select_terrain",
                return_value=TerrainType.FLAT,
            ),
            patch("src.simulation.core.cli_bridge.run") as simulation_run,
        ):
            self.assertEqual(run_launcher(), 0)

        simulation_run.assert_called_once_with(
            profile="sport",
            terrain=TerrainType.FLAT,
            control=SimulationControl.RL,
            checkpoint=checkpoint,
        )


class KeyboardJoystickTests(unittest.TestCase):
    def test_wasd_and_arrows_control_independent_axes(self) -> None:
        joystick = KeyboardJoystick(release_timeout=0.35)

        self.assertTrue(joystick.press("w", now=10.0))
        self.assertTrue(joystick.press("d", now=10.1))
        self.assertTrue(joystick.press("\x1b[D", now=10.2))

        self.assertEqual(
            joystick.state(now=10.25),
            JoystickState(x=1.0, y=1.0, yaw=1.0),
        )

    def test_axes_self_center_and_space_stops_immediately(self) -> None:
        joystick = KeyboardJoystick(release_timeout=0.2)
        joystick.press("a", now=5.0)
        joystick.press("right", now=5.0)

        self.assertEqual(
            joystick.state(now=5.1),
            JoystickState(x=-1.0, yaw=-1.0),
        )
        self.assertEqual(joystick.state(now=5.21), JoystickState())

        joystick.press("w", now=6.0)
        joystick.press(" ", now=6.01)
        self.assertEqual(joystick.state(now=6.02), JoystickState())

    def test_joystick_scales_to_body_velocity(self) -> None:
        config = GaitConfig(max_vx=0.18, max_vy=0.12, max_yaw_rate=0.9)

        command = JoystickState(x=-1.0, y=1.0, yaw=-1.0).to_velocity_command(
            config
        )

        self.assertAlmostEqual(command.vx, 0.18)
        self.assertAlmostEqual(command.vy, 0.12)
        self.assertAlmostEqual(command.yaw_rate, -0.9)

    def test_joystick_ui_contains_axes_and_scaled_values(self) -> None:
        state = JoystickState(x=1.0, y=-1.0, yaw=1.0)
        command = state.to_velocity_command(GaitConfig())

        screen = render_joystick_ui(state, command, profile_name="sport")

        self.assertIn("W/S: forward/back", screen)
        self.assertIn("x=+1.00", screen)
        self.assertIn("yaw=+0.900 rad/s", screen)
        self.assertIn("profile=sport", screen)

    def test_old_control_adapts_velocity_without_inventing_strafe(self) -> None:
        self.assertEqual(legacy_movement_for(VelocityCommand(vx=0.1)), "forward")
        self.assertEqual(legacy_movement_for(VelocityCommand(vx=-0.1)), "backward")
        self.assertEqual(legacy_movement_for(VelocityCommand(yaw_rate=0.2)), "left")
        self.assertEqual(legacy_movement_for(VelocityCommand(yaw_rate=-0.2)), "right")
        self.assertIsNone(legacy_movement_for(VelocityCommand(vy=0.1)))

    def test_live_cli_sends_motion_then_neutral_on_quit(self) -> None:
        class FakeGait:
            config = GaitConfig(command_time_constant=0.0)

            def __init__(self) -> None:
                self.reset_called = False
                self.commands = []

            def reset_from_controller(self) -> None:
                self.reset_called = True

            def update(self, command, *, dt, send) -> None:
                self.commands.append((command, dt, send))

        class FakeTerminal:
            def __init__(self) -> None:
                self.keys = iter(("w", None, "q", None))
                self.screens: list[str] = []

            def __enter__(self):
                return self

            def __exit__(self, *_):
                return None

            def read_key(self, _timeout):
                return next(self.keys, None)

            def draw(self, screen: str) -> None:
                self.screens.append(screen)

        gait = FakeGait()
        terminal = FakeTerminal()
        robot = SimpleNamespace(
            controller=object(),
            profile=object(),
            profile_name="standard",
        )

        with (
            patch("src.cli.NonRLWalk", return_value=gait),
            patch("src.cli._JoystickTerminal", return_value=terminal),
        ):
            run_joystick_cli(robot)

        self.assertTrue(gait.reset_called)
        self.assertAlmostEqual(gait.commands[0][0].vx, gait.config.max_vx)
        self.assertEqual(
            gait.commands[-1][0],
            JoystickState().to_velocity_command(gait.config),
        )
        self.assertTrue(all(send for _, _, send in gait.commands))
        self.assertTrue(terminal.screens)


if __name__ == "__main__":
    unittest.main()
