from __future__ import annotations

import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from src.hardware import ControllerProtocol
from src.locomotion import NonRLWalk, VelocityCommand
from src.main import SCONE
from src.simulation import DEFAULT_MODEL_PATH, MuJoCoController, load_model
from src.simulation.core.cli_bridge import (
    NON_RL_SIMULATION_GAIT_CONFIG,
    SimulationControl,
    run,
)
from src.simulation.core.simulator_cli import build_parser


class SimulationBackendTests(unittest.TestCase):
    def test_non_rl_simulation_uses_ik_safe_stride(self) -> None:
        self.assertEqual(NON_RL_SIMULATION_GAIT_CONFIG.max_stride, 0.060)
        self.assertEqual(
            NON_RL_SIMULATION_GAIT_CONFIG.max_lateral_stride,
            0.050,
        )
        self.assertEqual(NON_RL_SIMULATION_GAIT_CONFIG.cycle_frequency, 0.7)
        self.assertEqual(
            NON_RL_SIMULATION_GAIT_CONFIG.ik_stride_backoff_attempts,
            4,
        )
        self.assertEqual(
            NON_RL_SIMULATION_GAIT_CONFIG.ik_tolerance,
            1e-3,
        )

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
        self.assertEqual(arguments["reference_motion"], "hardcoded")

    def test_rl_cli_defaults_to_legacy_hardcoded_reference(self) -> None:
        arguments = build_parser().parse_args(
            ["--control", "rl", "--checkpoint", "policy.zip"]
        )

        self.assertEqual(arguments.rl_reference_motion, "hardcoded")

    def test_model_maps_all_actuators_to_controller_contract(self) -> None:
        model = load_model(DEFAULT_MODEL_PATH)
        controller = MuJoCoController(model, mujoco.MjData(model), verbose=False)
        try:
            self.assertIsInstance(controller, ControllerProtocol)
            self.assertEqual(model.nu, 18)
        finally:
            controller.close()

    def test_legacy_mode_cycle_settles_opposed_arc_frames(self) -> None:
        model = load_model(DEFAULT_MODEL_PATH, floating_base=False)
        data = mujoco.MjData(model)
        controller = MuJoCoController(model, data, verbose=False)
        robot = SCONE(controller, profile="standard")

        def advance_simulation(seconds: float) -> None:
            steps = max(1, int(np.ceil(seconds / model.opt.timestep)))
            for _ in range(steps):
                controller.update(model.opt.timestep)
                mujoco.mj_step(model, data)

        def assert_opposed_arc_pairs() -> None:
            # The SCONEv2 mesh opening points along local -Y.  A paper-valid
            # virtual wheel therefore has opposing opening directions.
            local_opening = np.array([0.0, -1.0, 0.0])
            directions = {}
            for leg in range(1, 7):
                geom_id = mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_GEOM,
                    f"ARC_SHAPED_WHEEL_{leg}_geom",
                )
                directions[leg] = (
                    data.geom_xmat[geom_id].reshape(3, 3) @ local_opening
                )
            for right, left in ((1, 2), (3, 4), (5, 6)):
                self.assertLess(
                    float(np.dot(directions[right], directions[left])),
                    -0.98,
                )

        try:
            with patch("time.sleep", side_effect=advance_simulation):
                robot.initialize()
                default_stage1_kd = controller._pid[7].kd
                self.assertEqual(robot.change_mode(), "drive")
                self.assertAlmostEqual(
                    controller._pid[7].kd,
                    default_stage1_kd
                    * controller._DRIVE_STAGE1_DAMPING_MULTIPLIER,
                )
                assert_opposed_arc_pairs()

                self.assertEqual(robot.change_mode(), "climb")
                self.assertAlmostEqual(controller._pid[7].kd, default_stage1_kd)
                assert_opposed_arc_pairs()
                robot.right()

            for motor_id in range(7, 19):
                self.assertLessEqual(
                    abs(controller.get_position(motor_id) - 2048),
                    192,
                )
        finally:
            controller.close()

    def test_standard_non_rl_walk_advances_in_dynamics_without_ik_failure(self) -> None:
        model = load_model(DEFAULT_MODEL_PATH, floating_base=True)
        data = mujoco.MjData(model)
        controller = MuJoCoController(model, data, verbose=False)
        robot = SCONE(controller, profile="standard")

        def advance_simulation(seconds: float) -> None:
            steps = max(1, int(np.ceil(seconds / model.opt.timestep)))
            for _ in range(steps):
                controller.update(model.opt.timestep)
                mujoco.mj_step(model, data)

        try:
            with patch("time.sleep", side_effect=advance_simulation):
                robot.initialize()

            # Do not recenter on the gravity-sagged transient pose.  The
            # selected Standard profile is the IK-safe gait origin in sim.
            gait = NonRLWalk(
                controller,
                robot.profile,
                config=NON_RL_SIMULATION_GAIT_CONFIG,
            )
            root_body_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_BODY,
                "UPPER_BODY_1",
            )
            start_position = data.xpos[root_body_id].copy()
            start_rotation = data.xmat[root_body_id].reshape(3, 3).copy()

            for _ in range(150):
                sample = gait.update(
                    VelocityCommand(vx=gait.config.max_vx),
                    dt=0.02,
                    send=True,
                )
                self.assertTrue(sample.converged, sample.failed_legs)
                advance_simulation(0.02)

            body_displacement = start_rotation.T @ (
                data.xpos[root_body_id] - start_position
            )
            self.assertGreater(float(body_displacement[0]), 0.08)
            self.assertGreater(
                float(data.xmat[root_body_id].reshape(3, 3)[2, 2]),
                0.98,
            )
        finally:
            controller.close()

    def test_drive_mode_moves_right_with_mirrored_arc_wheel_axes(self) -> None:
        model = load_model(DEFAULT_MODEL_PATH, floating_base=True)
        data = mujoco.MjData(model)
        controller = MuJoCoController(model, data, verbose=False)
        robot = SCONE(controller, profile="standard")

        def advance_simulation(seconds: float) -> None:
            steps = max(1, int(np.ceil(seconds / model.opt.timestep)))
            for _ in range(steps):
                controller.update(model.opt.timestep)
                mujoco.mj_step(model, data)

        try:
            with patch("time.sleep", side_effect=advance_simulation):
                robot.initialize()
                self.assertEqual(robot.change_mode(), "drive")
                advance_simulation(1.0)

                root_body_id = mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    "UPPER_BODY_1",
                )
                start_position = data.xpos[root_body_id].copy()
                start_rotation = data.xmat[root_body_id].reshape(3, 3).copy()
                robot.right()

            displacement = start_rotation.T @ (
                data.xpos[root_body_id] - start_position
            )
            self.assertLess(float(displacement[1]), -0.20)
            self.assertLess(abs(float(displacement[0])), 0.05)
            self.assertGreater(
                float(data.xmat[root_body_id].reshape(3, 3)[2, 2]),
                0.98,
            )
        finally:
            controller.close()

    def test_climb_mode_advances_onto_easy_stairs(self) -> None:
        model = load_model(
            DEFAULT_MODEL_PATH,
            floating_base=True,
            terrain="stairs-1",
        )
        data = mujoco.MjData(model)
        controller = MuJoCoController(model, data, verbose=False)
        robot = SCONE(controller, profile="standard")

        def advance_simulation(seconds: float) -> None:
            steps = max(1, int(np.ceil(seconds / model.opt.timestep)))
            for _ in range(steps):
                controller.update(model.opt.timestep)
                mujoco.mj_step(model, data)

        try:
            with patch("time.sleep", side_effect=advance_simulation):
                robot.initialize()
                # SCONE climbs sideways.  Four legacy turns align the wheel
                # planes with the +world-Y stair course.
                for _ in range(4):
                    robot.left()
                self.assertEqual(robot.change_mode(), "drive")
                advance_simulation(0.5)
                robot.left()  # approach the first riser in Drive
                self.assertEqual(robot.change_mode(), "climb")

                root_body_id = mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    "UPPER_BODY_1",
                )
                start_position = data.xpos[root_body_id].copy()
                for _ in range(3):
                    robot.right()

            displacement = data.xpos[root_body_id] - start_position
            self.assertGreater(float(displacement[1]), 0.30)
            self.assertGreater(float(displacement[2]), 0.025)
            self.assertGreater(
                float(data.xmat[root_body_id].reshape(3, 3)[2, 2]),
                0.98,
            )
        finally:
            controller.close()


if __name__ == "__main__":
    unittest.main()
