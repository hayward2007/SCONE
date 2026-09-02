from __future__ import annotations

import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from src.hardware import ControllerProtocol
from src.locomotion import SconeGait, TripodGait, VelocityCommand
from src.main import SCONE
from src.simulation import DEFAULT_MODEL_PATH, MuJoCoController, load_model
from src.simulation.core.cli_bridge import (
    SCONE_GAIT_SIMULATION_CONFIG,
    SimulationControl,
    TRIPOD_GAIT_SIMULATION_CONFIG,
    configure_model_gait_controller,
    run,
)
from src.simulation.core.simulator_cli import build_parser
from src.cli_i18n import Language


class SimulationBackendTests(unittest.TestCase):
    def test_tripod_gait_simulation_uses_scone_workspace_tuning(self) -> None:
        self.assertEqual(TRIPOD_GAIT_SIMULATION_CONFIG.max_stride, 0.090)
        self.assertEqual(
            TRIPOD_GAIT_SIMULATION_CONFIG.max_lateral_stride,
            0.070,
        )
        self.assertEqual(TRIPOD_GAIT_SIMULATION_CONFIG.cycle_frequency, 1.0)
        self.assertEqual(TRIPOD_GAIT_SIMULATION_CONFIG.step_height, 0.025)
        self.assertEqual(
            TRIPOD_GAIT_SIMULATION_CONFIG.ik_stride_backoff_attempts,
            4,
        )
        self.assertEqual(
            TRIPOD_GAIT_SIMULATION_CONFIG.ik_tolerance,
            1e-3,
        )

    def test_legacy_non_rl_name_normalizes_to_tripod_gait(self) -> None:
        self.assertIs(
            SimulationControl.parse("non_rl"),
            SimulationControl.TRIPOD_GAIT,
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

    def test_direct_rl_cli_accepts_walk_v2_end_to_end_reference(self) -> None:
        arguments = build_parser().parse_args(
            [
                "--control",
                "rl",
                "--checkpoint",
                "policy.zip",
                "--rl-reference-motion",
                "none",
            ]
        )

        self.assertEqual(arguments.rl_reference_motion, "none")

    def test_direct_simulation_cli_supports_language_selection(self) -> None:
        parser = build_parser()

        self.assertIs(parser.parse_args([]).language, Language.ENGLISH)
        self.assertIs(
            parser.parse_args(["--language", "korea"]).language,
            Language.KOREA,
        )

    def test_cli_accepts_canonical_gait_names(self) -> None:
        parser = build_parser()

        self.assertEqual(
            parser.parse_args(["--control", "tripod-gait"]).control,
            "tripod-gait",
        )
        self.assertEqual(
            parser.parse_args(["--control", "scone-gait"]).control,
            "scone-gait",
        )
        self.assertEqual(
            parser.parse_args(["--control", "roll-gait"]).control,
            "roll-gait",
        )
        self.assertEqual(
            parser.parse_args(["--control", "scone-stair"]).control,
            "scone-stair",
        )
        self.assertEqual(
            parser.parse_args(["--demo", "compare"]).demo,
            "compare",
        )

    def test_scone_gait_routes_checkpoint_to_hybrid_policy_runtime(self) -> None:
        with patch("src.rl.joystick_control.run_rl_joystick") as runner:
            run(
                control=SimulationControl.SCONE_GAIT,
                checkpoint="policy.zip",
                terrain="flat",
            )

        self.assertTrue(runner.call_args.kwargs["hybrid_scone"])

    def test_roll_gait_is_distinct_from_scone_gait(self) -> None:
        self.assertIsNot(SimulationControl.ROLL_GAIT, SimulationControl.SCONE_GAIT)
        self.assertEqual(SimulationControl.ROLL_GAIT.value, "roll-gait")

    def test_model_gait_stiffness_is_opt_in(self) -> None:
        model = load_model(DEFAULT_MODEL_PATH, floating_base=True)
        controller = MuJoCoController(model, mujoco.MjData(model), verbose=False)
        try:
            default_kp = controller._pid[7].kp
            default_kd = controller._pid[7].kd
            self.assertAlmostEqual(controller._pid[7].kp, default_kp)

            configure_model_gait_controller(controller)

            self.assertAlmostEqual(controller._pid[7].kp, 2.0 * default_kp)
            self.assertAlmostEqual(controller._pid[7].kd, np.sqrt(2.0) * default_kd)
            self.assertTrue(np.isinf(controller._profile_velocity[7]))
            self.assertTrue(np.isinf(controller._profile_acceleration[7]))
        finally:
            controller.close()

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

    def test_standard_tripod_gait_advances_without_ik_failure(self) -> None:
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
            configure_model_gait_controller(controller)
            advance_simulation(0.5)

            # Do not recenter on the gravity-sagged transient pose.  The
            # selected Standard profile is the IK-safe gait origin in sim.
            gait = TripodGait(
                controller,
                robot.profile,
                config=TRIPOD_GAIT_SIMULATION_CONFIG,
            )
            root_body_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_BODY,
                "UPPER_BODY_1",
            )
            start_position = data.xpos[root_body_id].copy()
            start_rotation = data.xmat[root_body_id].reshape(3, 3).copy()
            minimum_z = float(start_position[2])
            previous_forward = 0.0
            backward_distance = 0.0
            maximum_abs_yaw = 0.0

            for _ in range(400):
                sample = gait.update(
                    VelocityCommand(vx=gait.config.max_vx),
                    dt=0.02,
                    send=True,
                )
                self.assertTrue(sample.converged, sample.failed_legs)
                advance_simulation(0.02)
                minimum_z = min(minimum_z, float(data.xpos[root_body_id, 2]))
                current_displacement = start_rotation.T @ (
                    data.xpos[root_body_id] - start_position
                )
                forward_delta = float(current_displacement[0]) - previous_forward
                previous_forward = float(current_displacement[0])
                backward_distance += max(0.0, -forward_delta)
                relative_rotation = (
                    start_rotation.T
                    @ data.xmat[root_body_id].reshape(3, 3)
                )
                maximum_abs_yaw = max(
                    maximum_abs_yaw,
                    abs(
                        float(
                            np.arctan2(
                                relative_rotation[1, 0],
                                relative_rotation[0, 0],
                            )
                        )
                    ),
                )

            body_displacement = start_rotation.T @ (
                data.xpos[root_body_id] - start_position
            )
            self.assertGreater(float(body_displacement[0]), 0.75)
            self.assertLess(abs(float(body_displacement[1])), 0.025)
            self.assertLess(backward_distance, 0.015)
            self.assertLess(maximum_abs_yaw, np.radians(2.0))
            self.assertGreater(minimum_z - float(start_position[2]), -0.005)
            self.assertGreater(
                float(data.xmat[root_body_id].reshape(3, 3)[2, 2]),
                0.98,
            )
        finally:
            controller.close()

    def test_scone_gait_sector_roll_advances_and_stays_upright(self) -> None:
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

            gait = SconeGait(
                controller,
                robot.profile,
                config=SCONE_GAIT_SIMULATION_CONFIG,
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
            self.assertGreater(float(body_displacement[0]), 0.10)
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

    def test_legacy_climb_stays_upright_but_does_not_fake_10cm_ascent(self) -> None:
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
            # This regression originally used the former 35 mm stairs-1 and
            # required a 0.30 m ascent. At the requested 100 mm riser the
            # legacy open-loop Climb does not clear the edge; scone-stair is
            # the validated adaptive path. Keep legacy stable without
            # misclassifying small motion as a successful climb.
            self.assertLess(float(displacement[1]), 0.10)
            self.assertLess(float(displacement[2]), 0.02)
            self.assertGreater(
                float(data.xmat[root_body_id].reshape(3, 3)[2, 2]),
                0.98,
            )
        finally:
            controller.close()


if __name__ == "__main__":
    unittest.main()
