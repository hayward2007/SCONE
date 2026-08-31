from __future__ import annotations

import unittest

import mujoco
import numpy as np

from src.locomotion.non_rl_walk import GaitConfig, NonRLWalk, VelocityCommand
from src.locomotion.profile import STANDARD


class RecordingController:
    def __init__(self) -> None:
        self.positions: dict[int, float] | None = None
        self.raw_positions = {motor_id: 2048 + motor_id for motor_id in range(1, 19)}

    def set_positions(self, positions) -> None:
        self.positions = dict(positions)

    def get_position(self, motor_id: int) -> int:
        return self.raw_positions[motor_id]


class NonRLWalkTests(unittest.TestCase):
    def setUp(self) -> None:
        self.gait = NonRLWalk(
            config=GaitConfig(
                command_time_constant=0.0,
                ik_tolerance=2e-4,
                max_vx=0.12,
                max_vy=0.08,
                max_yaw_rate=0.6,
            )
        )

    def test_idle_command_holds_nominal_stance(self) -> None:
        sample = self.gait.step(VelocityCommand(), dt=0.02)

        np.testing.assert_allclose(
            sample.foot_targets,
            self.gait.nominal_foot_positions,
            atol=1e-12,
        )
        self.assertEqual(sample.stance_legs, (1, 2, 3, 4, 5, 6))
        self.assertTrue(sample.converged)

    def test_tuning_limits_are_validated(self) -> None:
        invalid = (
            {"max_stride": 0.05, "max_lateral_stride": 0.06},
            {"ik_stride_backoff_attempts": -1},
            {"ik_stride_backoff_factor": 1.0},
        )

        for arguments in invalid:
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                GaitConfig(**arguments)

    def test_nominal_support_points_share_ground_height(self) -> None:
        heights = self.gait.nominal_foot_positions[:, 2]

        self.assertLess(float(np.ptp(heights)), 0.002)

    def test_support_points_use_sector_tip_patch_centre(self) -> None:
        for leg, leg_kinematics in self.gait.kinematics.legs.items():
            geom_id = leg_kinematics._required_id(
                mujoco.mjtObj.mjOBJ_GEOM,
                f"TIRE_{leg}_geom",
            )
            # TIRE mesh thickness is its local X axis. The centred support
            # point must lie on the geom centre plane, not either outer edge.
            self.assertAlmostEqual(
                float(leg_kinematics.end_effector_point[0]),
                float(leg_kinematics.model.geom_pos[geom_id, 0]),
                places=6,
            )

    def test_tripods_alternate_support(self) -> None:
        command = VelocityCommand(vx=0.04)
        _, stance_at_zero = self.gait.foot_targets(command, phase=0.0)
        _, stance_at_half = self.gait.foot_targets(command, phase=0.5)

        self.assertEqual(stance_at_zero, NonRLWalk.TRIPOD_A)
        self.assertEqual(stance_at_half, NonRLWalk.TRIPOD_B)

    def test_gait_uses_fixed_hardware_compatible_cadence(self) -> None:
        gait = NonRLWalk(
            config=GaitConfig(
                cycle_frequency=0.8,
                max_vx=0.5,
                command_time_constant=0.0,
            )
        )

        sample = gait.step([0.50, 0.0, 0.0], dt=0.02)

        self.assertAlmostEqual(sample.cycle_frequency, 0.8)

    def test_sample_reports_stride_clipping(self) -> None:
        gait = NonRLWalk(
            config=GaitConfig(
                cycle_frequency=0.7,
                max_stride=0.060,
                max_lateral_stride=0.050,
                max_vx=0.5,
                ik_stride_backoff_attempts=4,
                command_time_constant=0.0,
            )
        )

        idle = gait.step([0.0, 0.0, 0.0], dt=0.02)
        fast = gait.step([0.5, 0.0, 0.0], dt=0.02)

        self.assertEqual(idle.stride_clip_fraction, 0.0)
        self.assertEqual(fast.stride_clip_fraction, 1.0)

    def test_yaw_generates_leg_specific_tangential_strokes(self) -> None:
        command = np.array([0.0, 0.0, 0.3])
        targets, _ = self.gait.foot_targets(command, phase=0.0)
        offsets = targets - self.gait.nominal_foot_positions

        # Rotation about the body cannot be represented by one common XY
        # translation: at least two legs must receive different directions.
        self.assertGreater(np.linalg.norm(offsets[0, :2] - offsets[2, :2]), 1e-6)

    def test_forward_frames_solve_and_stay_within_motor_range(self) -> None:
        for _ in range(50):
            sample = self.gait.step([0.035, 0.0, 0.0], dt=0.02)
            self.assertTrue(sample.converged, sample.failed_legs)
            self.assertTrue(np.all(sample.motor_degrees >= 0.0))
            self.assertTrue(np.all(sample.motor_degrees <= 360.0))

    def test_standard_stance_solves_full_forward_stride(self) -> None:
        gait = NonRLWalk(
            profile=STANDARD,
            config=GaitConfig(
                command_time_constant=0.0,
                cycle_frequency=0.7,
                max_stride=0.060,
                max_lateral_stride=0.050,
                ik_tolerance=1e-3,
                ik_stride_backoff_attempts=4,
            ),
        )

        for _ in range(100):
            sample = gait.step([gait.config.max_vx, 0.0, 0.0], dt=0.02)
            self.assertTrue(sample.converged, sample.failed_legs)

    def test_standard_stance_solves_tuned_lateral_stride(self) -> None:
        gait = NonRLWalk(
            profile=STANDARD,
            config=GaitConfig(
                command_time_constant=0.0,
                cycle_frequency=0.7,
                max_stride=0.060,
                max_lateral_stride=0.050,
                max_vy=0.25,
                ik_tolerance=1e-3,
                ik_stride_backoff_attempts=4,
            ),
        )

        for direction in (-1.0, 1.0):
            gait.reset()
            for _ in range(100):
                sample = gait.step([0.0, direction * 0.20, 0.0], dt=0.02)
                self.assertTrue(sample.converged, sample.failed_legs)

    def test_standard_stance_solves_combined_tuned_stride(self) -> None:
        gait = NonRLWalk(
            profile=STANDARD,
            config=GaitConfig(
                command_time_constant=0.0,
                cycle_frequency=0.7,
                max_stride=0.060,
                max_lateral_stride=0.050,
                max_vx=0.5,
                max_vy=0.25,
                max_yaw_rate=0.8,
                ik_tolerance=1e-3,
                ik_stride_backoff_attempts=4,
            ),
        )

        for _ in range(100):
            sample = gait.step([0.25, 0.15, 0.30], dt=0.02)
            self.assertTrue(sample.converged, sample.failed_legs)

    def test_ik_backoff_recovers_oversized_combined_stride(self) -> None:
        gait = NonRLWalk(
            profile=STANDARD,
            config=GaitConfig(
                command_time_constant=0.0,
                cycle_frequency=0.7,
                max_stride=0.070,
                max_lateral_stride=0.070,
                max_vx=0.5,
                max_vy=0.25,
                max_yaw_rate=0.8,
                ik_tolerance=1e-3,
                ik_stride_backoff_attempts=4,
            ),
        )

        minimum_backoff_scale = 1.0
        for _ in range(100):
            sample = gait.step([0.5, 0.25, 0.8], dt=0.02)
            self.assertTrue(sample.converged, sample.failed_legs)
            minimum_backoff_scale = min(
                minimum_backoff_scale,
                sample.ik_backoff_scale,
            )
        self.assertLess(minimum_backoff_scale, 1.0)

    def test_send_is_one_batch_of_eighteen_positions(self) -> None:
        controller = RecordingController()
        gait = NonRLWalk(
            controller=controller,
            config=GaitConfig(command_time_constant=0.0),
        )
        sample = gait.step([0.02, 0.0, 0.0], dt=0.02)
        gait.send(sample)

        self.assertIsNotNone(controller.positions)
        self.assertEqual(set(controller.positions or {}), set(range(1, 19)))

    def test_reset_from_controller_uses_present_raw_positions(self) -> None:
        controller = RecordingController()
        gait = NonRLWalk(controller=controller)

        degrees = gait.reset_from_controller()

        expected = np.array(
            [controller.raw_positions[motor_id] for motor_id in range(1, 19)]
        ) / 4096.0 * 360.0
        np.testing.assert_allclose(degrees, expected)


if __name__ == "__main__":
    unittest.main()
