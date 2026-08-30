from __future__ import annotations

import unittest

import numpy as np

from src.locomotion.non_rl_walk import GaitConfig, NonRLWalk, VelocityCommand


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

    def test_nominal_support_points_share_ground_height(self) -> None:
        heights = self.gait.nominal_foot_positions[:, 2]

        self.assertLess(float(np.ptp(heights)), 0.002)

    def test_tripods_alternate_support(self) -> None:
        command = VelocityCommand(vx=0.04)
        _, stance_at_zero = self.gait.foot_targets(command, phase=0.0)
        _, stance_at_half = self.gait.foot_targets(command, phase=0.5)

        self.assertEqual(stance_at_zero, NonRLWalk.TRIPOD_A)
        self.assertEqual(stance_at_half, NonRLWalk.TRIPOD_B)

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
