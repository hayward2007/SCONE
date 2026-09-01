from __future__ import annotations

import unittest

import numpy as np

from src.locomotion import SconeGait, SconeGaitConfig, VelocityCommand


class SconeGaitTests(unittest.TestCase):
    def setUp(self) -> None:
        self.gait = SconeGait(
            config=SconeGaitConfig(command_time_constant=0.0)
        )

    def test_sector_tangent_calibration_is_finite_for_every_leg(self) -> None:
        command = VelocityCommand(vx=0.08, vy=0.03, yaw_rate=0.2)

        solutions = [
            self.gait.steering_solution(leg, command)
            for leg in range(1, 7)
        ]

        self.assertTrue(np.isfinite(np.asarray(solutions)).all())
        self.assertTrue(
            all(
                abs(steering) <= self.gait.config.max_steering_degrees
                and polarity in (-1.0, 1.0)
                and 0.0 <= alignment <= 1.0
                for steering, polarity, alignment in solutions
            )
        )

    def test_active_command_sweeps_sector_joints_within_bounds(self) -> None:
        lower_frames = []
        for _ in range(80):
            sample = self.gait.step(VelocityCommand(vx=0.08), dt=0.02)
            lower_frames.append(sample.motor_degrees[12:].copy())
            self.assertTrue(sample.converged, sample.failed_legs)

        lower_frames = np.asarray(lower_frames)
        self.assertGreater(float(np.ptp(lower_frames)), 5.0)
        self.assertTrue(np.all(lower_frames >= 0.0))
        self.assertTrue(np.all(lower_frames <= 360.0))

    def test_idle_command_holds_tripod_nominal_pose(self) -> None:
        sample = self.gait.step(VelocityCommand(), dt=0.02)

        np.testing.assert_allclose(
            sample.motor_degrees,
            self.gait.nominal_motor_degrees,
            atol=1e-10,
        )

    def test_invalid_sector_tuning_is_rejected(self) -> None:
        for arguments in (
            {"sector_sweep_degrees": 0.0},
            {"steering_blend": 1.1},
            {"minimum_roll_alignment": -0.1},
        ):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                SconeGaitConfig(**arguments)


if __name__ == "__main__":
    unittest.main()
