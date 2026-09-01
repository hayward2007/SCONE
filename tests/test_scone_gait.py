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
            {"point_support_ratio": 1.0},
            {"swing_roll_hold_ratio": 1.0},
            {"effective_roll_radius": 0.0},
            {"max_roll_rate_degrees": 0.0},
        ):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                SconeGaitConfig(**arguments)

    def test_sector_holds_one_phase_before_late_stance_propulsion(self) -> None:
        duty = self.gait.config.duty_factor
        hold = self.gait.config.point_support_ratio

        self.gait.reset(phase=0.0)
        start = self.gait.roll_coordinate(1)
        self.gait.reset(phase=duty * hold * 0.95)
        end_of_hold = self.gait.roll_coordinate(1)
        self.gait.reset(phase=duty * (hold + 0.5 * (1.0 - hold)))
        propulsion = self.gait.roll_coordinate(1)

        self.assertEqual(start, -0.5)
        self.assertEqual(end_of_hold, -0.5)
        self.assertGreater(propulsion, end_of_hold)

    def test_continuous_rotation_accumulates_instead_of_reversing(self) -> None:
        gait = SconeGait(
            config=SconeGaitConfig(
                command_time_constant=0.0,
                cycle_frequency=1.2,
                max_vx=0.5,
                continuous_rotation=True,
                rolling_blend=1.0,
            )
        )
        history = []

        for _ in range(200):
            sample = gait.step(VelocityCommand(vx=0.5), dt=0.02)
            self.assertTrue(sample.converged, sample.failed_legs)
            history.append(gait.continuous_roll_degrees)

        history = np.asarray(history)
        self.assertGreater(float(np.max(np.abs(history[-1]))), 250.0)
        for leg in range(6):
            differences = np.diff(history[:, leg])
            self.assertFalse(
                np.any(differences > 1e-9) and np.any(differences < -1e-9)
            )

    def test_roll_gate_holds_then_rotates_without_reverse(self) -> None:
        duty = self.gait.config.duty_factor
        hold = self.gait.config.point_support_ratio

        self.gait.reset(phase=duty * hold * 0.95)
        point_support = self.gait.roll_gate(1)
        self.gait.reset(phase=duty * (hold + 0.8 * (1.0 - hold)))
        late_stance = self.gait.roll_gate(1)
        self.gait.reset(phase=duty + 0.1 * (1.0 - duty))
        early_swing = self.gait.roll_gate(1)

        self.assertEqual(point_support, 0.0)
        self.assertGreater(late_stance, 0.0)
        self.assertGreater(early_swing, 0.0)


if __name__ == "__main__":
    unittest.main()
