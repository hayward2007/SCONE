from __future__ import annotations

import unittest

import numpy as np

from src.kinematics import JointAngles, LegKinematics, RobotKinematics


SPORT_MOTOR_DEGREES = np.array(
    [135, 135, 180, 180, 225, 225] + [170] * 6 + [195] * 6,
    dtype=np.float64,
)
SPORT_RADIANS = np.radians(SPORT_MOTOR_DEGREES - 180.0)


class LegKinematicsTests(unittest.TestCase):
    def test_motor_degree_conversion_round_trip(self) -> None:
        angles = JointAngles.from_motor_degrees([135, 170, 195])

        np.testing.assert_allclose(
            angles.as_motor_degrees(), [135, 170, 195], atol=1e-12
        )
        np.testing.assert_allclose(
            JointAngles.from_raw(angles.as_raw()).as_motor_degrees(),
            angles.as_raw() / 4096.0 * 360.0,
            atol=1e-12,
        )

    def test_every_leg_fk_ik_round_trip(self) -> None:
        for leg in range(1, 7):
            with self.subTest(leg=leg):
                kinematics = LegKinematics(leg)
                expected = JointAngles(
                    SPORT_RADIANS[leg - 1],
                    SPORT_RADIANS[leg + 5],
                    SPORT_RADIANS[leg + 11],
                )
                target = kinematics.fk(expected).position
                initial = expected.as_array() + np.array([0.03, -0.02, 0.01])

                result = kinematics.ik(
                    target,
                    initial_angles=initial,
                    tolerance=1e-7,
                    max_iterations=100,
                )

                self.assertTrue(result.converged)
                self.assertLess(result.residual, 1e-7)
                reproduced = kinematics.fk(result.angles).position
                np.testing.assert_allclose(reproduced, target, atol=1e-7)


class RobotKinematicsTests(unittest.TestCase):
    def test_whole_robot_fk_ik_and_actuator_order(self) -> None:
        kinematics = RobotKinematics()
        poses = kinematics.fk(SPORT_RADIANS)
        targets = np.stack([poses[leg].position for leg in range(1, 7)])
        initial = SPORT_RADIANS + 0.015

        results = kinematics.ik(
            targets,
            initial_angles=initial,
            tolerance=1e-7,
            max_iterations=100,
        )

        self.assertTrue(all(result.converged for result in results.values()))
        solved = kinematics.results_as_actuator_radians(results)
        reproduced = kinematics.fk(solved)
        for leg in range(1, 7):
            np.testing.assert_allclose(
                reproduced[leg].position, targets[leg - 1], atol=1e-7
            )


if __name__ == "__main__":
    unittest.main()
