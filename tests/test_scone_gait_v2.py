from __future__ import annotations

import unittest

import numpy as np

from src.locomotion import (
    LegRole,
    SconeGaitV2,
    SconeGaitV2Config,
    VelocityCommand,
)
from src.locomotion.sector_wheel import active_support_point


NOMINAL_HEADINGS = {1: 135.0, 2: -135.0, 3: 90.0, 4: -90.0, 5: 45.0, 6: -45.0}


def _roles_for(gait: SconeGaitV2, command: tuple[float, float, float]) -> str:
    """Return a per-leg role string without stepping the gait."""

    travel = gait.contact_travel(np.asarray(command, dtype=np.float64))
    letters = []
    for leg in range(1, 7):
        solution = gait._wheels.solve(
            leg,
            travel[leg - 1],
            max_steering_degrees=gait.config.max_steering_degrees,
        )
        letters.append(
            "R" if solution.alignment >= gait.config.min_roll_alignment else "w"
        )
    return "".join(letters)


class SectorWheelGeometryTests(unittest.TestCase):
    """Freeze the model measurements the whole controller is derived from."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.gait = SconeGaitV2(config=SconeGaitV2Config(command_time_constant=0.0))

    def test_wheels_roll_radially_with_unit_gain_steering(self) -> None:
        for leg, expected in NOMINAL_HEADINGS.items():
            with self.subTest(leg=leg):
                geometry = self.gait._wheels.geometry[leg]
                self.assertAlmostEqual(
                    geometry.heading_degrees, expected, delta=1.0
                )
                self.assertIn(geometry.steering_gain, (-1.0, 1.0))
                self.assertAlmostEqual(geometry.roll_radius, 0.122, delta=0.002)

    def test_stance_pose_is_centred_in_the_arc_the_actuator_can_reach(self) -> None:
        """Both rolling directions have to be equally cheap.

        The profile pose parks each sector against the trailing end of its own
        tread.  The gait stands somewhere else on purpose, and the window it
        plans inside has to come out symmetric -- an asymmetric one is what
        made the two rolling directions behave differently.
        """

        window = self.gait.sector_window_degrees
        for leg in range(1, 7):
            with self.subTest(leg=leg):
                geometry = self.gait._wheels.geometry[leg]
                self.assertLess(geometry.arc_min_degrees, -120.0)
                self.assertGreater(geometry.arc_max_degrees, 80.0)
                low, high = window[leg - 1]
                self.assertGreater(high, 60.0)
                self.assertLess(low, -60.0)
                self.assertLess(abs(low + high), 10.0)

    def test_stance_pose_does_not_change_how_the_robot_stands(self) -> None:
        stance = self.gait.nominal_motor_degrees
        profile = self.gait._profile_motor_degrees(self.gait.profile)
        self.assertGreater(float(np.max(np.abs(stance[12:] - profile[12:]))), 60.0)
        np.testing.assert_allclose(stance[:12], profile[:12])
        for leg in range(1, 7):
            with self.subTest(leg=leg):
                parked = active_support_point(self.gait.kinematics, leg, profile)
                stood = active_support_point(self.gait.kinematics, leg, stance)
                self.assertLess(abs(parked[2] - stood[2]), 0.002)

    def test_stage_two_rotation_does_not_move_the_contact(self) -> None:
        """The measurement that licenses adding the sector angle to IK.

        If this stops holding, ``SconeGaitV2`` is displacing the foot every
        time it rolls and the whole decomposition is wrong.
        """

        kinematics = self.gait.kinematics
        nominal = self.gait.nominal_motor_degrees
        window = self.gait.sector_window_degrees
        for leg in range(1, 7):
            reference = active_support_point(kinematics, leg, nominal)
            low, high = window[leg - 1]
            for offset in np.arange(low, high + 1e-9, 10.0):
                with self.subTest(leg=leg, offset=round(float(offset))):
                    probe = nominal.copy()
                    probe[leg + 11] += float(offset)
                    contact = active_support_point(kinematics, leg, probe)
                    # The 4 mm allowance is the mesh's own faceting: the patch
                    # centroid hops between facets as the arc turns.
                    self.assertLess(
                        float(np.linalg.norm(contact[:2] - reference[:2])),
                        0.004,
                    )
                    self.assertLess(abs(contact[2] - reference[2]), 0.002)


class RoleAssignmentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.gait = SconeGaitV2(config=SconeGaitV2Config(command_time_constant=0.0))

    def test_forward_rolls_the_corners_and_walks_the_middle_pair(self) -> None:
        self.assertEqual(_roles_for(self.gait, (0.30, 0.0, 0.0)), "RRwwRR")
        self.assertEqual(_roles_for(self.gait, (-0.30, 0.0, 0.0)), "RRwwRR")

    def test_lateral_travel_rolls_every_leg(self) -> None:
        self.assertEqual(_roles_for(self.gait, (0.0, 0.20, 0.0)), "RRRRRR")

    def test_yaw_cannot_be_rolled_and_falls_back_to_walking(self) -> None:
        # A pure turn needs every wheel steered 90 degrees off its radial
        # heading, which no steering limit inside the footprint can reach.
        self.assertEqual(_roles_for(self.gait, (0.0, 0.0, 0.6)), "wwwwww")

    def test_roles_are_adopted_at_lift_off_not_mid_stance(self) -> None:
        gait = SconeGaitV2(config=SconeGaitV2Config(command_time_constant=0.0))
        first = gait.step(VelocityCommand(vx=0.30), dt=0.02)
        self.assertTrue(all(role is LegRole.STEP for role in gait.leg_roles.values()))
        self.assertTrue(first.converged, first.failed_legs)
        for _ in range(60):
            gait.step(VelocityCommand(vx=0.30), dt=0.02)
        rolling = gait.rolling_legs
        self.assertEqual(rolling, (1, 2, 5, 6))


class RollingStrokeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SconeGaitV2Config(command_time_constant=0.0)
        self.gait = SconeGaitV2(config=self.config)

    def test_rolling_takes_the_stroke_out_of_the_ik_request(self) -> None:
        """This substitution, not a wider stroke, is where the speed comes from."""

        for _ in range(60):
            self.gait.step(VelocityCommand(vx=0.30), dt=0.02)
        travel = self.gait.contact_travel(np.array([0.30, 0.0, 0.0]))
        for leg in self.gait.rolling_legs:
            with self.subTest(leg=leg):
                residual = self.gait._articulated_velocity(leg - 1, travel[leg - 1])
                self.assertLess(
                    float(np.linalg.norm(residual)),
                    0.35 * float(np.linalg.norm(travel[leg - 1])),
                )
        for leg in (3, 4):
            with self.subTest(leg=leg):
                residual = self.gait._articulated_velocity(leg - 1, travel[leg - 1])
                np.testing.assert_allclose(residual, travel[leg - 1])

    def test_sector_angle_stays_inside_the_arc_and_the_actuator_range(self) -> None:
        window = self.gait.sector_window_degrees
        for _ in range(400):
            sample = self.gait.step(VelocityCommand(vx=0.35, vy=0.05), dt=0.02)
            self.assertTrue(sample.converged, sample.failed_legs)
            sector = self.gait.sector_degrees
            self.assertTrue(np.all(sector >= window[:, 0] - 1e-9))
            self.assertTrue(np.all(sector <= window[:, 1] + 1e-9))
            self.assertTrue(np.all(sample.motor_degrees >= 0.0))
            self.assertTrue(np.all(sample.motor_degrees <= 360.0))

    def test_at_least_three_legs_stay_grounded(self) -> None:
        for _ in range(400):
            sample = self.gait.step(VelocityCommand(vx=0.35), dt=0.02)
            self.assertGreaterEqual(len(sample.stance_legs), 3)

    def test_idle_command_holds_the_nominal_pose(self) -> None:
        sample = self.gait.step(VelocityCommand(), dt=0.02)

        np.testing.assert_allclose(
            sample.motor_degrees,
            self.gait.nominal_motor_degrees,
            atol=1e-10,
        )

    def test_gait_degrades_to_walking_when_nothing_can_roll(self) -> None:
        gait = SconeGaitV2(
            config=SconeGaitV2Config(
                command_time_constant=0.0,
                max_steering_degrees=0.0,
                min_roll_alignment=0.99,
            )
        )
        for _ in range(120):
            sample = gait.step(VelocityCommand(vx=0.18), dt=0.02)
            self.assertTrue(sample.converged, sample.failed_legs)
        self.assertEqual(gait.rolling_legs, ())
        np.testing.assert_allclose(gait.sector_degrees, np.zeros(6), atol=1e-12)


class ConfigValidationTests(unittest.TestCase):
    def test_invalid_tuning_is_rejected(self) -> None:
        for arguments in (
            {"roll_authority": 1.1},
            {"max_steering_degrees": 91.0},
            {"min_roll_alignment": -0.1},
            {"minimum_rolling_legs": 7},
            {"arc_margin_degrees": -1.0},
            {"roll_derate_degrees": 0.0},
            {"max_roll_rate_degrees": 0.0},
            {"reindex_threshold": 0.0},
            {"min_arc_reserve_degrees": 0.0},
            {"arc_reserve_safety": 0.9},
            {"swing_lift_reference_speed": 0.0},
            {"ik_branch_guard_degrees": 0.0},
        ):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                SconeGaitV2Config(**arguments)


class FlatGroundTests(unittest.TestCase):
    """One physics rollout, because the point of the gait is a measured speed."""

    def test_forward_travel_beats_the_walking_stride_ceiling(self) -> None:
        from benchmark.common import BenchmarkConfig
        from benchmark.flat import run_flat_trial

        record = run_flat_trial(
            "role-split-scone",
            (0.45, 0.0, 0.0),
            command_name="forward",
            config=BenchmarkConfig(measure_seconds=4.0),
        )

        # articulated-walk tops out at 0.1125 m/s on this model because its
        # stroke is capped at 90 mm; 0.18 m/s cannot be reached by walking.
        self.assertGreater(record["mean_vx_mps"], 0.18)
        self.assertTrue(record["completed"])
        self.assertIsNone(record["termination_reason"])
        self.assertLess(abs(record["yaw_change_degrees"]), 10.0)
        self.assertLess(
            abs(record["displacement_y_m"]),
            0.30 * abs(record["displacement_x_m"]),
        )
        self.assertGreater(record["minimum_upright"], 0.9)


if __name__ == "__main__":
    unittest.main()
