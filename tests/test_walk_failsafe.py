"""Fail-safe trainer: leg loss, the schedule it forces, and the reward budget."""

from __future__ import annotations

import math
import unittest

import mujoco
import numpy as np

from src.locomotion.fault_gait import (
    FaultAdaptiveGait,
    LEG_INDICES,
    MAX_BODY_SHIFT_M,
    plan_schedule,
)
from src.locomotion.support_polygon import (
    UNSUPPORTED_MARGIN,
    convex_hull,
    polygon_area,
    stability_margin,
)
from src.locomotion.tripod_gait import TripodGait
from src.rl.policy_compat import (
    FAILSAFE_OBSERVATION_SHAPE,
    is_failsafe_checkpoint,
    task_for_observation_shape,
)
from src.rl.stance import STANDARD_STANDING_DEGREES
from src.simulation.core.model import (
    DETACHED_MASS_SCALE,
    LegFailureRuntime,
    amputate_legs,
    load_model,
    validate_failed_legs,
)


MUTATED_FIELDS = (
    "geom_contype",
    "geom_conaffinity",
    "geom_rgba",
    "body_mass",
    "body_inertia",
    "actuator_ctrlrange",
    "actuator_ctrllimited",
    "dof_frictionloss",
)


class SupportPolygonTests(unittest.TestCase):
    SQUARE = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))

    def test_margin_is_signed_distance_to_the_boundary(self) -> None:
        self.assertAlmostEqual(stability_margin((0.5, 0.5), self.SQUARE), 0.5)
        self.assertAlmostEqual(stability_margin((0.0, 0.5), self.SQUARE), 0.0)
        self.assertAlmostEqual(stability_margin((1.5, 0.5), self.SQUARE), -0.5)

    def test_contact_order_does_not_matter(self) -> None:
        shuffled = ((1.0, 1.0), (0.0, 0.0), (0.0, 1.0), (1.0, 0.0))
        self.assertAlmostEqual(
            stability_margin((0.5, 0.5), shuffled),
            stability_margin((0.5, 0.5), self.SQUARE),
        )

    def test_degenerate_support_is_ranked_below_any_polygon(self) -> None:
        # A walking robot passes through these every stride, so they have to be
        # answers rather than exceptions -- and ordered consistently.
        segment = stability_margin((0.5, 0.5), ((0.0, 0.0), (1.0, 0.0)))
        point = stability_margin((0.5, 0.5), ((0.0, 0.0),))
        self.assertAlmostEqual(segment, -0.5)
        self.assertLess(point, segment)
        self.assertEqual(stability_margin((0.0, 0.0), ()), UNSUPPORTED_MARGIN)

    def test_hull_drops_collinear_points_and_reports_zero_area(self) -> None:
        self.assertEqual(len(convex_hull(((0, 0), (1, 1), (2, 2)))), 2)
        self.assertAlmostEqual(polygon_area(((0, 0), (1, 1), (2, 2))), 0.0)
        self.assertAlmostEqual(polygon_area(self.SQUARE), 1.0)


class LegAmputationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.model = load_model()
        self.runtime = LegFailureRuntime(self.model)

    def test_the_model_layout_never_changes(self) -> None:
        # One policy has to cover the healthy robot and every fault, so the
        # observation, the action and the joint order must be fault-invariant.
        nominal = load_model()
        for mode in ("detached", "limp"):
            for legs in ([5], [2, 5]):
                faulted = load_model(xml_transform=amputate_legs(legs, mode))
                self.assertEqual(
                    (faulted.nq, faulted.nv, faulted.nu),
                    (nominal.nq, nominal.nv, nominal.nu),
                )

    def test_detaching_removes_mass_collision_and_drawing(self) -> None:
        faulted = load_model(xml_transform=amputate_legs([5], "detached"))
        tire = mujoco.mj_name2id(faulted, mujoco.mjtObj.mjOBJ_GEOM, "TIRE_5_geom")
        self.assertEqual(int(faulted.geom_contype[tire]), 0)
        self.assertEqual(float(faulted.geom_rgba[tire, 3]), 0.0)
        lost = float(load_model().body_mass.sum() - faulted.body_mass.sum())
        self.assertGreater(lost, 0.4)
        # Scaled, not zeroed: MuJoCo rejects a jointed body whose inertia is
        # below mjMINVAL, and every body of a detached leg still carries one.
        self.assertLess(DETACHED_MASS_SCALE, 0.01)
        stub = mujoco.mj_name2id(faulted, mujoco.mjtObj.mjOBJ_BODY, "ARC_SHAPED_WHEEL_5")
        self.assertGreater(float(faulted.body_mass[stub]), 0.0)
        self.assertLess(float(faulted.body_mass[stub]), 0.001)

    def test_a_limp_leg_keeps_its_mass_but_loses_its_drive(self) -> None:
        faulted = load_model(xml_transform=amputate_legs([5], "limp"))
        self.assertAlmostEqual(
            float(faulted.body_mass.sum()), float(load_model().body_mass.sum())
        )
        actuator = mujoco.mj_name2id(
            faulted, mujoco.mjtObj.mjOBJ_ACTUATOR, "A11_xm430_w350"
        )
        self.assertLess(float(faulted.actuator_ctrlrange[actuator, 1]), 1e-3)
        joint = mujoco.mj_name2id(faulted, mujoco.mjtObj.mjOBJ_JOINT, "M11_stage1_L5")
        friction = float(faulted.dof_frictionloss[faulted.jnt_dofadr[joint]])
        self.assertGreater(friction, 0.0)

    def test_untouched_legs_are_untouched(self) -> None:
        faulted = load_model(xml_transform=amputate_legs([5], "detached"))
        tire = mujoco.mj_name2id(faulted, mujoco.mjtObj.mjOBJ_GEOM, "TIRE_3_geom")
        self.assertEqual(int(faulted.geom_contype[tire]), 1)
        self.assertEqual(float(faulted.geom_rgba[tire, 3]), 1.0)

    def test_runtime_mutation_matches_the_compiler(self) -> None:
        # An episode samples a new fault every reset; recompiling six meshes
        # each time is far too slow, so the runtime path has to be exact.
        for mode in ("detached", "limp"):
            for legs in ([], [5], [1], [2, 5], [1, 2, 3]):
                compiled = load_model(xml_transform=amputate_legs(legs, mode))
                self.runtime.apply(legs, mode)
                for field in MUTATED_FIELDS:
                    np.testing.assert_allclose(
                        np.asarray(getattr(compiled, field), dtype=np.float64),
                        np.asarray(getattr(self.model, field), dtype=np.float64),
                        rtol=1e-9,
                        atol=1e-12,
                        err_msg=f"{field} diverged for {legs} in {mode}",
                    )

    def test_restore_returns_the_compiled_values(self) -> None:
        self.runtime.apply([1, 4], "detached")
        self.runtime.restore()
        pristine = load_model()
        for field in MUTATED_FIELDS:
            np.testing.assert_allclose(
                np.asarray(getattr(self.model, field), dtype=np.float64),
                np.asarray(getattr(pristine, field), dtype=np.float64),
            )

    def test_unknown_legs_and_modes_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            validate_failed_legs([7])
        with self.assertRaises(ValueError):
            amputate_legs([1], "sheared-off")


class FaultScheduleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.gait = FaultAdaptiveGait()
        cls.feet = np.asarray(cls.gait._nominal_feet, dtype=np.float64)
        cls.center = cls.feet[:, :2].mean(axis=0)

    def test_an_intact_robot_keeps_the_alternating_tripod(self) -> None:
        schedule = plan_schedule(LEG_INDICES, self.feet, self.center)
        self.assertEqual(schedule.pattern, "tripod")
        self.assertAlmostEqual(schedule.duty_factor, 0.5)
        self.assertEqual(schedule.phase_offsets, dict(TripodGait.PHASE_OFFSETS))
        self.assertTrue(schedule.statically_stable)

    def test_losing_a_leg_forces_one_swing_at_a_time(self) -> None:
        for lost in LEG_INDICES:
            healthy = [leg for leg in LEG_INDICES if leg != lost]
            schedule = plan_schedule(healthy, self.feet, self.center)
            self.assertEqual(schedule.pattern, "wave")
            self.assertAlmostEqual(schedule.duty_factor, 4 / 5)
            self.assertEqual(sorted(schedule.phase_offsets), healthy)
            # Exactly one leg is in swing at every phase, so four feet are
            # always down. This is the property the tripod cannot provide.
            for step in range(50):
                phase = step / 50
                swinging = [
                    leg
                    for leg in healthy
                    if (phase + schedule.phase_offsets[leg]) % 1.0
                    >= schedule.duty_factor
                ]
                self.assertLessEqual(len(swinging), 1)

    def test_the_tripod_scaffold_is_unstable_the_moment_a_leg_is_lost(self) -> None:
        # This is why the trainer exists: the alternating tripod's own support
        # group becomes a two-foot line, so no residual can rescue it.
        for lost in LEG_INDICES:
            group = [
                leg
                for leg in (TripodGait.TRIPOD_A if lost in TripodGait.TRIPOD_A
                            else TripodGait.TRIPOD_B)
                if leg != lost
            ]
            margin = stability_margin(
                self.center, self.feet[[leg - 1 for leg in group], :2]
            )
            self.assertLess(margin, 0.0)

    def test_the_body_shift_restores_a_positive_margin_on_five_legs(self) -> None:
        for lost in LEG_INDICES:
            healthy = [leg for leg in LEG_INDICES if leg != lost]
            schedule = plan_schedule(healthy, self.feet, self.center)
            self.assertTrue(
                schedule.statically_stable,
                f"losing leg {lost} left the scaffold at {schedule.worst_margin:+.4f}",
            )
            self.assertLessEqual(
                float(np.linalg.norm(schedule.body_shift_xy)),
                MAX_BODY_SHIFT_M + 1e-9,
            )

    def test_the_shift_is_reachable_and_bounded_in_joint_space(self) -> None:
        for lost in (1, 3, 5):
            self.gait.set_healthy_legs(
                [leg for leg in LEG_INDICES if leg != lost],
                center_of_mass_xy=self.center,
            )
            offset = self.gait.stance_offset_degrees()
            self.assertEqual(offset.shape, (18,))
            # Bounded well inside the conservative +-60 degree joint envelope
            # the trainer clips targets to.
            self.assertLess(float(np.max(np.abs(offset))), 45.0)
            np.testing.assert_allclose(
                self.gait.realised_shift_xy,
                self.gait.schedule.body_shift_xy,
                atol=1e-9,
            )

    def test_three_legs_is_the_floor(self) -> None:
        with self.assertRaises(ValueError):
            plan_schedule([1, 2], self.feet, self.center)


class FailsafeEnvironmentTests(unittest.TestCase):
    @staticmethod
    def _env(**kwargs):
        from src.rl.walk_failsafe import SconeFailsafeEnv

        kwargs.setdefault("standing_pose_degrees", STANDARD_STANDING_DEGREES)
        return SconeFailsafeEnv(curriculum="full", **kwargs)

    @staticmethod
    def _roll(env, steps: int):
        """Drive the bare scaffold and return body-frame velocity and margins."""

        action = np.zeros(18, dtype=np.float32)
        rotation = env.data.xmat[env.root_body_id].reshape(3, 3).copy()
        start = env.data.qpos[
            env.root_qpos_address : env.root_qpos_address + 3
        ].copy()
        margins: list[float] = []
        contacts = np.zeros(6)
        for _ in range(steps):
            _obs, _reward, terminated, truncated, info = env.step(action)
            margins.append(info["support_margin"])
            flags, _margin, _offset = env._cached_contacts
            contacts += flags
            if terminated or truncated:
                break
        moved = rotation.T @ (
            env.data.qpos[env.root_qpos_address : env.root_qpos_address + 3] - start
        )
        seconds = len(margins) * env.control_dt
        return moved[:2] / seconds, np.array(margins), contacts / len(margins)

    def test_the_observation_width_identifies_this_trainer(self) -> None:
        env = self._env(fixed_failed_legs=[5])
        observation, info = env.reset(seed=0)
        self.assertEqual(observation.shape, FAILSAFE_OBSERVATION_SHAPE)
        self.assertEqual(info["failed_legs"], (5,))
        self.assertTrue(is_failsafe_checkpoint(observation.shape))
        self.assertEqual(
            task_for_observation_shape(observation.shape), "walk-failsafe"
        )
        env.close()

    def test_the_health_mask_and_contact_flags_report_the_fault(self) -> None:
        env = self._env(fixed_failed_legs=[5])
        observation, _info = env.reset(seed=0)
        health = observation[70:76]
        np.testing.assert_allclose(health, [1, 1, 1, 1, 0, 1])
        for _ in range(50):
            observation, *_rest = env.step(np.zeros(18, dtype=np.float32))
        # A detached leg has no collision geometry, so it can never report one.
        self.assertEqual(float(observation[76 + 4]), 0.0)
        env.close()

    def test_a_failed_leg_receives_no_residual(self) -> None:
        env = self._env(fixed_failed_legs=[5])
        env.reset(seed=0)
        np.testing.assert_allclose(
            env.action_mask(), [1, 1, 1, 1, 0, 1] * 3
        )
        env.step(np.ones(18, dtype=np.float32))
        np.testing.assert_allclose(env._last_action[[4, 10, 16]], 0.0)
        env.close()

    def test_the_scaffold_tracks_the_command_it_was_calibrated_for(self) -> None:
        # Accuracy is the trainer's only positive term, so a scaffold that
        # cannot answer its own curriculum would leave nothing to learn.
        for failed, command, tolerance in (
            ([], 0.06, 0.35),
            ([], 0.12, 0.35),
            ([5], 0.06, 0.45),
        ):
            env = self._env(fixed_command=[command, 0.0, 0.0], fixed_failed_legs=failed)
            env.reset(seed=0)
            velocity, _margins, _contacts = self._roll(env, 300)
            env.close()
            self.assertAlmostEqual(
                velocity[0] / command, 1.0, delta=tolerance,
                msg=f"legs lost {failed} at {command} m/s gave {velocity[0]:.4f}",
            )

    def test_five_legs_keep_four_feet_down_under_the_wave_schedule(self) -> None:
        env = self._env(fixed_command=[0.06, 0.0, 0.0], fixed_failed_legs=[5])
        env.reset(seed=0)
        _velocity, margins, contacts = self._roll(env, 300)
        env.close()
        self.assertEqual(float(contacts[4]), 0.0)
        self.assertGreater(float(np.mean(contacts.sum() * np.ones(1))), 3.0)
        self.assertGreater(float(np.mean(margins)), 0.0)

    def test_the_scaffold_survives_every_single_leg_loss(self) -> None:
        from src.rl.walk_failsafe import SconeFailsafeEnv

        for lost in LEG_INDICES:
            env = self._env(fixed_command=[0.06, 0.0, 0.0], fixed_failed_legs=[lost])
            env.reset(seed=0)
            action = np.zeros(18, dtype=np.float32)
            for _ in range(250):
                _obs, _reward, terminated, _truncated, info = env.step(action)
                self.assertFalse(
                    terminated,
                    f"leg {lost}: {info['fallen']=} {info['forbidden_collision']=}",
                )
            env.close()
        self.assertTrue(issubclass(SconeFailsafeEnv, object))

    def test_a_limp_leg_keeps_touching_the_ground(self) -> None:
        from src.rl.walk_failsafe import WalkConfig

        env = self._env(
            fixed_command=[0.06, 0.0, 0.0],
            fixed_failed_legs=[5],
            walk_config=WalkConfig(failure_mode="limp"),
        )
        env.reset(seed=0)
        _velocity, _margins, contacts = self._roll(env, 200)
        env.close()
        self.assertGreater(float(contacts[4]), 0.5)


class RewardBudgetTests(unittest.TestCase):
    def test_penalties_can_never_out_budget_the_reward(self) -> None:
        from src.rl.walk_failsafe import RewardConfig

        config = RewardConfig()
        self.assertLessEqual(config.penalty_budget, config.reward_budget + 1e-9)
        # walk_v3 shipped 57.5/s of reachable penalty against 4.0/s of
        # reachable reward and its optimal policy became to stop moving.
        with self.assertRaises(ValueError):
            RewardConfig(tracking_weight=0.1)

    def test_tracking_is_the_only_way_to_score(self) -> None:
        from src.rl.walk_failsafe import RewardConfig

        config = RewardConfig()
        self.assertEqual(config.reward_budget, config.tracking_weight)

    def test_ending_an_episode_is_never_cheaper_than_surviving_it(self) -> None:
        from src.rl.walk_failsafe import RewardConfig

        config = RewardConfig()
        self.assertGreaterEqual(config.termination_severity, 1.0)
        with self.assertRaises(ValueError):
            RewardConfig(termination_severity=0.5)

    def test_standing_still_under_a_moving_command_scores_about_nothing(self) -> None:
        from src.rl.walk_failsafe import RewardConfig

        config = RewardConfig()
        credit = math.exp(-(0.12**2) / config.linear_velocity_sigma**2)
        self.assertLess(credit * config.tracking_weight, 0.75)

    def test_a_zero_entropy_coefficient_is_refused(self) -> None:
        from src.rl.walk_failsafe import _ppo_kwargs, build_parser

        args = build_parser().parse_args(["train", "--entropy-coefficient", "0"])
        with self.assertRaises(ValueError):
            _ppo_kwargs(args)


class LauncherWiringTests(unittest.TestCase):
    def test_the_launcher_registers_the_trainer(self) -> None:
        from src.rl.inquiry import TRAINING_TASKS

        task = TRAINING_TASKS["walk-failsafe"]
        self.assertEqual(task.module, "src.rl.walk_failsafe")
        self.assertEqual(task.checkpoint_prefix, "scone_walk_failsafe")

    def test_replay_routes_85_observations_to_this_trainer(self) -> None:
        from src.rl.remote_watch import ENVIRONMENT_FOR_TASK, TASK_CHOICES
        from src.rl.walk_failsafe import SconeFailsafeEnv

        self.assertIn("walk-failsafe", TASK_CHOICES)
        self.assertIs(ENVIRONMENT_FOR_TASK["walk-failsafe"], SconeFailsafeEnv)

    def test_other_trainers_reference_names_do_not_abort_a_replay(self) -> None:
        # A run recorded by walk_v3 names "hardcoded", which this trainer does
        # not implement; the viewer has to translate rather than die on argv.
        from src.rl.remote_watch import reference_motion_for_environment

        self.assertEqual(
            reference_motion_for_environment("hardcoded", task="walk-failsafe"),
            "fault-adaptive",
        )
        self.assertEqual(
            reference_motion_for_environment("none", task="walk-failsafe"), "none"
        )

    def test_the_cli_accepts_the_launcher_argument_layout(self) -> None:
        from src.rl.walk_failsafe import build_parser

        args = build_parser().parse_args([
            "--terrain", "flat", "--terrain-seed", "7",
            "--reference-motion", "fault-adaptive",
            "--standing-pose-degrees", *(f"{value:g}" for value in STANDARD_STANDING_DEGREES),
            "train", "--curriculum", "easy", "--timesteps", "1000",
            "--num-envs", "2", "--checkpoint-every", "500",
            "--keep-checkpoints", "3", "--seed", "0", "--device", "cpu",
            "--output", "runs/x", "--tensorboard-log", "runs/x/tensorboard",
        ])
        self.assertEqual(args.curriculum, "easy")
        self.assertEqual(args.max_failed_legs, 2)
        self.assertEqual(len(args.standing_pose_degrees), 18)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
