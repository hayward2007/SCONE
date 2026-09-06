from __future__ import annotations

import math
import unittest
from types import SimpleNamespace

import mujoco
import numpy as np

from src.rl.inquiry import TRAINING_TASKS, build_training_arguments, TrainingConfig
from src.rl.policy_compat import (
    V3_OBSERVATION_SHAPE,
    is_v3_checkpoint,
    task_for_observation_shape,
)
from src.rl.stance import STANDARD_STANDING_DEGREES
from src.rl.walk_v3 import (
    CURRICULUM_RANGES,
    OBSERVATION_COMMAND_SCALE,
    OBSERVATION_DIM,
    RewardConfig,
    SconeWalkEnvV3,
    WalkConfig,
    _fixed_evaluation_score,
    _ppo_training_kwargs,
    _require_bounded_resume_policy,
    _validate_training_batch_size,
    build_parser,
    training_walk_config,
)


def _fast_config(**overrides) -> WalkConfig:
    """A config that skips the settle transient so tests stay quick."""

    return WalkConfig(settle_seconds=0.0, **overrides)


def _drive_body(env: SconeWalkEnvV3, body_velocity) -> None:
    """Give the base a velocity expressed in the frame the reward measures.

    The free joint's linear qvel is world-frame while the command and the
    reward live in the chassis frame, and at the reset pose the two are yawed
    90 degrees apart, so the rotation here is not optional.
    """

    address = int(env.model.jnt_dofadr[env.root_joint_id])
    world_from_body = env.data.xmat[env.root_body_id].reshape(3, 3)
    env.data.qvel[address: address + 3] = world_from_body @ np.asarray(
        body_velocity, dtype=np.float64
    )
    mujoco.mj_forward(env.model, env.data)


class WalkV3ObservationTests(unittest.TestCase):
    def test_observation_adds_contact_flags_to_the_legacy_layout(self) -> None:
        # The default settle is what training and replay use; the robot spawns
        # a few millimetres above its settled stance.
        env = SconeWalkEnvV3(fixed_command=[0.0, 0.0, 0.0])
        try:
            observation, _ = env.reset(seed=7)
            self.assertEqual(observation.shape, (OBSERVATION_DIM,))
            self.assertEqual(observation.shape, V3_OBSERVATION_SHAPE)
            contacts = observation[70:76]
            self.assertTrue(np.all(np.isin(contacts, (0.0, 1.0))))
            # A settled hexapod stands on all six feet.
            self.assertEqual(float(contacts.sum()), 6.0)
        finally:
            env.close()

    def test_reset_observation_is_defined_without_a_settle(self) -> None:
        env = SconeWalkEnvV3(
            fixed_command=[0.0, 0.0, 0.0], walk_config=_fast_config()
        )
        try:
            observation, info = env.reset(seed=7)
            self.assertTrue(np.all(np.isfinite(observation)))
            self.assertEqual(observation.shape, (OBSERVATION_DIM,))
            self.assertGreater(info["reference_height"], 0.0)
        finally:
            env.close()

    def test_checkpoint_width_identifies_the_trainer(self) -> None:
        self.assertTrue(is_v3_checkpoint((76,)))
        self.assertEqual(task_for_observation_shape((76,)), "walk-v3")
        self.assertEqual(task_for_observation_shape((82,)), "walk-v2")
        self.assertEqual(task_for_observation_shape((70,)), "walk")


class WalkV3NoSpeedLimitTests(unittest.TestCase):
    def test_velocity_command_is_never_clipped(self) -> None:
        env = SconeWalkEnvV3(walk_config=_fast_config())
        try:
            # Three times the observation normalization range: v1 and v2 both
            # clamped this away, which is the speed limit v3 removes.
            accepted = env.set_velocity_command([1.20, 0.60, 2.70])
            np.testing.assert_allclose(accepted, [1.20, 0.60, 2.70])
            env.reset(seed=7)
            observation, _, _, _, _ = env.step(np.zeros(18, dtype=np.float32))
            np.testing.assert_allclose(
                observation[63:66],
                np.asarray([1.20, 0.60, 2.70]) / OBSERVATION_COMMAND_SCALE,
                rtol=1e-4,
            )
            self.assertGreater(float(np.max(observation[63:66])), 1.0)
        finally:
            env.close()

    def test_exceeding_the_command_never_costs_reward(self) -> None:
        env = SconeWalkEnvV3(
            fixed_command=[0.20, 0.0, 0.0], walk_config=_fast_config()
        )
        try:
            env.reset(seed=7)

            def tracking(velocity: float) -> tuple[float, float]:
                _drive_body(env, [velocity, 0.0, 0.0])
                contact, slip = env._foot_contacts()
                _, terms, _, _ = env._reward(
                    np.zeros(18, dtype=np.float64), contact, slip
                )
                # Slip and drift are separate physical costs; these two are
                # the command-tracking pair.
                return terms["speed"], terms["speed"] + terms["drift"]

            on_command, on_tracking = tracking(0.20)
            overspeed, over_tracking = tracking(0.40)
            slower, _ = tracking(0.10)

            self.assertAlmostEqual(on_command, overspeed, places=9)
            self.assertGreaterEqual(over_tracking, on_tracking - 1e-9)
            self.assertLess(slower, on_command)
        finally:
            env.close()

    def test_reference_stride_tracks_the_command_then_saturates(self) -> None:
        env = SconeWalkEnvV3(walk_config=_fast_config())
        try:
            strides = [env.stride_for_speed(speed) for speed in
                       (0.0, 0.05, 0.10, 0.20, 0.30)]
            self.assertEqual(strides, sorted(strides))
            self.assertEqual(strides[0], 0.0)
            # Above the calibrated envelope the scaffold runs out at the joint
            # bound instead of the command being refused.
            bound = env.walk_config.stride_degrees_max
            self.assertEqual(env.stride_for_speed(0.60), bound)
            self.assertEqual(env.stride_for_speed(5.00), bound)
            yaw_strides = [env.stride_for_yaw_rate(rate) for rate in
                           (0.0, 0.30, 0.60, 0.90)]
            self.assertEqual(yaw_strides, sorted(yaw_strides))
        finally:
            env.close()

    def test_curriculum_never_asks_far_beyond_the_scaffold(self) -> None:
        env = SconeWalkEnvV3(walk_config=_fast_config())
        try:
            top_speed = max(
                speed for speed, _ in env.walk_config.stride_calibration
            )
            # v1 asked for 0.50 m/s against a 0.10 m/s reference, which left no
            # gradient at speed. Every v3 stage stays within a small stretch.
            for name, limits in CURRICULUM_RANGES.items():
                with self.subTest(curriculum=name):
                    self.assertLessEqual(float(limits[0]), top_speed * 1.25)
        finally:
            env.close()


class WalkV3FreeHeightLockedAttitudeTests(unittest.TestCase):
    def test_no_height_reward_and_no_height_termination(self) -> None:
        self.assertFalse(hasattr(WalkConfig(), "max_height_drop"))
        self.assertFalse(
            any("height" in field for field in vars(RewardConfig()))
        )
        env = SconeWalkEnvV3(
            fixed_command=[0.20, 0.0, 0.0], walk_config=_fast_config()
        )
        try:
            env.reset(seed=7)
            self.assertNotIn("height", env._reward(
                np.zeros(18), *env._foot_contacts()
            )[1])

            height_address = env.root_qpos_address + 2
            settled = float(env.data.qpos[height_address])
            # Drop the chassis far below the settled stance: v1 and v2 both end
            # the episode here; v3 must not, because height is free.
            env.data.qpos[height_address] = settled - 0.10
            mujoco.mj_forward(env.model, env.data)
            contact, slip = env._foot_contacts()
            _, terms, terminated, diagnostics = env._reward(
                np.zeros(18), contact, slip
            )
            self.assertLess(diagnostics["height_change"], -0.05)
            self.assertFalse(diagnostics["forbidden_collision"])
            self.assertFalse(terminated)
            self.assertNotIn("termination", terms)
        finally:
            env.close()

    def test_vertical_motion_is_not_penalized_but_roll_pitch_rate_is(self) -> None:
        env = SconeWalkEnvV3(
            fixed_command=[0.20, 0.0, 0.0], walk_config=_fast_config()
        )
        try:
            env.reset(seed=7)
            address = int(env.model.jnt_dofadr[env.root_joint_id])

            def attitude_term(linear, angular) -> float:
                _drive_body(env, linear)
                env.data.qvel[address + 3: address + 6] = angular
                mujoco.mj_forward(env.model, env.data)
                contact, slip = env._foot_contacts()
                return env._reward(np.zeros(18), contact, slip)[1]["attitude_rate"]

            still = attitude_term([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            bobbing = attitude_term([0.0, 0.0, 0.5], [0.0, 0.0, 0.0])
            rolling = attitude_term([0.0, 0.0, 0.0], [0.5, 0.0, 0.0])

            self.assertAlmostEqual(still, bobbing, places=9)
            self.assertLess(rolling, still)
        finally:
            env.close()

    def test_tilting_the_body_ends_the_episode(self) -> None:
        env = SconeWalkEnvV3(
            fixed_command=[0.20, 0.0, 0.0], walk_config=_fast_config()
        )
        try:
            env.reset(seed=7)

            def roll(degrees: float) -> tuple[bool, float, float]:
                angle = math.radians(degrees)
                env.data.qpos[
                    env.root_qpos_address + 3: env.root_qpos_address + 7
                ] = [math.cos(angle / 2.0), math.sin(angle / 2.0), 0.0, 0.0]
                mujoco.mj_forward(env.model, env.data)
                contact, slip = env._foot_contacts()
                _, terms, terminated, diagnostics = env._reward(
                    np.zeros(18), contact, slip
                )
                return terminated, terms["upright"], diagnostics["tilt_degrees"]

            limit = env.walk_config.max_tilt_degrees
            upright_terminated, level_cost, level_tilt = roll(0.0)
            small_terminated, small_cost, small_tilt = roll(limit / 2.0)
            large_terminated, _, large_tilt = roll(limit + 10.0)

            self.assertLess(level_tilt, 1e-6)
            self.assertFalse(upright_terminated)
            self.assertFalse(small_terminated)
            self.assertLess(small_cost, level_cost)
            self.assertGreater(large_tilt, limit)
            self.assertTrue(large_terminated)
        finally:
            env.close()

    def test_attitude_limit_is_far_tighter_than_a_fall_detector(self) -> None:
        # The measured zero-residual scaffold peaks at 4.4 degrees of tilt at
        # its top speed, so this is a real constraint with margin, where v1 and
        # v2 only detected a 60 degree collapse.
        self.assertLessEqual(WalkConfig().max_tilt_degrees, 20.0)


class WalkV3RewardShapeTests(unittest.TestCase):
    def test_an_ideal_stand_under_an_idle_command_scores_zero(self) -> None:
        env = SconeWalkEnvV3(
            fixed_command=[0.0, 0.0, 0.0], walk_config=_fast_config()
        )
        try:
            env.reset(seed=7)
            total = 0.0
            for _ in range(25):
                _, reward, terminated, _, _ = env.step(np.zeros(18, dtype=np.float32))
                total += reward
                self.assertFalse(terminated)
            # No positive survival constant to harvest, and nothing to pay for.
            self.assertLess(abs(total), 0.05)
        finally:
            env.close()

    def test_idle_drift_and_idle_residual_are_costs(self) -> None:
        env = SconeWalkEnvV3(
            fixed_command=[0.0, 0.0, 0.0], walk_config=_fast_config()
        )
        try:
            env.reset(seed=7)
            _drive_body(env, [0.2, 0.0, 0.0])
            contact, slip = env._foot_contacts()
            _, drifting, _, _ = env._reward(np.zeros(18), contact, slip)
            _, pushing, _, _ = env._reward(np.ones(18), contact, slip)

            self.assertLess(drifting["idle_velocity"], 0.0)
            self.assertEqual(drifting["idle_action"], 0.0)
            self.assertLess(pushing["idle_action"], 0.0)
            self.assertEqual(drifting["speed"], 0.0)
        finally:
            env.close()

    def test_heading_is_a_hold_objective_only(self) -> None:
        env = SconeWalkEnvV3(walk_config=_fast_config())
        try:
            env.reset(seed=7)
            # Turning: the heading target follows the body, so a small rate
            # deficit cannot integrate into an unbounded heading cost.
            env.set_velocity_command([0.0, 0.0, 0.90])
            turning = [
                env.step(np.zeros(18, dtype=np.float32))[4]["reward_terms"]["heading"]
                for _ in range(40)
            ]
            self.assertTrue(all(abs(value) < 1e-9 for value in turning))

            env.set_velocity_command([0.20, 0.0, 0.0])
            for _ in range(40):
                info = env.step(np.zeros(18, dtype=np.float32))[4]
            self.assertLessEqual(info["reward_terms"]["heading"], 0.0)
        finally:
            env.close()

    def test_wrong_way_motion_scores_below_standing_still(self) -> None:
        env = SconeWalkEnvV3(
            fixed_command=[0.20, 0.0, 0.0], walk_config=_fast_config()
        )
        try:
            env.reset(seed=7)

            def speed_term(velocity: float) -> float:
                _drive_body(env, [velocity, 0.0, 0.0])
                contact, slip = env._foot_contacts()
                return env._reward(np.zeros(18), contact, slip)[1]["speed"]

            self.assertEqual(speed_term(0.0), 0.0)
            self.assertLess(speed_term(-0.20), 0.0)
        finally:
            env.close()


class WalkV3TrainingTests(unittest.TestCase):
    def test_default_remote_rollout_has_complete_minibatches(self) -> None:
        parser = build_parser()
        args = parser.parse_args(["train"])
        _validate_training_batch_size(args.n_steps, args.num_envs, args.batch_size)
        for num_envs in (1, 3, 8, 9, 16):
            with self.subTest(num_envs=num_envs):
                _validate_training_batch_size(
                    args.n_steps, num_envs, args.batch_size
                )
        with self.assertRaises(ValueError):
            _validate_training_batch_size(512, 4, 3000)

    def test_new_policy_uses_squashed_gsde_and_a_kl_guard(self) -> None:
        args = build_parser().parse_args(["train"])
        kwargs = _ppo_training_kwargs(args)

        self.assertTrue(kwargs["use_sde"])
        self.assertTrue(kwargs["policy_kwargs"]["squash_output"])
        self.assertGreater(kwargs["target_kl"], 0.0)
        self.assertEqual(kwargs["gamma"], 0.995)

    def test_unbounded_checkpoint_cannot_resume_a_v3_run(self) -> None:
        clipped = SimpleNamespace(
            use_sde=False, policy=SimpleNamespace(squash_output=False)
        )
        bounded = SimpleNamespace(
            use_sde=True, policy=SimpleNamespace(squash_output=True)
        )
        with self.assertRaises(ValueError):
            _require_bounded_resume_policy(clipped)
        _require_bounded_resume_policy(bounded)

    def test_replay_defaults_are_nominal_and_training_is_staged(self) -> None:
        replay = WalkConfig()
        easy = training_walk_config("easy")
        full = training_walk_config("full")

        self.assertEqual(replay.observation_noise, 0.0)
        self.assertEqual(replay.mass_scale_range, (1.0, 1.0))
        self.assertEqual(replay.initial_joint_noise_degrees, 0.0)
        self.assertGreater(easy.observation_noise, 0.0)
        self.assertLess(
            easy.initial_joint_noise_degrees, full.initial_joint_noise_degrees
        )
        with self.assertRaises(ValueError):
            training_walk_config("expert")

    def test_evaluation_scores_delivery_against_the_command(self) -> None:
        def record(name, command, vx, yaw_rate=0.0, survived=True, tilt=3.0):
            return {
                "name": name, "command": list(command), "vx": vx, "vy": 0.0,
                "yaw_rate": yaw_rate, "heading_error": 0.0,
                "tilt_degrees": tilt, "survived": survived,
            }

        delivered = _fixed_evaluation_score([
            record("idle", (0.0, 0.0, 0.0), 0.0),
            record("forward", (0.30, 0.0, 0.0), 0.30),
        ])
        stalled = _fixed_evaluation_score([
            record("idle", (0.0, 0.0, 0.0), 0.0),
            record("forward", (0.30, 0.0, 0.0), 0.0),
        ])
        overspeed = _fixed_evaluation_score([
            record("idle", (0.0, 0.0, 0.0), 0.0),
            record("forward", (0.30, 0.0, 0.0), 0.45),
        ])

        self.assertGreater(delivered["score"], stalled["score"])
        self.assertEqual(stalled["direction_failures"], 1)
        # Delivery is capped at the command, so overspeed neither wins nor
        # loses the comparison.
        self.assertAlmostEqual(
            overspeed["command_delivery"], delivered["command_delivery"], places=9
        )
        self.assertAlmostEqual(delivered["top_speed"], 0.30, places=9)

    def test_lateral_gap_is_measured_without_being_unpromotable(self) -> None:
        def record(name, command, vx, vy=0.0, yaw_rate=0.0):
            return {
                "name": name, "command": list(command), "vx": vx, "vy": vy,
                "yaw_rate": yaw_rate, "heading_error": 0.0,
                "tilt_degrees": 3.0, "survived": True,
            }

        # The hardcoded reference has no lateral scaffold, so a pure vy command
        # is a direction failure for the zero-residual baseline itself.
        scaffold = _fixed_evaluation_score([
            record("forward", (0.30, 0.0, 0.0), 0.28),
            record("lateral", (0.0, 0.08, 0.0), 0.0),
        ])
        self.assertEqual(scaffold["direction_failures"], 1)

        # A policy that is faster forward and still cannot go sideways must
        # remain promotable against that baseline.
        policy = _fixed_evaluation_score([
            record("forward", (0.30, 0.0, 0.0), 0.30),
            record("lateral", (0.0, 0.08, 0.0), 0.0),
        ])
        self.assertGreater(policy["score"], scaffold["score"])
        self.assertLessEqual(
            policy["direction_failures"], scaffold["direction_failures"]
        )

    def test_cli_accepts_the_shared_launcher_argument_layout(self) -> None:
        config = TrainingConfig(
            task="walk-v3",
            run_name="walk_v3_test",
            curriculum="easy",
            timesteps=1000,
            num_envs=2,
            checkpoint_every=500,
            keep_checkpoints=3,
            reference_motion="hardcoded",
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
        )
        arguments = build_training_arguments(config)
        args = build_parser().parse_args(arguments)

        self.assertEqual(args.command_name, "train")
        self.assertEqual(args.reference_motion, "hardcoded")
        self.assertEqual(args.curriculum, "easy")
        self.assertEqual(args.num_envs, 2)
        self.assertEqual(
            tuple(args.standing_pose_degrees), STANDARD_STANDING_DEGREES
        )
        self.assertEqual(TRAINING_TASKS["walk-v3"].module, "src.rl.walk_v3")
        self.assertEqual(
            TRAINING_TASKS["walk-v3"].checkpoint_prefix, "scone_walk_v3"
        )

    def test_task_rejects_a_reference_it_does_not_scaffold(self) -> None:
        with self.assertRaises(ValueError):
            TrainingConfig(
                task="walk-v3",
                run_name="walk_v3_gait",
                curriculum="easy",
                timesteps=1000,
                num_envs=1,
                checkpoint_every=500,
                keep_checkpoints=1,
                reference_motion="tripod-gait",
            )
        with self.assertRaises(ValueError):
            SconeWalkEnvV3(reference_motion="tripod-gait")


if __name__ == "__main__":
    unittest.main()
