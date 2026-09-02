from __future__ import annotations

import unittest
from types import SimpleNamespace

import numpy as np

from src.rl.walk_v2 import (
    SconeWalkEnvV2,
    WalkConfig,
    _ppo_training_kwargs,
    _require_bounded_resume_policy,
    _validate_evaluation_arguments,
    _validate_training_batch_size,
    _fixed_evaluation_score,
)


class WalkV2RandomizationTests(unittest.TestCase):
    def test_domain_randomization_restarts_from_nominal_every_reset(self) -> None:
        strength = 1.1
        mass = 1.05
        friction = 0.8
        env = SconeWalkEnvV2(
            fixed_command=[0.0, 0.0, 0.0],
            walk_config=WalkConfig(
                settle_seconds=0.0,
                mirror_probability=0.0,
                initial_joint_noise_degrees=0.0,
                initial_yaw_randomization=False,
                push_velocity=0.0,
                observation_noise=0.0,
                action_delay_probability=0.0,
                mass_scale_range=(mass, mass),
                friction_scale_range=(friction, friction),
                strength_scale_range=(strength, strength),
            ),
        )
        try:
            nominal_mass = env._nominal_body_mass.copy()
            nominal_inertia = env._nominal_body_inertia.copy()
            nominal_friction = env._nominal_geom_friction.copy()
            nominal_gain = env._nominal_actuator_gainprm.copy()
            nominal_force = env._nominal_actuator_forcerange.copy()

            env.reset(seed=7)
            first_gain = env.model.actuator_gainprm.copy()
            first_force = env.model.actuator_forcerange.copy()
            env.reset(seed=8)

            np.testing.assert_allclose(env.model.body_mass, nominal_mass * mass)
            np.testing.assert_allclose(env.model.body_inertia, nominal_inertia * mass)
            np.testing.assert_allclose(
                env.model.geom_friction[:, 0], nominal_friction[:, 0] * friction
            )
            np.testing.assert_allclose(
                env.model.geom_friction[:, 1:], nominal_friction[:, 1:]
            )
            np.testing.assert_allclose(
                env.model.actuator_gainprm[:, 0], nominal_gain[:, 0] / strength
            )
            np.testing.assert_allclose(
                env.model.actuator_gainprm[:, 1:], nominal_gain[:, 1:]
            )
            np.testing.assert_allclose(
                env.model.actuator_forcerange, nominal_force * strength
            )
            np.testing.assert_allclose(env.model.actuator_gainprm, first_gain)
            np.testing.assert_allclose(env.model.actuator_forcerange, first_force)
        finally:
            env.close()


class WalkV2PPOConfigurationTests(unittest.TestCase):
    @staticmethod
    def _args(**changes):
        values = {
            "n_steps": 512,
            "num_envs": 9,
            "batch_size": 512,
            "learning_rate": 1e-4,
            "n_epochs": 3,
            "target_kl": 0.02,
            "entropy_coefficient": 0.0,
            "max_grad_norm": 0.5,
            "sde_sample_freq": 4,
            "log_std_init": -1.5,
        }
        values.update(changes)
        return SimpleNamespace(**values)

    def test_default_remote_rollout_has_complete_minibatches(self) -> None:
        _validate_training_batch_size(512, 9, 512)
        with self.assertRaisesRegex(ValueError, "must divide"):
            _validate_training_batch_size(512, 9, 1024)
        with self.assertRaisesRegex(ValueError, "must all be positive"):
            _validate_training_batch_size(0, 9, 512)

    def test_new_policy_uses_squashed_gsde_and_kl_guard(self) -> None:
        kwargs = _ppo_training_kwargs(self._args())

        self.assertTrue(kwargs["use_sde"])
        self.assertEqual(kwargs["sde_sample_freq"], 4)
        self.assertEqual(kwargs["target_kl"], 0.02)
        self.assertEqual(kwargs["learning_rate"], 1e-4)
        self.assertEqual(kwargs["n_epochs"], 3)
        self.assertEqual(kwargs["ent_coef"], 0.0)
        self.assertEqual(kwargs["max_grad_norm"], 0.5)
        self.assertTrue(kwargs["policy_kwargs"]["squash_output"])
        self.assertTrue(kwargs["policy_kwargs"]["use_expln"])
        self.assertEqual(kwargs["policy_kwargs"]["log_std_init"], -1.5)

    def test_old_clipped_gaussian_checkpoint_cannot_resume(self) -> None:
        old_model = SimpleNamespace(
            use_sde=False,
            policy=SimpleNamespace(squash_output=False),
        )
        with self.assertRaisesRegex(ValueError, "start a new walk_v2 run"):
            _require_bounded_resume_policy(old_model)

        new_model = SimpleNamespace(
            use_sde=True,
            policy=SimpleNamespace(squash_output=True),
        )
        _require_bounded_resume_policy(new_model)

    def test_fixed_evaluation_arguments_must_be_positive(self) -> None:
        _validate_evaluation_arguments(SimpleNamespace(
            eval_every=None,
            eval_episodes=3,
            eval_seconds=10.0,
        ))
        with self.assertRaisesRegex(ValueError, "eval_episodes"):
            _validate_evaluation_arguments(SimpleNamespace(
                eval_every=100,
                eval_episodes=0,
                eval_seconds=10.0,
            ))

    def test_fixed_evaluation_separates_tracking_from_dense_reward(self) -> None:
        good = [{
            "command": [0.25, 0.0, 0.0],
            "vx": 0.24,
            "vy": 0.0,
            "yaw_rate": 0.0,
            "heading_error": 0.01,
            "saturation_fraction": 0.02,
            "survived": True,
        }]
        stalled = [{
            **good[0],
            "vx": 0.0,
            "saturation_fraction": 0.95,
        }]

        good_summary = _fixed_evaluation_score(good)
        stalled_summary = _fixed_evaluation_score(stalled)

        self.assertGreater(good_summary["score"], stalled_summary["score"])
        self.assertEqual(good_summary["direction_failures"], 0)
        self.assertEqual(stalled_summary["direction_failures"], 1)


if __name__ == "__main__":
    unittest.main()
