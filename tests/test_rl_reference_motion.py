from __future__ import annotations

import unittest
from unittest.mock import patch

import numpy as np

from src.rl.stance import STANDARD_STANDING_DEGREES
from src.rl.walk_learn import (
    SconeWalkEnv,
    _make_vector_env,
    _resolve_reference_motion,
)


class ResidualReferenceMotionTests(unittest.TestCase):
    @staticmethod
    def _rollout(
        command: list[float],
        steps: int = 250,
        *,
        reference_motion: str = "hardcoded",
    ) -> tuple[np.ndarray, float]:
        env = SconeWalkEnv(
            fixed_command=command,
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
            reference_motion=reference_motion,
        )
        try:
            env.reset(seed=7)
            start = env.data.qpos[
                env.root_qpos_address : env.root_qpos_address + 3
            ].copy()
            world_from_body = env.data.xmat[env.root_body_id].reshape(3, 3).copy()
            start_yaw = env._heading_yaw()
            action = np.zeros(18, dtype=np.float32)
            for _ in range(steps):
                _, _, terminated, truncated, _ = env.step(action)
                if terminated or truncated:
                    raise AssertionError("reference gait terminated during sign check")
            world_delta = env.data.qpos[
                env.root_qpos_address : env.root_qpos_address + 3
            ].copy() - start
            delta = world_from_body.T @ world_delta
            yaw_delta = env._heading_yaw() - start_yaw
            return delta, yaw_delta
        finally:
            env.close()

    def test_multiple_training_envs_use_real_subprocess_workers(self) -> None:
        factories = [lambda: object(), lambda: object()]

        with (
            patch("src.rl.walk_learn.DummyVecEnv") as dummy,
            patch("src.rl.walk_learn.SubprocVecEnv", return_value="parallel") as subproc,
        ):
            selected = _make_vector_env(factories)

        self.assertEqual(selected, "parallel")
        subproc.assert_called_once_with(factories)
        dummy.assert_not_called()

    def test_reference_default_depends_on_training_or_replay(self) -> None:
        self.assertEqual(_resolve_reference_motion("train", None), "non_rl")
        self.assertEqual(_resolve_reference_motion("check", None), "non_rl")
        self.assertEqual(_resolve_reference_motion("enjoy", None), "hardcoded")
        self.assertEqual(
            _resolve_reference_motion("enjoy", "non_rl"),
            "non_rl",
        )

    def test_one_training_env_avoids_subprocess_overhead(self) -> None:
        factories = [lambda: object()]

        with (
            patch("src.rl.walk_learn.DummyVecEnv", return_value="single") as dummy,
            patch("src.rl.walk_learn.SubprocVecEnv") as subproc,
        ):
            selected = _make_vector_env(factories)

        self.assertEqual(selected, "single")
        dummy.assert_called_once_with(factories)
        subproc.assert_not_called()

    def test_forward_reference_moves_in_positive_body_x(self) -> None:
        forward, _ = self._rollout([0.25, 0.0, 0.0])
        reverse, _ = self._rollout([-0.25, 0.0, 0.0])

        self.assertGreater(float(forward[0]), 0.01)
        self.assertLess(float(reverse[0]), -0.01)

    def test_yaw_reference_uses_counter_clockwise_positive_sign(self) -> None:
        _, left_yaw = self._rollout([0.0, 0.0, 0.4])
        _, right_yaw = self._rollout([0.0, 0.0, -0.4])

        self.assertGreater(left_yaw, 0.1)
        self.assertLess(right_yaw, -0.1)

    def test_non_rl_reference_keeps_hardcoded_command_directions(self) -> None:
        forward, _ = self._rollout(
            [0.25, 0.0, 0.0],
            reference_motion="non_rl",
        )
        reverse, _ = self._rollout(
            [-0.25, 0.0, 0.0],
            reference_motion="non_rl",
        )
        _, left_yaw = self._rollout(
            [0.0, 0.0, 0.4],
            reference_motion="non_rl",
        )
        _, right_yaw = self._rollout(
            [0.0, 0.0, -0.4],
            reference_motion="non_rl",
        )

        self.assertGreater(float(forward[0]), 0.01)
        self.assertLess(float(reverse[0]), -0.01)
        self.assertGreater(left_yaw, 0.1)
        self.assertLess(right_yaw, -0.1)

    def test_lateral_command_has_no_horizontal_reference_stride(self) -> None:
        env = SconeWalkEnv(
            fixed_command=[0.0, 0.2, 0.0],
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
        )
        try:
            env.reset(seed=7)
            env._phase = 0.25
            reference = env._reference_motion_degrees()
            np.testing.assert_allclose(reference[:6], env.default_degrees[:6])
        finally:
            env.close()

    def test_non_rl_reference_supplies_lateral_ik_motion(self) -> None:
        env = SconeWalkEnv(
            fixed_command=[0.0, 0.2, 0.0],
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
            reference_motion="non_rl",
        )
        try:
            env.reset(seed=7)
            reference = env._reference_motion_degrees()

            self.assertEqual(reference.shape, (18,))
            self.assertTrue(np.isfinite(reference).all())
            self.assertGreater(
                float(np.max(np.abs(reference - env.default_degrees))),
                0.1,
            )
            self.assertAlmostEqual(env._phase, env._non_rl_reference.phase)
            self.assertEqual(env._reference_cycle_frequency, 0.7)
            self.assertGreater(env._reference_stride_clip_fraction, 0.0)
            self.assertGreater(env._reference_ik_backoff_scale, 0.0)
            self.assertLessEqual(env._reference_ik_backoff_scale, 1.0)
        finally:
            env.close()

    def test_rl_env_preserves_checkpoint_training_motion_profile(self) -> None:
        for reference_motion in ("hardcoded", "non_rl"):
            with self.subTest(reference_motion=reference_motion):
                env = SconeWalkEnv(
                    standing_pose_degrees=STANDARD_STANDING_DEGREES,
                    reference_motion=reference_motion,
                )
                try:
                    env.reset(seed=7)

                    self.assertTrue(
                        all(
                            np.isinf(
                                env.controller._profile_velocity[motor_id]
                            )
                            for motor_id in range(1, 19)
                        )
                    )
                    self.assertTrue(
                        all(
                            np.isinf(
                                env.controller._profile_acceleration[motor_id]
                            )
                            for motor_id in range(1, 19)
                        )
                    )
                finally:
                    env.close()


if __name__ == "__main__":
    unittest.main()
