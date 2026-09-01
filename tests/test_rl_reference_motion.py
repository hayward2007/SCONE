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
        self.assertEqual(_resolve_reference_motion("train", None), "tripod-gait")
        self.assertEqual(_resolve_reference_motion("check", None), "tripod-gait")
        self.assertEqual(_resolve_reference_motion("enjoy", None), "hardcoded")
        self.assertEqual(
            _resolve_reference_motion("enjoy", "non_rl"),
            "tripod-gait",
        )
        self.assertEqual(
            _resolve_reference_motion("train", "scone-gait"),
            "scone-gait",
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

    def test_tripod_gait_reference_keeps_hardcoded_command_directions(self) -> None:
        forward, _ = self._rollout(
            [0.25, 0.0, 0.0],
            reference_motion="tripod-gait",
        )
        reverse, _ = self._rollout(
            [-0.25, 0.0, 0.0],
            reference_motion="tripod-gait",
        )
        _, left_yaw = self._rollout(
            [0.0, 0.0, 0.4],
            reference_motion="tripod-gait",
        )
        _, right_yaw = self._rollout(
            [0.0, 0.0, -0.4],
            reference_motion="tripod-gait",
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

    def test_tripod_gait_reference_supplies_lateral_ik_motion(self) -> None:
        env = SconeWalkEnv(
            fixed_command=[0.0, 0.2, 0.0],
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
            reference_motion="tripod-gait",
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
            self.assertAlmostEqual(env._phase, env._reference_gait.phase)
            self.assertEqual(env._reference_cycle_frequency, 0.7)
            self.assertGreater(env._reference_stride_clip_fraction, 0.0)
            self.assertGreater(env._reference_ik_backoff_scale, 0.0)
            self.assertLessEqual(env._reference_ik_backoff_scale, 1.0)
        finally:
            env.close()

    def test_scone_gait_reference_uses_sector_roll_controller(self) -> None:
        tripod = SconeWalkEnv(
            fixed_command=[0.2, 0.0, 0.0],
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
            reference_motion="tripod-gait",
        )
        scone = SconeWalkEnv(
            fixed_command=[0.2, 0.0, 0.0],
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
            reference_motion="scone-gait",
        )
        try:
            tripod.reset(seed=7)
            scone.reset(seed=7)
            tripod_reference = tripod._reference_motion_degrees()
            scone_reference = scone._reference_motion_degrees()

            self.assertEqual(type(tripod._reference_gait).__name__, "TripodGait")
            self.assertEqual(type(scone._reference_gait).__name__, "SconeGait")
            self.assertGreater(
                float(
                    np.max(
                        np.abs(
                            scone_reference[12:] - tripod_reference[12:]
                        )
                    )
                ),
                0.1,
            )
        finally:
            tripod.close()
            scone.close()

    def test_rl_env_preserves_checkpoint_training_motion_profile(self) -> None:
        for reference_motion in ("hardcoded", "tripod-gait", "scone-gait"):
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

    def test_replay_override_preserves_multi_turn_lower_targets(self) -> None:
        env = SconeWalkEnv(
            fixed_command=[0.0, 0.0, 0.0],
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
            reference_motion="hardcoded",
        )
        try:
            env.reset(seed=7)
            override = env.default_degrees.copy()
            override[12:] += 720.0

            env.set_reference_override(
                override,
                blend=1.0,
                unwrapped_lower=True,
            )
            env._apply_action(np.zeros(18, dtype=np.float32))
            full_override = (
                np.degrees(env.controller._target[13:19]) + 180.0
            )

            env.set_reference_override(
                override,
                blend=0.0,
                unwrapped_lower=True,
            )
            env._apply_action(np.zeros(18, dtype=np.float32))
            ppo_same_branch = (
                np.degrees(env.controller._target[13:19]) + 180.0
            )

            np.testing.assert_allclose(full_override, override[12:], atol=0.1)
            np.testing.assert_allclose(ppo_same_branch, override[12:], atol=0.1)

            before, _ = env._joint_state()
            for motor_id in range(13, 19):
                env.data.qpos[env.controller._qpos_addresses[motor_id]] += (
                    4.0 * np.pi
                )
            after, _ = env._joint_state()
            np.testing.assert_allclose(after[12:], before[12:], atol=1e-9)
            _, _, _, diagnostics = env._reward(
                np.zeros(18, dtype=np.float32)
            )
            self.assertFalse(diagnostics["hard_joint_limit"])
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
