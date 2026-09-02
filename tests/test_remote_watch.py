from __future__ import annotations

import tempfile
import unittest
import zipfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import mujoco

from src.rl.remote_watch import (
    LocalCheckpointSource,
    _observation_for_policy,
    mirror_checkpoint,
)
from src.rl.policy_compat import load_compatible_policy
from src.rl.stance import SPORT_STANDING_DEGREES, STANDARD_STANDING_DEGREES
from src.rl.walk_learn import (
    GracefulStopCallback,
    RewardConfig,
    SconeWalkEnv,
    WalkConfig,
    _write_resume_pointer,
)
from src.rl.walk_v2 import (
    SconeWalkEnvV2,
    WalkConfig as WalkConfigV2,
)


class RemoteWatchCompatibilityTests(unittest.TestCase):
    def test_walk_v2_live_command_uses_v2_observation_limits(self) -> None:
        env = SconeWalkEnvV2(fixed_command=[0.0, 0.0, 0.0])
        try:
            accepted = env.set_velocity_command([2.0, -2.0, 2.0])
        finally:
            env.close()

        np.testing.assert_allclose(accepted, [0.7, -0.25, 0.9])

    def test_walk_v2_human_step_opens_the_render_path(self) -> None:
        env = SconeWalkEnvV2(
            fixed_command=[0.0, 0.0, 0.0],
            render_mode="human",
            walk_config=WalkConfigV2(
                settle_seconds=0.0,
                frame_skip=1,
                mirror_probability=0.0,
                initial_joint_noise_degrees=0.0,
                initial_yaw_randomization=False,
                mass_scale_range=(1.0, 1.0),
                friction_scale_range=(1.0, 1.0),
                strength_scale_range=(1.0, 1.0),
                observation_noise=0.0,
                action_delay_probability=0.0,
            ),
        )
        try:
            env.reset(seed=7)
            with patch.object(env, "render") as render:
                env.step(np.zeros(18, dtype=np.float32))
            render.assert_called_once_with()
        finally:
            env.close()

    def test_standard_stance_starts_higher_than_sport_without_collision(self) -> None:
        measurements = {}
        for name, pose in (
            ("sport", SPORT_STANDING_DEGREES),
            ("standard", STANDARD_STANDING_DEGREES),
        ):
            env = SconeWalkEnv(
                fixed_command=[0.0, 0.0, 0.0],
                standing_pose_degrees=pose,
            )
            try:
                env.reset(seed=7)
                _, _, terminated, diagnostics = env._reward(
                    np.zeros(18, dtype=np.float32)
                )
                measurements[name] = (
                    env._reference_height,
                    diagnostics["stance_contacts"],
                    diagnostics["forbidden_collision"],
                    terminated,
                )
            finally:
                env.close()

        self.assertGreater(
            measurements["standard"][0],
            measurements["sport"][0] + 0.15,
        )
        self.assertEqual(measurements["standard"][1], 6)
        self.assertFalse(measurements["standard"][2])
        self.assertFalse(measurements["standard"][3])

    def test_height_reward_only_penalizes_body_drop(self) -> None:
        env = SconeWalkEnv(
            fixed_command=[0.0, 0.0, 0.0],
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
        )
        try:
            env.reset(seed=7)
            root_height_address = env.root_qpos_address + 2
            env.data.qpos[root_height_address] = env._reference_height + 0.03
            mujoco.mj_forward(env.model, env.data)
            _, raised_terms, _, _ = env._reward(np.zeros(18, dtype=np.float32))

            env.data.qpos[root_height_address] = env._reference_height - 0.03
            mujoco.mj_forward(env.model, env.data)
            _, dropped_terms, _, _ = env._reward(np.zeros(18, dtype=np.float32))
        finally:
            env.close()

        self.assertEqual(raised_terms["height"], 0.0)
        self.assertLess(dropped_terms["height"], 0.0)
        self.assertGreater(RewardConfig().upright_weight, RewardConfig().height_weight)

    def test_reward_aggregates_are_not_counted_twice(self) -> None:
        env = SconeWalkEnv(
            fixed_command=[0.0, 0.0, 0.0],
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
        )
        try:
            env.reset(seed=7)
            total, terms, terminated, _ = env._reward(
                np.zeros(18, dtype=np.float32)
            )
        finally:
            env.close()

        expected = sum(
            terms[name] for name in ("velocity", "direction", "stability", "damping")
        )
        if terminated:
            expected += terms["termination"]
        self.assertAlmostEqual(total, expected)

    def test_idle_reward_penalizes_residual_action(self) -> None:
        env = SconeWalkEnv(
            fixed_command=[0.0, 0.0, 0.0],
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
        )
        try:
            env.reset(seed=7)
            _, zero_terms, _, _ = env._reward(
                np.zeros(18, dtype=np.float32)
            )
            _, biased_terms, _, _ = env._reward(
                np.ones(18, dtype=np.float32)
            )
        finally:
            env.close()

        self.assertGreater(zero_terms["idle_velocity"], 0.0)
        self.assertEqual(zero_terms["idle_action"], 0.0)
        self.assertLess(biased_terms["idle_action"], 0.0)

    def test_training_can_sample_only_idle_command_segments(self) -> None:
        env = SconeWalkEnv(
            walk_config=WalkConfig(idle_command_probability=1.0),
            standing_pose_degrees=STANDARD_STANDING_DEGREES,
        )
        try:
            _, info = env.reset(seed=7)
        finally:
            env.close()

        np.testing.assert_array_equal(info["command_target"], np.zeros(3))

    def test_graceful_stop_callback_stops_only_after_request(self) -> None:
        requested = False
        callback = GracefulStopCallback(lambda: requested)

        self.assertTrue(callback._on_step())
        requested = True
        self.assertFalse(callback._on_step())

    def test_resume_pointer_is_relative_and_atomically_published(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_dir = Path(temporary_directory)
            checkpoint = run_dir / "checkpoints" / "scone_walk_100_steps.zip"
            checkpoint.parent.mkdir()
            checkpoint.touch()

            _write_resume_pointer(run_dir, checkpoint)

            self.assertEqual(
                (run_dir / "resume.checkpoint").read_text(encoding="utf-8"),
                "checkpoints/scone_walk_100_steps.zip\n",
            )
            self.assertFalse((run_dir / "resume.tmp").exists())

    def test_legacy_policy_load_does_not_bind_current_environment(self) -> None:
        policy = SimpleNamespace(
            observation_space=SimpleNamespace(shape=(68,)),
            action_space=SimpleNamespace(shape=(18,)),
        )
        env = SimpleNamespace(
            observation_space=SimpleNamespace(shape=(70,)),
            action_space=SimpleNamespace(shape=(18,)),
        )

        with patch("src.rl.policy_compat.PPO.load", return_value=policy) as load:
            loaded = load_compatible_policy(Path("legacy.zip"), env, "cpu")

        self.assertIs(loaded, policy)
        load.assert_called_once_with(Path("legacy.zip"), device="cpu")

    def test_live_velocity_command_clips_to_policy_observation_range(self) -> None:
        env = SconeWalkEnv(fixed_command=[0.0, 0.0, 0.0])
        try:
            accepted = env.set_velocity_command([2.0, -2.0, 0.4])
        finally:
            env.close()

        np.testing.assert_allclose(accepted, [0.5, -0.25, 0.4])

    def test_current_policy_receives_all_70_values(self) -> None:
        policy = SimpleNamespace(
            observation_space=SimpleNamespace(shape=(70,))
        )
        observation = np.arange(70, dtype=np.float32)

        adapted = _observation_for_policy(policy, observation)

        self.assertIs(adapted, observation)

    def test_legacy_policy_receives_first_68_values(self) -> None:
        policy = SimpleNamespace(
            observation_space=SimpleNamespace(shape=(68,))
        )
        observation = np.arange(70, dtype=np.float32)

        adapted = _observation_for_policy(policy, observation)

        np.testing.assert_array_equal(adapted, observation[:68])


class CheckpointMirroringTests(unittest.TestCase):
    @staticmethod
    def _write_valid_checkpoint(path: Path, marker: str) -> None:
        with zipfile.ZipFile(path, "w") as archive:
            archive.writestr("data", marker)

    def test_latest_complete_checkpoint_is_copied_and_validated(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            remote = root / "remote"
            remote.mkdir()
            self._write_valid_checkpoint(remote / "scone_walk_100_steps.zip", "old")
            self._write_valid_checkpoint(remote / "scone_walk_200_steps.zip", "new")
            source = LocalCheckpointSource(remote)

            candidate = source.latest("scone_walk")
            self.assertIsNotNone(candidate)
            self.assertEqual(candidate.step, 200)
            downloaded = mirror_checkpoint(source, candidate, root / "local")

            self.assertEqual(downloaded.name, "scone_walk_200_steps.zip")
            with zipfile.ZipFile(downloaded) as archive:
                self.assertEqual(archive.read("data"), b"new")

    def test_corrupt_checkpoint_is_never_published(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            remote = root / "remote"
            remote.mkdir()
            (remote / "scone_walk_300_steps.zip").write_bytes(b"not a zip")
            source = LocalCheckpointSource(remote)
            candidate = source.latest("scone_walk")

            with self.assertRaises(RuntimeError):
                mirror_checkpoint(source, candidate, root / "local")

            self.assertFalse((root / "local" / "scone_walk_300_steps.zip").exists())
            self.assertFalse((root / "local" / "scone_walk_300_steps.zip.part").exists())

    def test_refresh_replaces_a_mutable_final_model_atomically(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            remote = root / "remote"
            remote.mkdir()
            final_model = remote / "final_model.zip"
            self._write_valid_checkpoint(final_model, "first")
            source = LocalCheckpointSource(remote)
            candidate = SimpleNamespace(source_path=str(final_model), step=0)
            downloaded = mirror_checkpoint(source, candidate, root / "local")

            self._write_valid_checkpoint(final_model, "second")
            refreshed = mirror_checkpoint(
                source,
                candidate,
                root / "local",
                refresh=True,
            )

            self.assertEqual(refreshed, downloaded)
            with zipfile.ZipFile(refreshed) as archive:
                self.assertEqual(archive.read("data"), b"second")


if __name__ == "__main__":
    unittest.main()
