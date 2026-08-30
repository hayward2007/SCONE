from __future__ import annotations

import tempfile
import unittest
import zipfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from src.rl.remote_watch import (
    LocalCheckpointSource,
    _observation_for_policy,
    mirror_checkpoint,
)
from src.rl.policy_compat import load_compatible_policy
from src.rl.walk_learn import (
    GracefulStopCallback,
    SconeWalkEnv,
    _write_resume_pointer,
)


class RemoteWatchCompatibilityTests(unittest.TestCase):
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
