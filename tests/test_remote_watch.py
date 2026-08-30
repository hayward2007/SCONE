from __future__ import annotations

import tempfile
import unittest
import zipfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from src.rl.remote_watch import (
    LocalCheckpointSource,
    _observation_for_policy,
    mirror_checkpoint,
)
from src.rl.walk_learn import SconeWalkEnv


class RemoteWatchCompatibilityTests(unittest.TestCase):
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
