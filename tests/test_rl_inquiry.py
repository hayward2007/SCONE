from __future__ import annotations

import subprocess
import unittest
from unittest.mock import patch

from src.rl.inquiry import (
    RemoteJob,
    RemoteSettings,
    TrainingConfig,
    _prompt_remote_job,
    _remote_path_expression,
    build_remote_dependency_check_command,
    build_remote_dependency_install_command,
    build_remote_launch_command,
    build_remote_pause_command,
    build_remote_reset_command,
    build_remote_resume_command,
    build_training_arguments,
    ensure_remote_dependencies,
)


class _FakeChoice:
    def __init__(self, *, value, name: str) -> None:
        self.value = value
        self.name = name


class _FakeQuestion:
    def __init__(self, value) -> None:
        self.value = value

    def execute(self):
        return self.value


class _FakeInquirer:
    def __init__(self) -> None:
        self.choices = []

    def select(self, *, message: str, choices):
        self.choices = choices
        return _FakeQuestion(choices[0].value)


class RLInquiryCommandTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = TrainingConfig(
            task="walk",
            run_name="walk_easy_test",
            curriculum="easy",
            timesteps=250_000,
            num_envs=4,
            checkpoint_every=25_000,
            keep_checkpoints=7,
            seed=3,
            device="cpu",
        )

    def test_training_arguments_capture_interactive_settings(self) -> None:
        arguments = build_training_arguments(self.config)

        self.assertIn("train", arguments)
        self.assertEqual(arguments[arguments.index("--terrain") + 1], "flat")
        self.assertEqual(arguments[arguments.index("--timesteps") + 1], "250000")
        self.assertEqual(arguments[arguments.index("--num-envs") + 1], "4")
        self.assertEqual(
            arguments[arguments.index("--checkpoint-every") + 1], "25000"
        )
        self.assertEqual(
            arguments[arguments.index("--output") + 1], "runs/walk_easy_test"
        )

    def test_remote_command_is_detached_and_persists_pid_and_log(self) -> None:
        command = build_remote_launch_command(
            self.config,
            RemoteSettings(host="ssh.hayward.kim", project_dir="~/Developer/SCONE"),
        )

        self.assertIn('cd "$HOME"/Developer/SCONE', command)
        self.assertIn("nohup env PYTHONPATH=.", command)
        self.assertIn("-m src.rl.walk_learn", command)
        self.assertIn("--terrain flat --terrain-seed 7 train", command)
        self.assertIn("runs/walk_easy_test/train.log", command)
        self.assertIn("runs/walk_easy_test/train.pid", command)
        self.assertIn("--checkpoint-every 25000", command)
        self.assertIn("reset it first", command)
        self.assertIn("exit 23", command)
        self.assertIn("rmdir runs/walk_easy_test/checkpoints", command)
        self.assertIn("graceful_stop.enabled", command)
        self.assertIn("train.state", command)
        self.assertNotIn("import gymnasium", command)

    def test_remote_pause_is_graceful_and_never_forces_process_exit(self) -> None:
        job = RemoteJob(
            host="ssh.hayward.kim",
            project_dir="~/Developer/SCONE",
            run_name="walk_easy_test",
        )

        command = build_remote_pause_command(
            job,
            has_resume_checkpoint=True,
        )

        self.assertIn('kill -TERM "$scone_pid"', command)
        self.assertIn('printf "paused\\n" > "$scone_run/train.state"', command)
        self.assertNotIn("kill -KILL", command)
        self.assertNotIn("rm ", command)
        self.assertEqual(
            subprocess.run(["sh", "-n", "-c", command], check=False).returncode,
            0,
        )

    def test_remote_resume_appends_log_and_uses_same_run_directory(self) -> None:
        settings = RemoteSettings(
            host="ssh.hayward.kim",
            project_dir="~/Developer/SCONE",
        )
        command = build_remote_resume_command(
            self.config,
            settings,
            "~/Developer/SCONE/runs/walk_easy_test/final_model.zip",
        )

        self.assertIn('scone_run=runs/walk_easy_test', command)
        self.assertIn('--resume "$scone_resume"', command)
        self.assertIn('>> "$scone_run/train.log"', command)
        self.assertIn('> "$scone_run/resume.checkpoint"', command)
        self.assertIn("remote training is already running", command)
        self.assertNotIn('> runs/walk_easy_test/train.log 2>&1', command)
        self.assertEqual(
            subprocess.run(["sh", "-n", "-c", command], check=False).returncode,
            0,
        )

    def test_remote_dependencies_use_project_virtual_environment(self) -> None:
        settings = RemoteSettings(
            host="ssh.hayward.kim",
            project_dir="~/Developer/SCONE",
        )

        check = build_remote_dependency_check_command(settings)
        install = build_remote_dependency_install_command(settings)

        self.assertIn(".venv/bin/python", check)
        self.assertIn("sys.version_info[:2] == (3, 12)", check)
        self.assertIn("import gymnasium", check)
        self.assertIn("$HOME/.pyenv/shims/python3.12", install)
        self.assertIn('"$scone_python312" -m venv .venv', install)
        self.assertIn(".venv.python-old_", install)
        self.assertIn("pip install --upgrade pip setuptools wheel", install)
        self.assertIn("--only-binary=mujoco", install)
        self.assertIn("-r requirements-rl.txt", install)

    def test_missing_remote_dependencies_are_installed_then_verified(self) -> None:
        missing = subprocess.CompletedProcess([], 1, "", "missing gymnasium")
        verified = subprocess.CompletedProcess([], 0, "", "")
        installed = subprocess.CompletedProcess([], 0)

        with (
            patch("src.rl.inquiry._run_ssh", side_effect=[missing, verified]),
            patch("src.rl.inquiry.subprocess.run", return_value=installed) as run,
        ):
            ensure_remote_dependencies(
                RemoteSettings(),
                install_missing=True,
            )

        run.assert_called_once()

    def test_remote_tilde_path_expands_with_quoted_tail(self) -> None:
        expression = _remote_path_expression("~/Developer/SCONE folder")

        self.assertEqual(expression, '"$HOME"/\'Developer/SCONE folder\'')

    def test_invalid_run_name_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            TrainingConfig(
                task="walk",
                run_name="../../other",
                curriculum="easy",
                timesteps=1,
                num_envs=1,
                checkpoint_every=1,
                keep_checkpoints=1,
            )

    def test_ssh_host_cannot_be_an_option(self) -> None:
        with self.assertRaises(ValueError):
            RemoteSettings(host="-oProxyCommand=bad")

    def test_remote_reset_archives_instead_of_deleting(self) -> None:
        job = RemoteJob(
            host="ssh.hayward.kim",
            project_dir="~/Developer/SCONE",
            run_name="walk_easy_test",
        )

        command = build_remote_reset_command(job, stop_running=False)

        self.assertIn("runs/.reset_backup/walk_easy_test", command)
        self.assertIn('mv -- "$scone_run" "$scone_backup"', command)
        self.assertIn("exit 30", command)
        self.assertNotIn("rm ", command)

    def test_remote_job_prompt_uses_scalar_choice_and_returns_job(self) -> None:
        job = RemoteJob(
            host="ssh.hayward.kim",
            project_dir="~/Developer/SCONE",
            run_name="walk_easy_test",
        )
        fake_inquirer = _FakeInquirer()

        with (
            patch(
                "src.rl.inquiry._inquirer",
                return_value=(fake_inquirer, _FakeChoice),
            ),
            patch("src.rl.inquiry._load_remote_jobs", return_value=[job]),
        ):
            selected = _prompt_remote_job("choose")

        self.assertIs(selected, job)
        self.assertEqual(fake_inquirer.choices[0].value, 0)


if __name__ == "__main__":
    unittest.main()
