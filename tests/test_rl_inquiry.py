from __future__ import annotations

import subprocess
import unittest
from pathlib import Path
from unittest.mock import patch

from src.rl.inquiry import (
    RemoteJob,
    RemoteSettings,
    TrainingConfig,
    _prompt_remote_job,
    _remote_path_expression,
    build_remote_capacity_command,
    build_remote_dependency_check_command,
    build_remote_dependency_install_command,
    build_remote_launch_command,
    build_remote_pause_command,
    build_remote_reset_command,
    build_remote_resume_command,
    build_training_arguments,
    ensure_remote_dependencies,
    format_remote_capacity,
    inspect_remote_capacity,
    prompt_reference_motion,
    prompt_standing_pose,
    recommend_num_envs,
    view_local_model,
    watch_remote_job,
)
from src.rl.stance import STANDARD_STANDING_DEGREES


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
        self.default = None

    def select(self, *, message: str, choices, **options):
        self.choices = choices
        self.default = options.get("default")
        selected = choices[0].value if self.default is None else self.default
        return _FakeQuestion(selected)


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
        self.assertEqual(
            arguments[arguments.index("--reference-motion") + 1], "tripod-gait"
        )
        pose_start = arguments.index("--standing-pose-degrees") + 1
        self.assertEqual(
            tuple(float(value) for value in arguments[pose_start : pose_start + 18]),
            self.config.standing_pose_degrees,
        )
        self.assertEqual(arguments[arguments.index("--timesteps") + 1], "250000")
        self.assertEqual(arguments[arguments.index("--num-envs") + 1], "4")
        self.assertEqual(
            arguments[arguments.index("--checkpoint-every") + 1], "25000"
        )
        self.assertEqual(
            arguments[arguments.index("--output") + 1], "runs/walk_easy_test"
        )

    def test_live_remote_viewer_passes_the_recorded_v2_task(self) -> None:
        job = RemoteJob(
            host="ssh.hayward.kim",
            project_dir="~/Developer/SCONE",
            run_name="walk-v2-test",
            task="walk-v2",
            reference_motion="hardcoded",
        )
        completed = subprocess.CompletedProcess([], 0)

        with patch("src.rl.inquiry.subprocess.run", return_value=completed) as run:
            result = watch_remote_job(job)

        self.assertEqual(result, 0)
        command = run.call_args.args[0]
        self.assertEqual(command[command.index("--task") + 1], "walk-v2")
        self.assertEqual(
            command[command.index("--prefix") + 1], "scone_walk_v2"
        )

    def test_local_replay_routes_82_observations_to_walk_v2(self) -> None:
        completed = subprocess.CompletedProcess([], 0)
        with (
            patch("src.rl.remote_watch._validate_ppo_zip"),
            patch(
                "src.rl.policy_compat.checkpoint_observation_shape",
                return_value=(82,),
            ),
            patch("src.rl.inquiry.subprocess.run", return_value=completed) as run,
        ):
            result = view_local_model(Path("new-v2.zip"), episodes=1)

        self.assertEqual(result, 0)
        command = run.call_args.args[0]
        self.assertEqual(command[command.index("-m") + 1], "src.rl.walk_v2")
        self.assertIn("--command", command)

    def test_remote_command_is_detached_and_persists_pid_and_log(self) -> None:
        command = build_remote_launch_command(
            self.config,
            RemoteSettings(host="ssh.hayward.kim", project_dir="~/Developer/SCONE"),
        )

        self.assertIn('cd "$HOME"/Developer/SCONE', command)
        self.assertIn("nohup env PYTHONPATH=.", command)
        self.assertIn("-m src.rl.walk_learn", command)
        self.assertIn("--terrain flat --terrain-seed 7", command)
        self.assertIn("--reference-motion tripod-gait", command)
        self.assertIn("--standing-pose-degrees", command)
        self.assertIn("195 195 195 195 195 195 train", command)
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
        self.assertIn("-r requirements.txt", install)

    def test_remote_capacity_command_is_portable_shell(self) -> None:
        command = build_remote_capacity_command()

        self.assertIn("command -v python3", command)
        self.assertIn("hw.physicalcpu", command)
        self.assertIn("/proc/meminfo", command)
        self.assertEqual(
            subprocess.run(["sh", "-n", "-c", command], check=False).returncode,
            0,
        )

    def test_parallel_recommendation_reserves_one_core_and_parent_memory(self) -> None:
        cpu_limited = recommend_num_envs(
            physical_cores=8,
            logical_cores=16,
            total_memory_bytes=32 * 1024**3,
            available_memory_bytes=20 * 1024**3,
            load_average_1m=1.5,
        )
        memory_limited = recommend_num_envs(
            physical_cores=16,
            logical_cores=16,
            total_memory_bytes=8 * 1024**3,
            available_memory_bytes=int(3.5 * 1024**3),
            load_average_1m=2.0,
        )

        self.assertEqual(cpu_limited.cpu_limit, 7)
        self.assertEqual(cpu_limited.recommended_num_envs, 7)
        self.assertEqual(memory_limited.memory_limit, 2)
        self.assertEqual(memory_limited.recommended_num_envs, 2)
        self.assertIn("추천 2개", format_remote_capacity(memory_limited))
        self.assertIn("메모리 기준", format_remote_capacity(memory_limited))

    def test_remote_capacity_probe_parses_ssh_response(self) -> None:
        response = subprocess.CompletedProcess(
            [],
            0,
            '{"physical_cores": 10, "logical_cores": 10, '
            '"total_memory_bytes": 68719476736, '
            '"available_memory_bytes": 34359738368, '
            '"load_average_1m": 2.25}\n',
            "",
        )

        with patch("src.rl.inquiry._run_ssh", return_value=response):
            capacity = inspect_remote_capacity(RemoteSettings())

        self.assertEqual(capacity.recommended_num_envs, 9)
        self.assertEqual(capacity.physical_cores, 10)

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

    def test_standing_pose_prompt_recommends_high_standard_pose(self) -> None:
        fake_inquirer = _FakeInquirer()

        with patch(
            "src.rl.inquiry._inquirer",
            return_value=(fake_inquirer, _FakeChoice),
        ):
            name, degrees = prompt_standing_pose()

        self.assertEqual(name, "standard")
        self.assertEqual(degrees, STANDARD_STANDING_DEGREES)

    def test_reference_motion_prompt_defaults_new_training_to_tripod_gait(self) -> None:
        fake_inquirer = _FakeInquirer()

        with patch(
            "src.rl.inquiry._inquirer",
            return_value=(fake_inquirer, _FakeChoice),
        ):
            selection = prompt_reference_motion()

        self.assertEqual(selection, "tripod-gait")
        self.assertEqual(fake_inquirer.choices[0].value, "tripod-gait")
        self.assertEqual(fake_inquirer.choices[1].value, "scone-gait")
        self.assertEqual(fake_inquirer.default, "tripod-gait")

    def test_reference_motion_prompt_can_restore_legacy_replay_reference(self) -> None:
        fake_inquirer = _FakeInquirer()

        with patch(
            "src.rl.inquiry._inquirer",
            return_value=(fake_inquirer, _FakeChoice),
        ):
            selection = prompt_reference_motion(default="hardcoded")

        self.assertEqual(selection, "hardcoded")
        self.assertEqual(fake_inquirer.default, "hardcoded")

    def test_legacy_non_rl_configuration_is_canonicalized(self) -> None:
        config = TrainingConfig(
            task="walk",
            run_name="legacy-alias",
            curriculum="easy",
            timesteps=1,
            num_envs=1,
            checkpoint_every=1,
            keep_checkpoints=1,
            reference_motion="non_rl",
        )

        self.assertEqual(config.reference_motion, "tripod-gait")


if __name__ == "__main__":
    unittest.main()
