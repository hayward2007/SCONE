"""Interactive launcher for local and SSH-based reinforcement learning.

The low-level training commands remain available in :mod:`src.rl.walk_learn`.
This module provides the human-facing workflow: configure a run, start it in
the foreground or on an SSH host, inspect remote logs, mirror checkpoints, and
open a trained policy in MuJoCo.
"""

from __future__ import annotations

import json
import os
import re
import shlex
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any, Sequence

from .stance import (
    SPORT_STANDING_DEGREES,
    STANDARD_STANDING_DEGREES,
    UPPER_STANDING_DEGREES,
    validate_standing_pose,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUNS_DIR = PROJECT_ROOT / "runs"
REMOTE_JOBS_FILE = RUNS_DIR / ".remote_jobs.json"
DEFAULT_REMOTE_HOST = "ssh.hayward.kim"
DEFAULT_REMOTE_PROJECT = "~/Developer/SCONE"

SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
SAFE_SSH_HOST = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.@-]*$")
TERRAIN_OPTIONS = (
    ("flat", "평지"),
    ("uneven", "울퉁불퉁한 지형"),
    ("stairs-1", "계단 1단계"),
    ("stairs-2", "계단 2단계"),
    ("stairs-3", "계단 3단계"),
    ("slope-1", "경사 1단계 (8°)"),
    ("slope-2", "경사 2단계 (15°)"),
    ("slope-3", "경사 3단계 (25°)"),
    ("mixed", "혼합 코스"),
)


@dataclass(frozen=True)
class TrainingTask:
    key: str
    label: str
    module: str
    checkpoint_prefix: str


TRAINING_TASKS = {
    "walk": TrainingTask(
        key="walk",
        label="걷기 정책 (PPO residual walk)",
        module="src.rl.walk_learn",
        checkpoint_prefix="scone_walk",
    )
}


class RemoteDependencyError(RuntimeError):
    """Raised when the SSH Python environment cannot import the RL stack."""


@dataclass(frozen=True)
class TrainingConfig:
    task: str
    run_name: str
    curriculum: str
    timesteps: int
    num_envs: int
    checkpoint_every: int
    keep_checkpoints: int
    terrain: str = "flat"
    terrain_seed: int = 7
    seed: int = 0
    device: str = "auto"
    standing_pose_name: str = "sport"
    standing_pose_degrees: tuple[float, ...] = SPORT_STANDING_DEGREES

    def __post_init__(self) -> None:
        if self.task not in TRAINING_TASKS:
            raise ValueError(f"unknown training task: {self.task}")
        if self.curriculum not in {"easy", "medium", "full"}:
            raise ValueError(f"unknown curriculum: {self.curriculum}")
        if SAFE_NAME.fullmatch(self.run_name) is None:
            raise ValueError(
                "run name may contain only letters, digits, '.', '_', and '-'"
            )
        if self.terrain not in {value for value, _ in TERRAIN_OPTIONS}:
            raise ValueError(f"unknown terrain: {self.terrain}")
        for name in ("timesteps", "num_envs", "checkpoint_every", "keep_checkpoints"):
            if getattr(self, name) < 1:
                raise ValueError(f"{name} must be at least 1")
        if not self.standing_pose_name.strip():
            raise ValueError("standing_pose_name cannot be empty")
        object.__setattr__(
            self,
            "standing_pose_degrees",
            validate_standing_pose(self.standing_pose_degrees),
        )

    @property
    def task_spec(self) -> TrainingTask:
        return TRAINING_TASKS[self.task]

    @property
    def relative_run_dir(self) -> str:
        return f"runs/{self.run_name}"


@dataclass(frozen=True)
class RemoteSettings:
    host: str = DEFAULT_REMOTE_HOST
    project_dir: str = DEFAULT_REMOTE_PROJECT
    port: int | None = None
    connect_timeout: int = 8

    def __post_init__(self) -> None:
        if SAFE_SSH_HOST.fullmatch(self.host) is None:
            raise ValueError(
                "SSH host may contain only letters, digits, '.', '_', '-', and '@'"
            )
        if "\n" in self.project_dir or "\0" in self.project_dir:
            raise ValueError("remote project path cannot contain newline or NUL")
        if self.port is not None and not 1 <= self.port <= 65535:
            raise ValueError("SSH port must be between 1 and 65535")


@dataclass(frozen=True)
class RemoteJob:
    host: str
    project_dir: str
    run_name: str
    task: str = "walk"
    port: int | None = None
    pid: int | None = None
    created_at: str = ""
    terrain: str = "flat"
    terrain_seed: int = 7
    curriculum: str = "easy"
    num_envs: int = 4
    checkpoint_every: int = 100_000
    keep_checkpoints: int = 10
    seed: int = 0
    device: str = "auto"
    standing_pose_name: str = "sport"
    standing_pose_degrees: tuple[float, ...] = SPORT_STANDING_DEGREES

    def __post_init__(self) -> None:
        RemoteSettings(host=self.host, project_dir=self.project_dir, port=self.port)
        if SAFE_NAME.fullmatch(self.run_name) is None:
            raise ValueError(f"invalid remote run name: {self.run_name!r}")
        if self.task not in TRAINING_TASKS:
            raise ValueError(f"unknown training task: {self.task}")
        if self.terrain not in {value for value, _ in TERRAIN_OPTIONS}:
            raise ValueError(f"unknown terrain: {self.terrain}")
        if self.curriculum not in {"easy", "medium", "full"}:
            raise ValueError(f"unknown curriculum: {self.curriculum}")
        for name in ("num_envs", "checkpoint_every", "keep_checkpoints"):
            if getattr(self, name) < 1:
                raise ValueError(f"{name} must be at least 1")
        if not self.standing_pose_name.strip():
            raise ValueError("standing_pose_name cannot be empty")
        object.__setattr__(
            self,
            "standing_pose_degrees",
            validate_standing_pose(self.standing_pose_degrees),
        )

    @property
    def settings(self) -> RemoteSettings:
        return RemoteSettings(
            host=self.host,
            project_dir=self.project_dir,
            port=self.port,
        )

    @property
    def task_spec(self) -> TrainingTask:
        return TRAINING_TASKS[self.task]

    @property
    def relative_run_dir(self) -> str:
        return f"runs/{self.run_name}"


def _remote_path_expression(path: str) -> str:
    """Quote a remote path while still allowing an initial ``~/`` to expand."""

    if "\n" in path or "\0" in path:
        raise ValueError("remote path cannot contain newline or NUL")
    if path == "~":
        return '"$HOME"'
    if path.startswith("~/"):
        return '"$HOME"/' + shlex.quote(path[2:])
    return shlex.quote(path)


def _ssh_command(settings: RemoteSettings) -> list[str]:
    command = [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        f"ConnectTimeout={settings.connect_timeout}",
    ]
    if settings.port is not None:
        command.extend(["-p", str(settings.port)])
    command.append(settings.host)
    return command


def _run_ssh(
    settings: RemoteSettings,
    remote_command: str,
    *,
    timeout: int = 30,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        [*_ssh_command(settings), remote_command],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )
    if check and result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(detail or f"SSH exited with code {result.returncode}")
    return result


def build_training_arguments(config: TrainingConfig) -> list[str]:
    """Build the common arguments used by local and remote training."""

    return [
        "--terrain",
        config.terrain,
        "--terrain-seed",
        str(config.terrain_seed),
        "--standing-pose-degrees",
        *(f"{degrees:g}" for degrees in config.standing_pose_degrees),
        "train",
        "--curriculum",
        config.curriculum,
        "--timesteps",
        str(config.timesteps),
        "--num-envs",
        str(config.num_envs),
        "--checkpoint-every",
        str(config.checkpoint_every),
        "--keep-checkpoints",
        str(config.keep_checkpoints),
        "--seed",
        str(config.seed),
        "--device",
        config.device,
        "--output",
        config.relative_run_dir,
        "--tensorboard-log",
        f"{config.relative_run_dir}/tensorboard",
    ]


def build_remote_launch_command(config: TrainingConfig, settings: RemoteSettings) -> str:
    """Return the POSIX-shell command that starts one detached remote trainer."""

    project = _remote_path_expression(settings.project_dir)
    run_dir = shlex.quote(config.relative_run_dir)
    train_command = [
        '"$scone_python"',
        "-m",
        shlex.quote(config.task_spec.module),
        *(shlex.quote(argument) for argument in build_training_arguments(config)),
    ]
    return " ".join(
        [
            f"cd {project} || exit 20;",
            # A dependency failure in older launcher versions left exactly
            # these two empty directories behind. rmdir never removes data.
            f"rmdir {run_dir}/checkpoints {run_dir} 2>/dev/null || true;",
            f"if [ -e {run_dir} ]; then",
            f'printf "remote run already exists; reset it first: %s\\n" {run_dir} >&2;',
            "exit 23; fi;",
            f"mkdir -p {run_dir}/checkpoints || exit 21;",
            f'printf "1\\n" > {run_dir}/graceful_stop.enabled;',
            f'printf "running\\n" > {run_dir}/train.state;',
            "scone_python=.venv/bin/python;",
            '[ -x "$scone_python" ] || exit 42;',
            "nohup env PYTHONPATH=. PYTHONUNBUFFERED=1",
            *train_command,
            f"> {run_dir}/train.log 2>&1 < /dev/null &",
            "scone_pid=$!;",
            f'printf "%s\\n" "$scone_pid" > {run_dir}/train.pid;',
            'printf "%s\\n" "$scone_pid"',
        ]
    )


def build_remote_resume_command(
    config: TrainingConfig,
    settings: RemoteSettings,
    resume_checkpoint: str,
) -> str:
    """Return a detached command that continues an existing remote run."""

    project = _remote_path_expression(settings.project_dir)
    run_dir = shlex.quote(config.relative_run_dir)
    resume_path = _remote_path_expression(resume_checkpoint)
    train_command = [
        '"$scone_python"',
        "-m",
        shlex.quote(config.task_spec.module),
        *(shlex.quote(argument) for argument in build_training_arguments(config)),
        "--resume",
        '"$scone_resume"',
    ]
    return " ".join(
        [
            f"cd {project} || exit 20;",
            f"scone_run={run_dir};",
            f"scone_resume={resume_path};",
            'if [ ! -d "$scone_run" ]; then',
            'printf "remote run directory not found: %s\\n" "$scone_run" >&2;',
            "exit 24; fi;",
            'if [ -f "$scone_run/train.pid" ]; then',
            'scone_old_pid=$(cat "$scone_run/train.pid");',
            'if kill -0 "$scone_old_pid" 2>/dev/null; then',
            'printf "remote training is already running (PID %s)\\n" '
            '"$scone_old_pid" >&2;',
            "exit 25; fi; fi;",
            'if [ ! -f "$scone_resume" ]; then',
            'printf "resume checkpoint not found: %s\\n" '
            '"$scone_resume" >&2;',
            "exit 26; fi;",
            'mkdir -p "$scone_run/checkpoints" || exit 21;',
            'printf "1\\n" > "$scone_run/graceful_stop.enabled";',
            'case "$scone_resume" in',
            '"$scone_run"/*) '
            'scone_pointer=${scone_resume#"$scone_run"/} ;;',
            '*) scone_pointer="$scone_resume" ;;',
            "esac;",
            'printf "%s\\n" "$scone_pointer" '
            '> "$scone_run/resume.checkpoint";',
            'printf "running\\n" > "$scone_run/train.state";',
            "scone_python=.venv/bin/python;",
            '[ -x "$scone_python" ] || exit 42;',
            'printf "\\n[RL] 이어서 학습을 시작합니다: %s\\n" '
            '"$scone_resume" >> "$scone_run/train.log";',
            "nohup env PYTHONPATH=. PYTHONUNBUFFERED=1",
            *train_command,
            '>> "$scone_run/train.log" 2>&1 < /dev/null &',
            "scone_pid=$!;",
            'printf "%s\\n" "$scone_pid" > "$scone_run/train.pid";',
            'printf "%s\\n" "$scone_pid"',
        ]
    )


def sync_project_to_remote(settings: RemoteSettings) -> None:
    """Copy runnable source to the SSH machine without touching remote run data."""

    print(f"[RL] 로컬 코드를 {settings.host}:{settings.project_dir} 로 동기화합니다...")
    _run_ssh(
        settings,
        f"mkdir -p {_remote_path_expression(settings.project_dir)}",
    )
    ssh_transport = ["ssh"]
    if settings.port is not None:
        ssh_transport.extend(["-p", str(settings.port)])
    command = [
        "rsync",
        "-az",
        "--exclude=.git/",
        "--exclude=.venv/",
        "--exclude=venv/",
        "--exclude=runs/",
        "--exclude=archive/",
        "--exclude=__pycache__/",
        "--exclude=*.pyc",
        "--exclude=.DS_Store",
        "-e",
        shlex.join(ssh_transport),
        f"{PROJECT_ROOT}/",
        f"{settings.host}:{settings.project_dir.rstrip('/')}/",
    ]
    result = subprocess.run(command, cwd=PROJECT_ROOT, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"rsync exited with code {result.returncode}")


def build_remote_dependency_check_command(settings: RemoteSettings) -> str:
    project = _remote_path_expression(settings.project_dir)
    import_check = shlex.quote(
        "import sys; assert sys.version_info[:2] == (3, 12), sys.version; "
        "import gymnasium, mujoco, stable_baselines3; import src.rl.walk_learn"
    )
    return " ".join(
        [
            f"cd {project} || exit 20;",
            "scone_python=.venv/bin/python;",
            '[ -x "$scone_python" ] || exit 42;',
            f'PYTHONPATH=. "$scone_python" -c {import_check}',
        ]
    )


def build_remote_dependency_install_command(settings: RemoteSettings) -> str:
    project = _remote_path_expression(settings.project_dir)
    version_check = shlex.quote(
        "import sys; assert sys.version_info[:2] == (3, 12), sys.version"
    )
    return " ".join(
        [
            f"cd {project} || exit 20;",
            'scone_python312="";',
            "if command -v python3.12 >/dev/null 2>&1; then",
            'scone_python312=$(command -v python3.12);',
            'elif [ -x "$HOME/.pyenv/shims/python3.12" ]; then',
            'scone_python312="$HOME/.pyenv/shims/python3.12";',
            'elif [ -x /opt/homebrew/bin/python3.12 ]; then',
            "scone_python312=/opt/homebrew/bin/python3.12;",
            'elif [ -x /usr/local/bin/python3.12 ]; then',
            "scone_python312=/usr/local/bin/python3.12;",
            "else",
            'printf "Python 3.12 was not found on the SSH host\\n" >&2;',
            "exit 42; fi;",
            f'"$scone_python312" -c {version_check} || exit 42;',
            "if [ -d .venv ]; then",
            "if [ ! -x .venv/bin/python ] ||",
            f"! .venv/bin/python -c {version_check} >/dev/null 2>&1; then",
            'scone_stamp=$(date +%Y%m%d_%H%M%S);',
            'scone_old_venv=.venv.python-old_${scone_stamp}_$$;',
            'mv .venv "$scone_old_venv" || exit 43;',
            'printf "[RL] previous virtualenv backed up to %s\\n" "$scone_old_venv";',
            "fi; fi;",
            "if [ ! -x .venv/bin/python ]; then",
            '"$scone_python312" -m venv .venv || exit 40;',
            "fi;",
            ".venv/bin/python -m pip install --upgrade pip setuptools wheel || exit 41;",
            ".venv/bin/python -m pip install --prefer-binary "
            "--only-binary=mujoco -r requirements-rl.txt",
        ]
    )


def ensure_remote_dependencies(
    settings: RemoteSettings,
    *,
    install_missing: bool,
) -> None:
    check = _run_ssh(
        settings,
        build_remote_dependency_check_command(settings),
        check=False,
        timeout=45,
    )
    if check.returncode == 0:
        return

    detail = check.stderr.strip() or check.stdout.strip()
    if not install_missing:
        raise RemoteDependencyError(
            detail
            or "원격 RL 의존성이 없습니다. 원격 자동 설치를 허용하고 다시 시도하세요."
        )

    print("[RL] 원격 .venv를 준비하고 RL 의존성을 설치합니다...")
    install = subprocess.run(
        [
            *_ssh_command(settings),
            build_remote_dependency_install_command(settings),
        ],
        cwd=PROJECT_ROOT,
        text=True,
        timeout=1_200,
        check=False,
    )
    if install.returncode != 0:
        raise RemoteDependencyError(
            "원격 의존성 설치에 실패했습니다. 위 pip/venv 오류를 확인하세요."
        )

    verified = _run_ssh(
        settings,
        build_remote_dependency_check_command(settings),
        check=False,
        timeout=45,
    )
    if verified.returncode != 0:
        detail = verified.stderr.strip() or verified.stdout.strip()
        raise RemoteDependencyError(
            detail or "설치 후에도 원격 RL 모듈을 불러오지 못했습니다."
        )


def run_local_training(config: TrainingConfig) -> int:
    local_run = RUNS_DIR / config.run_name
    if local_run.exists():
        raise FileExistsError(
            f"로컬 실행 폴더가 이미 있습니다: {local_run}. "
            "다른 실행 이름을 사용하거나 기존 폴더를 먼저 보관하세요."
        )
    env = os.environ.copy()
    existing_pythonpath = env.get("PYTHONPATH")
    env["PYTHONPATH"] = (
        str(PROJECT_ROOT)
        if not existing_pythonpath
        else os.pathsep.join([str(PROJECT_ROOT), existing_pythonpath])
    )
    command = [
        sys.executable,
        "-m",
        config.task_spec.module,
        *build_training_arguments(config),
    ]
    print(f"[RL] 로컬 학습을 시작합니다: {shlex.join(command)}")
    return subprocess.run(command, cwd=PROJECT_ROOT, env=env, check=False).returncode


def run_environment_check(
    *,
    curriculum: str,
    terrain: str,
    steps: int,
    random_actions: bool,
    standing_pose_degrees: Sequence[float] = SPORT_STANDING_DEGREES,
) -> int:
    """Run the environment/reward smoke check before committing to training."""

    command = [
        sys.executable,
        "-m",
        "src.rl.walk_learn",
        "--terrain",
        terrain,
        "--standing-pose-degrees",
        *(f"{degrees:g}" for degrees in standing_pose_degrees),
        "check",
        "--curriculum",
        curriculum,
        "--steps",
        str(steps),
    ]
    if random_actions:
        command.append("--random-actions")
    return subprocess.run(command, cwd=PROJECT_ROOT, check=False).returncode


def start_remote_training(
    config: TrainingConfig,
    settings: RemoteSettings,
    *,
    sync_code: bool,
    install_missing_dependencies: bool = True,
) -> RemoteJob:
    if sync_code:
        sync_project_to_remote(settings)
    else:
        check_command = (
            f"test -f {_remote_path_expression(settings.project_dir + '/src/rl/walk_learn.py')}"
        )
        _run_ssh(settings, check_command)

    ensure_remote_dependencies(
        settings,
        install_missing=install_missing_dependencies,
    )

    result = _run_ssh(
        settings,
        build_remote_launch_command(config, settings),
        timeout=45,
    )
    try:
        pid = int(result.stdout.strip().splitlines()[-1])
    except (IndexError, ValueError) as exc:
        raise RuntimeError(f"remote trainer did not return a PID: {result.stdout!r}") from exc

    job = RemoteJob(
        host=settings.host,
        project_dir=settings.project_dir,
        port=settings.port,
        run_name=config.run_name,
        task=config.task,
        pid=pid,
        created_at=datetime.now().astimezone().isoformat(timespec="seconds"),
        terrain=config.terrain,
        terrain_seed=config.terrain_seed,
        curriculum=config.curriculum,
        num_envs=config.num_envs,
        checkpoint_every=config.checkpoint_every,
        keep_checkpoints=config.keep_checkpoints,
        seed=config.seed,
        device=config.device,
        standing_pose_name=config.standing_pose_name,
        standing_pose_degrees=config.standing_pose_degrees,
    )
    _save_remote_job(job)
    return job


def _remote_job_command(job: RemoteJob, body: str) -> str:
    project = _remote_path_expression(job.project_dir)
    run_dir = shlex.quote(job.relative_run_dir)
    return f"cd {project} || exit 20; scone_run={run_dir}; {body}"


def remote_job_status(job: RemoteJob, *, log_lines: int = 20) -> str:
    body = " ".join(
        [
            'if [ ! -f "$scone_run/train.pid" ]; then',
            'printf "상태: PID 파일 없음\\n";',
            "else",
            'scone_pid=$(cat "$scone_run/train.pid");',
            'if kill -0 "$scone_pid" 2>/dev/null; then scone_state=실행중;',
            'elif [ -f "$scone_run/train.state" ] &&',
            '[ "$(cat "$scone_run/train.state")" = paused ]; then',
            "scone_state=일시정지됨;",
            "else scone_state=종료됨; fi;",
            'printf "상태: %s (PID %s)\\n" "$scone_state" "$scone_pid";',
            "fi;",
            'if [ -f "$scone_run/final_model.zip" ]; then',
            'printf "최종 모델: 저장됨\\n";',
            "fi;",
            'if [ -s "$scone_run/resume.checkpoint" ]; then',
            'printf "이어하기 체크포인트: %s\\n" '
            '"$(sed -n \'1p\' "$scone_run/resume.checkpoint")";',
            "fi;",
            f'if [ -f "$scone_run/train.log" ]; then tail -n {int(log_lines)} "$scone_run/train.log"; fi',
        ]
    )
    result = _run_ssh(job.settings, _remote_job_command(job, body), check=False)
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(detail or f"SSH exited with code {result.returncode}")
    return result.stdout.rstrip()


def remote_job_is_running(job: RemoteJob) -> bool:
    body = " ".join(
        [
            'if [ -f "$scone_run/train.pid" ]; then',
            'scone_pid=$(cat "$scone_run/train.pid");',
            'if kill -0 "$scone_pid" 2>/dev/null; then printf "running\\n";',
            'else printf "stopped\\n"; fi;',
            'else printf "stopped\\n"; fi',
        ]
    )
    result = _run_ssh(job.settings, _remote_job_command(job, body))
    return result.stdout.strip() == "running"


def build_remote_pause_command(
    job: RemoteJob,
    *,
    has_resume_checkpoint: bool,
) -> str:
    """Build a graceful, non-forcing stop command for a remote trainer."""

    ready_flag = "1" if has_resume_checkpoint else "0"
    body = " ".join(
        [
            'if [ ! -f "$scone_run/train.pid" ]; then',
            'printf "remote training PID file not found\\n" >&2;',
            "exit 24; fi;",
            'scone_pid=$(cat "$scone_run/train.pid");',
            'if ! kill -0 "$scone_pid" 2>/dev/null; then',
            'printf "remote training is not running (PID %s)\\n" '
            '"$scone_pid" >&2;',
            "exit 25; fi;",
            f'if [ ! -f "$scone_run/graceful_stop.enabled" ] && '
            f'[ {ready_flag} != 1 ]; then',
            'printf "no safe resume checkpoint exists yet; pause cancelled\\n" >&2;',
            "exit 34; fi;",
            'kill -TERM "$scone_pid" || exit 30;',
            "scone_wait=0;",
            'while kill -0 "$scone_pid" 2>/dev/null && '
            '[ "$scone_wait" -lt 300 ]; do',
            "sleep 0.2; scone_wait=$((scone_wait + 1)); done;",
            'if kill -0 "$scone_pid" 2>/dev/null; then',
            'printf "remote training did not stop within 60 seconds; '
            'no forced kill was sent\\n" >&2;',
            "exit 31; fi;",
            'printf "paused\\n" > "$scone_run/train.state";',
            'printf "paused\\n"',
        ]
    )
    return _remote_job_command(job, body)


def build_remote_reset_command(job: RemoteJob, *, stop_running: bool) -> str:
    """Build a recoverable remote reset command for one exact run directory."""

    project = _remote_path_expression(job.project_dir)
    run_dir = shlex.quote(job.relative_run_dir)
    backup_root = shlex.quote("runs/.reset_backup")
    backup_prefix = shlex.quote(f"runs/.reset_backup/{job.run_name}")
    stop_flag = "1" if stop_running else "0"
    return " ".join(
        [
            f"cd {project} || exit 20;",
            f"scone_run={run_dir};",
            'if [ ! -d "$scone_run" ]; then',
            'printf "remote run directory not found: %s\\n" "$scone_run" >&2;',
            "exit 24; fi;",
            'scone_pid="";',
            'if [ -f "$scone_run/train.pid" ]; then',
            'scone_pid=$(cat "$scone_run/train.pid"); fi;',
            'if [ -n "$scone_pid" ] && kill -0 "$scone_pid" 2>/dev/null; then',
            f'if [ {stop_flag} != 1 ]; then',
            'printf "remote training is still running (PID %s)\\n" "$scone_pid" >&2;',
            "exit 30; fi;",
            'kill "$scone_pid";',
            "scone_wait=0;",
            'while kill -0 "$scone_pid" 2>/dev/null && [ "$scone_wait" -lt 300 ]; do',
            "sleep 0.2; scone_wait=$((scone_wait + 1)); done;",
            'if kill -0 "$scone_pid" 2>/dev/null; then',
            'printf "remote training did not stop; reset cancelled\\n" >&2;',
            "exit 31; fi; fi;",
            f"mkdir -p {backup_root} || exit 32;",
            'scone_stamp=$(date +%Y%m%d_%H%M%S);',
            f'scone_backup={backup_prefix}_${{scone_stamp}}_$$;',
            'mv -- "$scone_run" "$scone_backup" || exit 33;',
            'printf "%s\\n" "$scone_backup"',
        ]
    )


def reset_remote_run(job: RemoteJob, *, stop_running: bool = False) -> str:
    """Archive a remote run so a reward-incompatible policy can start cleanly."""

    result = _run_ssh(
        job.settings,
        build_remote_reset_command(job, stop_running=stop_running),
        timeout=75,
    )
    backup_path = result.stdout.strip().splitlines()
    if not backup_path:
        raise RuntimeError("remote reset did not return its backup path")
    return backup_path[-1]


def _remote_existing_file(settings: RemoteSettings, remote_path: str) -> str | None:
    expression = _remote_path_expression(remote_path)
    result = _run_ssh(
        settings,
        f'if [ -f {expression} ]; then printf "%s\\n" {expression}; fi',
    )
    path = result.stdout.strip()
    return path or None


def _remote_resume_pointer(job: RemoteJob) -> str | None:
    body = " ".join(
        [
            'if [ -s "$scone_run/resume.checkpoint" ]; then',
            'scone_pointer=$(sed -n \'1p\' "$scone_run/resume.checkpoint");',
            'case "$scone_pointer" in',
            '/*) scone_checkpoint="$scone_pointer" ;;',
            '*) scone_checkpoint="$scone_run/$scone_pointer" ;;',
            "esac;",
            'if [ -f "$scone_checkpoint" ]; then',
            'printf "%s\\n" "$scone_checkpoint"; fi; fi',
        ]
    )
    result = _run_ssh(job.settings, _remote_job_command(job, body))
    checkpoint = result.stdout.strip()
    return checkpoint or None


def find_remote_resume_checkpoint(job: RemoteJob) -> str | None:
    """Return the exact remote policy file that should be resumed."""

    pointer = _remote_resume_pointer(job)
    if pointer is not None:
        return pointer

    from .remote_watch import SSHCheckpointSource

    remote_run = f"{job.project_dir.rstrip('/')}/{job.relative_run_dir}"
    source = SSHCheckpointSource(
        job.host,
        f"{remote_run}/checkpoints",
        port=job.port,
    )
    candidate = source.latest(job.task_spec.checkpoint_prefix)
    if candidate is not None:
        return candidate.source_path

    return _remote_existing_file(
        job.settings,
        f"{remote_run}/final_model.zip",
    )


def pause_remote_training(job: RemoteJob) -> str:
    """Gracefully pause a trainer and return its resumable checkpoint path."""

    fallback = find_remote_resume_checkpoint(job)
    _run_ssh(
        job.settings,
        build_remote_pause_command(
            job,
            has_resume_checkpoint=fallback is not None,
        ),
        timeout=75,
    )
    checkpoint = find_remote_resume_checkpoint(job) or fallback
    if checkpoint is None:
        raise RuntimeError(
            "학습은 중지됐지만 이어서 사용할 체크포인트를 찾지 못했습니다."
        )
    _save_remote_job(replace(job, pid=None))
    return checkpoint


def resume_remote_training(
    job: RemoteJob,
    *,
    additional_timesteps: int,
    sync_code: bool,
    install_missing_dependencies: bool = True,
) -> tuple[RemoteJob, str]:
    """Continue a stopped job in the same run directory and log files."""

    if additional_timesteps < 1:
        raise ValueError("additional_timesteps must be at least 1")
    if remote_job_is_running(job):
        raise RuntimeError(f"{job.run_name} 학습은 이미 실행 중입니다.")

    if sync_code:
        sync_project_to_remote(job.settings)
    else:
        check_command = (
            f"test -f "
            f"{_remote_path_expression(job.project_dir + '/src/rl/walk_learn.py')}"
        )
        _run_ssh(job.settings, check_command)
    ensure_remote_dependencies(
        job.settings,
        install_missing=install_missing_dependencies,
    )

    checkpoint = find_remote_resume_checkpoint(job)
    if checkpoint is None:
        raise FileNotFoundError(
            f"{job.host}:{job.project_dir}/{job.relative_run_dir} 에 "
            "이어갈 체크포인트가 없습니다."
        )
    config = TrainingConfig(
        task=job.task,
        run_name=job.run_name,
        curriculum=job.curriculum,
        timesteps=additional_timesteps,
        num_envs=job.num_envs,
        checkpoint_every=job.checkpoint_every,
        keep_checkpoints=job.keep_checkpoints,
        terrain=job.terrain,
        terrain_seed=job.terrain_seed,
        seed=job.seed,
        device=job.device,
        standing_pose_name=job.standing_pose_name,
        standing_pose_degrees=job.standing_pose_degrees,
    )
    result = _run_ssh(
        job.settings,
        build_remote_resume_command(config, job.settings, checkpoint),
        timeout=45,
    )
    try:
        pid = int(result.stdout.strip().splitlines()[-1])
    except (IndexError, ValueError) as exc:
        raise RuntimeError(
            f"remote trainer did not return a PID: {result.stdout!r}"
        ) from exc
    resumed_job = replace(job, pid=pid)
    _save_remote_job(resumed_job)
    return resumed_job, checkpoint


def download_remote_artifacts(job: RemoteJob) -> list[Path]:
    """Mirror the newest checkpoint and final model, if present, into ``runs``."""

    # Import lazily so the launcher can still explain missing RL dependencies.
    from .remote_watch import (
        CheckpointCandidate,
        SSHCheckpointSource,
        mirror_checkpoint,
    )

    remote_run = f"{job.project_dir.rstrip('/')}/{job.relative_run_dir}"
    remote_checkpoints = f"{remote_run}/checkpoints"
    local_run = RUNS_DIR / job.run_name
    source = SSHCheckpointSource(
        job.host,
        remote_checkpoints,
        port=job.port,
    )
    downloaded: list[Path] = []
    candidate = source.latest(job.task_spec.checkpoint_prefix)
    if candidate is not None:
        downloaded.append(
            mirror_checkpoint(source, candidate, local_run / "checkpoints")
        )

    remote_final = _remote_existing_file(
        job.settings,
        f"{remote_run}/final_model.zip",
    )
    if remote_final is not None:
        final_candidate = CheckpointCandidate(remote_final, candidate.step if candidate else 0)
        downloaded.append(
            mirror_checkpoint(source, final_candidate, local_run, refresh=True)
        )

    if not downloaded:
        raise FileNotFoundError(
            f"아직 {job.host}:{remote_run} 에 내려받을 체크포인트가 없습니다."
        )
    return downloaded


def local_model_files() -> list[Path]:
    if not RUNS_DIR.exists():
        return []
    return sorted(
        (
            path
            for path in RUNS_DIR.rglob("*.zip")
            if path.is_file() and not path.name.endswith(".part")
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )


def view_local_model(
    checkpoint: Path,
    *,
    command: Sequence[float] = (0.25, 0.0, 0.0),
    episodes: int = 3,
    terrain: str = "flat",
    terrain_seed: int = 7,
    standing_pose_degrees: Sequence[float] = SPORT_STANDING_DEGREES,
) -> int:
    from .remote_watch import _validate_ppo_zip

    checkpoint = checkpoint.expanduser().resolve()
    _validate_ppo_zip(checkpoint)
    executable = shutil.which("mjpython") or sys.executable
    process = [
        executable,
        "-m",
        "src.rl.walk_learn",
        "--terrain",
        terrain,
        "--terrain-seed",
        str(terrain_seed),
        "--standing-pose-degrees",
        *(f"{degrees:g}" for degrees in standing_pose_degrees),
        "enjoy",
        str(checkpoint),
        "--command",
        *(str(value) for value in command),
        "--episodes",
        str(episodes),
    ]
    return subprocess.run(process, cwd=PROJECT_ROOT, check=False).returncode


def watch_remote_job(job: RemoteJob) -> int:
    executable = shutil.which("mjpython") or sys.executable
    remote_run = f"{job.project_dir.rstrip('/')}/{job.relative_run_dir}"
    process = [
        executable,
        "-m",
        "src.rl.remote_watch",
        "--host",
        job.host,
        "--checkpoint-dir",
        f"{remote_run}/checkpoints",
        "--cache-dir",
        str(RUNS_DIR / job.run_name / "checkpoints"),
        "--prefix",
        job.task_spec.checkpoint_prefix,
        "--terrain",
        job.terrain,
        "--terrain-seed",
        str(job.terrain_seed),
        "--standing-pose-degrees",
        *(f"{degrees:g}" for degrees in job.standing_pose_degrees),
    ]
    if job.port is not None:
        process.extend(["--port", str(job.port)])
    return subprocess.run(process, cwd=PROJECT_ROOT, check=False).returncode


def _load_remote_jobs() -> list[RemoteJob]:
    try:
        raw_jobs = json.loads(REMOTE_JOBS_FILE.read_text(encoding="utf-8"))
        return [RemoteJob(**item) for item in raw_jobs]
    except FileNotFoundError:
        return []
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        print(f"[RL] 원격 작업 기록을 읽지 못했습니다: {exc}")
        return []


def _save_remote_job(job: RemoteJob) -> None:
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    jobs = [
        existing
        for existing in _load_remote_jobs()
        if not (
            existing.host == job.host
            and existing.project_dir == job.project_dir
            and existing.run_name == job.run_name
        )
    ]
    jobs.insert(0, job)
    temporary = REMOTE_JOBS_FILE.with_suffix(".tmp")
    temporary.write_text(
        json.dumps([asdict(item) for item in jobs], ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temporary.replace(REMOTE_JOBS_FILE)


def _inquirer() -> tuple[Any, Any]:
    try:
        from InquirerPy import inquirer
        from InquirerPy.base.control import Choice
    except ImportError as exc:
        raise RuntimeError(
            "InquirerPy가 필요합니다. `python -m pip install InquirerPy` 후 다시 실행하세요."
        ) from exc
    return inquirer, Choice


def prompt_standing_pose() -> tuple[str, tuple[float, ...]]:
    """Interactively choose the nominal RL body posture."""

    inquirer, Choice = _inquirer()
    selection = inquirer.select(
        message="RL 기본 자세(몸체 높이)를 선택하세요.",
        choices=[
            Choice(
                value="standard",
                name="높은 자세 · Standard (중간 240°, 아래 255°) · 추천",
            ),
            Choice(
                value="sport",
                name="낮은 자세 · Sport (중간 170°, 아래 195°) · 기존 RL",
            ),
            Choice(
                value="custom",
                name="사용자 정의 · 중간/아래 관절 각도 직접 입력",
            ),
        ],
        default="standard",
    ).execute()
    if selection == "standard":
        return "standard", STANDARD_STANDING_DEGREES
    if selection == "sport":
        return "sport", SPORT_STANDING_DEGREES

    middle = float(
        inquirer.number(
            message="중간 관절(ID 7~12) 기준 각도",
            default=240.0,
            min_allowed=0.0,
            max_allowed=360.0,
            float_allowed=True,
        ).execute()
    )
    lower = float(
        inquirer.number(
            message="아래 관절(ID 13~18) 기준 각도",
            default=255.0,
            min_allowed=0.0,
            max_allowed=360.0,
            float_allowed=True,
        ).execute()
    )
    pose = UPPER_STANDING_DEGREES + (middle,) * 6 + (lower,) * 6
    return f"custom(M={middle:g},L={lower:g})", validate_standing_pose(pose)


def _prompt_training_config() -> TrainingConfig:
    inquirer, Choice = _inquirer()
    task = inquirer.select(
        message="무엇을 학습할까요?",
        choices=[Choice(value=item.key, name=item.label) for item in TRAINING_TASKS.values()],
    ).execute()
    curriculum = inquirer.select(
        message="학습 범위(커리큘럼)를 선택하세요.",
        choices=[
            Choice(value="easy", name="easy · 전진부터 학습"),
            Choice(value="medium", name="medium · 전진 + 회전"),
            Choice(value="full", name="full · 전후/좌우 + 회전"),
        ],
        default="easy",
    ).execute()
    terrain = inquirer.select(
        message="어떤 지형에서 학습할까요?",
        choices=[Choice(value=value, name=label) for value, label in TERRAIN_OPTIONS],
        default="flat",
    ).execute()
    standing_pose_name, standing_pose_degrees = prompt_standing_pose()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_name = inquirer.text(
        message="실행 이름을 입력하세요.",
        default=f"{task}_{curriculum}_{timestamp}",
        validate=lambda value: SAFE_NAME.fullmatch(value) is not None,
        invalid_message="영문/숫자로 시작하고 영문, 숫자, ., _, - 만 사용하세요.",
    ).execute()
    timesteps = int(
        inquirer.number(
            message="총 몇 timestep을 학습할까요?",
            default=1_000_000,
            min_allowed=1,
        ).execute()
    )
    num_envs = int(
        inquirer.number(
            message="병렬 환경 개수는 몇 개로 할까요?",
            default=4,
            min_allowed=1,
        ).execute()
    )
    checkpoint_every = int(
        inquirer.number(
            message="몇 timestep마다 체크포인트를 저장할까요?",
            default=min(100_000, timesteps),
            min_allowed=1,
        ).execute()
    )
    keep_checkpoints = int(
        inquirer.number(
            message="최근 체크포인트를 몇 개 보관할까요?",
            default=10,
            min_allowed=1,
        ).execute()
    )
    device = inquirer.select(
        message="학습 장치를 선택하세요.",
        choices=["auto", "cpu", "cuda", "mps"],
        default="auto",
    ).execute()
    return TrainingConfig(
        task=task,
        run_name=run_name,
        curriculum=curriculum,
        timesteps=timesteps,
        num_envs=num_envs,
        checkpoint_every=checkpoint_every,
        keep_checkpoints=keep_checkpoints,
        terrain=terrain,
        device=device,
        standing_pose_name=standing_pose_name,
        standing_pose_degrees=standing_pose_degrees,
    )


def _prompt_remote_settings() -> RemoteSettings:
    inquirer, _ = _inquirer()
    host = inquirer.text(
        message="SSH 호스트를 입력하세요.",
        default=DEFAULT_REMOTE_HOST,
        validate=lambda value: SAFE_SSH_HOST.fullmatch(value) is not None,
        invalid_message="유효한 SSH 별칭 또는 user@hostname을 입력하세요.",
    ).execute()
    project_dir = inquirer.text(
        message="원격 SCONE 프로젝트 경로를 입력하세요.",
        default=DEFAULT_REMOTE_PROJECT,
    ).execute()
    return RemoteSettings(host=host, project_dir=project_dir)


def _manual_remote_job() -> RemoteJob:
    inquirer, Choice = _inquirer()
    settings = _prompt_remote_settings()
    run_name = inquirer.text(
        message="원격 실행 이름을 입력하세요.",
        validate=lambda value: SAFE_NAME.fullmatch(value) is not None,
        invalid_message="영문/숫자로 시작하고 영문, 숫자, ., _, - 만 사용하세요.",
    ).execute()
    terrain = inquirer.select(
        message="이 학습에 사용한 지형을 선택하세요.",
        choices=[Choice(value=value, name=label) for value, label in TERRAIN_OPTIONS],
        default="flat",
    ).execute()
    standing_pose_name, standing_pose_degrees = prompt_standing_pose()
    return RemoteJob(
        host=settings.host,
        project_dir=settings.project_dir,
        port=settings.port,
        run_name=run_name,
        terrain=terrain,
        standing_pose_name=standing_pose_name,
        standing_pose_degrees=standing_pose_degrees,
    )


def _prompt_remote_job(message: str) -> RemoteJob:
    inquirer, Choice = _inquirer()
    jobs = _load_remote_jobs()
    choices = [
        Choice(
            # InquirerPy normalizes dataclass choice values with ``asdict``.
            # Keep the UI value scalar and recover the RemoteJob ourselves.
            value=index,
            name=f"{job.run_name} · {job.host} · {job.created_at or '시간 미상'}",
        )
        for index, job in enumerate(jobs)
    ]
    choices.append(Choice(value=-1, name="기록에 없는 실행 직접 입력"))
    selected = inquirer.select(message=message, choices=choices).execute()
    if selected == -1:
        return _manual_remote_job()
    if not isinstance(selected, int) or not 0 <= selected < len(jobs):
        raise ValueError(f"유효하지 않은 원격 학습 선택값입니다: {selected!r}")
    return jobs[selected]


def _prompt_local_model() -> Path | None:
    inquirer, Choice = _inquirer()
    models = local_model_files()
    if not models:
        print("[RL] 로컬 runs 폴더에 .zip 모델이 없습니다.")
        return None
    return inquirer.select(
        message="어떤 모델을 볼까요?",
        choices=[
            Choice(value=path, name=str(path.relative_to(PROJECT_ROOT)))
            for path in models
        ],
    ).execute()


def _start_training_flow() -> None:
    inquirer, Choice = _inquirer()
    config = _prompt_training_config()
    location = inquirer.select(
        message="어디에서 학습할까요?",
        choices=[
            Choice(value="remote", name=f"SSH 원격 백그라운드 ({DEFAULT_REMOTE_HOST})"),
            Choice(value="local", name="이 컴퓨터에서 실행"),
        ],
        default="remote",
    ).execute()
    print(
        f"\n[RL] {config.task_spec.label} / {config.curriculum} / "
        f"지형 {config.terrain} / 자세 {config.standing_pose_name} / "
        f"{config.timesteps:,} timestep / 실행명 {config.run_name}"
    )
    if not inquirer.confirm(message="이 설정으로 시작할까요?", default=True).execute():
        return

    if location == "local":
        run_local_training(config)
        return

    settings = _prompt_remote_settings()
    sync_code = inquirer.confirm(
        message="실행 전에 현재 로컬 코드를 원격 프로젝트로 동기화할까요?",
        default=True,
    ).execute()
    install_dependencies = inquirer.confirm(
        message=(
            "원격 Python 3.12 .venv 또는 RL 의존성이 없으면 자동으로 준비할까요?"
        ),
        default=True,
    ).execute()
    job = start_remote_training(
        config,
        settings,
        sync_code=sync_code,
        install_missing_dependencies=install_dependencies,
    )
    print(f"\n[RL] 원격 학습을 시작했습니다 (PID {job.pid}).")
    print(f"     로그: {job.host}:{job.project_dir}/{job.relative_run_dir}/train.log")
    print(
        f"     체크포인트: {job.host}:{job.project_dir}/"
        f"{job.relative_run_dir}/checkpoints"
    )


def _environment_check_flow() -> None:
    inquirer, Choice = _inquirer()
    curriculum = inquirer.select(
        message="테스트할 커리큘럼을 선택하세요.",
        choices=["easy", "medium", "full"],
        default="easy",
    ).execute()
    terrain = inquirer.select(
        message="테스트할 지형을 선택하세요.",
        choices=[Choice(value=value, name=label) for value, label in TERRAIN_OPTIONS],
        default="flat",
    ).execute()
    standing_pose_name, standing_pose_degrees = prompt_standing_pose()
    steps = int(
        inquirer.number(
            message="몇 policy step을 검사할까요?",
            default=500,
            min_allowed=1,
        ).execute()
    )
    random_actions = inquirer.confirm(
        message="무작위 residual action도 검사할까요?",
        default=False,
    ).execute()
    result = run_environment_check(
        curriculum=curriculum,
        terrain=terrain,
        steps=steps,
        random_actions=random_actions,
        standing_pose_degrees=standing_pose_degrees,
    )
    if result != 0:
        raise RuntimeError(f"학습 환경 테스트가 exit code {result}로 실패했습니다")


def _view_model_flow() -> None:
    inquirer, Choice = _inquirer()
    checkpoint = _prompt_local_model()
    if checkpoint is None:
        return
    vx = float(inquirer.number(message="전진 속도 vx (m/s)", default=0.25, float_allowed=True).execute())
    vy = float(inquirer.number(message="측면 속도 vy (m/s)", default=0.0, float_allowed=True).execute())
    yaw = float(inquirer.number(message="회전 속도 (rad/s)", default=0.0, float_allowed=True).execute())
    episodes = int(inquirer.number(message="몇 episode를 볼까요?", default=3, min_allowed=1).execute())
    terrain = inquirer.select(
        message="모델을 어떤 지형에서 볼까요?",
        choices=[Choice(value=value, name=label) for value, label in TERRAIN_OPTIONS],
        default="flat",
    ).execute()
    standing_pose_name, standing_pose_degrees = prompt_standing_pose()
    print(f"[RL] 재생 기본 자세: {standing_pose_name}")
    view_local_model(
        checkpoint,
        command=(vx, vy, yaw),
        episodes=episodes,
        terrain=terrain,
        standing_pose_degrees=standing_pose_degrees,
    )


def _pause_remote_flow() -> None:
    inquirer, _ = _inquirer()
    job = _prompt_remote_job("어떤 원격 학습을 일시정지할까요?")
    if not remote_job_is_running(job):
        print(f"\n[RL] {job.run_name} 학습은 이미 중지되어 있습니다.\n")
        return
    if not inquirer.confirm(
        message=(
            f"{job.run_name} 학습을 안전하게 중지하고 이어하기 "
            "체크포인트를 남길까요?"
        ),
        default=True,
    ).execute():
        return

    checkpoint = pause_remote_training(job)
    print("\n[RL] 원격 학습을 일시정지했습니다.")
    print(f"     이어하기 체크포인트: {job.host}:{checkpoint}")
    print("     `원격 학습 이어하기`에서 같은 실행을 계속할 수 있습니다.\n")


def _resume_remote_flow() -> None:
    inquirer, _ = _inquirer()
    job = _prompt_remote_job("어떤 원격 학습을 이어서 진행할까요?")
    if remote_job_is_running(job):
        print(f"\n[RL] {job.run_name} 학습은 이미 실행 중입니다.\n")
        return

    print(
        f"\n[RL] 저장된 설정: {job.curriculum} / 지형 {job.terrain} / "
        f"자세 {job.standing_pose_name} / 병렬 환경 {job.num_envs}개 / "
        f"체크포인트 {job.checkpoint_every:,} step마다"
    )
    if not inquirer.confirm(
        message=(
            "기존 체크포인트와 보상함수·관측 구조가 호환되나요? "
            "바꿨다면 이어하기 대신 원격 초기화 후 새 학습을 사용하세요."
        ),
        default=True,
    ).execute():
        return
    additional_timesteps = int(
        inquirer.number(
            message="추가로 몇 timestep을 학습할까요?",
            default=1_000_000,
            min_allowed=1,
        ).execute()
    )
    sync_code = inquirer.confirm(
        message="이어가기 전에 현재 로컬 코드를 원격 프로젝트로 동기화할까요?",
        default=True,
    ).execute()
    install_dependencies = inquirer.confirm(
        message="원격 Python 3.12/RL 의존성을 확인하고 필요하면 준비할까요?",
        default=True,
    ).execute()
    resumed_job, checkpoint = resume_remote_training(
        job,
        additional_timesteps=additional_timesteps,
        sync_code=sync_code,
        install_missing_dependencies=install_dependencies,
    )
    print(f"\n[RL] 원격 학습을 이어서 시작했습니다 (PID {resumed_job.pid}).")
    print(f"     시작 체크포인트: {resumed_job.host}:{checkpoint}")
    print(f"     추가 학습량: {additional_timesteps:,} timestep")
    print(
        f"     로그: {resumed_job.host}:{resumed_job.project_dir}/"
        f"{resumed_job.relative_run_dir}/train.log\n"
    )


def _reset_remote_flow() -> None:
    inquirer, _ = _inquirer()
    job = _prompt_remote_job("어떤 원격 실행과 체크포인트를 초기화할까요?")
    running = remote_job_is_running(job)
    if running:
        confirmed = inquirer.confirm(
            message=(
                f"{job.run_name} 학습이 실행 중입니다. 학습을 종료하고 "
                "실행 전체를 백업한 뒤 초기화할까요?"
            ),
            default=False,
        ).execute()
    else:
        confirmed = inquirer.confirm(
            message=(
                f"{job.host}:{job.project_dir}/{job.relative_run_dir} 를 "
                "원격 .reset_backup으로 이동할까요?"
            ),
            default=False,
        ).execute()
    if not confirmed:
        return

    typed_name = inquirer.text(
        message=f"확인을 위해 실행 이름 `{job.run_name}`을 입력하세요."
    ).execute()
    if typed_name != job.run_name:
        print("[RL] 실행 이름이 일치하지 않아 초기화를 취소했습니다.")
        return

    backup = reset_remote_run(job, stop_running=running)
    print("\n[RL] 원격 실행을 초기화했습니다.")
    print(f"     기존 데이터 백업: {job.host}:{job.project_dir}/{backup}")
    print(f"     같은 실행명 `{job.run_name}`으로 완전 새 학습을 시작할 수 있습니다.\n")


def main() -> int:
    try:
        inquirer, Choice = _inquirer()
    except RuntimeError as exc:
        print(f"[RL] {exc}", file=sys.stderr)
        return 2

    while True:
        try:
            action = inquirer.select(
                message="SCONE 강화학습",
                choices=[
                    Choice(value="check", name="학습 환경/보상 스모크 테스트"),
                    Choice(value="start", name="새 학습 시작"),
                    Choice(value="status", name="원격 학습 상태와 로그 보기"),
                    Choice(value="pause", name="원격 학습 일시정지"),
                    Choice(value="resume", name="원격 학습 이어하기"),
                    Choice(value="download", name="원격 최신 체크포인트 내려받기"),
                    Choice(value="watch", name="원격 학습을 내려받으며 실시간 보기"),
                    Choice(value="view", name="로컬에 저장된 모델 보기"),
                    Choice(
                        value="reset",
                        name="원격 실행/체크포인트 완전 초기화 (새 학습)",
                    ),
                    Choice(value="quit", name="돌아가기"),
                ],
            ).execute()
            if action == "quit":
                return 0
            if action == "check":
                _environment_check_flow()
            elif action == "start":
                _start_training_flow()
            elif action == "status":
                job = _prompt_remote_job("어떤 원격 학습을 확인할까요?")
                print(f"\n{remote_job_status(job)}\n")
            elif action == "pause":
                _pause_remote_flow()
            elif action == "resume":
                _resume_remote_flow()
            elif action == "download":
                job = _prompt_remote_job("어떤 원격 학습을 내려받을까요?")
                paths = download_remote_artifacts(job)
                print("\n[RL] 내려받기 완료:")
                for path in paths:
                    print(f"  {path}")
            elif action == "watch":
                watch_remote_job(_prompt_remote_job("어떤 원격 학습을 볼까요?"))
            elif action == "view":
                _view_model_flow()
            elif action == "reset":
                _reset_remote_flow()
        except (EOFError, KeyboardInterrupt):
            print("\n[RL] 취소했습니다.")
            return 0
        except (
            OSError,
            RuntimeError,
            ValueError,
            FileNotFoundError,
            subprocess.SubprocessError,
        ) as exc:
            print(f"\n[RL] 작업을 완료하지 못했습니다: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
