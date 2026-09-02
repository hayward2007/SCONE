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

from ..cli_i18n import Language, localize
from ..cli_ui import clear_terminal, display_width, render_panel, show_picker_screen
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
REMOTE_MEMORY_RESERVE_BYTES = 2 * 1024**3
ESTIMATED_ENV_MEMORY_BYTES = 768 * 1024**2

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
REFERENCE_MOTION_OPTIONS = (
    (
        "tripod-gait",
        "tripod-gait · 고전 교대 삼각보 + IK",
    ),
    (
        "scone-gait",
        "scone-gait · 부채꼴 rolling/creep 기준 (실험)",
    ),
    (
        "none",
        "none · 기준 모션 없이 end-to-end (walk-v2 전용)",
    ),
    (
        "hardcoded",
        "하드코딩 모션 · 기존 PPO 학습 기준 (재생 권장)",
    ),
)
REFERENCE_MOTION_ALIASES = {"non_rl": "tripod-gait"}
REFERENCE_MOTION_VALUES = {
    value for value, _ in REFERENCE_MOTION_OPTIONS
} | set(REFERENCE_MOTION_ALIASES)


def normalize_reference_motion(value: str) -> str:
    return REFERENCE_MOTION_ALIASES.get(value, value)


@dataclass(frozen=True)
class TrainingTask:
    key: str
    label: str
    module: str
    checkpoint_prefix: str
    # walk_learn has no end-to-end mode, so "none" is offered only where the
    # trainer implements it.
    reference_motions: tuple[str, ...] = (
        "tripod-gait", "scone-gait", "hardcoded",
    )


TRAINING_TASKS = {
    "walk": TrainingTask(
        key="walk",
        label="걷기 정책 (PPO residual walk)",
        module="src.rl.walk_learn",
        checkpoint_prefix="scone_walk",
    ),
    "walk-v2": TrainingTask(
        key="walk-v2",
        label="걷기 정책 v2 (정규 좌표계 · 좌우 대칭 · 접촉 보상)",
        module="src.rl.walk_v2",
        checkpoint_prefix="scone_walk_v2",
        reference_motions=("tripod-gait", "scone-gait", "hardcoded", "none"),
    ),
}

_ENGLISH_TERRAIN_OPTIONS = (
    ("flat", "Flat ground"),
    ("uneven", "Uneven terrain"),
    ("stairs-1", "Stairs level 1 · 10 cm"),
    ("stairs-2", "Stairs level 2 · 15 cm"),
    ("stairs-3", "Stairs level 3 · 20 cm"),
    ("slope-1", "Slope level 1 · 8°"),
    ("slope-2", "Slope level 2 · 15°"),
    ("slope-3", "Slope level 3 · 25°"),
    ("mixed", "Mixed course"),
)

_ENGLISH_REFERENCE_MOTION_OPTIONS = {
    "tripod-gait": "tripod-gait · alternating tripod + IK",
    "scone-gait": "scone-gait · sector rolling/creep reference (experimental)",
    "none": "none · end-to-end without a reference (walk-v2 only)",
    "hardcoded": "hardcoded · original PPO training reference (replay default)",
}


def _terrain_options(language: Language | str) -> tuple[tuple[str, str], ...]:
    return (
        TERRAIN_OPTIONS
        if Language.parse(language) is Language.KOREA
        else _ENGLISH_TERRAIN_OPTIONS
    )


def _reference_motion_options(
    language: Language | str,
) -> tuple[tuple[str, str], ...]:
    if Language.parse(language) is Language.KOREA:
        return REFERENCE_MOTION_OPTIONS
    return tuple(
        (value, _ENGLISH_REFERENCE_MOTION_OPTIONS[value])
        for value, _label in REFERENCE_MOTION_OPTIONS
    )


def _task_label(task: "TrainingTask", language: Language | str) -> str:
    if Language.parse(language) is Language.KOREA:
        return task.label
    return {
        "walk": "Walking policy · PPO residual walk",
        "walk-v2": "Walking policy v2 · canonical coordinates and contact reward",
    }.get(task.key, task.key)


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
    reference_motion: str = "tripod-gait"
    standing_pose_name: str = "sport"
    standing_pose_degrees: tuple[float, ...] = SPORT_STANDING_DEGREES

    def __post_init__(self) -> None:
        if self.task not in TRAINING_TASKS:
            raise ValueError(f"unknown training task: {self.task}")
        if self.curriculum not in {"easy", "medium", "full"}:
            raise ValueError(f"unknown curriculum: {self.curriculum}")
        if self.reference_motion not in REFERENCE_MOTION_VALUES:
            raise ValueError(f"unknown reference motion: {self.reference_motion}")
        object.__setattr__(
            self,
            "reference_motion",
            normalize_reference_motion(self.reference_motion),
        )
        allowed = TRAINING_TASKS[self.task].reference_motions
        if self.reference_motion not in allowed:
            raise ValueError(
                f"{self.task} does not support reference motion "
                f"{self.reference_motion!r}; choose from {allowed}"
            )
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
class RemoteCapacity:
    """CPU/memory limits used to recommend a real parallel env count."""

    physical_cores: int
    logical_cores: int
    total_memory_bytes: int
    available_memory_bytes: int
    load_average_1m: float
    cpu_limit: int
    memory_limit: int
    recommended_num_envs: int

    def __post_init__(self) -> None:
        for name in (
            "physical_cores",
            "logical_cores",
            "total_memory_bytes",
            "available_memory_bytes",
            "cpu_limit",
            "memory_limit",
            "recommended_num_envs",
        ):
            if getattr(self, name) < 1:
                raise ValueError(f"{name} must be at least 1")
        if self.load_average_1m < 0.0:
            raise ValueError("load_average_1m cannot be negative")

    @property
    def available_memory_gib(self) -> float:
        return self.available_memory_bytes / 1024**3

    @property
    def is_busy(self) -> bool:
        return self.load_average_1m >= self.physical_cores * 0.75


def recommend_num_envs(
    *,
    physical_cores: int,
    logical_cores: int,
    total_memory_bytes: int,
    available_memory_bytes: int,
    load_average_1m: float,
) -> RemoteCapacity:
    """Reserve one physical core and enough RAM for the OS/PPO process."""

    if min(physical_cores, logical_cores) < 1:
        raise ValueError("remote core counts must be positive")
    if total_memory_bytes < 1:
        raise ValueError("remote total memory must be positive")
    if available_memory_bytes < 1:
        available_memory_bytes = max(1, int(total_memory_bytes * 0.75))

    cpu_limit = max(1, physical_cores - 1)
    memory_budget = max(
        ESTIMATED_ENV_MEMORY_BYTES,
        available_memory_bytes - REMOTE_MEMORY_RESERVE_BYTES,
    )
    memory_limit = max(1, memory_budget // ESTIMATED_ENV_MEMORY_BYTES)
    recommended = max(1, min(cpu_limit, memory_limit))
    return RemoteCapacity(
        physical_cores=physical_cores,
        logical_cores=logical_cores,
        total_memory_bytes=total_memory_bytes,
        available_memory_bytes=available_memory_bytes,
        load_average_1m=max(0.0, load_average_1m),
        cpu_limit=cpu_limit,
        memory_limit=memory_limit,
        recommended_num_envs=recommended,
    )


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
    # Records created before this field existed used the hand-authored gait.
    # Keep that compatibility default while new TrainingConfig defaults to
    # the canonical tripod-gait reference explicitly.
    reference_motion: str = "hardcoded"
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
        if self.reference_motion not in REFERENCE_MOTION_VALUES:
            raise ValueError(f"unknown reference motion: {self.reference_motion}")
        object.__setattr__(
            self,
            "reference_motion",
            normalize_reference_motion(self.reference_motion),
        )
        allowed = TRAINING_TASKS[self.task].reference_motions
        if self.reference_motion not in allowed:
            raise ValueError(
                f"{self.task} does not support reference motion "
                f"{self.reference_motion!r}; choose from {allowed}"
            )
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


_REMOTE_CAPACITY_PYTHON = r"""
import json
import os
import platform
import re
import subprocess


def command_output(arguments):
    try:
        return subprocess.check_output(arguments, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


logical = os.cpu_count() or 1
physical = logical
total_memory = 0
available_memory = 0
system = platform.system()

if system == "Darwin":
    physical_text = command_output(["sysctl", "-n", "hw.physicalcpu"])
    total_text = command_output(["sysctl", "-n", "hw.memsize"])
    if physical_text.isdigit():
        physical = int(physical_text)
    if total_text.isdigit():
        total_memory = int(total_text)

    vm_stat = command_output(["vm_stat"])
    page_match = re.search(r"page size of ([0-9]+) bytes", vm_stat)
    page_size = int(page_match.group(1)) if page_match else 4096
    available_pages = 0
    for label in (
        "Pages free",
        "Pages inactive",
        "Pages speculative",
        "Pages purgeable",
    ):
        match = re.search(r"^" + re.escape(label) + r":\s*([0-9]+)", vm_stat, re.M)
        if match:
            available_pages += int(match.group(1))
    available_memory = available_pages * page_size

elif system == "Linux":
    try:
        cpuinfo = open("/proc/cpuinfo", encoding="utf-8").read()
    except OSError:
        cpuinfo = ""
    physical_ids = set()
    for block in cpuinfo.split("\n\n"):
        fields = {}
        for line in block.splitlines():
            if ":" in line:
                key, value = line.split(":", 1)
                fields[key.strip()] = value.strip()
        if "core id" in fields:
            physical_ids.add((fields.get("physical id", "0"), fields["core id"]))
    if physical_ids:
        physical = len(physical_ids)

    try:
        meminfo = open("/proc/meminfo", encoding="utf-8").read()
    except OSError:
        meminfo = ""
    values = {
        key: int(value.split()[0]) * 1024
        for key, value in (
            line.split(":", 1) for line in meminfo.splitlines() if ":" in line
        )
        if value.split()
    }
    total_memory = values.get("MemTotal", 0)
    available_memory = values.get("MemAvailable", 0)

if total_memory <= 0:
    try:
        total_memory = os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES")
    except (OSError, ValueError):
        total_memory = max(available_memory, 8 * 1024**3)
if available_memory <= 0:
    available_memory = int(total_memory * 0.75)
total_memory = max(total_memory, available_memory)

try:
    load_average = os.getloadavg()[0]
except (AttributeError, OSError):
    load_average = 0.0

print(json.dumps({
    "physical_cores": max(1, physical),
    "logical_cores": max(1, logical),
    "total_memory_bytes": total_memory,
    "available_memory_bytes": available_memory,
    "load_average_1m": max(0.0, load_average),
}))
""".strip()


def build_remote_capacity_command() -> str:
    """Build a dependency-free macOS/Linux resource probe."""

    return " ".join(
        [
            'scone_probe_python="$(command -v python3 || command -v python)";',
            '[ -n "$scone_probe_python" ] || exit 41;',
            '"$scone_probe_python" -c',
            shlex.quote(_REMOTE_CAPACITY_PYTHON),
        ]
    )


def inspect_remote_capacity(settings: RemoteSettings) -> RemoteCapacity:
    """Read the SSH host and calculate a conservative max parallel count."""

    result = _run_ssh(
        settings,
        build_remote_capacity_command(),
        timeout=max(15, settings.connect_timeout + 5),
    )
    try:
        raw = json.loads(result.stdout.strip().splitlines()[-1])
        return recommend_num_envs(
            physical_cores=int(raw["physical_cores"]),
            logical_cores=int(raw["logical_cores"]),
            total_memory_bytes=int(raw["total_memory_bytes"]),
            available_memory_bytes=int(raw["available_memory_bytes"]),
            load_average_1m=float(raw["load_average_1m"]),
        )
    except (IndexError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise RuntimeError(
            f"SSH 머신 사양 응답을 해석하지 못했습니다: {result.stdout!r}"
        ) from error


def format_remote_capacity(
    capacity: RemoteCapacity,
    *,
    language: Language | str = Language.KOREA,
) -> str:
    """Return one compact localized explanation for the CLI."""

    bottleneck = (
        "CPU"
        if capacity.cpu_limit <= capacity.memory_limit
        else localize(language, "memory", "메모리")
    )
    message = localize(
        language,
        f"physical {capacity.physical_cores} / logical {capacity.logical_cores} cores, "
        f"{capacity.available_memory_gib:.1f} GiB available, "
        f"1-minute load {capacity.load_average_1m:.1f} → "
        f"recommend {capacity.recommended_num_envs} envs "
        f"(CPU limit {capacity.cpu_limit}, memory limit {capacity.memory_limit}; "
        f"{bottleneck} bound)",
        f"물리 {capacity.physical_cores}코어 / 논리 {capacity.logical_cores}코어, "
        f"사용 가능 메모리 {capacity.available_memory_gib:.1f} GiB, "
        f"1분 load {capacity.load_average_1m:.1f} → "
        f"추천 {capacity.recommended_num_envs}개 "
        f"(CPU 한도 {capacity.cpu_limit}, 메모리 한도 {capacity.memory_limit}; "
        f"{bottleneck} 기준)",
    )
    if capacity.is_busy:
        message += localize(
            language,
            " · Host load is high; verify actual rollout speed.",
            " · 현재 다른 작업 부하가 높으므로 실제 학습 속도를 확인하세요.",
        )
    return message


def build_training_arguments(config: TrainingConfig) -> list[str]:
    """Build the common arguments used by local and remote training."""

    return [
        "--terrain",
        config.terrain,
        "--terrain-seed",
        str(config.terrain_seed),
        "--reference-motion",
        config.reference_motion,
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
            "--only-binary=mujoco -r requirements.txt",
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
    reference_motion: str = "tripod-gait",
    standing_pose_degrees: Sequence[float] = SPORT_STANDING_DEGREES,
) -> int:
    """Run the environment/reward smoke check before committing to training."""

    command = [
        sys.executable,
        "-m",
        "src.rl.walk_learn",
        "--terrain",
        terrain,
        "--reference-motion",
        reference_motion,
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
        reference_motion=config.reference_motion,
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
        reference_motion=job.reference_motion,
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
    reference_motion: str = "hardcoded",
    standing_pose_degrees: Sequence[float] = SPORT_STANDING_DEGREES,
) -> int:
    from .policy_compat import checkpoint_observation_shape, is_v2_checkpoint
    from .remote_watch import _validate_ppo_zip

    checkpoint = checkpoint.expanduser().resolve()
    _validate_ppo_zip(checkpoint)
    module = (
        "src.rl.walk_v2"
        if is_v2_checkpoint(checkpoint_observation_shape(checkpoint))
        else "src.rl.walk_learn"
    )
    executable = shutil.which("mjpython") or sys.executable
    process = [
        executable,
        "-m",
        module,
        "--terrain",
        terrain,
        "--terrain-seed",
        str(terrain_seed),
        "--reference-motion",
        reference_motion,
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
        "--task",
        job.task,
        "--terrain",
        job.terrain,
        "--terrain-seed",
        str(job.terrain_seed),
        "--reference-motion",
        job.reference_motion,
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


def prompt_standing_pose(
    *,
    language: Language | str = Language.ENGLISH,
) -> tuple[str, tuple[float, ...]]:
    """Interactively choose the nominal RL body posture."""

    inquirer, Choice = _inquirer()
    message = localize(
        language,
        "Select the RL standing posture",
        "RL 기본 자세(몸체 높이)를 선택하세요",
    )
    show_picker_screen(
        localize(language, "SCONE / RL POSTURE", "SCONE / RL 기본 자세"),
        message,
        localize(
            language,
            "Use Up/Down, then press Enter; Ctrl-C returns",
            "위/아래로 이동한 뒤 Enter로 선택, Ctrl-C로 돌아가기",
        ),
    )
    selection = inquirer.select(
        message=message,
        choices=[
            Choice(
                value="standard",
                name=localize(
                    language,
                    "- High / Standard (middle 240°, lower 255°) / recommended",
                    "- 높은 자세 / Standard (중간 240°, 아래 255°) / 추천",
                ),
            ),
            Choice(
                value="sport",
                name=localize(
                    language,
                    "- Low / Sport (middle 170°, lower 195°) / legacy RL",
                    "- 낮은 자세 / Sport (중간 170°, 아래 195°) / 기존 RL",
                ),
            ),
            Choice(
                value="custom",
                name=localize(
                    language,
                    "- Custom / enter middle/lower joint angles",
                    "- 사용자 정의 / 중간/아래 관절 각도 직접 입력",
                ),
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
            message=localize(language, "Middle joint angle · IDs 7–12", "중간 관절(ID 7~12) 기준 각도"),
            default=240.0,
            min_allowed=0.0,
            max_allowed=360.0,
            float_allowed=True,
        ).execute()
    )
    lower = float(
        inquirer.number(
            message=localize(language, "Lower joint angle · IDs 13–18", "아래 관절(ID 13~18) 기준 각도"),
            default=255.0,
            min_allowed=0.0,
            max_allowed=360.0,
            float_allowed=True,
        ).execute()
    )
    pose = UPPER_STANDING_DEGREES + (middle,) * 6 + (lower,) * 6
    return f"custom(M={middle:g},L={lower:g})", validate_standing_pose(pose)


def prompt_reference_motion(
    *,
    default: str = "tripod-gait",
    allowed: tuple[str, ...] | None = None,
    language: Language | str = Language.ENGLISH,
) -> str:
    """Choose the baseline that the residual policy will correct."""

    default = normalize_reference_motion(default)
    options = [
        (value, label)
        for value, label in _reference_motion_options(language)
        if allowed is None or value in allowed
    ]
    if not options:
        raise ValueError("no reference motion is available for this task")
    if default not in {value for value, _ in options}:
        default = options[0][0]
    inquirer, Choice = _inquirer()
    message = localize(
        language,
        "Select the residual RL reference motion",
        "Residual RL의 기준 모션을 선택하세요",
    )
    show_picker_screen(
        localize(
            language,
            "SCONE / RL REFERENCE MOTION",
            "SCONE / RL 기준 모션",
        ),
        message,
        localize(
            language,
            "Use Up/Down, then press Enter; Ctrl-C returns",
            "위/아래로 이동한 뒤 Enter로 선택, Ctrl-C로 돌아가기",
        ),
    )
    return inquirer.select(
        message=message,
        choices=[Choice(value=value, name=f"- {label}") for value, label in options],
        default=default,
    ).execute()


def _prompt_training_config(
    *,
    recommended_num_envs: int = 4,
    num_envs_hint: str | None = None,
    language: Language | str = Language.ENGLISH,
) -> TrainingConfig:
    if recommended_num_envs < 1:
        raise ValueError("recommended_num_envs must be at least 1")
    inquirer, Choice = _inquirer()
    task = inquirer.select(
        message=localize(language, "Select a training task", "무엇을 학습할까요"),
        choices=[
            Choice(value=item.key, name=_task_label(item, language))
            for item in TRAINING_TASKS.values()
        ],
    ).execute()
    reference_motion = prompt_reference_motion(
        default="tripod-gait",
        allowed=TRAINING_TASKS[task].reference_motions,
        language=language,
    )
    curriculum = inquirer.select(
        message=localize(language, "Select a training curriculum", "학습 범위(커리큘럼)를 선택하세요"),
        choices=[
            Choice(value="easy", name=localize(language, "easy · forward first", "easy · 전진부터 학습")),
            Choice(value="medium", name=localize(language, "medium · forward + turning", "medium · 전진 + 회전")),
            Choice(value="full", name=localize(language, "full · planar motion + turning", "full · 전후/좌우 + 회전")),
        ],
        default="easy",
    ).execute()
    terrain = inquirer.select(
        message=localize(language, "Select training terrain", "어떤 지형에서 학습할까요"),
        choices=[Choice(value=value, name=label) for value, label in _terrain_options(language)],
        default="flat",
    ).execute()
    standing_pose_name, standing_pose_degrees = prompt_standing_pose(language=language)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_name = inquirer.text(
        message=localize(language, "Enter a run name", "실행 이름을 입력하세요"),
        default=f"{task}_{curriculum}_{timestamp}",
        validate=lambda value: SAFE_NAME.fullmatch(value) is not None,
        invalid_message=localize(
            language,
            "Start with a letter or digit; use only letters, digits, ., _, and -.",
            "영문/숫자로 시작하고 영문, 숫자, ., _, - 만 사용하세요.",
        ),
    ).execute()
    timesteps = int(
        inquirer.number(
            message=localize(language, "Total training timesteps", "총 몇 timestep을 학습할까요"),
            default=1_000_000,
            min_allowed=1,
        ).execute()
    )
    num_envs = int(
        inquirer.number(
            message=(
                localize(language, "Number of parallel environments", "병렬 환경 개수는 몇 개로 할까요")
                if num_envs_hint is None
                else localize(
                    language,
                    f"Number of parallel environments ({num_envs_hint})",
                    f"병렬 환경 개수는 몇 개로 할까요? ({num_envs_hint})",
                )
            ),
            default=recommended_num_envs,
            min_allowed=1,
        ).execute()
    )
    checkpoint_every = int(
        inquirer.number(
            message=localize(language, "Checkpoint interval in timesteps", "몇 timestep마다 체크포인트를 저장할까요"),
            default=min(100_000, timesteps),
            min_allowed=1,
        ).execute()
    )
    keep_checkpoints = int(
        inquirer.number(
            message=localize(language, "Number of recent checkpoints to keep", "최근 체크포인트를 몇 개 보관할까요"),
            default=10,
            min_allowed=1,
        ).execute()
    )
    device = inquirer.select(
        message=localize(language, "Select a training device", "학습 장치를 선택하세요"),
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
        reference_motion=reference_motion,
        standing_pose_name=standing_pose_name,
        standing_pose_degrees=standing_pose_degrees,
    )


def _prompt_remote_settings(
    *,
    language: Language | str = Language.ENGLISH,
) -> RemoteSettings:
    inquirer, _ = _inquirer()
    host = inquirer.text(
        message=localize(language, "SSH host", "SSH 호스트를 입력하세요"),
        default=DEFAULT_REMOTE_HOST,
        validate=lambda value: SAFE_SSH_HOST.fullmatch(value) is not None,
        invalid_message=localize(
            language,
            "Enter a valid SSH alias or user@hostname.",
            "유효한 SSH 별칭 또는 user@hostname을 입력하세요.",
        ),
    ).execute()
    project_dir = inquirer.text(
        message=localize(language, "Remote SCONE project path", "원격 SCONE 프로젝트 경로를 입력하세요"),
        default=DEFAULT_REMOTE_PROJECT,
    ).execute()
    return RemoteSettings(host=host, project_dir=project_dir)


def _manual_remote_job(
    *,
    language: Language | str = Language.ENGLISH,
) -> RemoteJob:
    inquirer, Choice = _inquirer()
    settings = _prompt_remote_settings(language=language)
    task = inquirer.select(
        message=localize(language, "Training task used by this run", "이 실행에서 사용한 학습 작업을 선택하세요"),
        choices=[
            Choice(value=item.key, name=_task_label(item, language))
            for item in TRAINING_TASKS.values()
        ],
        default="walk",
    ).execute()
    run_name = inquirer.text(
        message=localize(language, "Remote run name", "원격 실행 이름을 입력하세요"),
        validate=lambda value: SAFE_NAME.fullmatch(value) is not None,
        invalid_message=localize(
            language,
            "Start with a letter or digit; use only letters, digits, ., _, and -.",
            "영문/숫자로 시작하고 영문, 숫자, ., _, - 만 사용하세요.",
        ),
    ).execute()
    terrain = inquirer.select(
        message=localize(language, "Terrain used by this run", "이 학습에 사용한 지형을 선택하세요"),
        choices=[Choice(value=value, name=label) for value, label in _terrain_options(language)],
        default="flat",
    ).execute()
    reference_motion = prompt_reference_motion(
        default="tripod-gait",
        allowed=TRAINING_TASKS[task].reference_motions,
        language=language,
    )
    standing_pose_name, standing_pose_degrees = prompt_standing_pose(language=language)
    return RemoteJob(
        host=settings.host,
        project_dir=settings.project_dir,
        port=settings.port,
        run_name=run_name,
        task=task,
        terrain=terrain,
        reference_motion=reference_motion,
        standing_pose_name=standing_pose_name,
        standing_pose_degrees=standing_pose_degrees,
    )


def _prompt_remote_job(
    message: str,
    *,
    language: Language | str = Language.ENGLISH,
) -> RemoteJob:
    inquirer, Choice = _inquirer()
    jobs = _load_remote_jobs()
    choices = [
        Choice(
            # InquirerPy normalizes dataclass choice values with ``asdict``.
            # Keep the UI value scalar and recover the RemoteJob ourselves.
            value=index,
            name=(
                f"{job.run_name} · {job.host} · "
                f"{job.created_at or localize(language, 'time unknown', '시간 미상')}"
            ),
        )
        for index, job in enumerate(jobs)
    ]
    choices.append(Choice(
        value=-1,
        name=localize(language, "Enter a run not listed here", "기록에 없는 실행 직접 입력"),
    ))
    selected = inquirer.select(message=message, choices=choices).execute()
    if selected == -1:
        return _manual_remote_job(language=language)
    if not isinstance(selected, int) or not 0 <= selected < len(jobs):
        raise ValueError(localize(
            language,
            f"Invalid remote run selection: {selected!r}",
            f"유효하지 않은 원격 학습 선택값입니다: {selected!r}",
        ))
    return jobs[selected]


def _prompt_local_model(
    *,
    language: Language | str = Language.ENGLISH,
) -> Path | None:
    inquirer, Choice = _inquirer()
    models = local_model_files()
    if not models:
        print(localize(
            language,
            "[RL] No .zip model was found in the local runs directory.",
            "[RL] 로컬 runs 폴더에 .zip 모델이 없습니다.",
        ))
        return None
    return inquirer.select(
        message=localize(language, "Select a local model", "어떤 모델을 볼까요"),
        choices=[
            Choice(value=path, name=str(path.relative_to(PROJECT_ROOT)))
            for path in models
        ],
    ).execute()


def _start_training_flow(
    *,
    language: Language | str = Language.ENGLISH,
) -> None:
    inquirer, Choice = _inquirer()
    location = inquirer.select(
        message=localize(language, "Select where to train", "어디에서 학습할까요"),
        choices=[
            Choice(
                value="remote",
                name=localize(
                    language,
                    f"SSH background job · {DEFAULT_REMOTE_HOST}",
                    f"SSH 원격 백그라운드 · {DEFAULT_REMOTE_HOST}",
                ),
            ),
            Choice(value="local", name=localize(language, "This computer", "이 컴퓨터에서 실행")),
        ],
        default="remote",
    ).execute()
    settings: RemoteSettings | None = None
    recommended_num_envs = 4
    num_envs_hint: str | None = None
    if location == "remote":
        settings = _prompt_remote_settings(language=language)
        print(localize(
            language,
            f"\n[RL] Inspecting parallel-training capacity on {settings.host}...",
            f"\n[RL] {settings.host}의 병렬 학습 자원을 확인합니다...",
        ))
        try:
            capacity = inspect_remote_capacity(settings)
        except (OSError, RuntimeError, subprocess.SubprocessError) as error:
            print(localize(
                language,
                f"[RL] Automatic recommendation failed: {error}\n"
                "     Starting from 4 environments; you can edit this after SSH connects.",
                f"[RL] 자동 추천을 계산하지 못했습니다: {error}\n"
                "     기본값 4개를 제안하며, SSH 연결 후 직접 바꿀 수 있습니다.",
            ))
        else:
            recommended_num_envs = capacity.recommended_num_envs
            num_envs_hint = localize(
                language,
                f"SSH recommends {recommended_num_envs}",
                f"SSH 추천 {recommended_num_envs}개",
            )
            print(f"[RL] {format_remote_capacity(capacity, language=language)}")
            print(localize(
                language,
                "     Reserves one physical core and 2 GiB for the OS/PPO; editable.\n",
                "     OS/PPO용 물리 코어 1개와 메모리 2 GiB를 남긴 "
                "출발값이며 직접 수정할 수 있습니다.\n",
            ))

    config = _prompt_training_config(
        recommended_num_envs=recommended_num_envs,
        num_envs_hint=num_envs_hint,
        language=language,
    )
    print(localize(
        language,
        f"\n[RL] {_task_label(config.task_spec, language)} / {config.curriculum} / "
        f"reference {config.reference_motion} / terrain {config.terrain} / "
        f"stance {config.standing_pose_name} / "
        f"{config.timesteps:,} timesteps / run {config.run_name}",
        f"\n[RL] {config.task_spec.label} / {config.curriculum} / "
        f"기준 {config.reference_motion} / 지형 {config.terrain} / "
        f"자세 {config.standing_pose_name} / "
        f"{config.timesteps:,} timestep / 실행명 {config.run_name}",
    ))
    if not inquirer.confirm(
        message=localize(language, "Start with these settings?", "이 설정으로 시작할까요"),
        default=True,
    ).execute():
        return

    if location == "local":
        run_local_training(config)
        return

    if settings is None:
        raise RuntimeError(localize(language, "Remote SSH settings are missing", "원격 학습 SSH 설정이 없습니다"))
    sync_code = inquirer.confirm(
        message=localize(
            language,
            "Sync the current local code to the remote project before launch?",
            "실행 전에 현재 로컬 코드를 원격 프로젝트로 동기화할까요",
        ),
        default=True,
    ).execute()
    install_dependencies = inquirer.confirm(
        message=(
            localize(
                language,
                "Prepare Python 3.12, .venv, and missing RL dependencies remotely?",
                "원격 Python 3.12 .venv 또는 RL 의존성이 없으면 자동으로 준비할까요",
            )
        ),
        default=True,
    ).execute()
    job = start_remote_training(
        config,
        settings,
        sync_code=sync_code,
        install_missing_dependencies=install_dependencies,
    )
    print(localize(
        language,
        f"\n[RL] Remote training started (PID {job.pid}).",
        f"\n[RL] 원격 학습을 시작했습니다 (PID {job.pid}).",
    ))
    print(localize(
        language,
        f"     Log: {job.host}:{job.project_dir}/{job.relative_run_dir}/train.log",
        f"     로그: {job.host}:{job.project_dir}/{job.relative_run_dir}/train.log",
    ))
    print(localize(
        language,
        f"     Checkpoints: {job.host}:{job.project_dir}/{job.relative_run_dir}/checkpoints",
        f"     체크포인트: {job.host}:{job.project_dir}/{job.relative_run_dir}/checkpoints",
    ))


def _environment_check_flow(
    *,
    language: Language | str = Language.ENGLISH,
) -> None:
    inquirer, Choice = _inquirer()
    # ``run_environment_check`` currently exercises the original walk
    # environment. Do not present walk-v2 as an option until it has its own
    # check runner.
    task = "walk"
    reference_motion = prompt_reference_motion(
        default="tripod-gait",
        allowed=TRAINING_TASKS[task].reference_motions,
        language=language,
    )
    curriculum = inquirer.select(
        message=localize(language, "Select a test curriculum", "테스트할 커리큘럼을 선택하세요"),
        choices=["easy", "medium", "full"],
        default="easy",
    ).execute()
    terrain = inquirer.select(
        message=localize(language, "Select test terrain", "테스트할 지형을 선택하세요"),
        choices=[Choice(value=value, name=label) for value, label in _terrain_options(language)],
        default="flat",
    ).execute()
    standing_pose_name, standing_pose_degrees = prompt_standing_pose(language=language)
    print(localize(
        language,
        f"[RL] Environment-check stance: {standing_pose_name}",
        f"[RL] 환경 테스트 기본 자세: {standing_pose_name}",
    ))
    steps = int(
        inquirer.number(
            message=localize(language, "Number of policy steps to test", "몇 policy step을 검사할까요"),
            default=500,
            min_allowed=1,
        ).execute()
    )
    random_actions = inquirer.confirm(
        message=localize(language, "Also test random residual actions?", "무작위 residual action도 검사할까요"),
        default=False,
    ).execute()
    result = run_environment_check(
        curriculum=curriculum,
        terrain=terrain,
        steps=steps,
        random_actions=random_actions,
        reference_motion=reference_motion,
        standing_pose_degrees=standing_pose_degrees,
    )
    if result != 0:
        raise RuntimeError(localize(
            language,
            f"Environment check failed with exit code {result}",
            f"학습 환경 테스트가 exit code {result}로 실패했습니다",
        ))


def _view_model_flow(
    *,
    language: Language | str = Language.ENGLISH,
) -> None:
    inquirer, Choice = _inquirer()
    checkpoint = _prompt_local_model(language=language)
    if checkpoint is None:
        return
    reference_motion = prompt_reference_motion(default="hardcoded", language=language)
    vx = float(inquirer.number(message=localize(language, "Forward speed vx (m/s)", "전진 속도 vx (m/s)"), default=0.25, float_allowed=True).execute())
    vy = float(inquirer.number(message=localize(language, "Lateral speed vy (m/s)", "측면 속도 vy (m/s)"), default=0.0, float_allowed=True).execute())
    yaw = float(inquirer.number(message=localize(language, "Yaw rate (rad/s)", "회전 속도 (rad/s)"), default=0.0, float_allowed=True).execute())
    episodes = int(inquirer.number(message=localize(language, "Number of episodes", "몇 episode를 볼까요"), default=3, min_allowed=1).execute())
    terrain = inquirer.select(
        message=localize(language, "Select replay terrain", "모델을 어떤 지형에서 볼까요"),
        choices=[Choice(value=value, name=label) for value, label in _terrain_options(language)],
        default="flat",
    ).execute()
    standing_pose_name, standing_pose_degrees = prompt_standing_pose(language=language)
    print(localize(
        language,
        f"[RL] Replay stance: {standing_pose_name}",
        f"[RL] 재생 기본 자세: {standing_pose_name}",
    ))
    view_local_model(
        checkpoint,
        command=(vx, vy, yaw),
        episodes=episodes,
        terrain=terrain,
        reference_motion=reference_motion,
        standing_pose_degrees=standing_pose_degrees,
    )


def _pause_remote_flow(
    *,
    language: Language | str = Language.ENGLISH,
) -> None:
    inquirer, _ = _inquirer()
    job = _prompt_remote_job(
        localize(language, "Select a remote run to pause", "어떤 원격 학습을 일시정지할까요"),
        language=language,
    )
    if not remote_job_is_running(job):
        print(localize(
            language,
            f"\n[RL] {job.run_name} is already stopped.\n",
            f"\n[RL] {job.run_name} 학습은 이미 중지되어 있습니다.\n",
        ))
        return
    if not inquirer.confirm(
        message=(
            localize(
                language,
                f"Pause {job.run_name} safely and save a resume checkpoint?",
                f"{job.run_name} 학습을 안전하게 중지하고 이어하기 "
                "체크포인트를 남길까요?",
            )
        ),
        default=True,
    ).execute():
        return

    checkpoint = pause_remote_training(job)
    print(localize(language, "\n[RL] Remote training paused.", "\n[RL] 원격 학습을 일시정지했습니다."))
    print(localize(language, f"     Resume checkpoint: {job.host}:{checkpoint}", f"     이어하기 체크포인트: {job.host}:{checkpoint}"))
    print(localize(language, "     Use Resume remote training to continue this run.\n", "     `원격 학습 이어하기`에서 같은 실행을 계속할 수 있습니다.\n"))


def _resume_remote_flow(
    *,
    language: Language | str = Language.ENGLISH,
) -> None:
    inquirer, _ = _inquirer()
    job = _prompt_remote_job(
        localize(language, "Select a remote run to resume", "어떤 원격 학습을 이어서 진행할까요"),
        language=language,
    )
    if remote_job_is_running(job):
        print(localize(language, f"\n[RL] {job.run_name} is already running.\n", f"\n[RL] {job.run_name} 학습은 이미 실행 중입니다.\n"))
        return

    print(localize(
        language,
        f"\n[RL] Saved settings: {job.curriculum} / reference {job.reference_motion} / "
        f"terrain {job.terrain} / stance {job.standing_pose_name} / "
        f"{job.num_envs} envs / checkpoint every {job.checkpoint_every:,} steps",
        f"\n[RL] 저장된 설정: {job.curriculum} / 기준 {job.reference_motion} / "
        f"지형 {job.terrain} / 자세 {job.standing_pose_name} / "
        f"병렬 환경 {job.num_envs}개 / 체크포인트 {job.checkpoint_every:,} step마다",
    ))
    if not inquirer.confirm(
        message=(
            localize(
                language,
                "Is the checkpoint compatible with the current reward and observation? "
                "If not, reset and start a new run instead of resuming.",
                "기존 체크포인트와 보상함수·관측 구조가 호환되나요? "
                "바꿨다면 이어하기 대신 원격 초기화 후 새 학습을 사용하세요.",
            )
        ),
        default=True,
    ).execute():
        return
    additional_timesteps = int(
        inquirer.number(
            message=localize(language, "Additional training timesteps", "추가로 몇 timestep을 학습할까요"),
            default=1_000_000,
            min_allowed=1,
        ).execute()
    )
    sync_code = inquirer.confirm(
        message=localize(language, "Sync local code before resuming?", "이어가기 전에 현재 로컬 코드를 원격 프로젝트로 동기화할까요"),
        default=True,
    ).execute()
    install_dependencies = inquirer.confirm(
        message=localize(language, "Check and prepare remote Python 3.12/RL dependencies?", "원격 Python 3.12/RL 의존성을 확인하고 필요하면 준비할까요"),
        default=True,
    ).execute()
    resumed_job, checkpoint = resume_remote_training(
        job,
        additional_timesteps=additional_timesteps,
        sync_code=sync_code,
        install_missing_dependencies=install_dependencies,
    )
    print(localize(language, f"\n[RL] Remote training resumed (PID {resumed_job.pid}).", f"\n[RL] 원격 학습을 이어서 시작했습니다 (PID {resumed_job.pid})."))
    print(localize(language, f"     Starting checkpoint: {resumed_job.host}:{checkpoint}", f"     시작 체크포인트: {resumed_job.host}:{checkpoint}"))
    print(localize(language, f"     Additional timesteps: {additional_timesteps:,}", f"     추가 학습량: {additional_timesteps:,} timestep"))
    print(localize(language, f"     Log: {resumed_job.host}:{resumed_job.project_dir}/{resumed_job.relative_run_dir}/train.log\n", f"     로그: {resumed_job.host}:{resumed_job.project_dir}/{resumed_job.relative_run_dir}/train.log\n"))


def _reset_remote_flow(
    *,
    language: Language | str = Language.ENGLISH,
) -> None:
    inquirer, _ = _inquirer()
    job = _prompt_remote_job(
        localize(language, "Select a remote run to archive and reset", "어떤 원격 실행과 체크포인트를 초기화할까요"),
        language=language,
    )
    running = remote_job_is_running(job)
    if running:
        confirmed = inquirer.confirm(
            message=(
                localize(
                    language,
                    f"{job.run_name} is running. Stop it, archive the entire run, and reset?",
                    f"{job.run_name} 학습이 실행 중입니다. 학습을 종료하고 "
                    "실행 전체를 백업한 뒤 초기화할까요?",
                )
            ),
            default=False,
        ).execute()
    else:
        confirmed = inquirer.confirm(
            message=(
                localize(
                    language,
                    f"Move {job.host}:{job.project_dir}/{job.relative_run_dir} "
                    "to remote .reset_backup?",
                    f"{job.host}:{job.project_dir}/{job.relative_run_dir} 를 "
                    "원격 .reset_backup으로 이동할까요?",
                )
            ),
            default=False,
        ).execute()
    if not confirmed:
        return

    typed_name = inquirer.text(
        message=localize(
            language,
            f"Type the run name `{job.run_name}` to confirm",
            f"확인을 위해 실행 이름 `{job.run_name}`을 입력하세요",
        )
    ).execute()
    if typed_name != job.run_name:
        print(localize(language, "[RL] Run name did not match; reset cancelled.", "[RL] 실행 이름이 일치하지 않아 초기화를 취소했습니다."))
        return

    backup = reset_remote_run(job, stop_running=running)
    print(localize(language, "\n[RL] Remote run archived and reset.", "\n[RL] 원격 실행을 초기화했습니다."))
    print(localize(language, f"     Archive: {job.host}:{job.project_dir}/{backup}", f"     기존 데이터 백업: {job.host}:{job.project_dir}/{backup}"))
    print(localize(language, f"     A fresh run may now reuse `{job.run_name}`.\n", f"     같은 실행명 `{job.run_name}`으로 완전 새 학습을 시작할 수 있습니다.\n"))



def _menu_header(
    language: Language | str = Language.ENGLISH,
) -> str:
    """One screenful of context so the menu is not a blind list of verbs."""

    remote_lines = ["[ REMOTE RUNS ]"]
    jobs = _load_remote_jobs()
    if not jobs:
        remote_lines.append(localize(
            language,
            "- No remote runs registered; use Start new training to create one",
            "- 등록된 원격 학습 없음; 새 학습 시작에서 생성할 수 있음",
        ))
    else:
        remote_lines.append(localize(
            language,
            f"- Registered remote runs: {len(jobs)}",
            f"- 등록된 원격 학습: {len(jobs)}개",
        ))
        for job in jobs[-6:]:
            task = TRAINING_TASKS.get(job.task)
            label = _task_label(task, language) if task is not None else job.task
            created = (job.created_at or "")[:16].replace("T", " ")
            remote_lines.append(
                f"- {job.run_name} / {label} / {job.host} / {created or '-'}"
            )
        if len(jobs) > 6:
            remote_lines.append(
                localize(
                    language,
                    f"- ... and {len(jobs) - 6} more",
                    f"- ... 외 {len(jobs) - 6}개",
                )
            )
    local = sorted(
        (path for path in RUNS_DIR.glob("*") if path.is_dir()),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    ) if RUNS_DIR.exists() else []
    local_lines = ["[ LOCAL RUNS ]"]
    if local:
        names = ", ".join(path.name for path in local[:4])
        more = (
            localize(language, f" and {len(local) - 4} more", f" 외 {len(local) - 4}개")
            if len(local) > 4
            else ""
        )
        local_lines.append(
            localize(
                language,
                f"- runs/: {names}{more}",
                f"- runs/: {names}{more}",
            )
        )
    else:
        local_lines.append(localize(language, "- No local runs", "- 로컬 실행 없음"))
    return render_panel(
        localize(
            language,
            "SCONE / REINFORCEMENT LEARNING",
            "SCONE / 강화학습",
        ),
        (tuple(remote_lines), tuple(local_lines)),
    )


def _menu_separator(
    Separator: Any,
    english: str,
    korean: str,
    language: Language | str,
) -> Any:
    label = localize(language, english, korean)
    prefix = f"-- {label} "
    return Separator(f"{prefix}{'-' * max(0, 58 - display_width(prefix))}")


def _main_menu_choices(
    Choice: Any,
    Separator: Any,
    has_remote: bool,
    *,
    language: Language | str = Language.ENGLISH,
) -> list[Any]:
    """Group the actions and hide the ones that cannot run yet."""

    choices: list[Any] = [
        _menu_separator(Separator, "VALIDATE", "준비", language),
        Choice(value="check", name=localize(language, "- Environment/reward smoke test", "- 학습 환경/보상 스모크 테스트")),
        _menu_separator(Separator, "TRAIN", "학습", language),
        Choice(value="start", name=localize(language, "- Start new training", "- 새 학습 시작")),
    ]
    if has_remote:
        choices += [
            _menu_separator(Separator, "REMOTE", "원격 관리", language),
            Choice(value="status", name=localize(language, "- View remote status and logs", "- 원격 학습 상태와 로그 보기")),
            Choice(value="pause", name=localize(language, "- Pause remote training safely", "- 원격 학습 일시정지")),
            Choice(value="resume", name=localize(language, "- Resume remote training", "- 원격 학습 이어하기")),
            Choice(value="download", name=localize(language, "- Download latest remote checkpoint", "- 원격 최신 체크포인트 내려받기")),
            Choice(value="watch", name=localize(language, "- Mirror and watch remote training", "- 원격 학습을 내려받으며 실시간 보기")),
            _menu_separator(Separator, "RESET", "정리", language),
            Choice(value="reset", name=localize(language, "- Archive and reset a remote run", "- 원격 실행/체크포인트 완전 초기화")),
        ]
    choices += [
        _menu_separator(Separator, "REPLAY", "보기", language),
        Choice(value="view", name=localize(language, "- Replay a local model", "- 로컬에 저장된 모델 보기")),
        _menu_separator(Separator, "EXIT", "종료", language),
        Choice(value="quit", name=localize(language, "- Back", "- 돌아가기")),
    ]
    return choices


def main(
    *,
    language: Language | str = Language.ENGLISH,
) -> int:
    try:
        inquirer, Choice = _inquirer()
        from InquirerPy.separator import Separator
    except (RuntimeError, ImportError) as exc:
        print(f"[RL] {exc}", file=sys.stderr)
        return 2

    while True:
        try:
            has_remote = bool(_load_remote_jobs())
            clear_terminal()
            print(_menu_header(language))
            action = inquirer.select(
                message=localize(language, "Choose an RL activity", "무엇을 할까요"),
                choices=_main_menu_choices(
                    Choice,
                    Separator,
                    has_remote,
                    language=language,
                ),
                pointer="\u276f",
                instruction=localize(
                    language,
                    "(Up/Down move, Enter select, Ctrl-C returns)",
                    "(위/아래 이동, Enter 선택, Ctrl-C 돌아가기)",
                ),
            ).execute()
            if action == "quit":
                return 0
            if action == "check":
                _environment_check_flow(language=language)
            elif action == "start":
                _start_training_flow(language=language)
            elif action == "status":
                job = _prompt_remote_job(
                    localize(language, "Select a remote run to inspect", "어떤 원격 학습을 확인할까요"),
                    language=language,
                )
                print(f"\n{remote_job_status(job)}\n")
            elif action == "pause":
                _pause_remote_flow(language=language)
            elif action == "resume":
                _resume_remote_flow(language=language)
            elif action == "download":
                job = _prompt_remote_job(
                    localize(language, "Select a remote run to download", "어떤 원격 학습을 내려받을까요"),
                    language=language,
                )
                paths = download_remote_artifacts(job)
                print(localize(language, "\n[RL] Download complete:", "\n[RL] 내려받기 완료:"))
                for path in paths:
                    print(f"  {path}")
            elif action == "watch":
                watch_remote_job(_prompt_remote_job(
                    localize(language, "Select a remote run to watch", "어떤 원격 학습을 볼까요"),
                    language=language,
                ))
            elif action == "view":
                _view_model_flow(language=language)
            elif action == "reset":
                _reset_remote_flow(language=language)
            inquirer.text(
                message=localize(
                    language,
                    "Press Enter to return to the RL menu",
                    "Enter를 눌러 강화학습 메뉴로 돌아가기",
                ),
                default="",
            ).execute()
        except (EOFError, KeyboardInterrupt):
            print(localize(language, "\n[RL] Cancelled.", "\n[RL] 취소했습니다."))
            return 0
        except (
            OSError,
            RuntimeError,
            ValueError,
            FileNotFoundError,
            subprocess.SubprocessError,
        ) as exc:
            print(localize(
                language,
                f"\n[RL] Operation failed: {exc}\n",
                f"\n[RL] 작업을 완료하지 못했습니다: {exc}\n",
            ))
            try:
                inquirer.text(
                    message=localize(
                        language,
                        "Press Enter to return to the RL menu",
                        "Enter를 눌러 강화학습 메뉴로 돌아가기",
                    ),
                    default="",
                ).execute()
            except (EOFError, KeyboardInterrupt):
                return 0


if __name__ == "__main__":
    raise SystemExit(main())
