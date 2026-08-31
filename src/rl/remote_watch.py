"""Watch SCONE PPO checkpoints over SSH and replay the newest policy locally.

The remote trainer stays headless. This program polls its checkpoint directory,
downloads a complete PPO zip through the system SSH client, and hot-swaps the
policy used by a local MuJoCo environment.

Example::

    mjpython -m src.rl.remote_watch \
        --host ssh.hayward.kim \
        --checkpoint-dir '~/Developer/SCONE/runs/scone_walk_easy/checkpoints' \
        --command 0.25 0.0 0.0
"""

from __future__ import annotations

import argparse
import os
import queue
import re
import shlex
import shutil
import subprocess
import threading
import time
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np
from stable_baselines3 import PPO

from .policy_compat import (
    LEGACY_OBSERVATION_SHAPE,
    load_compatible_policy as _load_policy,
    observation_for_policy as _observation_for_policy,
)
from .stance import SPORT_STANDING_DEGREES
from .walk_learn import (
    CURRICULUM_RANGES,
    DEFAULT_MODEL_PATH,
    NeutralResidualGate,
    REFERENCE_MOTION_CHOICES,
    SconeWalkEnv,
)
from src.simulation.terrain import TERRAIN_CHOICES, TerrainType


CHECKPOINT_NAME = re.compile(r"^(?P<prefix>.+)_(?P<steps>[0-9]+)_steps\.zip$")
SAFE_SSH_HOST = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.@-]*$")
SAFE_PREFIX = re.compile(r"^[A-Za-z0-9_.-]+$")

@dataclass(frozen=True)
class CheckpointCandidate:
    source_path: str
    step: int


class CheckpointSource(Protocol):
    label: str

    def latest(self, prefix: str) -> CheckpointCandidate | None:
        """Return the newest available checkpoint, or None while waiting."""

    def fetch(self, candidate: CheckpointCandidate, destination: Path) -> None:
        """Copy a checkpoint to destination, raising on failure."""


def _checkpoint_step(path: str, prefix: str) -> int | None:
    match = CHECKPOINT_NAME.match(Path(path).name)
    if match is None or match.group("prefix") != prefix:
        return None
    return int(match.group("steps"))


def _validate_ppo_zip(path: Path) -> None:
    try:
        with zipfile.ZipFile(path) as archive:
            corrupt_member = archive.testzip()
    except (OSError, zipfile.BadZipFile) as exc:
        raise RuntimeError(f"checkpoint is not a complete zip: {path}") from exc
    if corrupt_member is not None:
        raise RuntimeError(
            f"checkpoint contains a corrupt member {corrupt_member!r}: {path}"
        )


class LocalCheckpointSource:
    def __init__(self, directory: Path) -> None:
        self.directory = directory.expanduser().resolve()
        self.label = str(self.directory)

    def latest(self, prefix: str) -> CheckpointCandidate | None:
        candidates: list[CheckpointCandidate] = []
        for path in self.directory.glob(f"{prefix}_*_steps.zip"):
            step = _checkpoint_step(str(path), prefix)
            if step is not None:
                candidates.append(CheckpointCandidate(str(path), step))
        return max(candidates, key=lambda item: item.step, default=None)

    def fetch(self, candidate: CheckpointCandidate, destination: Path) -> None:
        shutil.copyfile(candidate.source_path, destination)


def _remote_path_expression(path: str) -> str:
    """Quote a remote path while preserving a leading ~/ expansion."""

    if "\n" in path or "\0" in path:
        raise ValueError("checkpoint directory cannot contain newline or NUL")
    if path == "~":
        return '"$HOME"'
    if path.startswith("~/"):
        return '"$HOME"/' + shlex.quote(path[2:])
    return shlex.quote(path)


class SSHCheckpointSource:
    def __init__(
        self,
        host: str,
        directory: str,
        *,
        port: int | None = None,
        identity_file: Path | None = None,
        connect_timeout: int = 8,
    ) -> None:
        if SAFE_SSH_HOST.fullmatch(host) is None:
            raise ValueError(
                "--host may contain only letters, digits, '.', '_', '-', and '@'"
            )
        self.host = host
        self.directory = directory
        self.port = port
        self.identity_file = (
            None if identity_file is None else identity_file.expanduser().resolve()
        )
        self.connect_timeout = connect_timeout
        self.label = f"{host}:{directory}"

    def _ssh_command(self) -> list[str]:
        command = [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            f"ConnectTimeout={self.connect_timeout}",
        ]
        if self.port is not None:
            command.extend(["-p", str(self.port)])
        if self.identity_file is not None:
            command.extend(["-i", str(self.identity_file)])
        command.append(self.host)
        return command

    def latest(self, prefix: str) -> CheckpointCandidate | None:
        directory = _remote_path_expression(self.directory)
        pattern = f"{prefix}_*_steps.zip"
        remote_command = (
            f"for scone_ckpt in {directory}/{pattern}; do "
            '[ -f "$scone_ckpt" ] && printf \'%s\\n\' "$scone_ckpt"; '
            "done"
        )
        result = subprocess.run(
            [*self._ssh_command(), remote_command],
            check=False,
            capture_output=True,
            text=True,
            timeout=self.connect_timeout + 10,
        )
        if result.returncode != 0:
            detail = result.stderr.strip() or f"ssh exit code {result.returncode}"
            raise RuntimeError(detail)

        candidates: list[CheckpointCandidate] = []
        for path in result.stdout.splitlines():
            step = _checkpoint_step(path, prefix)
            if step is not None:
                candidates.append(CheckpointCandidate(path, step))
        return max(candidates, key=lambda item: item.step, default=None)

    def fetch(self, candidate: CheckpointCandidate, destination: Path) -> None:
        remote_command = f"cat -- {shlex.quote(candidate.source_path)}"
        with destination.open("wb") as output:
            result = subprocess.run(
                [*self._ssh_command(), remote_command],
                check=False,
                stdout=output,
                stderr=subprocess.PIPE,
                timeout=max(60, self.connect_timeout + 10),
            )
        if result.returncode != 0:
            detail = result.stderr.decode(errors="replace").strip()
            raise RuntimeError(detail or f"ssh exit code {result.returncode}")


def mirror_checkpoint(
    source: CheckpointSource,
    candidate: CheckpointCandidate,
    cache_dir: Path,
    *,
    refresh: bool = False,
) -> Path:
    """Download, validate, then atomically publish a checkpoint locally."""

    cache_dir.mkdir(parents=True, exist_ok=True)
    destination = cache_dir / Path(candidate.source_path).name
    if destination.exists() and not refresh:
        try:
            _validate_ppo_zip(destination)
            return destination
        except RuntimeError:
            pass

    partial = destination.with_suffix(destination.suffix + ".part")
    try:
        source.fetch(candidate, partial)
        _validate_ppo_zip(partial)
        os.replace(partial, destination)
    finally:
        partial.unlink(missing_ok=True)
    return destination


class CheckpointPoller:
    """Poll and mirror checkpoints without blocking the MuJoCo viewer."""

    def __init__(
        self,
        source: CheckpointSource,
        prefix: str,
        cache_dir: Path,
        poll_interval: float,
    ) -> None:
        self.source = source
        self.prefix = prefix
        self.cache_dir = cache_dir
        self.poll_interval = poll_interval
        self.updates: queue.SimpleQueue[tuple[int, Path]] = queue.SimpleQueue()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="scone-checkpoint-poller", daemon=True
        )
        self._mirrored_step = -1
        self._last_error = ""

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=max(2.0, self.poll_interval + 1.0))

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                candidate = self.source.latest(self.prefix)
                if candidate is not None and candidate.step > self._mirrored_step:
                    path = mirror_checkpoint(self.source, candidate, self.cache_dir)
                    self.updates.put((candidate.step, path))
                    self._mirrored_step = candidate.step
                    self._last_error = ""
                    print(
                        f"[remote-watch] downloaded step {candidate.step:,}: {path}",
                        flush=True,
                    )
            except Exception as exc:  # Keep the viewer alive through SSH outages.
                message = str(exc)
                if message != self._last_error:
                    print(f"[remote-watch] waiting: {message}", flush=True)
                    self._last_error = message
            self._stop.wait(self.poll_interval)


def _newest_update(
    updates: queue.SimpleQueue[tuple[int, Path]],
) -> tuple[int, Path] | None:
    newest: tuple[int, Path] | None = None
    while True:
        try:
            newest = updates.get_nowait()
        except queue.Empty:
            return newest


def run_download_once(
    source: CheckpointSource, prefix: str, cache_dir: Path
) -> int:
    candidate = source.latest(prefix)
    if candidate is None:
        print(f"no {prefix}_*_steps.zip checkpoint found in {source.label}")
        return 1
    path = mirror_checkpoint(source, candidate, cache_dir)
    print(f"downloaded step {candidate.step:,}: {path}")
    return 0


def run_viewer(args: argparse.Namespace, source: CheckpointSource) -> int:
    env = SconeWalkEnv(
        args.model,
        curriculum=args.curriculum,
        fixed_command=args.command,
        render_mode="human",
        terrain=args.terrain,
        terrain_seed=args.terrain_seed,
        standing_pose_degrees=args.standing_pose_degrees,
        reference_motion=args.reference_motion,
    )
    observation, _ = env.reset(seed=args.seed)
    zero_action = np.zeros(env.action_space.shape, dtype=np.float32)
    neutral_gate = NeutralResidualGate()
    policy: PPO | None = None
    active_step = -1
    poller = CheckpointPoller(
        source, args.prefix, args.cache_dir, args.poll_interval
    )
    poller.start()
    print(
        f"[remote-watch] watching {source.label}; baseline gait is shown until "
        "the first checkpoint arrives",
        flush=True,
    )

    try:
        while True:
            frame_start = time.perf_counter()
            update = _newest_update(poller.updates)
            if update is not None and update[0] > active_step:
                step, checkpoint_path = update
                try:
                    next_policy = _load_policy(checkpoint_path, env, args.device)
                except Exception as exc:
                    print(
                        f"[remote-watch] rejected step {step:,}: {exc}", flush=True
                    )
                else:
                    policy = next_policy
                    active_step = step
                    compatibility = (
                        " (legacy 68-observation compatibility mode)"
                        if policy.observation_space.shape
                        == LEGACY_OBSERVATION_SHAPE
                        else ""
                    )
                    print(
                        f"[remote-watch] now replaying step {active_step:,}"
                        f"{compatibility}",
                        flush=True,
                    )

            if policy is None:
                action = zero_action
            else:
                policy_observation = _observation_for_policy(policy, observation)
                policy_action, _ = policy.predict(
                    policy_observation, deterministic=True
                )
                action = (
                    policy_action
                    if args.raw_policy
                    else neutral_gate.apply(
                        args.command, policy_action, env.control_dt
                    )
                )
            observation, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                observation, _ = env.reset()
                neutral_gate.reset()

            if env._viewer is not None and not env._viewer.is_running():
                break
            target_dt = env.control_dt / args.render_speed
            remaining = target_dt - (time.perf_counter() - frame_start)
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        poller.stop()
        env.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay the newest SCONE training checkpoint over SSH."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--host", help="SSH config alias or user@hostname")
    source.add_argument(
        "--local-dir",
        type=Path,
        help="Watch a local checkpoint directory instead of SSH",
    )
    parser.add_argument(
        "--checkpoint-dir",
        help="Checkpoint directory on the SSH host; required with --host",
    )
    parser.add_argument(
        "--port",
        type=int,
        help="Override the port from ~/.ssh/config",
    )
    parser.add_argument("--identity-file", type=Path)
    parser.add_argument("--connect-timeout", type=int, default=8)
    parser.add_argument("--prefix", default="scone_walk")
    parser.add_argument("--poll-interval", type=float, default=5.0)
    parser.add_argument("--cache-dir", type=Path, default=Path("runs/remote_watch"))
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    parser.add_argument(
        "--terrain",
        choices=TERRAIN_CHOICES,
        default=TerrainType.FLAT.value,
    )
    parser.add_argument("--terrain-seed", type=int, default=7)
    parser.add_argument(
        "--reference-motion",
        choices=REFERENCE_MOTION_CHOICES,
        default="hardcoded",
    )
    parser.add_argument(
        "--standing-pose-degrees",
        type=float,
        nargs=18,
        metavar="DEG",
        default=SPORT_STANDING_DEGREES,
    )
    parser.add_argument(
        "--curriculum", choices=tuple(CURRICULUM_RANGES), default="full"
    )
    parser.add_argument(
        "--command",
        type=float,
        nargs=3,
        metavar=("VX", "VY", "YAW_RATE"),
        default=[0.25, 0.0, 0.0],
    )
    parser.add_argument(
        "--raw-policy",
        action="store_true",
        help="disable the neutral-command residual gate for policy diagnosis",
    )
    parser.add_argument("--render-speed", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="cpu")
    parser.add_argument(
        "--download-only",
        action="store_true",
        help="Download and validate the latest checkpoint, then exit",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if SAFE_PREFIX.fullmatch(args.prefix) is None:
        raise SystemExit("--prefix may contain only letters, digits, '.', '_', and '-'")
    if args.poll_interval <= 0.0 or args.render_speed <= 0.0:
        raise SystemExit("--poll-interval and --render-speed must be positive")
    if args.port is not None and not 1 <= args.port <= 65535:
        raise SystemExit("--port must be between 1 and 65535")

    args.model = args.model.expanduser().resolve()
    args.cache_dir = args.cache_dir.expanduser().resolve()
    if not args.model.exists():
        raise SystemExit(f"model not found: {args.model}")

    if args.host is not None:
        if args.checkpoint_dir is None:
            raise SystemExit("--checkpoint-dir is required with --host")
        source: CheckpointSource = SSHCheckpointSource(
            args.host,
            args.checkpoint_dir,
            port=args.port,
            identity_file=args.identity_file,
            connect_timeout=args.connect_timeout,
        )
    else:
        if args.checkpoint_dir is not None:
            raise SystemExit("--checkpoint-dir is used only with --host")
        source = LocalCheckpointSource(args.local_dir)

    if args.download_only:
        return run_download_once(source, args.prefix, args.cache_dir)
    return run_viewer(args, source)


if __name__ == "__main__":
    raise SystemExit(main())
