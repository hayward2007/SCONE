"""Interactive x/y/yaw joystick runner for a saved SCONE PPO policy."""

from __future__ import annotations

import threading
import time
from pathlib import Path

from src.cli import JoystickLimits, run_velocity_joystick_cli
from src.locomotion import VelocityCommand
from src.simulation.terrain import TerrainType

from .remote_watch import _load_policy, _observation_for_policy, _validate_ppo_zip
from .walk_learn import (
    DEFAULT_MODEL_PATH,
    OBSERVATION_COMMAND_SCALE,
    SconeWalkEnv,
    WalkConfig,
)


class _VelocityMailbox:
    def __init__(self) -> None:
        self._command = VelocityCommand()
        self._lock = threading.Lock()

    def update(self, command: VelocityCommand, _dt: float) -> None:
        with self._lock:
            self._command = command

    def read(self) -> VelocityCommand:
        with self._lock:
            return self._command


def run_rl_joystick(
    checkpoint: str | Path,
    *,
    model_path: str | Path = DEFAULT_MODEL_PATH,
    terrain: TerrainType | str = TerrainType.FLAT,
    terrain_seed: int = 7,
    device: str = "auto",
    seed: int = 7,
) -> None:
    """Run a PPO policy whose command is supplied by the common CLI joystick."""

    checkpoint_path = Path(checkpoint).expanduser().resolve()
    _validate_ppo_zip(checkpoint_path)
    env = SconeWalkEnv(
        model_path,
        fixed_command=[0.0, 0.0, 0.0],
        render_mode="human",
        walk_config=WalkConfig(episode_seconds=24.0 * 60.0 * 60.0),
        terrain=terrain,
        terrain_seed=terrain_seed,
    )
    policy = _load_policy(checkpoint_path, env, device)
    observation, _ = env.reset(seed=seed)
    mailbox = _VelocityMailbox()
    stop_event = threading.Event()
    cli_errors: list[BaseException] = []
    limits = JoystickLimits(
        max_vx=float(OBSERVATION_COMMAND_SCALE[0]),
        max_vy=float(OBSERVATION_COMMAND_SCALE[1]),
        max_yaw_rate=float(OBSERVATION_COMMAND_SCALE[2]),
    )

    def input_worker() -> None:
        try:
            run_velocity_joystick_cli(
                limits=limits,
                apply_command=mailbox.update,
                profile_name="policy",
                control_name="rl",
                control_hint=checkpoint_path.name,
                stop_event=stop_event,
            )
        except BaseException as error:
            cli_errors.append(error)
            stop_event.set()

    worker = threading.Thread(
        target=input_worker,
        name="scone-rl-joystick-input",
        daemon=True,
    )
    worker.start()

    try:
        while not stop_event.is_set():
            frame_started = time.perf_counter()
            command = mailbox.read()
            env.set_velocity_command(command.as_array())
            policy_observation = _observation_for_policy(policy, observation)
            action, _ = policy.predict(policy_observation, deterministic=True)
            observation, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                observation, _ = env.reset()

            if env._viewer is not None and not env._viewer.is_running():
                stop_event.set()
                break
            remaining = env.control_dt - (time.perf_counter() - frame_started)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        stop_event.set()
        worker.join()
        env.close()

    if cli_errors:
        raise cli_errors[0]


__all__ = ["run_rl_joystick"]
