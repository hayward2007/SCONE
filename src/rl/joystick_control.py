"""Interactive x/y/yaw joystick runner for a saved SCONE PPO policy."""

from __future__ import annotations

import threading
import time
from collections.abc import Sequence
from pathlib import Path

from src.cli import JoystickLimits, run_velocity_joystick_cli
from src.locomotion import LegacyVelocityAdapter, VelocityCommand, Walk
from src.main import SCONE
from src.simulation.terrain import TerrainType

from .motion_profile import motion_profile_for_standing_pose
from .remote_watch import _load_policy, _observation_for_policy, _validate_ppo_zip
from .stance import SPORT_STANDING_DEGREES
from .walk_learn import (
    DEFAULT_MODEL_PATH,
    NeutralResidualGate,
    OBSERVATION_COMMAND_SCALE,
    REFERENCE_MOTION_CHOICES,
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


class _RLModeRouter:
    """Route one joystick between PPO Walk and legacy Drive/Climb modes."""

    def __init__(
        self,
        controller,
        standing_pose_degrees: Sequence[float],
        mailbox: _VelocityMailbox,
    ) -> None:
        self.profile = motion_profile_for_standing_pose(standing_pose_degrees)
        self.mailbox = mailbox
        self.robot = SCONE(controller, profile=self.profile)
        # The environment reset already seeds/enables the simulated joints.
        # Calling SCONE.initialize() here would unnecessarily replay home and
        # would also change the pose under the loaded policy.
        self.robot.initialized = True
        self.adapter = LegacyVelocityAdapter(self.robot)
        self.adapter.start()
        self._lock = threading.RLock()
        self._transitioning = False
        self._resume_pending = False

    def apply_command(self, command: VelocityCommand, dt: float) -> None:
        with self._lock:
            policy_active = (
                not self._transitioning and self.robot.mode_name == "walk"
            )
        if policy_active:
            self.mailbox.update(command, dt)
        else:
            self.adapter.update(command)

    def handle_key(self, key: str) -> bool:
        if key != "r":
            return False
        self.mailbox.update(VelocityCommand(), 0.0)
        self.adapter.update(VelocityCommand())
        with self._lock:
            self._transitioning = True
        try:
            next_mode = self.robot.change_mode()
        finally:
            with self._lock:
                self._transitioning = False
        if next_mode == "walk":
            with self._lock:
                self._resume_pending = True
        return True

    def policy_active(self) -> bool:
        with self._lock:
            return not self._transitioning and self.robot.mode_name == "walk"

    def consume_resume_pending(self) -> bool:
        with self._lock:
            pending = self._resume_pending
            self._resume_pending = False
            return pending

    def control_name(self) -> str:
        with self._lock:
            if self._transitioning:
                return "rl/transition"
            mode = self.robot.mode_name
        return "rl/walk" if mode == "walk" else f"old/{mode}"

    def rebind_policy_controller(self, controller) -> None:
        """Point the legacy state machine at a controller recreated by reset."""

        with self._lock:
            self.robot.controller = controller
            self.robot.mode = Walk(controller, self.profile)
            self.robot.initialized = True
            self._transitioning = False
            self._resume_pending = False

    def close(self) -> None:
        self.adapter.close()


def run_rl_joystick(
    checkpoint: str | Path,
    *,
    model_path: str | Path = DEFAULT_MODEL_PATH,
    terrain: TerrainType | str = TerrainType.FLAT,
    terrain_seed: int = 7,
    device: str = "auto",
    seed: int = 7,
    standing_pose_degrees: Sequence[float] = SPORT_STANDING_DEGREES,
    reference_motion: str = "non_rl",
) -> None:
    """Run a PPO policy whose command is supplied by the common CLI joystick."""

    checkpoint_path = Path(checkpoint).expanduser().resolve()
    _validate_ppo_zip(checkpoint_path)
    if reference_motion not in REFERENCE_MOTION_CHOICES:
        raise ValueError(
            f"unknown reference motion {reference_motion!r}; "
            f"choose from {REFERENCE_MOTION_CHOICES}"
        )
    env = SconeWalkEnv(
        model_path,
        fixed_command=[0.0, 0.0, 0.0],
        render_mode="human",
        walk_config=WalkConfig(episode_seconds=24.0 * 60.0 * 60.0),
        terrain=terrain,
        terrain_seed=terrain_seed,
        standing_pose_degrees=standing_pose_degrees,
        reference_motion=reference_motion,
    )
    policy = _load_policy(checkpoint_path, env, device)
    observation, _ = env.reset(seed=seed)
    mailbox = _VelocityMailbox()
    mode_router = _RLModeRouter(
        env.controller,
        standing_pose_degrees,
        mailbox,
    )
    neutral_gate = NeutralResidualGate()
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
                apply_command=mode_router.apply_command,
                profile_name=mode_router.profile.name,
                control_name=mode_router.control_name,
                control_hint=(
                    f"R: RL Walk→Drive→Climb; ref={reference_motion}; "
                    f"checkpoint={checkpoint_path.name}"
                ),
                stop_event=stop_event,
                handle_key=mode_router.handle_key,
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
            if not mode_router.policy_active():
                env.advance_external_control()
                if env._viewer is not None and not env._viewer.is_running():
                    stop_event.set()
                    break
                remaining = env.control_dt - (time.perf_counter() - frame_started)
                if remaining > 0.0:
                    time.sleep(remaining)
                continue
            if mode_router.consume_resume_pending():
                observation = env.resume_after_external_control()
                neutral_gate.reset()
            command = mailbox.read()
            env.set_velocity_command(command.as_array())
            policy_observation = _observation_for_policy(policy, observation)
            policy_action, _ = policy.predict(
                policy_observation, deterministic=True
            )
            action = neutral_gate.apply(
                command.as_array(), policy_action, env.control_dt
            )
            observation, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                observation, _ = env.reset()
                mode_router.rebind_policy_controller(env.controller)
                neutral_gate.reset()

            if env._viewer is not None and not env._viewer.is_running():
                stop_event.set()
                break
            remaining = env.control_dt - (time.perf_counter() - frame_started)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        stop_event.set()
        worker.join()
        mode_router.close()
        env.close()

    if cli_errors:
        raise cli_errors[0]


__all__ = ["NeutralResidualGate", "run_rl_joystick"]
