"""Interactive x/y/yaw joystick runner for a saved SCONE PPO policy."""

from __future__ import annotations

import threading
import time
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.cli import JoystickLimits, run_velocity_joystick_cli
from src.locomotion import (
    LegacyVelocityAdapter,
    SconeGait,
    SconeGaitConfig,
    VelocityCommand,
    Walk,
)
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


@dataclass(frozen=True)
class SconeHybridControlConfig:
    """Replay-time transition between PPO and SCONE's hybrid fast gait."""

    hybrid_start_speed: float = 0.10
    hybrid_full_speed: float = 0.18
    cycle_frequency: float = 1.2
    duty_factor: float = 0.60
    step_height: float = 0.025
    max_stride: float = 0.090
    max_lateral_stride: float = 0.070
    sector_sweep_degrees: float = 30.0
    point_support_ratio: float = 0.55
    swing_roll_hold_ratio: float = 0.70
    effective_roll_radius: float = 0.1225
    max_roll_rate_degrees: float = 360.0

    def __post_init__(self) -> None:
        if self.hybrid_start_speed < 0.0:
            raise ValueError("hybrid_start_speed cannot be negative")
        if self.hybrid_full_speed <= self.hybrid_start_speed:
            raise ValueError(
                "hybrid_full_speed must be greater than hybrid_start_speed"
            )
        if self.effective_roll_radius <= 0.0:
            raise ValueError("effective_roll_radius must be positive")
        if self.max_roll_rate_degrees <= 0.0:
            raise ValueError("max_roll_rate_degrees must be positive")

    def hybrid_blend(self, command: VelocityCommand) -> float:
        """Return 0 for PPO-only and 1 for the full fast hybrid reference.

        Only translation selects the hybrid.  A command with zero planar
        speed and non-zero yaw therefore remains entirely under PPO control.
        Smoothstep avoids a target jump at either transition boundary.
        """

        planar_speed = float(np.hypot(command.vx, command.vy))
        ratio = np.clip(
            (planar_speed - self.hybrid_start_speed)
            / (self.hybrid_full_speed - self.hybrid_start_speed),
            0.0,
            1.0,
        )
        return float(ratio * ratio * (3.0 - 2.0 * ratio))

    def gait_config(self, control_dt: float) -> SconeGaitConfig:
        return SconeGaitConfig(
            control_frequency=1.0 / control_dt,
            cycle_frequency=self.cycle_frequency,
            duty_factor=self.duty_factor,
            step_height=self.step_height,
            max_stride=self.max_stride,
            max_lateral_stride=self.max_lateral_stride,
            max_vx=float(OBSERVATION_COMMAND_SCALE[0]),
            max_vy=float(OBSERVATION_COMMAND_SCALE[1]),
            max_yaw_rate=float(OBSERVATION_COMMAND_SCALE[2]),
            command_time_constant=0.0,
            sector_sweep_degrees=self.sector_sweep_degrees,
            point_support_ratio=self.point_support_ratio,
            swing_roll_hold_ratio=self.swing_roll_hold_ratio,
            continuous_rotation=True,
            rolling_blend=1.0,
            effective_roll_radius=self.effective_roll_radius,
            max_roll_rate_degrees=self.max_roll_rate_degrees,
            ik_tolerance=1e-3,
            ik_stride_backoff_attempts=4,
        )


class SconeHybridController:
    """Blend a legacy-compatible PPO replay into the fast SCONE gait."""

    def __init__(
        self,
        env: SconeWalkEnv,
        *,
        config: SconeHybridControlConfig | None = None,
    ) -> None:
        self.env = env
        self.config = config or SconeHybridControlConfig()
        self.gait = SconeGait(
            profile=env._motion_profile,
            model_path=env.model_path,
            config=self.config.gait_config(env.control_dt),
        )
        self.last_blend = 0.0
        self._branch_anchor = np.asarray(
            env.default_degrees,
            dtype=np.float64,
        ).copy()
        self.reset()

    def reset(self) -> None:
        self.gait.reset(
            phase=self.env._phase,
            motor_degrees=self.env.default_degrees,
        )
        self._branch_anchor = np.asarray(
            self.env.default_degrees,
            dtype=np.float64,
        ).copy()
        self.last_blend = 0.0
        self.env.set_reference_override(
            self._branch_anchor,
            blend=0.0,
            unwrapped_lower=True,
        )

    def apply(
        self,
        command: VelocityCommand,
        policy_action: Sequence[float],
    ) -> np.ndarray:
        """Install the hybrid reference and attenuate the PPO residual."""

        action = np.asarray(policy_action, dtype=np.float32)
        if action.size != 18:
            raise ValueError("policy_action must contain 18 residual values")
        action = action.reshape(18)
        blend = self.config.hybrid_blend(command)
        if blend <= 1e-9:
            self.last_blend = 0.0
            # Keep the PPO lower reference on the nearest equivalent 360°
            # branch.  This prevents a completed sector revolution from being
            # undone merely because the command returned to low speed.
            self.env.set_reference_override(
                self._branch_anchor,
                blend=0.0,
                unwrapped_lower=True,
            )
            return action
        if self.last_blend <= 1e-9:
            # Align the dormant gait with the checkpoint reference exactly at
            # the first mixed frame.  Reset is intentionally not repeated on
            # every PPO-only frame because it recalibrates sector geometry.
            self.gait.reset(
                phase=self.env._phase,
                motor_degrees=self.env.default_degrees,
            )
            self.gait.set_continuous_roll_degrees(
                360.0
                * np.round(
                    (
                        self._branch_anchor[12:]
                        - np.asarray(self.env.default_degrees)[12:]
                    )
                    / 360.0
                )
            )
        sample = self.gait.step(command, self.env.control_dt)
        if not sample.converged:
            raise RuntimeError(
                "scone-gait hybrid reference IK failed for legs "
                f"{sample.failed_legs}"
            )
        self.last_blend = blend
        self._branch_anchor = sample.motor_degrees.copy()
        self.env.set_reference_override(
            sample.motor_degrees,
            blend=blend,
            unwrapped_lower=True,
        )
        # PPO owns slow walking and in-place yaw.  At high speed the model
        # reference owns the motion; the transition band mixes both smoothly.
        return action * np.float32(1.0 - blend)

    def control_name(self) -> str:
        if self.last_blend <= 1e-6:
            return "scone-gait/ppo"
        turns = float(
            np.max(np.abs(self.gait.continuous_roll_degrees)) / 360.0
        )
        if self.last_blend >= 1.0 - 1e-6:
            return f"scone-gait/hybrid/roll-{turns:.1f}turn"
        return (
            f"scone-gait/mix-{self.last_blend:.2f}/"
            f"roll-{turns:.1f}turn"
        )


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
    reference_motion: str = "hardcoded",
    hybrid_scone: bool = False,
    hybrid_config: SconeHybridControlConfig | None = None,
) -> None:
    """Run a PPO policy whose command is supplied by the common CLI joystick.

    Replay defaults to the original hardcoded residual reference because the
    pre-selection checkpoints were trained against that exact action meaning.
    A gait-referenced checkpoint must select the same ``tripod-gait`` or
    ``scone-gait`` reference used during training. ``non_rl`` remains a legacy
    alias for ``tripod-gait``.
    """

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
    hybrid = (
        SconeHybridController(env, config=hybrid_config)
        if hybrid_scone
        else None
    )
    stop_event = threading.Event()
    cli_errors: list[BaseException] = []
    limits = JoystickLimits(
        max_vx=float(OBSERVATION_COMMAND_SCALE[0]),
        max_vy=float(OBSERVATION_COMMAND_SCALE[1]),
        max_yaw_rate=float(OBSERVATION_COMMAND_SCALE[2]),
    )

    def input_worker() -> None:
        def displayed_control_name() -> str:
            if not mode_router.policy_active():
                return mode_router.control_name()
            return hybrid.control_name() if hybrid is not None else "rl/walk"

        try:
            run_velocity_joystick_cli(
                limits=limits,
                apply_command=mode_router.apply_command,
                profile_name=mode_router.profile.name,
                control_name=displayed_control_name,
                control_hint=(
                    (
                        "slow/in-place yaw=PPO, fast=point-support+sector-roll; "
                        if hybrid is not None
                        else "R: RL Walk→Drive→Climb; "
                    )
                    + f"ref={reference_motion}; "
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
                if hybrid is not None:
                    env.set_reference_override(None)
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
                if hybrid is not None:
                    hybrid.reset()
            command = mailbox.read()
            env.set_velocity_command(command.as_array())
            policy_observation = _observation_for_policy(policy, observation)
            policy_action, _ = policy.predict(
                policy_observation, deterministic=True
            )
            action = neutral_gate.apply(
                command.as_array(), policy_action, env.control_dt
            )
            if hybrid is not None:
                action = hybrid.apply(command, action)
            observation, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                observation, _ = env.reset()
                mode_router.rebind_policy_controller(env.controller)
                neutral_gate.reset()
                if hybrid is not None:
                    hybrid.reset()

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


__all__ = [
    "NeutralResidualGate",
    "SconeHybridControlConfig",
    "SconeHybridController",
    "run_rl_joystick",
]
