"""Command-conditioned residual-RL environment for SCONE walking.

The policy command is ``[vx, vy, yaw_rate]`` in the robot body frame.  The
18-dimensional action is deliberately *not* a raw current command: SCONE's
XM430 motors support current control, but its Protocol-1 MX-28AT motors do not.
Instead, the policy adds small joint-position residuals to a smooth reference
tripod gait derived from ``src/provider/walk.py``.  The existing MuJoCo
``dcmotor`` + outer PD loop then converts those targets to physically limited
motor voltage/torque.

Examples
--------
Run API and numerical checks without training::

    python -m src.rl.walk_learn check --steps 500

Train a first forward-only PPO policy::

    python -m src.rl.walk_learn train --curriculum easy --timesteps 1000000

Preview a saved policy at a fixed body-frame velocity command::

    mjpython -m src.rl.walk_learn enjoy runs/scone_walk/final_model.zip \
        --command 0.25 0.0 0.0
"""

from __future__ import annotations

import argparse
import math
import re
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Sequence

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecMonitor

from src.hardware.actuator import Actuator
from src.locomotion import GaitConfig, NonRLWalk
from src.rl.motion_profile import motion_profile_for_standing_pose
from src.rl.policy_compat import load_compatible_policy, observation_for_policy
from src.rl.stance import SPORT_STANDING_DEGREES, validate_standing_pose
from src.simulation.core.controller import MuJoCoController
from src.simulation.core.model import load_model
from src.simulation.core.pid import spec_for_motor_id
from src.simulation.core.viewer import configure_simulation_viewer
from src.simulation.terrain import TERRAIN_CHOICES, TerrainType


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_PATH = PROJECT_ROOT / "assets" / "model.xml"


@dataclass(frozen=True)
class RewardConfig:
    """Normalized reward scales and initial weights.

    These are safe starting values, not fitted physical constants.  Each raw
    penalty is normalized before weighting so TensorBoard plots remain useful
    while the weights are tuned.
    """

    linear_velocity_sigma: float = 0.25       # m/s
    yaw_velocity_sigma: float = 0.15          # rad/s
    heading_error_sigma: float = 0.60         # rad
    projected_gravity_sigma: float = 0.25
    height_sigma: float = 0.05                # m
    vertical_velocity_sigma: float = 0.30     # m/s
    roll_pitch_rate_sigma: float = 0.80       # rad/s
    slip_deadzone: float = 0.02               # m/s at the actual contact point
    slip_sigma: float = 0.20                   # m/s after the deadzone
    soft_joint_offset: float = math.radians(60.0)
    hard_joint_offset: float = math.radians(90.0)

    velocity_weight: float = 2.0
    yaw_weight: float = 1.0
    heading_weight: float = 0.75
    upright_weight: float = 0.5
    # Height is only a weak one-sided collapse guard. Body attitude from
    # projected gravity is the primary posture objective, so the policy may
    # discover a stance that rises above the reset reference without penalty.
    height_weight: float = 0.05
    oscillation_weight: float = 0.1
    action_rate_weight: float = 0.02
    action_magnitude_weight: float = 0.25
    # General tracking tolerances are intentionally broad enough for moving
    # commands. At idle, use a much tighter velocity target and explicitly
    # discourage residual action so the learned policy has a true stop state.
    idle_velocity_weight: float = 1.0
    idle_action_weight: float = 0.5
    idle_linear_velocity_sigma: float = 0.03
    idle_yaw_velocity_sigma: float = 0.05
    idle_activity_threshold: float = 0.05
    current_weight: float = 0.02
    slip_weight: float = 0.1
    joint_limit_weight: float = 0.2
    collision_weight: float = 1.0
    termination_penalty: float = 5.0

    def __post_init__(self) -> None:
        if self.idle_activity_threshold <= 0.0:
            raise ValueError("idle_activity_threshold must be positive")
        if min(
            self.idle_linear_velocity_sigma,
            self.idle_yaw_velocity_sigma,
        ) <= 0.0:
            raise ValueError("idle velocity sigmas must be positive")


@dataclass(frozen=True)
class WalkConfig:
    physics_timestep: float = 0.002            # 500 Hz MuJoCo
    frame_skip: int = 10                       # 50 Hz policy
    episode_seconds: float = 10.0
    command_filter_seconds: float = 0.35
    command_hold_seconds_min: float = 2.0
    command_hold_seconds_max: float = 4.0
    idle_command_probability: float = 0.20
    gait_frequency_min: float = 0.6
    gait_frequency_max: float = 1.4
    legacy_stride_degrees: float = 20.0
    legacy_lift_degrees: float = 20.0
    settle_seconds: float = 0.20
    max_height_drop: float = 0.12
    max_tilt_degrees: float = 60.0
    contact_force_threshold: float = 1.0

    def __post_init__(self) -> None:
        if not 0.0 <= self.idle_command_probability <= 1.0:
            raise ValueError("idle_command_probability must be between 0 and 1")


CURRICULUM_RANGES: dict[str, np.ndarray] = {
    # [|vx|, |vy|, |yaw_rate|]. Easy intentionally starts with the motion for
    # which the hand-authored Walk controller already provides a reference.
    "easy": np.array([0.30, 0.00, 0.00], dtype=np.float64),
    "medium": np.array([0.40, 0.00, 0.60], dtype=np.float64),
    "full": np.array([0.50, 0.25, 0.80], dtype=np.float64),
}

OBSERVATION_COMMAND_SCALE = np.array([0.50, 0.25, 0.80], dtype=np.float64)
REFERENCE_MOTION_CHOICES = ("non_rl", "hardcoded")


class NeutralResidualGate:
    """Decay policy residuals to zero while the velocity command is neutral.

    A low-pass filter alone cannot remove a constant policy bias. Active
    commands pass through unchanged; neutral commands ignore new policy bias.
    """

    def __init__(
        self,
        *,
        command_threshold: float = 0.02,
        decay_seconds: float = 0.10,
        zero_epsilon: float = 5e-3,
    ) -> None:
        if not 0.0 <= command_threshold < 1.0:
            raise ValueError("command_threshold must be in [0, 1)")
        if decay_seconds < 0.0 or zero_epsilon < 0.0:
            raise ValueError("decay_seconds and zero_epsilon cannot be negative")
        self.command_threshold = command_threshold
        self.decay_seconds = decay_seconds
        self.zero_epsilon = zero_epsilon
        self._action = np.zeros(18, dtype=np.float32)

    def reset(self) -> None:
        self._action.fill(0.0)

    def apply(
        self,
        command: Sequence[float],
        policy_action: Sequence[float],
        dt: float,
    ) -> np.ndarray:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        parsed_command = np.asarray(command, dtype=np.float64)
        parsed_action = np.asarray(policy_action, dtype=np.float32)
        if parsed_command.shape != (3,):
            raise ValueError("command must contain [vx, vy, yaw_rate]")
        if parsed_action.size != self._action.size:
            raise ValueError("policy_action must contain 18 residual values")
        parsed_action = parsed_action.reshape(self._action.shape)

        activity = float(
            np.max(np.abs(parsed_command / OBSERVATION_COMMAND_SCALE))
        )
        if activity > self.command_threshold:
            self._action[:] = parsed_action
            return self._action.copy()

        if self.decay_seconds == 0.0:
            self._action.fill(0.0)
        else:
            self._action *= math.exp(-dt / self.decay_seconds)
            if float(np.max(np.abs(self._action))) <= self.zero_epsilon:
                self._action.fill(0.0)
        return self._action.copy()


class SconeWalkEnv(gym.Env[np.ndarray, np.ndarray]):
    """MuJoCo walking task with a command-conditioned residual action."""

    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(
        self,
        model_path: Path | str = DEFAULT_MODEL_PATH,
        *,
        curriculum: str = "easy",
        fixed_command: np.ndarray | list[float] | None = None,
        render_mode: str | None = None,
        reward_config: RewardConfig | None = None,
        walk_config: WalkConfig | None = None,
        terrain: TerrainType | str = TerrainType.FLAT,
        terrain_seed: int = 7,
        standing_pose_degrees: Sequence[float] | None = None,
        reference_motion: str = "hardcoded",
    ) -> None:
        super().__init__()
        if curriculum not in CURRICULUM_RANGES:
            raise ValueError(
                f"Unknown curriculum {curriculum!r}; choose from "
                f"{tuple(CURRICULUM_RANGES)}."
            )
        if render_mode not in (None, "human"):
            raise ValueError("render_mode must be None or 'human'")
        if reference_motion not in REFERENCE_MOTION_CHOICES:
            raise ValueError(
                f"Unknown reference motion {reference_motion!r}; choose from "
                f"{REFERENCE_MOTION_CHOICES}."
            )

        self.model_path = Path(model_path).expanduser().resolve()
        self.curriculum = curriculum
        self.command_range = CURRICULUM_RANGES[curriculum].copy()
        self.fixed_command = (
            None
            if fixed_command is None
            else np.asarray(fixed_command, dtype=np.float64)
        )
        if self.fixed_command is not None and self.fixed_command.shape != (3,):
            raise ValueError("fixed_command must contain [vx, vy, yaw_rate]")

        self.render_mode = render_mode
        self.reward_config = reward_config or RewardConfig()
        self.walk_config = walk_config or WalkConfig()
        self.reference_motion = reference_motion
        self.terrain = TerrainType.parse(terrain)
        self.terrain_seed = terrain_seed

        self.model = load_model(
            self.model_path,
            terrain=self.terrain,
            terrain_seed=terrain_seed,
        )
        self.model.opt.timestep = self.walk_config.physics_timestep
        self.data = mujoco.MjData(self.model)
        self.controller: MuJoCoController

        self.control_dt = self.model.opt.timestep * self.walk_config.frame_skip
        self.max_episode_steps = round(
            self.walk_config.episode_seconds / self.control_dt
        )

        self.root_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "root_freejoint"
        )
        if self.root_joint_id < 0:
            raise ValueError("walk learning requires model.xml root_freejoint")
        self.root_body_id = int(self.model.jnt_bodyid[self.root_joint_id])
        self.root_qpos_address = int(self.model.jnt_qposadr[self.root_joint_id])

        self.floor_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "simulation_floor"
        )
        if self.floor_geom_id < 0:
            raise ValueError("walk learning requires simulation_floor")
        self.ground_geom_ids = {self.floor_geom_id}
        for geom_id in range(self.model.ngeom):
            name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                geom_id,
            )
            if name and name.startswith("terrain_"):
                self.ground_geom_ids.add(geom_id)

        self.tire_geom_ids: set[int] = set()
        self.tire_geom_to_body: dict[int, int] = {}
        for leg in range(1, 7):
            geom_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, f"TIRE_{leg}_geom"
            )
            if geom_id < 0:
                raise ValueError(f"Missing contact geom TIRE_{leg}_geom")
            self.tire_geom_ids.add(geom_id)
            self.tire_geom_to_body[geom_id] = int(self.model.geom_bodyid[geom_id])

        # Keep posture data local to the simulated environment; selecting an
        # RL stance must never open or command the physical controller.
        self.default_degrees = np.asarray(
            validate_standing_pose(
                SPORT_STANDING_DEGREES
                if standing_pose_degrees is None
                else standing_pose_degrees
            ),
            dtype=np.float64,
        )
        self.default_radians = np.array(
            [
                MuJoCoController.raw_to_radians(
                    MuJoCoController.degrees_to_raw(i, degrees)
                )
                for i, degrees in enumerate(self.default_degrees, start=1)
            ],
            dtype=np.float64,
        )
        self.residual_scale_degrees = np.array(
            [10.0] * 6 + [12.0] * 6 + [15.0] * 6, dtype=np.float64
        )
        self._motion_profile = motion_profile_for_standing_pose(
            self.default_degrees
        )
        self._non_rl_reference: NonRLWalk | None = None
        if self.reference_motion == "non_rl":
            self._non_rl_reference = NonRLWalk(
                profile=self._motion_profile,
                model_path=self.model_path,
                config=GaitConfig(
                    control_frequency=1.0 / self.control_dt,
                    cycle_frequency=0.7,
                    max_stride=0.060,
                    max_lateral_stride=0.050,
                    ik_tolerance=1e-3,
                    ik_stride_backoff_attempts=4,
                    max_vx=float(OBSERVATION_COMMAND_SCALE[0]),
                    max_vy=float(OBSERVATION_COMMAND_SCALE[1]),
                    max_yaw_rate=float(OBSERVATION_COMMAND_SCALE[2]),
                    command_time_constant=0.0,
                ),
            )
            self._non_rl_reference.reset(motor_degrees=self.default_degrees)

        self.action_space = spaces.Box(-1.0, 1.0, shape=(18,), dtype=np.float32)
        # base linear velocity 3, angular velocity 3, projected gravity 3,
        # joint position 18, joint velocity 18, previous action 18, command 3,
        # gait phase sin/cos 2, heading error sin/cos 2 = 70 values.
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(70,), dtype=np.float32
        )

        self._phase = 0.0
        self._episode_step = 0
        self._next_command_step = 0
        self._command = np.zeros(3, dtype=np.float64)
        self._command_target = np.zeros(3, dtype=np.float64)
        self._heading = 0.0
        self._target_heading = 0.0
        self._last_action = np.zeros(18, dtype=np.float64)
        self._reference_height = 0.0
        self._reference_cycle_frequency = 0.0
        self._reference_stride_clip_fraction = 0.0
        self._reference_ik_backoff_scale = 1.0
        self._viewer: Any | None = None

        self._jacobian_position = np.zeros((3, self.model.nv), dtype=np.float64)
        self._jacobian_rotation = np.zeros((3, self.model.nv), dtype=np.float64)
        self._contact_force = np.zeros(6, dtype=np.float64)
        self._body_velocity = np.zeros(6, dtype=np.float64)

    def set_velocity_command(
        self, command: np.ndarray | list[float] | tuple[float, float, float]
    ) -> np.ndarray:
        """Set the live ``[vx, vy, yaw_rate]`` command used on the next step.

        Manual commands use the full observation normalization range, which is
        also the range used by the full curriculum. A checkpoint trained only
        on an easier curriculum may not track lateral or yaw commands well,
        but the environment still exposes all three command axes consistently.
        """

        parsed = np.asarray(command, dtype=np.float64)
        if parsed.shape != (3,):
            raise ValueError("velocity command must contain [vx, vy, yaw_rate]")
        if not np.all(np.isfinite(parsed)):
            raise ValueError("velocity command must contain only finite values")
        clipped = np.clip(
            parsed,
            -OBSERVATION_COMMAND_SCALE,
            OBSERVATION_COMMAND_SCALE,
        )
        self.fixed_command = clipped.copy()
        self._command_target[:] = clipped
        return clipped.copy()

    def _sample_command(self) -> np.ndarray:
        if self.fixed_command is not None:
            return self.fixed_command.copy()
        if self.np_random.random() < self.walk_config.idle_command_probability:
            return np.zeros(3, dtype=np.float64)

        command = self.np_random.uniform(-self.command_range, self.command_range)
        if self.curriculum == "easy":
            # Start with forward motion most of the time, but include a small
            # amount of reverse motion so zero velocity is not a boundary.
            command[0] = self.np_random.uniform(-0.08, self.command_range[0])
        return command

    def _schedule_next_command(self) -> None:
        hold = self.np_random.uniform(
            self.walk_config.command_hold_seconds_min,
            self.walk_config.command_hold_seconds_max,
        )
        self._next_command_step = self._episode_step + max(
            1, round(hold / self.control_dt)
        )

    def _update_command(self) -> None:
        if self.fixed_command is None and self._episode_step >= self._next_command_step:
            self._command_target = self._sample_command()
            self._schedule_next_command()

        if self.fixed_command is not None:
            self._command[:] = self.fixed_command
            return

        alpha = 1.0 - math.exp(
            -self.control_dt / self.walk_config.command_filter_seconds
        )
        self._command += alpha * (self._command_target - self._command)

    def _command_activity(self) -> float:
        safe_scale = np.where(
            OBSERVATION_COMMAND_SCALE > 0.0,
            OBSERVATION_COMMAND_SCALE,
            1.0,
        )
        return float(np.clip(np.max(np.abs(self._command / safe_scale)), 0.0, 1.0))

    def _advance_phase(self) -> None:
        if self._non_rl_reference is not None:
            # NonRLWalk owns its Phoenix gait phase; _reference_motion_degrees
            # copies it back so the policy observation stays phase-aligned.
            return
        activity = self._command_activity()
        frequency = self.walk_config.gait_frequency_min + activity * (
            self.walk_config.gait_frequency_max
            - self.walk_config.gait_frequency_min
        )
        self._reference_cycle_frequency = frequency
        self._reference_stride_clip_fraction = 0.0
        self._reference_ik_backoff_scale = 1.0
        self._phase = (self._phase + frequency * self.control_dt) % 1.0

    def _reference_motion_degrees(self) -> np.ndarray:
        """Return the selected model-based or hand-authored reference frame.

        The recommended ``non_rl`` reference runs the continuous Phoenix gait
        and IK solver used by interactive Non-RL control. ``hardcoded`` keeps
        the earlier sinusoidal version of the blocking Walk tripod sequence.
        """

        if self._non_rl_reference is not None:
            sample = self._non_rl_reference.step(self._command, self.control_dt)
            if not sample.converged:
                raise RuntimeError(
                    "Non-RL residual reference IK failed for legs "
                    f"{sample.failed_legs}"
                )
            self._phase = sample.phase
            self._reference_cycle_frequency = sample.cycle_frequency
            self._reference_stride_clip_fraction = sample.stride_clip_fraction
            self._reference_ik_backoff_scale = sample.ik_backoff_scale
            return sample.motor_degrees.copy()

        reference = self.default_degrees.copy()
        phase_sine = math.sin(2.0 * math.pi * self._phase)
        activity = self._command_activity()

        # The legacy motor-angle convention advances the chassis in +body-X
        # when its stride offset is negative (verified from root displacement).
        vx_scale = float(np.clip(-self._command[0] / 0.50, -1.0, 1.0))
        yaw_scale = float(np.clip(self._command[2] / 0.80, -1.0, 1.0))
        tripod_a = set(Actuator.Index.UPPER_DIAGONAL_LEFT)   # {2, 3, 6}

        for motor_id in Actuator.Index.UPPER:
            tripod_sign = -1.0 if motor_id in tripod_a else 1.0
            side_sign = 1.0 if motor_id % 2 == 1 else -1.0
            command_scale = float(
                np.clip(vx_scale + yaw_scale * side_sign, -1.0, 1.0)
            )
            reference[motor_id - 1] += (
                self.walk_config.legacy_stride_degrees
                * phase_sine
                * tripod_sign
                * command_scale
            )

        # In the original controller, middle_initial - 20 degrees lifts the
        # selected tripod.  The positive and negative half-cycles alternately
        # lift the two groups, with zero lift velocity at maximum clearance.
        lift_a = max(0.0, phase_sine) * activity
        lift_b = max(0.0, -phase_sine) * activity
        for upper_motor_id in Actuator.Index.UPPER:
            middle_motor_id = upper_motor_id + 6
            lift = lift_a if upper_motor_id in tripod_a else lift_b
            reference[middle_motor_id - 1] -= (
                self.walk_config.legacy_lift_degrees * lift
            )

        return reference

    def _apply_action(self, action: np.ndarray) -> None:
        clipped = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        reference = self._reference_motion_degrees()
        targets = reference + self.residual_scale_degrees * clipped

        # model.xml currently has no mechanical joint ranges.  This provisional
        # conservative clip must be replaced with measured SCONE hard stops.
        targets = np.clip(
            targets,
            self.default_degrees - 60.0,
            self.default_degrees + 60.0,
        )
        for motor_id, target in enumerate(targets, start=1):
            self.controller.set_position(motor_id, float(target))

    def _joint_state(self) -> tuple[np.ndarray, np.ndarray]:
        positions = np.array(
            [self.controller._joint_position(i) for i in Actuator.Index.ALL],
            dtype=np.float64,
        )
        velocities = np.array(
            [self.controller._joint_velocity(i) for i in Actuator.Index.ALL],
            dtype=np.float64,
        )
        return positions, velocities

    def _base_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        # Ask MuJoCo for a world-oriented spatial velocity, then explicitly
        # rotate it into the visible BODY_FRAME axes. Using flg_local=1 here
        # followed the exported freejoint/object frame convention, which does
        # not have SCONE's intended [forward, lateral, up] component order.
        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_BODY,
            self.root_body_id,
            self._body_velocity,
            0,
        )
        world_from_body = self.data.xmat[self.root_body_id].reshape(3, 3)
        angular_velocity = world_from_body.T @ self._body_velocity[:3]
        linear_velocity = world_from_body.T @ self._body_velocity[3:]
        projected_gravity = world_from_body.T @ np.array([0.0, 0.0, -1.0])
        return linear_velocity, angular_velocity, projected_gravity

    def _heading_yaw(self) -> float:
        world_from_body = self.data.xmat[self.root_body_id].reshape(3, 3)
        return float(math.atan2(world_from_body[1, 0], world_from_body[0, 0]))

    def _heading_error(self, current_heading: float | None = None) -> float:
        heading = self._heading_yaw() if current_heading is None else current_heading
        error = heading - self._target_heading
        return float(math.atan2(math.sin(error), math.cos(error)))

    def _observation(self) -> np.ndarray:
        linear_velocity, angular_velocity, gravity = self._base_state()
        joint_position, joint_velocity = self._joint_state()
        heading_error = self._heading_error()
        observation = np.concatenate(
            [
                linear_velocity / 2.0,
                angular_velocity / 5.0,
                gravity,
                (joint_position - self.default_radians) / math.pi,
                joint_velocity / 10.0,
                self._last_action,
                self._command / OBSERVATION_COMMAND_SCALE,
                np.array(
                    [
                        math.sin(2.0 * math.pi * self._phase),
                        math.cos(2.0 * math.pi * self._phase),
                        math.sin(heading_error),
                        math.cos(heading_error),
                    ]
                ),
            ]
        )
        return observation.astype(np.float32)

    def _contact_normal_force(self, contact_index: int) -> float:
        self._contact_force.fill(0.0)
        mujoco.mj_contactForce(
            self.model, self.data, contact_index, self._contact_force
        )
        return abs(float(self._contact_force[0]))

    def _slip_penalty(self) -> tuple[float, int]:
        """Measure tangential speed at the actual tire/floor contact point.

        This matches SCONE walking: the tip of an arc-sector is a planted foot
        during stance.  A swing leg has no contact and therefore receives no
        slip penalty.  Measuring the distal body's center velocity would be
        incorrect and would also penalize legitimate future rolling modes.
        """

        values: list[float] = []
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            if geom1 in self.ground_geom_ids and geom2 in self.tire_geom_ids:
                tire_geom = geom2
            elif geom2 in self.ground_geom_ids and geom1 in self.tire_geom_ids:
                tire_geom = geom1
            else:
                continue
            if self._contact_normal_force(contact_index) < self.walk_config.contact_force_threshold:
                continue

            body_id = self.tire_geom_to_body[tire_geom]
            self._jacobian_position.fill(0.0)
            self._jacobian_rotation.fill(0.0)
            mujoco.mj_jac(
                self.model,
                self.data,
                self._jacobian_position,
                self._jacobian_rotation,
                contact.pos,
                body_id,
            )
            point_velocity = self._jacobian_position @ self.data.qvel
            normal = np.asarray(contact.frame[:3], dtype=np.float64)
            tangential = point_velocity - normal * float(point_velocity @ normal)
            slip_speed = float(np.linalg.norm(tangential))
            excess = max(0.0, slip_speed - self.reward_config.slip_deadzone)
            values.append((excess / self.reward_config.slip_sigma) ** 2)

        if not values:
            return 0.0, 0
        return float(np.mean(values)), len(values)

    def _forbidden_floor_collision(self) -> bool:
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            if geom1 in self.ground_geom_ids:
                other = geom2
            elif geom2 in self.ground_geom_ids:
                other = geom1
            else:
                continue
            if other in self.tire_geom_ids:
                continue
            if self.model.geom_bodyid[other] == 0:
                continue
            if self._contact_normal_force(contact_index) >= self.walk_config.contact_force_threshold:
                return True
        return False

    def _normalized_current_penalty(self) -> float:
        normalized_currents = []
        for motor_id in Actuator.Index.ALL:
            actuator_id = int(self.controller._actuator_ids[motor_id])
            voltage = float(self.data.ctrl[actuator_id])
            velocity = self.controller._joint_velocity(motor_id)
            spec = spec_for_motor_id(motor_id)
            current = (voltage - spec.K * velocity) / spec.R
            stall_current = spec.stall_torque / spec.K
            normalized_currents.append(current / stall_current)
        return float(np.mean(np.square(normalized_currents)))

    def _reward(
        self, action: np.ndarray
    ) -> tuple[float, dict[str, float], bool, dict[str, float | int | bool]]:
        reward = self.reward_config
        linear_velocity, angular_velocity, gravity = self._base_state()
        joint_position, _ = self._joint_state()

        linear_error = linear_velocity[:2] - self._command[:2]
        linear_tracking = math.exp(
            -float(linear_error @ linear_error) / reward.linear_velocity_sigma**2
        )
        yaw_error = float(angular_velocity[2] - self._command[2])
        yaw_tracking = math.exp(-(yaw_error**2) / reward.yaw_velocity_sigma**2)
        heading_error = self._heading_error()
        heading_tracking = math.exp(
            -(heading_error**2) / reward.heading_error_sigma**2
        )
        upright = math.exp(
            -float(gravity[:2] @ gravity[:2]) / reward.projected_gravity_sigma**2
        )

        root_height = float(self.data.qpos[self.root_qpos_address + 2])
        height_drop = max(0.0, self._reference_height - root_height)
        height_penalty = (height_drop / reward.height_sigma) ** 2
        oscillation_penalty = (
            (linear_velocity[2] / reward.vertical_velocity_sigma) ** 2
            + (angular_velocity[0] / reward.roll_pitch_rate_sigma) ** 2
            + (angular_velocity[1] / reward.roll_pitch_rate_sigma) ** 2
        )
        action_rate_penalty = float(np.mean(np.square(action - self._last_action)))
        action_magnitude_penalty = float(np.mean(np.square(action)))
        activity = self._command_activity()
        idle_fraction = float(
            np.clip(
                1.0 - activity / reward.idle_activity_threshold,
                0.0,
                1.0,
            )
        )
        idle_velocity_tracking = math.exp(
            -float(linear_velocity[:2] @ linear_velocity[:2])
            / reward.idle_linear_velocity_sigma**2
            - float(angular_velocity[2] ** 2) / reward.idle_yaw_velocity_sigma**2
        )
        idle_velocity_term = (
            reward.idle_velocity_weight
            * idle_fraction
            * idle_velocity_tracking
            * self.control_dt
        )
        idle_action_penalty = idle_fraction * action_magnitude_penalty
        current_penalty = self._normalized_current_penalty()
        slip_penalty, stance_contacts = self._slip_penalty()

        joint_offset = np.abs(joint_position - self.default_radians)
        excess = np.maximum(0.0, joint_offset - reward.soft_joint_offset)
        joint_limit_penalty = float(
            np.mean(np.square(excess / math.radians(15.0)))
        )

        forbidden_collision = self._forbidden_floor_collision()
        collision_penalty = float(forbidden_collision)

        velocity_term = (
            reward.velocity_weight * linear_tracking * self.control_dt
            + idle_velocity_term
        )
        direction_term = (
            reward.yaw_weight * yaw_tracking * self.control_dt
            + reward.heading_weight * heading_tracking * self.control_dt
        )
        stability_term = (
            reward.upright_weight * upright * self.control_dt
            - reward.height_weight * height_penalty * self.control_dt
            - reward.oscillation_weight * oscillation_penalty * self.control_dt
            - reward.slip_weight * slip_penalty * self.control_dt
            - reward.joint_limit_weight * joint_limit_penalty * self.control_dt
            - reward.collision_weight * collision_penalty * self.control_dt
        )
        damping_term = (
            -reward.action_rate_weight * action_rate_penalty * self.control_dt
            - reward.action_magnitude_weight * action_magnitude_penalty * self.control_dt
            - reward.idle_action_weight * idle_action_penalty * self.control_dt
            - reward.current_weight * current_penalty * self.control_dt
        )

        raw_terms = {
            "velocity": velocity_term,
            "yaw": reward.yaw_weight * yaw_tracking * self.control_dt,
            "heading": reward.heading_weight * heading_tracking * self.control_dt,
            "upright": reward.upright_weight * upright * self.control_dt,
            "height": -reward.height_weight * height_penalty * self.control_dt,
            "oscillation": -reward.oscillation_weight * oscillation_penalty * self.control_dt,
            "action_rate": -reward.action_rate_weight * action_rate_penalty * self.control_dt,
            "action_magnitude": -reward.action_magnitude_weight
            * action_magnitude_penalty
            * self.control_dt,
            "idle_velocity": idle_velocity_term,
            "idle_action": -reward.idle_action_weight
            * idle_action_penalty
            * self.control_dt,
            "current": -reward.current_weight * current_penalty * self.control_dt,
            "slip": -reward.slip_weight * slip_penalty * self.control_dt,
            "joint_limit": -reward.joint_limit_weight * joint_limit_penalty * self.control_dt,
            "collision": -reward.collision_weight * collision_penalty * self.control_dt,
        }
        weighted_terms = {
            **raw_terms,
            "velocity": velocity_term,
            "direction": direction_term,
            "stability": stability_term,
            "damping": damping_term,
        }

        finite = bool(
            np.isfinite(self.data.qpos).all()
            and np.isfinite(self.data.qvel).all()
        )
        upright_cosine = -float(gravity[2])
        fallen = (
            upright_cosine < math.cos(math.radians(self.walk_config.max_tilt_degrees))
            or root_height < self._reference_height - self.walk_config.max_height_drop
        )
        hard_joint_limit = bool(np.any(joint_offset > reward.hard_joint_offset))
        terminated = (not finite) or fallen or forbidden_collision or hard_joint_limit
        total = velocity_term + direction_term + stability_term + damping_term
        if terminated:
            weighted_terms["termination"] = -reward.termination_penalty
            total -= reward.termination_penalty
        diagnostics: dict[str, float | int | bool] = {
            "vx": float(linear_velocity[0]),
            "vy": float(linear_velocity[1]),
            "yaw_rate": float(angular_velocity[2]),
            "heading_error": heading_error,
            "target_heading": self._target_heading,
            "height": root_height,
            "height_drop": height_drop,
            "command_activity": activity,
            "reference_cycle_frequency": self._reference_cycle_frequency,
            "reference_stride_clip_fraction": self._reference_stride_clip_fraction,
            "reference_ik_backoff_scale": self._reference_ik_backoff_scale,
            "stance_contacts": stance_contacts,
            "forbidden_collision": forbidden_collision,
            "fallen": fallen,
            "hard_joint_limit": hard_joint_limit,
        }
        return float(total), weighted_terms, terminated, diagnostics

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        del options
        mujoco.mj_resetData(self.model, self.data)
        self.controller = MuJoCoController(
            self.model,
            self.data,
            verbose=False,
            standing_pose_degrees=self.default_degrees,
        )
        self.controller.enable_torque()

        # Existing PPO checkpoints were trained with the simulation
        # controller's unlimited profile velocity/acceleration. Applying the
        # physical CLI profile only during replay changes the actuator
        # dynamics and breaks the learned phase relationship.

        self._phase = float(self.np_random.random())
        if self._non_rl_reference is not None:
            self._non_rl_reference.reset(
                phase=self._phase,
                motor_degrees=self.default_degrees,
            )
        self._episode_step = 0
        self._last_action.fill(0.0)
        self._reference_cycle_frequency = 0.0
        self._reference_stride_clip_fraction = 0.0
        self._reference_ik_backoff_scale = 1.0
        self._command.fill(0.0)
        self._command_target = self._sample_command()
        self._heading = self._heading_yaw()
        self._target_heading = self._heading
        self._schedule_next_command()
        if self.fixed_command is not None:
            self._command[:] = self.fixed_command

        # Let contacts settle while holding the stable Standard pose.  No
        # reward is accumulated during this reset-only physical transient.
        settle_steps = round(
            self.walk_config.settle_seconds / self.model.opt.timestep
        )
        for _ in range(settle_steps):
            self.controller.update(self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)

        self._reference_height = float(
            self.data.qpos[self.root_qpos_address + 2]
        )
        observation = self._observation()
        return observation, {
            "command": self._command.copy(),
            "command_target": self._command_target.copy(),
        }

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action64 = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        self._update_command()
        self._target_heading += self._command[2] * self.control_dt
        self._advance_phase()
        self._apply_action(action64)

        for _ in range(self.walk_config.frame_skip):
            self.controller.update(self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)

        reward, terms, terminated, diagnostics = self._reward(action64)
        self._last_action[:] = action64
        self._episode_step += 1
        truncated = self._episode_step >= self.max_episode_steps

        info: dict[str, Any] = {
            "reward_terms": terms,
            "command": self._command.copy(),
            **diagnostics,
        }
        observation = self._observation()
        if self.render_mode == "human":
            self.render()
        return observation, reward, terminated, truncated, info

    def advance_external_control(self) -> None:
        """Advance one policy-sized frame without writing RL joint targets.

        RL joystick mode uses this while the shared controller is executing a
        blocking Walk/Drive/Climb transition on its input thread.  Keeping the
        physics loop here prevents the policy from overwriting those targets.
        """

        for _ in range(self.walk_config.frame_skip):
            with self.controller.lock:
                self.controller.update(self.model.opt.timestep)
                mujoco.mj_step(self.model, self.data)
        if self.render_mode == "human":
            self.render()

    def resume_after_external_control(self) -> np.ndarray:
        """Re-align policy state after legacy modes return to the Walk pose."""

        self.fixed_command = np.zeros(3, dtype=np.float64)
        self._command.fill(0.0)
        self._command_target.fill(0.0)
        self._last_action.fill(0.0)
        self._reference_cycle_frequency = 0.0
        self._reference_stride_clip_fraction = 0.0
        self._reference_ik_backoff_scale = 1.0
        self._heading = self._heading_yaw()
        self._target_heading = self._heading
        self._reference_height = float(
            self.data.qpos[self.root_qpos_address + 2]
        )
        if self._non_rl_reference is not None:
            self._non_rl_reference.reset(
                phase=self._phase,
                motor_degrees=self.default_degrees,
            )
        return self._observation()

    def render(self) -> None:
        if self.render_mode != "human":
            return
        if self._viewer is None:
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
            configure_simulation_viewer(
                self._viewer,
                self.model,
                self.data,
                tracking_body_id=self.root_body_id,
            )
        if self._viewer.is_running():
            self._viewer.sync()

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        if hasattr(self, "controller"):
            self.controller.close()


class RewardTermsCallback(BaseCallback):
    """Write each reward component and measured velocity to the PPO logger."""

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            for name, value in info.get("reward_terms", {}).items():
                self.logger.record_mean(f"reward/{name}", float(value))
            for name in (
                "vx",
                "vy",
                "yaw_rate",
                "height",
                "height_drop",
                "reference_cycle_frequency",
                "reference_stride_clip_fraction",
                "reference_ik_backoff_scale",
                "stance_contacts",
            ):
                if name in info:
                    self.logger.record_mean(f"state/{name}", float(info[name]))
        return True


class GracefulStopCallback(BaseCallback):
    """Finish ``learn`` cleanly when SIGINT or SIGTERM requests a pause."""

    def __init__(self, stop_requested: Callable[[], bool]) -> None:
        super().__init__()
        self.stop_requested = stop_requested

    def _on_step(self) -> bool:
        return not self.stop_requested()


def _write_resume_pointer(run_dir: Path, checkpoint: Path) -> None:
    """Atomically record the exact checkpoint that a resume should load."""

    try:
        stored_path = checkpoint.relative_to(run_dir)
    except ValueError:
        stored_path = checkpoint
    pointer = run_dir / "resume.checkpoint"
    temporary = pointer.with_suffix(".tmp")
    temporary.write_text(f"{stored_path}\n", encoding="utf-8")
    temporary.replace(pointer)


class PruningCheckpointCallback(CheckpointCallback):
    """Save PPO checkpoints and retain only the newest files."""

    def __init__(self, *args: Any, keep_last: int = 10, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        if keep_last < 1:
            raise ValueError("keep_last must be at least 1")
        self.keep_last = keep_last

    def _on_step(self) -> bool:
        result = super()._on_step()
        if self.n_calls % self.save_freq == 0:
            saved_checkpoint = (
                Path(self.save_path)
                / f"{self.name_prefix}_{self.num_timesteps}_steps.zip"
            )
            if saved_checkpoint.is_file():
                _write_resume_pointer(Path(self.save_path).parent, saved_checkpoint)
            checkpoints: list[tuple[int, Path]] = []
            pattern = f"{self.name_prefix}_*_steps.zip"
            for path in Path(self.save_path).glob(pattern):
                match = re.search(r"_([0-9]+)_steps\.zip$", path.name)
                if match is not None:
                    checkpoints.append((int(match.group(1)), path))
            checkpoints.sort(key=lambda item: item[0])
            for _, path in checkpoints[:-self.keep_last]:
                path.unlink(missing_ok=True)
        return result


def _build_env(
    model_path: Path,
    curriculum: str,
    fixed_command: list[float] | None = None,
    render_mode: str | None = None,
    terrain: TerrainType | str = TerrainType.FLAT,
    terrain_seed: int = 7,
    standing_pose_degrees: Sequence[float] = SPORT_STANDING_DEGREES,
    reference_motion: str = "hardcoded",
) -> SconeWalkEnv:
    return SconeWalkEnv(
        model_path,
        curriculum=curriculum,
        fixed_command=fixed_command,
        render_mode=render_mode,
        terrain=terrain,
        terrain_seed=terrain_seed,
        standing_pose_degrees=standing_pose_degrees,
        reference_motion=reference_motion,
    )


def _make_vector_env(
    factories: Sequence[Callable[[], SconeWalkEnv]],
) -> DummyVecEnv | SubprocVecEnv:
    """Use subprocesses only when more than one environment was requested."""

    if not factories:
        raise ValueError("at least one environment factory is required")
    if len(factories) == 1:
        return DummyVecEnv(list(factories))
    return SubprocVecEnv(list(factories))


def run_check(args: argparse.Namespace) -> int:
    env = _build_env(
        args.model,
        args.curriculum,
        terrain=args.terrain,
        terrain_seed=args.terrain_seed,
        standing_pose_degrees=args.standing_pose_degrees,
        reference_motion=args.reference_motion,
    )
    check_env(env, warn=True, skip_render_check=True)
    observation, _ = env.reset(seed=args.seed)
    totals: dict[str, float] = {}
    completed = 0
    try:
        for _ in range(args.steps):
            action = (
                env.action_space.sample()
                if args.random_actions
                else np.zeros(18, dtype=np.float32)
            )
            observation, _, terminated, truncated, info = env.step(action)
            if not np.isfinite(observation).all():
                raise RuntimeError("non-finite observation produced")
            for name, value in info["reward_terms"].items():
                totals[name] = totals.get(name, 0.0) + float(value)
            completed += 1
            if terminated or truncated:
                observation, _ = env.reset()
    finally:
        env.close()

    print(f"walk environment check passed: {completed} policy steps")
    if completed:
        print("mean weighted reward terms per policy step:")
        for name in sorted(totals):
            print(f"  {name:>14}: {totals[name] / completed:+.6f}")
    return 0


def run_train(args: argparse.Namespace) -> int:
    run_dir = args.output.expanduser().resolve()
    run_dir.mkdir(parents=True, exist_ok=True)

    factories = [
        (
            lambda environment_index=environment_index: _build_env(
                args.model,
                args.curriculum,
                terrain=args.terrain,
                terrain_seed=args.terrain_seed + environment_index,
                standing_pose_degrees=args.standing_pose_degrees,
                reference_motion=args.reference_motion,
            )
        )
        for environment_index in range(args.num_envs)
    ]
    monitor_name = (
        "monitor.csv"
        if args.resume is None
        else f"monitor_resume_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    )
    # DummyVecEnv only interleaves environments in one process. Use real
    # subprocess workers whenever the CLI requests parallelism so the SSH
    # CPU/memory recommendation translates into actual MuJoCo throughput.
    vector_env = _make_vector_env(factories)
    vec_env = VecMonitor(vector_env, filename=str(run_dir / monitor_name))
    checkpoint = PruningCheckpointCallback(
        save_freq=max(1, args.checkpoint_every // args.num_envs),
        save_path=str(run_dir / "checkpoints"),
        name_prefix="scone_walk",
        keep_last=args.keep_checkpoints,
    )
    stop_requested = False

    def request_stop(signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        if not stop_requested:
            print(
                f"[RL] signal {signum} received; finishing the current step "
                "and saving a resume checkpoint...",
                flush=True,
            )
        stop_requested = True

    callbacks = [
        checkpoint,
        RewardTermsCallback(),
        GracefulStopCallback(lambda: stop_requested),
    ]

    tensorboard_log = (
        None
        if args.tensorboard_log is None
        else str(args.tensorboard_log.expanduser().resolve())
    )
    if args.resume is None:
        model = PPO(
            "MlpPolicy",
            vec_env,
            learning_rate=3e-4,
            n_steps=1024,
            batch_size=256,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.005,
            max_grad_norm=1.0,
            policy_kwargs={"net_arch": {"pi": [256, 256], "vf": [256, 256]}},
            tensorboard_log=tensorboard_log,
            verbose=1,
            seed=args.seed,
            device=args.device,
        )
        reset_num_timesteps = True
    else:
        model = PPO.load(
            args.resume.expanduser().resolve(),
            env=vec_env,
            device=args.device,
            tensorboard_log=tensorboard_log,
        )
        reset_num_timesteps = False
    previous_signal_handlers = {
        signum: signal.signal(signum, request_stop)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=callbacks,
            reset_num_timesteps=reset_num_timesteps,
        )
        final_model = run_dir / "final_model.zip"
        model.save(final_model)
        _write_resume_pointer(run_dir, final_model)
    finally:
        for signum, previous_handler in previous_signal_handlers.items():
            signal.signal(signum, previous_handler)
        vec_env.close()
    print(f"saved trained policy to {final_model}")
    return 0


def run_enjoy(args: argparse.Namespace) -> int:
    env = _build_env(
        args.model,
        args.curriculum,
        fixed_command=args.command,
        render_mode="human",
        terrain=args.terrain,
        terrain_seed=args.terrain_seed,
        standing_pose_degrees=args.standing_pose_degrees,
        reference_motion=args.reference_motion,
    )
    model = load_compatible_policy(args.checkpoint, env, args.device)
    observation, _ = env.reset(seed=args.seed)
    neutral_gate = NeutralResidualGate()
    episodes = 0
    try:
        while episodes < args.episodes:
            frame_start = time.perf_counter()
            policy_observation = observation_for_policy(model, observation)
            policy_action, _ = model.predict(
                policy_observation, deterministic=True
            )
            action = (
                policy_action
                if args.raw_policy
                else neutral_gate.apply(args.command, policy_action, env.control_dt)
            )
            observation, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                episodes += 1
                observation, _ = env.reset()
                neutral_gate.reset()
            remaining = env.control_dt - (time.perf_counter() - frame_start)
            if remaining > 0:
                time.sleep(remaining)
            if env._viewer is not None and not env._viewer.is_running():
                break
    finally:
        env.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Train and inspect the SCONE command-conditioned Walk policy."
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL_PATH,
        help="MJCF path (default: model.xml)",
    )
    parser.add_argument(
        "--terrain",
        choices=TERRAIN_CHOICES,
        default=TerrainType.FLAT.value,
        help="procedural terrain preset",
    )
    parser.add_argument(
        "--terrain-seed",
        type=int,
        default=7,
        help="base seed; vectorized training offsets it per environment",
    )
    parser.add_argument(
        "--standing-pose-degrees",
        type=float,
        nargs=18,
        metavar="DEG",
        default=SPORT_STANDING_DEGREES,
        help=(
            "standing motor degrees for actuator IDs 1..18 "
            "(default: legacy Sport RL stance)"
        ),
    )
    parser.add_argument(
        "--reference-motion",
        choices=REFERENCE_MOTION_CHOICES,
        default=None,
        help=(
            "baseline motion under the residual policy; defaults to non_rl "
            "for check/train and hardcoded for legacy-compatible enjoy replay"
        ),
    )
    subparsers = parser.add_subparsers(dest="command_name", required=True)

    check = subparsers.add_parser("check", help="Validate reset, step, rewards and contacts")
    check.add_argument("--steps", type=int, default=500)
    check.add_argument("--curriculum", choices=tuple(CURRICULUM_RANGES), default="easy")
    check.add_argument("--seed", type=int, default=0)
    check.add_argument("--random-actions", action="store_true")
    check.set_defaults(handler=run_check)

    train = subparsers.add_parser("train", help="Train a PPO residual policy")
    train.add_argument("--timesteps", type=int, default=1_000_000)
    train.add_argument("--curriculum", choices=tuple(CURRICULUM_RANGES), default="easy")
    train.add_argument("--num-envs", type=int, default=4)
    train.add_argument("--seed", type=int, default=0)
    train.add_argument("--device", default="auto")
    train.add_argument("--checkpoint-every", type=int, default=100_000)
    train.add_argument("--keep-checkpoints", type=int, default=10)
    train.add_argument("--output", type=Path, default=Path("runs/scone_walk"))
    train.add_argument("--tensorboard-log", type=Path)
    train.add_argument(
        "--resume",
        type=Path,
        help="Continue training from a PPO .zip checkpoint",
    )
    train.set_defaults(handler=run_train)

    enjoy = subparsers.add_parser("enjoy", help="Preview a trained policy")
    enjoy.add_argument("checkpoint", type=Path)
    enjoy.add_argument(
        "--command",
        type=float,
        nargs=3,
        metavar=("VX", "VY", "YAW_RATE"),
        default=[0.25, 0.0, 0.0],
    )
    enjoy.add_argument("--curriculum", choices=tuple(CURRICULUM_RANGES), default="full")
    enjoy.add_argument("--episodes", type=int, default=3)
    enjoy.add_argument("--seed", type=int, default=0)
    enjoy.add_argument("--device", default="auto")
    enjoy.add_argument(
        "--raw-policy",
        action="store_true",
        help="disable the neutral-command residual gate for policy diagnosis",
    )
    enjoy.set_defaults(handler=run_enjoy)
    return parser


def _resolve_reference_motion(
    command_name: str,
    selected: str | None,
) -> str:
    """Keep new training on Non-RL while replaying legacy PPO correctly."""

    if selected is not None:
        return selected
    return "hardcoded" if command_name == "enjoy" else "non_rl"


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    args.reference_motion = _resolve_reference_motion(
        args.command_name,
        args.reference_motion,
    )
    args.model = args.model.expanduser().resolve()
    if not args.model.exists():
        raise SystemExit(f"Model not found: {args.model}")
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
