"""Redesigned walking-PPO environment for SCONE.

This module is a clean second design; ``walk_learn.py`` is left untouched so
existing checkpoints keep replaying. The differences that matter are:

* **Canonical frame.** Observations, commands and rewards use x forward,
  y left, z up (REP-103). The exported chassis frame is rotated 180 degrees
  from that, so a single constant converts between them (see
  ``CANONICAL_FROM_CHASSIS``).
* **Left/right symmetry is enforced by construction.** The kinematic model was
  verified mirror-symmetric (all nine joint-axis pairs are exact axial-vector
  mirrors), so an episode-level random mirror makes the policy see both sides
  of every situation.
* **Every foot is observable and accountable.** Contact flags and normalized
  normal forces enter the observation; air time, inactivity and load sharing
  enter the reward. A leg that contributes nothing is now penalized.
* **Soft landings are rewarded.** The rate of rise of each foot's normal force
  is penalized, so hard touchdowns cost score.
* **The reference gait can actually reach the commanded speed**, which the
  first design could not: its reference saturated at 0.084 m/s against
  commands up to 0.50 m/s, leaving no gradient toward speed.
* **Randomization exists.** Initial pose, heading, mass, friction, actuator
  strength, observation noise, action delay and pushes.

See docs/18-actuator-model-and-frame-convention.md for the measurements this
design is based on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from ..hardware import Actuator
from ..locomotion import TripodGait, VelocityCommand
from ..locomotion.tripod_gait import GaitConfig
from ..simulation.core.controller import MuJoCoController
from ..simulation.core.model import load_model
from ..simulation.terrain import TerrainType
from .motion_profile import motion_profile_for_standing_pose
from .stance import STANCE_PRESETS, validate_standing_pose


DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[1] / "assets" / "model.xml"

# ---------------------------------------------------------------------------
# Frame convention
# ---------------------------------------------------------------------------
# The exported chassis frame (body UPPER_BODY_1) has +x toward the rear and +y
# toward the right:
#   * internal payloads are laid out BATTERY_L2_L1 (x=-0.005) -> RASPBERRY_PI ->
#     BATTERY_L4_L3 -> BATTERY_L6_L5 (x=+0.255) and model.xml documents that
#     ordering as front-to-rear, so leg pair 1/2 is the front pair at -x;
#   * legs 1, 3, 5 are named and coloured as the right side and sit at +y,
#     which is the left half-space in a right-handed x-forward frame.
# Both readings agree that the chassis frame is the canonical frame yawed by
# 180 degrees. Flip this to the identity if the intended front is the +x end.
CANONICAL_FROM_CHASSIS = np.diag([-1.0, -1.0, 1.0])

RIGHT_LEGS = (1, 3, 5)
LEFT_LEGS = (2, 4, 6)
# Alternating tripods: each set is one leg per longitudinal station, sides
# alternating. {2, 4, 6} is NOT a tripod -- it is the whole left side.
TRIPOD_A = (1, 4, 5)
TRIPOD_B = (2, 3, 6)
# Mirror pairs leg n with its opposite-side twin. The model is an exact axial
# mirror and the degree->radian map carries no per-motor sign, so mirroring is
# a pure permutation with no sign change.
LEG_MIRROR = {1: 2, 2: 1, 3: 4, 4: 3, 5: 6, 6: 5}
JOINT_MIRROR = np.array(
    [(LEG_MIRROR[(i % 6) + 1] - 1) + 6 * (i // 6) for i in range(18)],
    dtype=np.int64,
)
FOOT_MIRROR = np.array([LEG_MIRROR[i + 1] - 1 for i in range(6)], dtype=np.int64)

REFERENCE_CHOICES = ("tripod-gait", "hardcoded", "none")


@dataclass(frozen=True)
class RewardConfig:
    """Reward weights and tolerances. All terms are scaled by the control step."""

    # tracking
    linear_velocity_sigma: float = 0.20
    yaw_velocity_sigma: float = 0.25
    heading_error_sigma: float = 0.20          # was 0.60 rad (34 deg) -- far too loose
    velocity_weight: float = 2.0
    yaw_weight: float = 0.8
    heading_weight: float = 1.0
    # A linear progress term that does not saturate, so the policy still gains
    # from going faster once it is inside the exponential's shoulder.
    progress_weight: float = 1.5

    # posture
    projected_gravity_sigma: float = 0.25
    upright_weight: float = 0.6
    target_height: float = 0.46                # m, measured optimum (see docs)
    height_sigma: float = 0.06
    height_weight: float = 0.4
    vertical_velocity_sigma: float = 0.30
    roll_pitch_rate_sigma: float = 0.80
    oscillation_weight: float = 0.10

    # gait quality
    air_time_target: float = 0.25              # s
    air_time_weight: float = 0.6
    inactivity_seconds: float = 1.0
    inactivity_weight: float = 1.0
    load_share_weight: float = 0.3
    impact_weight: float = 0.5
    impact_reference_force: float = 40.0       # N, about one body weight
    slip_deadzone: float = 0.02
    slip_sigma: float = 0.20
    slip_weight: float = 0.15

    # effort
    action_rate_weight: float = 0.05
    action_magnitude_weight: float = 0.05      # was 0.25: it paid for idle legs
    torque_weight: float = 0.02
    joint_limit_weight: float = 0.3
    soft_joint_offset: float = math.radians(55.0)
    hard_joint_offset: float = math.radians(85.0)
    collision_weight: float = 1.0
    termination_penalty: float = 5.0


@dataclass(frozen=True)
class WalkConfig:
    physics_timestep: float = 0.002
    frame_skip: int = 10                       # 50 Hz policy
    episode_seconds: float = 20.0
    settle_seconds: float = 0.25
    command_filter_seconds: float = 0.30
    command_hold_seconds: tuple[float, float] = (2.0, 5.0)
    idle_command_probability: float = 0.15
    max_tilt_degrees: float = 60.0
    max_height_drop: float = 0.15
    contact_force_threshold: float = 1.0
    stance_preset: str = "standard"            # 240/255: the measured fast posture

    # symmetry and robustness
    mirror_probability: float = 0.5
    initial_joint_noise_degrees: float = 4.0
    initial_yaw_randomization: bool = True
    push_interval_seconds: tuple[float, float] = (2.0, 5.0)
    push_velocity: float = 0.25                # m/s impulse on the base
    observation_noise: float = 0.01
    action_delay_probability: float = 0.5      # one control step of delay
    mass_scale_range: tuple[float, float] = (0.90, 1.10)
    friction_scale_range: tuple[float, float] = (0.70, 1.30)
    strength_scale_range: tuple[float, float] = (0.85, 1.15)


# Command ranges [|vx|, |vy|, |yaw|]. The reference gait is configured to be
# able to reach the top of each stage; that was the missing piece before.
CURRICULUM_RANGES: dict[str, np.ndarray] = {
    "easy": np.array([0.12, 0.00, 0.00]),
    "medium": np.array([0.22, 0.08, 0.50]),
    "full": np.array([0.35, 0.15, 0.90]),
}
OBSERVATION_COMMAND_SCALE = np.array([0.35, 0.15, 0.90])

_OBS_DIM = 82


def mirror_observation(obs: np.ndarray) -> np.ndarray:
    """Reflect one observation through the robot's sagittal plane."""
    o = np.array(obs, dtype=np.float64, copy=True)
    o[0:3] *= (1.0, -1.0, 1.0)                 # linear velocity (polar)
    o[3:6] *= (-1.0, 1.0, -1.0)                # angular velocity (axial)
    o[6:9] *= (1.0, -1.0, 1.0)                 # projected gravity (polar)
    o[9:27] = o[9:27][JOINT_MIRROR]
    o[27:45] = o[27:45][JOINT_MIRROR]
    o[45:63] = o[45:63][JOINT_MIRROR]
    o[63:66] *= (1.0, -1.0, -1.0)              # command
    o[66:68] *= -1.0                           # phase: the mirror swaps tripods
    o[68] *= -1.0                              # sin(heading error)
    o[70:76] = o[70:76][FOOT_MIRROR]           # contact flags
    o[76:82] = o[76:82][FOOT_MIRROR]           # normal forces
    return o


def mirror_action(action: np.ndarray) -> np.ndarray:
    return np.asarray(action, dtype=np.float64)[JOINT_MIRROR]


class SconeWalkEnvV2(gym.Env[np.ndarray, np.ndarray]):
    """Residual (or end-to-end) walking environment in the canonical frame."""

    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        model_path: Path | str = DEFAULT_MODEL_PATH,
        *,
        curriculum: str = "easy",
        reference_motion: str = "tripod-gait",
        fixed_command: Sequence[float] | None = None,
        reward_config: RewardConfig | None = None,
        walk_config: WalkConfig | None = None,
        terrain: TerrainType | str = TerrainType.FLAT,
        terrain_seed: int = 7,
        render_mode: str | None = None,
    ) -> None:
        super().__init__()
        if curriculum not in CURRICULUM_RANGES:
            raise ValueError(f"unknown curriculum {curriculum!r}")
        if reference_motion not in REFERENCE_CHOICES:
            raise ValueError(f"unknown reference motion {reference_motion!r}")

        self.reward_config = reward_config or RewardConfig()
        self.walk_config = walk_config or WalkConfig()
        self.curriculum = curriculum
        self.command_range = CURRICULUM_RANGES[curriculum].copy()
        self.reference_motion = reference_motion
        self.render_mode = render_mode
        self.model_path = Path(model_path).expanduser().resolve()
        self.terrain = TerrainType.parse(terrain)
        self.terrain_seed = terrain_seed
        self.fixed_command = (
            None if fixed_command is None
            else np.asarray(fixed_command, dtype=np.float64)
        )

        self.model = load_model(
            self.model_path, terrain=self.terrain, terrain_seed=terrain_seed
        )
        self.model.opt.timestep = self.walk_config.physics_timestep
        self.data = mujoco.MjData(self.model)
        self.control_dt = self.model.opt.timestep * self.walk_config.frame_skip
        self.max_episode_steps = round(
            self.walk_config.episode_seconds / self.control_dt
        )

        self._nominal_geom_friction = self.model.geom_friction.copy()
        self._nominal_body_mass = self.model.body_mass.copy()

        self.root_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "root_freejoint"
        )
        self.root_body_id = int(self.model.jnt_bodyid[self.root_joint_id])
        self.root_qpos_address = int(self.model.jnt_qposadr[self.root_joint_id])
        self.root_dof_address = int(self.model.jnt_dofadr[self.root_joint_id])

        self.floor_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "simulation_floor"
        )
        self.ground_geom_ids = {self.floor_geom_id}
        for geom_id in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            if name and name.startswith("terrain_"):
                self.ground_geom_ids.add(geom_id)

        self.foot_geom_ids: list[int] = []
        for leg in range(1, 7):
            geom_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, f"TIRE_{leg}_geom"
            )
            if geom_id < 0:
                raise ValueError(f"missing contact geom TIRE_{leg}_geom")
            self.foot_geom_ids.append(geom_id)
        self._foot_index = {g: i for i, g in enumerate(self.foot_geom_ids)}

        self.default_degrees = np.asarray(
            validate_standing_pose(STANCE_PRESETS[self.walk_config.stance_preset]),
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
        self._motion_profile = motion_profile_for_standing_pose(self.default_degrees)
        self.residual_scale_degrees = np.array(
            [14.0] * 6 + [16.0] * 6 + [18.0] * 6, dtype=np.float64
        )

        self._reference_gait: TripodGait | None = None
        if self.reference_motion == "tripod-gait":
            self._reference_gait = TripodGait(
                profile=self._motion_profile,
                model_path=self.model_path,
                config=GaitConfig(
                    control_frequency=1.0 / self.control_dt,
                    # Sized so the reference can reach the top of the command
                    # range: u_max = stride * f / duty = 0.11 * 1.2 / 0.5.
                    cycle_frequency=1.2,
                    duty_factor=0.5,
                    step_height=0.030,
                    max_stride=0.110,
                    max_lateral_stride=0.070,
                    max_vx=float(OBSERVATION_COMMAND_SCALE[0]),
                    max_vy=float(OBSERVATION_COMMAND_SCALE[1]),
                    max_yaw_rate=float(OBSERVATION_COMMAND_SCALE[2]),
                    command_time_constant=0.0,
                    ik_tolerance=1e-3,
                    ik_stride_backoff_attempts=4,
                ),
            )
            self._reference_gait.reset(motor_degrees=self.default_degrees)

        self.action_space = spaces.Box(-1.0, 1.0, shape=(18,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(_OBS_DIM,), dtype=np.float32
        )

        self._phase = 0.0
        self._episode_step = 0
        self._command = np.zeros(3)
        self._command_target = np.zeros(3)
        self._next_command_step = 0
        self._next_push_step = 10 ** 9
        self._target_heading = 0.0
        self._last_action = np.zeros(18)
        self._pending_action = np.zeros(18)
        self._delay_action = False
        self._mirrored = False
        self._strength_scale = 1.0
        self._air_time = np.zeros(6)
        self._contact_seconds_since = np.zeros(6)
        self._previous_normal = np.zeros(6)
        self._reference_height = 0.0
        self._contact_force = np.zeros(6)
        self._viewer: Any | None = None

    # -- frame helpers ------------------------------------------------------

    def _base_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return canonical-frame linear velocity, angular velocity, gravity."""
        buffer = np.zeros(6)
        mujoco.mj_objectVelocity(
            self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
            self.root_body_id, buffer, 0,
        )
        world_from_chassis = self.data.xmat[self.root_body_id].reshape(3, 3)
        angular = CANONICAL_FROM_CHASSIS @ (world_from_chassis.T @ buffer[:3])
        linear = CANONICAL_FROM_CHASSIS @ (world_from_chassis.T @ buffer[3:])
        gravity = CANONICAL_FROM_CHASSIS @ (
            world_from_chassis.T @ np.array([0.0, 0.0, -1.0])
        )
        return linear, angular, gravity

    def _canonical_yaw(self) -> float:
        world_from_chassis = self.data.xmat[self.root_body_id].reshape(3, 3)
        forward = world_from_chassis @ CANONICAL_FROM_CHASSIS[:, 0]
        return float(math.atan2(forward[1], forward[0]))

    def _heading_error(self) -> float:
        error = self._canonical_yaw() - self._target_heading
        return float(math.atan2(math.sin(error), math.cos(error)))

    # -- command ------------------------------------------------------------

    def _sample_command(self) -> np.ndarray:
        if self.fixed_command is not None:
            return self.fixed_command.copy()
        if self.np_random.random() < self.walk_config.idle_command_probability:
            return np.zeros(3)
        command = self.np_random.uniform(-self.command_range, self.command_range)
        if self.curriculum == "easy":
            command[0] = self.np_random.uniform(-0.04, self.command_range[0])
        return command

    def _update_command(self) -> None:
        if self.fixed_command is not None:
            self._command[:] = self.fixed_command
            return
        if self._episode_step >= self._next_command_step:
            self._command_target = self._sample_command()
            hold = self.np_random.uniform(*self.walk_config.command_hold_seconds)
            self._next_command_step = self._episode_step + max(
                1, round(hold / self.control_dt)
            )
        alpha = 1.0 - math.exp(
            -self.control_dt / self.walk_config.command_filter_seconds
        )
        self._command += alpha * (self._command_target - self._command)

    def _command_activity(self) -> float:
        return float(
            np.clip(np.max(np.abs(self._command / OBSERVATION_COMMAND_SCALE)), 0.0, 1.0)
        )

    # -- contacts -----------------------------------------------------------

    def _foot_contacts(self) -> tuple[np.ndarray, np.ndarray, float]:
        """Return per-foot normal force, tangential slip speed and mean slip."""
        normal = np.zeros(6)
        slip = np.zeros(6)
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 in self._foot_index and geom2 in self.ground_geom_ids:
                foot = self._foot_index[geom1]
            elif geom2 in self._foot_index and geom1 in self.ground_geom_ids:
                foot = self._foot_index[geom2]
            else:
                continue
            mujoco.mj_contactForce(self.model, self.data, index, self._contact_force)
            force = abs(float(self._contact_force[0]))
            if force < self.walk_config.contact_force_threshold:
                continue
            normal[foot] += force
            frame = np.asarray(contact.frame).reshape(3, 3)
            velocity = np.zeros(6)
            mujoco.mj_objectVelocity(
                self.model, self.data, mujoco.mjtObj.mjOBJ_GEOM,
                geom1 if geom1 in self._foot_index else geom2, velocity, 0,
            )
            tangential = velocity[3:] - float(velocity[3:] @ frame[0]) * frame[0]
            slip[foot] = max(slip[foot], float(np.linalg.norm(tangential)))
        active = normal > 0.0
        mean_slip = float(
            np.mean(np.maximum(0.0, slip[active] - self.reward_config.slip_deadzone))
        ) if active.any() else 0.0
        return normal, slip, mean_slip

    # -- observation --------------------------------------------------------

    def _joint_state(self) -> tuple[np.ndarray, np.ndarray]:
        position = np.array(
            [self.controller._joint_position(i) for i in Actuator.Index.ALL]
        )
        velocity = np.array(
            [self.controller._joint_velocity(i) for i in Actuator.Index.ALL]
        )
        return position, velocity

    def _observation(self, normal: np.ndarray) -> np.ndarray:
        linear, angular, gravity = self._base_state()
        position, velocity = self._joint_state()
        heading_error = self._heading_error()
        weight = float(self._nominal_body_mass.sum()) * 9.81
        observation = np.concatenate([
            linear / 2.0,
            angular / 5.0,
            gravity,
            (position - self.default_radians) / math.pi,
            velocity / 10.0,
            self._last_action,
            self._command / OBSERVATION_COMMAND_SCALE,
            [math.sin(2 * math.pi * self._phase), math.cos(2 * math.pi * self._phase)],
            [math.sin(heading_error), math.cos(heading_error)],
            (normal > 0.0).astype(np.float64),
            np.clip(normal / (weight / 3.0), 0.0, 2.0),
        ])
        noise = self.walk_config.observation_noise
        if noise > 0.0:
            observation = observation + self.np_random.normal(0.0, noise, observation.shape)
        if self._mirrored:
            observation = mirror_observation(observation)
        return observation.astype(np.float32)

    # -- reference and action ----------------------------------------------

    def _reference_degrees(self) -> np.ndarray:
        if self._reference_gait is not None:
            # The gait works in the chassis frame; the command is canonical.
            chassis_command = np.array([
                -self._command[0], -self._command[1], self._command[2]
            ])
            sample = self._reference_gait.step(chassis_command, self.control_dt)
            self._phase = sample.phase
            return sample.motor_degrees.copy()

        reference = self.default_degrees.copy()
        if self.reference_motion == "none":
            return reference

        activity = self._command_activity()
        frequency = 0.6 + activity * 0.9
        self._phase = (self._phase + frequency * self.control_dt) % 1.0
        phase_sine = math.sin(2.0 * math.pi * self._phase)
        # Canonical +vx must advance the chassis toward its own -x, which the
        # legacy stride convention produces with a positive offset.
        forward_scale = float(np.clip(self._command[0] / 0.35, -1.0, 1.0))
        yaw_scale = float(np.clip(self._command[2] / 0.90, -1.0, 1.0))
        lead = set(TRIPOD_B)
        for motor_id in Actuator.Index.UPPER:
            tripod_sign = -1.0 if motor_id in lead else 1.0
            side_sign = 1.0 if motor_id in RIGHT_LEGS else -1.0
            scale = float(np.clip(forward_scale + yaw_scale * side_sign, -1.0, 1.0))
            reference[motor_id - 1] += 22.0 * phase_sine * tripod_sign * scale
        lift_a = max(0.0, phase_sine) * activity
        lift_b = max(0.0, -phase_sine) * activity
        for motor_id in Actuator.Index.UPPER:
            lift = lift_a if motor_id in lead else lift_b
            reference[motor_id + 5] -= 22.0 * lift
        return reference

    def _apply_action(self, action: np.ndarray) -> None:
        reference = self._reference_degrees()
        targets = reference + self.residual_scale_degrees * action
        targets = np.clip(
            targets, self.default_degrees - 65.0, self.default_degrees + 65.0
        )
        for motor_id, target in enumerate(targets, start=1):
            self.controller.set_position(motor_id, float(target))

    # -- reward -------------------------------------------------------------

    def _reward(
        self, action: np.ndarray, normal: np.ndarray, mean_slip: float
    ) -> tuple[float, dict[str, float], bool, dict[str, Any]]:
        cfg = self.reward_config
        dt = self.control_dt
        linear, angular, gravity = self._base_state()
        position, _ = self._joint_state()

        error = linear[:2] - self._command[:2]
        velocity_tracking = math.exp(-float(error @ error) / cfg.linear_velocity_sigma ** 2)
        command_speed = float(np.linalg.norm(self._command[:2]))
        if command_speed > 1e-6:
            direction = self._command[:2] / command_speed
            progress = float(np.clip(linear[:2] @ direction, 0.0, None))
        else:
            progress = 0.0
        yaw_error = float(angular[2] - self._command[2])
        yaw_tracking = math.exp(-yaw_error ** 2 / cfg.yaw_velocity_sigma ** 2)
        heading_error = self._heading_error()
        heading_tracking = math.exp(-heading_error ** 2 / cfg.heading_error_sigma ** 2)
        upright = math.exp(
            -float(gravity[:2] @ gravity[:2]) / cfg.projected_gravity_sigma ** 2
        )

        height = float(self.data.qpos[self.root_qpos_address + 2]) - self._floor_height
        height_penalty = ((height - cfg.target_height) / cfg.height_sigma) ** 2
        oscillation = (
            (linear[2] / cfg.vertical_velocity_sigma) ** 2
            + (angular[0] / cfg.roll_pitch_rate_sigma) ** 2
            + (angular[1] / cfg.roll_pitch_rate_sigma) ** 2
        )

        contact = normal > 0.0
        activity = self._command_activity()
        # Air time: pay for a swing that lasts about air_time_target, only at the
        # instant of touchdown, and only while a motion is commanded.
        touchdown = contact & (self._previous_normal <= 0.0)
        air_reward = float(
            np.sum(np.clip(self._air_time[touchdown] - 0.05, 0.0, cfg.air_time_target))
        ) * activity
        self._air_time[~contact] += dt
        self._air_time[contact] = 0.0
        self._contact_seconds_since[contact] = 0.0
        self._contact_seconds_since[~contact] += dt
        inactive = np.clip(
            self._contact_seconds_since - cfg.inactivity_seconds, 0.0, None
        )
        inactivity_penalty = float(np.sum(inactive)) * activity

        total_normal = float(normal.sum())
        if total_normal > 1e-6:
            share = normal / total_normal
            load_penalty = float(np.sum((share - 1.0 / 6.0) ** 2)) * 6.0
        else:
            load_penalty = 1.0
        impact = np.clip(normal - self._previous_normal, 0.0, None) / dt
        impact_penalty = float(
            np.sum((impact / (cfg.impact_reference_force / 0.05)) ** 2)
        )
        self._previous_normal = normal.copy()

        joint_offset = np.abs(position - self.default_radians)
        excess = np.maximum(0.0, joint_offset - cfg.soft_joint_offset)
        joint_limit_penalty = float(np.mean(np.square(excess / math.radians(15.0))))
        torque_penalty = float(
            np.mean(np.square(self.data.qfrc_actuator[self.root_dof_address + 6:]))
        ) / 4.0
        action_rate = float(np.mean(np.square(action - self._last_action)))
        action_magnitude = float(np.mean(np.square(action)))
        collision = float(self._forbidden_collision())

        terms = {
            "velocity": cfg.velocity_weight * velocity_tracking * dt,
            "progress": cfg.progress_weight * progress * dt,
            "yaw": cfg.yaw_weight * yaw_tracking * dt,
            "heading": cfg.heading_weight * heading_tracking * dt,
            "upright": cfg.upright_weight * upright * dt,
            "air_time": cfg.air_time_weight * air_reward,
            "height": -cfg.height_weight * height_penalty * dt,
            "oscillation": -cfg.oscillation_weight * oscillation * dt,
            "inactivity": -cfg.inactivity_weight * inactivity_penalty * dt,
            "load_share": -cfg.load_share_weight * load_penalty * dt,
            "impact": -cfg.impact_weight * impact_penalty * dt,
            "slip": -cfg.slip_weight * (mean_slip / cfg.slip_sigma) ** 2 * dt,
            "joint_limit": -cfg.joint_limit_weight * joint_limit_penalty * dt,
            "torque": -cfg.torque_weight * torque_penalty * dt,
            "action_rate": -cfg.action_rate_weight * action_rate * dt,
            "action_magnitude": -cfg.action_magnitude_weight * action_magnitude * dt,
            "collision": -cfg.collision_weight * collision * dt,
        }

        finite = bool(
            np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()
        )
        fallen = (
            -float(gravity[2]) < math.cos(math.radians(self.walk_config.max_tilt_degrees))
            or height < self._reference_height - self.walk_config.max_height_drop
        )
        hard_limit = bool(np.any(joint_offset > cfg.hard_joint_offset))
        terminated = (not finite) or fallen or bool(collision) or hard_limit
        total = float(sum(terms.values()))
        if terminated:
            terms["termination"] = -cfg.termination_penalty
            total -= cfg.termination_penalty

        diagnostics = {
            "vx": float(linear[0]), "vy": float(linear[1]),
            "yaw_rate": float(angular[2]), "heading_error": heading_error,
            "height": height, "stance_contacts": int(contact.sum()),
            "command_activity": activity, "mirrored": self._mirrored,
            "foot_contact": contact.astype(np.float64).tolist(),
        }
        return total, terms, terminated, diagnostics

    def _forbidden_collision(self) -> bool:
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 in self._foot_index or geom2 in self._foot_index:
                continue
            if geom1 in self.ground_geom_ids or geom2 in self.ground_geom_ids:
                mujoco.mj_contactForce(self.model, self.data, index, self._contact_force)
                if abs(float(self._contact_force[0])) >= self.walk_config.contact_force_threshold:
                    return True
        return False

    # -- gym API ------------------------------------------------------------

    def _randomize_model(self) -> None:
        cfg = self.walk_config
        mass = self.np_random.uniform(*cfg.mass_scale_range)
        friction = self.np_random.uniform(*cfg.friction_scale_range)
        self.model.body_mass[:] = self._nominal_body_mass * mass
        self.model.geom_friction[:] = self._nominal_geom_friction
        self.model.geom_friction[:, 0] *= friction
        self._strength_scale = float(self.np_random.uniform(*cfg.strength_scale_range))
        for actuator in range(self.model.nu):
            self.model.actuator_gainprm[actuator, 0] *= self._strength_scale

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        del options
        mujoco.mj_resetData(self.model, self.data)
        self._randomize_model()

        self._mirrored = bool(
            self.np_random.random() < self.walk_config.mirror_probability
        )
        self._delay_action = bool(
            self.np_random.random() < self.walk_config.action_delay_probability
        )

        self.controller = MuJoCoController(
            self.model, self.data, verbose=False,
            standing_pose_degrees=tuple(self.default_degrees),
        )
        self.controller.enable_torque()

        noise = self.walk_config.initial_joint_noise_degrees
        if noise > 0.0:
            jitter = self.np_random.normal(0.0, noise, 18)
            for motor_id, offset in enumerate(jitter, start=1):
                self.controller.set_position(
                    motor_id, float(self.default_degrees[motor_id - 1] + offset)
                )
        if self.walk_config.initial_yaw_randomization:
            yaw = float(self.np_random.uniform(-math.pi, math.pi))
            quaternion = np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
            self.data.qpos[self.root_qpos_address + 3: self.root_qpos_address + 7] = quaternion

        self._phase = float(self.np_random.random())
        if self._reference_gait is not None:
            self._reference_gait.reset(
                phase=self._phase, motor_degrees=self.default_degrees
            )

        for _ in range(round(self.walk_config.settle_seconds / self.model.opt.timestep)):
            self.controller.update(self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)

        self._floor_height = float(
            self.model.geom_pos[self.floor_geom_id][2]
        )
        self._reference_height = (
            float(self.data.qpos[self.root_qpos_address + 2]) - self._floor_height
        )
        self._episode_step = 0
        self._last_action.fill(0.0)
        self._pending_action.fill(0.0)
        self._air_time.fill(0.0)
        self._contact_seconds_since.fill(0.0)
        self._previous_normal.fill(0.0)
        self._command.fill(0.0)
        self._command_target = self._sample_command()
        self._next_command_step = 0
        self._target_heading = self._canonical_yaw()
        push = self.np_random.uniform(*self.walk_config.push_interval_seconds)
        self._next_push_step = round(push / self.control_dt)

        normal, _, _ = self._foot_contacts()
        return self._observation(normal), {"command": self._command.copy()}

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        raw = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        if self._mirrored:
            raw = mirror_action(raw)
        applied = self._pending_action if self._delay_action else raw
        self._pending_action = raw.copy()

        self._update_command()
        self._target_heading += self._command[2] * self.control_dt
        self._apply_action(applied)

        if self._episode_step >= self._next_push_step:
            impulse = self.np_random.normal(0.0, self.walk_config.push_velocity, 2)
            self.data.qvel[self.root_dof_address: self.root_dof_address + 2] += impulse
            interval = self.np_random.uniform(*self.walk_config.push_interval_seconds)
            self._next_push_step = self._episode_step + round(interval / self.control_dt)

        for _ in range(self.walk_config.frame_skip):
            self.controller.update(self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)

        normal, _, mean_slip = self._foot_contacts()
        reward, terms, terminated, diagnostics = self._reward(applied, normal, mean_slip)
        self._last_action[:] = applied
        self._episode_step += 1
        truncated = self._episode_step >= self.max_episode_steps
        info = {"reward_terms": terms, "command": self._command.copy(), **diagnostics}
        return self._observation(normal), reward, terminated, truncated, info

    def render(self) -> None:  # pragma: no cover - viewer only
        if self.render_mode != "human":
            return
        import mujoco.viewer
        if self._viewer is None:
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self._viewer.sync()

    def close(self) -> None:  # pragma: no cover - viewer only
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None


__all__ = [
    "CANONICAL_FROM_CHASSIS",
    "CURRICULUM_RANGES",
    "REFERENCE_CHOICES",
    "RewardConfig",
    "SconeWalkEnvV2",
    "WalkConfig",
    "mirror_action",
    "mirror_observation",
]


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------

def _build_env(args: Any, index: int = 0):
    return SconeWalkEnvV2(
        args.model,
        curriculum=args.curriculum,
        reference_motion=args.reference_motion,
        terrain=args.terrain,
        terrain_seed=args.terrain_seed + index,
    )


def run_check(args: Any) -> int:
    env = _build_env(args)
    observation, _ = env.reset(seed=args.seed)
    print(f"observation {observation.shape}  action {env.action_space.shape}")
    total = 0.0
    for _ in range(args.steps):
        action = (
            env.action_space.sample() if args.random_actions else np.zeros(18, np.float32)
        )
        observation, reward, terminated, truncated, info = env.step(action)
        total += reward
        if terminated or truncated:
            break
    print(f"return {total:.3f}  vx {info['vx']:+.4f}  stance {info['stance_contacts']}")
    print("reward terms:", {k: round(v, 5) for k, v in info["reward_terms"].items()})
    env.close()
    return 0


def run_train(args: Any) -> int:
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecMonitor

    run_dir = Path(args.output).expanduser().resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    factories = [
        (lambda i=i: _build_env(args, i)) for i in range(args.num_envs)
    ]
    vector = (
        DummyVecEnv(factories) if args.num_envs == 1 else SubprocVecEnv(factories)
    )
    env = VecMonitor(vector, filename=str(run_dir / "monitor.csv"))
    model = PPO(
        "MlpPolicy", env,
        learning_rate=3e-4, n_steps=args.n_steps, batch_size=args.batch_size,
        n_epochs=5, gamma=0.995, gae_lambda=0.95, clip_range=0.2,
        ent_coef=0.003, max_grad_norm=1.0,
        policy_kwargs={"net_arch": {"pi": [512, 256, 128], "vf": [512, 256, 128]}},
        tensorboard_log=args.tensorboard_log, verbose=1,
        seed=args.seed, device=args.device,
    )
    model.learn(total_timesteps=args.timesteps)
    model.save(run_dir / "final_model.zip")
    env.close()
    print(f"saved {run_dir / 'final_model.zip'}")
    return 0


def run_enjoy(args: Any) -> int:
    from stable_baselines3 import PPO

    env = SconeWalkEnvV2(
        args.model, curriculum=args.curriculum,
        reference_motion=args.reference_motion, render_mode="human",
        terrain=args.terrain, terrain_seed=args.terrain_seed,
    )
    model = PPO.load(Path(args.checkpoint).expanduser().resolve(), device=args.device)
    for _ in range(args.episodes):
        observation, _ = env.reset(seed=args.seed)
        done = False
        while not done:
            action, _ = model.predict(observation, deterministic=True)
            observation, _, terminated, truncated, _ = env.step(action)
            env.render()
            done = terminated or truncated
    env.close()
    return 0


def build_parser():
    import argparse

    parser = argparse.ArgumentParser(
        prog="walk-v2", description="Redesigned SCONE walking PPO"
    )
    parser.add_argument("--model", default=str(DEFAULT_MODEL_PATH))
    parser.add_argument("--terrain", default="flat")
    parser.add_argument("--terrain-seed", type=int, default=7)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--reference-motion", choices=REFERENCE_CHOICES, default="tripod-gait",
        help="baseline motion the policy corrects; 'none' trains end to end",
    )
    parser.add_argument(
        "--curriculum", choices=tuple(CURRICULUM_RANGES), default="easy"
    )
    sub = parser.add_subparsers(dest="command_name", required=True)

    check = sub.add_parser("check", help="single rollout smoke test")
    check.add_argument("--steps", type=int, default=300)
    check.add_argument("--random-actions", action="store_true")
    check.set_defaults(func=run_check)

    train = sub.add_parser("train")
    train.add_argument("--output", default="runs/walk_v2")
    train.add_argument("--timesteps", type=int, default=50_000_000)
    train.add_argument("--num-envs", type=int, default=64)
    train.add_argument("--n-steps", type=int, default=256)
    train.add_argument("--batch-size", type=int, default=8192)
    train.add_argument("--tensorboard-log", default=None)
    train.set_defaults(func=run_train)

    enjoy = sub.add_parser("enjoy")
    enjoy.add_argument("checkpoint")
    enjoy.add_argument("--episodes", type=int, default=3)
    enjoy.set_defaults(func=run_enjoy)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
