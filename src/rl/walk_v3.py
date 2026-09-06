"""Third SCONE walking-PPO design: residual RL on a speed-scaled hardcoded gait.

``walk_v3`` goes back to what actually worked -- ``walk_learn``'s residual
policy on top of the hand-authored sinusoidal tripod -- and removes the three
things that capped it. ``walk_learn.py`` and ``walk_v2.py`` are left untouched
so their checkpoints keep replaying.

What changed against ``walk_learn`` (v1)
---------------------------------------
* **No speed limit.** v1's reference saturated far below its own command range:
  at the default 20 degrees of stride it reaches 0.10 m/s while the full
  curriculum asked for 0.50 m/s, so there was no reward gradient left at speed
  (docs/17-ppo-diagnosis-and-fix-plan.md section 2). Here the reference stride,
  foot lift and cadence are *derived from the command* with measured
  coefficients, the velocity command is never clipped, and the speed reward is
  one-sided: reaching the command pays in full and exceeding it is never
  punished.
* **Body height is free.** No height reward term and no height-drop
  termination. The chassis may bob, crouch or rise; only an actual
  non-foot/floor contact ends the episode.
* **Body orientation is not.** Attitude is the hard constraint: a tight
  projected-gravity cost, a roll/pitch *rate* cost, and termination at
  ``max_tilt_degrees``. For reference, the zero-residual scaffold measured
  1.6-4.8 degrees of peak tilt across the whole speed envelope, so the 15
  degree limit is a real constraint with margin rather than a fall detector.

Kept from ``walk_v2`` (they were measured fixes, not the failed objective)
-------------------------------------------------------------------------
* Bounded actions: gSDE with a squashed policy, so PPO's log probability is
  computed for the action the environment actually receives. v2 measured 84-95
  percent of raw action components pinned at the clip without this.
* Domain randomization only in training profiles; replay and joystick sessions
  stay on the nominal model.
* A fixed-command evaluation that promotes a best model only when it beats the
  *zero-residual* scaffold, which is exactly the comparison v2 kept losing.

Deliberately not carried over: the canonical-frame rotation, the sagittal
mirror augmentation and the contact-force/air-time/load-share reward family.
v3 keeps v1's chassis-frame command convention (see ``walk_v2``'s
``CANONICAL_FROM_CHASSIS`` note and docs/18 for the 180-degree difference and
the open question of which end is the front).

Known gap: the hardcoded reference has no lateral scaffold -- a pure ``vy``
command measures 0.0000 m/s open loop -- so lateral tracking is entirely the
residual's job. Forward and yaw are both scaffolded.

Examples
--------
Inspect one rollout of the scaffold, no learning::

    python -m src.rl.walk_v3 check --command 0.30 0 0 --steps 300

Train::

    python -m src.rl.walk_v3 train --curriculum easy --timesteps 20000000

Replay::

    mjpython -m src.rl.walk_v3 enjoy runs/walk_v3/best_model.zip --command 0.3 0 0
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Sequence

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from src.hardware.actuator import Actuator
from src.rl.motion_profile import motion_profile_for_standing_pose
from src.rl.stance import STANCE_PRESETS, validate_standing_pose
from src.simulation.core.controller import MuJoCoController
from src.simulation.core.model import load_model
from src.simulation.core.pid import spec_for_motor_id
from src.simulation.core.viewer import configure_simulation_viewer
from src.simulation.terrain import TERRAIN_CHOICES, TerrainType


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_PATH = PROJECT_ROOT / "assets" / "model.xml"
CHECKPOINT_PREFIX = "scone_walk_v3"

# v1's 70 values plus six foot-contact flags. The extra six are what let a
# policy know which legs are carrying it (docs/17 section 3), and they also
# make a v3 checkpoint self-identifying: 68/70 is walk_learn, 82 is walk_v2.
OBSERVATION_DIM = 76

# v3 scaffolds forward and yaw from the hand-authored tripod only. The IK gaits
# are not offered here: their stride cap is the speed ceiling this design
# exists to remove.
REFERENCE_CHOICES = ("hardcoded", "none")


def normalize_reference_motion(value: str) -> str:
    """v3 has no legacy spellings; kept for a uniform trainer interface."""

    return value


# Command normalization for the observation only. Commands are NOT clipped to
# it: a larger command simply produces an observation component above 1.0, and
# the one-sided speed reward means asking for more than the robot can give is
# never punished.
OBSERVATION_COMMAND_SCALE = np.array([0.40, 0.20, 0.90], dtype=np.float64)

# Sampling ranges [|vx|, |vy|, |yaw_rate|]. The zero-residual scaffold was
# measured at 0.10-0.32 m/s depending on stride, so unlike v1 (0.30-0.50
# against a 0.10 m/s reference) every stage here is inside what the reference
# plus residual can actually produce. |vy| stays small because the hardcoded
# reference gives lateral motion no scaffold at all.
CURRICULUM_RANGES: dict[str, np.ndarray] = {
    "easy": np.array([0.20, 0.00, 0.00], dtype=np.float64),
    "medium": np.array([0.30, 0.05, 0.50], dtype=np.float64),
    "full": np.array([0.40, 0.12, 0.90], dtype=np.float64),
}


@dataclass(frozen=True)
class RewardConfig:
    """Weights and tolerances. Every term is scaled by the control step.

    An ideal standing robot under an idle command scores exactly zero: there
    are no positive survival constants, which is what let v2's policies harvest
    reward for ignoring the command (docs/21 section 11.2).
    """

    # -- speed: one-sided, so overspeed is never a penalty -----------------
    # Credit is measured in units of this speed, and saturates at the
    # commanded speed. Faster than commanded is therefore always at least as
    # good as commanded, and never better -- which keeps the command
    # meaningful without putting a ceiling on it.
    speed_reference: float = 0.30              # m/s scoring 1.0 of credit
    speed_weight: float = 3.0
    # Motion across the command direction is a direction error, not speed.
    drift_weight: float = 1.0
    yaw_reference: float = 0.90                # rad/s scoring 1.0 of credit
    yaw_weight: float = 1.0
    heading_error_sigma: float = 0.25          # rad
    heading_weight: float = 0.50
    # Heading is a *hold* objective: it only means something while no turn is
    # commanded. Above this yaw command the heading cost is switched off and
    # the yaw-rate term does the work, because integrating a heading target
    # through even a small rate deficit grows a cost without bound.
    yaw_hold_threshold: float = 0.05           # rad/s

    # -- orientation: the hard constraint ---------------------------------
    # 0.10 of projected gravity is 5.7 degrees of tilt, so one unit of cost is
    # already more tilt than the measured scaffold ever produced.
    projected_gravity_sigma: float = 0.10
    upright_weight: float = 1.50
    roll_pitch_rate_sigma: float = 0.60        # rad/s
    roll_pitch_rate_weight: float = 0.30
    # Shared saturation for the quadratic attitude and heading costs, so one
    # bad frame cannot dominate an episode's return.
    cost_cap: float = 25.0
    # There is deliberately no height term and no vertical-velocity term:
    # vertical motion of the body is allowed by design.

    # -- legs --------------------------------------------------------------
    # A foot that has not touched down for this long starts costing score,
    # but only while the robot is actually making progress, so shuffling in
    # place cannot buy leg-participation credit.
    inactivity_seconds: float = 0.75
    inactivity_weight: float = 0.20
    slip_deadzone: float = 0.02                # m/s at the contact point
    slip_sigma: float = 0.20
    slip_weight: float = 0.10

    # -- effort and safety -------------------------------------------------
    # v1 used 0.25 for action magnitude, which paid the policy to leave legs
    # unused (docs/17 section 3, item 3). Authority is the point here.
    action_rate_weight: float = 0.02
    action_magnitude_weight: float = 0.02
    current_weight: float = 0.02
    soft_joint_offset: float = math.radians(70.0)
    hard_joint_offset: float = math.radians(100.0)
    joint_limit_weight: float = 0.20
    collision_weight: float = 1.00
    termination_penalty: float = 5.00

    # -- idle --------------------------------------------------------------
    idle_velocity_weight: float = 0.75
    idle_action_weight: float = 0.15
    idle_linear_velocity_sigma: float = 0.04   # m/s
    idle_yaw_velocity_sigma: float = 0.08      # rad/s
    idle_activity_threshold: float = 0.05
    idle_cost_cap: float = 4.0

    def __post_init__(self) -> None:
        if min(
            self.speed_reference,
            self.yaw_reference,
            self.heading_error_sigma,
            self.projected_gravity_sigma,
            self.roll_pitch_rate_sigma,
            self.slip_sigma,
            self.idle_linear_velocity_sigma,
            self.idle_yaw_velocity_sigma,
            self.idle_activity_threshold,
            self.yaw_hold_threshold,
        ) <= 0.0:
            raise ValueError("every reward scale must be positive")
        if self.inactivity_seconds <= 0.0:
            raise ValueError("inactivity_seconds must be positive")


@dataclass(frozen=True)
class WalkConfig:
    """Environment timing, the command-driven reference, and randomization.

    The reference coefficients are measured, not guessed. Holding a fixed
    command with a zero residual on flat ground in the Standard stance:

    ==========  ========  =========  ===============  ==========
    stride deg  lift deg  cadence    achieved m/s     peak tilt
    ==========  ========  =========  ===============  ==========
    20          20        1.4 Hz     0.101            1.6 deg
    30          20        1.4 Hz     0.150            ~2 deg
    50          20        1.4 Hz     0.209            2.2 deg
    60          40        1.4 Hz     0.249            4.3 deg
    85          40        1.4 Hz     0.324            4.8 deg
    ==========  ========  =========  ===============  ==========

    Speed is very close to linear in stride and barely responds to cadence
    above about 1.4 Hz, and long strides need proportionally more foot lift or
    the swing tripod drags. ``stride_degrees_per_speed`` and
    ``lift_degrees_per_stride`` encode exactly that, so the reference tracks
    the command over the whole range instead of saturating at one speed.
    """

    physics_timestep: float = 0.002            # 500 Hz MuJoCo
    frame_skip: int = 10                       # 50 Hz policy
    episode_seconds: float = 20.0
    settle_seconds: float = 0.20
    command_filter_seconds: float = 0.30
    command_hold_seconds: tuple[float, float] = (2.0, 4.0)
    idle_command_probability: float = 0.15

    # -- command-driven hardcoded reference --------------------------------
    # Stride is read off the measured inverse of the table above rather than a
    # single gain, because speed is not linear in stride: it climbs steeply to
    # about 40 degrees, plateaus, then climbs again. One gain therefore
    # over-delivers by 35 percent at 0.10 m/s while under-delivering at 0.30.
    # Pairs are (achieved m/s, stride degrees), monotone in both columns.
    stride_calibration: tuple[tuple[float, float], ...] = (
        (0.000, 0.0), (0.020, 10.0), (0.067, 15.0), (0.101, 20.0),
        (0.131, 25.0), (0.154, 30.0), (0.192, 40.0), (0.203, 50.0),
        (0.242, 70.0), (0.301, 80.0), (0.336, 90.0),
    )
    # The same treatment for the turn scaffold, measured the same way: pairs
    # are (achieved rad/s, differential stride degrees). Open loop it reaches
    # 1.66 rad/s, well past the full curriculum.
    yaw_stride_calibration: tuple[tuple[float, float], ...] = (
        (0.000, 0.0), (0.021, 5.0), (0.169, 10.0), (0.364, 15.0),
        (0.445, 20.0), (0.606, 30.0), (0.827, 40.0), (0.959, 50.0),
        (1.222, 65.0), (1.563, 80.0), (1.661, 90.0),
    )
    # Measured clearance optimum: about 20 degrees is enough up to a 40 degree
    # stride, and beyond that it has to grow with the stride or the swinging
    # tripod drags (stride 85 wants 40). Below the floor the clearance follows
    # the stride itself so an idle command lifts nothing at all.
    lift_degrees_per_stride: float = 0.47      # foot clearance per stride deg
    lift_degrees_floor: float = 20.0
    lift_degrees_max: float = 40.0
    # Cadence ramps with the stride, but it is not the speed lever: measured,
    # anything above about 1.5 Hz buys nothing and above 2 Hz costs speed.
    cadence_hz_min: float = 1.20
    cadence_hz_max: float = 1.50
    # Joint-travel bounds, not speed bounds. model.xml still carries no
    # mechanical joint ranges (see docs/02 and v1's note), so the reference
    # amplitude and the final target both stay inside a conservative envelope
    # until real hard stops are measured.
    stride_degrees_max: float = 90.0
    joint_target_bound_degrees: float = 70.0

    # -- orientation constraint and contacts -------------------------------
    max_tilt_degrees: float = 15.0
    contact_force_threshold: float = 1.0
    stance_preset: str = "standard"
    # Deliberately absent: max_height_drop. v3 does not terminate on height.

    # -- randomization: nominal here, staged in training_walk_config -------
    initial_joint_noise_degrees: float = 0.0
    initial_yaw_randomization: bool = False
    observation_noise: float = 0.0
    mass_scale_range: tuple[float, float] = (1.0, 1.0)
    friction_scale_range: tuple[float, float] = (1.0, 1.0)
    strength_scale_range: tuple[float, float] = (1.0, 1.0)

    def __post_init__(self) -> None:
        if not 0.0 <= self.idle_command_probability <= 1.0:
            raise ValueError("idle_command_probability must be between 0 and 1")
        if self.frame_skip < 1 or self.physics_timestep <= 0.0:
            raise ValueError("frame_skip and physics_timestep must be positive")
        if self.cadence_hz_min <= 0.0 or self.cadence_hz_max < self.cadence_hz_min:
            raise ValueError("cadence bounds must be positive and ordered")
        for name, calibration in (
            ("stride_calibration", self.stride_calibration),
            ("yaw_stride_calibration", self.yaw_stride_calibration),
        ):
            demands = [demand for demand, _ in calibration]
            strides = [stride for _, stride in calibration]
            if len(demands) < 2:
                raise ValueError(f"{name} needs at least two points")
            if demands != sorted(demands) or strides != sorted(strides):
                raise ValueError(f"{name} must be monotone in both columns")
            if demands[0] < 0.0 or strides[0] < 0.0:
                raise ValueError(f"{name} cannot contain negative values")
        if self.stride_degrees_max <= 0.0 or self.lift_degrees_max < 0.0:
            raise ValueError("stride and lift bounds cannot be negative")
        if not 0.0 < self.max_tilt_degrees < 90.0:
            raise ValueError("max_tilt_degrees must be between 0 and 90")
        if self.command_hold_seconds[0] > self.command_hold_seconds[1]:
            raise ValueError("command_hold_seconds must be ordered")


def training_walk_config(curriculum: str) -> WalkConfig:
    """Return the training-only randomization for one curriculum stage.

    Replay, the live remote viewer and joystick control all keep ``WalkConfig``
    defaults, so a checkpoint is always inspected on the nominal robot.
    """

    if curriculum not in CURRICULUM_RANGES:
        raise ValueError(f"unknown curriculum {curriculum!r}")
    if curriculum == "easy":
        return replace(
            WalkConfig(),
            initial_joint_noise_degrees=1.0,
            initial_yaw_randomization=True,
            observation_noise=0.002,
            mass_scale_range=(0.98, 1.02),
            friction_scale_range=(0.90, 1.10),
            strength_scale_range=(0.95, 1.05),
        )
    if curriculum == "medium":
        return replace(
            WalkConfig(),
            initial_joint_noise_degrees=2.0,
            initial_yaw_randomization=True,
            observation_noise=0.005,
            mass_scale_range=(0.95, 1.05),
            friction_scale_range=(0.80, 1.20),
            strength_scale_range=(0.90, 1.10),
        )
    return replace(
        WalkConfig(),
        initial_joint_noise_degrees=3.0,
        initial_yaw_randomization=True,
        observation_noise=0.008,
        mass_scale_range=(0.92, 1.08),
        friction_scale_range=(0.75, 1.25),
        strength_scale_range=(0.88, 1.12),
    )


class SconeWalkEnvV3(gym.Env[np.ndarray, np.ndarray]):
    """Residual walking task with free body height and locked attitude."""

    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(
        self,
        model_path: Path | str = DEFAULT_MODEL_PATH,
        *,
        curriculum: str = "easy",
        reference_motion: str = "hardcoded",
        fixed_command: Sequence[float] | None = None,
        reward_config: RewardConfig | None = None,
        walk_config: WalkConfig | None = None,
        terrain: TerrainType | str = TerrainType.FLAT,
        terrain_seed: int = 7,
        standing_pose_degrees: Sequence[float] | None = None,
        render_mode: str | None = None,
    ) -> None:
        super().__init__()
        reference_motion = normalize_reference_motion(reference_motion)
        if curriculum not in CURRICULUM_RANGES:
            raise ValueError(f"unknown curriculum {curriculum!r}")
        if reference_motion not in REFERENCE_CHOICES:
            raise ValueError(
                f"unknown reference motion {reference_motion!r}; choose from "
                f"{REFERENCE_CHOICES}"
            )
        if render_mode not in (None, "human"):
            raise ValueError("render_mode must be None or 'human'")

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
        if self.fixed_command is not None and self.fixed_command.shape != (3,):
            raise ValueError("fixed_command must contain [vx, vy, yaw_rate]")

        self.model = load_model(
            self.model_path, terrain=self.terrain, terrain_seed=terrain_seed
        )
        self.model.opt.timestep = self.walk_config.physics_timestep
        self.data = mujoco.MjData(self.model)
        self.controller: MuJoCoController
        self.control_dt = self.model.opt.timestep * self.walk_config.frame_skip
        self.max_episode_steps = round(
            self.walk_config.episode_seconds / self.control_dt
        )

        self._nominal_body_mass = self.model.body_mass.copy()
        self._nominal_body_inertia = self.model.body_inertia.copy()
        self._nominal_geom_friction = self.model.geom_friction.copy()
        self._nominal_actuator_gainprm = self.model.actuator_gainprm.copy()
        self._nominal_actuator_forcerange = self.model.actuator_forcerange.copy()

        self.root_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "root_freejoint"
        )
        if self.root_joint_id < 0:
            raise ValueError("walk_v3 requires model.xml root_freejoint")
        self.root_body_id = int(self.model.jnt_bodyid[self.root_joint_id])
        self.root_qpos_address = int(self.model.jnt_qposadr[self.root_joint_id])

        self.floor_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "simulation_floor"
        )
        if self.floor_geom_id < 0:
            raise ValueError("walk_v3 requires simulation_floor")
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
        self._foot_index = {geom: index for index, geom in enumerate(self.foot_geom_ids)}
        self._foot_body = {
            geom: int(self.model.geom_bodyid[geom]) for geom in self.foot_geom_ids
        }

        self.default_degrees = np.asarray(
            validate_standing_pose(
                STANCE_PRESETS[self.walk_config.stance_preset]
                if standing_pose_degrees is None
                else standing_pose_degrees
            ),
            dtype=np.float64,
        )
        self.default_radians = np.array(
            [
                MuJoCoController.raw_to_radians(
                    MuJoCoController.degrees_to_raw(motor_id, degrees)
                )
                for motor_id, degrees in enumerate(self.default_degrees, start=1)
            ],
            dtype=np.float64,
        )
        self._motion_profile = motion_profile_for_standing_pose(self.default_degrees)
        # Authority, not trim: the reference stride at speed is tens of
        # degrees, so a residual that can only nudge it cannot rewrite the gait.
        self.residual_scale_degrees = np.array(
            [15.0] * 6 + [18.0] * 6 + [20.0] * 6, dtype=np.float64
        )
        self._tripod_a = set(Actuator.Index.UPPER_DIAGONAL_LEFT)   # {2, 3, 6}

        self.action_space = spaces.Box(-1.0, 1.0, shape=(18,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(OBSERVATION_DIM,), dtype=np.float32
        )

        self._phase = 0.0
        self._episode_step = 0
        self._next_command_step = 0
        self._command = np.zeros(3, dtype=np.float64)
        self._command_target = np.zeros(3, dtype=np.float64)
        self._target_heading = 0.0
        self._last_action = np.zeros(18, dtype=np.float64)
        self._contact_seconds_since = np.zeros(6, dtype=np.float64)
        self._reference_cadence = 0.0
        self._reference_stride_degrees = 0.0
        self._reference_height = 0.0
        self._reference_override_degrees: np.ndarray | None = None
        self._reference_override_blend = 0.0
        self._reference_override_unwrapped_lower = False
        self._contact_force = np.zeros(6, dtype=np.float64)
        self._body_velocity = np.zeros(6, dtype=np.float64)
        self._jacobian_position = np.zeros((3, self.model.nv), dtype=np.float64)
        self._jacobian_rotation = np.zeros((3, self.model.nv), dtype=np.float64)
        self._viewer: Any | None = None

    # -- command ------------------------------------------------------------

    def set_velocity_command(
        self, command: np.ndarray | list[float] | tuple[float, float, float]
    ) -> np.ndarray:
        """Install a live ``[vx, vy, yaw_rate]`` command, unclipped.

        v1 and v2 both clamped the joystick to their observation
        normalization. v3 does not: a command above that range is a legitimate
        request for more speed, it merely produces an observation component
        above 1.0, and the one-sided speed reward never punishes the policy for
        failing to reach it or for exceeding it.
        """

        parsed = np.asarray(command, dtype=np.float64)
        if parsed.shape != (3,):
            raise ValueError("velocity command must contain [vx, vy, yaw_rate]")
        if not np.all(np.isfinite(parsed)):
            raise ValueError("velocity command must contain only finite values")
        self.fixed_command = parsed.copy()
        self._command_target[:] = parsed
        return parsed.copy()

    def _sample_command(self) -> np.ndarray:
        if self.fixed_command is not None:
            return self.fixed_command.copy()
        if self.np_random.random() < self.walk_config.idle_command_probability:
            return np.zeros(3, dtype=np.float64)

        command = np.zeros(3, dtype=np.float64)
        limit = self.command_range
        if self.curriculum == "easy":
            # One primitive at a time, mostly forward, and never a command so
            # small that it is indistinguishable from idle.
            forward = self.np_random.uniform(0.04, limit[0])
            command[0] = forward if self.np_random.random() < 0.90 else -forward
            return command
        choice = self.np_random.random()
        if choice < 0.55:
            command[0] = self._signed(limit[0], 0.04, 0.85)
        elif choice < 0.75:
            command[2] = self._signed(limit[2], 0.10, 0.50)
        elif choice < 0.90:
            command[1] = self._signed(limit[1], 0.025, 0.50)
        else:
            command[0] = self._signed(limit[0], 0.04, 0.85)
            command[2] = self._signed(limit[2], 0.10, 0.50)
        return command

    def _signed(
        self, limit: float, minimum: float, positive_probability: float
    ) -> float:
        if limit <= minimum:
            return 0.0
        magnitude = float(self.np_random.uniform(minimum, limit))
        sign = 1.0 if self.np_random.random() < positive_probability else -1.0
        return sign * magnitude

    def _schedule_next_command(self) -> None:
        hold = self.np_random.uniform(*self.walk_config.command_hold_seconds)
        self._next_command_step = self._episode_step + max(
            1, round(hold / self.control_dt)
        )

    def _update_command(self) -> None:
        if self.fixed_command is not None:
            self._command[:] = self.fixed_command
            return
        if self._episode_step >= self._next_command_step:
            self._command_target = self._sample_command()
            self._schedule_next_command()
        alpha = 1.0 - math.exp(
            -self.control_dt / self.walk_config.command_filter_seconds
        )
        self._command += alpha * (self._command_target - self._command)

    def _command_activity(self) -> float:
        """Normalized command size, uncapped so fast commands stay distinct."""

        return float(np.max(np.abs(self._command / OBSERVATION_COMMAND_SCALE)))

    def _command_gate(self) -> float:
        threshold = self.reward_config.idle_activity_threshold
        return float(np.clip(self._command_activity() / threshold, 0.0, 1.0))

    # -- reference ----------------------------------------------------------

    def _stride_from(
        self, demand: float, calibration: tuple[tuple[float, float], ...]
    ) -> float:
        if not math.isfinite(demand) or demand < 0.0:
            raise ValueError("demand must be finite and non-negative")
        stride = float(np.interp(
            demand,
            [point[0] for point in calibration],
            [point[1] for point in calibration],
        ))
        return min(stride, self.walk_config.stride_degrees_max)

    def stride_for_speed(self, speed: float) -> float:
        """Return the reference stride, in degrees, that produces ``speed``.

        This is the measured stride/speed curve read backwards, which is what
        makes the scaffold track the command instead of saturating at one
        speed. Above the calibrated top speed it returns the stride bound: the
        command is not clipped, the scaffold simply runs out and the residual
        has to supply the rest.
        """

        return self._stride_from(speed, self.walk_config.stride_calibration)

    def stride_for_yaw_rate(self, yaw_rate: float) -> float:
        """Return the differential stride, in degrees, that turns at ``yaw_rate``."""

        return self._stride_from(yaw_rate, self.walk_config.yaw_stride_calibration)

    def _reference_degrees(self) -> np.ndarray:
        """Return this frame's reference motor degrees for the command.

        The pattern is v1's: the two tripods swing their hip (upper) joints in
        antiphase while the swinging tripod lifts its middle joints. What is
        new is that stride, lift and cadence are all functions of the command,
        with no saturation inside the joint-travel envelope, so the scaffold
        keeps producing more speed when more speed is asked for.
        """

        reference = self.default_degrees.copy()
        if self.reference_motion == "none":
            self._reference_cadence = 0.0
            self._reference_stride_degrees = 0.0
            return reference

        config = self.walk_config
        # v1's verified sign: the legacy motor-angle convention advances the
        # chassis toward +body-x when the stride offset is negative.
        forward_degrees = -math.copysign(
            self.stride_for_speed(abs(self._command[0])), self._command[0]
        )
        yaw_degrees = math.copysign(
            self.stride_for_yaw_rate(abs(self._command[2])), self._command[2]
        )
        bound = config.stride_degrees_max
        per_leg = {
            motor_id: float(np.clip(
                forward_degrees + yaw_degrees * (1.0 if motor_id % 2 else -1.0),
                -bound,
                bound,
            ))
            for motor_id in Actuator.Index.UPPER
        }
        stride_magnitude = max(abs(value) for value in per_leg.values())
        self._reference_stride_degrees = stride_magnitude
        cadence = config.cadence_hz_min + (
            config.cadence_hz_max - config.cadence_hz_min
        ) * min(1.0, stride_magnitude / bound)
        self._reference_cadence = cadence
        self._phase = (self._phase + cadence * self.control_dt) % 1.0
        phase_sine = math.sin(2.0 * math.pi * self._phase)

        for motor_id, stride in per_leg.items():
            tripod_sign = -1.0 if motor_id in self._tripod_a else 1.0
            reference[motor_id - 1] += stride * phase_sine * tripod_sign

        # Long strides need proportionally more clearance or the swing tripod
        # drags.
        lift_degrees = min(
            config.lift_degrees_max,
            max(
                config.lift_degrees_per_stride * stride_magnitude,
                min(config.lift_degrees_floor, stride_magnitude),
            ),
        )
        lift_a = max(0.0, phase_sine)
        lift_b = max(0.0, -phase_sine)
        for upper_motor_id in Actuator.Index.UPPER:
            lift = lift_a if upper_motor_id in self._tripod_a else lift_b
            reference[upper_motor_id + 5] -= lift_degrees * lift
        return reference

    def set_reference_override(
        self,
        motor_degrees: Sequence[float] | None,
        *,
        blend: float = 1.0,
        unwrapped_lower: bool = False,
    ) -> None:
        """Blend a replay-only reference in, as the interactive hybrid does."""

        if motor_degrees is None:
            self._reference_override_degrees = None
            self._reference_override_blend = 0.0
            self._reference_override_unwrapped_lower = False
            return
        parsed = np.asarray(motor_degrees, dtype=np.float64)
        if parsed.shape != (18,) or not np.all(np.isfinite(parsed)):
            raise ValueError("reference override must contain 18 finite degrees")
        if not 0.0 <= blend <= 1.0:
            raise ValueError("reference override blend must be in [0, 1]")
        self._reference_override_degrees = parsed.copy()
        self._reference_override_blend = float(blend)
        self._reference_override_unwrapped_lower = bool(unwrapped_lower)

    def _apply_action(self, action: np.ndarray) -> None:
        reference = self._reference_degrees()
        if self._reference_override_degrees is not None:
            blend = self._reference_override_blend
            if self._reference_override_unwrapped_lower:
                lower_reference = reference[12:]
                lower_override = self._reference_override_degrees[12:]
                reference[12:] = lower_reference + 360.0 * np.round(
                    (lower_override - lower_reference) / 360.0
                )
            reference = (
                (1.0 - blend) * reference
                + blend * self._reference_override_degrees
            )
        targets = reference + self.residual_scale_degrees * action
        bound = self.walk_config.joint_target_bound_degrees
        bounded = np.clip(
            targets, self.default_degrees - bound, self.default_degrees + bound
        )
        if self._reference_override_unwrapped_lower:
            bounded[12:] = targets[12:]
        for motor_id, target in enumerate(bounded, start=1):
            self.controller.set_position(motor_id, float(target))

    # -- state --------------------------------------------------------------

    def _joint_state(self) -> tuple[np.ndarray, np.ndarray]:
        position = np.array(
            [self.controller._joint_position(i) for i in Actuator.Index.ALL],
            dtype=np.float64,
        )
        velocity = np.array(
            [self.controller._joint_velocity(i) for i in Actuator.Index.ALL],
            dtype=np.float64,
        )
        if self._reference_override_unwrapped_lower:
            lower_delta = position[12:] - self.default_radians[12:]
            position[12:] = self.default_radians[12:] + (
                lower_delta + math.pi
            ) % (2.0 * math.pi) - math.pi
        return position, velocity

    def _base_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Chassis-frame linear velocity, angular velocity, projected gravity."""

        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_BODY,
            self.root_body_id,
            self._body_velocity,
            0,
        )
        world_from_body = self.data.xmat[self.root_body_id].reshape(3, 3)
        angular = world_from_body.T @ self._body_velocity[:3]
        linear = world_from_body.T @ self._body_velocity[3:]
        gravity = world_from_body.T @ np.array([0.0, 0.0, -1.0])
        return linear, angular, gravity

    def _heading_yaw(self) -> float:
        world_from_body = self.data.xmat[self.root_body_id].reshape(3, 3)
        return float(math.atan2(world_from_body[1, 0], world_from_body[0, 0]))

    def _heading_error(self) -> float:
        error = self._heading_yaw() - self._target_heading
        return float(math.atan2(math.sin(error), math.cos(error)))

    def _contact_normal_force(self, contact_index: int) -> float:
        self._contact_force.fill(0.0)
        mujoco.mj_contactForce(
            self.model, self.data, contact_index, self._contact_force
        )
        return abs(float(self._contact_force[0]))

    def _foot_contacts(self) -> tuple[np.ndarray, float]:
        """Return per-foot contact flags and the mean normalized slip cost.

        Slip is measured at the actual contact point through the body jacobian,
        as in v1: the tip of an arc-sector foot is what is planted, so the
        distal body's center velocity would be the wrong quantity.
        """

        contact_flags = np.zeros(6, dtype=np.float64)
        slip_values: list[float] = []
        threshold = self.walk_config.contact_force_threshold
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 in self._foot_index and geom2 in self.ground_geom_ids:
                foot_geom = geom1
            elif geom2 in self._foot_index and geom1 in self.ground_geom_ids:
                foot_geom = geom2
            else:
                continue
            if self._contact_normal_force(index) < threshold:
                continue
            contact_flags[self._foot_index[foot_geom]] = 1.0

            self._jacobian_position.fill(0.0)
            self._jacobian_rotation.fill(0.0)
            mujoco.mj_jac(
                self.model,
                self.data,
                self._jacobian_position,
                self._jacobian_rotation,
                contact.pos,
                self._foot_body[foot_geom],
            )
            point_velocity = self._jacobian_position @ self.data.qvel
            normal = np.asarray(contact.frame[:3], dtype=np.float64)
            tangential = point_velocity - normal * float(point_velocity @ normal)
            excess = max(
                0.0,
                float(np.linalg.norm(tangential)) - self.reward_config.slip_deadzone,
            )
            slip_values.append((excess / self.reward_config.slip_sigma) ** 2)
        mean_slip = float(np.mean(slip_values)) if slip_values else 0.0
        return contact_flags, mean_slip

    def _forbidden_collision(self) -> bool:
        """True when anything other than a foot carries load from the ground.

        This is v3's collapse guard. Height is otherwise unconstrained, so the
        chassis touching down is the only thing that makes a low body illegal.
        """

        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 in self.ground_geom_ids:
                other = geom2
            elif geom2 in self.ground_geom_ids:
                other = geom1
            else:
                continue
            if other in self._foot_index:
                continue
            if self.model.geom_bodyid[other] == 0:
                continue
            if self._contact_normal_force(index) >= self.walk_config.contact_force_threshold:
                return True
        return False

    def _normalized_current_penalty(self) -> float:
        currents = []
        for motor_id in Actuator.Index.ALL:
            actuator_id = int(self.controller._actuator_ids[motor_id])
            voltage = float(self.data.ctrl[actuator_id])
            velocity = self.controller._joint_velocity(motor_id)
            spec = spec_for_motor_id(motor_id)
            current = (voltage - spec.K * velocity) / spec.R
            currents.append(current / (spec.stall_torque / spec.K))
        return float(np.mean(np.square(currents)))

    # -- observation --------------------------------------------------------

    def _observation(self, contact_flags: np.ndarray) -> np.ndarray:
        linear, angular, gravity = self._base_state()
        position, velocity = self._joint_state()
        heading_error = self._heading_error()
        observation = np.concatenate([
            linear / 2.0,
            angular / 5.0,
            gravity,
            (position - self.default_radians) / math.pi,
            velocity / 10.0,
            self._last_action,
            # Not clipped: above 1.0 means "faster than the reference range".
            self._command / OBSERVATION_COMMAND_SCALE,
            [
                math.sin(2.0 * math.pi * self._phase),
                math.cos(2.0 * math.pi * self._phase),
                math.sin(heading_error),
                math.cos(heading_error),
            ],
            contact_flags,
        ])
        noise = self.walk_config.observation_noise
        if noise > 0.0:
            observation = observation + self.np_random.normal(
                0.0, noise, observation.shape
            )
        return observation.astype(np.float32)

    # -- reward -------------------------------------------------------------

    def _reward(
        self, action: np.ndarray, contact_flags: np.ndarray, mean_slip: float
    ) -> tuple[float, dict[str, float], bool, dict[str, Any]]:
        config = self.reward_config
        dt = self.control_dt
        linear, angular, gravity = self._base_state()
        position, _ = self._joint_state()

        # -- speed: one-sided credit along the commanded direction ----------
        command_speed = float(np.linalg.norm(self._command[:2]))
        if command_speed > 1e-6:
            direction = self._command[:2] / command_speed
            progress = float(linear[:2] @ direction)
            drift = float(np.linalg.norm(linear[:2] - direction * progress))
            # min() is what removes the speed limit: full credit at the
            # command, and nothing to lose by overshooting it.
            speed_credit = min(progress, command_speed) / config.speed_reference
            drift_cost = drift / config.speed_reference
        else:
            progress = 0.0
            drift = 0.0
            speed_credit = 0.0
            drift_cost = 0.0

        command_yaw = float(self._command[2])
        if abs(command_yaw) > 1e-6:
            yaw_progress = float(angular[2] * math.copysign(1.0, command_yaw))
            yaw_credit = min(yaw_progress, abs(command_yaw)) / config.yaw_reference
        else:
            yaw_progress = 0.0
            yaw_credit = 0.0

        # -- orientation: the hard constraint ------------------------------
        heading_error = self._heading_error()
        yaw_hold = 1.0 - float(np.clip(
            abs(command_yaw) / config.yaw_hold_threshold, 0.0, 1.0
        ))
        heading_cost = yaw_hold * min(
            config.cost_cap, (heading_error / config.heading_error_sigma) ** 2
        )
        upright_cost = min(
            config.cost_cap,
            float(gravity[:2] @ gravity[:2]) / config.projected_gravity_sigma ** 2,
        )
        # Only the angular rates: vertical velocity is free by design.
        attitude_rate_cost = min(
            config.cost_cap,
            (angular[0] / config.roll_pitch_rate_sigma) ** 2
            + (angular[1] / config.roll_pitch_rate_sigma) ** 2,
        )

        # -- legs ----------------------------------------------------------
        down = contact_flags > 0.0
        self._contact_seconds_since[down] = 0.0
        self._contact_seconds_since[~down] += dt
        inactive = np.clip(
            self._contact_seconds_since - config.inactivity_seconds, 0.0, None
        )
        moving = float(np.clip(max(speed_credit, yaw_credit), 0.0, 1.0))
        inactivity_cost = float(np.sum(inactive)) * moving

        # -- effort, limits, idle ------------------------------------------
        joint_offset = np.abs(position - self.default_radians)
        if self._reference_override_unwrapped_lower:
            joint_offset[12:] = 0.0
        excess = np.maximum(0.0, joint_offset - config.soft_joint_offset)
        joint_limit_cost = float(np.mean(np.square(excess / math.radians(15.0))))
        action_rate = float(np.mean(np.square(action - self._last_action)))
        action_magnitude = float(np.mean(np.square(action)))
        current_cost = self._normalized_current_penalty()
        collision = self._forbidden_collision()

        idle_fraction = 1.0 - self._command_gate()
        idle_velocity_cost = min(
            config.idle_cost_cap,
            float(linear[:2] @ linear[:2]) / config.idle_linear_velocity_sigma ** 2
            + float(angular[2] ** 2) / config.idle_yaw_velocity_sigma ** 2,
        )

        terms = {
            "speed": config.speed_weight * speed_credit * dt,
            "drift": -config.drift_weight * drift_cost * dt,
            "yaw": config.yaw_weight * yaw_credit * dt,
            "heading": -config.heading_weight * heading_cost * dt,
            "upright": -config.upright_weight * upright_cost * dt,
            "attitude_rate": (
                -config.roll_pitch_rate_weight * attitude_rate_cost * dt
            ),
            "inactivity": -config.inactivity_weight * inactivity_cost * dt,
            "slip": -config.slip_weight * mean_slip * dt,
            "joint_limit": -config.joint_limit_weight * joint_limit_cost * dt,
            "action_rate": -config.action_rate_weight * action_rate * dt,
            "action_magnitude": (
                -config.action_magnitude_weight * action_magnitude * dt
            ),
            "current": -config.current_weight * current_cost * dt,
            "idle_velocity": (
                -config.idle_velocity_weight * idle_fraction * idle_velocity_cost * dt
            ),
            "idle_action": (
                -config.idle_action_weight * idle_fraction * action_magnitude * dt
            ),
            "collision": -config.collision_weight * float(collision) * dt,
        }

        height = float(self.data.qpos[self.root_qpos_address + 2])
        tilt_degrees = math.degrees(
            math.acos(float(np.clip(-gravity[2], -1.0, 1.0)))
        )
        finite = bool(
            np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()
        )
        # Attitude and collision end an episode. Height never does.
        tilted = tilt_degrees > self.walk_config.max_tilt_degrees
        hard_joint_limit = bool(np.any(joint_offset > config.hard_joint_offset))
        terminated = (not finite) or tilted or collision or hard_joint_limit
        total = float(sum(terms.values()))
        if terminated:
            terms["termination"] = -config.termination_penalty
            total -= config.termination_penalty

        diagnostics: dict[str, Any] = {
            "vx": float(linear[0]),
            "vy": float(linear[1]),
            "vz": float(linear[2]),
            "yaw_rate": float(angular[2]),
            "progress": progress,
            "drift": drift,
            "speed_credit": speed_credit,
            "yaw_credit": yaw_credit,
            "heading_error": heading_error,
            "tilt_degrees": tilt_degrees,
            "height": height,
            "height_change": height - self._reference_height,
            "command_activity": self._command_activity(),
            "reference_cadence": self._reference_cadence,
            "reference_stride_degrees": self._reference_stride_degrees,
            "stance_contacts": int(down.sum()),
            "foot_contact": contact_flags.tolist(),
            "tilted": tilted,
            "forbidden_collision": bool(collision),
            "hard_joint_limit": hard_joint_limit,
        }
        return total, terms, terminated, diagnostics

    # -- gym API ------------------------------------------------------------

    def _randomize_model(self) -> None:
        config = self.walk_config
        mass = float(self.np_random.uniform(*config.mass_scale_range))
        friction = float(self.np_random.uniform(*config.friction_scale_range))
        strength = float(self.np_random.uniform(*config.strength_scale_range))
        # Always restart from nominal: multiplying the live model on every
        # reset is what silently drifted v2's motor dynamics (docs/21 P0-2).
        self.model.body_mass[:] = self._nominal_body_mass * mass
        self.model.body_inertia[:] = self._nominal_body_inertia * mass
        self.model.geom_friction[:] = self._nominal_geom_friction
        self.model.geom_friction[:, 0] *= friction
        self.model.actuator_gainprm[:] = self._nominal_actuator_gainprm
        self.model.actuator_forcerange[:] = self._nominal_actuator_forcerange
        # dcmotor gainprm[0] is terminal resistance R: dividing it scales the
        # whole torque-speed line without moving the no-load speed.
        self.model.actuator_gainprm[:, 0] /= strength
        self.model.actuator_forcerange[:] *= strength
        mujoco.mj_setConst(self.model, self.data)

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        del options
        mujoco.mj_resetData(self.model, self.data)
        self._randomize_model()
        self.controller = MuJoCoController(
            self.model,
            self.data,
            verbose=False,
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
            self.data.qpos[self.root_qpos_address + 3: self.root_qpos_address + 7] = (
                np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])
            )

        self._phase = float(self.np_random.random())
        self._episode_step = 0
        self._next_command_step = 0
        self._last_action.fill(0.0)
        self._contact_seconds_since.fill(0.0)
        self._reference_cadence = 0.0
        self._reference_stride_degrees = 0.0
        self.set_reference_override(None)
        self._command.fill(0.0)
        self._command_target = self._sample_command()
        self._schedule_next_command()
        if self.fixed_command is not None:
            self._command[:] = self.fixed_command

        for _ in range(
            round(self.walk_config.settle_seconds / self.model.opt.timestep)
        ):
            self.controller.update(self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)
        # mj_resetData leaves no contacts and no derived state, so without this
        # a zero-length settle would hand the policy an observation whose
        # contact flags and velocities are all zero.
        mujoco.mj_forward(self.model, self.data)

        self._target_heading = self._heading_yaw()
        # Recorded for diagnostics only. Nothing in the reward or the
        # termination rule reads it: body height is free in v3.
        self._reference_height = float(self.data.qpos[self.root_qpos_address + 2])
        contact_flags, _ = self._foot_contacts()
        return self._observation(contact_flags), {
            "command": self._command.copy(),
            "command_target": self._command_target.copy(),
            "reference_height": self._reference_height,
        }

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        applied = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        self._update_command()
        if abs(self._command[2]) > self.reward_config.yaw_hold_threshold:
            # Turning: follow the body instead of integrating a target the
            # policy may track a few percent slow.
            self._target_heading = self._heading_yaw()
        else:
            self._target_heading += self._command[2] * self.control_dt
        self._apply_action(applied)

        for _ in range(self.walk_config.frame_skip):
            self.controller.update(self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)

        contact_flags, mean_slip = self._foot_contacts()
        reward, terms, terminated, diagnostics = self._reward(
            applied, contact_flags, mean_slip
        )
        self._last_action[:] = applied
        self._episode_step += 1
        truncated = self._episode_step >= self.max_episode_steps
        info: dict[str, Any] = {
            "reward_terms": terms,
            "command": self._command.copy(),
            **diagnostics,
        }
        observation = self._observation(contact_flags)
        if self.render_mode == "human":
            self.render()
        return observation, reward, terminated, truncated, info

    def advance_external_control(self) -> None:
        """Keep MuJoCo and its viewer alive during legacy mode transitions."""

        for _ in range(self.walk_config.frame_skip):
            with self.controller.lock:
                self.controller.update(self.model.opt.timestep)
                mujoco.mj_step(self.model, self.data)
        if self.render_mode == "human":
            self.render()

    def resume_after_external_control(self) -> np.ndarray:
        """Re-align policy state after a blocking legacy mode hands back."""

        self.fixed_command = np.zeros(3, dtype=np.float64)
        self._command.fill(0.0)
        self._command_target.fill(0.0)
        self._last_action.fill(0.0)
        self._contact_seconds_since.fill(0.0)
        self.set_reference_override(None)
        self._target_heading = self._heading_yaw()
        self._reference_height = float(self.data.qpos[self.root_qpos_address + 2])
        contact_flags, _ = self._foot_contacts()
        return self._observation(contact_flags)

    def render(self) -> None:  # pragma: no cover - viewer only
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

    def close(self) -> None:  # pragma: no cover - viewer only
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        if hasattr(self, "controller"):
            self.controller.close()


__all__ = [
    "CHECKPOINT_PREFIX",
    "CURRICULUM_RANGES",
    "OBSERVATION_COMMAND_SCALE",
    "OBSERVATION_DIM",
    "REFERENCE_CHOICES",
    "RewardConfig",
    "SconeWalkEnvV3",
    "WalkConfig",
    "build_parser",
    "main",
    "normalize_reference_motion",
    "training_walk_config",
]


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------
# The argument layout matches src.rl.walk_learn and src.rl.walk_v2 so the SSH
# launcher in src.rl.inquiry drives all three trainers with one
# ``build_training_arguments`` output and the same pause/resume contract:
#
#   runs/<name>/checkpoints/scone_walk_v3_<steps>_steps.zip   rolling saves
#   runs/<name>/resume.checkpoint                             pointer to load
#   runs/<name>/train.log | train.pid | train.state           launcher files


def _validate_training_batch_size(n_steps: int, num_envs: int, batch_size: int) -> None:
    """Reject PPO minibatches that would truncate the rollout buffer."""

    if n_steps < 1 or num_envs < 1 or batch_size < 1:
        raise ValueError("n_steps, num_envs, and batch_size must all be positive")
    rollout_size = n_steps * num_envs
    if batch_size > rollout_size or rollout_size % batch_size:
        divisors = [
            candidate
            for candidate in range(1, min(batch_size, rollout_size) + 1)
            if rollout_size % candidate == 0
        ]
        suggestions = ", ".join(str(value) for value in divisors[-4:])
        raise ValueError(
            f"batch size {batch_size} must divide n_steps * num_envs "
            f"({n_steps} * {num_envs} = {rollout_size}); "
            f"nearby valid values: {suggestions}"
        )


def _ppo_training_kwargs(args: Any) -> dict[str, Any]:
    """PPO settings for a new v3 run.

    The action distribution is the part of walk_v2 worth keeping: gSDE with a
    squashed policy means PPO scores the bounded action the environment
    receives. Without it, v2 measured 84-95 percent of raw action components
    stuck at the clip while the latent mean ran up to 19x the action range.
    """

    _validate_training_batch_size(args.n_steps, args.num_envs, args.batch_size)
    if args.learning_rate <= 0.0:
        raise ValueError("learning rate must be positive")
    if args.target_kl <= 0.0:
        raise ValueError("target KL must be positive")
    if args.n_epochs < 1:
        raise ValueError("n_epochs must be at least 1")
    if args.max_grad_norm <= 0.0:
        raise ValueError("max_grad_norm must be positive")
    if args.sde_sample_freq == 0 or args.sde_sample_freq < -1:
        raise ValueError("sde_sample_freq must be -1 or a positive integer")
    if not math.isfinite(args.log_std_init):
        raise ValueError("log_std_init must be finite")

    return {
        "learning_rate": args.learning_rate,
        "n_steps": args.n_steps,
        "batch_size": args.batch_size,
        "n_epochs": args.n_epochs,
        # 0.995 at 50 Hz is a 4 s horizon, which is several gait cycles.
        "gamma": 0.995,
        "gae_lambda": 0.95,
        "clip_range": 0.2,
        "ent_coef": args.entropy_coefficient,
        "max_grad_norm": args.max_grad_norm,
        "target_kl": args.target_kl,
        "use_sde": True,
        "sde_sample_freq": args.sde_sample_freq,
        "policy_kwargs": {
            "net_arch": {"pi": [256, 256], "vf": [256, 256]},
            "squash_output": True,
            "log_std_init": args.log_std_init,
            "use_expln": True,
        },
    }


def _require_bounded_resume_policy(model: Any) -> None:
    """Prevent an unbounded clipped-Gaussian policy from entering a v3 run."""

    if not bool(getattr(model, "use_sde", False)) or not bool(
        getattr(getattr(model, "policy", None), "squash_output", False)
    ):
        raise ValueError(
            "this checkpoint uses an unbounded clipped-Gaussian policy; "
            "start a new walk_v3 run instead of resuming it"
        )


def _validate_evaluation_arguments(args: Any) -> None:
    if args.eval_every is not None and args.eval_every < 1:
        raise ValueError("eval_every must be positive when specified")
    if args.eval_episodes < 1:
        raise ValueError("eval_episodes must be at least 1")
    if not math.isfinite(args.eval_seconds) or args.eval_seconds <= 0.0:
        raise ValueError("eval_seconds must be a positive finite number")


# The suite spans the measured scaffold envelope and then asks for more than it
# can give (forward_060), because a design with no speed limit has to be
# scored on how much of an out-of-range request it can deliver.
FIXED_EVALUATION_COMMANDS: tuple[tuple[str, tuple[float, float, float]], ...] = (
    ("idle", (0.0, 0.0, 0.0)),
    ("forward_010", (0.10, 0.0, 0.0)),
    ("forward_020", (0.20, 0.0, 0.0)),
    ("forward_030", (0.30, 0.0, 0.0)),
    ("forward_060", (0.60, 0.0, 0.0)),
    ("reverse_015", (-0.15, 0.0, 0.0)),
    ("lateral_008", (0.0, 0.08, 0.0)),
    ("yaw_060", (0.0, 0.0, 0.60)),
)
IDLE_SPEED_TOLERANCE = 0.05                    # m/s that scores a full idle miss


def _fixed_evaluation_score(records: Sequence[dict[str, Any]]) -> dict[str, float | int]:
    """Score command delivery, not dense reward.

    Per command: how much of the requested speed was actually delivered along
    the requested direction, minus sideways drift, and for an idle command how
    much unwanted motion appeared. Survival and attitude enter as separate
    factors so a fast policy that ends up on its side cannot win.
    """

    if not records:
        raise ValueError("fixed evaluation requires at least one record")
    delivered: list[float] = []
    headings: list[float] = []
    tilts: list[float] = []
    survived: list[float] = []
    speeds: list[float] = []
    direction_failures = 0
    for record in records:
        command = np.asarray(record["command"], dtype=np.float64)
        command_speed = float(np.linalg.norm(command[:2]))
        command_yaw = float(command[2])
        if command_speed > 1e-6:
            direction = command[:2] / command_speed
            achieved = np.asarray([record["vx"], record["vy"]], dtype=np.float64)
            progress = float(achieved @ direction)
            drift = float(np.linalg.norm(achieved - direction * progress))
            delivered.append(
                float(np.clip(progress / command_speed, -1.0, 1.0))
                - drift / command_speed
            )
            speeds.append(progress)
            if progress <= 0.0:
                direction_failures += 1
        elif abs(command_yaw) > 1e-6:
            rate = float(record["yaw_rate"]) * math.copysign(1.0, command_yaw)
            delivered.append(float(np.clip(rate / abs(command_yaw), -1.0, 1.0)))
            if rate <= 0.0:
                direction_failures += 1
        else:
            motion = math.hypot(float(record["vx"]), float(record["vy"]))
            delivered.append(-min(1.0, motion / IDLE_SPEED_TOLERANCE))
        headings.append(abs(float(record["heading_error"])))
        tilts.append(float(record["tilt_degrees"]))
        survived.append(float(bool(record["survived"])))

    survival_rate = float(np.mean(survived))
    command_delivery = float(np.mean(delivered))
    heading_error = float(np.mean(headings))
    peak_tilt = float(np.max(tilts))
    score = (
        command_delivery
        + survival_rate
        - 0.10 * heading_error / math.pi
    )
    return {
        "score": score,
        "command_delivery": command_delivery,
        "survival_rate": survival_rate,
        "heading_error": heading_error,
        "peak_tilt_degrees": peak_tilt,
        "top_speed": float(max(speeds)) if speeds else 0.0,
        "direction_failures": direction_failures,
    }


def _write_json_atomic(path: Path, payload: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def _write_resume_pointer(run_dir: Path, checkpoint: Path) -> None:
    """Atomically record the exact checkpoint a resume should load."""

    try:
        stored = checkpoint.relative_to(run_dir)
    except ValueError:
        stored = checkpoint
    pointer = run_dir / "resume.checkpoint"
    temporary = pointer.with_suffix(".tmp")
    temporary.write_text(f"{stored}\n", encoding="utf-8")
    temporary.replace(pointer)


def _env_factory(args: Any, index: int, curriculum: str):
    def build() -> SconeWalkEnvV3:
        return SconeWalkEnvV3(
            args.model,
            curriculum=curriculum,
            reference_motion=args.reference_motion,
            terrain=args.terrain,
            terrain_seed=args.terrain_seed + index,
            standing_pose_degrees=args.standing_pose_degrees,
            fixed_command=getattr(args, "command", None),
            walk_config=training_walk_config(curriculum),
        )

    return build


def _evaluation_walk_config(seconds: float) -> WalkConfig:
    """Nominal robot, fixed length: the environment a checkpoint is judged in."""

    return replace(WalkConfig(), episode_seconds=seconds)


def _make_callbacks(run_dir: Path, args: Any, stop_requested):
    import re

    from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback

    class PruningCheckpointCallback(CheckpointCallback):
        """Save checkpoints, keep the newest N, publish a resume pointer."""

        def __init__(self, *a: Any, keep_last: int = 10, **kw: Any) -> None:
            super().__init__(*a, **kw)
            self.keep_last = max(1, keep_last)

        def _on_step(self) -> bool:
            result = super()._on_step()
            if self.n_calls % self.save_freq == 0:
                saved = (
                    Path(self.save_path)
                    / f"{self.name_prefix}_{self.num_timesteps}_steps.zip"
                )
                if saved.is_file():
                    _write_resume_pointer(Path(self.save_path).parent, saved)
                found: list[tuple[int, Path]] = []
                for path in Path(self.save_path).glob(
                    f"{self.name_prefix}_*_steps.zip"
                ):
                    match = re.search(r"_([0-9]+)_steps\.zip$", path.name)
                    if match:
                        found.append((int(match.group(1)), path))
                found.sort(key=lambda item: item[0])
                for _, path in found[: -self.keep_last]:
                    path.unlink(missing_ok=True)
            return result

    class RewardTermsCallback(BaseCallback):
        """Average reward and action diagnostics over the whole rollout."""

        def __init__(self) -> None:
            super().__init__()
            self._totals: dict[str, float] = {}
            self._count = 0
            self._action_totals: dict[str, float] = {}
            self._action_count = 0

        def _on_step(self) -> bool:
            for info in self.locals.get("infos", []):
                terms = info.get("reward_terms")
                if not terms:
                    continue
                self._count += 1
                for key, value in terms.items():
                    self._totals[key] = self._totals.get(key, 0.0) + float(value)
                for key in (
                    "vx", "vy", "yaw_rate", "progress", "drift", "tilt_degrees",
                    "height_change", "stance_contacts", "reference_cadence",
                    "reference_stride_degrees",
                ):
                    if key in info:
                        self._totals[f"state/{key}"] = (
                            self._totals.get(f"state/{key}", 0.0) + float(info[key])
                        )
            actions = np.asarray(
                self.locals.get("clipped_actions", self.locals.get("actions", [])),
                dtype=np.float64,
            )
            if actions.size:
                self._action_count += 1
                self._action_totals["abs_mean"] = (
                    self._action_totals.get("abs_mean", 0.0)
                    + float(np.mean(np.abs(actions)))
                )
                self._action_totals["saturation_fraction"] = (
                    self._action_totals.get("saturation_fraction", 0.0)
                    + float(np.mean(np.abs(actions) >= 0.98))
                )
            return True

        def _on_rollout_end(self) -> None:
            if self._count:
                for key, value in self._totals.items():
                    name = key if key.startswith("state/") else f"reward/{key}"
                    self.logger.record(name, value / self._count)
            if self._action_count:
                for key, value in self._action_totals.items():
                    self.logger.record(f"action/{key}", value / self._action_count)
            self._totals.clear()
            self._count = 0
            self._action_totals.clear()
            self._action_count = 0

    class FixedCommandEvalCallback(BaseCallback):
        """Promote a best model only when it beats the zero-residual gait."""

        def __init__(self, eval_freq: int) -> None:
            super().__init__()
            self.eval_freq = max(1, eval_freq)
            self.eval_env: SconeWalkEnvV3 | None = None
            self.baseline: dict[str, Any] | None = None
            self.best_candidate_score = -math.inf
            existing = run_dir / "best_candidate_metrics.json"
            if existing.is_file():
                try:
                    self.best_candidate_score = float(
                        json.loads(existing.read_text(encoding="utf-8"))
                        ["summary"]["score"]
                    )
                except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
                    self.best_candidate_score = -math.inf

        def _on_training_start(self) -> None:
            self.eval_env = SconeWalkEnvV3(
                args.model,
                curriculum="full",
                reference_motion=args.reference_motion,
                fixed_command=(0.0, 0.0, 0.0),
                walk_config=_evaluation_walk_config(args.eval_seconds),
                terrain="flat",
                terrain_seed=args.terrain_seed,
                standing_pose_degrees=args.standing_pose_degrees,
            )
            baseline_path = run_dir / "evaluation_baseline.json"
            if baseline_path.is_file():
                try:
                    self.baseline = json.loads(
                        baseline_path.read_text(encoding="utf-8")
                    )
                except (OSError, json.JSONDecodeError):
                    self.baseline = None
            if self.baseline is None:
                self.baseline = self._evaluate(zero_residual=True)
                _write_json_atomic(baseline_path, self.baseline)

        def _evaluate(self, *, zero_residual: bool) -> dict[str, Any]:
            if self.eval_env is None:
                raise RuntimeError("evaluation environment is not initialized")
            records: list[dict[str, Any]] = []
            for name, command in FIXED_EVALUATION_COMMANDS:
                self.eval_env.set_velocity_command(command)
                for episode in range(args.eval_episodes):
                    observation, _ = self.eval_env.reset(seed=args.seed + episode)
                    samples: list[np.ndarray] = []
                    headings: list[float] = []
                    tilts: list[float] = []
                    total_return = 0.0
                    terminated = truncated = False
                    while not (terminated or truncated):
                        if zero_residual:
                            action = np.zeros(18, dtype=np.float32)
                        else:
                            action, _ = self.model.predict(
                                observation, deterministic=True
                            )
                        observation, reward, terminated, truncated, info = (
                            self.eval_env.step(action)
                        )
                        total_return += float(reward)
                        samples.append(
                            np.array([info["vx"], info["vy"], info["yaw_rate"]])
                        )
                        headings.append(abs(float(info["heading_error"])))
                        tilts.append(float(info["tilt_degrees"]))
                    achieved = np.mean(samples, axis=0) if samples else np.zeros(3)
                    records.append({
                        "name": name,
                        "episode": episode,
                        "command": list(command),
                        "return": total_return,
                        "vx": float(achieved[0]),
                        "vy": float(achieved[1]),
                        "yaw_rate": float(achieved[2]),
                        "heading_error": (
                            float(np.mean(headings)) if headings else math.inf
                        ),
                        "tilt_degrees": float(np.max(tilts)) if tilts else 90.0,
                        "survived": bool(truncated and not terminated),
                    })
            return {
                "num_timesteps": int(self.num_timesteps),
                "zero_residual": zero_residual,
                "summary": _fixed_evaluation_score(records),
                "records": records,
            }

        def _on_step(self) -> bool:
            if self.n_calls % self.eval_freq:
                return True
            result = self._evaluate(zero_residual=False)
            summary = result["summary"]
            baseline_summary = self.baseline["summary"] if self.baseline else {}
            score = float(summary["score"])
            baseline_score = float(baseline_summary.get("score", -math.inf))
            # Beat the scaffold overall, and do not go the wrong way on an axis
            # the scaffold got right. Demanding zero failures outright would be
            # unpromotable by construction: the hardcoded reference has no
            # lateral scaffold, so it fails that axis itself.
            baseline_failures = int(baseline_summary.get("direction_failures", 0))
            promoted = (
                score > baseline_score
                and int(summary["direction_failures"]) <= baseline_failures
            )
            result["baseline_score"] = baseline_score
            result["beats_zero_residual"] = promoted

            with (run_dir / "evaluation_history.jsonl").open(
                "a", encoding="utf-8"
            ) as history:
                history.write(json.dumps(result, ensure_ascii=False) + "\n")
            for key in (
                "score", "command_delivery", "survival_rate", "heading_error",
                "peak_tilt_degrees", "top_speed", "direction_failures",
            ):
                self.logger.record(f"eval/fixed_{key}", float(summary[key]))
            self.logger.record("eval/zero_residual_score", baseline_score)
            self.logger.record("eval/beats_zero_residual", float(promoted))

            if score > self.best_candidate_score:
                self.best_candidate_score = score
                self.model.save(run_dir / "best_candidate_model.zip")
                _write_json_atomic(run_dir / "best_candidate_metrics.json", result)
            if promoted:
                metrics_path = run_dir / "best_model_metrics.json"
                previous = -math.inf
                if metrics_path.is_file():
                    try:
                        previous = float(
                            json.loads(metrics_path.read_text(encoding="utf-8"))
                            ["summary"]["score"]
                        )
                    except (
                        OSError, KeyError, TypeError, ValueError, json.JSONDecodeError
                    ):
                        previous = -math.inf
                if score > previous:
                    self.model.save(run_dir / "best_model.zip")
                    _write_json_atomic(metrics_path, result)
            print(
                f"[RL eval] step={self.num_timesteps} score={score:.4f} "
                f"zero={baseline_score:.4f} "
                f"top_speed={float(summary['top_speed']):.3f} m/s "
                f"tilt={float(summary['peak_tilt_degrees']):.1f} deg "
                f"promoted={'yes' if promoted else 'no'}",
                flush=True,
            )
            return True

        def _on_training_end(self) -> None:
            if self.eval_env is not None:
                self.eval_env.close()
                self.eval_env = None

    class GracefulStopCallback(BaseCallback):
        def _on_step(self) -> bool:
            return not stop_requested()

    return [
        PruningCheckpointCallback(
            save_freq=max(1, args.checkpoint_every // max(1, args.num_envs)),
            save_path=str(run_dir / "checkpoints"),
            name_prefix=CHECKPOINT_PREFIX,
            keep_last=args.keep_checkpoints,
        ),
        RewardTermsCallback(),
        FixedCommandEvalCallback(
            eval_freq=max(
                1,
                (args.eval_every or args.checkpoint_every) // max(1, args.num_envs),
            )
        ),
        GracefulStopCallback(),
    ]


def _bar(value: float, scale: float, width: int = 18) -> str:
    if scale <= 0.0:
        return " " * width
    half = width // 2
    filled = int(round(min(1.0, abs(value) / scale) * half))
    if value >= 0.0:
        return " " * half + "█" * filled + " " * (half - filled)
    return " " * (half - filled) + "█" * filled + " " * half


def run_check(args: Any) -> int:
    """Roll out one episode and print what the scaffold and reward produce."""

    env = SconeWalkEnvV3(
        args.model,
        curriculum=args.curriculum,
        reference_motion=args.reference_motion,
        terrain=args.terrain,
        terrain_seed=args.terrain_seed,
        standing_pose_degrees=args.standing_pose_degrees,
        fixed_command=args.command,
    )
    observation, _ = env.reset(seed=args.seed)
    if observation.shape != (OBSERVATION_DIM,):
        env.close()
        raise RuntimeError(f"unexpected observation shape {observation.shape}")

    contact_steps = np.zeros(6)
    totals: dict[str, float] = {}
    samples: list[tuple[float, float, float]] = []
    heights: list[float] = []
    tilts: list[float] = []
    total = 0.0
    steps = 0
    terminated = truncated = False
    info: dict[str, Any] = {}
    try:
        for _ in range(args.steps):
            action = (
                env.action_space.sample()
                if args.random_actions
                else np.zeros(18, dtype=np.float32)
            )
            observation, reward, terminated, truncated, info = env.step(action)
            if not np.isfinite(observation).all():
                raise RuntimeError("non-finite observation produced")
            total += reward
            steps += 1
            contact_steps += np.asarray(info["foot_contact"], dtype=np.float64)
            samples.append((info["vx"], info["vy"], info["yaw_rate"]))
            heights.append(info["height_change"])
            tilts.append(info["tilt_degrees"])
            for key, value in info["reward_terms"].items():
                totals[key] = totals.get(key, 0.0) + float(value)
            if terminated or truncated:
                break
        command = np.asarray(info.get("command", np.zeros(3)), dtype=np.float64)
        mean = np.mean(samples, axis=0) if samples else np.zeros(3)
        width = 64
        ended = (
            "terminated" if terminated
            else "truncated" if truncated
            else "step limit"
        )
        print()
        print("─" * width)
        print(f" walk_v3   reference {env.reference_motion}"
              f"   curriculum {args.curriculum}   {ended}")
        print("─" * width)
        print(f" steps {steps:4d} ({steps * env.control_dt:5.2f} s)"
              f"    return {total:9.3f}")
        print(f" reference stride {info.get('reference_stride_degrees', 0.0):5.1f} deg"
              f"   cadence {info.get('reference_cadence', 0.0):4.2f} Hz")
        print()
        print("            commanded      achieved     delivered")
        for index, (name, unit) in enumerate(
            ((" vx", "m/s"), (" vy", "m/s"), ("yaw", "rad/s"))
        ):
            target = float(command[index])
            got = float(mean[index])
            ratio = f"{100.0 * got / target:6.0f}%" if abs(target) > 1e-3 else "     -"
            print(f"   {name}     {target:+8.3f}      {got:+8.3f} {unit:<6s} {ratio}")
        print()
        print(" free height, locked attitude")
        print(f"   body height change   min {min(heights, default=0.0):+.4f} m"
              f"   max {max(heights, default=0.0):+.4f} m   (unconstrained)")
        print(f"   body tilt            max {max(tilts, default=0.0):6.2f} deg"
              f"   limit {env.walk_config.max_tilt_degrees:.1f} deg")
        print()
        print(" contact duty per leg")
        print("   " + "  ".join(
            f"L{leg}:{100.0 * contact_steps[leg - 1] / max(1, steps):5.1f}%"
            for leg in range(1, 7)
        ))
        print()
        print(" reward terms, summed over the rollout")
        ranked = sorted(totals.items(), key=lambda item: -abs(item[1]))
        scale = max((abs(value) for _, value in ranked), default=1.0)
        for key, value in ranked:
            if abs(value) < 1e-9:
                continue
            print(f"   {key:<18s} {value:+9.4f}  |{_bar(value, scale)}|")
        zeroed = [key for key, value in ranked if abs(value) < 1e-9]
        if zeroed:
            print(f"   (zero: {', '.join(zeroed)})")
        print("─" * width)
    finally:
        env.close()
    return 0


def run_train(args: Any) -> int:
    import signal

    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import (
        DummyVecEnv,
        SubprocVecEnv,
        VecMonitor,
    )

    _validate_evaluation_arguments(args)
    new_model_kwargs = None if args.resume is not None else _ppo_training_kwargs(args)

    run_dir = Path(args.output).expanduser().resolve()
    (run_dir / "checkpoints").mkdir(parents=True, exist_ok=True)
    factories = [
        _env_factory(args, index, args.curriculum) for index in range(args.num_envs)
    ]
    vector = DummyVecEnv(factories) if args.num_envs == 1 else SubprocVecEnv(factories)
    monitor_name = (
        "monitor.csv" if args.resume is None
        else f"monitor_resume_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    )
    env = VecMonitor(vector, filename=str(run_dir / monitor_name))

    stop_requested = False

    def request_stop(signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        if not stop_requested:
            print(
                f"[RL] signal {signum} received; finishing the current rollout "
                "and saving a resume checkpoint...",
                flush=True,
            )
        stop_requested = True

    callbacks = _make_callbacks(run_dir, args, lambda: stop_requested)
    tensorboard_log = (
        None if args.tensorboard_log is None
        else str(Path(args.tensorboard_log).expanduser().resolve())
    )
    if args.resume is None:
        model = PPO(
            "MlpPolicy",
            env,
            **new_model_kwargs,
            tensorboard_log=tensorboard_log,
            verbose=1,
            seed=args.seed,
            device=args.device,
        )
        reset_timesteps = True
    else:
        model = PPO.load(
            Path(args.resume).expanduser().resolve(),
            env=env,
            device=args.device,
            tensorboard_log=tensorboard_log,
        )
        _require_bounded_resume_policy(model)
        reset_timesteps = False

    previous = {
        signum: signal.signal(signum, request_stop)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=callbacks,
            reset_num_timesteps=reset_timesteps,
        )
        final_model = run_dir / "final_model.zip"
        model.save(final_model)
        _write_resume_pointer(run_dir, final_model)
        print(f"[RL] saved {final_model}", flush=True)
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        env.close()
    if stop_requested:
        print("[RL] stopped on request; resume with --resume", flush=True)
    return 0


def run_enjoy(args: Any) -> int:
    from stable_baselines3 import PPO

    env = SconeWalkEnvV3(
        args.model,
        curriculum=args.curriculum,
        reference_motion=args.reference_motion,
        fixed_command=args.command,
        terrain=args.terrain,
        terrain_seed=args.terrain_seed,
        standing_pose_degrees=args.standing_pose_degrees,
        render_mode=None if args.headless else "human",
    )
    model = PPO.load(Path(args.checkpoint).expanduser().resolve(), device=args.device)
    try:
        for episode in range(args.episodes):
            observation, _ = env.reset(seed=args.seed + episode)
            total = 0.0
            speeds: list[float] = []
            done = False
            while not done:
                frame_start = time.perf_counter()
                action, _ = model.predict(observation, deterministic=True)
                observation, reward, terminated, truncated, info = env.step(action)
                total += float(reward)
                speeds.append(float(info["progress"]))
                done = terminated or truncated
                if not args.headless:
                    remaining = env.control_dt - (time.perf_counter() - frame_start)
                    if remaining > 0.0:
                        time.sleep(remaining)
                    if env._viewer is not None and not env._viewer.is_running():
                        done = True
                        episode = args.episodes
            print(
                f"episode {episode}: return {total:8.2f}   mean progress "
                f"{float(np.mean(speeds)) if speeds else 0.0:+.3f} m/s",
                flush=True,
            )
    finally:
        env.close()
    return 0


def build_parser():
    import argparse

    parser = argparse.ArgumentParser(
        prog="python -m src.rl.walk_v3",
        description=(
            "SCONE walking PPO v3: residual on a speed-scaled hardcoded gait, "
            "free body height, locked attitude"
        ),
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    parser.add_argument(
        "--terrain", choices=TERRAIN_CHOICES, default=TerrainType.FLAT.value
    )
    parser.add_argument("--terrain-seed", type=int, default=7)
    parser.add_argument(
        "--reference-motion",
        default="hardcoded",
        help="baseline the residual corrects: hardcoded | none (end to end)",
    )
    parser.add_argument(
        "--stance",
        choices=tuple(STANCE_PRESETS),
        default=None,
        help="named standing pose; ignored when --standing-pose-degrees is given",
    )
    parser.add_argument(
        "--standing-pose-degrees",
        type=float,
        nargs=18,
        metavar="DEG",
        default=None,
        help="18 actuator degrees for the standing pose",
    )
    sub = parser.add_subparsers(dest="command_name", required=True)

    check = sub.add_parser("check", help="one rollout, no learning")
    check.add_argument("--curriculum", choices=tuple(CURRICULUM_RANGES), default="easy")
    check.add_argument("--steps", type=int, default=500)
    check.add_argument("--seed", type=int, default=0)
    check.add_argument("--random-actions", action="store_true")
    check.add_argument(
        "--command",
        type=float,
        nargs=3,
        metavar=("VX", "VY", "YAW"),
        default=None,
        help="hold one command instead of sampling, e.g. --command 0.30 0 0",
    )
    check.set_defaults(func=run_check)

    train = sub.add_parser("train", help="headless training, SSH friendly")
    train.add_argument("--curriculum", choices=tuple(CURRICULUM_RANGES), default="easy")
    train.add_argument("--timesteps", type=int, default=20_000_000)
    train.add_argument("--num-envs", type=int, default=8)
    train.add_argument("--checkpoint-every", type=int, default=500_000)
    train.add_argument("--keep-checkpoints", type=int, default=10)
    train.add_argument("--n-steps", type=int, default=512)
    # 512 divides 512 * num_envs for every positive num_envs, so a change of
    # remote host capacity cannot create a truncated final minibatch.
    train.add_argument("--batch-size", type=int, default=512)
    train.add_argument("--learning-rate", type=float, default=2e-4)
    train.add_argument("--n-epochs", type=int, default=4)
    train.add_argument("--target-kl", type=float, default=0.03)
    train.add_argument("--entropy-coefficient", type=float, default=0.0)
    train.add_argument("--max-grad-norm", type=float, default=0.5)
    train.add_argument("--sde-sample-freq", type=int, default=4)
    train.add_argument("--log-std-init", type=float, default=-2.0)
    train.add_argument(
        "--eval-every",
        type=int,
        default=None,
        help="fixed-command evaluation interval; defaults to the checkpoint interval",
    )
    train.add_argument("--eval-episodes", type=int, default=2)
    train.add_argument("--eval-seconds", type=float, default=8.0)
    train.add_argument("--seed", type=int, default=0)
    train.add_argument("--device", default="auto")
    train.add_argument("--output", type=Path, default=Path("runs/walk_v3"))
    train.add_argument("--tensorboard-log", default=None)
    train.add_argument("--resume", type=Path, default=None)
    train.set_defaults(func=run_train)

    enjoy = sub.add_parser("enjoy", help="replay a checkpoint")
    enjoy.add_argument("checkpoint")
    enjoy.add_argument("--curriculum", choices=tuple(CURRICULUM_RANGES), default="full")
    enjoy.add_argument("--episodes", type=int, default=3)
    enjoy.add_argument("--seed", type=int, default=0)
    enjoy.add_argument("--device", default="auto")
    enjoy.add_argument(
        "--command",
        type=float,
        nargs=3,
        metavar=("VX", "VY", "YAW"),
        default=[0.30, 0.0, 0.0],
        help="hold one velocity command while replaying; not clipped",
    )
    enjoy.add_argument("--headless", action="store_true")
    enjoy.set_defaults(func=run_enjoy)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    args.reference_motion = normalize_reference_motion(args.reference_motion)
    if args.reference_motion not in REFERENCE_CHOICES:
        raise SystemExit(
            f"unknown reference motion {args.reference_motion!r}; "
            f"choose from {REFERENCE_CHOICES}"
        )
    if args.standing_pose_degrees is None and args.stance is not None:
        args.standing_pose_degrees = list(STANCE_PRESETS[args.stance])
    args.model = Path(args.model).expanduser().resolve()
    if not args.model.exists():
        raise SystemExit(f"model not found: {args.model}")
    return int(args.func(args))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
