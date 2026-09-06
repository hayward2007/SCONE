"""Fail-safe residual-RL walking: keep going after a leg stops working.

SCONE has six legs and an alternating tripod needs all of them.  Groups
``(1, 4, 5)`` and ``(2, 3, 6)`` each carry the robot alone, so the moment one
leg is lost its group is a two-foot line and the machine tips about it twice a
cycle.  That is not something a residual policy can repair: the scaffold it
rides on is statically unstable by construction.

This trainer therefore changes two things at once.

1.  The **scaffold** becomes :class:`~src.locomotion.fault_gait.FaultAdaptiveGait`,
    which reschedules itself around the working legs -- a wave gait with one
    foot in the air at a time, plus a body shift that puts the centre of mass
    back inside the support polygon.  Measured on the nominal stance, losing
    any single leg leaves the tripod scaffold at a worst-case margin of
    ``-3.2 mm`` (outside) and the rescheduled scaffold at ``+42..+54 mm``.
2.  The **policy** is told what is broken.  Six health flags, six contact
    flags and three support-polygon features are appended to the v1
    observation, so one checkpoint covers the healthy robot and every fault
    case instead of needing a policy per fault.

The objective is deliberately not v3's.  ``walk_v3`` maximises speed under a
stability gate; this one maximises how accurately the robot holds a commanded
motion, an attitude and a support margin, and treats speed only as the command
it was asked to track.  It is built from ``walk_learn`` (v1), which is the
lineage the user asked for and the one whose two-sided velocity tracking
already means "go the speed I said", not "go as fast as you can".

Reward budget
-------------
``walk_v3``'s 100M-step run failed with a 14:1 ratio between the largest
achievable penalty and the largest achievable reward, so the optimal policy
was to fight its own scaffold (docs/27).  Here every penalty is normalised
into ``[0, 1]`` before weighting and :class:`RewardConfig` asserts

    sum(penalty weights) <= sum(reward weights)

which bounds that ratio at 1:1 by construction and makes the assertion fail
loudly if a future edit breaks it.

Examples
--------
Inspect the scaffold and reward for a specific fault, without training::

    python -m src.rl.walk_failsafe check --failed-legs 5 --command 0.2 0 0

Train, sampling zero, one or two failures per episode::

    python -m src.rl.walk_failsafe train --curriculum easy --timesteps 5000000

Replay, with the right rear leg missing::

    mjpython -m src.rl.walk_failsafe enjoy runs/scone_walk_failsafe/best_model.zip \\
        --failed-legs 5 --command 0.2 0 0
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, Sequence

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecMonitor

from src.hardware.actuator import Actuator
from src.locomotion import GaitConfig, TripodGait
from src.locomotion.fault_gait import (
    FaultAdaptiveGait,
    GaitSchedule,
    HEALTHY_GAIT_CHOICES,
    LEG_INDICES,
    plan_schedule,
)
from src.locomotion.support_polygon import UNSUPPORTED_MARGIN, stability_margin
from src.rl.motion_profile import motion_profile_for_standing_pose
from src.rl.policy_compat import load_compatible_policy
from src.rl.stance import SPORT_STANDING_DEGREES, validate_standing_pose
from src.simulation.core.controller import MuJoCoController
from src.simulation.core.model import (
    FAILURE_MODES,
    LegFailureRuntime,
    load_model,
    validate_failed_legs,
)
from src.simulation.core.pid import spec_for_motor_id
from src.simulation.core.viewer import configure_simulation_viewer
from src.simulation.terrain import TERRAIN_CHOICES, TerrainType


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_PATH = PROJECT_ROOT / "assets" / "model.xml"

# Right side warm, left side cool, matching the MJCF inspection colours.
LEG_LABELS = {
    1: ("R front", "빨강"), 3: ("R mid", "주황"), 5: ("R rear", "노랑"),
    2: ("L front", "초록"), 4: ("L mid", "파랑"), 6: ("L rear", "남색"),
}

REFERENCE_CHOICES = ("fault-adaptive", "tripod-gait", "none")
# Run records written by the other trainers name references this one does not
# implement. They all mean "scaffold the hand-authored gait", which here is the
# fault-adaptive schedule, so they map onto it rather than aborting a replay.
REFERENCE_ALIASES = {
    "hardcoded": "fault-adaptive",
    "non_rl": "tripod-gait",
    "scone-gait": "fault-adaptive",
}


def normalize_reference_motion(value: str) -> str:
    return REFERENCE_ALIASES.get(value, value)

FAILURE_ONSET_CHOICES = ("reset",)

# Set from the measured scaffold, not from ambition. With one leg lost the
# wave scaffold holds a positive support margin up to a 60 degree stride,
# which delivers 0.088 m/s; the full curriculum asks for 0.14 so the residual
# has somewhere to go, and no more. walk_v3's mistake of commanding several
# times what the scaffold can produce (docs/23 section 2.2) is what this
# avoids.
CURRICULUM_RANGES: dict[str, np.ndarray] = {
    "easy": np.array([0.06, 0.00, 0.00], dtype=np.float64),
    "medium": np.array([0.10, 0.04, 0.20], dtype=np.float64),
    "full": np.array([0.14, 0.06, 0.30], dtype=np.float64),
}
OBSERVATION_COMMAND_SCALE = np.array([0.14, 0.06, 0.30], dtype=np.float64)

# How many legs fail in an episode. Zero is kept in the mix so one checkpoint
# still walks an undamaged robot; two is kept rare because several two-leg
# sets have no statically stable wave schedule at all (fault_gait docstring).
FAILURE_COUNT_WEIGHTS = (0.30, 0.50, 0.20)

OBSERVATION_SHAPE = (85,)


@dataclass(frozen=True)
class RewardConfig:
    """One thing to earn, everything else to lose.

    The shape of this objective is the lesson from the two runs before it.

    ``walk_v2`` paid the policy for posture it could hold while ignoring the
    command, and it learned to do exactly that.  So there is only **one**
    positive term here: how well the measured motion matches the commanded
    motion.  Standing still scores it 1.0 under an idle command and ~0 under
    a moving one, which is the whole of "accuracy" in a single expression and
    leaves nothing to harvest.

    ``walk_v3`` let the reachable penalty reach 57.5/s against a reachable
    reward of 4.0/s, and its optimal policy became to suppress its own
    scaffold; 100M steps ended worse than doing nothing.  So every penalty
    here is divided by an explicit cap and clipped into ``[0, 1]``, which
    makes each weight equal to that term's worst-case cost per second, and
    :meth:`__post_init__` refuses a configuration whose penalties out-budget
    the reward.

    Stability and accuracy are therefore expressed as *deficits*: holding the
    heading, the attitude, the ride height and a positive support margin costs
    nothing, and losing them costs score in proportion.
    """

    # -- the only positive term --------------------------------------------
    # Two-sided on purpose: this trainer is not asked to go fast, so overspeed
    # is as much a tracking error as underspeed.
    linear_velocity_sigma: float = 0.10        # m/s
    yaw_velocity_sigma: float = 0.15           # rad/s
    tracking_weight: float = 3.00

    # -- accuracy deficits --------------------------------------------------
    heading_error_cap: float = 0.60            # rad of error scoring full cost
    heading_weight: float = 0.50

    # -- attitude, height, support -----------------------------------------
    # 0.26 of projected gravity is 15 degrees of tilt; the measured wave-gait
    # scaffold stays under 5 degrees, so full cost is three times worse than
    # anything the scaffold does on its own.
    tilt_cap: float = 0.26
    tilt_weight: float = 0.50
    height_cap: float = 0.060                  # m of deviation, either way
    height_weight: float = 0.25
    # Full credit at this margin and full cost at its negation. The wave
    # schedule alone reaches +42..+54 mm with one leg lost, so the target is
    # inside what the scaffold already achieves: the policy is scored on
    # holding it while moving, not on discovering it.
    support_margin_target: float = 0.040       # m
    support_margin_weight: float = 0.60

    # -- smoothness, effort, participation ---------------------------------
    oscillation_cap: float = 4.0
    oscillation_weight: float = 0.20
    slip_cap: float = 4.0
    slip_weight: float = 0.20
    joint_limit_cap: float = 1.0
    joint_limit_weight: float = 0.20
    action_rate_cap: float = 0.25
    action_rate_weight: float = 0.15
    action_magnitude_cap: float = 0.50
    action_magnitude_weight: float = 0.15
    current_cap: float = 1.0
    current_weight: float = 0.10
    # A foot that has not touched down for this long while the robot is being
    # asked to move is not participating. The wave schedule keeps every
    # working leg's swing well inside one cycle, so this only fires when the
    # policy parks a leg.
    inactivity_seconds: float = 1.50
    inactivity_cap: float = 3.0
    inactivity_weight: float = 0.15

    # -- limits and termination --------------------------------------------
    soft_joint_offset: float = math.radians(60.0)
    hard_joint_offset: float = math.radians(90.0)
    slip_deadzone: float = 0.02                # m/s at the contact point
    idle_activity_threshold: float = 0.05
    # Ending an episode costs the worst survival that was still available,
    # times this. Above 1.0 no reachable stream of penalties is ever worse
    # than surviving it, so falling over can never become the optimal move --
    # which a fixed constant cannot guarantee on a 12 s episode.
    termination_severity: float = 1.10

    def __post_init__(self) -> None:
        if min(
            self.linear_velocity_sigma,
            self.yaw_velocity_sigma,
            self.support_margin_target,
            self.idle_activity_threshold,
        ) <= 0.0:
            raise ValueError("every reward tolerance must be positive")
        if min(
            self.heading_error_cap,
            self.tilt_cap,
            self.height_cap,
            self.oscillation_cap,
            self.slip_cap,
            self.joint_limit_cap,
            self.action_rate_cap,
            self.action_magnitude_cap,
            self.current_cap,
            self.inactivity_cap,
        ) <= 0.0:
            raise ValueError("every penalty cap must be positive")
        if self.termination_severity < 1.0:
            raise ValueError("termination_severity below 1.0 rewards falling over")
        if self.reward_budget <= 0.0:
            raise ValueError("reward budget must be positive")
        if self.penalty_budget > self.reward_budget + 1e-9:
            raise ValueError(
                f"penalty budget {self.penalty_budget:.2f}/s exceeds reward "
                f"budget {self.reward_budget:.2f}/s; the optimal policy would "
                "be to stop moving (this is the walk_v3 failure)"
            )

    @property
    def reward_budget(self) -> float:
        """Largest reward per second a perfect policy can collect."""

        return self.tracking_weight

    @property
    def penalty_budget(self) -> float:
        """Largest penalty per second, excluding the terminal charge."""

        return (
            self.heading_weight
            + self.tilt_weight
            + self.height_weight
            + self.support_margin_weight
            + self.oscillation_weight
            + self.slip_weight
            + self.joint_limit_weight
            + self.action_rate_weight
            + self.action_magnitude_weight
            + self.current_weight
            + self.inactivity_weight
        )


@dataclass
class WalkConfig:
    physics_timestep: float = 0.002            # 500 Hz MuJoCo
    frame_skip: int = 10                       # 50 Hz policy
    episode_seconds: float = 12.0
    command_filter_seconds: float = 0.35
    command_hold_seconds_min: float = 2.5
    command_hold_seconds_max: float = 5.0
    idle_command_probability: float = 0.20
    settle_seconds: float = 0.30
    max_height_drop: float = 0.10
    max_tilt_degrees: float = 40.0
    contact_force_threshold: float = 1.0

    # -- scaffold ----------------------------------------------------------
    # Joint space, not Cartesian. The Cartesian tripod reaches 0.055 m/s at
    # this command through IK and stroke clipping; the joint-space scaffold
    # measured below reaches 0.180 m/s intact and 0.088 m/s on five legs, at a
    # positive support margin throughout. Both were measured on this model.
    cadence_hz: float = 1.20
    # Clearance grows with stride or the swinging leg drags, and never falls
    # below the stride itself so an idle command lifts nothing (v3's rule,
    # re-measured here).
    lift_degrees_per_stride: float = 0.47
    lift_degrees_floor: float = 20.0
    lift_degrees_max: float = 40.0
    # Beyond a 60 degree stride the measured support margin goes negative for
    # every leg count, which is the property this trainer exists to protect.
    stride_degrees_max: float = 60.0
    yaw_stride_degrees_max: float = 45.0
    joint_target_bound_degrees: float = 60.0
    # (achieved m/s, stride degrees), measured per leg count at cadence 1.2 on
    # the flat terrain with the body shift applied. Monotone in both columns.
    stride_calibration: dict[int, tuple[tuple[float, float], ...]] = field(
        default_factory=lambda: {
            6: ((0.000, 0.0), (0.008, 10.0), (0.047, 20.0), (0.073, 30.0),
                (0.101, 40.0), (0.140, 50.0), (0.180, 60.0)),
            5: ((0.000, 0.0), (0.017, 20.0), (0.030, 30.0), (0.042, 40.0),
                (0.063, 50.0), (0.088, 60.0)),
            4: ((0.000, 0.0), (0.003, 10.0), (0.029, 20.0), (0.044, 30.0),
                (0.061, 40.0), (0.088, 50.0), (0.114, 60.0)),
        }
    )
    # (achieved rad/s, differential stride degrees). Above 45 degrees the
    # measured turn rate stops being monotone -- it flips sign -- which is why
    # yaw_stride_degrees_max stops there.
    yaw_stride_calibration: dict[int, tuple[tuple[float, float], ...]] = field(
        default_factory=lambda: {
            6: ((0.000, 0.0), (0.041, 10.0), (0.232, 20.0), (0.355, 30.0)),
            5: ((0.000, 0.0), (0.013, 10.0), (0.108, 20.0), (0.170, 30.0),
                (0.285, 45.0)),
            4: ((0.000, 0.0), (0.013, 10.0), (0.108, 20.0), (0.170, 30.0),
                (0.285, 45.0)),
        }
    )
    healthy_gait: str = "tripod"
    # The static body shift that recovers the support margin costs a little
    # speed; an ablation can turn it off without touching the scheduler.
    apply_body_shift: bool = True

    # -- faults ------------------------------------------------------------
    failure_mode: str = "detached"
    max_failed_legs: int = 2
    failure_onset: str = "reset"
    # A fault the policy is not told about is a different research problem
    # (fault detection). Keep the flag so an ablation can turn the mask off
    # without editing the observation layout.
    observe_leg_health: bool = True

    def __post_init__(self) -> None:
        if not 0.0 <= self.idle_command_probability <= 1.0:
            raise ValueError("idle_command_probability must be between 0 and 1")
        if self.failure_mode not in FAILURE_MODES + ("mixed",):
            raise ValueError(
                f"failure_mode must be one of {FAILURE_MODES + ('mixed',)}"
            )
        if not 0 <= self.max_failed_legs <= 2:
            raise ValueError("max_failed_legs must be 0, 1 or 2")
        if self.failure_onset not in FAILURE_ONSET_CHOICES:
            raise ValueError(f"failure_onset must be one of {FAILURE_ONSET_CHOICES}")
        if self.healthy_gait not in HEALTHY_GAIT_CHOICES:
            raise ValueError(f"healthy_gait must be one of {HEALTHY_GAIT_CHOICES}")
        if self.max_tilt_degrees <= 0.0 or self.max_height_drop <= 0.0:
            raise ValueError("termination thresholds must be positive")


def _normalized(cost: float, cap: float) -> float:
    """Map a raw penalty into ``[0, 1]`` so its weight is also its budget."""

    return float(np.clip(cost / cap, 0.0, 1.0))


class SconeFailsafeEnv(gym.Env[np.ndarray, np.ndarray]):
    """Command-conditioned residual RL on a fault-adaptive scaffold."""

    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(
        self,
        model_path: Path | str = DEFAULT_MODEL_PATH,
        *,
        curriculum: str = "easy",
        fixed_command: np.ndarray | Sequence[float] | None = None,
        fixed_failed_legs: Sequence[int] | None = None,
        render_mode: str | None = None,
        reward_config: RewardConfig | None = None,
        walk_config: WalkConfig | None = None,
        terrain: TerrainType | str = TerrainType.FLAT,
        terrain_seed: int = 7,
        standing_pose_degrees: Sequence[float] | None = None,
        reference_motion: str = "fault-adaptive",
    ) -> None:
        super().__init__()
        if curriculum not in CURRICULUM_RANGES:
            raise ValueError(
                f"Unknown curriculum {curriculum!r}; choose from "
                f"{tuple(CURRICULUM_RANGES)}."
            )
        if render_mode not in (None, "human"):
            raise ValueError("render_mode must be None or 'human'")
        if reference_motion not in REFERENCE_CHOICES:
            raise ValueError(
                f"Unknown reference motion {reference_motion!r}; choose from "
                f"{REFERENCE_CHOICES}."
            )

        self.model_path = Path(model_path).expanduser().resolve()
        self.curriculum = curriculum
        self.command_range = CURRICULUM_RANGES[curriculum].copy()
        self.reward_config = reward_config or RewardConfig()
        self.walk_config = walk_config or WalkConfig()
        self.reference_motion = reference_motion
        self.render_mode = render_mode
        self.terrain = TerrainType.parse(terrain)
        self.terrain_seed = terrain_seed

        self.fixed_command = (
            None
            if fixed_command is None
            else np.asarray(fixed_command, dtype=np.float64)
        )
        if self.fixed_command is not None and self.fixed_command.shape != (3,):
            raise ValueError("fixed_command must contain [vx, vy, yaw_rate]")
        self.fixed_failed_legs = (
            None
            if fixed_failed_legs is None
            else validate_failed_legs(fixed_failed_legs)
        )

        self.model = load_model(
            self.model_path,
            terrain=self.terrain,
            terrain_seed=terrain_seed,
        )
        self.model.opt.timestep = self.walk_config.physics_timestep
        self.data = mujoco.MjData(self.model)
        self.controller: MuJoCoController
        # One compiled model, mutated in place: an episode picks a new fault
        # every reset and recompiling six meshes each time is far too slow.
        self.failure = LegFailureRuntime(self.model)

        self.control_dt = self.model.opt.timestep * self.walk_config.frame_skip
        self.max_episode_steps = round(
            self.walk_config.episode_seconds / self.control_dt
        )

        self.root_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "root_freejoint"
        )
        if self.root_joint_id < 0:
            raise ValueError("fail-safe learning requires model.xml root_freejoint")
        self.root_body_id = int(self.model.jnt_bodyid[self.root_joint_id])
        self.root_qpos_address = int(self.model.jnt_qposadr[self.root_joint_id])

        self.floor_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "simulation_floor"
        )
        if self.floor_geom_id < 0:
            raise ValueError("fail-safe learning requires simulation_floor")
        self.ground_geom_ids = {self.floor_geom_id}
        for geom_id in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            if name and name.startswith("terrain_"):
                self.ground_geom_ids.add(geom_id)

        self.tire_geom_ids: set[int] = set()
        self.tire_geom_to_body: dict[int, int] = {}
        self.tire_geom_to_leg: dict[int, int] = {}
        for leg in LEG_INDICES:
            geom_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, f"TIRE_{leg}_geom"
            )
            if geom_id < 0:
                raise ValueError(f"Missing contact geom TIRE_{leg}_geom")
            self.tire_geom_ids.add(geom_id)
            self.tire_geom_to_body[geom_id] = int(self.model.geom_bodyid[geom_id])
            self.tire_geom_to_leg[geom_id] = leg

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
        # Stage-2 gets the most authority because it sets where on the open arc
        # the robot stands, which is the lever a five-legged stance needs most.
        self.residual_scale_degrees = np.array(
            [10.0] * 6 + [12.0] * 6 + [15.0] * 6, dtype=np.float64
        )
        self._motion_profile = motion_profile_for_standing_pose(self.default_degrees)

        # The gait object is used for scheduling and for the one static IK
        # solve that realises the body shift. Its per-frame Cartesian path is
        # not used: the scaffold below writes joint targets directly.
        gait_config = GaitConfig(
            control_frequency=1.0 / self.control_dt,
            cycle_frequency=self.walk_config.cadence_hz,
            ik_tolerance=1e-3,
            ik_stride_backoff_attempts=4,
            max_vx=float(OBSERVATION_COMMAND_SCALE[0]),
            max_vy=float(OBSERVATION_COMMAND_SCALE[1]),
            max_yaw_rate=float(OBSERVATION_COMMAND_SCALE[2]),
            command_time_constant=0.0,
        )
        self._reference_gait: FaultAdaptiveGait | TripodGait | None
        if self.reference_motion == "fault-adaptive":
            self._reference_gait = FaultAdaptiveGait(
                profile=self._motion_profile,
                model_path=self.model_path,
                config=gait_config,
                healthy_gait=self.walk_config.healthy_gait,
            )
        elif self.reference_motion == "tripod-gait":
            self._reference_gait = TripodGait(
                profile=self._motion_profile,
                model_path=self.model_path,
                config=gait_config,
            )
        else:
            self._reference_gait = None
        if self._reference_gait is not None:
            self._reference_gait.reset(motor_degrees=self.default_degrees)
            self._nominal_feet = np.asarray(
                self._reference_gait._nominal_feet, dtype=np.float64
            )
        else:
            self._nominal_feet = np.zeros((6, 3), dtype=np.float64)

        self.action_space = spaces.Box(-1.0, 1.0, shape=(18,), dtype=np.float32)
        # walk_learn's 70 values, then six leg-health flags, six foot-contact
        # flags, the signed support margin, and the ground-plane offset from
        # the support centroid to the centre of mass, in body axes.
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=OBSERVATION_SHAPE, dtype=np.float32
        )

        self._phase = 0.0
        self._episode_step = 0
        self._next_command_step = 0
        self._command = np.zeros(3, dtype=np.float64)
        self._command_target = np.zeros(3, dtype=np.float64)
        self._target_heading = 0.0
        self._last_action = np.zeros(18, dtype=np.float64)
        self._reference_height = 0.0
        self._reference_cadence = 0.0
        self._reference_stride_degrees = 0.0
        self._stance_offset_degrees = np.zeros(18, dtype=np.float64)
        self._leg_health = np.ones(6, dtype=np.float64)
        self._failed_legs: tuple[int, ...] = ()
        self._failure_mode = (
            "detached"
            if self.walk_config.failure_mode == "mixed"
            else self.walk_config.failure_mode
        )
        self._schedule: GaitSchedule | None = None
        self._contact_seconds_since = np.zeros(6, dtype=np.float64)
        self._cached_contacts: tuple[np.ndarray, float, np.ndarray] = (
            np.zeros(6, dtype=np.float64),
            UNSUPPORTED_MARGIN,
            np.zeros(2, dtype=np.float64),
        )
        self._viewer: Any | None = None

        self._jacobian_position = np.zeros((3, self.model.nv), dtype=np.float64)
        self._jacobian_rotation = np.zeros((3, self.model.nv), dtype=np.float64)
        self._contact_force = np.zeros(6, dtype=np.float64)
        self._body_velocity = np.zeros(6, dtype=np.float64)

    # ------------------------------------------------------------------
    # faults
    # ------------------------------------------------------------------

    @property
    def failed_legs(self) -> tuple[int, ...]:
        return self._failed_legs

    @property
    def healthy_legs(self) -> tuple[int, ...]:
        return tuple(leg for leg in LEG_INDICES if leg not in self._failed_legs)

    @property
    def schedule(self) -> GaitSchedule | None:
        return self._schedule

    def _sample_failure(self) -> tuple[tuple[int, ...], str]:
        if self.fixed_failed_legs is not None:
            legs = self.fixed_failed_legs
        else:
            limit = self.walk_config.max_failed_legs
            weights = np.array(FAILURE_COUNT_WEIGHTS[: limit + 1], dtype=np.float64)
            weights /= weights.sum()
            count = int(self.np_random.choice(len(weights), p=weights))
            legs = tuple(
                sorted(
                    int(leg)
                    for leg in self.np_random.choice(
                        np.array(LEG_INDICES), size=count, replace=False
                    )
                )
            )
        if self.walk_config.failure_mode == "mixed":
            mode = "detached" if self.np_random.random() < 0.5 else "limp"
        else:
            mode = self.walk_config.failure_mode
        return legs, mode

    def set_failed_legs(
        self,
        legs: Iterable[int],
        mode: str | None = None,
    ) -> tuple[int, ...]:
        """Fail these legs from the next reset, and re-plan the scaffold now."""

        self._failed_legs = self.failure.apply(
            legs, self._failure_mode if mode is None else mode
        )
        if mode is not None:
            self._failure_mode = mode
        self._leg_health = np.array(
            [0.0 if leg in self._failed_legs else 1.0 for leg in LEG_INDICES],
            dtype=np.float64,
        )
        mujoco.mj_setConst(self.model, self.data)
        self._replan_schedule()
        return self._failed_legs

    def _body_frame_center_of_mass(self) -> np.ndarray:
        """Ground-plane centre of mass in body axes, for the gait scheduler."""

        mujoco.mj_forward(self.model, self.data)
        world_from_body = self.data.xmat[self.root_body_id].reshape(3, 3)
        offset = self.data.subtree_com[self.root_body_id] - self.data.xpos[
            self.root_body_id
        ]
        return np.asarray(world_from_body.T @ offset, dtype=np.float64)[:2]

    def _replan_schedule(self) -> None:
        healthy = self.healthy_legs
        if len(healthy) < 3:
            raise ValueError("at least three working legs are required to walk")
        center = self._body_frame_center_of_mass()
        if isinstance(self._reference_gait, FaultAdaptiveGait):
            self._schedule = self._reference_gait.set_healthy_legs(
                healthy, center_of_mass_xy=center
            )
            self._stance_offset_degrees = (
                self._reference_gait.stance_offset_degrees()
                if self.walk_config.apply_body_shift
                else np.zeros(18, dtype=np.float64)
            )
            return
        # Without a fault-adaptive scaffold the schedule is still reported, so
        # an ablation can be read against the same diagnostics.
        self._schedule = plan_schedule(
            healthy,
            self._nominal_feet,
            center,
            healthy_gait=self.walk_config.healthy_gait,
        )
        self._stance_offset_degrees = np.zeros(18, dtype=np.float64)

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------

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

    def _heading_error(self) -> float:
        error = self._heading_yaw() - self._target_heading
        return float(math.atan2(math.sin(error), math.cos(error)))

    def _contact_normal_force(self, contact_index: int) -> float:
        self._contact_force.fill(0.0)
        mujoco.mj_contactForce(
            self.model, self.data, contact_index, self._contact_force
        )
        return abs(float(self._contact_force[0]))

    def _refresh_contacts(self) -> tuple[np.ndarray, float, np.ndarray]:
        """Solve contacts and support once per physics frame, and cache it.

        ``step`` needs them for the reward and ``_observation`` needs the same
        numbers immediately afterwards. Solving twice doubled the cost of the
        most expensive part of a step for no new information.
        """

        flags, points = self._foot_contacts()
        margin, offset = self._support_state(points)
        self._cached_contacts = (flags, margin, offset)
        return self._cached_contacts

    def _foot_contacts(self) -> tuple[np.ndarray, dict[int, np.ndarray]]:
        """Per-leg contact flags and one world contact point per touching leg."""

        flags = np.zeros(6, dtype=np.float64)
        points: dict[int, np.ndarray] = {}
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 in self.ground_geom_ids and geom2 in self.tire_geom_ids:
                tire_geom = geom2
            elif geom2 in self.ground_geom_ids and geom1 in self.tire_geom_ids:
                tire_geom = geom1
            else:
                continue
            if (
                self._contact_normal_force(contact_index)
                < self.walk_config.contact_force_threshold
            ):
                continue
            leg = self.tire_geom_to_leg[tire_geom]
            flags[leg - 1] = 1.0
            position = np.asarray(contact.pos, dtype=np.float64)
            if leg in points:
                # Two patches on one open arc: the mid-point is the effective
                # support, and using it keeps the polygon from being widened
                # by whichever vertex the solver reported first.
                points[leg] = 0.5 * (points[leg] + position)
            else:
                points[leg] = position
        return flags, points

    def _support_state(
        self,
        contact_points: dict[int, np.ndarray],
    ) -> tuple[float, np.ndarray]:
        """Signed support margin, and the CoM-to-centroid offset in body axes.

        The margin is a statement about gravity, so it is measured in the world
        ground plane rather than in the tilted body frame.  The offset is then
        rotated into body axes so the policy sees the same number whatever
        direction the robot is facing.
        """

        world_from_body = self.data.xmat[self.root_body_id].reshape(3, 3)
        center = np.asarray(self.data.subtree_com[self.root_body_id])[:2]
        if not contact_points:
            return UNSUPPORTED_MARGIN, np.zeros(2, dtype=np.float64)
        contacts = np.array(
            [point[:2] for point in contact_points.values()], dtype=np.float64
        )
        margin = stability_margin(center, contacts)
        offset_world = center - contacts.mean(axis=0)
        offset_body = world_from_body[:2, :2].T @ offset_world
        return float(margin), np.asarray(offset_body, dtype=np.float64)

    def _observation(self) -> np.ndarray:
        linear_velocity, angular_velocity, gravity = self._base_state()
        joint_position, joint_velocity = self._joint_state()
        heading_error = self._heading_error()
        contact_flags, margin, offset = self._cached_contacts
        health = (
            self._leg_health
            if self.walk_config.observe_leg_health
            else np.ones(6, dtype=np.float64)
        )
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
                health,
                contact_flags,
                np.array([margin / self.reward_config.support_margin_target]),
                offset / 0.10,
            ]
        )
        return observation.astype(np.float32)

    # ------------------------------------------------------------------
    # command and action
    # ------------------------------------------------------------------

    def set_velocity_command(self, command: Sequence[float]) -> np.ndarray:
        parsed = np.asarray(command, dtype=np.float64)
        if parsed.shape != (3,):
            raise ValueError("velocity command must contain [vx, vy, yaw_rate]")
        if not np.all(np.isfinite(parsed)):
            raise ValueError("velocity command must contain only finite values")
        clipped = np.clip(parsed, -OBSERVATION_COMMAND_SCALE, OBSERVATION_COMMAND_SCALE)
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
            command[0] = self.np_random.uniform(-0.05, self.command_range[0])
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
            OBSERVATION_COMMAND_SCALE > 0.0, OBSERVATION_COMMAND_SCALE, 1.0
        )
        return float(np.clip(np.max(np.abs(self._command / safe_scale)), 0.0, 1.0))

    def action_mask(self) -> np.ndarray:
        """Zero for the joints of failed legs, one elsewhere.

        A detached leg has nothing to move and an unpowered one cannot be
        moved, so a residual written there is at best wasted authority.
        Masking it also keeps the action-magnitude penalty honest: the policy
        is not charged for output it was never allowed to use.
        """

        mask = np.ones(18, dtype=np.float64)
        for leg in self._failed_legs:
            mask[[leg - 1, leg + 5, leg + 11]] = 0.0
        return mask

    @staticmethod
    def _interpolate(table: tuple[tuple[float, float], ...], value: float) -> float:
        """Invert a measured (achieved, command) table at ``value``.

        Beyond the last measured point the final segment's slope continues, so
        a command past the scaffold's range still asks for more rather than
        silently saturating -- the residual is what has to supply the rest.
        """

        achieved = [row[0] for row in table]
        degrees = [row[1] for row in table]
        if value <= achieved[0]:
            return degrees[0]
        if value >= achieved[-1]:
            span = achieved[-1] - achieved[-2]
            slope = (degrees[-1] - degrees[-2]) / span if span > 0 else 0.0
            return degrees[-1] + slope * (value - achieved[-1])
        return float(np.interp(value, achieved, degrees))

    def stride_for_speed(self, speed: float) -> float:
        table = self.walk_config.stride_calibration[self._calibration_key()]
        return min(
            self.walk_config.stride_degrees_max,
            self._interpolate(table, abs(speed)),
        )

    def stride_for_yaw_rate(self, yaw_rate: float) -> float:
        table = self.walk_config.yaw_stride_calibration[self._calibration_key()]
        return min(
            self.walk_config.yaw_stride_degrees_max,
            self._interpolate(table, abs(yaw_rate)),
        )

    def _calibration_key(self) -> int:
        # Four is the fewest legs that can bound an area at all, so anything
        # below it reads the four-leg table and is handled by termination.
        return int(np.clip(len(self.healthy_legs), 4, 6))

    @staticmethod
    def _quintic(value: float) -> float:
        return value**3 * (10.0 + value * (-15.0 + 6.0 * value))

    @staticmethod
    def _swing_lift(value: float) -> float:
        return 16.0 * value**2 * (1.0 - value) ** 2

    def _reference_degrees(self) -> np.ndarray:
        """One frame of the fault-adaptive wave scaffold, in joint space.

        Each working leg gets an explicit stance window and swing window from
        the schedule, so the duty factor the scheduler chose is the duty factor
        the robot executes.  A sinusoidal hip sweep -- what v1 and v3 both use
        -- cannot do that: its stance and swing are always half the cycle each,
        whatever offsets the legs are given, and on five legs it measured a
        support margin of -0.208 m against this scaffold's +0.007 m.

        The price is speed, which is the trade this trainer exists to make.
        """

        reference = self.default_degrees + self._stance_offset_degrees
        if self._reference_gait is None or self._schedule is None:
            self._reference_cadence = 0.0
            self._reference_stride_degrees = 0.0
            return reference

        config = self.walk_config
        schedule = self._schedule
        forward = self.stride_for_speed(self._command[0])
        forward = math.copysign(forward, self._command[0]) if self._command[0] else 0.0
        # Measured sign: a positive differential stride turns the body the
        # negative way, so the command is inverted here rather than in the
        # table.
        turn = self.stride_for_yaw_rate(self._command[2])
        turn = -math.copysign(turn, self._command[2]) if self._command[2] else 0.0
        lateral = 0.0  # the scaffold has no lateral term; vy is the residual's

        per_leg = {
            leg: float(
                np.clip(
                    forward + turn * (1.0 if leg % 2 else -1.0),
                    -config.stride_degrees_max - config.yaw_stride_degrees_max,
                    config.stride_degrees_max + config.yaw_stride_degrees_max,
                )
            )
            for leg in schedule.healthy_legs
        }
        magnitude = max((abs(value) for value in per_leg.values()), default=0.0)
        self._reference_stride_degrees = magnitude
        self._reference_cadence = config.cadence_hz
        self._phase = (self._phase + config.cadence_hz * self.control_dt) % 1.0

        lift = min(
            config.lift_degrees_max,
            max(
                config.lift_degrees_per_stride * magnitude,
                min(config.lift_degrees_floor, magnitude),
            ),
        )
        duty = schedule.duty_factor
        for leg, amplitude in per_leg.items():
            leg_phase = (self._phase + schedule.phase_offsets[leg]) % 1.0
            if leg_phase < duty:
                # Stance: sweep the hip back across the stroke at constant rate.
                reference[leg - 1] += amplitude * (0.5 - leg_phase / duty)
            else:
                swing = (leg_phase - duty) / (1.0 - duty)
                reference[leg - 1] += amplitude * (self._quintic(swing) - 0.5)
                reference[leg + 5] -= lift * self._swing_lift(swing)
        del lateral
        return reference

    def _apply_action(self, action: np.ndarray) -> np.ndarray:
        """Write joint targets, and return the action actually used."""

        mask = self.action_mask()
        clipped = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0) * mask
        reference = self._reference_degrees()
        # Failed legs are parked at the standing pose rather than driven along
        # a scaffold they cannot follow.
        reference = np.where(mask > 0.0, reference, self.default_degrees)
        targets = reference + self.residual_scale_degrees * clipped
        bound = self.walk_config.joint_target_bound_degrees
        bounded = np.clip(
            targets, self.default_degrees - bound, self.default_degrees + bound
        )
        for motor_id, target in enumerate(bounded, start=1):
            self.controller.set_position(motor_id, float(target))
        return clipped

    # ------------------------------------------------------------------
    # penalties
    # ------------------------------------------------------------------

    def _slip_penalty(self) -> tuple[float, int]:
        values: list[float] = []
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if geom1 in self.ground_geom_ids and geom2 in self.tire_geom_ids:
                tire_geom = geom2
            elif geom2 in self.ground_geom_ids and geom1 in self.tire_geom_ids:
                tire_geom = geom1
            else:
                continue
            if (
                self._contact_normal_force(contact_index)
                < self.walk_config.contact_force_threshold
            ):
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
            excess = max(
                0.0,
                float(np.linalg.norm(tangential)) - self.reward_config.slip_deadzone,
            )
            values.append((excess / 0.20) ** 2)
        if not values:
            return 0.0, 0
        return float(np.mean(values)), len(values)

    def _forbidden_floor_collision(self) -> bool:
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
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
            if (
                self._contact_normal_force(contact_index)
                >= self.walk_config.contact_force_threshold
            ):
                return True
        return False

    def _normalized_current_penalty(self) -> float:
        """Mean squared normalised current, over the working motors only."""

        values = []
        for motor_id in Actuator.Index.ALL:
            leg = ((motor_id - 1) % 6) + 1
            if leg in self._failed_legs:
                continue
            actuator_id = int(self.controller._actuator_ids[motor_id])
            voltage = float(self.data.ctrl[actuator_id])
            velocity = self.controller._joint_velocity(motor_id)
            spec = spec_for_motor_id(motor_id)
            current = (voltage - spec.K * velocity) / spec.R
            values.append(current / (spec.stall_torque / spec.K))
        if not values:
            return 0.0
        return float(np.mean(np.square(values)))

    # ------------------------------------------------------------------
    # reward
    # ------------------------------------------------------------------

    def _reward(
        self,
        action: np.ndarray,
        contact_flags: np.ndarray,
        margin: float,
    ) -> tuple[float, dict[str, float], bool, dict[str, Any]]:
        config = self.reward_config
        dt = self.control_dt
        linear, angular, gravity = self._base_state()
        position, _ = self._joint_state()

        # -- the one thing to earn ------------------------------------------
        linear_error = linear[:2] - self._command[:2]
        yaw_error = float(angular[2] - self._command[2])
        tracking = math.exp(
            -float(linear_error @ linear_error) / config.linear_velocity_sigma**2
            - yaw_error**2 / config.yaw_velocity_sigma**2
        )

        # -- accuracy and stability deficits --------------------------------
        heading_error = self._heading_error()
        heading_cost = _normalized(abs(heading_error), config.heading_error_cap)
        tilt = float(np.linalg.norm(gravity[:2]))
        tilt_cost = _normalized(tilt, config.tilt_cap)
        height = float(self.data.qpos[self.root_qpos_address + 2])
        height_cost = _normalized(
            abs(height - self._reference_height), config.height_cap
        )
        target = config.support_margin_target
        support_cost = _normalized(target - margin, 2.0 * target)

        oscillation = (
            (linear[2] / 0.30) ** 2
            + (angular[0] / 0.80) ** 2
            + (angular[1] / 0.80) ** 2
        )
        oscillation_cost = _normalized(oscillation, config.oscillation_cap)
        slip, stance_contacts = self._slip_penalty()
        slip_cost = _normalized(slip, config.slip_cap)

        joint_offset = np.abs(position - self.default_radians)
        excess = np.maximum(0.0, joint_offset - config.soft_joint_offset)
        joint_limit_cost = _normalized(
            float(np.mean(np.square(excess / math.radians(15.0)))),
            config.joint_limit_cap,
        )
        action_rate_cost = _normalized(
            float(np.mean(np.square(action - self._last_action))),
            config.action_rate_cap,
        )
        action_magnitude_cost = _normalized(
            float(np.mean(np.square(action))), config.action_magnitude_cap
        )
        current_cost = _normalized(
            self._normalized_current_penalty(), config.current_cap
        )

        down = contact_flags > 0.0
        self._contact_seconds_since[down] = 0.0
        self._contact_seconds_since[~down] += dt
        # Only working legs can be idle, and only while the robot is moving.
        idle_seconds = np.clip(
            self._contact_seconds_since - config.inactivity_seconds, 0.0, None
        )
        moving = float(
            self._command_activity() > config.idle_activity_threshold
        )
        inactivity_cost = _normalized(
            float(np.sum(idle_seconds * self._leg_health)) * moving,
            config.inactivity_cap,
        )

        terms = {
            "tracking": config.tracking_weight * tracking * dt,
            "heading": -config.heading_weight * heading_cost * dt,
            "tilt": -config.tilt_weight * tilt_cost * dt,
            "height": -config.height_weight * height_cost * dt,
            "support": -config.support_margin_weight * support_cost * dt,
            "oscillation": -config.oscillation_weight * oscillation_cost * dt,
            "slip": -config.slip_weight * slip_cost * dt,
            "joint_limit": -config.joint_limit_weight * joint_limit_cost * dt,
            "action_rate": -config.action_rate_weight * action_rate_cost * dt,
            "action_magnitude": (
                -config.action_magnitude_weight * action_magnitude_cost * dt
            ),
            "current": -config.current_weight * current_cost * dt,
            "inactivity": -config.inactivity_weight * inactivity_cost * dt,
        }

        finite = bool(
            np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()
        )
        collision = self._forbidden_floor_collision()
        fallen = (
            -float(gravity[2])
            < math.cos(math.radians(self.walk_config.max_tilt_degrees))
            or height < self._reference_height - self.walk_config.max_height_drop
        )
        hard_joint_limit = bool(np.any(joint_offset > config.hard_joint_offset))
        terminated = (not finite) or fallen or collision or hard_joint_limit

        total = float(sum(terms.values()))
        if terminated:
            # Charge the worst survival that was still on the table, so ending
            # the episode can never beat living through it.
            remaining = max(0, self.max_episode_steps - self._episode_step) * dt
            penalty = config.termination_severity * config.penalty_budget * remaining
            terms["termination"] = -penalty
            total -= penalty

        diagnostics: dict[str, Any] = {
            "vx": float(linear[0]),
            "vy": float(linear[1]),
            "yaw_rate": float(angular[2]),
            "tracking": tracking,
            "heading_error": heading_error,
            "tilt_degrees": math.degrees(math.asin(min(1.0, tilt))),
            "height": height,
            "support_margin": margin,
            "stance_contacts": stance_contacts,
            "failed_legs": self._failed_legs,
            "failure_mode": self._failure_mode,
            "gait_pattern": "" if self._schedule is None else self._schedule.pattern,
            "duty_factor": (
                0.0 if self._schedule is None else self._schedule.duty_factor
            ),
            "scaffold_margin": (
                0.0 if self._schedule is None else self._schedule.worst_margin
            ),
            "reference_cadence": self._reference_cadence,
            "reference_stride_degrees": self._reference_stride_degrees,
            "command_activity": self._command_activity(),
            "forbidden_collision": collision,
            "fallen": fallen,
            "hard_joint_limit": hard_joint_limit,
        }
        return total, terms, terminated, diagnostics

    # ------------------------------------------------------------------
    # gym API
    # ------------------------------------------------------------------

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        del options
        legs, mode = self._sample_failure()
        self._failure_mode = mode
        self.failure.apply(legs, mode)
        self._failed_legs = tuple(legs)
        self._leg_health = np.array(
            [0.0 if leg in self._failed_legs else 1.0 for leg in LEG_INDICES],
            dtype=np.float64,
        )

        mujoco.mj_resetData(self.model, self.data)
        # Masses changed, so the derived constants MuJoCo caches must follow.
        mujoco.mj_setConst(self.model, self.data)
        self.controller = MuJoCoController(
            self.model,
            self.data,
            verbose=False,
            standing_pose_degrees=self.default_degrees,
        )
        self.controller.enable_torque()
        self._replan_schedule()

        self._phase = float(self.np_random.random())
        if self._reference_gait is not None:
            self._reference_gait.reset(
                phase=self._phase, motor_degrees=self.default_degrees
            )
        self._episode_step = 0
        self._last_action.fill(0.0)
        self._contact_seconds_since.fill(0.0)
        self._reference_cadence = 0.0
        self._reference_stride_degrees = 0.0
        self._command.fill(0.0)
        self._command_target = self._sample_command()
        self._target_heading = self._heading_yaw()
        self._schedule_next_command()
        if self.fixed_command is not None:
            self._command[:] = self.fixed_command

        settle_steps = round(
            self.walk_config.settle_seconds / self.model.opt.timestep
        )
        for _ in range(settle_steps):
            self.controller.update(self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)
        # A zero-settle configuration would otherwise observe the state left
        # by mj_resetData, in which no contact or velocity has been computed.
        mujoco.mj_forward(self.model, self.data)

        self._reference_height = float(self.data.qpos[self.root_qpos_address + 2])
        self._target_heading = self._heading_yaw()
        self._refresh_contacts()
        return self._observation(), {
            "command": self._command.copy(),
            "failed_legs": self._failed_legs,
            "failure_mode": self._failure_mode,
        }

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        self._update_command()
        self._target_heading += self._command[2] * self.control_dt
        used_action = self._apply_action(action)

        for _ in range(self.walk_config.frame_skip):
            self.controller.update(self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)

        contact_flags, margin, _offset = self._refresh_contacts()
        reward, terms, terminated, diagnostics = self._reward(
            used_action, contact_flags, margin
        )
        self._last_action[:] = used_action
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


# ----------------------------------------------------------------------
# training
# ----------------------------------------------------------------------

# Fixed fault cases the promotion gate is measured on. The healthy robot and
# every single-leg loss, plus the two-leg loss that the scheduler reports as
# its hardest statically stable case.
EVALUATION_FAULTS: tuple[tuple[int, ...], ...] = (
    (), (1,), (2,), (3,), (4,), (5,), (6,), (2, 5),
)
EVALUATION_COMMANDS: tuple[tuple[float, float, float], ...] = (
    (0.06, 0.0, 0.0), (0.12, 0.0, 0.0), (0.0, 0.0, 0.25),
)


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str))
    temporary.replace(path)


def _write_resume_pointer(run_dir: Path, checkpoint: Path) -> None:
    (run_dir / "resume.checkpoint").write_text(checkpoint.name)


class RewardTermsCallback(BaseCallback):
    """Write reward components and measured state to the PPO logger."""

    def _on_step(self) -> bool:
        infos = self.locals.get("infos") or []
        terms: dict[str, list[float]] = {}
        state: dict[str, list[float]] = {}
        for info in infos:
            for name, value in (info.get("reward_terms") or {}).items():
                terms.setdefault(name, []).append(float(value))
            for name in (
                "vx", "vy", "yaw_rate", "tracking", "heading_error",
                "tilt_degrees", "support_margin", "stance_contacts",
                "duty_factor", "scaffold_margin", "reference_stride_degrees",
            ):
                if name in info:
                    state.setdefault(name, []).append(float(info[name]))
            failed = info.get("failed_legs")
            if failed is not None:
                state.setdefault("failed_leg_count", []).append(float(len(failed)))
        for name, values in terms.items():
            self.logger.record(f"reward/{name}", float(np.mean(values)))
        for name, values in state.items():
            self.logger.record(f"state/{name}", float(np.mean(values)))
        return True


class GracefulStopCallback(BaseCallback):
    """Stop after the current rollout when asked, saving a resume checkpoint."""

    def __init__(self, run_dir: Path, should_stop: Callable[[], bool]) -> None:
        super().__init__()
        self.run_dir = run_dir
        self.should_stop = should_stop
        (run_dir / "graceful_stop.enabled").write_text("1")

    def _on_step(self) -> bool:
        if not self.should_stop():
            return True
        checkpoint = self.run_dir / "checkpoints" / (
            f"scone_walk_failsafe_{self.num_timesteps}_steps.zip"
        )
        self.model.save(checkpoint)
        _write_resume_pointer(self.run_dir, checkpoint)
        print(f"[RL] graceful stop at {self.num_timesteps} steps", flush=True)
        return False


class PruningCheckpointCallback(CheckpointCallback):
    """Keep only the most recent ``keep`` checkpoints."""

    def __init__(self, *args: Any, keep: int, run_dir: Path, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.keep = keep
        self.run_dir = run_dir

    def _on_step(self) -> bool:
        result = super()._on_step()
        if self.n_calls % self.save_freq == 0:
            folder = Path(self.save_path)
            saved = sorted(
                folder.glob(f"{self.name_prefix}_*_steps.zip"),
                key=lambda path: int(path.stem.split("_")[-2]),
            )
            for stale in saved[: max(0, len(saved) - self.keep)]:
                stale.unlink(missing_ok=True)
            if saved:
                _write_resume_pointer(self.run_dir, saved[-1])
        return result


def _episode_score(records: list[dict[str, float]]) -> dict[str, float]:
    """Summarise one evaluation episode into comparable scalars."""

    if not records:
        return {
            "score": 0.0, "tracking": 0.0, "margin": UNSUPPORTED_MARGIN,
            "heading_error": math.pi, "tilt_degrees": 90.0, "survived": 0.0,
        }
    tracking = float(np.mean([row["tracking"] for row in records]))
    margin = float(np.mean([row["support_margin"] for row in records]))
    heading = float(np.mean([abs(row["heading_error"]) for row in records]))
    tilt = float(np.max([row["tilt_degrees"] for row in records]))
    survived = float(records[-1]["survived"])
    # One number for the promotion gate, in the trainer's own currency:
    # accuracy first, then the support margin it is supposed to protect.
    score = (
        tracking
        + 0.5 * float(np.clip(margin / 0.040, -1.0, 1.0))
        - 0.5 * heading / math.pi
    ) * survived
    return {
        "score": score, "tracking": tracking, "margin": margin,
        "heading_error": heading, "tilt_degrees": tilt, "survived": survived,
    }


def evaluate_policy_over_faults(
    make_env: Callable[[tuple[int, ...], tuple[float, float, float]], SconeFailsafeEnv],
    act: Callable[[np.ndarray], np.ndarray],
    *,
    seconds: float = 8.0,
    faults: Sequence[tuple[int, ...]] = EVALUATION_FAULTS,
    commands: Sequence[tuple[float, float, float]] = EVALUATION_COMMANDS,
) -> dict[str, Any]:
    """Score a controller across every fault case and command, deterministically."""

    per_case: dict[str, dict[str, float]] = {}
    for fault in faults:
        for command in commands:
            env = make_env(tuple(fault), command)
            observation, _ = env.reset(seed=0)
            records: list[dict[str, float]] = []
            steps = int(seconds / env.control_dt)
            survived = 1.0
            for _ in range(steps):
                observation, _reward, terminated, truncated, info = env.step(
                    act(observation)
                )
                records.append({
                    "tracking": float(info["tracking"]),
                    "support_margin": float(info["support_margin"]),
                    "heading_error": float(info["heading_error"]),
                    "tilt_degrees": float(info["tilt_degrees"]),
                    "survived": survived,
                })
                if terminated:
                    survived = 0.0
                    records[-1]["survived"] = 0.0
                    break
                if truncated:
                    break
            env.close()
            key = f"legs{'-'.join(map(str, fault)) or 'none'}|{command[0]:g},{command[1]:g},{command[2]:g}"
            per_case[key] = _episode_score(records)
    scores = [case["score"] for case in per_case.values()]
    return {
        "score": float(np.mean(scores)),
        "worst_score": float(np.min(scores)),
        "survival_rate": float(
            np.mean([case["survived"] for case in per_case.values()])
        ),
        "cases": per_case,
    }


class FaultEvalCallback(BaseCallback):
    """Score the policy on every fault case, and promote only real gains.

    ``walk_v3`` ran for 17 hours after the answer was already in: its gate
    reported ``beats_zero_residual = 0`` from the first minute and nothing
    acted on it (docs/27).  Here the same measurement stops the run.
    """

    def __init__(
        self,
        run_dir: Path,
        make_env: Callable[
            [tuple[int, ...], tuple[float, float, float]], SconeFailsafeEnv
        ],
        *,
        every: int,
        seconds: float,
        patience: int,
    ) -> None:
        super().__init__()
        self.run_dir = run_dir
        self.make_env = make_env
        self.every = every
        self.seconds = seconds
        self.patience = patience
        self.baseline: dict[str, Any] | None = None
        self.best: float | None = None
        self.failures_since_promotion = 0
        self._next = every

    def _on_training_start(self) -> None:
        zero = np.zeros(18, dtype=np.float32)
        self.baseline = evaluate_policy_over_faults(
            self.make_env, lambda _observation: zero, seconds=self.seconds
        )
        _write_json_atomic(self.run_dir / "scaffold_baseline.json", self.baseline)
        print(
            "[RL] zero-residual scaffold: score "
            f"{self.baseline['score']:.4f}, worst {self.baseline['worst_score']:.4f}, "
            f"survival {self.baseline['survival_rate']:.2f}",
            flush=True,
        )

    def _on_step(self) -> bool:
        if self.num_timesteps < self._next:
            return True
        self._next = self.num_timesteps + self.every

        def act(observation: np.ndarray) -> np.ndarray:
            action, _state = self.model.predict(observation, deterministic=True)
            return np.asarray(action, dtype=np.float32)

        result = evaluate_policy_over_faults(
            self.make_env, act, seconds=self.seconds
        )
        assert self.baseline is not None
        beats = bool(
            result["score"] > self.baseline["score"]
            and result["worst_score"] >= self.baseline["worst_score"]
        )
        self.logger.record("eval/score", result["score"])
        self.logger.record("eval/worst_score", result["worst_score"])
        self.logger.record("eval/survival_rate", result["survival_rate"])
        self.logger.record("eval/scaffold_score", self.baseline["score"])
        self.logger.record("eval/beats_scaffold", float(beats))

        with (self.run_dir / "evaluation_history.jsonl").open("a") as stream:
            stream.write(json.dumps({
                "num_timesteps": self.num_timesteps,
                "beats_scaffold": beats,
                "summary": {
                    key: result[key]
                    for key in ("score", "worst_score", "survival_rate")
                },
            }, default=str) + "\n")

        improved = beats and (self.best is None or result["score"] > self.best)
        if improved:
            self.best = result["score"]
            self.model.save(self.run_dir / "best_model.zip")
            _write_json_atomic(self.run_dir / "best_model_metrics.json", result)
            self.failures_since_promotion = 0
            print(
                f"[RL] promoted at {self.num_timesteps}: score "
                f"{result['score']:.4f} > scaffold {self.baseline['score']:.4f}",
                flush=True,
            )
            return True

        self.failures_since_promotion += 1
        if self.patience and self.failures_since_promotion >= self.patience:
            print(
                f"[RL] stopping at {self.num_timesteps}: {self.patience} "
                "consecutive evaluations failed to beat the scaffold. Running "
                "longer will not fix an objective the policy cannot climb.",
                flush=True,
            )
            return False
        return True


def _env_factory(
    args: Any,
    index: int,
    curriculum: str,
) -> Callable[[], SconeFailsafeEnv]:
    def build() -> SconeFailsafeEnv:
        environment = SconeFailsafeEnv(
            curriculum=curriculum,
            terrain=args.terrain,
            terrain_seed=args.terrain_seed + index,
            standing_pose_degrees=args.standing_pose_degrees,
            reference_motion=args.reference_motion,
            walk_config=_walk_config_from(args),
        )
        environment.reset(seed=args.seed + index)
        return environment

    return build


def _walk_config_from(args: Any) -> WalkConfig:
    return WalkConfig(
        failure_mode=args.failure_mode,
        max_failed_legs=args.max_failed_legs,
        apply_body_shift=not args.no_body_shift,
        healthy_gait=args.healthy_gait,
    )


def _evaluation_env_factory(
    args: Any,
) -> Callable[[tuple[int, ...], tuple[float, float, float]], SconeFailsafeEnv]:
    def build(
        failed: tuple[int, ...],
        command: tuple[float, float, float],
    ) -> SconeFailsafeEnv:
        # Evaluation is always the nominal failure mode: a promotion decision
        # must not depend on which coin the sampler flipped.
        config = _walk_config_from(args)
        config.failure_mode = (
            "detached" if config.failure_mode == "mixed" else config.failure_mode
        )
        return SconeFailsafeEnv(
            curriculum="full",
            fixed_command=list(command),
            fixed_failed_legs=list(failed),
            terrain=args.terrain,
            terrain_seed=args.terrain_seed,
            standing_pose_degrees=args.standing_pose_degrees,
            reference_motion=args.reference_motion,
            walk_config=config,
        )

    return build


def _ppo_kwargs(args: Any) -> dict[str, Any]:
    """PPO settings, with the two guards walk_v3's run needed and lacked.

    That run spent 57% of its updates with ``approx_kl = inf``: a zero entropy
    coefficient let the gSDE standard deviation collapse, the squashed
    policy's ``log(1 - tanh^2)`` correction diverged at saturation, and PPO
    then early-stopped every update at step 0 for 66M steps (docs/27).  A
    small entropy floor keeps the distribution alive, and the KL target is
    left at a value that stops a real divergence rather than a numeric one.
    """

    if args.entropy_coefficient <= 0.0:
        raise ValueError(
            "entropy_coefficient must be positive: a zero coefficient is what "
            "collapsed walk_v3's policy distribution"
        )
    return {
        "learning_rate": args.learning_rate,
        "n_steps": args.n_steps,
        "batch_size": args.batch_size,
        "n_epochs": args.n_epochs,
        "gamma": 0.99,
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
        },
    }


def run_check(args: Any) -> int:
    """Report the scaffold and the reward for one fault case, no training."""

    config = _walk_config_from(args)
    if config.failure_mode == "mixed":
        config.failure_mode = "detached"
    env = SconeFailsafeEnv(
        curriculum=args.curriculum,
        fixed_command=args.command,
        fixed_failed_legs=args.failed_legs,
        terrain=args.terrain,
        terrain_seed=args.terrain_seed,
        standing_pose_degrees=args.standing_pose_degrees,
        reference_motion=args.reference_motion,
        walk_config=config,
    )
    check_env(env, warn=True, skip_render_check=True)
    observation, info = env.reset(seed=args.seed)
    schedule = env.schedule
    assert schedule is not None

    failed = ", ".join(
        f"{leg} ({LEG_LABELS[leg][0]}/{LEG_LABELS[leg][1]})"
        for leg in env.failed_legs
    ) or "없음"
    print(f"  결손 다리   {failed}  [{env._failure_mode}]")
    print(
        f"  스케줄      {schedule.pattern}  duty={schedule.duty_factor:.3f}  "
        f"순서={schedule.swing_order}"
    )
    print(
        f"  몸체 시프트 ({schedule.body_shift_xy[0]:+.3f}, "
        f"{schedule.body_shift_xy[1]:+.3f}) m  →  정적 여유 "
        f"{schedule.worst_margin:+.4f} m "
        f"({'안정' if schedule.statically_stable else '불안정'})"
    )
    print(f"  관측        {observation.shape}  행동 마스크 {env.action_mask().astype(int).tolist()}")
    print(
        f"  보상 예산   +{env.reward_config.reward_budget:.2f}/s  "
        f"-{env.reward_config.penalty_budget:.2f}/s"
    )

    action = (
        np.zeros(18, dtype=np.float32)
        if args.zero_residual
        else env.action_space.sample()
    )
    totals: dict[str, float] = {}
    margins: list[float] = []
    tilts: list[float] = []
    contacts = np.zeros(6)
    start = env.data.qpos[env.root_qpos_address : env.root_qpos_address + 3].copy()
    rotation = env.data.xmat[env.root_body_id].reshape(3, 3).copy()
    total = 0.0
    steps = 0
    for _ in range(args.steps):
        if not args.zero_residual:
            action = env.action_space.sample()
        observation, reward, terminated, truncated, info = env.step(action)
        total += reward
        steps += 1
        for name, value in info["reward_terms"].items():
            totals[name] = totals.get(name, 0.0) + value
        margins.append(info["support_margin"])
        tilts.append(info["tilt_degrees"])
        flags, _margin, _offset = env._cached_contacts
        contacts += flags
        if terminated or truncated:
            print(f"  종료        {steps} 스텝 ({'terminated' if terminated else 'truncated'})")
            break

    displacement = rotation.T @ (
        env.data.qpos[env.root_qpos_address : env.root_qpos_address + 3] - start
    )
    seconds = steps * env.control_dt
    print(
        f"\n  명령 {np.round(env._command, 3).tolist()}  →  실제 "
        f"vx {displacement[0] / seconds:+.4f}  vy {displacement[1] / seconds:+.4f} m/s"
    )
    print(
        f"  지지 여유   평균 {np.mean(margins):+.4f}  최소 {np.min(margins):+.4f} m"
        f"   |  최대 기울기 {np.max(tilts):.2f}°"
    )
    print("  접촉 듀티   " + "  ".join(
        f"L{leg}{'✗' if leg in env.failed_legs else ''} {contacts[leg - 1] / steps:.2f}"
        for leg in LEG_INDICES
    ))
    print(f"\n  보상 {total:+.3f} ({total / seconds:+.4f}/s)")
    width = 34
    peak = max((abs(value) for value in totals.values()), default=1.0) or 1.0
    for name, value in sorted(totals.items(), key=lambda item: -abs(item[1])):
        bar = "█" * max(1, round(width * abs(value) / peak))
        print(f"    {name:<18}{value / seconds:+8.4f}/s  {bar}")
    env.close()
    return 0


def run_train(args: Any) -> int:
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

    for name in ("SIGINT", "SIGTERM"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), request_stop)

    callbacks: list[BaseCallback] = [
        PruningCheckpointCallback(
            save_freq=max(1, args.checkpoint_every // args.num_envs),
            save_path=str(run_dir / "checkpoints"),
            name_prefix="scone_walk_failsafe",
            keep=args.keep_checkpoints,
            run_dir=run_dir,
        ),
        RewardTermsCallback(),
        FaultEvalCallback(
            run_dir,
            _evaluation_env_factory(args),
            every=args.eval_every,
            seconds=args.eval_seconds,
            patience=args.eval_patience,
        ),
        GracefulStopCallback(run_dir, lambda: stop_requested),
    ]
    tensorboard_log = (
        None if args.tensorboard_log is None
        else str(Path(args.tensorboard_log).expanduser().resolve())
    )
    if args.resume is None:
        model = PPO(
            "MlpPolicy",
            env,
            **_ppo_kwargs(args),
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
        reset_timesteps = False

    model.learn(
        total_timesteps=args.timesteps,
        callback=callbacks,
        reset_num_timesteps=reset_timesteps,
        progress_bar=False,
    )
    final = run_dir / "final_model.zip"
    model.save(final)
    _write_resume_pointer(run_dir, final)
    print(f"[RL] saved {final}", flush=True)
    env.close()
    return 0


def run_enjoy(args: Any) -> int:
    config = _walk_config_from(args)
    if config.failure_mode == "mixed":
        config.failure_mode = "detached"
    env = SconeFailsafeEnv(
        curriculum="full",
        fixed_command=args.command,
        fixed_failed_legs=args.failed_legs,
        render_mode=None if args.headless else "human",
        terrain=args.terrain,
        terrain_seed=args.terrain_seed,
        standing_pose_degrees=args.standing_pose_degrees,
        reference_motion=args.reference_motion,
        walk_config=config,
    )
    policy = load_compatible_policy(args.checkpoint, env, args.device)
    observation, _info = env.reset(seed=args.seed)
    schedule = env.schedule
    assert schedule is not None
    print(
        f"[RL] {schedule.pattern} duty={schedule.duty_factor:.3f} "
        f"failed={env.failed_legs or '없음'} margin={schedule.worst_margin:+.4f} m"
    )
    frame = env.control_dt
    episodes = max(0, int(getattr(args, "episodes", 0) or 0))
    steps = int(args.seconds / frame)
    played = 0
    step = 0
    while (episodes and played < episodes) or (not episodes and step < steps):
        began = time.perf_counter()
        action, _state = policy.predict(observation, deterministic=True)
        observation, _reward, terminated, truncated, _info = env.step(action)
        step += 1
        if terminated or truncated:
            played += 1
            observation, _info = env.reset()
        if not args.headless:
            time.sleep(max(0.0, frame - (time.perf_counter() - began)))
    env.close()
    return 0


def _add_failure_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--failure-mode",
        choices=FAILURE_MODES + ("mixed",),
        default="detached",
        help=(
            "detached: the leg is gone, with its mass, collision and drawing. "
            "limp: the motors lose power and gearbox friction holds the joints, "
            "so the leg stays attached and drags. mixed: sample per episode."
        ),
    )
    parser.add_argument(
        "--max-failed-legs", type=int, default=2, choices=(0, 1, 2),
        help="Upper bound on legs lost per episode; 0, 1 and 2 are mixed.",
    )
    parser.add_argument(
        "--healthy-gait", choices=HEALTHY_GAIT_CHOICES, default="tripod",
        help="What the undamaged robot runs; faults always switch to a wave.",
    )
    parser.add_argument(
        "--no-body-shift", action="store_true",
        help="Ablation: keep the nominal stance instead of shifting the body "
             "back inside the support polygon after a fault.",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m src.rl.walk_failsafe",
        description=(
            "Fail-safe residual RL: hold a commanded motion, an attitude and a "
            "support margin with one or two legs out of action."
        ),
    )
    parser.add_argument("--terrain", choices=TERRAIN_CHOICES, default="flat")
    parser.add_argument("--terrain-seed", type=int, default=7)
    parser.add_argument(
        "--reference-motion", choices=REFERENCE_CHOICES, default="fault-adaptive"
    )
    parser.add_argument(
        "--standing-pose-degrees", type=float, nargs=18, default=None,
        help="18 motor degrees; defaults to the Standard stance.",
    )
    parser.add_argument("--stance", choices=("standard", "sport"), default="standard")
    # "command_name", not "command": check and enjoy both define a --command
    # velocity, and a subparser dest of the same name is silently overwritten
    # by it. The other three trainers already use this spelling.
    subparsers = parser.add_subparsers(dest="command_name", required=True)

    check = subparsers.add_parser("check", help="Report the scaffold and reward")
    check.add_argument("--curriculum", choices=tuple(CURRICULUM_RANGES), default="full")
    check.add_argument("--command", type=float, nargs=3, default=[0.06, 0.0, 0.0])
    check.add_argument("--failed-legs", type=int, nargs="*", default=[])
    check.add_argument("--steps", type=int, default=400)
    check.add_argument("--seed", type=int, default=0)
    check.add_argument(
        "--random-residual", dest="zero_residual", action="store_false",
        help="Drive random residuals instead of measuring the bare scaffold.",
    )
    check.set_defaults(zero_residual=True)
    _add_failure_arguments(check)
    check.set_defaults(handler=run_check)

    train = subparsers.add_parser("train", help="Train a fail-safe policy")
    train.add_argument("--curriculum", choices=tuple(CURRICULUM_RANGES), default="easy")
    train.add_argument("--timesteps", type=int, default=20_000_000)
    train.add_argument("--num-envs", type=int, default=8)
    train.add_argument("--n-steps", type=int, default=2048)
    train.add_argument("--batch-size", type=int, default=4096)
    train.add_argument("--n-epochs", type=int, default=4)
    train.add_argument("--learning-rate", type=float, default=2e-4)
    train.add_argument("--target-kl", type=float, default=0.03)
    train.add_argument(
        "--entropy-coefficient", type=float, default=0.003,
        help="Must stay positive; zero is what collapsed walk_v3's policy.",
    )
    train.add_argument("--max-grad-norm", type=float, default=0.5)
    train.add_argument("--sde-sample-freq", type=int, default=4)
    train.add_argument("--log-std-init", type=float, default=-2.0)
    train.add_argument("--checkpoint-every", type=int, default=100_000)
    train.add_argument("--keep-checkpoints", type=int, default=10)
    train.add_argument("--eval-every", type=int, default=200_000)
    train.add_argument("--eval-seconds", type=float, default=6.0)
    train.add_argument(
        "--eval-patience", type=int, default=25,
        help="Stop after this many consecutive evaluations that fail to beat "
             "the zero-residual scaffold; 0 disables the gate.",
    )
    train.add_argument("--seed", type=int, default=0)
    train.add_argument("--device", default="auto")
    train.add_argument(
        "--output", type=Path, default=Path("runs/scone_walk_failsafe")
    )
    train.add_argument("--tensorboard-log", type=Path, default=None)
    train.add_argument("--resume", type=Path, default=None)
    _add_failure_arguments(train)
    train.set_defaults(handler=run_train)

    enjoy = subparsers.add_parser("enjoy", help="Replay a saved policy")
    enjoy.add_argument("checkpoint", type=Path)
    enjoy.add_argument("--command", type=float, nargs=3, default=[0.06, 0.0, 0.0])
    enjoy.add_argument("--failed-legs", type=int, nargs="*", default=[])
    enjoy.add_argument("--seconds", type=float, default=30.0)
    enjoy.add_argument(
        "--episodes", type=int, default=0,
        help="Replay this many episodes instead of a fixed wall-clock span; "
             "the launcher's replay menu uses this form.",
    )
    enjoy.add_argument("--seed", type=int, default=0)
    enjoy.add_argument("--device", default="auto")
    enjoy.add_argument("--headless", action="store_true")
    _add_failure_arguments(enjoy)
    enjoy.set_defaults(handler=run_enjoy)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.standing_pose_degrees is None:
        from src.rl.stance import STANCE_PRESETS

        args.standing_pose_degrees = list(STANCE_PRESETS[args.stance])
    if getattr(args, "failed_legs", None):
        args.failed_legs = list(validate_failed_legs(args.failed_legs))
    return int(args.handler(args))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
