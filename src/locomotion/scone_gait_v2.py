"""``scone-gait-v2`` -- a role-split gait that steers and rolls the sectors.

``SconeGait`` asks all six legs to walk and to sweep their sector at the same
time, and measures 0.0889 m/s against ``TripodGait``'s 0.1125 m/s on the same
stride budget.  The rotation is not helping; it is scrubbing.
``docs/28-scone-gait-v2-role-split-rolling.md`` records why.

This controller starts from the measured geometry instead.  Each sector is a
225-degree arc centred on its own stage-2 axis, so it is a wheel with a gap:
rotating it inside the arc approximately preserves its support point, and
the six wheels roll radially outward from the chassis.  The upper joints are
exact vertical-axis steering columns.

From that, one frame of control is:

* work out the ground travel each contact needs from the commanded twist;
* ask each wheel how well it could roll that travel after steering, and give
  the legs that can the ``ROLL`` role and the legs that cannot the ``STEP``
  role -- for a forward command the kinematics hand back the four corner legs
  and the middle pair respectively;
* split each rolling leg's travel into a rolling part and an articulated part,
  and send only the articulated part to IK, so rolling buys stride budget
  rather than competing with it;
* add the sector angle to the IK solution rather than blending it in, which
  the contact-invariance measurement licenses and which ``SconeGait`` does not
  do on its default path;
* let a rolling leg skip the swings it does not need, and spend its 200-plus
  degrees of arc before re-indexing during its own tripod's swing window.

At least three legs are scheduled in stance. Actual loaded support is a
physical outcome and is not guaranteed by the phase schedule.

This is a simulation-first controller.  The sector angle stays inside the
measured arc *and* inside the 0..360 degree actuator range, so unlike
``roll-gait`` it needs no velocity mode and no multi-turn targets, but it has
not been validated against measured TPU contact behaviour on hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np
from numpy.typing import ArrayLike, NDArray

from src.hardware import ControllerProtocol
from src.kinematics import IKResult, RobotKinematics
from src.kinematics.leg import DEFAULT_MODEL_PATH

from .profile import MotionProfile, SPORT, get_profile
from .sector_wheel import RollSolution, SectorWheelModel
from .tripod_gait import (
    GaitConfig,
    GaitSample,
    TripodGait,
    VelocityCommand,
)


Vector2 = NDArray[np.float64]
Vector3 = NDArray[np.float64]


class LegRole(str, Enum):
    """What one leg is doing for the current command."""

    ROLL = "roll"
    STEP = "step"


@dataclass(frozen=True)
class SconeGaitV2Config(GaitConfig):
    """Tuning for the role-split steer-and-roll gait.

    The stride budget matches ``TRIPOD_GAIT_SIMULATION_CONFIG`` so that any
    speed difference against the walking baseline is the rolling, not a wider
    stroke.  ``max_vx`` is deliberately far above the walking limit: rolling
    exists to lift that ceiling, and clamping the command to the old one would
    hide whether it did.
    """

    cycle_frequency: float = 2.0
    duty_factor: float = 0.60
    step_height: float = 0.025
    max_stride: float = 0.090
    max_lateral_stride: float | None = 0.070
    max_vx: float = 0.45
    max_vy: float = 0.25
    max_yaw_rate: float = 0.9
    ik_tolerance: float = 1e-3
    ik_stride_backoff_attempts: int = 4

    # Share of the required contact travel that rolling is asked to carry.
    # A steered leg's rolling direction is radial to its own hip, so whatever
    # rolling does not take has to be articulated straight out of or into the
    # leg -- the least forgiving direction in the workspace.  Holding back a
    # share does not buy a graceful fallback; it buys an IK failure.  The
    # residual left at 1.0 is the misalignment component, which is lateral.
    roll_authority: float = 1.0
    # Upper-joint steering limit.  45 degrees would align a corner leg
    # perfectly with a forward command, but it also folds the four corner feet
    # into a 0.19 m wide footprint.  35 degrees keeps 0.32 m of width and
    # still reaches 0.985 alignment.
    max_steering_degrees: float = 35.0
    min_roll_alignment: float = 0.70
    minimum_rolling_legs: int = 2
    # Restrict the ROLL role to these legs.  ``None`` lets the kinematics
    # decide, which is the default and what the four-corner forward gait uses.
    # Naming a pair turns the gait into a two-leg drive: those legs roll and
    # the other four walk, whatever the command.
    roll_legs: tuple[int, ...] | None = None
    # Kept away from both ends of the measured arc and from the 0..360 degree
    # actuator range.
    arc_margin_degrees: float = 20.0
    motor_headroom_degrees: float = 15.0
    # The arc is not symmetric about the stance pose: from nominal there are
    # about 240 degrees of tread one way and 4 the other.  A leg that has to
    # roll the short way pre-indexes during its swing, so it reserves only the
    # arc the command actually needs instead of the whole window.
    min_arc_reserve_degrees: float = 35.0
    arc_reserve_safety: float = 1.8
    # Roll authority fades to zero over the last of the arc so the articulated
    # stroke takes over before the sector runs out of tread.  It has to be
    # small against the reserve: a derate window as wide as the reserve fades
    # rolling out over the whole of every stance, and it does so only for the
    # legs whose arc runs the short way, which turns into a heading bias.
    roll_derate_degrees: float = 12.0
    max_roll_rate_degrees: float = 540.0
    # A rolling leg re-indexes at its next swing window once it has spent this
    # fraction of its arc.
    reindex_threshold: float = 0.50
    # Swing clearance is scaled by how fast the ground is moving under the
    # robot, not by ``TripodGait``'s command activity.  Activity is the command
    # divided by ``max_vx``, so raising the speed ceiling would otherwise
    # shrink the step height of every slower command -- and a re-indexing
    # sector, which turns 40 degrees while unloaded, needs its clearance
    # whatever fraction of the ceiling was asked for.
    swing_lift_reference_speed: float = 0.05
    # IK is warm-started from the previous frame, and a steered leg has two
    # solutions for the same contact.  Once a leg falls into the folded one
    # the warm start keeps it there for the rest of the run -- measured at
    # 86 degrees of stage-2 away from the stance branch on leg 1, with the
    # robot riding up on that leg and stalling.  A solved stage-2 further
    # than this from the stance pose is re-solved from the stance pose.
    ik_branch_guard_degrees: float = 40.0
    arc_lift_tolerance: float = 0.002
    arc_scan_step: float = 2.0

    # Explicit experimental restrictions. Defaults preserve the pre-audit
    # controller; the paper adapter opts into the rewind-feasible allocation.
    subtract_rolling_travel: bool = True
    rolling_swing_policy: str = "adaptive"
    limit_reindex_rate: bool = False
    anticipate_reindex_budget: bool = False

    def __post_init__(self) -> None:
        super().__post_init__()
        if not 0.0 <= self.roll_authority <= 1.0:
            raise ValueError("roll_authority must be between 0 and 1")
        if not 0.0 <= self.max_steering_degrees <= 90.0:
            raise ValueError("max_steering_degrees must be in [0, 90]")
        if not 0.0 <= self.min_roll_alignment <= 1.0:
            raise ValueError("min_roll_alignment must be between 0 and 1")
        if not 0 <= self.minimum_rolling_legs <= 6:
            raise ValueError("minimum_rolling_legs must be between 0 and 6")
        if self.roll_legs is not None:
            legs = tuple(self.roll_legs)
            if len(set(legs)) != len(legs) or not set(legs) <= set(range(1, 7)):
                raise ValueError("roll_legs must be distinct leg numbers 1..6")
        if self.arc_margin_degrees < 0.0 or self.motor_headroom_degrees < 0.0:
            raise ValueError("arc margin and motor headroom cannot be negative")
        if self.roll_derate_degrees <= 0.0:
            raise ValueError("roll_derate_degrees must be positive")
        if self.max_roll_rate_degrees <= 0.0:
            raise ValueError("max_roll_rate_degrees must be positive")
        if not 0.0 < self.reindex_threshold <= 1.0:
            raise ValueError("reindex_threshold must be in (0, 1]")
        if self.min_arc_reserve_degrees <= 0.0:
            raise ValueError("min_arc_reserve_degrees must be positive")
        if self.arc_reserve_safety < 1.0:
            raise ValueError("arc_reserve_safety must be at least 1")
        if self.swing_lift_reference_speed <= 0.0:
            raise ValueError("swing_lift_reference_speed must be positive")
        if self.ik_branch_guard_degrees <= 0.0:
            raise ValueError("ik_branch_guard_degrees must be positive")
        if self.rolling_swing_policy not in ("adaptive", "periodic", "disabled"):
            raise ValueError("rolling_swing_policy must be adaptive, periodic or disabled")


class SconeGaitV2(TripodGait):
    """Alternating-tripod scheduler with steered, rolling stance legs."""

    config: SconeGaitV2Config

    def __init__(
        self,
        controller: ControllerProtocol | None = None,
        profile: str | MotionProfile = SPORT,
        *,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        config: SconeGaitV2Config | None = None,
        end_effector_points: dict[int, ArrayLike] | None = None,
    ) -> None:
        selected = config or SconeGaitV2Config()
        resolved = get_profile(profile) if isinstance(profile, str) else profile
        stance = self._stance_motor_degrees(resolved, model_path, selected)
        if end_effector_points is None:
            end_effector_points = self._infer_support_points(
                RobotKinematics(model_path),
                np.radians(stance - 180.0),
            )
        super().__init__(
            controller,
            resolved,
            model_path=model_path,
            config=selected,
            end_effector_points=end_effector_points,
        )
        self._wheels: SectorWheelModel | None = None
        super().reset(motor_degrees=stance)
        self._build_wheel_model()
        self._reset_role_state()

    # -- setup -----------------------------------------------------------

    @staticmethod
    def _sector_window_for(
        arc_min: float,
        arc_max: float,
        nominal_lower: float,
        headroom: float,
    ) -> tuple[float, float]:
        """Intersect one leg's tread with the 0..360 degree actuator range."""

        return (
            max(arc_min, -nominal_lower + headroom),
            min(arc_max, 360.0 - nominal_lower - headroom),
        )

    @classmethod
    def _stance_motor_degrees(
        cls,
        profile: MotionProfile,
        model_path: str | Path,
        config: SconeGaitV2Config,
    ) -> NDArray[np.float64]:
        """Stand with each sector at the centre of the arc it can still use.

        The profile pose parks every sector against the trailing end of its
        own tread: 240 degrees of arc one way, four the other.  A leg that has
        to roll the short way would then have to pre-index most of the arc
        during a single swing, and the two directions would behave nothing
        alike -- which is exactly the front/rear asymmetry that turned into a
        heading bias.  Centring the stance pose inside the tread, and inside
        the actuator range, makes both directions symmetric and cheap.

        Rotating the sector does not change the contact height, so this moves
        the tread under the robot without changing how it stands.
        """

        nominal = cls._profile_motor_degrees(profile)
        probe = SectorWheelModel(
            RobotKinematics(model_path),
            nominal,
            arc_lift_tolerance=config.arc_lift_tolerance,
            arc_scan_step=config.arc_scan_step,
        )
        stance = nominal.copy()
        for leg in range(1, 7):
            geometry = probe.geometry[leg]
            low, high = cls._sector_window_for(
                geometry.arc_min_degrees,
                geometry.arc_max_degrees,
                float(nominal[leg + 11]),
                config.motor_headroom_degrees,
            )
            stance[leg + 11] += 0.5 * (low + high)
        return stance

    def _build_wheel_model(self) -> None:
        self._wheels = SectorWheelModel(
            self.kinematics,
            self._nominal_motor_degrees,
            arc_lift_tolerance=self.config.arc_lift_tolerance,
            arc_scan_step=self.config.arc_scan_step,
        )
        self._sector_window = np.zeros((6, 2), dtype=np.float64)
        for leg in range(1, 7):
            geometry = self._wheels.geometry[leg]
            lower, upper = self._sector_window_for(
                geometry.arc_min_degrees,
                geometry.arc_max_degrees,
                float(self._nominal_motor_degrees[leg + 11]),
                self.config.motor_headroom_degrees,
            )
            margin = min(
                self.config.arc_margin_degrees,
                max(0.0, 0.25 * (upper - lower)),
            )
            self._sector_window[leg - 1] = (lower + margin, upper - margin)

    def _reset_role_state(self) -> None:
        self._roles = [LegRole.STEP] * 6
        self._steering = np.zeros(6, dtype=np.float64)
        self._steering_from = np.zeros(6, dtype=np.float64)
        self._rate_sign = np.zeros(6, dtype=np.float64)
        self._roll_direction = np.zeros((6, 2), dtype=np.float64)
        self._sector = np.zeros(6, dtype=np.float64)
        self._sector_start = np.zeros(6, dtype=np.float64)
        self._sector_room = np.zeros(6, dtype=np.float64)
        self._sector_at_liftoff = np.zeros(6, dtype=np.float64)
        self._stance_offset = np.zeros((6, 3), dtype=np.float64)
        self._offset_at_liftoff = np.zeros((6, 3), dtype=np.float64)
        self._swinging = np.zeros(6, dtype=bool)
        self._reindexing = np.zeros(6, dtype=bool)
        self._was_in_window = np.zeros(6, dtype=bool)

    def reset(
        self,
        *,
        phase: float = 0.0,
        motor_degrees: ArrayLike | None = None,
    ) -> None:
        """Reset the gait and re-measure the sectors for the new stance."""

        super().reset(phase=phase, motor_degrees=motor_degrees)
        if getattr(self, "_wheels", None) is not None:
            self._build_wheel_model()
            self._reset_role_state()

    # -- diagnostics -----------------------------------------------------

    @property
    def leg_roles(self) -> dict[int, LegRole]:
        return {leg: self._roles[leg - 1] for leg in range(1, 7)}

    @property
    def rolling_legs(self) -> tuple[int, ...]:
        return tuple(
            leg for leg in range(1, 7) if self._roles[leg - 1] is LegRole.ROLL
        )

    @property
    def sector_degrees(self) -> NDArray[np.float64]:
        """Signed sector angle of each leg, relative to the nominal stance."""

        return self._sector.copy()

    @property
    def steering_degrees(self) -> NDArray[np.float64]:
        return self._steering.copy()

    @property
    def sector_window_degrees(self) -> NDArray[np.float64]:
        """Per-leg ``(low, high)`` sector limits after margin and headroom."""

        return self._sector_window.copy()

    # -- geometry --------------------------------------------------------

    def contact_travel(self, command: VelocityCommand | ArrayLike) -> NDArray[np.float64]:
        """Body-frame velocity each ground contact must have, shape ``(6, 2)``."""

        parsed = (
            command.as_array()
            if isinstance(command, VelocityCommand)
            else VelocityCommand.from_array(command).as_array()
        )
        vx, vy, yaw_rate = parsed
        points = self._nominal_feet[:, :2].copy()
        if self.config.yaw_about_footprint_centroid:
            points -= points.mean(axis=0)
        return -np.stack(
            [
                vx - yaw_rate * points[:, 1],
                vy + yaw_rate * points[:, 0],
            ],
            axis=1,
        )

    def steered_foot(self, leg: int, steering_degrees: float) -> Vector3:
        """Nominal contact of one leg after the upper joint steers it.

        The upper joint turns the wheel heading and the foot azimuth together,
        so steering *is* moving the foot around the hip.  The leg workspace is
        a surface of revolution about that axis, so IK reach is unaffected.
        """

        assert self._wheels is not None
        geometry = self._wheels.geometry[leg]
        angle = math.radians(geometry.steering_gain * steering_degrees)
        nominal = self._nominal_feet[leg - 1]
        radial = nominal[:2] - geometry.hip
        cosine, sine = math.cos(angle), math.sin(angle)
        rotated = np.array(
            [
                cosine * radial[0] - sine * radial[1],
                sine * radial[0] + cosine * radial[1],
            ],
            dtype=np.float64,
        )
        return np.array(
            [
                geometry.hip[0] + rotated[0],
                geometry.hip[1] + rotated[1],
                nominal[2],
            ],
            dtype=np.float64,
        )

    def _clip_stroke(self, stroke: Vector2) -> tuple[Vector2, bool]:
        lateral = (
            self.config.max_stride
            if self.config.max_lateral_stride is None
            else self.config.max_lateral_stride
        )
        radius = float(
            np.linalg.norm(
                [stroke[0] / self.config.max_stride, stroke[1] / lateral]
            )
        )
        if radius > 1.0:
            return stroke / radius, True
        return stroke, False

    # -- roll planning ---------------------------------------------------

    def _derate(self, leg_index: int) -> float:
        """Fade roll authority out over the last of the usable arc."""

        rate_sign = self._rate_sign[leg_index]
        if rate_sign == 0.0:
            return 0.0
        low, high = self._sector_window[leg_index]
        remaining = (
            high - self._sector[leg_index]
            if rate_sign > 0.0
            else self._sector[leg_index] - low
        )
        ratio = float(
            np.clip(remaining / self.config.roll_derate_degrees, 0.0, 1.0)
        )
        return self._quintic(ratio)

    def _budget_spent(self, leg_index: int) -> float:
        room = float(self._sector_room[leg_index])
        if room <= 0.0:
            return 1.0
        return abs(
            self._sector[leg_index] - self._sector_start[leg_index]
        ) / room

    def _arc_reserve(self, leg: int, travel: Vector2) -> float:
        """Arc one stance is expected to consume, bounded by what swing can undo."""

        assert self._wheels is not None
        geometry = self._wheels.geometry[leg]
        stance_time = self.config.duty_factor / self.config.cycle_frequency
        needed = math.degrees(
            float(np.linalg.norm(travel))
            * self.config.roll_authority
            * stance_time
            / geometry.roll_radius
        )
        swing_time = (1.0 - self.config.duty_factor) / self.config.cycle_frequency
        # A quintic rewind peaks at 1.875x its average rate.
        rewindable = self.config.max_roll_rate_degrees * swing_time / 1.875
        low, high = self._sector_window[leg - 1]
        return float(
            np.clip(
                self.config.arc_reserve_safety * needed,
                self.config.min_arc_reserve_degrees,
                min(high - low, rewindable),
            )
        )

    def _adopt(
        self,
        leg: int,
        solution: RollSolution,
        qualified: bool,
        travel: Vector2,
    ) -> None:
        index = leg - 1
        assert self._wheels is not None
        self._steering_from[index] = self._steering[index]
        if not qualified:
            self._roles[index] = LegRole.STEP
            self._steering[index] = 0.0
            self._rate_sign[index] = 0.0
            self._roll_direction[index] = 0.0
            self._sector_start[index] = 0.0
            self._sector_room[index] = 0.0
            return
        self._roles[index] = LegRole.ROLL
        self._steering[index] = solution.steering_degrees
        self._rate_sign[index] = solution.rate_sign
        self._roll_direction[index] = self._wheels.direction_unit(solution)
        low, high = self._sector_window[index]
        reserve = self._arc_reserve(leg, travel)
        # Start as close to the stance pose as the reserve allows, so a leg
        # only pre-indexes as far as this command needs it to.
        if solution.rate_sign > 0.0:
            start = min(high - reserve, 0.0)
        else:
            start = max(low + reserve, 0.0)
        # A fixed home makes the rewind-distance bound invariant when roles
        # change. All default arc windows contain zero; the projection also
        # supports asymmetric custom windows. This is a sector-component
        # rate constraint, not a bound on the summed IK + sector joint rate.
        if self.config.limit_reindex_rate:
            start = float(np.clip(0.0, low, high))
        self._sector_start[index] = float(np.clip(start, low, high))
        self._sector_room[index] = reserve

    # -- control ---------------------------------------------------------

    def step(
        self,
        command: VelocityCommand | ArrayLike,
        dt: float | None = None,
    ) -> GaitSample:
        """Advance one frame and solve all 18 actuator targets."""

        assert self._wheels is not None
        if dt is None:
            dt = 1.0 / self.config.control_frequency
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        requested = (
            command
            if isinstance(command, VelocityCommand)
            else VelocityCommand.from_array(command)
        )
        filtered = self._filter_command(requested, dt)
        activity = self._activity(filtered)
        moving = activity > self.config.idle_epsilon
        if moving:
            self._phase = (
                self._phase + dt * self.config.cycle_frequency
            ) % 1.0
        filtered_command = VelocityCommand.from_array(filtered)

        travel = self.contact_travel(filtered)
        lift_scale = float(
            np.clip(
                np.max(np.linalg.norm(travel, axis=1))
                / self.config.swing_lift_reference_speed,
                0.0,
                1.0,
            )
        )
        solutions = [
            self._wheels.solve(
                leg,
                travel[leg - 1],
                max_steering_degrees=self.config.max_steering_degrees,
            )
            for leg in range(1, 7)
        ]
        allowed = (
            set(range(1, 7))
            if self.config.roll_legs is None
            else set(self.config.roll_legs)
        )
        qualified = [
            moving
            and leg in allowed
            and solution.alignment >= self.config.min_roll_alignment
            and float(np.linalg.norm(travel[leg - 1])) > self.config.idle_epsilon
            for leg, solution in enumerate(solutions, start=1)
        ]
        if sum(qualified) < self.config.minimum_rolling_legs:
            qualified = [False] * 6

        targets = np.empty((6, 3), dtype=np.float64)
        previous_sector = self._sector.copy()
        bases = np.empty((6, 3), dtype=np.float64)
        stance_legs: list[int] = []
        clipped_legs = 0

        for leg in range(1, 7):
            index = leg - 1
            leg_phase = (self._phase + self.PHASE_OFFSETS[leg]) % 1.0
            in_window = moving and leg_phase >= self.config.duty_factor
            if not in_window:
                self._reindexing[index] = False

            swinging = self._swing_decision(index, in_window, travel[index])
            if self.config.limit_reindex_rate:
                # Decide at the window entry. Starting a quintic late in a
                # window can skip its low-speed beginning and jump the target.
                if in_window and self._was_in_window[index] and not self._swinging[index]:
                    swinging = False
                self._was_in_window[index] = in_window
            starting_swing = swinging and not self._swinging[index]
            if starting_swing:
                self._offset_at_liftoff[index] = self._stance_offset[index]
                self._sector_at_liftoff[index] = self._sector[index]
            # Roles and steering are only ever adopted at lift-off.  A
            # grounded leg cannot be re-steered without dragging its contact
            # across the floor, so a leg keeps what it latched until the swing
            # that carries its foot to the new azimuth.
            if starting_swing:
                self._adopt(
                    leg,
                    solutions[index],
                    qualified[index],
                    travel[index],
                )

            stroke, clipped = self._clip_stroke(
                self._articulated_velocity(index, travel[index])
                * self.config.duty_factor
                / self.config.cycle_frequency
            )
            clipped_legs += int(clipped)

            if swinging:
                progress = (
                    leg_phase - self.config.duty_factor
                ) / (1.0 - self.config.duty_factor)
                blend = self._quintic(progress)
                steering = (
                    self._steering_from[index]
                    + blend
                    * (self._steering[index] - self._steering_from[index])
                )
                # Stance carries the contact along ``travel``, so the foot has
                # to land at the near end of that stroke, not the far one.
                landing = np.array([-0.5 * stroke[0], -0.5 * stroke[1], 0.0])
                offset = (
                    self._offset_at_liftoff[index]
                    + blend * (landing - self._offset_at_liftoff[index])
                )
                offset = offset.copy()
                offset[2] += (
                    self.config.step_height
                    * lift_scale
                    * self._swing_lift(progress)
                )
                self._stance_offset[index] = offset
                self._sector[index] = (
                    self._sector_at_liftoff[index]
                    + blend
                    * (self._sector_start[index] - self._sector_at_liftoff[index])
                )
                if self.config.limit_reindex_rate:
                    # Covers interrupted/resumed windows and coarse sampling;
                    # the distance budget alone assumes a complete swing.
                    increment = self.config.max_roll_rate_degrees * dt
                    self._sector[index] = np.clip(
                        self._sector[index],
                        previous_sector[index] - increment,
                        previous_sector[index] + increment,
                    )
            else:
                stance_legs.append(leg)
                steering = self._steering[index]
                self._roll_and_walk(index, travel[index], dt)
                offset = self._stance_offset[index]

            self._swinging[index] = swinging
            bases[index] = self.steered_foot(leg, steering)
            targets[index] = bases[index] + offset

        self._last_stride_clip_fraction = clipped_legs / 6.0
        self._last_cycle_frequency = self.config.cycle_frequency

        results, solved_targets, backoff = self._solve_ik(targets, bases)
        solved = self._last_angles.copy()
        for leg, result in results.items():
            if result.converged:
                angles = result.angles
                solved[leg - 1] = angles.body
                solved[leg + 5] = angles.stage1
                solved[leg + 11] = angles.stage2
        self._last_angles = solved

        motor_degrees = np.degrees(solved) + 180.0
        # The arc is centred on the stage-2 axis, so this addition rotates the
        # tread under the robot without moving the contact the IK just placed.
        motor_degrees[12:18] += self._sector
        motor_degrees = np.clip(motor_degrees, 0.0, 360.0)

        return GaitSample(
            phase=self._phase,
            command=filtered_command,
            foot_targets=solved_targets,
            motor_degrees=motor_degrees,
            ik_results=results,
            stance_legs=tuple(stance_legs),
            cycle_frequency=self._last_cycle_frequency,
            stride_clip_fraction=self._last_stride_clip_fraction,
            ik_backoff_scale=backoff,
        )

    def _swing_decision(
        self,
        index: int,
        in_window: bool,
        travel: Vector2,
    ) -> bool:
        """Decide whether a leg leaves the ground in its own swing window.

        A stepping leg always does.  A rolling leg only does when it has spent
        its arc or when the command has turned away from the direction it
        latched, so a leg that is still rolling usefully keeps its contact and
        the gait spends fewer frames in swing.
        """

        if not in_window:
            return False
        if self._roles[index] is LegRole.STEP:
            return True
        if self.config.rolling_swing_policy == "disabled":
            return False
        if self.config.rolling_swing_policy == "periodic":
            return True
        if self._reindexing[index]:
            return True
        if self.config.anticipate_reindex_budget:
            # Skipping this window must leave enough budget to reach the
            # next window. Checking only already-spent reserve can defer a
            # rewind for an entire cycle and force unreachable articulation.
            assert self._wheels is not None
            geometry = self._wheels.geometry[index + 1]
            projected_speed = self.config.roll_authority * self._derate(index) * float(
                np.dot(travel, self._roll_direction[index]))
            rate = self._rate_sign[index] * math.degrees(projected_speed / geometry.roll_radius)
            rate = float(np.clip(rate, -self.config.max_roll_rate_degrees,
                                 self.config.max_roll_rate_degrees))
            swing_time = (1.0 - self.config.duty_factor) / self.config.cycle_frequency
            budget = self.config.max_roll_rate_degrees * swing_time / 1.875
            low, high = self._sector_window[index]
            home = float(np.clip(0.0, low, high))
            future = self._sector[index] + rate / self.config.cycle_frequency
            if future < max(low, home - budget) or future > min(high, home + budget):
                self._reindexing[index] = True
                return True
        speed = float(np.linalg.norm(travel))
        stale = speed > self.config.idle_epsilon and (
            float(np.dot(travel / speed, self._roll_direction[index]))
            < self.config.min_roll_alignment
        )
        if stale or self._budget_spent(index) >= self.config.reindex_threshold:
            self._reindexing[index] = True
            return True
        return False

    def _articulated_velocity(self, index: int, travel: Vector2) -> Vector2:
        """Contact travel left over once rolling has taken its share."""

        if self._roles[index] is not LegRole.ROLL or not self.config.subtract_rolling_travel:
            return travel
        direction = self._roll_direction[index]
        component = float(np.dot(travel, direction))
        authority = self.config.roll_authority * self._derate(index)
        return travel - authority * component * direction

    def _roll_and_walk(self, index: int, travel: Vector2, dt: float) -> None:
        """Integrate one grounded leg's sector angle and articulated stroke."""

        assert self._wheels is not None
        rolled = np.zeros(2, dtype=np.float64)
        if self._roles[index] is LegRole.ROLL:
            geometry = self._wheels.geometry[index + 1]
            direction = self._roll_direction[index]
            rate_sign = self._rate_sign[index]
            authority = self.config.roll_authority * self._derate(index)
            speed = authority * float(np.dot(travel, direction))
            rate = rate_sign * math.degrees(speed / geometry.roll_radius)
            rate = float(
                np.clip(
                    rate,
                    -self.config.max_roll_rate_degrees,
                    self.config.max_roll_rate_degrees,
                )
            )
            low, high = self._sector_window[index]
            if self.config.limit_reindex_rate:
                swing_time = (1.0 - self.config.duty_factor) / self.config.cycle_frequency
                rewindable = self.config.max_roll_rate_degrees * swing_time / 1.875
                home = float(np.clip(0.0, low, high))
                low = max(low, home - rewindable)
                high = min(high, home + rewindable)
            updated = float(np.clip(self._sector[index] + rate * dt, low, high))
            applied = updated - self._sector[index]
            self._sector[index] = updated
            realized = (
                rate_sign * math.radians(applied / dt) * geometry.roll_radius
            )
            rolled = realized * direction

        residual = travel - rolled if self.config.subtract_rolling_travel else travel
        offset = self._stance_offset[index].copy()
        offset[:2] += residual * dt
        offset[2] = 0.0
        offset[:2], _ = self._clip_stroke(offset[:2])
        self._stance_offset[index] = offset

    def _solve_ik(
        self,
        targets: NDArray[np.float64],
        bases: NDArray[np.float64],
    ) -> tuple[dict[int, IKResult], NDArray[np.float64], float]:
        # Back off toward the steered stance position this frame actually
        # used.  Backing off toward the fully steered pose while a leg is
        # mid-swing pulls it somewhere it was not asked to be, and the retry
        # can be less reachable than the request.
        requested = targets.copy()
        nominal = bases
        results = self._guard_branches(targets, self._inverse(targets, self._last_angles))
        backoff = 1.0
        for _ in range(self.config.ik_stride_backoff_attempts):
            if all(result.converged for result in results.values()):
                break
            backoff *= self.config.ik_stride_backoff_factor
            targets = nominal + (requested - nominal) * backoff
            results = self._guard_branches(
                targets,
                self._inverse(targets, self._last_angles),
            )
        return results, targets, backoff

    def _inverse(
        self,
        targets: NDArray[np.float64],
        seed: NDArray[np.float64],
    ) -> dict[int, IKResult]:
        return self.kinematics.inverse(
            targets,
            initial_angles=seed,
            frame="body",
            tolerance=self.config.ik_tolerance,
            max_iterations=self.config.ik_max_iterations,
            damping=self.config.ik_damping,
            max_step=self.config.ik_max_step,
        )

    def _guard_branches(
        self,
        targets: NDArray[np.float64],
        results: dict[int, IKResult],
    ) -> dict[int, IKResult]:
        """Pull any leg that has folded into the far IK branch back out.

        Steering gives the same contact two solutions, and the warm start has
        no reason to prefer the one the rest of the gait assumes.  A leg that
        takes the other one stops behaving like a wheel under the robot and
        starts behaving like a strut it climbs.
        """

        tolerance = math.radians(self.config.ik_branch_guard_degrees)

        def deviation(result: IKResult, leg: int) -> float:
            return abs(
                float(result.angles.stage2) - float(self._nominal_angles[leg + 11])
            )

        suspect = [
            leg
            for leg, result in results.items()
            if not result.converged or deviation(result, leg) > tolerance
        ]
        if not suspect:
            return results
        fallback = self._inverse(targets, self._nominal_angles)
        for leg in suspect:
            candidate = fallback[leg]
            if not candidate.converged:
                continue
            current = results[leg]
            if not current.converged or deviation(candidate, leg) < deviation(
                current, leg
            ):
                results[leg] = candidate
        return results


__all__ = [
    "LegRole",
    "SconeGaitV2",
    "SconeGaitV2Config",
]
