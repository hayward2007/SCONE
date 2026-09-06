"""Fault-adaptive reference gait: alternating tripod until a leg is lost.

An alternating tripod is only a valid support pattern while all six legs work.
Groups ``(1, 4, 5)`` and ``(2, 3, 6)`` each carry the robot alone, so losing
one leg leaves that group with two feet -- a line, not a polygon -- and the
robot tips about it every half cycle.  This is exactly the failure a fail-safe
controller has to avoid, and no residual policy can fix a scaffold that is
statically unstable by construction.

The classical answer is to trade speed for support: run a wave gait in which
only one leg leaves the ground at a time, so ``n`` working legs always leave
``n - 1`` feet down.  That fixes the duty factor at

    D = (n - 1) / n

which is 0.8 with five legs and 0.75 with four, against the tripod's 0.5.  The
stance stroke a foot must travel for a commanded body velocity ``v`` is

    s = v * D / f

so the same speed costs a 1.6x longer stroke at five legs than at six.  Under
a fixed workspace limit that is where the speed goes: stability is bought with
stride, not with cadence.

What is left to choose is the *order* the legs swing in.  With evenly spaced
offsets ``k / n`` and one leg up at a time, the set of support polygons is
``{H \\ {k}}`` whatever the order, so a static margin cannot rank orders.  What
the order does control is how far the missing support jumps between
consecutive swings: retracting two neighbouring legs back to back walks the
unsupported region along one side of the body and drags the centre of mass
with it.  :func:`plan_schedule` therefore maximises the minimum distance
between consecutively swung feet, which is the measurable form of the textbook
rule "alternate sides, back to front", and breaks ties on the static margin.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import permutations
from typing import Iterable, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .support_polygon import stability_margin
from .tripod_gait import TripodGait, VelocityCommand


LEG_INDICES = (1, 2, 3, 4, 5, 6)
HEALTHY_GAIT_CHOICES = ("tripod", "wave")

# Four feet is the fewest that can bound an area containing the centre of mass.
# Below that the scaffold cannot be statically stable at any phase and the
# schedule says so rather than pretending otherwise.
MINIMUM_SUPPORTED_LEGS = 4

# The margin search optimises geometry, not reachability, so a shift it likes
# may be outside some leg's workspace. These control how far it is backed off.
SHIFT_BACKOFF_ATTEMPTS = 5
SHIFT_BACKOFF_FACTOR = 0.7


@dataclass(frozen=True)
class GaitSchedule:
    """A support pattern for one particular set of working legs."""

    healthy_legs: tuple[int, ...]
    duty_factor: float
    phase_offsets: dict[int, float]
    pattern: str
    swing_order: tuple[int, ...]
    worst_margin: float
    # Body-frame translation applied to the body, realised by shifting every
    # nominal foot target by its negation. Zero for the healthy tripod.
    body_shift_xy: NDArray[np.float64] = field(
        default_factory=lambda: np.zeros(2, dtype=np.float64)
    )

    @property
    def statically_stable(self) -> bool:
        """Whether every swing phase still leaves the CoM inside the polygon."""

        return self.worst_margin > 0.0


def _tripod_schedule(
    nominal_feet_xy: NDArray[np.float64],
    center_of_mass_xy: NDArray[np.float64],
) -> GaitSchedule:
    groups = (TripodGait.TRIPOD_A, TripodGait.TRIPOD_B)
    worst = min(
        stability_margin(center_of_mass_xy, nominal_feet_xy[[leg - 1 for leg in group]])
        for group in groups
    )
    return GaitSchedule(
        healthy_legs=LEG_INDICES,
        duty_factor=0.5,
        phase_offsets=dict(TripodGait.PHASE_OFFSETS),
        pattern="tripod",
        swing_order=TripodGait.TRIPOD_B + TripodGait.TRIPOD_A,
        worst_margin=float(worst),
    )


def _order_score(
    order: Sequence[int],
    nominal_feet_xy: NDArray[np.float64],
) -> float:
    """Smallest gap between consecutively swung feet, around the cycle."""

    return min(
        float(
            np.linalg.norm(
                nominal_feet_xy[order[index] - 1]
                - nominal_feet_xy[order[(index + 1) % len(order)] - 1]
            )
        )
        for index in range(len(order))
    )


# The body may be shifted this far in the ground plane to recover support.
# 60 mm is inside the stance stroke the workspace already allows, so a shift
# never costs stride range that walking needs.
MAX_BODY_SHIFT_M = 0.060
_SHIFT_GRID = 13
_SHIFT_REFINEMENTS = 4


def _worst_margin(
    point: NDArray[np.float64],
    feet: NDArray[np.float64],
    working: Sequence[int],
) -> float:
    """Smallest margin across the swing phases of a one-leg-up wave gait."""

    return min(
        stability_margin(
            point,
            feet[[other - 1 for other in working if other != swinging]],
        )
        for swinging in working
    )


def _best_body_shift(
    feet: NDArray[np.float64],
    center: NDArray[np.float64],
    working: Sequence[int],
) -> tuple[NDArray[np.float64], float]:
    """Find the body translation that maximises the worst-case margin.

    Shifting the body by ``b`` moves the centre of mass to ``center + b``
    relative to the feet.  Each polygon's margin is concave in that point and
    the worst case is a minimum of concave functions, so the objective is
    concave and a shrinking grid converges on its maximum.
    """

    best = np.zeros(2, dtype=np.float64)
    best_score = _worst_margin(center, feet, working)
    span = MAX_BODY_SHIFT_M
    for _ in range(_SHIFT_REFINEMENTS):
        axis = np.linspace(-span, span, _SHIFT_GRID)
        improved = False
        for dx in axis:
            for dy in axis:
                candidate = best + np.array([dx, dy])
                if float(np.linalg.norm(candidate)) > MAX_BODY_SHIFT_M + 1e-12:
                    continue
                score = _worst_margin(center + candidate, feet, working)
                if score > best_score + 1e-12:
                    best_score = score
                    best_candidate = candidate
                    improved = True
        if improved:
            best = best_candidate
        span /= (_SHIFT_GRID - 1) / 2.0
    return best, best_score


def plan_schedule(
    healthy_legs: Iterable[int],
    nominal_feet_xy: ArrayLike,
    center_of_mass_xy: ArrayLike,
    *,
    healthy_gait: str = "tripod",
) -> GaitSchedule:
    """Choose duty factor, swing order and phase offsets for the working legs.

    ``nominal_feet_xy`` is the six ground-plane foot positions in body frame,
    indexed by ``leg - 1``; entries for failed legs are ignored but must be
    present so the caller never has to renumber.
    """

    if healthy_gait not in HEALTHY_GAIT_CHOICES:
        raise ValueError(
            f"unknown healthy_gait {healthy_gait!r}; expected {HEALTHY_GAIT_CHOICES}"
        )
    feet = np.asarray(nominal_feet_xy, dtype=np.float64)[:, :2]
    if feet.shape != (6, 2):
        raise ValueError("nominal_feet_xy must supply six body-frame positions")
    center = np.asarray(center_of_mass_xy, dtype=np.float64)
    if center.shape != (2,):
        raise ValueError("center_of_mass_xy must be a 2-vector")

    working = tuple(sorted({int(leg) for leg in healthy_legs}))
    unknown = [leg for leg in working if leg not in LEG_INDICES]
    if unknown:
        raise ValueError(f"unknown leg numbers {unknown}; expected {LEG_INDICES}")
    if len(working) < 3:
        raise ValueError("at least three working legs are required to walk")

    if len(working) == 6 and healthy_gait == "tripod":
        return _tripod_schedule(feet, center)

    count = len(working)
    duty_factor = (count - 1) / count
    # The first leg can be fixed: the cycle is rotationally symmetric, so only
    # the relative order matters. That is 120 candidates at six legs.
    head, *rest = working
    best_order = None
    best_score = -np.inf
    for tail in permutations(rest):
        order = (head, *tail)
        score = _order_score(order, feet)
        if score > best_score:
            best_score = score
            best_order = order
    assert best_order is not None

    # Half-slot offsets. With offsets at exactly k/n the swing window [D, 1)
    # is exactly one slot wide and the legs sit exactly on its edges, so
    # floating-point rounding can put two legs in the air at once -- which is
    # the one thing this schedule exists to prevent.
    offsets = {
        leg: ((index + 0.5) / count + duty_factor) % 1.0
        for index, leg in enumerate(best_order)
    }
    shift, worst = _best_body_shift(feet, center, working)
    return GaitSchedule(
        healthy_legs=working,
        duty_factor=duty_factor,
        phase_offsets=offsets,
        pattern="wave",
        swing_order=best_order,
        worst_margin=float(worst),
        body_shift_xy=shift,
    )


class FaultAdaptiveGait(TripodGait):
    """Tripod gait that reschedules itself around missing legs.

    Everything the parent does -- IK, stroke clipping, quintic swing, command
    filtering -- is reused unchanged.  What this class replaces is the fixed
    ``PHASE_OFFSETS``/``duty_factor`` pair with a schedule derived from the set
    of working legs, and the yaw centre with the centroid of the feet that are
    actually on the ground.
    """

    def __init__(
        self,
        *args,
        center_of_mass_xy: ArrayLike | None = None,
        healthy_gait: str = "tripod",
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self._healthy_gait = healthy_gait
        self._center_of_mass_xy = (
            self._nominal_feet[:, :2].mean(axis=0)
            if center_of_mass_xy is None
            else np.asarray(center_of_mass_xy, dtype=np.float64)
        )
        self._schedule = plan_schedule(
            LEG_INDICES,
            self._nominal_feet,
            self._center_of_mass_xy,
            healthy_gait=healthy_gait,
        )
        self._healthy_centroid = self._nominal_feet[:, :2].mean(axis=0)
        self._realised_shift_xy = np.zeros(2, dtype=np.float64)
        self._stance_offset = np.zeros(18, dtype=np.float64)

    @property
    def realised_shift_xy(self) -> NDArray[np.float64]:
        """The part of the planned shift the workspace actually allowed."""

        return self._realised_shift_xy.copy()

    @property
    def schedule(self) -> GaitSchedule:
        return self._schedule

    @property
    def healthy_legs(self) -> tuple[int, ...]:
        return self._schedule.healthy_legs

    def set_healthy_legs(
        self,
        legs: Iterable[int],
        *,
        center_of_mass_xy: ArrayLike | None = None,
    ) -> GaitSchedule:
        """Re-plan for a new set of working legs and return the schedule."""

        if center_of_mass_xy is not None:
            self._center_of_mass_xy = np.asarray(center_of_mass_xy, dtype=np.float64)
        self._schedule = plan_schedule(
            legs,
            self._nominal_feet,
            self._center_of_mass_xy,
            healthy_gait=self._healthy_gait,
        )
        working = [leg - 1 for leg in self._schedule.healthy_legs]
        self._healthy_centroid = self._nominal_feet[working, :2].mean(axis=0)
        # Solve the shift once, here, so the Cartesian and joint-space paths
        # agree on how much of it the workspace actually allowed.
        self._stance_offset = self._solve_stance_offset(self._schedule.body_shift_xy)
        return self._schedule

    def stance_offset_degrees(
        self,
        shift_xy: ArrayLike | None = None,
    ) -> NDArray[np.float64]:
        if shift_xy is None:
            return self._stance_offset.copy()
        return self._solve_stance_offset(np.asarray(shift_xy, dtype=np.float64))

    def _solve_stance_offset(
        self,
        shift_xy: ArrayLike,
    ) -> NDArray[np.float64]:
        """Per-motor degrees that move the body by ``shift_xy`` and stand still.

        A joint-space scaffold cannot express a lateral body shift as a common
        offset -- the hips sweep fore-aft and a symmetric stage-1 offset only
        changes ride height -- so the shift is solved once, here, with the same
        IK the Cartesian scaffold uses, and the result is added to the standing
        pose as a constant.  One solve per schedule, not one per frame.
        """

        shift = np.asarray(shift_xy, dtype=np.float64)
        if float(np.linalg.norm(shift)) <= 1e-9:
            self._realised_shift_xy = np.zeros(2, dtype=np.float64)
            return np.zeros(18, dtype=np.float64)
        # The margin search does not know the workspace, so the shift it
        # returns can be out of reach for some leg. Backing off keeps whatever
        # part of the improvement is reachable instead of refusing the fault
        # outright, which is the same policy the stride solver already uses.
        scale = 1.0
        for _ in range(SHIFT_BACKOFF_ATTEMPTS + 1):
            targets = self._nominal_feet.copy()
            targets[:, :2] -= shift * scale
            results = self.kinematics.inverse(
                targets,
                initial_angles=self._nominal_angles,
                frame="body",
                tolerance=self.config.ik_tolerance,
                max_iterations=self.config.ik_max_iterations,
                damping=self.config.ik_damping,
                max_step=self.config.ik_max_step,
            )
            unconverged = [
                leg
                for leg, result in results.items()
                if not result.converged and leg in self._schedule.healthy_legs
            ]
            if not unconverged:
                self._realised_shift_xy = shift * scale
                solved = self._nominal_angles.copy()
                for leg, result in results.items():
                    if result.converged:
                        solved[leg - 1] = result.angles.body
                        solved[leg + 5] = result.angles.stage1
                        solved[leg + 11] = result.angles.stage2
                return np.degrees(solved - self._nominal_angles)
            scale *= SHIFT_BACKOFF_FACTOR
        # Every backoff failed, so the nominal stance is what is left.
        self._realised_shift_xy = np.zeros(2, dtype=np.float64)
        return np.zeros(18, dtype=np.float64)

    def _stride_for_leg(
        self,
        leg: int,
        command,
    ):
        """Same stroke as the parent, but about the working feet's centroid.

        The parent turns the body about the centroid of all six nominal feet.
        After a leg is lost that point is no longer the centre of the support
        pattern, and yawing about it reintroduces the lateral translation the
        centroid correction was added to remove.
        """

        vx, vy, yaw_rate = command
        x, y = self._nominal_feet[leg - 1, :2]
        if self.config.yaw_about_footprint_centroid:
            x, y = np.array([x, y]) - self._healthy_centroid
        point_velocity = np.array(
            [vx - yaw_rate * y, vy + yaw_rate * x], dtype=np.float64
        )
        stance_time = self._schedule.duty_factor / self.config.cycle_frequency
        stroke_xy = point_velocity * stance_time
        lateral_limit = (
            self.config.max_stride
            if self.config.max_lateral_stride is None
            else self.config.max_lateral_stride
        )
        workspace_radius = float(
            np.linalg.norm(
                [
                    stroke_xy[0] / self.config.max_stride,
                    stroke_xy[1] / lateral_limit,
                ]
            )
        )
        clipped = workspace_radius > 1.0
        if clipped:
            stroke_xy /= workspace_radius
        return (
            np.array([stroke_xy[0], stroke_xy[1], 0.0], dtype=np.float64),
            clipped,
        )

    def foot_targets(
        self,
        command: VelocityCommand | ArrayLike,
        *,
        phase: float | None = None,
    ) -> tuple[NDArray[np.float64], tuple[int, ...]]:
        """Generate body-frame foot targets under the current schedule.

        Failed legs are held at their nominal pose: a detached leg has nothing
        to move and an unpowered one cannot be moved, so commanding either is
        at best wasted authority and at worst a torque the policy is charged
        for.
        """

        parsed = (
            command.as_array()
            if isinstance(command, VelocityCommand)
            else VelocityCommand.from_array(command).as_array()
        )
        parsed = self._clamp_command(VelocityCommand.from_array(parsed))
        activity = self._activity(parsed)
        self._last_cycle_frequency = self.config.cycle_frequency
        cycle_phase = self._phase if phase is None else float(phase) % 1.0
        schedule = self._schedule
        working = schedule.healthy_legs
        targets = self._nominal_feet.copy()
        # Moving the body by +b is the same as moving every foot by -b.
        targets[:, :2] -= self._realised_shift_xy

        if activity <= self.config.idle_epsilon:
            self._last_stride_clip_fraction = 0.0
            return targets, working

        duty = schedule.duty_factor
        stance_legs: list[int] = []
        clipped_legs = 0
        for leg in working:
            leg_phase = (cycle_phase + schedule.phase_offsets[leg]) % 1.0
            stroke, clipped = self._stride_for_leg(leg, parsed)
            clipped_legs += int(clipped)
            if leg_phase < duty:
                stance_legs.append(leg)
                stance_progress = leg_phase / duty
                targets[leg - 1] += (0.5 - stance_progress) * stroke
            else:
                swing_progress = (leg_phase - duty) / (1.0 - duty)
                blend = self._quintic(swing_progress)
                targets[leg - 1] += (blend - 0.5) * stroke
                targets[leg - 1, 2] += (
                    self.config.step_height * activity * self._swing_lift(swing_progress)
                )
        self._last_stride_clip_fraction = clipped_legs / max(1, len(working))
        return targets, tuple(stance_legs)


__all__ = [
    "MAX_BODY_SHIFT_M",
    "FaultAdaptiveGait",
    "GaitSchedule",
    "HEALTHY_GAIT_CHOICES",
    "LEG_INDICES",
    "MINIMUM_SUPPORTED_LEGS",
    "plan_schedule",
]
