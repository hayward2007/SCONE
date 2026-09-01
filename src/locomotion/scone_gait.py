"""SCONE-specific point-support/sector-roll hybrid gait.

``TripodGait`` plans every grounded tire as one fixed point.  SCONE's TPU end
frame is instead a circular sector whose active contact moves as the lower
joint rotates.  ``SconeGait`` keeps the stable alternating-tripod scheduler and
IK posture, then coordinates upper-joint steering with a lower-joint sector
rotation.  The default residual-RL reference remains a bounded sweep.  The
interactive high-speed hybrid can instead enable phase-gated, unwrapped
rotation: each stance begins as one planted point, then the sector accumulates
rotation through late stance and early swing instead of undoing the angle on
every cycle.  This is deliberately different from the simulation-only
``roll-gait`` whose distal frames free-run continuously in velocity mode.

This is an experimental simulation-first controller.  It does not change the
default hardware gait and must be validated against measured joint limits and
TPU contact behaviour before physical use.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import ArrayLike, NDArray

from src.hardware import ControllerProtocol
from src.kinematics.leg import DEFAULT_MODEL_PATH

from .profile import MotionProfile, SPORT
from .tripod_gait import (
    GaitConfig,
    GaitSample,
    TripodGait,
    VelocityCommand,
)


Vector2 = NDArray[np.float64]
Vector3 = NDArray[np.float64]


@dataclass(frozen=True)
class SconeGaitConfig(GaitConfig):
    """Tuning for the sector-roll/creep hybrid.

    The smaller Cartesian stroke keeps the Phoenix-derived IK component as a
    stabilizer while the sector sweep contributes propulsion.
    """

    cycle_frequency: float = 0.65
    step_height: float = 0.025
    max_stride: float = 0.035
    max_lateral_stride: float | None = 0.025
    sector_sweep_degrees: float = 30.0
    max_steering_degrees: float = 55.0
    rolling_blend: float = 0.75
    # Upper-link steering is available for experiments, but it introduces
    # asymmetric yaw drift in the present MuJoCo contact model.  Keep the
    # validated default at zero and let the sector sweep plus tripod IK carry
    # the motion until hardware contact measurements justify enabling it.
    steering_blend: float = 0.0
    steering_probe_degrees: float = 5.0
    minimum_roll_alignment: float = 0.15
    # Fraction of each stance that keeps the extra sector-roll coordinate
    # fixed.  The underlying tripod IK still moves body/stage-1/stage-2 as
    # needed to keep the Cartesian support point planted.
    point_support_ratio: float = 0.55
    # Keep rotating while the foot is unloaded, then use the remaining swing
    # interval to decelerate to zero before touchdown.
    swing_roll_hold_ratio: float = 0.70
    # Interactive scone-gait only: accumulate the sector angle instead of
    # returning it during swing.  The default stays False because ordinary
    # RL reference targets and physical position mode are bounded to one turn.
    continuous_rotation: bool = False
    effective_roll_radius: float = 0.1225
    max_roll_rate_degrees: float = 360.0

    def __post_init__(self) -> None:
        super().__post_init__()
        if not 0.0 < self.sector_sweep_degrees <= 60.0:
            raise ValueError("sector_sweep_degrees must be in (0, 60]")
        if not 0.0 < self.max_steering_degrees <= 90.0:
            raise ValueError("max_steering_degrees must be in (0, 90]")
        if not 0.0 < self.steering_probe_degrees <= 15.0:
            raise ValueError("steering_probe_degrees must be in (0, 15]")
        if not 0.0 <= self.rolling_blend <= 1.0:
            raise ValueError("rolling_blend must be between 0 and 1")
        if not 0.0 <= self.steering_blend <= 1.0:
            raise ValueError("steering_blend must be between 0 and 1")
        if not 0.0 <= self.minimum_roll_alignment <= 1.0:
            raise ValueError("minimum_roll_alignment must be between 0 and 1")
        if not 0.0 < self.point_support_ratio < 1.0:
            raise ValueError("point_support_ratio must be between 0 and 1")
        if not 0.0 <= self.swing_roll_hold_ratio < 1.0:
            raise ValueError("swing_roll_hold_ratio must be in [0, 1)")
        if self.effective_roll_radius <= 0.0:
            raise ValueError("effective_roll_radius must be positive")
        if self.max_roll_rate_degrees <= 0.0:
            raise ValueError("max_roll_rate_degrees must be positive")


class SconeGait(TripodGait):
    """Alternating-tripod gait augmented with steerable sector rolling."""

    config: SconeGaitConfig

    def __init__(
        self,
        controller: ControllerProtocol | None = None,
        profile: str | MotionProfile = SPORT,
        *,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        config: SconeGaitConfig | None = None,
        end_effector_points: dict[int, ArrayLike] | None = None,
    ) -> None:
        super().__init__(
            controller,
            profile,
            model_path=model_path,
            config=config or SconeGaitConfig(),
            end_effector_points=end_effector_points,
        )
        self._nominal_roll_angles = np.zeros(6, dtype=np.float64)
        self._steering_gains = np.ones(6, dtype=np.float64)
        self._continuous_roll_degrees = np.zeros(6, dtype=np.float64)
        self._calibrate_sector_directions()

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _active_support_point(
        self,
        leg: int,
        motor_degrees: NDArray[np.float64],
    ) -> Vector3:
        """Return the lowest sector-patch centre in the body frame."""

        self.kinematics.forward_motor_degrees(motor_degrees, frame="body")
        model = self.kinematics.model
        data = self.kinematics.data
        geom_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            f"TIRE_{leg}_geom",
        )
        if geom_id < 0:
            raise ValueError(f"model is missing TIRE_{leg}_geom")
        mesh_id = int(model.geom_dataid[geom_id])
        if mesh_id < 0 or model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            raise ValueError(f"TIRE_{leg}_geom must be a mesh geom")

        address = int(model.mesh_vertadr[mesh_id])
        count = int(model.mesh_vertnum[mesh_id])
        local_vertices = model.mesh_vert[address : address + count]
        world_vertices = (
            local_vertices @ data.geom_xmat[geom_id].reshape(3, 3).T
            + data.geom_xpos[geom_id]
        )
        root_id = self.kinematics.legs[leg].root_body_id
        world_from_body = data.xmat[root_id].reshape(3, 3)
        body_vertices = (
            world_vertices - data.xpos[root_id]
        ) @ world_from_body
        lowest = float(np.min(body_vertices[:, 2]))
        patch = body_vertices[
            body_vertices[:, 2] <= lowest + self.SUPPORT_PATCH_DEPTH
        ]
        return np.mean(patch, axis=0)

    def _sector_tangent(self, leg: int, upper_offset_degrees: float) -> Vector2:
        """Numerically differentiate active contact against lower rotation."""

        probe = 1.0
        lower_index = leg + 11
        upper_index = leg - 1
        minus = self._nominal_motor_degrees.copy()
        plus = self._nominal_motor_degrees.copy()
        minus[upper_index] += upper_offset_degrees
        plus[upper_index] += upper_offset_degrees
        minus[lower_index] -= probe
        plus[lower_index] += probe
        tangent = (
            self._active_support_point(leg, plus)
            - self._active_support_point(leg, minus)
        )[:2]
        norm = float(np.linalg.norm(tangent))
        if norm <= 1e-9:
            raise ValueError(f"leg {leg} sector contact has no rolling tangent")
        return tangent / norm

    def _calibrate_sector_directions(self) -> None:
        """Measure each sector's rolling direction and upper steering sign."""

        probe = self.config.steering_probe_degrees
        for leg in range(1, 7):
            nominal = self._sector_tangent(leg, 0.0)
            steered = self._sector_tangent(leg, probe)
            nominal_angle = math.atan2(float(nominal[1]), float(nominal[0]))
            steered_angle = math.atan2(float(steered[1]), float(steered[0]))
            gain = self._wrap_angle(steered_angle - nominal_angle) / math.radians(
                probe
            )
            if abs(gain) < 0.25:
                raise ValueError(f"leg {leg} upper joint cannot steer sector tangent")
            self._nominal_roll_angles[leg - 1] = nominal_angle
            self._steering_gains[leg - 1] = gain
        self.kinematics.forward(self._nominal_angles, frame="body")

    def reset(
        self,
        *,
        phase: float = 0.0,
        motor_degrees: ArrayLike | None = None,
    ) -> None:
        """Reset the gait and recalibrate sector tangents for the new stance."""

        super().reset(phase=phase, motor_degrees=motor_degrees)
        if hasattr(self, "_nominal_roll_angles"):
            self._calibrate_sector_directions()
        if hasattr(self, "_continuous_roll_degrees"):
            self._continuous_roll_degrees.fill(0.0)

    @property
    def continuous_roll_degrees(self) -> NDArray[np.float64]:
        """Return the signed, accumulated sector rotation for all six legs."""

        return self._continuous_roll_degrees.copy()

    def set_continuous_roll_degrees(self, values: ArrayLike) -> None:
        """Seed the six multi-turn branches after a PPO/hybrid transition."""

        parsed = np.asarray(values, dtype=np.float64)
        if parsed.shape != (6,) or not np.all(np.isfinite(parsed)):
            raise ValueError("continuous roll seed must contain six finite degrees")
        self._continuous_roll_degrees = parsed.copy()

    def steering_solution(
        self,
        leg: int,
        command: VelocityCommand | ArrayLike,
    ) -> tuple[float, float, float]:
        """Return upper offset degrees, lower polarity, and roll alignment."""

        parsed = (
            command.as_array()
            if isinstance(command, VelocityCommand)
            else VelocityCommand.from_array(command).as_array()
        )
        vx, vy, yaw_rate = self._clamp_command(
            VelocityCommand.from_array(parsed)
        )
        x, y = self._nominal_feet[leg - 1, :2]
        body_velocity = np.array(
            [vx - yaw_rate * y, vy + yaw_rate * x],
            dtype=np.float64,
        )
        desired_contact = -body_velocity
        speed = float(np.linalg.norm(desired_contact))
        if speed <= self.config.idle_epsilon:
            return 0.0, 1.0, 0.0

        desired_angle = math.atan2(
            float(desired_contact[1]),
            float(desired_contact[0]),
        )
        nominal = float(self._nominal_roll_angles[leg - 1])
        gain = float(self._steering_gains[leg - 1])
        limit = math.radians(self.config.max_steering_degrees)
        candidates: list[tuple[float, float, float, float]] = []
        for polarity in (1.0, -1.0):
            base_angle = nominal + (0.0 if polarity > 0.0 else math.pi)
            requested = self._wrap_angle(desired_angle - base_angle) / gain
            applied = float(np.clip(requested, -limit, limit))
            actual = base_angle + gain * applied
            alignment = max(
                0.0,
                math.cos(self._wrap_angle(desired_angle - actual)),
            )
            candidates.append((alignment, -abs(applied), applied, polarity))
        alignment, _, applied, polarity = max(candidates)
        return math.degrees(applied), polarity, alignment

    def roll_coordinate(self, leg: int) -> float:
        """Return the bounded hybrid roll coordinate for one leg.

        ``-0.5`` is held through the first part of stance (point support).
        The sector rotates smoothly to ``+0.5`` only in late stance, then the
        unloaded swing returns it smoothly to ``-0.5``.  Therefore no joint
        is asked to rotate continuously while it is the sole contact point.
        """

        leg_phase = (self._phase + self.PHASE_OFFSETS[leg]) % 1.0
        if leg_phase < self.config.duty_factor:
            progress = leg_phase / self.config.duty_factor
            if progress <= self.config.point_support_ratio:
                return -0.5
            propulsion_progress = (
                progress - self.config.point_support_ratio
            ) / (1.0 - self.config.point_support_ratio)
            return -0.5 + self._quintic(propulsion_progress)
        progress = (
            leg_phase - self.config.duty_factor
        ) / (1.0 - self.config.duty_factor)
        return 0.5 - self._quintic(progress)

    def roll_gate(self, leg: int) -> float:
        """Return the phase gate for simultaneous walking and rotation.

        The first ``point_support_ratio`` of stance is exactly zero so the
        sector behaves as one planted point.  It accelerates during late
        stance, remains continuous through lift-off, and decelerates to zero
        before the next touchdown.  Unlike :meth:`roll_coordinate`, this gate
        never asks the accumulated angle to reverse.
        """

        leg_phase = (self._phase + self.PHASE_OFFSETS[leg]) % 1.0
        if leg_phase < self.config.duty_factor:
            progress = leg_phase / self.config.duty_factor
            if progress <= self.config.point_support_ratio:
                return 0.0
            propulsion_progress = (
                progress - self.config.point_support_ratio
            ) / (1.0 - self.config.point_support_ratio)
            return self._quintic(propulsion_progress)
        swing_progress = (
            leg_phase - self.config.duty_factor
        ) / (1.0 - self.config.duty_factor)
        if swing_progress <= self.config.swing_roll_hold_ratio:
            return 1.0
        braking_progress = (
            swing_progress - self.config.swing_roll_hold_ratio
        ) / (1.0 - self.config.swing_roll_hold_ratio)
        return 1.0 - self._quintic(braking_progress)

    def roll_rate_degrees(
        self,
        leg: int,
        command: VelocityCommand | ArrayLike,
    ) -> float:
        """Ideal no-slip sector rate magnitude for one foot in degrees/s."""

        parsed = (
            command.as_array()
            if isinstance(command, VelocityCommand)
            else VelocityCommand.from_array(command).as_array()
        )
        vx, vy, yaw_rate = self._clamp_command(
            VelocityCommand.from_array(parsed)
        )
        x, y = self._nominal_feet[leg - 1, :2]
        contact_speed = float(
            np.linalg.norm([vx - yaw_rate * y, vy + yaw_rate * x])
        )
        ideal_rate = math.degrees(
            contact_speed / self.config.effective_roll_radius
        )
        return min(ideal_rate, self.config.max_roll_rate_degrees)

    def step(
        self,
        command: VelocityCommand | ArrayLike,
        dt: float | None = None,
    ) -> GaitSample:
        step_dt = 1.0 / self.config.control_frequency if dt is None else dt
        base = super().step(command, step_dt)
        motor_degrees = base.motor_degrees.copy()
        activity = self._activity(base.command.as_array())
        if activity <= self.config.idle_epsilon:
            return base

        for leg in range(1, 7):
            steering, polarity, alignment = self.steering_solution(
                leg,
                base.command,
            )
            upper_index = leg - 1
            lower_index = leg + 11
            steering_weight = self.config.steering_blend * activity
            steering_target = self._nominal_motor_degrees[upper_index] + steering
            motor_degrees[upper_index] = (
                (1.0 - steering_weight) * motor_degrees[upper_index]
                + steering_weight * steering_target
            )

            usable_alignment = max(
                alignment,
                self.config.minimum_roll_alignment,
            )
            if self.config.continuous_rotation:
                # The active patch moves opposite the ground-reaction travel.
                # Accumulate the angle in only one direction; do not reset it
                # during swing as the former bounded coordinate did.
                self._continuous_roll_degrees[leg - 1] += (
                    -polarity
                    * self.roll_rate_degrees(leg, base.command)
                    * self.roll_gate(leg)
                    * usable_alignment
                    * step_dt
                )
                roll_target = (
                    motor_degrees[lower_index]
                    + self._continuous_roll_degrees[leg - 1]
                )
            else:
                roll_target = self._nominal_motor_degrees[lower_index] + (
                    # The active patch moves opposite the ground-reaction travel:
                    # invert the measured contact tangent when commanding the
                    # joint sweep that propels the body.
                    -polarity
                    * self.roll_coordinate(leg)
                    * self.config.sector_sweep_degrees
                    * activity
                    * usable_alignment
                )
            motor_degrees[lower_index] = (
                (1.0 - self.config.rolling_blend) * motor_degrees[lower_index]
                + self.config.rolling_blend * roll_target
            )

        if self.config.continuous_rotation:
            # The high-speed hybrid is simulation-only and intentionally uses
            # multi-turn lower targets.  Upper/stage-1 joints stay bounded.
            motor_degrees[:12] = np.clip(motor_degrees[:12], 0.0, 360.0)
        else:
            motor_degrees = np.clip(motor_degrees, 0.0, 360.0)
        return replace(base, motor_degrees=motor_degrees)


__all__ = [
    "SconeGait",
    "SconeGaitConfig",
]
