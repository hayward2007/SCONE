"""Model-based, non-RL gait generation for SCONE.

The scheduler follows the classic Lynxmotion Phoenix gait architecture:
each leg has a phase offset, supporting feet travel opposite the requested
body motion, lifted feet return to the front of their stroke, and inverse
kinematics turns the resulting foot positions into actuator targets.

This module deliberately does not replace :class:`src.locomotion.walk.Walk`.
The legacy class contains SCONE's proven discrete hardware motions; this
module is a continuous velocity-command gait that can be evaluated in MuJoCo
before it is enabled on the physical robot.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import mujoco
import numpy as np
from numpy.typing import ArrayLike, NDArray

from src.hardware import ControllerProtocol
from src.kinematics import IKResult, RobotKinematics
from src.kinematics.leg import DEFAULT_MODEL_PATH

from .profile import MotionProfile, SPORT, get_profile


Vector3 = NDArray[np.float64]


@dataclass(frozen=True)
class VelocityCommand:
    """Desired body-frame velocity.

    ``vx`` and ``vy`` are metres per second. ``yaw_rate`` is radians per
    second. The signs follow the body axes in ``model.xml``.
    """

    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0

    @classmethod
    def from_array(cls, value: ArrayLike) -> "VelocityCommand":
        array = np.asarray(value, dtype=np.float64)
        if array.shape != (3,):
            raise ValueError(f"velocity command must have shape (3,), got {array.shape}")
        return cls(*(float(item) for item in array))

    def as_array(self) -> Vector3:
        return np.array([self.vx, self.vy, self.yaw_rate], dtype=np.float64)


@dataclass(frozen=True)
class GaitConfig:
    """Tunable gait and safety limits.

    Defaults are intentionally conservative. They are starting values for
    MuJoCo validation, not claims about safe physical-robot limits.
    """

    control_frequency: float = 50.0
    cycle_frequency: float = 0.8
    duty_factor: float = 0.5
    step_height: float = 0.035
    max_stride: float = 0.070
    max_lateral_stride: float | None = None
    max_vx: float = 0.18
    max_vy: float = 0.12
    max_yaw_rate: float = 0.9
    command_time_constant: float = 0.15
    idle_epsilon: float = 1e-3
    ik_tolerance: float = 1e-4
    ik_max_iterations: int = 80
    ik_damping: float = 2e-3
    ik_max_step: float = 0.15
    ik_stride_backoff_attempts: int = 0
    ik_stride_backoff_factor: float = 0.8

    def __post_init__(self) -> None:
        if self.control_frequency <= 0.0 or self.cycle_frequency <= 0.0:
            raise ValueError("control_frequency and cycle_frequency must be positive")
        if not 0.0 < self.duty_factor < 1.0:
            raise ValueError("duty_factor must be between 0 and 1")
        if self.step_height < 0.0 or self.max_stride <= 0.0:
            raise ValueError("step_height must be non-negative and max_stride positive")
        if (
            self.max_lateral_stride is not None
            and not 0.0 < self.max_lateral_stride <= self.max_stride
        ):
            raise ValueError(
                "max_lateral_stride must be positive and no larger than max_stride"
            )
        if min(self.max_vx, self.max_vy, self.max_yaw_rate) <= 0.0:
            raise ValueError("velocity limits must be positive")
        if self.command_time_constant < 0.0 or self.idle_epsilon < 0.0:
            raise ValueError("filter time and idle epsilon cannot be negative")
        if self.ik_stride_backoff_attempts < 0:
            raise ValueError("ik_stride_backoff_attempts cannot be negative")
        if not 0.0 < self.ik_stride_backoff_factor < 1.0:
            raise ValueError("ik_stride_backoff_factor must be between 0 and 1")


@dataclass(frozen=True)
class GaitSample:
    """One generated control frame."""

    phase: float
    command: VelocityCommand
    foot_targets: NDArray[np.float64]
    motor_degrees: NDArray[np.float64]
    ik_results: dict[int, IKResult]
    stance_legs: tuple[int, ...]
    cycle_frequency: float = 0.8
    stride_clip_fraction: float = 0.0
    ik_backoff_scale: float = 1.0

    @property
    def converged(self) -> bool:
        return all(result.converged for result in self.ik_results.values())

    @property
    def failed_legs(self) -> tuple[int, ...]:
        return tuple(
            leg for leg, result in self.ik_results.items() if not result.converged
        )


class NonRLWalk:
    """Continuous Phoenix-style alternating-tripod gait for SCONE.

    The alternating support groups match the existing SCONE wiring and
    legacy walk implementation: ``(1, 4, 5)`` and ``(2, 3, 6)``.
    """

    TRIPOD_A = (1, 4, 5)
    TRIPOD_B = (2, 3, 6)
    PHASE_OFFSETS = {
        1: 0.0,
        2: 0.5,
        3: 0.5,
        4: 0.0,
        5: 0.0,
        6: 0.5,
    }
    # Average the vertices in the lowest 0.1 mm of the sector tip.  Selecting
    # one absolute-lowest vertex locks IK to either lateral edge of the 44 mm
    # wide TPU frame and creates an avoidable sideways moment at contact.
    SUPPORT_PATCH_DEPTH = 1e-4

    def __init__(
        self,
        controller: ControllerProtocol | None = None,
        profile: str | MotionProfile = SPORT,
        *,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        config: GaitConfig | None = None,
        end_effector_points: dict[int, ArrayLike] | None = None,
    ) -> None:
        self.controller = controller
        self.profile = get_profile(profile) if isinstance(profile, str) else profile
        self.config = config or GaitConfig()
        self._nominal_motor_degrees = self._profile_motor_degrees(self.profile)
        self._nominal_angles = np.radians(self._nominal_motor_degrees - 180.0)

        # TIRE_n's body origin is a CAD transform origin, not the point that
        # supports the robot. Unless the caller supplies calibrated points,
        # use the centre of each sector tip's lowest contact patch in the
        # nominal pose. This keeps IK off either lateral TPU edge.
        if end_effector_points is None:
            origin_kinematics = RobotKinematics(model_path)
            end_effector_points = self._infer_support_points(
                origin_kinematics,
                self._nominal_angles,
            )
        self.kinematics = RobotKinematics(
            model_path,
            end_effector_points=end_effector_points,
        )

        self._phase = 0.0
        self._filtered_command = np.zeros(3, dtype=np.float64)
        self._last_update_time: float | None = None
        self._last_angles = self._nominal_angles.copy()
        self._nominal_feet = self._forward_positions(self._nominal_angles)
        self._last_cycle_frequency = self.config.cycle_frequency
        self._last_stride_clip_fraction = 0.0

    @staticmethod
    def _profile_motor_degrees(profile: MotionProfile) -> NDArray[np.float64]:
        return np.array(
            list(profile.upper_initial_position)
            + [profile.middle_initial_position] * 6
            + [profile.lower_initial_position] * 6,
            dtype=np.float64,
        )

    @staticmethod
    def _infer_support_points(
        kinematics: RobotKinematics,
        nominal_angles: ArrayLike,
    ) -> dict[int, Vector3]:
        """Find the centre of each nominal TPU sector-tip contact patch."""

        # forward() sets all 18 qpos values on the shared MjData instance.
        kinematics.forward(nominal_angles, frame="body")
        model = kinematics.model
        data = kinematics.data
        root_body_id = kinematics.legs[1].root_body_id
        world_from_body = data.xmat[root_body_id].reshape(3, 3)
        root_position = data.xpos[root_body_id]
        points: dict[int, Vector3] = {}

        for leg in range(1, 7):
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
            world_from_geom = data.geom_xmat[geom_id].reshape(3, 3)
            world_vertices = (
                local_vertices @ world_from_geom.T + data.geom_xpos[geom_id]
            )
            body_vertices = (world_vertices - root_position) @ world_from_body
            lowest_height = float(np.min(body_vertices[:, 2]))
            support_patch = world_vertices[
                body_vertices[:, 2]
                <= lowest_height + NonRLWalk.SUPPORT_PATCH_DEPTH
            ]
            support_point = np.mean(support_patch, axis=0)

            tire_body_id = int(model.geom_bodyid[geom_id])
            world_from_tire = data.xmat[tire_body_id].reshape(3, 3)
            points[leg] = world_from_tire.T @ (
                support_point - data.xpos[tire_body_id]
            )
        return points

    def _forward_positions(self, angles: ArrayLike) -> NDArray[np.float64]:
        poses = self.kinematics.forward(angles, frame="body")
        return np.stack([poses[leg].position for leg in range(1, 7)])

    @property
    def phase(self) -> float:
        return self._phase

    @property
    def nominal_foot_positions(self) -> NDArray[np.float64]:
        return self._nominal_feet.copy()

    def reset(
        self,
        *,
        phase: float = 0.0,
        motor_degrees: ArrayLike | None = None,
    ) -> None:
        """Reset phase/filter state and optionally calibrate another stance.

        Passing the measured 18 actuator positions is recommended before
        physical use. It makes the current stance, rather than a hard-coded
        profile, the centre of every foot stroke.
        """

        degrees = (
            self._nominal_motor_degrees
            if motor_degrees is None
            else np.asarray(motor_degrees, dtype=np.float64)
        )
        if degrees.shape != (18,):
            raise ValueError("motor_degrees must contain actuator IDs 1..18")
        self._phase = float(phase) % 1.0
        self._filtered_command.fill(0.0)
        self._last_update_time = None
        self._nominal_motor_degrees = degrees.copy()
        self._nominal_angles = np.radians(degrees - 180.0)
        self._last_angles = self._nominal_angles.copy()
        self._nominal_feet = self._forward_positions(self._nominal_angles)
        self._last_cycle_frequency = self.config.cycle_frequency
        self._last_stride_clip_fraction = 0.0

    def reset_from_controller(self) -> NDArray[np.float64]:
        """Read the present raw positions and use them as the nominal stance."""

        if self.controller is None:
            raise RuntimeError("no controller was supplied to NonRLWalk")
        raw = np.array(
            [self.controller.get_position(motor_id) for motor_id in range(1, 19)],
            dtype=np.float64,
        )
        motor_degrees = raw / 4096.0 * 360.0
        self.reset(motor_degrees=motor_degrees)
        return motor_degrees

    def _clamp_command(self, command: VelocityCommand) -> Vector3:
        cfg = self.config
        return np.array(
            [
                np.clip(command.vx, -cfg.max_vx, cfg.max_vx),
                np.clip(command.vy, -cfg.max_vy, cfg.max_vy),
                np.clip(command.yaw_rate, -cfg.max_yaw_rate, cfg.max_yaw_rate),
            ],
            dtype=np.float64,
        )

    def _filter_command(self, command: VelocityCommand, dt: float) -> Vector3:
        target = self._clamp_command(command)
        tau = self.config.command_time_constant
        alpha = 1.0 if tau == 0.0 else 1.0 - math.exp(-dt / tau)
        self._filtered_command += alpha * (target - self._filtered_command)
        return self._filtered_command.copy()

    def _activity(self, command: Vector3) -> float:
        cfg = self.config
        normalized = np.abs(
            command / np.array([cfg.max_vx, cfg.max_vy, cfg.max_yaw_rate])
        )
        return float(np.clip(normalized.max(), 0.0, 1.0))

    def _stride_for_leg(
        self,
        leg: int,
        command: Vector3,
    ) -> tuple[Vector3, bool]:
        """Return the stance stroke caused by the requested body twist.

        For yaw, ``omega x r`` is evaluated separately at every nominal foot
        position. This is what makes a turn rotate about the body instead of
        degenerating into a sideways translation.
        """

        vx, vy, yaw_rate = command
        x, y = self._nominal_feet[leg - 1, :2]
        point_velocity = np.array(
            [vx - yaw_rate * y, vy + yaw_rate * x], dtype=np.float64
        )
        stance_time = self.config.duty_factor / self.config.cycle_frequency
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

    @staticmethod
    def _quintic(value: float) -> float:
        """Minimum-jerk interpolation with zero endpoint velocity/acceleration."""

        return value**3 * (10.0 + value * (-15.0 + 6.0 * value))

    @staticmethod
    def _swing_lift(value: float) -> float:
        """Unit-height swing arc with zero vertical velocity at touchdown."""

        return 16.0 * value**2 * (1.0 - value) ** 2

    def foot_targets(
        self,
        command: VelocityCommand | ArrayLike,
        *,
        phase: float | None = None,
    ) -> tuple[NDArray[np.float64], tuple[int, ...]]:
        """Generate all six body-frame foot targets without running IK."""

        parsed = (
            command.as_array()
            if isinstance(command, VelocityCommand)
            else VelocityCommand.from_array(command).as_array()
        )
        parsed = self._clamp_command(VelocityCommand.from_array(parsed))
        activity = self._activity(parsed)
        self._last_cycle_frequency = self.config.cycle_frequency
        cycle_phase = self._phase if phase is None else float(phase) % 1.0
        targets = self._nominal_feet.copy()
        stance_legs: list[int] = []

        if activity <= self.config.idle_epsilon:
            self._last_stride_clip_fraction = 0.0
            return targets, tuple(range(1, 7))

        clipped_legs = 0
        for leg in range(1, 7):
            leg_phase = (cycle_phase + self.PHASE_OFFSETS[leg]) % 1.0
            stroke, clipped = self._stride_for_leg(leg, parsed)
            clipped_legs += int(clipped)
            if leg_phase < self.config.duty_factor:
                stance_legs.append(leg)
                stance_progress = leg_phase / self.config.duty_factor
                targets[leg - 1] += (0.5 - stance_progress) * stroke
            else:
                swing_progress = (
                    (leg_phase - self.config.duty_factor)
                    / (1.0 - self.config.duty_factor)
                )
                blend = self._quintic(swing_progress)
                targets[leg - 1] += (blend - 0.5) * stroke
                targets[leg - 1, 2] += (
                    self.config.step_height
                    * activity
                    * self._swing_lift(swing_progress)
                )
        self._last_stride_clip_fraction = clipped_legs / 6.0
        return targets, tuple(stance_legs)

    def step(
        self,
        command: VelocityCommand | ArrayLike,
        dt: float | None = None,
    ) -> GaitSample:
        """Advance the gait and solve one 18-actuator position frame."""

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
        if activity > self.config.idle_epsilon:
            self._phase = (
                self._phase + dt * self.config.cycle_frequency
            ) % 1.0

        filtered_command = VelocityCommand.from_array(filtered)
        targets, stance_legs = self.foot_targets(
            filtered_command,
            phase=self._phase,
        )
        requested_targets = targets.copy()
        results = self.kinematics.inverse(
            targets,
            initial_angles=self._last_angles,
            frame="body",
            tolerance=self.config.ik_tolerance,
            max_iterations=self.config.ik_max_iterations,
            damping=self.config.ik_damping,
            max_step=self.config.ik_max_step,
        )
        ik_backoff_scale = 1.0
        for _ in range(self.config.ik_stride_backoff_attempts):
            if all(result.converged for result in results.values()):
                break
            ik_backoff_scale *= self.config.ik_stride_backoff_factor
            backed_off_targets = self._nominal_feet + (
                requested_targets - self._nominal_feet
            ) * ik_backoff_scale
            results = self.kinematics.inverse(
                backed_off_targets,
                initial_angles=self._last_angles,
                frame="body",
                tolerance=self.config.ik_tolerance,
                max_iterations=self.config.ik_max_iterations,
                damping=self.config.ik_damping,
                max_step=self.config.ik_max_step,
            )
            targets = backed_off_targets

        solved = self._last_angles.copy()
        for leg, result in results.items():
            if result.converged:
                angles = result.angles
                solved[leg - 1] = angles.body
                solved[leg + 5] = angles.stage1
                solved[leg + 11] = angles.stage2
        self._last_angles = solved
        motor_degrees = np.degrees(solved) + 180.0
        return GaitSample(
            phase=self._phase,
            command=filtered_command,
            foot_targets=targets,
            motor_degrees=motor_degrees,
            ik_results=results,
            stance_legs=stance_legs,
            cycle_frequency=self._last_cycle_frequency,
            stride_clip_fraction=self._last_stride_clip_fraction,
            ik_backoff_scale=ik_backoff_scale,
        )

    def send(self, sample: GaitSample, *, require_converged: bool = True) -> None:
        """Send a generated frame through either controller backend."""

        if self.controller is None:
            raise RuntimeError("no controller was supplied to NonRLWalk")
        if require_converged and not sample.converged:
            raise RuntimeError(
                "gait frame was not sent because IK failed for legs "
                f"{sample.failed_legs}"
            )
        if not np.all(np.isfinite(sample.motor_degrees)):
            raise RuntimeError("gait frame contains a non-finite motor target")
        if np.any(sample.motor_degrees < 0.0) or np.any(sample.motor_degrees > 360.0):
            raise RuntimeError("gait frame exceeds the 0..360 degree motor range")
        self.controller.set_positions(
            {
                motor_id: float(sample.motor_degrees[motor_id - 1])
                for motor_id in range(1, 19)
            }
        )

    def update(
        self,
        command: VelocityCommand | ArrayLike,
        dt: float | None = None,
        *,
        send: bool = False,
    ) -> GaitSample:
        """Generate one frame and optionally send it to the controller."""

        sample = self.step(command, dt)
        if send:
            self.send(sample)
        return sample

    def run(
        self,
        command_provider: Callable[[], VelocityCommand | ArrayLike],
        *,
        stop: Callable[[], bool],
    ) -> None:
        """Run a real-time control loop until ``stop()`` becomes true."""

        period = 1.0 / self.config.control_frequency
        self._last_update_time = time.monotonic()
        while not stop():
            started = time.monotonic()
            dt = started - self._last_update_time
            self._last_update_time = started
            self.update(command_provider(), dt=max(dt, 1e-6), send=True)
            remaining = period - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)


PhoenixTripodGait = NonRLWalk


__all__ = [
    "GaitConfig",
    "GaitSample",
    "NonRLWalk",
    "PhoenixTripodGait",
    "VelocityCommand",
]
