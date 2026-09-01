"""Simulation-only synchronized-phase stair motion for SCONE."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from enum import Enum

import mujoco
import numpy as np

from ...cli import JoystickLimits, run_velocity_joystick_cli
from ...hardware import Actuator
from ...locomotion import VelocityCommand
from ...main import SCONE
from ..terrain import STAIR_PRESETS, TerrainType
from .controller import MuJoCoController


# XM430 goal-velocity unit: 0.229 rpm = 0.229 * 6 degrees/second.
LOWER_DEGREES_PER_SECOND_PER_VELOCITY_UNIT = 0.229 * 6.0


class StairControlState(str, Enum):
    IDLE = "idle"
    SYNCHRONIZING = "synchronizing"
    CLIMBING = "climbing"


@dataclass(frozen=True)
class SconeStairConfig:
    """Tuning for the six-sector, common-phase stair motion.

    ``phase_velocity`` uses the same numerical unit as a DYNAMIXEL goal
    velocity.  The controller integrates it into one shared geometric phase
    and sends mirrored extended-position targets to the lower six actuators.
    The position loop therefore corrects phase drift caused by unequal stair
    contact loads instead of letting each sector free-run.
    """

    max_vy: float = 0.12
    idle_epsilon: float = 1e-3
    synchronized_phase_degrees: float = 60.0
    tall_synchronized_phase_degrees: float = 90.0
    phase_velocity: float = 200.0
    easy_phase_velocity: float = 250.0
    easy_rise_limit: float = 0.125
    tall_rise_threshold: float = 0.175
    neutral_front_stage1_degrees: float = 180.0
    medium_front_stage1_degrees: float = 184.0
    tall_front_stage1_degrees: float = 195.0
    legacy_front_stage1_degrees: float = 270.0
    front_stage1_profile_velocity: int = 100
    front_stage1_tolerance_raw: int = 256
    front_stage1_sync_timeout: float = 4.0
    profile_velocity: int = 240
    profile_acceleration: int = 80
    phase_tolerance_raw: int = 96
    phase_sync_timeout: float = 4.0

    def __post_init__(self) -> None:
        if self.max_vy <= 0.0 or self.idle_epsilon < 0.0:
            raise ValueError("invalid stair command limits")
        if not all(
            0.0 <= phase < 360.0
            for phase in (
                self.synchronized_phase_degrees,
                self.tall_synchronized_phase_degrees,
            )
        ):
            raise ValueError("synchronized phases must be in [0, 360)")
        if not all(
            0.0 <= angle <= 360.0
            for angle in (
                self.neutral_front_stage1_degrees,
                self.medium_front_stage1_degrees,
                self.tall_front_stage1_degrees,
                self.legacy_front_stage1_degrees,
            )
        ):
            raise ValueError("front stage-1 angles must be in [0, 360]")
        if min(
            self.phase_velocity,
            self.easy_phase_velocity,
            self.front_stage1_profile_velocity,
            self.profile_velocity,
            self.profile_acceleration,
        ) <= 0:
            raise ValueError("stair phase/profile rates must be positive")
        if not 0.0 < self.easy_rise_limit < self.tall_rise_threshold:
            raise ValueError("stair rise thresholds must be positive and ordered")
        if min(self.front_stage1_tolerance_raw, self.phase_tolerance_raw) < 0:
            raise ValueError("stair synchronization tolerances cannot be negative")
        if min(self.front_stage1_sync_timeout, self.phase_sync_timeout) <= 0.0:
            raise ValueError("stair synchronization timeouts must be positive")


def synchronized_lower_degrees(phase_degrees: float) -> dict[int, float]:
    """Return one geometric sector phase in the mirrored MuJoCo joint axes.

    Odd lower joints use ``phase`` and even lower joints use ``360-phase``.
    This is the position equivalent of
    :meth:`MuJoCoController.arc_wheel_velocities`: the model's even joint axes
    point in the opposite direction, so equal numeric joint angles would not
    represent equal physical C-frame phases.

    The phase is deliberately not wrapped.  Extended-position mode can follow
    a common target through multiple revolutions while preserving the same
    odd/even geometric relationship.
    """

    if not math.isfinite(phase_degrees):
        raise ValueError("synchronized phase must be finite")
    return {
        motor_id: (
            float(phase_degrees)
            if motor_id % 2 == 1
            else float(360.0 - phase_degrees)
        )
        for motor_id in Actuator.Index.LOWER
    }


def synchronized_phase_spread_degrees(controller: MuJoCoController) -> float:
    """Measure the smallest circular spread of the six geometric phases."""

    phases = []
    for motor_id in Actuator.Index.LOWER:
        degrees = controller.get_position(motor_id) / 4096.0 * 360.0
        phases.append(degrees if motor_id % 2 == 1 else 360.0 - degrees)
    radians = np.radians(np.asarray(phases, dtype=np.float64))
    mean = math.atan2(float(np.sin(radians).mean()), float(np.cos(radians).mean()))
    errors = np.arctan2(np.sin(radians - mean), np.cos(radians - mean))
    return float(np.degrees(errors.max() - errors.min()))


class SconeStairClimber:
    """Climb with a leading stage-1 brace and one shared sector phase.

    The original physical stair motion aligns all C-shaped terminal frames and
    moves them together.  A velocity-only simulation path was effectively a
    Drive controller and could lose the intended relationship under unequal
    contact load.  This implementation instead owns one unwrapped phase
    ``theta``:

    * odd lower target = ``theta``
    * even lower target = ``360 - theta``

    Before lower rotation, the three leading stage-1 legs (IDs 7/9/11) acquire
    a rise-dependent partial brace derived from Legacy ``Climb.left()``'s
    vertical 270-degree pose.  The lower actuators then remain in
    extended-position mode.  Their independent position loops correct phase
    error while the shared target rotates.  No tripod gets a different lower
    speed, so the defining stair-motion phase is never intentionally broken.
    """

    def __init__(
        self,
        controller: MuJoCoController,
        *,
        terrain: TerrainType | str,
        config: SconeStairConfig | None = None,
    ) -> None:
        if not isinstance(controller, MuJoCoController):
            raise TypeError("SconeStairClimber requires MuJoCoController")
        self.controller = controller
        self.model = controller.model
        self.data = controller.data
        self.terrain = TerrainType.parse(terrain)
        if self.terrain not in STAIR_PRESETS:
            raise ValueError(
                "synchronized stair motion requires stairs-1, stairs-2, or stairs-3"
            )
        self.config = config or SconeStairConfig()
        self.root_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "UPPER_BODY_1",
        )
        if self.root_body_id < 0:
            raise ValueError("simulation model is missing UPPER_BODY_1")

        self.maximum_rise = max(STAIR_PRESETS[self.terrain].rises)
        self.state = StairControlState.IDLE
        self.initial_phase_degrees = (
            self.config.tall_synchronized_phase_degrees
            if self.maximum_rise >= self.config.tall_rise_threshold
            else self.config.synchronized_phase_degrees
        )
        self.phase_degrees = self.initial_phase_degrees
        self.selected_phase_velocity = (
            self.config.easy_phase_velocity
            if self.maximum_rise <= self.config.easy_rise_limit
            else self.config.phase_velocity
        )
        self.front_stage1_degrees = (
            self.config.tall_front_stage1_degrees
            if self.maximum_rise >= self.config.tall_rise_threshold
            else self.config.medium_front_stage1_degrees
            if self.maximum_rise > self.config.easy_rise_limit
            else self.config.neutral_front_stage1_degrees
        )
        self.front_stage1_sync_entries = 0
        self.phase_sync_entries = 0
        self.maximum_phase_spread_degrees = 0.0
        self._activity = 0.0
        self._prepared = False
        self._active = False

    def _phase_targets(self) -> dict[int, float]:
        return synchronized_lower_degrees(self.phase_degrees)

    def _record_phase_spread(self) -> float:
        spread = synchronized_phase_spread_degrees(self.controller)
        self.maximum_phase_spread_degrees = max(
            self.maximum_phase_spread_degrees,
            spread,
        )
        return spread

    def prepare_front_stage1(self) -> dict[int, int]:
        """Place the leading stage-1 legs in the measured stair brace pose.

        Legacy ``Climb.left()`` lowered IDs 7/9/11 to 270 degrees.  Holding
        that fully vertical pose destabilizes the current 10/15/20 cm model,
        so the improved motion keeps the same leading-leg concept but selects
        a measured, rise-dependent partial brace.
        """

        self.state = StairControlState.SYNCHRONIZING
        self.controller.set_speeds(
            {
                motor_id: self.config.front_stage1_profile_velocity
                for motor_id in Actuator.Index.MIDDLE_RIGHT
            }
        )
        targets = {
            motor_id: self.front_stage1_degrees
            for motor_id in Actuator.Index.MIDDLE_RIGHT
        }
        self.controller.set_positions(targets)
        self.front_stage1_sync_entries += 1
        return {
            motor_id: self.controller.degrees_to_raw(motor_id, degrees)
            for motor_id, degrees in targets.items()
        }

    def prepare(self) -> dict[int, int]:
        """Align all terminal frames before beginning the stair motion."""

        self.state = StairControlState.SYNCHRONIZING
        self.controller.set_all_mode(Actuator.OperatingMode.EXTENDED_POSITION)
        self.controller.set_speeds(
            {
                motor_id: self.config.profile_velocity
                for motor_id in Actuator.Index.LOWER
            }
        )
        self.controller.set_accelerations(
            {
                motor_id: self.config.profile_acceleration
                for motor_id in Actuator.Index.LOWER
            }
        )
        targets = self._phase_targets()
        self.controller.set_positions(targets)
        self.phase_sync_entries += 1
        self._prepared = True
        return {
            motor_id: self.controller.degrees_to_raw(motor_id, degrees)
            for motor_id, degrees in targets.items()
        }

    def activate(self) -> None:
        if not self._prepared:
            raise RuntimeError("prepare SconeStairClimber before activating it")
        self._active = True
        self.state = StairControlState.IDLE
        self._record_phase_spread()

    def stop(self) -> None:
        """Hold the last common phase without leaving a velocity command."""

        if self._prepared:
            self.controller.set_positions(self._phase_targets())
        self.state = StairControlState.IDLE
        self._activity = 0.0
        if self._active:
            self._record_phase_spread()

    def update(self, command: VelocityCommand, dt: float) -> None:
        """Advance the shared stair phase by one 50 Hz controller frame."""

        if not self._active:
            raise RuntimeError("prepare and activate SconeStairClimber first")
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        vy = float(np.clip(command.vy, -self.config.max_vy, self.config.max_vy))
        activity = abs(vy) / self.config.max_vy
        if activity <= self.config.idle_epsilon:
            self.stop()
            return

        # Positive body-y is the preset ascent direction.  The odd geometric
        # phase decreases in that direction; even targets mirror it.
        direction = -1.0 if vy > 0.0 else 1.0
        self._activity = activity
        self.phase_degrees += (
            direction
            * self.selected_phase_velocity
            * LOWER_DEGREES_PER_SECOND_PER_VELOCITY_UNIT
            * activity
            * dt
        )
        self.controller.set_positions(self._phase_targets())
        self.state = StairControlState.CLIMBING
        self._record_phase_spread()


def prepare_scone_stair_pose(robot: SCONE) -> None:
    """Turn side-on and enter the centred body posture used by stair motion.

    ``Drive`` is used only as the existing, tested posture transition.  The
    actual stair controller immediately replaces its lower velocity mode with
    synchronized extended-position control.
    """

    if robot.mode_name != "walk":
        raise RuntimeError("scone-stair preparation must start in Walk mode")
    for _ in range(4):
        robot.left()
    if robot.change_mode() != "drive":
        raise RuntimeError("scone-stair preparation failed to enter centred posture")


def prepare_synchronized_stair_motion(climber: SconeStairClimber) -> None:
    """Acquire the front brace, common phase, then activate the controller."""

    front_targets = climber.prepare_front_stage1()
    if not climber.controller.wait_until_raw_positions(
        front_targets,
        tolerance=climber.config.front_stage1_tolerance_raw,
        timeout=climber.config.front_stage1_sync_timeout,
    ):
        actual = {
            motor_id: climber.controller.get_position(motor_id)
            for motor_id in front_targets
        }
        raise RuntimeError(
            "front stage-1 stair brace did not settle: "
            f"targets={front_targets}, actual={actual}"
        )

    targets = climber.prepare()
    if not climber.controller.wait_until_raw_positions(
        targets,
        tolerance=climber.config.phase_tolerance_raw,
        timeout=climber.config.phase_sync_timeout,
    ):
        actual = {
            motor_id: climber.controller.get_position(motor_id)
            for motor_id in Actuator.Index.LOWER
        }
        raise RuntimeError(
            "synchronized stair phase did not settle: "
            f"targets={targets}, actual={actual}"
        )
    climber.activate()


def run_scone_stair_joystick_cli(
    robot: SCONE,
    *,
    terrain: TerrainType | str,
    stop_event: threading.Event | None = None,
    config: SconeStairConfig | None = None,
) -> None:
    """Run the simulation-only common-phase stair motion from A/D input."""

    if not isinstance(robot.controller, MuJoCoController):
        raise TypeError("scone-stair is available only in MuJoCo simulation")
    prepare_scone_stair_pose(robot)
    climber = SconeStairClimber(
        robot.controller,
        terrain=terrain,
        config=config,
    )
    prepare_synchronized_stair_motion(climber)
    limits = JoystickLimits(
        max_vx=0.0,
        max_vy=climber.config.max_vy,
        max_yaw_rate=0.0,
    )
    try:
        run_velocity_joystick_cli(
            limits=limits,
            apply_command=climber.update,
            profile_name=robot.profile_name,
            control_name=lambda: f"scone-stair/{climber.state.value}",
            control_hint=(
                "A/D: all six C-frames share one closed-loop stair phase; "
                "Drive velocity and alternating lower phases are not used"
            ),
            stop_event=stop_event,
        )
    finally:
        climber.stop()


__all__ = [
    "LOWER_DEGREES_PER_SECOND_PER_VELOCITY_UNIT",
    "SconeStairClimber",
    "SconeStairConfig",
    "StairControlState",
    "prepare_scone_stair_pose",
    "prepare_synchronized_stair_motion",
    "run_scone_stair_joystick_cli",
    "synchronized_lower_degrees",
    "synchronized_phase_spread_degrees",
]
