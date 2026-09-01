"""Simulation-only adaptive stair controller for SCONE's C-shaped feet."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum

import mujoco
import numpy as np
from numpy.typing import NDArray

from ...cli import JoystickLimits, run_velocity_joystick_cli
from ...hardware import Actuator
from ...locomotion import SCONE_V2_ARC_WHEEL, VelocityCommand
from ...main import SCONE
from ..terrain import STAIR_PRESETS, TerrainType
from .controller import MuJoCoController


class StairControlState(str, Enum):
    IDLE = "idle"
    ROLLING = "rolling"
    TRIPOD_ASSIST = "tripod-assist"


@dataclass(frozen=True)
class SconeStairConfig:
    """Tuning for roll-first, alternating-tripod stair ascent.

    Velocity values use the existing DYNAMIXEL goal-velocity units.  These
    values have only been evaluated with :class:`MuJoCoController`.
    """

    max_vy: float = 0.12
    idle_epsilon: float = 1e-3
    rolling_velocity: int = 150
    assist_support_velocity: int = 105
    assist_swing_velocity: int = 185
    support_middle_degrees: float = 250.0
    swing_middle_degrees: float = 165.0
    neutral_middle_degrees: float = 180.0
    assist_phase_seconds: float = 0.75
    transition_seconds: float = 0.18
    assist_phase_count: int = 6
    stall_window_seconds: float = 0.80
    minimum_progress_metres: float = 0.025
    direct_roll_clearance: float = 0.003
    first_riser_y: float = 0.35
    prehook_distance: float = 0.27

    def __post_init__(self) -> None:
        if self.max_vy <= 0.0 or self.idle_epsilon < 0.0:
            raise ValueError("invalid stair command limits")
        if min(
            self.rolling_velocity,
            self.assist_support_velocity,
            self.assist_swing_velocity,
        ) <= 0:
            raise ValueError("stair velocities must be positive")
        if not 0.0 <= self.swing_middle_degrees <= 360.0:
            raise ValueError("swing middle angle must be in [0, 360]")
        if not 0.0 <= self.support_middle_degrees <= 360.0:
            raise ValueError("support middle angle must be in [0, 360]")
        if not 0.0 <= self.neutral_middle_degrees <= 360.0:
            raise ValueError("neutral middle angle must be in [0, 360]")
        if self.assist_phase_seconds <= 0.0:
            raise ValueError("assist phase duration must be positive")
        if not 0.0 < self.transition_seconds <= self.assist_phase_seconds:
            raise ValueError("transition duration must fit within one phase")
        if self.assist_phase_count < 2:
            raise ValueError("at least two assist phases are required")
        if self.stall_window_seconds <= 0.0 or self.minimum_progress_metres <= 0.0:
            raise ValueError("stall detector values must be positive")
        if self.direct_roll_clearance < 0.0:
            raise ValueError("direct-roll clearance cannot be negative")
        if self.prehook_distance < 0.0:
            raise ValueError("prehook distance cannot be negative")


class SconeStairClimber:
    """Use rolling where it works and add tripod hooking only when needed.

    The controller assumes the procedural course direction: ascent is world
    ``+Y`` after :func:`prepare_scone_stair_pose` turns SCONE side-on.  It is
    intentionally simulation-only so unmeasured TPU friction, motor current,
    and joint-clearance assumptions cannot leak into the hardware launcher.
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
        self.config = config or SconeStairConfig()
        self.root_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "UPPER_BODY_1",
        )
        if self.root_body_id < 0:
            raise ValueError("simulation model is missing UPPER_BODY_1")

        profile = STAIR_PRESETS.get(self.terrain)
        self.maximum_rise = 0.0 if profile is None else max(profile.rises)
        self.direct_roll_reachable = (
            self.maximum_rise == 0.0
            or SCONE_V2_ARC_WHEEL.can_reach_riser(
                self.maximum_rise,
                clearance=self.config.direct_roll_clearance,
            )
        )
        self.tall_stair = not self.direct_roll_reachable
        self.state = StairControlState.IDLE
        self._direction = 0
        self._activity = 0.0
        self._state_elapsed = 0.0
        self._phase_index = 0
        self._phase_elapsed = 0.0
        self._progress_y = self._root_y()
        self._commanded_middle = np.full(
            6,
            self.config.neutral_middle_degrees,
            dtype=np.float64,
        )
        self._commanded_velocity = np.zeros(6, dtype=np.float64)
        self._phase_start_middle = self._commanded_middle.copy()
        self._phase_start_velocity = self._commanded_velocity.copy()
        self.assist_entries = 0
        self._known_prehook_used = False

    def _root_y(self) -> float:
        with self.controller.lock:
            return float(self.data.xpos[self.root_body_id, 1])

    @staticmethod
    def _smoothstep(value: float) -> float:
        value = float(np.clip(value, 0.0, 1.0))
        return value * value * (3.0 - 2.0 * value)

    def _set_middle(self, targets: NDArray[np.float64]) -> None:
        self.controller.set_positions(
            {
                motor_id: float(targets[motor_id - 7])
                for motor_id in Actuator.Index.MIDDLE
            }
        )
        self._commanded_middle = targets.copy()

    def _set_lower_velocity(self, targets: NDArray[np.float64]) -> None:
        self.controller.set_velocities(
            {
                motor_id: int(round(targets[motor_id - 13]))
                for motor_id in Actuator.Index.LOWER
            }
        )
        self._commanded_velocity = targets.copy()

    def _mapped_velocity(self, raw_velocity: float) -> NDArray[np.float64]:
        mapped = self.controller.arc_wheel_velocities(int(round(raw_velocity)))
        return np.array(
            [mapped[motor_id] for motor_id in Actuator.Index.LOWER],
            dtype=np.float64,
        )

    def _enter_rolling(self) -> None:
        self.state = StairControlState.ROLLING
        self._state_elapsed = 0.0
        self._progress_y = self._root_y()
        self._set_middle(
            np.full(6, self.config.neutral_middle_degrees, dtype=np.float64)
        )

    def _enter_assist(self) -> None:
        # Synchronize the otherwise free-running sector phase before assigning
        # different speeds to the two tripods.  The successful baseline stops
        # after its approach pulse; blending directly from full rolling made
        # the first support swap throw the chassis sideways on tall stairs.
        self._set_lower_velocity(np.zeros(6, dtype=np.float64))
        self.state = StairControlState.TRIPOD_ASSIST
        self.assist_entries += 1
        self._state_elapsed = 0.0
        self._phase_index = 0
        self._phase_elapsed = 0.0
        self._phase_start_middle = self._commanded_middle.copy()
        self._phase_start_velocity = self._commanded_velocity.copy()

    def _assist_targets(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        support_middle, swing_middle = (
            (
                Actuator.Index.MIDDLE_DIAGONAL_RIGHT,
                Actuator.Index.MIDDLE_DIAGONAL_LEFT,
            )
            if self._phase_index % 2 == 0
            else (
                Actuator.Index.MIDDLE_DIAGONAL_LEFT,
                Actuator.Index.MIDDLE_DIAGONAL_RIGHT,
            )
        )
        support_lower, swing_lower = (
            (
                Actuator.Index.LOWER_DIAGONAL_RIGHT,
                Actuator.Index.LOWER_DIAGONAL_LEFT,
            )
            if self._phase_index % 2 == 0
            else (
                Actuator.Index.LOWER_DIAGONAL_LEFT,
                Actuator.Index.LOWER_DIAGONAL_RIGHT,
            )
        )
        middle = np.full(6, self.config.neutral_middle_degrees, dtype=np.float64)
        support_angle = self.config.neutral_middle_degrees + self._activity * (
            self.config.support_middle_degrees
            - self.config.neutral_middle_degrees
        )
        swing_angle = self.config.neutral_middle_degrees + self._activity * (
            self.config.swing_middle_degrees
            - self.config.neutral_middle_degrees
        )
        for motor_id in support_middle:
            middle[motor_id - 7] = support_angle
        for motor_id in swing_middle:
            middle[motor_id - 7] = swing_angle

        raw = np.zeros(6, dtype=np.float64)
        for motor_id in support_lower:
            raw[motor_id - 13] = (
                self._direction
                * self.config.assist_support_velocity
                * self._activity
            )
        for motor_id in swing_lower:
            raw[motor_id - 13] = (
                self._direction
                * self.config.assist_swing_velocity
                * self._activity
            )
        mapped = np.zeros(6, dtype=np.float64)
        for motor_id in Actuator.Index.LOWER:
            value = self.controller.arc_wheel_velocities(
                int(round(raw[motor_id - 13]))
            )
            mapped[motor_id - 13] = value[motor_id]
        return middle, mapped

    def _apply_assist(self) -> None:
        target_middle, target_velocity = self._assist_targets()
        blend = self._smoothstep(
            self._phase_elapsed / self.config.transition_seconds
        )
        self._set_middle(
            self._phase_start_middle
            + blend * (target_middle - self._phase_start_middle)
        )
        self._set_lower_velocity(
            self._phase_start_velocity
            + blend * (target_velocity - self._phase_start_velocity)
        )

    def _advance_assist_phase(self) -> None:
        self._phase_index += 1
        if self._phase_index >= self.config.assist_phase_count:
            self._enter_rolling()
            return
        self._phase_elapsed = 0.0
        self._phase_start_middle = self._commanded_middle.copy()
        self._phase_start_velocity = self._commanded_velocity.copy()

    def stop(self) -> None:
        self._set_lower_velocity(np.zeros(6, dtype=np.float64))
        self.state = StairControlState.IDLE
        self._direction = 0
        self._activity = 0.0
        self._state_elapsed = 0.0

    def update(self, command: VelocityCommand, dt: float) -> None:
        """Advance the roll/assist state machine by one controller frame."""

        if dt <= 0.0:
            raise ValueError("dt must be positive")
        vy = float(np.clip(command.vy, -self.config.max_vy, self.config.max_vy))
        activity = abs(vy) / self.config.max_vy
        if activity <= self.config.idle_epsilon:
            self.stop()
            return

        # Positive body-y is the legacy "left" stair direction and uses a
        # negative raw velocity before the simulation's odd/even axis map.
        direction = -1 if vy > 0.0 else 1
        world_direction = 1.0 if vy > 0.0 else -1.0
        if self.state is StairControlState.IDLE or direction != self._direction:
            self._direction = direction
            self._activity = activity
            self.controller.set_all_mode(Actuator.OperatingMode.VELOCITY)
            self._enter_rolling()
        self._activity = activity
        self._state_elapsed += dt

        if self.state is StairControlState.ROLLING:
            rolling = self._mapped_velocity(
                self._direction * self.config.rolling_velocity * activity
            )
            self._set_lower_velocity(rolling)
            root_y = self._root_y()
            near_known_tall_stair = (
                self.tall_stair
                and not self._known_prehook_used
                and world_direction > 0.0
                and root_y
                >= self.config.first_riser_y - self.config.prehook_distance
            )
            if near_known_tall_stair:
                self._known_prehook_used = True
                self._enter_assist()
                self._apply_assist()
                return
            if self._state_elapsed >= self.config.stall_window_seconds:
                progress = world_direction * (root_y - self._progress_y)
                if progress < self.config.minimum_progress_metres:
                    self._enter_assist()
                    self._apply_assist()
                else:
                    self._state_elapsed = 0.0
                    self._progress_y = root_y
            return

        self._phase_elapsed += dt
        while (
            self.state is StairControlState.TRIPOD_ASSIST
            and self._phase_elapsed >= self.config.assist_phase_seconds
        ):
            self._phase_elapsed -= self.config.assist_phase_seconds
            self._advance_assist_phase()
        if self.state is StairControlState.TRIPOD_ASSIST:
            self._apply_assist()
        else:
            self._set_lower_velocity(
                self._mapped_velocity(
                    self._direction
                    * self.config.rolling_velocity
                    * self._activity
                )
            )


def prepare_scone_stair_pose(robot: SCONE) -> None:
    """Turn the initialized simulation side-on and enter the Drive posture."""

    if robot.mode_name != "walk":
        raise RuntimeError("scone-stair preparation must start in Walk mode")
    for _ in range(4):
        robot.left()
    if robot.change_mode() != "drive":
        raise RuntimeError("scone-stair preparation failed to enter Drive mode")


def run_scone_stair_joystick_cli(
    robot: SCONE,
    *,
    terrain: TerrainType | str,
    stop_event: threading.Event | None = None,
    config: SconeStairConfig | None = None,
) -> None:
    """Run the simulation-only adaptive stair controller from A/D input."""

    if not isinstance(robot.controller, MuJoCoController):
        raise TypeError("scone-stair is available only in MuJoCo simulation")
    prepare_scone_stair_pose(robot)
    climber = SconeStairClimber(
        robot.controller,
        terrain=terrain,
        config=config,
    )
    limits = JoystickLimits(max_vx=0.0, max_vy=climber.config.max_vy, max_yaw_rate=0.0)
    run_velocity_joystick_cli(
        limits=limits,
        apply_command=climber.update,
        profile_name=robot.profile_name,
        control_name=lambda: f"scone-stair/{climber.state.value}",
        control_hint=(
            "A/D: side-on stair travel; continuous roll switches to "
            "alternating tripod hook assist when required"
        ),
        stop_event=stop_event,
    )


__all__ = [
    "SconeStairClimber",
    "SconeStairConfig",
    "StairControlState",
    "prepare_scone_stair_pose",
    "run_scone_stair_joystick_cli",
]
