"""Tripod gait used by the physical robot and simulation."""

from __future__ import annotations

import time

from src.hardware import Actuator, ControllerProtocol

from .mode import Mode
from .profile import MotionProfile


class Walk(Mode):
    name = "walk"
    MOVING_DEGREES = 20

    def __init__(
        self,
        controller: ControllerProtocol,
        profile: MotionProfile,
        *,
        transition: bool = False,
    ) -> None:
        super().__init__(controller, profile)
        self._set_simulated_drive_stage1_damping(False)
        if transition:
            self._enter_walk_pose()

    def _positions(self, ids: tuple[int, ...], value) -> None:
        self.controller.set_positions(
            {motor_id: value(motor_id) if callable(value) else value for motor_id in ids}
        )

    def _enter_walk_pose(self) -> None:
        self.controller.set_all_speed(self.profile.walking_speed)
        self._hold_left()
        time.sleep(0.05)
        for motor_id in Actuator.Index.LOWER_DIAGONAL_LEFT:
            self.controller.set_speed(motor_id, self.profile.boost_speed)
        self._positions(
            Actuator.Index.LOWER_DIAGONAL_LEFT,
            self.profile.lower_initial_position,
        )
        time.sleep(0.3)
        self._positions(
            Actuator.Index.UPPER_DIAGONAL_LEFT,
            lambda motor_id: self.profile.upper_initial_position[motor_id - 1],
        )
        time.sleep(0.5)
        self._release_left()
        self._hold_right()
        time.sleep(0.05)
        for motor_id in Actuator.Index.LOWER_DIAGONAL_RIGHT:
            self.controller.set_speed(motor_id, self.profile.boost_speed)
        self._positions(
            Actuator.Index.LOWER_DIAGONAL_RIGHT,
            self.profile.lower_initial_position,
        )
        time.sleep(0.3)
        self._positions(
            Actuator.Index.UPPER_DIAGONAL_RIGHT,
            lambda motor_id: self.profile.upper_initial_position[motor_id - 1],
        )
        time.sleep(0.5)
        self._release_right()
        time.sleep(0.05)

    def _hold_left(self) -> None:
        for motor_id in Actuator.Index.MIDDLE_DIAGONAL_LEFT:
            self.controller.set_speed(motor_id, self.profile.safety_speed)
        self._positions(
            Actuator.Index.MIDDLE_DIAGONAL_LEFT,
            self.profile.middle_initial_position - self.MOVING_DEGREES,
        )
        time.sleep(0.5)

    def _hold_right(self) -> None:
        for motor_id in Actuator.Index.MIDDLE_DIAGONAL_RIGHT:
            self.controller.set_speed(motor_id, self.profile.safety_speed)
        self._positions(
            Actuator.Index.MIDDLE_DIAGONAL_RIGHT,
            self.profile.middle_initial_position - self.MOVING_DEGREES,
        )
        time.sleep(0.5)

    def _release_left(self) -> None:
        self._positions(
            Actuator.Index.MIDDLE_DIAGONAL_LEFT,
            self.profile.middle_initial_position,
        )
        time.sleep(0.5)

    def _release_right(self) -> None:
        self._positions(
            Actuator.Index.MIDDLE_DIAGONAL_RIGHT,
            self.profile.middle_initial_position,
        )
        time.sleep(0.5)

    def _reset_upper(self) -> None:
        self._positions(
            Actuator.Index.UPPER,
            lambda motor_id: self.profile.upper_initial_position[motor_id - 1],
        )

    def forward(self) -> None:
        self._hold_left()
        time.sleep(0.1)
        positions = {
            motor_id: self.profile.upper_initial_position[motor_id - 1]
            - self.MOVING_DEGREES
            for motor_id in Actuator.Index.UPPER_DIAGONAL_LEFT
        }
        positions.update(
            {
                motor_id: self.profile.upper_initial_position[motor_id - 1]
                + self.MOVING_DEGREES
                for motor_id in Actuator.Index.UPPER_DIAGONAL_RIGHT
            }
        )
        self.controller.set_positions(positions)
        time.sleep(0.5)
        self._release_left()
        self._hold_right()
        time.sleep(0.1)
        self._reset_upper()
        time.sleep(0.5)
        self._release_right()

    def backward(self) -> None:
        self._hold_right()
        time.sleep(0.1)
        positions = {
            motor_id: self.profile.upper_initial_position[motor_id - 1]
            + self.MOVING_DEGREES
            for motor_id in Actuator.Index.UPPER_DIAGONAL_RIGHT
        }
        positions.update(
            {
                motor_id: self.profile.upper_initial_position[motor_id - 1]
                - self.MOVING_DEGREES
                for motor_id in Actuator.Index.UPPER_DIAGONAL_LEFT
            }
        )
        self.controller.set_positions(positions)
        time.sleep(0.5)
        self._release_right()
        self._hold_left()
        time.sleep(0.1)
        self._reset_upper()
        time.sleep(0.5)
        self._release_left()

    def right(self) -> None:
        self._turn(Actuator.Index.UPPER_DIAGONAL_LEFT)

    def left(self) -> None:
        self._turn(Actuator.Index.UPPER_DIAGONAL_RIGHT)

    def _turn(self, first_tripod: tuple[int, ...]) -> None:
        first_is_left = first_tripod == Actuator.Index.UPPER_DIAGONAL_LEFT
        hold_first = self._hold_left if first_is_left else self._hold_right
        release_first = self._release_left if first_is_left else self._release_right
        hold_second = self._hold_right if first_is_left else self._hold_left
        release_second = self._release_right if first_is_left else self._release_left

        hold_first()
        time.sleep(0.05)
        positions = {}
        for motor_id in Actuator.Index.UPPER_DIAGONAL_LEFT:
            sign = 1 if motor_id % 2 == 1 else -1
            positions[motor_id] = (
                self.profile.upper_initial_position[motor_id - 1]
                - self.MOVING_DEGREES * sign
            )
        for motor_id in Actuator.Index.UPPER_DIAGONAL_RIGHT:
            sign = 1 if motor_id % 2 == 1 else -1
            positions[motor_id] = (
                self.profile.upper_initial_position[motor_id - 1]
                + self.MOVING_DEGREES * sign
            )
        self.controller.set_positions(positions)
        time.sleep(0.05)
        release_first()
        hold_second()
        time.sleep(0.05)
        self._reset_upper()
        time.sleep(0.05)
        release_second()

    def change_mode(self) -> Mode:
        from .drive import Drive

        self._hold_left()
        time.sleep(0.05)
        for motor_id in Actuator.Index.LOWER_DIAGONAL_LEFT:
            self.controller.set_speed(motor_id, self.profile.boost_speed)
        lower_left_targets = {
            motor_id: Actuator.Position.CENTER if motor_id % 2 == 1 else 0
            for motor_id in Actuator.Index.LOWER_DIAGONAL_LEFT
        }
        self.controller.set_raw_positions(lower_left_targets)
        time.sleep(0.3)
        self._settle_simulated_raw_positions(lower_left_targets)
        upper_left_targets = {
            motor_id: Actuator.Position.CENTER
            for motor_id in Actuator.Index.UPPER_DIAGONAL_LEFT
        }
        self.controller.set_raw_positions(upper_left_targets)
        time.sleep(0.5)
        self._settle_simulated_raw_positions(upper_left_targets)
        self._release_left()
        self._hold_right()
        time.sleep(0.05)
        for motor_id in Actuator.Index.LOWER_DIAGONAL_RIGHT:
            self.controller.set_speed(motor_id, self.profile.boost_speed)
        lower_right_targets = {
            motor_id: Actuator.Position.CENTER if motor_id % 2 == 0 else 0
            for motor_id in Actuator.Index.LOWER_DIAGONAL_RIGHT
        }
        self.controller.set_raw_positions(lower_right_targets)
        time.sleep(0.3)
        self._settle_simulated_raw_positions(lower_right_targets)
        upper_right_targets = {
            motor_id: Actuator.Position.CENTER
            for motor_id in Actuator.Index.UPPER_DIAGONAL_RIGHT
        }
        self.controller.set_raw_positions(upper_right_targets)
        time.sleep(0.5)
        self._settle_simulated_raw_positions(upper_right_targets)
        self._release_right()
        time.sleep(0.05)
        middle_targets = {
            motor_id: Actuator.Position.CENTER for motor_id in Actuator.Index.MIDDLE
        }
        self.controller.set_raw_positions(middle_targets)
        self._settle_simulated_raw_positions(middle_targets)
        return Drive(self.controller, self.profile)


__all__ = ["Walk"]
