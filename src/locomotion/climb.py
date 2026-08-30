"""Climbing posture and arc-wheel motions."""

from __future__ import annotations

import time

from src.hardware import Actuator, ControllerProtocol

from .mode import Mode
from .profile import MotionProfile


class Climb(Mode):
    name = "climb"

    def __init__(self, controller: ControllerProtocol, profile: MotionProfile) -> None:
        super().__init__(controller, profile)
        self.controller.set_all_mode(Actuator.OperatingMode.POSITION)
        self.controller.set_all_speed(self.profile.safety_speed)
        self._prepare_tripod(
            Actuator.Index.MIDDLE_DIAGONAL_LEFT,
            Actuator.Index.LOWER_DIAGONAL_LEFT,
        )
        self._prepare_tripod(
            Actuator.Index.MIDDLE_DIAGONAL_RIGHT,
            Actuator.Index.LOWER_DIAGONAL_RIGHT,
        )

    def _prepare_tripod(
        self, middle_ids: tuple[int, ...], lower_ids: tuple[int, ...]
    ) -> None:
        self.controller.set_positions(
            {
                motor_id: self.profile.middle_initial_position - 20
                for motor_id in middle_ids
            }
        )
        time.sleep(0.5)
        for motor_id in lower_ids:
            self.controller.set_speed(motor_id, self.profile.boost_speed)
        self.controller.set_raw_positions(
            {motor_id: Actuator.Position.CENTER for motor_id in lower_ids}
        )
        time.sleep(0.5)
        self.controller.set_raw_positions(
            {motor_id: Actuator.Position.CENTER for motor_id in middle_ids}
        )
        time.sleep(0.5)

    def _default_stance(self) -> None:
        self.controller.set_all_mode(Actuator.OperatingMode.POSITION)
        for motor_id in Actuator.Index.LOWER:
            self.controller.set_speed(motor_id, self.profile.safety_speed)
        self.controller.set_positions(
            {motor_id: 180 for motor_id in Actuator.Index.MIDDLE + Actuator.Index.LOWER}
        )
        time.sleep(1)

    def _side_stance(self, middle_ids: tuple[int, ...]) -> None:
        for motor_id in Actuator.Index.LOWER:
            self.controller.set_speed(motor_id, self.profile.safety_speed)
        self.controller.set_positions(
            {motor_id: 270 for motor_id in Actuator.Index.LOWER + middle_ids}
        )
        time.sleep(1)
        self.controller.set_all_mode(Actuator.OperatingMode.VELOCITY)

    def _run(self, middle_ids: tuple[int, ...], velocity: int) -> None:
        self._side_stance(middle_ids)
        self.controller.set_velocities(
            {motor_id: velocity for motor_id in Actuator.Index.LOWER}
        )
        time.sleep(2.5)
        self.controller.set_velocities(
            {motor_id: 0 for motor_id in Actuator.Index.LOWER}
        )
        self._default_stance()

    def left(self) -> None:
        self._run(Actuator.Index.MIDDLE_RIGHT, -self.profile.climbing_speed)

    def right(self) -> None:
        self._run(Actuator.Index.MIDDLE_LEFT, self.profile.climbing_speed)

    def change_mode(self) -> Mode:
        from .walk import Walk

        return Walk(self.controller, self.profile, transition=True)


__all__ = ["Climb"]
