"""Arc-wheel velocity mode."""

from __future__ import annotations

import time

from src.hardware import Actuator, ControllerProtocol

from .mode import Mode
from .profile import MotionProfile


class Drive(Mode):
    name = "drive"

    def __init__(self, controller: ControllerProtocol, profile: MotionProfile) -> None:
        super().__init__(controller, profile)
        self._set_simulated_drive_stage1_damping(True)
        self.controller.set_all_mode(Actuator.OperatingMode.VELOCITY)

    def _run(self, velocity: int) -> None:
        self.controller.set_velocities(self._arc_wheel_velocities(velocity))
        time.sleep(1)
        self.controller.set_velocities(
            {motor_id: 0 for motor_id in Actuator.Index.LOWER}
        )

    def left(self) -> None:
        self._run(-self.profile.driving_speed)

    def right(self) -> None:
        self._run(self.profile.driving_speed)

    def change_mode(self) -> Mode:
        from .climb import Climb

        self._set_simulated_drive_stage1_damping(False)
        return Climb(self.controller, self.profile)


__all__ = ["Drive"]
