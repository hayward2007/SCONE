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
        self._set_simulated_drive_stage1_damping(False)
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
        prepare_middle_degrees = self.profile.middle_initial_position - 20
        simulation_target = getattr(
            self.controller,
            "climb_prepare_middle_degrees",
            None,
        )
        if simulation_target is not None:
            # Climb is entered from Drive's 180-degree centre.  Standard's
            # legacy 240-20 target pushes a simulated foot down instead of
            # lifting it, so only the MuJoCo adapter replaces it with 160.
            prepare_middle_degrees = simulation_target(prepare_middle_degrees)
        prepare_middle_targets = {
            motor_id: prepare_middle_degrees for motor_id in middle_ids
        }
        self.controller.set_positions(prepare_middle_targets)
        time.sleep(0.5)
        self._settle_simulated_raw_positions(
            {
                motor_id: int(prepare_middle_degrees / 360 * 4096)
                for motor_id in middle_ids
            },
            tolerance=96,
        )
        for motor_id in lower_ids:
            self.controller.set_speed(motor_id, self.profile.boost_speed)
        lower_targets = {
            motor_id: Actuator.Position.CENTER for motor_id in lower_ids
        }
        self.controller.set_raw_positions(lower_targets)
        time.sleep(0.5)
        self._settle_simulated_raw_positions(lower_targets)
        middle_targets = {
            motor_id: Actuator.Position.CENTER for motor_id in middle_ids
        }
        self.controller.set_raw_positions(middle_targets)
        time.sleep(0.5)
        self._settle_simulated_raw_positions(middle_targets)

    def _default_stance(self) -> None:
        self.controller.set_all_mode(Actuator.OperatingMode.POSITION)
        for motor_id in Actuator.Index.LOWER:
            self.controller.set_speed(motor_id, self.profile.safety_speed)
        stance_ids = Actuator.Index.MIDDLE + Actuator.Index.LOWER
        self.controller.set_positions({motor_id: 180 for motor_id in stance_ids})
        time.sleep(1)
        self._settle_simulated_raw_positions(
            {motor_id: Actuator.Position.CENTER for motor_id in stance_ids},
            # A middle joint can retain roughly 10-15 degrees of static
            # deflection while the chassis is already supported on a tread.
            tolerance=192,
        )

    def _side_stance(self, middle_ids: tuple[int, ...]) -> None:
        for motor_id in Actuator.Index.LOWER:
            self.controller.set_speed(motor_id, self.profile.safety_speed)
        self.controller.set_positions(
            {motor_id: 270 for motor_id in Actuator.Index.LOWER + middle_ids}
        )
        time.sleep(1)
        # The selected middle joints intentionally remain load-bearing.  Wait
        # only for the arc wheels to enter the requested stair-contact phase
        # before changing those wheel actuators to velocity mode.
        self._settle_simulated_raw_positions(
            {motor_id: 3072 for motor_id in Actuator.Index.LOWER},
            tolerance=192,
        )
        self.controller.set_all_mode(Actuator.OperatingMode.VELOCITY)

    def _run(self, middle_ids: tuple[int, ...], velocity: int) -> None:
        self._side_stance(middle_ids)
        # Stair motion deliberately combines one load-bearing middle tripod
        # with a common raw wheel direction.  Unlike symmetric Drive, that
        # asymmetric contact phase is what advances the body toward the step.
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
