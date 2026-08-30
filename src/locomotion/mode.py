"""Base class for locomotion state-machine nodes."""

from __future__ import annotations

from collections.abc import Mapping

from src.hardware import ControllerProtocol

from .profile import MotionProfile


class Mode:
    name = "mode"

    def __init__(self, controller: ControllerProtocol, profile: MotionProfile) -> None:
        self.controller = controller
        self.profile = profile

    def _settle_simulated_raw_positions(
        self,
        positions: Mapping[int, int],
        *,
        tolerance: int = 64,
        timeout: float = 4.0,
    ) -> None:
        """Synchronize a pose only when the backend offers simulation waits."""

        waiter = getattr(self.controller, "wait_until_raw_positions", None)
        if waiter is None:
            return
        if waiter(positions, tolerance=tolerance, timeout=timeout):
            return
        actual = {
            motor_id: self.controller.get_position(motor_id)
            for motor_id in positions
        }
        raise RuntimeError(
            "simulated actuators did not settle before the mode transition: "
            f"targets={dict(positions)}, actual={actual}"
        )

    def _arc_wheel_velocities(self, velocity: int) -> dict[int, int]:
        """Return backend-calibrated arc-wheel signs when one is available."""

        mapper = getattr(self.controller, "arc_wheel_velocities", None)
        if mapper is not None:
            return mapper(velocity)
        from src.hardware import Actuator

        return {motor_id: velocity for motor_id in Actuator.Index.LOWER}

    def _set_simulated_drive_stage1_damping(self, enabled: bool) -> None:
        """Toggle an optional MuJoCo-only stage-1 stability adapter."""

        tuner = getattr(self.controller, "set_drive_stage1_damping", None)
        if tuner is not None:
            tuner(enabled)

    def change_mode(self) -> "Mode":
        raise NotImplementedError


__all__ = ["Mode"]
