"""Public, backend-independent SCONE robot API."""

from __future__ import annotations

import threading
import time
from enum import Enum

from .hardware import Actuator, ControllerProtocol
from .locomotion import Mode, Walk, get_profile
from .locomotion.profile import MotionProfile


class RobotCommand(str, Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    CHANGE_MODE = "change_mode"
    HOME = "home"


class RobotStatus(str, Enum):
    IDLE = "idle"
    INITIALIZING = "initializing"
    MOVING = "moving"
    SHUTTING_DOWN = "shutting_down"
    CLOSED = "closed"


class UnsupportedCommandError(RuntimeError):
    pass


class SCONE:
    """High-level API shared by physical and simulated SCONE robots.

    Construction never opens a port or starts a CLI. Pass a controller backend,
    then call :meth:`initialize`, motion methods, and :meth:`close` explicitly.
    """

    STARTING_MIDDLE_POSITION = 135
    ENDING_MIDDLE_POSITION = 150

    def __init__(
        self,
        controller: ControllerProtocol,
        *,
        profile: str | MotionProfile = "standard",
    ) -> None:
        self.controller = controller
        self.profile = get_profile(profile) if isinstance(profile, str) else profile
        self.mode: Mode = Walk(controller, self.profile)
        self.status = RobotStatus.IDLE
        self.initialized = False
        self._closed = False
        self._command_lock = threading.RLock()

    @property
    def profile_name(self) -> str:
        return self.profile.name

    @property
    def mode_name(self) -> str:
        return self.mode.name

    def initialize(self) -> None:
        """Move the robot to the configured profile's safe starting pose."""

        with self._command_lock:
            self._ensure_open()
            self.status = RobotStatus.INITIALIZING
            self.controller.set_torques(Actuator.Index.ALL, Actuator.Torque.OFF)
            # Stage-1 XM430-W350-T joints are position actuators in every
            # SCONE locomotion mode.  Set them explicitly as well as the
            # distal frames so a mode left behind by an external tool cannot
            # make a load-bearing leg run as a velocity actuator.
            for motor_id in Actuator.Index.XM:
                self.controller.set_mode(motor_id, Actuator.OperatingMode.POSITION)
            self.controller.set_accelerations(
                {motor_id: 20 for motor_id in Actuator.Index.XM}
            )
            # Physical set_mode may re-enable a motor.
            self.controller.set_torques(Actuator.Index.ALL, Actuator.Torque.OFF)
            time.sleep(0.1)

            self.controller.enable_torque()
            self.controller.set_all_speed(self.profile.safety_speed)
            self.controller.set_positions(
                {
                    motor_id: self.STARTING_MIDDLE_POSITION
                    for motor_id in Actuator.Index.MIDDLE
                }
            )
            time.sleep(0.5)
            self.controller.set_positions(
                {
                    motor_id: self.profile.upper_initial_position[motor_id - 1]
                    for motor_id in Actuator.Index.UPPER
                }
            )
            self.controller.set_speeds(
                {
                    motor_id: self.profile.boost_speed
                    for motor_id in Actuator.Index.LOWER
                }
            )
            self.controller.set_positions(
                {
                    motor_id: self.profile.lower_initial_position
                    for motor_id in Actuator.Index.LOWER
                }
            )
            time.sleep(0.7)
            self.controller.set_all_speed(self.profile.safety_speed)
            self.controller.set_positions(
                {
                    motor_id: self.profile.middle_initial_position
                    for motor_id in Actuator.Index.MIDDLE
                }
            )
            time.sleep(1.0)
            self.controller.set_all_speed(self.profile.walking_speed)

            self.mode = Walk(self.controller, self.profile)
            self.initialized = True
            self.status = RobotStatus.IDLE

    home = initialize

    def set_profile(self, profile: str | MotionProfile, *, apply: bool = True) -> None:
        with self._command_lock:
            self.profile = get_profile(profile) if isinstance(profile, str) else profile
            self.mode = Walk(self.controller, self.profile)
            if apply and self.initialized:
                self.initialize()

    def execute(self, command: RobotCommand | str) -> None:
        command = RobotCommand(command)
        if command is RobotCommand.HOME:
            self.home()
            return
        if command is RobotCommand.CHANGE_MODE:
            self.change_mode()
            return
        getattr(self, command.value)()

    def _move(self, movement: str) -> None:
        with self._command_lock:
            self._ensure_ready()
            method = getattr(self.mode, movement, None)
            if method is None:
                raise UnsupportedCommandError(
                    f"{movement!r} is not available in {self.mode_name} mode"
                )
            self.status = RobotStatus.MOVING
            try:
                method()
            finally:
                self.status = RobotStatus.IDLE

    def forward(self) -> None:
        self._move("forward")

    def backward(self) -> None:
        self._move("backward")

    def left(self) -> None:
        self._move("left")

    def right(self) -> None:
        self._move("right")

    def change_mode(self) -> str:
        with self._command_lock:
            self._ensure_ready()
            self.status = RobotStatus.MOVING
            try:
                self.mode = self.mode.change_mode()
            finally:
                self.status = RobotStatus.IDLE
            return self.mode_name

    def shutdown(self) -> None:
        """Return to a safer pose and release torque without closing the backend."""

        with self._command_lock:
            if not self.initialized or self._closed:
                return
            self.status = RobotStatus.SHUTTING_DOWN
            self.controller.set_speeds(
                {
                    motor_id: self.profile.safety_speed
                    for motor_id in Actuator.Index.MIDDLE
                }
            )
            self.controller.set_positions(
                {
                    motor_id: self.ENDING_MIDDLE_POSITION
                    for motor_id in Actuator.Index.MIDDLE
                }
            )
            time.sleep(1.0)
            self.controller.disable_torque()
            self.initialized = False
            self.status = RobotStatus.IDLE

    def close(self) -> None:
        if self._closed:
            return
        try:
            self.shutdown()
        finally:
            self.controller.close()
            self._closed = True
            self.status = RobotStatus.CLOSED

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("SCONE is closed")

    def _ensure_ready(self) -> None:
        self._ensure_open()
        if not self.initialized:
            raise RuntimeError(
                "SCONE.initialize() must be called before motion commands"
            )

    def __enter__(self) -> "SCONE":
        self.initialize()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def main() -> int:
    from .cli import main as run_cli

    return run_cli()


__all__ = ["RobotCommand", "RobotStatus", "SCONE", "UnsupportedCommandError", "main"]


if __name__ == "__main__":
    raise SystemExit(main())
