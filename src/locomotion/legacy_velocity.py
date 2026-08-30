"""Adapt continuous body commands to SCONE's blocking legacy motions."""

from __future__ import annotations

import threading
from typing import Protocol

from .non_rl_walk import VelocityCommand


class LegacyMotionRobot(Protocol):
    def forward(self) -> None: ...

    def backward(self) -> None: ...

    def left(self) -> None: ...

    def right(self) -> None: ...


def legacy_movement_for(command: VelocityCommand) -> str | None:
    """Select one legacy motion from a body-frame velocity command.

    The old gait has forward/backward and in-place yaw primitives, but no
    lateral/strafe primitive. Yaw therefore takes priority when both axes are
    requested, while a pure ``vy`` command intentionally produces no motion.
    """

    if command.yaw_rate > 0.0:
        return "left"
    if command.yaw_rate < 0.0:
        return "right"
    if command.vx > 0.0:
        return "forward"
    if command.vx < 0.0:
        return "backward"
    return None


class LegacyVelocityAdapter:
    """Run blocking legacy motions from the latest velocity command.

    Keyboard input and the MuJoCo viewer remain responsive while a legacy
    stride is executing. Commands received during a stride replace the queued
    command, so releasing a key prevents another stride from starting.
    """

    def __init__(self, robot: LegacyMotionRobot) -> None:
        self.robot = robot
        self._command = VelocityCommand()
        self._lock = threading.Lock()
        self._updated = threading.Event()
        self._stop = threading.Event()
        self._error: BaseException | None = None
        self._worker = threading.Thread(
            target=self._run,
            name="scone-legacy-velocity-adapter",
            daemon=True,
        )

    def start(self) -> None:
        self._worker.start()

    def update(self, command: VelocityCommand) -> None:
        self.raise_if_failed()
        with self._lock:
            self._command = command
        self._updated.set()

    def close(self) -> None:
        with self._lock:
            self._command = VelocityCommand()
        self._stop.set()
        self._updated.set()
        if self._worker.is_alive():
            self._worker.join()
        self.raise_if_failed()

    def raise_if_failed(self) -> None:
        if self._error is not None:
            raise self._error

    def _latest_command(self) -> VelocityCommand:
        with self._lock:
            return self._command

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                self._updated.wait(0.05)
                self._updated.clear()
                if self._stop.is_set():
                    break
                movement = legacy_movement_for(self._latest_command())
                if movement is not None:
                    getattr(self.robot, movement)()
        except BaseException as error:
            self._error = error
            self._stop.set()


__all__ = ["LegacyMotionRobot", "LegacyVelocityAdapter", "legacy_movement_for"]
