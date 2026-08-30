"""Base class for locomotion state-machine nodes."""

from __future__ import annotations

from src.hardware import ControllerProtocol

from .profile import MotionProfile


class Mode:
    name = "mode"

    def __init__(self, controller: ControllerProtocol, profile: MotionProfile) -> None:
        self.controller = controller
        self.profile = profile

    def change_mode(self) -> "Mode":
        raise NotImplementedError


__all__ = ["Mode"]
