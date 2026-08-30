"""Stable public module: ``import SCONE``."""

from src.main import RobotCommand, RobotStatus, SCONE, UnsupportedCommandError

__all__ = ["RobotCommand", "RobotStatus", "SCONE", "UnsupportedCommandError"]


if __name__ == "__main__":
    from src.cli import main

    raise SystemExit(main())
