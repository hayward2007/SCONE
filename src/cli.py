"""The single interactive command surface for hardware and simulation."""

from __future__ import annotations

from collections.abc import Callable

from .hardware import HardwareProbe, discover_hardware
from .main import RobotCommand, SCONE, UnsupportedCommandError


KEY_BINDINGS = {
    "w": RobotCommand.FORWARD,
    "s": RobotCommand.BACKWARD,
    "a": RobotCommand.LEFT,
    "d": RobotCommand.RIGHT,
    "r": RobotCommand.CHANGE_MODE,
    "h": RobotCommand.HOME,
}


def _read_key() -> str:
    from getch import getch

    return getch().lower()


def print_remote_help() -> None:
    print("\nSCONE remote")
    print("  W/S: forward/backward   A/D: left/right")
    print("  R: Walk -> Drive -> Climb   H: home")
    print("  ?: help   Q: return to launcher\n")


def run_control_cli(robot: SCONE, *, key_reader: Callable[[], str] = _read_key) -> None:
    """Interpret keys once, then dispatch the same API on either backend."""

    print_remote_help()
    while True:
        key = key_reader().lower()
        if key == "q":
            return
        if key == "?":
            print_remote_help()
            continue
        command = KEY_BINDINGS.get(key)
        if command is None:
            print(f"[SCONE] unknown key {key!r}; press ? for help")
            continue
        try:
            robot.execute(command)
            print(
                f"[SCONE] command={command.value} "
                f"mode={robot.mode_name} profile={robot.profile_name}"
            )
        except UnsupportedCommandError as error:
            print(f"[SCONE] {error}")


def _select_profile() -> str:
    while True:
        choice = input("Profile [1: Standard, 2: Sport] (default 1): ").strip()
        if choice in ("", "1"):
            return "standard"
        if choice == "2":
            return "sport"
        print("Please enter 1 or 2.")


def _hardware_label(probe: HardwareProbe) -> str:
    if probe.available:
        return f"하드웨어 조종 ({probe.device_name})"
    return "하드웨어 조종 (현재 불가)"


def _run_hardware(probe: HardwareProbe, profile: str) -> None:
    if not probe.available or probe.device_name is None:
        print(f"[SCONE] hardware control is currently unavailable: {probe.detail}")
        return
    from .hardware import Controller

    robot = SCONE(Controller(probe.device_name), profile=profile)
    try:
        robot.initialize()
        run_control_cli(robot)
    finally:
        robot.close()


def main() -> int:
    print("[SCONE] searching for a physical controller...")
    probe = discover_hardware()
    while True:
        print("\nSCONE launcher")
        print("  1. 시뮬레이션 조종")
        print(f"  2. {_hardware_label(probe)}")
        print("  3. 하드웨어 다시 탐색")
        print("  Q. Quit")
        choice = input("Select: ").strip().lower()

        if choice == "1":
            from .simulation.cli_bridge import run

            run(profile=_select_profile())
        elif choice == "2":
            _run_hardware(probe, _select_profile())
        elif choice == "3":
            print("[SCONE] searching for a physical controller...")
            probe = discover_hardware()
        elif choice == "q":
            return 0
        else:
            print("Please select 1, 2, 3, or Q.")


__all__ = ["KEY_BINDINGS", "main", "run_control_cli"]
