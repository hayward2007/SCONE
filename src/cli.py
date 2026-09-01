"""The single interactive command surface for hardware and simulation."""

from __future__ import annotations

import os
import select
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, TextIO

from .hardware import HardwareProbe, discover_hardware
from .locomotion import (
    GaitConfig,
    LegacyVelocityAdapter,
    SconeGait,
    SconeGaitConfig,
    TripodGait,
    VelocityCommand,
)
from .main import RobotCommand, SCONE, UnsupportedCommandError


KEY_BINDINGS = {
    "w": RobotCommand.FORWARD,
    "s": RobotCommand.BACKWARD,
    "a": RobotCommand.LEFT,
    "d": RobotCommand.RIGHT,
    "r": RobotCommand.CHANGE_MODE,
    "h": RobotCommand.HOME,
}

_JOYSTICK_KEY_BINDINGS = {
    "w": ("y", 1.0),
    "s": ("y", -1.0),
    "a": ("x", -1.0),
    "d": ("x", 1.0),
    "left": ("yaw", 1.0),
    "right": ("yaw", -1.0),
}

_TERMINAL_KEY_SEQUENCES = {
    b"\x1b[A": "up",
    b"\x1b[B": "down",
    b"\x1b[D": "left",
    b"\x1b[C": "right",
    b"\x1bOA": "up",
    b"\x1bOB": "down",
    b"\x1bOD": "left",
    b"\x1bOC": "right",
}


@dataclass(frozen=True)
class JoystickLimits:
    """Body-command values represented by a fully deflected CLI joystick."""

    max_vx: float = 0.18
    max_vy: float = 0.12
    max_yaw_rate: float = 0.9


@dataclass(frozen=True)
class JoystickState:
    """Normalized terminal joystick state in the range ``-1..1``.

    ``x`` is the left/right stick axis and ``y`` is the forward/back axis.
    Positive yaw is a left (counter-clockwise) turn.
    """

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def to_velocity_command(
        self, config: GaitConfig | JoystickLimits
    ) -> VelocityCommand:
        """Scale UI axes to the body-frame command used by the gait."""

        return VelocityCommand(
            vx=self.y * config.max_vx,
            # The UI's +x points right; the model body frame's +y points left.
            vy=-self.x * config.max_vy,
            yaw_rate=self.yaw * config.max_yaw_rate,
        )


class KeyboardJoystick:
    """Convert key-repeat events into a self-centering three-axis joystick.

    Terminals do not expose key-up events. Each key press therefore keeps its
    axis active briefly; normal OS key repeat refreshes that deadline while a
    key is held. If input stops, the axis safely returns to zero.
    """

    def __init__(self, *, release_timeout: float = 0.35) -> None:
        if release_timeout <= 0.0:
            raise ValueError("release_timeout must be positive")
        self.release_timeout = release_timeout
        self._values = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._deadlines = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def press(self, key: str, *, now: float | None = None) -> bool:
        """Apply one normalized key event; return whether it was recognized."""

        key = _normalize_key(key)
        if key == "space":
            self.clear()
            return True
        binding = _JOYSTICK_KEY_BINDINGS.get(key)
        if binding is None:
            return False
        timestamp = time.monotonic() if now is None else now
        axis, value = binding
        self._values[axis] = value
        self._deadlines[axis] = timestamp + self.release_timeout
        return True

    def clear(self) -> None:
        for axis in self._values:
            self._values[axis] = 0.0
            self._deadlines[axis] = 0.0

    def state(self, *, now: float | None = None) -> JoystickState:
        timestamp = time.monotonic() if now is None else now
        values = {
            axis: value if timestamp < self._deadlines[axis] else 0.0
            for axis, value in self._values.items()
        }
        return JoystickState(**values)


def _normalize_key(key: str) -> str:
    aliases = {
        "\x1b[a": "up",
        "\x1b[b": "down",
        "\x1b[d": "left",
        "\x1b[c": "right",
        "\x1boa": "up",
        "\x1bob": "down",
        "\x1bod": "left",
        "\x1boc": "right",
        "key_up": "up",
        "key_down": "down",
        "key_left": "left",
        "key_right": "right",
        " ": "space",
    }
    lowered = key.lower()
    return aliases.get(lowered, lowered)


def render_joystick_ui(
    state: JoystickState,
    command: VelocityCommand,
    *,
    profile_name: str,
    control_name: str = "tripod-gait",
    control_hint: str = "",
) -> str:
    """Render the current joystick position as a compact terminal dashboard."""

    grid = [[" " for _ in range(15)] for _ in range(5)]
    for column in range(15):
        grid[2][column] = "─"
    for row in range(5):
        grid[row][7] = "│"
    grid[2][7] = "┼"
    point_column = int(round(7 + state.x * 6))
    point_row = int(round(2 - state.y * 2))
    grid[point_row][point_column] = "●"

    # Positive yaw is a left turn, so it is drawn toward the bar's left label.
    yaw_column = int(round(6 - state.yaw * 6))
    yaw_bar = list("──────┼──────")
    yaw_bar[yaw_column] = "●"
    motion = "ACTIVE" if any((state.x, state.y, state.yaw)) else "NEUTRAL"
    grid_lines = [f"      │{''.join(row)}│" for row in grid]

    lines = [
        "SCONE velocity joystick",
        "W/S: forward/back   A/D: strafe   ←/→: yaw   SPACE: stop   Q: quit",
        "Keys self-center shortly after release.",
        "",
        "                 W / +Y",
        "      ┌───────────────┐",
        *grid_lines,
        "      └───────────────┘",
        "      A / -X       D / +X",
        "",
        f" yaw left  [{''.join(yaw_bar)}]  yaw right",
        f" stick     x={state.x:+.2f}  y={state.y:+.2f}  yaw={state.yaw:+.2f}",
        f" body cmd  vx={command.vx:+.3f} m/s  vy={command.vy:+.3f} m/s  "
        f"yaw={command.yaw_rate:+.3f} rad/s",
        f" control   {control_name:<10} profile={profile_name:<10} state={motion}",
    ]
    if control_hint:
        lines.append(f" note      {control_hint}")
    return "\n".join(lines)


class _JoystickTerminal:
    """Raw, non-blocking terminal reader with an ANSI alternate-screen UI."""

    def __init__(
        self,
        input_stream: TextIO = sys.stdin,
        output_stream: TextIO = sys.stdout,
    ) -> None:
        self.input_stream = input_stream
        self.output_stream = output_stream
        self._buffer = bytearray()
        self._attributes: list | None = None

    def __enter__(self) -> "_JoystickTerminal":
        import termios
        import tty

        if not self.input_stream.isatty() or not self.output_stream.isatty():
            raise RuntimeError("the joystick CLI requires an interactive terminal")
        descriptor = self.input_stream.fileno()
        self._attributes = termios.tcgetattr(descriptor)
        tty.setcbreak(descriptor)
        self.output_stream.write("\x1b[?1049h\x1b[?25l\x1b[2J\x1b[H")
        self.output_stream.flush()
        return self

    def __exit__(self, *_: object) -> None:
        import termios

        if self._attributes is not None:
            termios.tcsetattr(
                self.input_stream.fileno(), termios.TCSADRAIN, self._attributes
            )
        self.output_stream.write("\x1b[?25h\x1b[?1049l")
        self.output_stream.flush()

    def read_key(self, timeout: float) -> str | None:
        parsed = self._pop_key()
        if parsed is not None:
            return parsed
        readable, _, _ = select.select(
            [self.input_stream.fileno()], [], [], max(0.0, timeout)
        )
        if not readable:
            return None
        self._buffer.extend(os.read(self.input_stream.fileno(), 32))
        return self._pop_key()

    def _pop_key(self) -> str | None:
        if not self._buffer:
            return None
        for sequence, key in _TERMINAL_KEY_SEQUENCES.items():
            if self._buffer.startswith(sequence):
                del self._buffer[: len(sequence)]
                return key
        if self._buffer[0] == 0x1B and any(
            sequence.startswith(self._buffer)
            for sequence in _TERMINAL_KEY_SEQUENCES
        ):
            return None
        value = bytes((self._buffer.pop(0),)).decode("utf-8", errors="ignore")
        return _normalize_key(value)

    def draw(self, screen: str) -> None:
        self.output_stream.write(f"\x1b[H{screen}\x1b[J")
        self.output_stream.flush()


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


def run_velocity_joystick_cli(
    *,
    limits: GaitConfig | JoystickLimits,
    apply_command: Callable[[VelocityCommand, float], None],
    profile_name: str,
    control_name: str | Callable[[], str],
    control_hint: str = "",
    stop_event: threading.Event | None = None,
    handle_key: Callable[[str], bool] | None = None,
) -> None:
    """Read the common terminal joystick and publish body velocity commands."""

    joystick = KeyboardJoystick()
    period = 1.0 / 50.0
    pending_keys: list[str] = []
    last_frame = time.monotonic()
    stop = stop_event or threading.Event()

    with _JoystickTerminal() as terminal:
        try:
            while not stop.is_set():
                frame_started = time.monotonic()
                key = terminal.read_key(0.0)
                while key is not None:
                    pending_keys.append(key)
                    key = terminal.read_key(0.0)

                quit_requested = False
                for pending_key in pending_keys:
                    normalized = _normalize_key(pending_key)
                    if normalized == "q":
                        quit_requested = True
                        stop.set()
                        break
                    if handle_key is not None and handle_key(normalized):
                        # A mode/home command must not leave the previous
                        # velocity axis active when the new pose is entered.
                        joystick.clear()
                        continue
                    joystick.press(normalized, now=frame_started)
                pending_keys.clear()
                if quit_requested:
                    break

                state = joystick.state(now=frame_started)
                command = state.to_velocity_command(limits)
                dt = max(frame_started - last_frame, 1e-6)
                last_frame = frame_started
                apply_command(command, dt)
                displayed_control_name = (
                    control_name() if callable(control_name) else control_name
                )
                terminal.draw(
                    render_joystick_ui(
                        state,
                        command,
                        profile_name=profile_name,
                        control_name=displayed_control_name,
                        control_hint=control_hint,
                    )
                )

                remaining = period - (time.monotonic() - frame_started)
                key = terminal.read_key(max(0.0, remaining))
                if key is not None:
                    pending_keys.append(key)
        finally:
            # Send an explicit neutral frame before the caller shuts down the
            # robot/controller, even when the loop exits because of an error.
            joystick.clear()
            apply_command(VelocityCommand(), period)
            stop.set()


def _run_gait_joystick_cli(
    robot: SCONE,
    gait: TripodGait,
    *,
    control_name: str,
    stop_event: threading.Event | None = None,
    calibrate_from_controller: bool = True,
) -> None:
    """Drive one model-based gait from the shared x/y/yaw joystick."""

    if calibrate_from_controller:
        gait.reset_from_controller()
    run_velocity_joystick_cli(
        limits=gait.config,
        apply_command=lambda command, dt: gait.update(command, dt=dt, send=True),
        profile_name=robot.profile_name,
        control_name=control_name,
        stop_event=stop_event,
    )


def run_tripod_gait_joystick_cli(
    robot: SCONE,
    *,
    stop_event: threading.Event | None = None,
    gait_config: GaitConfig | None = None,
    calibrate_from_controller: bool = True,
) -> None:
    """Drive the classic alternating ``tripod-gait`` controller."""

    if robot.profile_name == "sport":
        print(
            "[SCONE] Sport는 차체가 매우 낮아 삼각 보행 중 발의 지면 여유가 "
            "상쇄될 수 있습니다. tripod-gait는 Standard 자세를 권장합니다."
        )
    gait = TripodGait(robot.controller, robot.profile, config=gait_config)
    _run_gait_joystick_cli(
        robot,
        gait,
        control_name="tripod-gait",
        stop_event=stop_event,
        calibrate_from_controller=calibrate_from_controller,
    )


def run_scone_gait_joystick_cli(
    robot: SCONE,
    *,
    stop_event: threading.Event | None = None,
    gait_config: SconeGaitConfig | None = None,
    calibrate_from_controller: bool = True,
) -> None:
    """Drive the experimental SCONE sector rolling/creep gait."""

    if robot.profile_name == "sport":
        print(
            "[SCONE] scone-gait는 부채꼴 말단의 접지 여유가 필요한 실험 모드라 "
            "Standard 자세를 권장합니다."
        )
    gait = SconeGait(robot.controller, robot.profile, config=gait_config)
    _run_gait_joystick_cli(
        robot,
        gait,
        control_name="scone-gait",
        stop_event=stop_event,
        calibrate_from_controller=calibrate_from_controller,
    )


# Compatibility name for code written before the gait names became explicit.
run_joystick_cli = run_tripod_gait_joystick_cli


def run_legacy_joystick_cli(
    robot: SCONE, *, stop_event: threading.Event | None = None
) -> None:
    """Drive and change legacy Walk/Drive/Climb modes from one joystick."""

    adapter = LegacyVelocityAdapter(robot)
    adapter.start()

    def handle_mode_key(key: str) -> bool:
        if key not in ("r", "h"):
            return False
        adapter.update(VelocityCommand())
        if key == "r":
            robot.change_mode()
        else:
            robot.home()
        return True

    try:
        run_velocity_joystick_cli(
            limits=JoystickLimits(),
            apply_command=lambda command, _dt: adapter.update(command),
            profile_name=robot.profile_name,
            control_name=lambda: f"old/{robot.mode_name}",
            control_hint=(
                "R: Walk→Drive→Climb, H: home; "
                "Walk=W/S+arrows, Drive/Climb=A/D (stairs: side-on)"
            ),
            stop_event=stop_event,
            handle_key=handle_mode_key,
        )
    finally:
        adapter.close()


def _inquirer() -> tuple[Any, Any]:
    try:
        from InquirerPy import inquirer
        from InquirerPy.base.control import Choice
    except ImportError as error:
        raise RuntimeError(
            "InquirerPy가 필요합니다. `python -m pip install -r requirements.txt` "
            "후 다시 실행하세요."
        ) from error
    return inquirer, Choice


def _select_profile() -> str:
    inquirer, Choice = _inquirer()
    return inquirer.select(
        message="동작 프로필을 선택하세요.",
        choices=[
            Choice(value="standard", name="Standard · 높은 보행 자세 (권장)"),
            Choice(value="sport", name="Sport · 낮은 차체 자세"),
        ],
        default="standard",
    ).execute()


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
    try:
        inquirer, Choice = _inquirer()
    except RuntimeError as error:
        print(f"[SCONE] {error}", file=sys.stderr)
        return 2

    print("[SCONE] searching for a physical controller...")
    probe = discover_hardware()
    while True:
        try:
            action = inquirer.select(
                message="SCONE 실행 메뉴",
                choices=[
                    Choice(value="simulation_demo", name="시뮬레이션 (자동 데모)"),
                    Choice(value="simulation_control", name="시뮬레이션 조종"),
                    Choice(value="hardware", name=_hardware_label(probe)),
                    Choice(value="rediscover", name="하드웨어 다시 탐색"),
                    Choice(value="rl", name="강화학습 관리"),
                    Choice(value="quit", name="종료"),
                ],
                default="simulation_control",
            ).execute()
        except (EOFError, KeyboardInterrupt):
            print("\n[SCONE] 런처를 종료합니다.")
            return 0
        try:
            if action == "simulation_demo":
                from .simulation.core.simulator_cli import (
                    select_stair_demo_strategy,
                    select_stair_terrain,
                )
                from .simulation.core.stair_demo import run_automatic_stair_demo

                run_automatic_stair_demo(
                    select_stair_demo_strategy(),
                    terrain=select_stair_terrain(),
                )
            elif action == "simulation_control":
                from .simulation.core.cli_bridge import run
                from .simulation.core.simulator_cli import (
                    select_rl_checkpoint,
                    select_simulation_control,
                    select_terrain,
                )

                control = select_simulation_control()
                if control.value in ("rl", "scone-gait"):
                    checkpoint = select_rl_checkpoint()
                    from .rl.inquiry import (
                        prompt_reference_motion,
                        prompt_standing_pose,
                    )

                    # Existing PPO checkpoints were trained with the original
                    # hardcoded sinusoidal reference. Keep replay aligned with
                    # that action meaning unless the user explicitly selects a
                    # checkpoint trained on a model-based gait reference.
                    rl_reference_motion = prompt_reference_motion(
                        default="hardcoded"
                    )
                    stance_name, rl_standing_pose = prompt_standing_pose()
                    print(
                        f"[RL] {control.value} 기준: {rl_reference_motion} / "
                        f"기본 자세: {stance_name}"
                    )
                else:
                    checkpoint = None
                    rl_standing_pose = None
                    rl_reference_motion = None
                profile = (
                    "sport"
                    if control.value in ("rl", "scone-gait")
                    else _select_profile()
                )
                run_arguments = dict(
                    profile=profile,
                    terrain=select_terrain(),
                    control=control,
                    checkpoint=checkpoint,
                )
                if rl_standing_pose is not None:
                    run_arguments["rl_standing_pose_degrees"] = rl_standing_pose
                if rl_reference_motion is not None:
                    run_arguments["rl_reference_motion"] = rl_reference_motion
                run(**run_arguments)
            elif action == "hardware":
                _run_hardware(probe, _select_profile())
            elif action == "rediscover":
                print("[SCONE] searching for a physical controller...")
                probe = discover_hardware()
            elif action == "rl":
                from .rl.inquiry import main as run_rl_launcher

                run_rl_launcher()
            elif action == "quit":
                return 0
        except (EOFError, KeyboardInterrupt):
            print("\n[SCONE] 런처를 종료합니다.")
            return 0
        except (FileNotFoundError, RuntimeError, ValueError) as error:
            print(f"\n[SCONE] {error}")


__all__ = [
    "JoystickState",
    "JoystickLimits",
    "KEY_BINDINGS",
    "KeyboardJoystick",
    "main",
    "render_joystick_ui",
    "run_control_cli",
    "run_joystick_cli",
    "run_legacy_joystick_cli",
    "run_scone_gait_joystick_cli",
    "run_tripod_gait_joystick_cli",
    "run_velocity_joystick_cli",
]
