"""The single interactive command surface for hardware and simulation."""

from __future__ import annotations

import argparse
import os
import select
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, TextIO

from .cli_i18n import Language, localize, parse_language_argument
from .cli_ui import clear_terminal, render_panel, show_picker_screen
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
    language: Language | str = Language.ENGLISH,
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
    active = any((state.x, state.y, state.yaw))
    motion = localize(
        language,
        "● ACTIVE" if active else "○ READY",
        "● 작동 중" if active else "○ 대기",
    )
    grid_lines = [f"       |{''.join(row)}|" for row in grid]
    controls = (
        "[ CONTROLS ]",
        localize(
            language,
            "- W/S: forward/back  - A/D: strafe  - Left/Right: yaw",
            "- W/S: 전진/후진  - A/D: 좌우 이동  - 왼쪽/오른쪽: 회전",
        ),
        localize(
            language,
            "- SPACE: neutral stop  - Q: quit  - released keys auto-center",
            "- SPACE: 즉시 중립 정지  - Q: 종료  - 키를 놓으면 자동 중앙 복귀",
        ),
    )
    motion_map = (
        "[ MOTION MAP ]",
        "              W / +Y",
        "       +---------------+",
        *grid_lines,
        "       +---------------+",
        "       A / -X       D / +X",
        localize(
            language,
            f"- yaw left  [{''.join(yaw_bar)}]  yaw right",
            f"- 좌회전    [{''.join(yaw_bar)}]  우회전",
        ),
    )
    telemetry = [
        "[ TELEMETRY ]",
        f"- stick     x={state.x:+.2f}  y={state.y:+.2f}  yaw={state.yaw:+.2f}",
        f"- body cmd  vx={command.vx:+.3f} m/s  vy={command.vy:+.3f} m/s  "
        f"yaw={command.yaw_rate:+.3f} rad/s",
        localize(
            language,
            f"- status    {motion}  - control={control_name}  - profile={profile_name}",
            f"- 상태      {motion}  - 제어={control_name}  - 프로필={profile_name}",
        ),
    ]
    if control_hint:
        telemetry.append(
            localize(
                language,
                f"- note      {control_hint}",
                f"- 안내      {control_hint}",
            )
        )
    return render_panel(
        localize(
            language,
            "SCONE / VELOCITY CONTROL",
            "SCONE / 속도 제어",
        ),
        (controls, motion_map, telemetry),
    )


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
        self._last_full_clear = 0.0
        self._full_clear_interval = 1.0

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
        self._last_full_clear = time.monotonic()
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
        now = time.monotonic()
        if now - self._last_full_clear >= self._full_clear_interval:
            prefix = "\x1b[2J\x1b[H"
            self._last_full_clear = now
        else:
            prefix = "\x1b[H"
        self.output_stream.write(f"{prefix}{screen}\x1b[J")
        self.output_stream.flush()


def _read_key() -> str:
    from getch import getch

    return getch().lower()


def print_remote_help(
    language: Language | str = Language.ENGLISH,
    *,
    mode_name: str = "-",
    profile_name: str = "-",
    notice: str = "",
) -> None:
    sections: list[tuple[str, ...]] = [
        (
            "[ CONTROLS ]",
            localize(
                language,
                "- W/S: forward/backward  - A/D: left/right",
                "- W/S: 전진/후진  - A/D: 좌/우",
            ),
            localize(
                language,
                "- R: Walk -> Drive -> Climb  - H: home",
                "- R: Walk -> Drive -> Climb  - H: 초기 자세",
            ),
            localize(
                language,
                "- ?: help  - Q: return to launcher",
                "- ?: 도움말  - Q: 런처로 돌아가기",
            ),
        ),
        (
            "[ STATUS ]",
            localize(
                language,
                f"- Mode: {mode_name}  - Profile: {profile_name}",
                f"- 모드: {mode_name}  - 프로필: {profile_name}",
            ),
            localize(
                language,
                f"- Last event: {notice or 'Ready for input'}",
                f"- 최근 상태: {notice or '입력 대기 중'}",
            ),
        ),
    ]
    print(
        render_panel(
            localize(
                language,
                "SCONE / HARDWARE CONTROL",
                "SCONE / 하드웨어 제어",
            ),
            sections,
        )
    )


def run_control_cli(
    robot: SCONE,
    *,
    key_reader: Callable[[], str] = _read_key,
    language: Language | str = Language.ENGLISH,
) -> None:
    """Interpret keys once, then dispatch the same API on either backend."""

    notice = ""
    while True:
        clear_terminal()
        print_remote_help(
            language,
            mode_name=robot.mode_name,
            profile_name=robot.profile_name,
            notice=notice,
        )
        key = key_reader().lower()
        if key == "q":
            return
        if key == "?":
            notice = localize(
                language,
                "Control help refreshed",
                "조작 도움말을 다시 표시했습니다",
            )
            continue
        command = KEY_BINDINGS.get(key)
        if command is None:
            notice = localize(
                language,
                f"Unknown key {key!r}; press ? for help",
                f"알 수 없는 키 {key!r}; ?를 눌러 도움말 확인",
            )
            continue
        try:
            robot.execute(command)
            notice = localize(
                language,
                f"Command={command.value} completed",
                f"명령={command.value} 실행 완료",
            )
        except UnsupportedCommandError as error:
            notice = str(error)


def run_velocity_joystick_cli(
    *,
    limits: GaitConfig | JoystickLimits,
    apply_command: Callable[[VelocityCommand, float], None],
    profile_name: str,
    control_name: str | Callable[[], str],
    control_hint: str = "",
    stop_event: threading.Event | None = None,
    handle_key: Callable[[str], bool] | None = None,
    language: Language | str = Language.ENGLISH,
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
                        language=language,
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
    language: Language | str = Language.ENGLISH,
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
        language=language,
    )


def run_tripod_gait_joystick_cli(
    robot: SCONE,
    *,
    stop_event: threading.Event | None = None,
    gait_config: GaitConfig | None = None,
    calibrate_from_controller: bool = True,
    language: Language | str = Language.ENGLISH,
) -> None:
    """Drive the classic alternating ``tripod-gait`` controller."""

    if robot.profile_name == "sport":
        print(localize(
            language,
            "[SCONE] Sport is very low and may remove swing clearance. "
            "Standard is recommended for tripod-gait.",
            "[SCONE] Sport는 차체가 매우 낮아 삼각 보행 중 발의 지면 여유가 "
            "상쇄될 수 있습니다. tripod-gait는 Standard 자세를 권장합니다.",
        ))
    gait = TripodGait(robot.controller, robot.profile, config=gait_config)
    _run_gait_joystick_cli(
        robot,
        gait,
        control_name="tripod-gait",
        stop_event=stop_event,
        calibrate_from_controller=calibrate_from_controller,
        language=language,
    )


def run_scone_gait_joystick_cli(
    robot: SCONE,
    *,
    stop_event: threading.Event | None = None,
    gait_config: SconeGaitConfig | None = None,
    calibrate_from_controller: bool = True,
    language: Language | str = Language.ENGLISH,
) -> None:
    """Drive the experimental SCONE sector rolling/creep gait."""

    if robot.profile_name == "sport":
        print(localize(
            language,
            "[SCONE] scone-gait is experimental and needs distal-frame "
            "clearance. Standard is recommended.",
            "[SCONE] scone-gait는 부채꼴 말단의 접지 여유가 필요한 실험 모드라 "
            "Standard 자세를 권장합니다.",
        ))
    gait = SconeGait(robot.controller, robot.profile, config=gait_config)
    _run_gait_joystick_cli(
        robot,
        gait,
        control_name="scone-gait",
        stop_event=stop_event,
        calibrate_from_controller=calibrate_from_controller,
        language=language,
    )


# Compatibility name for code written before the gait names became explicit.
run_joystick_cli = run_tripod_gait_joystick_cli


def run_legacy_joystick_cli(
    robot: SCONE,
    *,
    stop_event: threading.Event | None = None,
    language: Language | str = Language.ENGLISH,
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
                localize(
                    language,
                    "R: Walk→Drive→Climb, H: home; Walk=W/S+arrows, "
                    "Drive/Climb=A/D (stairs: side-on)",
                    "R: Walk→Drive→Climb, H: 초기 자세; Walk=W/S+방향키, "
                    "Drive/Climb=A/D (계단은 측면 접근)",
                )
            ),
            stop_event=stop_event,
            handle_key=handle_mode_key,
            language=language,
        )
    finally:
        adapter.close()


def _inquirer(
    language: Language | str = Language.ENGLISH,
) -> tuple[Any, Any]:
    try:
        from InquirerPy import inquirer
        from InquirerPy.base.control import Choice
    except ImportError as error:
        raise RuntimeError(
            localize(
                language,
                "InquirerPy is required. Run "
                "`python -m pip install -r requirements.txt` and try again.",
                "InquirerPy가 필요합니다. `python -m pip install -r requirements.txt` "
                "후 다시 실행하세요.",
            )
        ) from error
    return inquirer, Choice


def _select_profile(
    language: Language | str = Language.ENGLISH,
) -> str:
    inquirer, Choice = _inquirer(language)
    show_picker_screen(
        localize(language, "SCONE / POSTURE PROFILE", "SCONE / 동작 프로필"),
        localize(language, "Select a posture profile", "동작 프로필을 선택하세요"),
        localize(
            language,
            "Use Up/Down, then press Enter",
            "위/아래로 이동한 뒤 Enter를 누르세요",
        ),
    )
    return inquirer.select(
        message=localize(language, "Select a posture profile", "동작 프로필을 선택하세요"),
        choices=[
            Choice(
                value="standard",
                name=localize(
                    language,
                    "- Standard / high-clearance walking (recommended)",
                    "- Standard / 높은 보행 자세 (권장)",
                ),
            ),
            Choice(
                value="sport",
                name=localize(
                    language,
                    "- Sport / low body posture",
                    "- Sport / 낮은 차체 자세",
                ),
            ),
        ],
        default="standard",
        pointer="❯",
    ).execute()


def _hardware_label(
    probe: HardwareProbe,
    language: Language | str = Language.ENGLISH,
) -> str:
    if probe.available:
        return localize(
            language,
            f"Hardware control · connected ({probe.device_name})",
            f"하드웨어 조종 · 연결됨 ({probe.device_name})",
        )
    return localize(
        language,
        "Hardware control · not detected",
        "하드웨어 조종 · 장치 없음",
    )


def _run_hardware(
    probe: HardwareProbe,
    profile: str,
    *,
    language: Language | str = Language.ENGLISH,
) -> None:
    if not probe.available or probe.device_name is None:
        print(localize(
            language,
            f"[SCONE] Hardware control is unavailable: {probe.detail}",
            f"[SCONE] 현재 하드웨어를 조종할 수 없습니다: {probe.detail}",
        ))
        return
    from .hardware import Controller

    robot = SCONE(Controller(probe.device_name), profile=profile)
    try:
        robot.initialize()
        run_control_cli(robot, language=language)
    finally:
        robot.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SCONE hardware, simulation, and RL launcher",
    )
    parser.add_argument(
        "--language",
        type=parse_language_argument,
        default=Language.ENGLISH,
        metavar="{english,korea}",
        help="terminal UI language (default: english)",
    )
    return parser


def _launcher_header(
    probe: HardwareProbe,
    language: Language | str,
    *,
    notice: str = "",
) -> str:
    connected = probe.available and probe.device_name is not None
    hardware = (
        localize(language, f"connected · {probe.device_name}", f"연결됨 · {probe.device_name}")
        if connected
        else localize(language, "not detected", "감지되지 않음")
    )
    return render_panel(
        localize(language, "SCONE / CONTROL CENTER", "SCONE / 통합 제어 센터"),
        (
            (
                "[ SYSTEM STATUS ]",
                f"- {localize(language, 'Hardware', '하드웨어')}: {hardware}",
                f"- {localize(language, 'Language', '언어')}: "
                f"{Language.parse(language).value}",
                localize(
                    language,
                    "- Safety: discovery only; no motion command sent",
                    "- 안전 상태: 장치 탐색만 수행, 동작 명령은 전송하지 않음",
                ),
            ),
            (
                "[ NAVIGATION ]",
                localize(
                    language,
                    "- Up/Down: move  - Enter: select  - Ctrl-C: quit",
                    "- 위/아래: 이동  - Enter: 선택  - Ctrl-C: 종료",
                ),
                localize(
                    language,
                    f"- Notice: {notice or 'Ready'}",
                    f"- 알림: {notice or '준비 완료'}",
                ),
            ),
        ),
    )


def main(argv: list[str] | tuple[str, ...] | None = None) -> int:
    # Programmatic callers keep the historical no-argument behavior. SCONE.py
    # explicitly forwards sys.argv so command-line flags are still parsed.
    args = build_parser().parse_args([] if argv is None else list(argv))
    language = args.language
    try:
        inquirer, Choice = _inquirer(language)
    except RuntimeError as error:
        print(f"[SCONE] {error}", file=sys.stderr)
        return 2

    clear_terminal()
    print(
        render_panel(
            localize(language, "SCONE / STARTING", "SCONE / 시작 중"),
            ((
                localize(
                    language,
                    "- Scanning for a physical controller",
                    "- 실제 컨트롤러 검색 중",
                ),
                localize(
                    language,
                    "- Safety lock: no motion commands are being sent",
                    "- 안전 잠금: 동작 명령은 전송하지 않음",
                ),
            ),),
        )
    )
    probe = discover_hardware()
    notice = localize(
        language,
        "Hardware scan completed",
        "하드웨어 검색 완료",
    )
    while True:
        try:
            clear_terminal()
            print(_launcher_header(probe, language, notice=notice))
            notice = ""
            action = inquirer.select(
                message=localize(language, "Choose an activity", "실행할 작업을 선택하세요"),
                choices=[
                    Choice(
                        value="simulation_control",
                        name=localize(
                            language,
                            "- Interactive simulation / terminal joystick",
                            "- 시뮬레이션 조종 / 터미널 조이스틱 직접 제어",
                        ),
                    ),
                    Choice(
                        value="simulation_demo",
                        name=localize(
                            language,
                            "- Automatic stair demo / no manual control",
                            "- 자동 계단 데모 / 조종 없이 비교 실행",
                        ),
                    ),
                    Choice(
                        value="hardware",
                        name=f"- {_hardware_label(probe, language)}",
                    ),
                    Choice(
                        value="rl",
                        name=localize(
                            language,
                            "- Reinforcement learning / train, inspect, replay",
                            "- 강화학습 관리 / 학습, 상태 확인, 재생",
                        ),
                    ),
                    Choice(
                        value="rediscover",
                        name=localize(language, "- Rescan hardware", "- 하드웨어 다시 탐색"),
                    ),
                    Choice(value="quit", name=localize(language, "- Quit", "- 종료")),
                ],
                default="simulation_control",
                pointer="❯",
                instruction=localize(
                    language,
                    "(↑/↓ move, Enter select, Ctrl-C quit)",
                    "(↑/↓ 이동, Enter 선택, Ctrl-C 종료)",
                ),
            ).execute()
        except (EOFError, KeyboardInterrupt):
            print(localize(language, "\n[SCONE] Launcher closed.", "\n[SCONE] 런처를 종료합니다."))
            return 0
        try:
            if action == "simulation_demo":
                from .simulation.core.simulator_cli import (
                    select_stair_demo_strategy,
                    select_stair_terrain,
                )
                from .simulation.core.stair_demo import run_automatic_stair_demo

                run_automatic_stair_demo(
                    select_stair_demo_strategy(language=language),
                    terrain=select_stair_terrain(language=language),
                    language=language,
                )
                notice = localize(language, "Stair demo closed", "계단 데모 종료")
            elif action == "simulation_control":
                from .simulation.core.cli_bridge import run
                from .simulation.core.simulator_cli import (
                    select_rl_checkpoint,
                    select_simulation_control,
                    select_terrain,
                )

                control = select_simulation_control(language=language)
                if control.value in ("rl", "scone-gait"):
                    checkpoint = select_rl_checkpoint(language=language)
                    from .rl.inquiry import (
                        prompt_reference_motion,
                        prompt_standing_pose,
                    )

                    # Existing PPO checkpoints were trained with the original
                    # hardcoded sinusoidal reference. Keep replay aligned with
                    # that action meaning unless the user explicitly selects a
                    # checkpoint trained on a model-based gait reference.
                    rl_reference_motion = prompt_reference_motion(
                        default="hardcoded",
                        language=language,
                    )
                    stance_name, rl_standing_pose = prompt_standing_pose(
                        language=language
                    )
                    print(localize(
                        language,
                        f"[RL] {control.value} · reference={rl_reference_motion} · "
                        f"stance={stance_name}",
                        f"[RL] {control.value} · 기준={rl_reference_motion} · "
                        f"기본 자세={stance_name}",
                    ))
                else:
                    checkpoint = None
                    rl_standing_pose = None
                    rl_reference_motion = None
                profile = (
                    "sport"
                    if control.value in ("rl", "scone-gait")
                    else _select_profile(language)
                )
                run_arguments = dict(
                    profile=profile,
                    terrain=select_terrain(language=language),
                    control=control,
                    checkpoint=checkpoint,
                    language=language,
                )
                if rl_standing_pose is not None:
                    run_arguments["rl_standing_pose_degrees"] = rl_standing_pose
                if rl_reference_motion is not None:
                    run_arguments["rl_reference_motion"] = rl_reference_motion
                run(**run_arguments)
                notice = localize(language, "Simulation closed", "시뮬레이션 종료")
            elif action == "hardware":
                _run_hardware(
                    probe,
                    _select_profile(language),
                    language=language,
                )
                notice = localize(language, "Hardware control closed", "하드웨어 조종 종료")
            elif action == "rediscover":
                probe = discover_hardware()
                notice = localize(
                    language,
                    "Hardware scan completed",
                    "하드웨어 검색 완료",
                )
            elif action == "rl":
                from .rl.inquiry import main as run_rl_launcher

                run_rl_launcher(language=language)
                notice = localize(language, "RL manager closed", "강화학습 관리 종료")
            elif action == "quit":
                return 0
        except (EOFError, KeyboardInterrupt):
            print(localize(language, "\n[SCONE] Launcher closed.", "\n[SCONE] 런처를 종료합니다."))
            return 0
        except (FileNotFoundError, RuntimeError, ValueError) as error:
            notice = localize(
                language,
                f"Error: {error}",
                f"오류: {error}",
            )


__all__ = [
    "Language",
    "JoystickState",
    "JoystickLimits",
    "KEY_BINDINGS",
    "KeyboardJoystick",
    "build_parser",
    "main",
    "render_joystick_ui",
    "run_control_cli",
    "run_joystick_cli",
    "run_legacy_joystick_cli",
    "run_scone_gait_joystick_cli",
    "run_tripod_gait_joystick_cli",
    "run_velocity_joystick_cli",
]
