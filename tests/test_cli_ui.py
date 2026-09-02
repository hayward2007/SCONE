from __future__ import annotations

import io
import unittest
from unittest.mock import patch

from src.cli import _JoystickTerminal
from src.cli_ui import clear_terminal, display_width, render_panel


class TerminalPanelTests(unittest.TestCase):
    def test_korean_panel_lines_have_the_same_terminal_width(self) -> None:
        panel = render_panel(
            "SCONE / 통합 제어 센터",
            (
                (
                    "[ SYSTEM STATUS ]",
                    "- 하드웨어: 감지되지 않음",
                    "- 언어: korea",
                ),
                (
                    "[ NAVIGATION ]",
                    "- 위/아래: 이동  - Enter: 선택  - Ctrl-C: 종료",
                ),
            ),
        )

        self.assertEqual({display_width(line) for line in panel.splitlines()}, {74})

    def test_clear_terminal_avoids_ansi_when_output_is_redirected(self) -> None:
        stream = io.StringIO()

        self.assertFalse(clear_terminal(stream))
        self.assertEqual(stream.getvalue(), "")

    def test_joystick_performs_a_periodic_full_clear(self) -> None:
        stream = io.StringIO()
        terminal = _JoystickTerminal(output_stream=stream)
        terminal._last_full_clear = 10.0

        with patch("src.cli.time.monotonic", return_value=11.1):
            terminal.draw("frame")

        self.assertTrue(stream.getvalue().startswith("\x1b[2J\x1b[Hframe"))


if __name__ == "__main__":
    unittest.main()
