"""Shared fixed-width terminal layout helpers for SCONE command surfaces."""

from __future__ import annotations

import re
import sys
import unicodedata
from collections.abc import Sequence
from typing import TextIO


UI_INNER_WIDTH = 72
_ANSI_ESCAPE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def display_width(value: str) -> int:
    """Return terminal columns used by ASCII, ANSI, and Korean text."""

    plain = _ANSI_ESCAPE.sub("", value)
    return sum(
        0
        if unicodedata.combining(character)
        else 2
        if unicodedata.east_asian_width(character) in ("W", "F")
        else 1
        for character in plain
    )


def _split_to_width(value: str, width: int) -> tuple[str, str]:
    """Split one string without cutting a full-width terminal character."""

    used = 0
    for index, character in enumerate(value):
        character_width = display_width(character)
        if used + character_width > width:
            return value[:index], value[index:]
        used += character_width
    return value, ""


def wrap_display(value: str, width: int) -> list[str]:
    """Wrap text to a display width while preserving Korean alignment."""

    if width <= 0:
        raise ValueError("width must be positive")
    if not value:
        return [""]

    lines: list[str] = []
    for source_line in value.splitlines() or [""]:
        remaining = source_line.rstrip()
        if not remaining.strip():
            lines.append("")
            continue
        while display_width(remaining) > width:
            prefix, suffix = _split_to_width(remaining, width)
            split_at = prefix.rfind(" ")
            if split_at > 0:
                suffix = remaining[split_at + 1 :]
                prefix = remaining[:split_at]
            lines.append(prefix.rstrip())
            remaining = suffix.lstrip()
        lines.append(remaining)
    return lines


def render_panel(
    title: str,
    sections: Sequence[Sequence[str]],
    *,
    inner_width: int = UI_INNER_WIDTH,
) -> str:
    """Render sections inside one ASCII panel with exactly equal line widths."""

    if inner_width < 20:
        raise ValueError("inner_width must be at least 20")

    border = f"+{'-' * inner_width}+"
    content_width = inner_width - 4

    def rows(value: str) -> list[str]:
        rendered: list[str] = []
        for line in wrap_display(value, content_width):
            padding = content_width - display_width(line)
            rendered.append(f"|  {line}{' ' * padding}  |")
        return rendered

    output = [border, *rows(title), border]
    for section in sections:
        for value in section:
            output.extend(rows(value))
        output.append(border)
    return "\n".join(output)


def clear_terminal(
    stream: TextIO | None = None,
    *,
    force: bool = False,
) -> bool:
    """Clear an interactive terminal; keep redirected logs free of ANSI codes."""

    target = sys.stdout if stream is None else stream
    is_tty = bool(getattr(target, "isatty", lambda: False)())
    if not force and not is_tty:
        return False
    target.write("\x1b[2J\x1b[H")
    target.flush()
    return True


def show_picker_screen(
    title: str,
    prompt: str,
    instruction: str,
    *,
    stream: TextIO | None = None,
) -> None:
    """Clear and draw a consistent header before an interactive picker."""

    target = sys.stdout if stream is None else stream
    clear_terminal(target)
    target.write(
        render_panel(
            title,
            (
                (
                    "[ SELECT ]",
                    f"- {prompt}",
                    f"- {instruction}",
                ),
            ),
        )
    )
    target.write("\n")
    target.flush()


__all__ = [
    "UI_INNER_WIDTH",
    "clear_terminal",
    "display_width",
    "render_panel",
    "show_picker_screen",
    "wrap_display",
]
