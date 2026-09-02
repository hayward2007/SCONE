"""Small, dependency-free localization helpers for SCONE terminal UIs."""

from __future__ import annotations

import argparse
from enum import Enum
from typing import Any


class Language(str, Enum):
    """Languages exposed by the public ``--language`` option.

    ``korea`` is kept as the canonical CLI value because it is the spelling
    requested by the project. Common short aliases are accepted as input.
    """

    ENGLISH = "english"
    KOREA = "korea"

    @classmethod
    def parse(cls, value: "Language | str") -> "Language":
        if isinstance(value, cls):
            return value
        normalized = str(value).strip().lower().replace("_", "-")
        aliases = {
            "en": cls.ENGLISH,
            "eng": cls.ENGLISH,
            "english": cls.ENGLISH,
            "ko": cls.KOREA,
            "kor": cls.KOREA,
            "korea": cls.KOREA,
            "korean": cls.KOREA,
            "한국어": cls.KOREA,
        }
        try:
            return aliases[normalized]
        except KeyError as error:
            raise ValueError(
                f"unknown language {value!r}; choose english or korea"
            ) from error


def localize(
    language: Language | str,
    english: str,
    korean: str,
) -> str:
    """Return one of two source-controlled UI strings."""

    return korean if Language.parse(language) is Language.KOREA else english


def parse_language_argument(value: str) -> Language:
    """Argparse adapter with a concise user-facing validation error."""

    try:
        return Language.parse(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "choose 'english' or 'korea'"
        ) from error


_ENGLISH_TERRAIN_LABELS = {
    "flat": "Flat ground",
    "uneven": "Uneven terrain",
    "stairs-1": "Stairs level 1 · 10 cm",
    "stairs-2": "Stairs level 2 · 15 cm",
    "stairs-3": "Stairs level 3 · 20 cm",
    "slope-1": "Slope level 1 · 8°",
    "slope-2": "Slope level 2 · 15°",
    "slope-3": "Slope level 3 · 25°",
    "mixed": "Mixed course",
}


def terrain_label(
    terrain: Any,
    language: Language | str,
    *,
    korean_label: str,
) -> str:
    """Localize a terrain label without importing the simulation package."""

    value = getattr(terrain, "value", str(terrain))
    return localize(
        language,
        _ENGLISH_TERRAIN_LABELS.get(value, value),
        korean_label,
    )


__all__ = [
    "Language",
    "localize",
    "parse_language_argument",
    "terrain_label",
]
