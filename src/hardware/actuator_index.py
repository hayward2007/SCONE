"""Physical actuator IDs and useful SCONE actuator groups."""

from __future__ import annotations


class ActuatorIndex:
    ALL = tuple(range(1, 19))

    UPPER = tuple(range(1, 7))
    MIDDLE = tuple(range(7, 13))
    LOWER = tuple(range(13, 19))
    XM = MIDDLE + LOWER

    UPPER_RIGHT = (1, 3, 5)
    UPPER_LEFT = (2, 4, 6)
    MIDDLE_RIGHT = tuple(i + 6 for i in UPPER_RIGHT)
    MIDDLE_LEFT = tuple(i + 6 for i in UPPER_LEFT)
    LOWER_RIGHT = tuple(i + 12 for i in UPPER_RIGHT)
    LOWER_LEFT = tuple(i + 12 for i in UPPER_LEFT)

    # LEFT/RIGHT identifies the tripod phase, not one physical robot side.
    UPPER_DIAGONAL_RIGHT = (1, 4, 5)
    UPPER_DIAGONAL_LEFT = (2, 3, 6)
    MIDDLE_DIAGONAL_RIGHT = tuple(i + 6 for i in UPPER_DIAGONAL_RIGHT)
    MIDDLE_DIAGONAL_LEFT = tuple(i + 6 for i in UPPER_DIAGONAL_LEFT)
    LOWER_DIAGONAL_RIGHT = tuple(i + 12 for i in UPPER_DIAGONAL_RIGHT)
    LOWER_DIAGONAL_LEFT = tuple(i + 12 for i in UPPER_DIAGONAL_LEFT)

    @classmethod
    def for_leg(cls, leg: int) -> tuple[int, int, int]:
        """Return ``(upper, middle, lower)`` motor IDs for leg 1..6."""

        if leg not in cls.UPPER:
            raise ValueError(f"leg must be between 1 and 6, got {leg}")
        return leg, leg + 6, leg + 12


__all__ = ["ActuatorIndex"]
