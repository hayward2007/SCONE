"""Support-polygon geometry for fault-tolerant standing and walking.

:func:`support_polygon_margin` in :mod:`.stair_geometry` answers the
quasi-static stair question: given three or more contacts *already ordered
around a convex polygon*, how far inside is the centre of mass.  A robot that
has lost a leg needs the same quantity under conditions that function rejects
by design -- unordered contacts, and fewer than three of them.

This module supplies the ordering and the degenerate cases, and defers to
``stair_geometry`` for the polygon interior so the two never disagree about
what "margin" means.

The sign convention is shared: positive is inside, zero is on an edge, and
negative is outside.  With two contacts the support region is a segment, whose
interior is empty, so the margin is the negated distance to that segment; with
one or none it is :data:`UNSUPPORTED_MARGIN`.  Those values are ordered
consistently with the polygon case, which is what lets a reward term or a gait
scheduler compare all of them on a single axis.
"""

from __future__ import annotations

from typing import Iterable

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .stair_geometry import support_polygon_margin


# Distance-like stand-in for "there is no support region at all".  Chosen well
# below any reachable negative margin (the robot is 0.32 m long) so a
# comparison never ranks a real polygon below a missing one.
UNSUPPORTED_MARGIN = -1.0

# Contacts closer together than this count as one point, so numeric jitter in a
# foot position cannot manufacture an edge.
_COINCIDENT_TOLERANCE = 1e-9


def convex_hull(points: ArrayLike) -> NDArray[np.float64]:
    """Return the convex hull of 2-D points, counter-clockwise, without repeats.

    Andrew's monotone chain.  Collinear points are dropped, so the result is
    always a strictly convex polygon, a segment, or a single point.
    """

    array = np.asarray(points, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 2:
        raise ValueError("points must have shape (n, 2)")
    if len(array) == 0:
        return array.reshape(0, 2)

    ordered = np.unique(np.round(array, 12), axis=0)
    ordered = ordered[np.lexsort((ordered[:, 1], ordered[:, 0]))]
    if len(ordered) <= 2:
        return ordered

    def _half(sequence: NDArray[np.float64]) -> list[NDArray[np.float64]]:
        chain: list[NDArray[np.float64]] = []
        for point in sequence:
            while len(chain) >= 2:
                first, second = chain[-2], chain[-1]
                cross = (second[0] - first[0]) * (point[1] - first[1]) - (
                    second[1] - first[1]
                ) * (point[0] - first[0])
                if cross > _COINCIDENT_TOLERANCE:
                    break
                chain.pop()
            chain.append(point)
        return chain

    lower = _half(ordered)
    upper = _half(ordered[::-1])
    hull = np.array(lower[:-1] + upper[:-1], dtype=np.float64)
    # Collinear input collapses to the two extreme points, which is the
    # correct hull -- returning the original points instead would claim a
    # polygon where there is only a segment.
    return hull if len(hull) >= 2 else ordered


def _segment_distance(
    point: NDArray[np.float64],
    start: NDArray[np.float64],
    end: NDArray[np.float64],
) -> float:
    edge = end - start
    length_squared = float(edge @ edge)
    if length_squared <= _COINCIDENT_TOLERANCE:
        return float(np.linalg.norm(point - start))
    travel = float(np.clip((point - start) @ edge / length_squared, 0.0, 1.0))
    return float(np.linalg.norm(point - (start + travel * edge)))


def stability_margin(
    center_of_mass_xy: ArrayLike,
    contacts_xy: Iterable[ArrayLike],
) -> float:
    """Signed distance from the CoM projection to the support boundary.

    Unlike :func:`.stair_geometry.support_polygon_margin` this accepts contacts
    in any order and never raises on a degenerate support set, because a
    walking robot passes through those states every stride.
    """

    point = np.asarray(center_of_mass_xy, dtype=np.float64)
    if point.shape != (2,):
        raise ValueError("center_of_mass_xy must be a 2-vector")
    contacts = np.asarray(tuple(contacts_xy), dtype=np.float64).reshape(-1, 2)

    hull = convex_hull(contacts)
    if len(hull) == 0:
        return UNSUPPORTED_MARGIN
    if len(hull) == 1:
        return max(UNSUPPORTED_MARGIN, -float(np.linalg.norm(point - hull[0])))
    if len(hull) == 2:
        return max(
            UNSUPPORTED_MARGIN,
            -_segment_distance(point, hull[0], hull[1]),
        )
    return float(support_polygon_margin(point, hull))


def polygon_area(points: ArrayLike) -> float:
    """Area of the convex hull of the given points; zero when degenerate."""

    hull = convex_hull(points)
    if len(hull) < 3:
        return 0.0
    return 0.5 * abs(
        float(
            np.sum(
                hull[:, 0] * np.roll(hull[:, 1], -1)
                - hull[:, 1] * np.roll(hull[:, 0], -1)
            )
        )
    )


__all__ = [
    "UNSUPPORTED_MARGIN",
    "convex_hull",
    "polygon_area",
    "stability_margin",
]
