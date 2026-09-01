"""Geometry and quasi-static checks for SCONE's sector wheel on stairs.

The helpers in this module are analysis tools, not a physical safety
certificate.  They deliberately expose the assumptions behind each result so
that CAD dimensions, tread geometry, friction, and measured actuator limits
can replace the present simulation values without rewriting the equations.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np
from numpy.typing import ArrayLike, NDArray


Vector2 = NDArray[np.float64]


@dataclass(frozen=True)
class ArcWheelGeometry:
    """Nominal SCONEv2 TPU sector dimensions in metres and degrees.

    The radii and width are dimensioned in the archived Fusion drawing and
    agree with ``TIRE.stl``.  ``occupied_arc_degrees`` comes from the current
    MuJoCo collision mesh: its complementary opening is about 135 degrees.
    """

    inner_radius: float = 0.1125
    outer_radius: float = 0.1225
    width: float = 0.044
    occupied_arc_degrees: float = 225.0

    def __post_init__(self) -> None:
        if not 0.0 < self.inner_radius < self.outer_radius:
            raise ValueError("arc radii must satisfy 0 < inner < outer")
        if self.width <= 0.0:
            raise ValueError("arc width must be positive")
        if not 0.0 < self.occupied_arc_degrees < 360.0:
            raise ValueError("occupied arc must be between 0 and 360 degrees")

    @property
    def opening_degrees(self) -> float:
        return 360.0 - self.occupied_arc_degrees

    @property
    def opening_chord(self) -> float:
        """Chord between the two outer-radius ends of the open sector."""

        return 2.0 * self.outer_radius * math.sin(
            math.radians(self.opening_degrees) / 2.0
        )

    def edge_in_radial_band(
        self,
        edge_distance: float,
        *,
        tolerance: float = 0.0,
    ) -> bool:
        """Whether a stair edge can intersect the annular TPU contact band."""

        if tolerance < 0.0:
            raise ValueError("tolerance cannot be negative")
        return (
            self.inner_radius - tolerance
            <= edge_distance
            <= self.outer_radius + tolerance
        )

    def conservative_riser_limit(
        self,
        *,
        nosing_radius: float = 0.0,
        clearance: float = 0.0,
    ) -> float:
        """Flat-approach rise limit after nosing and clearance allowances."""

        if min(nosing_radius, clearance) < 0.0:
            raise ValueError("nosing radius and clearance cannot be negative")
        return max(0.0, self.outer_radius - nosing_radius - clearance)

    def can_reach_riser(
        self,
        rise: float,
        *,
        nosing_radius: float = 0.0,
        clearance: float = 0.0,
    ) -> bool:
        return 0.0 < rise <= self.conservative_riser_limit(
            nosing_radius=nosing_radius,
            clearance=clearance,
        )


SCONE_V2_ARC_WHEEL = ArcWheelGeometry()


def wheel_edge_offset(radius: float, rise: float) -> float:
    """Horizontal centre-to-edge offset while pivoting over a sharp riser.

    With the wheel centre initially one radius above the lower tread, the
    vertical edge-to-centre distance is ``radius - rise``.  Pythagoras gives
    ``x = sqrt(radius**2 - (radius - rise)**2)``.
    """

    if radius <= 0.0 or not 0.0 <= rise <= 2.0 * radius:
        raise ValueError("rise must be in [0, 2 * radius]")
    return math.sqrt(max(0.0, 2.0 * radius * rise - rise * rise))


def quasi_static_pivot_torque(
    load_force: float,
    radius: float,
    rise: float,
    *,
    safety_factor: float = 1.0,
    efficiency: float = 1.0,
) -> float:
    """Minimum ideal wheel torque for gravity moment about a sharp edge.

    ``load_force`` is the normal gravitational load assigned to this hooked
    sector, not automatically the whole robot weight.  Compliance, impact,
    acceleration, rounded nosings, and load imbalance require an additional
    experimentally justified safety factor.
    """

    if load_force < 0.0:
        raise ValueError("load_force cannot be negative")
    if safety_factor < 1.0:
        raise ValueError("safety_factor must be at least one")
    if not 0.0 < efficiency <= 1.0:
        raise ValueError("efficiency must be in (0, 1]")
    return (
        safety_factor
        * load_force
        * wheel_edge_offset(radius, rise)
        / efficiency
    )


def quasi_static_horizontal_push(
    load_force: float,
    radius: float,
    rise: float,
    *,
    safety_factor: float = 1.0,
) -> float:
    """Horizontal axle force whose edge moment balances gravity."""

    if load_force < 0.0 or safety_factor < 1.0:
        raise ValueError("invalid load or safety factor")
    vertical_offset = radius - rise
    if vertical_offset <= 0.0:
        return math.inf
    return (
        safety_factor
        * load_force
        * wheel_edge_offset(radius, rise)
        / vertical_offset
    )


def required_friction_coefficient(
    tangential_force: float,
    normal_force: float,
) -> float:
    """Coulomb requirement ``mu_required = |F_t| / F_n``."""

    if normal_force <= 0.0:
        return math.inf
    return abs(tangential_force) / normal_force


def stair_slope(rise: float, tread: float) -> float:
    """Return the equivalent stair flight slope in radians."""

    if rise <= 0.0 or tread <= 0.0:
        raise ValueError("rise and tread must be positive")
    return math.atan2(rise, tread)


def legged_wheel_opening_ratio(
    geometry: ArcWheelGeometry,
    rise: float,
    tread: float,
) -> float:
    """Opening chord divided by one step's rise/tread diagonal.

    This dimensionless comparison is useful for cross-checking legged-wheel
    literature, but it is not a SCONE success criterion by itself because
    SCONE has one continuous C-sector per leg rather than an N-spoke wheel.
    """

    if rise <= 0.0 or tread <= 0.0:
        raise ValueError("rise and tread must be positive")
    return geometry.opening_chord / math.hypot(rise, tread)


def support_polygon_margin(
    center_of_mass_xy: ArrayLike,
    contacts_xy: Iterable[ArrayLike],
) -> float:
    """Signed minimum distance from CoM projection to a convex support hull.

    Positive means strictly inside, zero lies on an edge, and negative is
    outside.  Contacts must already be ordered around a convex polygon; the
    function accepts clockwise or counter-clockwise order.
    """

    point = np.asarray(center_of_mass_xy, dtype=np.float64)
    polygon = np.asarray(tuple(contacts_xy), dtype=np.float64)
    if point.shape != (2,) or polygon.ndim != 2 or polygon.shape[1] != 2:
        raise ValueError("point and contacts must be two-dimensional")
    if len(polygon) < 3:
        raise ValueError("at least three contacts are required")
    signed_area_twice = float(
        np.sum(
            polygon[:, 0] * np.roll(polygon[:, 1], -1)
            - polygon[:, 1] * np.roll(polygon[:, 0], -1)
        )
    )
    if abs(signed_area_twice) <= 1e-12:
        raise ValueError("support polygon is degenerate")
    orientation = 1.0 if signed_area_twice > 0.0 else -1.0
    margins = []
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge = end - start
        length = float(np.linalg.norm(edge))
        if length <= 1e-12:
            raise ValueError("support polygon has a zero-length edge")
        cross = edge[0] * (point[1] - start[1]) - edge[1] * (
            point[0] - start[0]
        )
        margins.append(orientation * float(cross) / length)
    return min(margins)


__all__ = [
    "ArcWheelGeometry",
    "SCONE_V2_ARC_WHEEL",
    "legged_wheel_opening_ratio",
    "quasi_static_horizontal_push",
    "quasi_static_pivot_torque",
    "required_friction_coefficient",
    "stair_slope",
    "support_polygon_margin",
    "wheel_edge_offset",
]
