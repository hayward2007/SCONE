from __future__ import annotations

import math
import unittest

from src.locomotion import (
    SCONE_V2_ARC_WHEEL,
    legged_wheel_opening_ratio,
    quasi_static_horizontal_push,
    quasi_static_pivot_torque,
    required_friction_coefficient,
    stair_slope,
    support_polygon_margin,
    wheel_edge_offset,
)


class StairGeometryTests(unittest.TestCase):
    def test_current_arc_dimensions_and_opening_chord(self) -> None:
        geometry = SCONE_V2_ARC_WHEEL

        self.assertAlmostEqual(geometry.inner_radius, 0.1125)
        self.assertAlmostEqual(geometry.outer_radius, 0.1225)
        self.assertAlmostEqual(geometry.width, 0.044)
        self.assertAlmostEqual(geometry.opening_degrees, 135.0)
        self.assertAlmostEqual(geometry.opening_chord, 0.22635, places=5)

    def test_riser_reach_reserves_nosing_and_clearance(self) -> None:
        geometry = SCONE_V2_ARC_WHEEL

        self.assertTrue(geometry.can_reach_riser(0.10, clearance=0.003))
        self.assertFalse(geometry.can_reach_riser(0.15, clearance=0.003))
        self.assertFalse(geometry.can_reach_riser(0.20, clearance=0.003))
        self.assertTrue(geometry.can_reach_riser(0.120))
        self.assertFalse(
            geometry.can_reach_riser(
                0.120,
                clearance=0.003,
            )
        )
        self.assertTrue(geometry.edge_in_radial_band(0.118))
        self.assertFalse(geometry.edge_in_radial_band(0.100))

    def test_sharp_edge_pivot_formulas(self) -> None:
        radius = 0.1225
        rise = 0.080
        load = 12.0
        offset = wheel_edge_offset(radius, rise)

        self.assertAlmostEqual(
            offset,
            math.sqrt(2.0 * radius * rise - rise * rise),
        )
        self.assertAlmostEqual(
            quasi_static_pivot_torque(load, radius, rise),
            load * offset,
        )
        self.assertAlmostEqual(
            quasi_static_horizontal_push(load, radius, rise),
            load * offset / (radius - rise),
        )
        self.assertGreater(wheel_edge_offset(radius, 0.20), 0.0)

    def test_friction_slope_and_opening_ratio_are_dimensionless_checks(self) -> None:
        self.assertAlmostEqual(required_friction_coefficient(4.0, 10.0), 0.4)
        self.assertAlmostEqual(stair_slope(0.1, 0.2), math.atan2(0.1, 0.2))
        self.assertGreater(
            legged_wheel_opening_ratio(SCONE_V2_ARC_WHEEL, 0.1, 0.2),
            1.0,
        )

    def test_support_polygon_margin_is_positive_inside_and_negative_outside(self) -> None:
        contacts = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))

        self.assertAlmostEqual(support_polygon_margin((0.5, 0.5), contacts), 0.5)
        self.assertLess(support_polygon_margin((1.2, 0.5), contacts), 0.0)


if __name__ == "__main__":
    unittest.main()
