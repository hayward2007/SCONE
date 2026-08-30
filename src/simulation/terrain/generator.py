"""Procedural MJCF terrain generation using MuJoCo primitive geoms."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

from .presets import SLOPE_PRESETS, STAIR_PRESETS
from .types import SlopeProfile, StairProfile, TerrainBuildResult, TerrainType


_CONTACT_ATTRIBUTES = {
    "contype": "1",
    "conaffinity": "1",
    "condim": "6",
    "friction": "1.0 0.005 0.0005",
    "solref": "0.02 1",
    "solimp": "0.85 0.95 0.002 0.5 2",
    # MuJoCo's interactive viewer hides geom groups 3..5 by default. Terrain
    # is primary scene geometry, so keep it in the always-visible group 0.
    "group": "0",
}


def _numbers(*values: float) -> str:
    return " ".join(f"{value:.9g}" for value in values)


class TerrainGenerator:
    """Append named terrain geoms to an MJCF ``worldbody`` element.

    SCONE's visible body +X axis points along world +Y in the exported MJCF,
    so courses advance along +Y and are centred near world X=0.
    """

    def __init__(
        self,
        worldbody: ET.Element,
        *,
        floor_z: float,
        center_x: float = 0.0,
        start_y: float = 0.35,
        seed: int = 7,
    ) -> None:
        self.worldbody = worldbody
        self.floor_z = float(floor_z)
        self.center_x = float(center_x)
        self.start_y = float(start_y)
        self.cursor_y = float(start_y)
        self.rng = np.random.default_rng(seed)
        self.geom_names: list[str] = []
        self.max_height = 0.0

    def _box(
        self,
        name: str,
        *,
        pos: tuple[float, float, float],
        size: tuple[float, float, float],
        rgba: tuple[float, float, float, float],
        euler: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> ET.Element:
        attributes = {
            "name": name,
            "type": "box",
            "pos": _numbers(*pos),
            "size": _numbers(*size),
            "euler": _numbers(*euler),
            "rgba": _numbers(*rgba),
            **_CONTACT_ATTRIBUTES,
        }
        geom = ET.SubElement(self.worldbody, "geom", attributes)
        self.geom_names.append(name)
        return geom

    def add_gap(self, length: float = 0.35) -> None:
        self.cursor_y += length

    def add_uneven(
        self,
        *,
        prefix: str = "terrain_uneven",
        length: float = 1.80,
        width: float = 1.20,
        tile_size: float = 0.20,
        min_height: float = 0.008,
        max_height: float = 0.060,
        max_tilt_degrees: float = 4.0,
    ) -> None:
        """Create a deterministic tiled rough patch with height and tilt noise."""

        rows = max(1, math.ceil(length / tile_size))
        columns = max(1, math.ceil(width / tile_size))
        actual_length = rows * tile_size
        actual_width = columns * tile_size
        gap = min(0.008, tile_size * 0.08)
        tilt = math.radians(max_tilt_degrees)

        for row in range(rows):
            for column in range(columns):
                height = float(self.rng.uniform(min_height, max_height))
                roll, pitch = self.rng.uniform(-tilt, tilt, size=2)
                x = self.center_x - actual_width / 2.0 + (column + 0.5) * tile_size
                y = self.cursor_y + (row + 0.5) * tile_size
                self._box(
                    f"{prefix}_r{row:02d}_c{column:02d}",
                    pos=(x, y, self.floor_z + height / 2.0),
                    size=(
                        (tile_size - gap) / 2.0,
                        (tile_size - gap) / 2.0,
                        height / 2.0,
                    ),
                    euler=(float(roll), float(pitch), 0.0),
                    rgba=(0.35, 0.28 + 0.8 * height, 0.16, 1.0),
                )
                self.max_height = max(self.max_height, height)
        self.cursor_y += actual_length

    def add_stairs(
        self,
        profile: StairProfile,
        *,
        prefix: str,
        return_to_floor: bool = False,
    ) -> None:
        """Generate variable-dimension stairs from a per-step profile."""

        height = 0.0
        for index, (rise, tread, width) in enumerate(
            zip(profile.rises, profile.tread_depths, profile.widths),
            start=1,
        ):
            height += rise
            self._box(
                f"{prefix}_up_{index:02d}",
                pos=(self.center_x, self.cursor_y + tread / 2.0, self.floor_z + height / 2.0),
                size=(width / 2.0, tread / 2.0, height / 2.0),
                rgba=(0.34 + index * 0.06, 0.38 + index * 0.04, 0.43, 1.0),
            )
            self.cursor_y += tread
            self.max_height = max(self.max_height, height)

        landing_width = max(profile.widths)
        self._box(
            f"{prefix}_landing",
            pos=(
                self.center_x,
                self.cursor_y + profile.landing_length / 2.0,
                self.floor_z + height / 2.0,
            ),
            size=(landing_width / 2.0, profile.landing_length / 2.0, height / 2.0),
            rgba=(0.48, 0.52, 0.56, 1.0),
        )
        self.cursor_y += profile.landing_length

        if not return_to_floor:
            return

        down_height = height
        reversed_steps = list(
            zip(profile.rises, profile.tread_depths, profile.widths)
        )[::-1]
        for index, (rise, tread, width) in enumerate(reversed_steps, start=1):
            down_height -= rise
            if down_height <= 1e-9:
                self.cursor_y += tread
                continue
            self._box(
                f"{prefix}_down_{index:02d}",
                pos=(
                    self.center_x,
                    self.cursor_y + tread / 2.0,
                    self.floor_z + down_height / 2.0,
                ),
                size=(width / 2.0, tread / 2.0, down_height / 2.0),
                rgba=(0.42, 0.46, 0.50, 1.0),
            )
            self.cursor_y += tread

    def _ramp(
        self,
        *,
        prefix: str,
        start_surface_z: float,
        angle_radians: float,
        profile: SlopeProfile,
    ) -> float:
        length = profile.length
        thickness = profile.thickness
        center_surface_z = start_surface_z + 0.5 * length * math.sin(angle_radians)
        center_z = center_surface_z - 0.5 * thickness * math.cos(angle_radians)
        self._box(
            prefix,
            pos=(self.center_x, self.cursor_y + length / 2.0, center_z),
            size=(profile.width / 2.0, length / 2.0, thickness / 2.0),
            euler=(angle_radians, 0.0, 0.0),
            rgba=(0.30, 0.42, 0.31, 1.0),
        )
        self.cursor_y += length
        return start_surface_z + length * math.sin(angle_radians)

    def add_slope(
        self,
        profile: SlopeProfile,
        *,
        prefix: str,
        return_to_floor: bool = False,
    ) -> None:
        """Generate an incline, top landing, and optional matching decline."""

        angle = math.radians(profile.angle_degrees)
        top_z = self._ramp(
            prefix=f"{prefix}_up",
            start_surface_z=self.floor_z,
            angle_radians=angle,
            profile=profile,
        )
        height = top_z - self.floor_z
        self.max_height = max(self.max_height, height)
        self._box(
            f"{prefix}_landing",
            pos=(
                self.center_x,
                self.cursor_y + profile.landing_length / 2.0,
                self.floor_z + height - profile.thickness / 2.0,
            ),
            size=(
                profile.width / 2.0,
                profile.landing_length / 2.0,
                profile.thickness / 2.0,
            ),
            rgba=(0.32, 0.46, 0.34, 1.0),
        )
        self.cursor_y += profile.landing_length
        if return_to_floor:
            self._ramp(
                prefix=f"{prefix}_down",
                start_surface_z=top_z,
                angle_radians=-angle,
                profile=profile,
            )

    def build(self, terrain: TerrainType | str) -> TerrainBuildResult:
        selected = TerrainType.parse(terrain)
        if selected is TerrainType.FLAT:
            pass
        elif selected is TerrainType.UNEVEN:
            self.add_uneven()
        elif selected in STAIR_PRESETS:
            self.add_stairs(
                STAIR_PRESETS[selected],
                prefix=f"terrain_{selected.value.replace('-', '_')}",
            )
        elif selected in SLOPE_PRESETS:
            self.add_slope(
                SLOPE_PRESETS[selected],
                prefix=f"terrain_{selected.value.replace('-', '_')}",
            )
        elif selected is TerrainType.MIXED:
            self._build_mixed()
        else:  # pragma: no cover - TerrainType makes this unreachable.
            raise AssertionError(selected)
        return TerrainBuildResult(
            terrain=selected,
            geom_names=tuple(self.geom_names),
            start_y=self.start_y,
            end_y=self.cursor_y,
            max_height=self.max_height,
        )

    def _build_mixed(self) -> None:
        """Build one traversable course containing every requested terrain."""

        self.add_uneven(prefix="terrain_mixed_uneven", length=1.60)
        self.add_gap()
        order = (
            TerrainType.STAIRS_1,
            TerrainType.SLOPE_1,
            TerrainType.STAIRS_2,
            TerrainType.SLOPE_2,
            TerrainType.STAIRS_3,
            TerrainType.SLOPE_3,
        )
        for selected in order:
            prefix = f"terrain_mixed_{selected.value.replace('-', '_')}"
            if selected in STAIR_PRESETS:
                self.add_stairs(
                    STAIR_PRESETS[selected],
                    prefix=prefix,
                    return_to_floor=True,
                )
            else:
                self.add_slope(
                    SLOPE_PRESETS[selected],
                    prefix=prefix,
                    return_to_floor=True,
                )
            self.add_gap()


def add_terrain(
    worldbody: ET.Element,
    terrain: TerrainType | str,
    *,
    floor_z: float,
    seed: int = 7,
    center_x: float = 0.0,
    start_y: float = 0.35,
) -> TerrainBuildResult:
    """Append a terrain preset to ``worldbody`` and return build metadata."""

    return TerrainGenerator(
        worldbody,
        floor_z=floor_z,
        center_x=center_x,
        start_y=start_y,
        seed=seed,
    ).build(terrain)


__all__ = ["TerrainGenerator", "add_terrain"]
