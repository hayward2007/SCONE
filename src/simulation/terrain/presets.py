"""Named SCONE terrain difficulty presets in SI units."""

from __future__ import annotations

from .types import SlopeProfile, StairProfile, TerrainType


# Every physical step intentionally has a different rise, tread depth, and
# width. The three presets then increase overall obstacle difficulty.
STAIR_PRESETS: dict[TerrainType, StairProfile] = {
    TerrainType.STAIRS_1: StairProfile(
        rises=(0.035, 0.045, 0.055),
        tread_depths=(0.30, 0.27, 0.24),
        widths=(0.85, 0.95, 1.05),
        landing_length=0.70,
    ),
    TerrainType.STAIRS_2: StairProfile(
        rises=(0.055, 0.070, 0.085),
        tread_depths=(0.27, 0.23, 0.20),
        widths=(0.90, 1.00, 1.10),
        landing_length=0.65,
    ),
    TerrainType.STAIRS_3: StairProfile(
        rises=(0.080, 0.100, 0.120),
        tread_depths=(0.24, 0.20, 0.17),
        widths=(0.95, 1.05, 1.15),
        landing_length=0.60,
    ),
}


SLOPE_PRESETS: dict[TerrainType, SlopeProfile] = {
    TerrainType.SLOPE_1: SlopeProfile(
        angle_degrees=8.0,
        length=1.40,
        width=0.90,
        landing_length=0.65,
    ),
    TerrainType.SLOPE_2: SlopeProfile(
        angle_degrees=15.0,
        length=1.20,
        width=1.00,
        landing_length=0.60,
    ),
    TerrainType.SLOPE_3: SlopeProfile(
        angle_degrees=25.0,
        length=1.00,
        width=1.10,
        landing_length=0.55,
    ),
}


TERRAIN_LABELS: dict[TerrainType, str] = {
    TerrainType.FLAT: "평지",
    TerrainType.UNEVEN: "울퉁불퉁한 지형",
    TerrainType.STAIRS_1: "계단 1단계",
    TerrainType.STAIRS_2: "계단 2단계",
    TerrainType.STAIRS_3: "계단 3단계",
    TerrainType.SLOPE_1: "경사 1단계 (8°)",
    TerrainType.SLOPE_2: "경사 2단계 (15°)",
    TerrainType.SLOPE_3: "경사 3단계 (25°)",
    TerrainType.MIXED: "혼합 코스",
}


__all__ = ["SLOPE_PRESETS", "STAIR_PRESETS", "TERRAIN_LABELS"]
