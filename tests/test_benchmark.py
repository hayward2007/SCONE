from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from benchmark.common import (
    BenchmarkConfig,
    Perturbation,
    temporary_stair_profile,
    write_records,
)
from benchmark.capture import CaptureConfig
from benchmark.flat import run_flat_trial
from benchmark.report import summarize_records
from src.simulation.terrain import STAIR_PRESETS, StairProfile, TerrainType


class BenchmarkTests(unittest.TestCase):
    def test_capture_config_requires_even_h264_dimensions(self) -> None:
        self.assertEqual(CaptureConfig().width, 640)
        with self.assertRaises(ValueError):
            CaptureConfig(width=641)
        with self.assertRaises(ValueError):
            CaptureConfig(crf=52)

    def test_perturbation_rejects_nonphysical_scales_and_phase(self) -> None:
        with self.assertRaises(ValueError):
            Perturbation(mass_scale=0.0)
        with self.assertRaises(ValueError):
            Perturbation(friction_scale=-1.0)
        with self.assertRaises(ValueError):
            Perturbation(gait_phase=1.0)

    def test_short_flat_trial_is_serializable_and_finite(self) -> None:
        record = run_flat_trial(
            "articulated-walk",
            (0.18, 0.0, 0.0),
            command_name="forward",
            perturbation=Perturbation(gait_phase=0.25),
            config=BenchmarkConfig(settle_seconds=0.02, measure_seconds=0.06),
        )
        self.assertEqual(record["benchmark"], "flat")
        self.assertEqual(record["controller"], "articulated-walk")
        self.assertTrue(math.isfinite(float(record["mean_vx_mps"])))
        self.assertGreater(float(record["duration_s"]), 0.0)
        json.dumps(record)

    def test_custom_stair_profile_context_restores_preset(self) -> None:
        original = STAIR_PRESETS[TerrainType.STAIRS_3]
        custom = StairProfile(
            rises=(0.12, 0.12, 0.12),
            tread_depths=(0.30, 0.30, 0.30),
            widths=(1.0, 1.0, 1.0),
        )
        with temporary_stair_profile(TerrainType.STAIRS_3, custom):
            self.assertIs(STAIR_PRESETS[TerrainType.STAIRS_3], custom)
        self.assertIs(STAIR_PRESETS[TerrainType.STAIRS_3], original)

    def test_summary_uses_wilson_success_interval(self) -> None:
        rows = summarize_records(
            [
                {
                    "benchmark": "flat",
                    "controller": "a",
                    "command_name": "forward",
                    "completed": True,
                    "mean_vx_mps": 0.1,
                },
                {
                    "benchmark": "flat",
                    "controller": "a",
                    "command_name": "forward",
                    "completed": False,
                    "mean_vx_mps": 0.2,
                },
            ]
        )
        self.assertEqual(rows[0]["N"], 2)
        self.assertEqual(rows[0]["success_rate"], 0.5)
        self.assertLess(rows[0]["success_wilson95_low"], 0.5)
        self.assertGreater(rows[0]["success_wilson95_high"], 0.5)
        self.assertAlmostEqual(rows[0]["mean_vx_mps_mean"], 0.15)

    def test_write_records_supports_jsonl_and_csv(self) -> None:
        records = [{"controller": "a", "completed": True, "value": 1.0}]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            jsonl = write_records(records, root / "result.jsonl")
            csv_path = write_records(records, root / "result.csv")
            self.assertIn('"controller": "a"', jsonl.read_text())
            self.assertIn("controller", csv_path.read_text())


if __name__ == "__main__":
    unittest.main()
