from __future__ import annotations

import json
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from benchmark.common import (
    BenchmarkConfig,
    Perturbation,
    temporary_stair_profile,
    write_records,
)
from benchmark.capture import CaptureConfig
from benchmark.flat import run_flat_trial
from benchmark.icra import sample_paired_perturbations
from benchmark.model_variants import (
    TIRE_GEOM_NAMES,
    replace_open_arcs_with_closed_wheels,
    transform_for_contact_geometry,
)
from benchmark.report import summarize_paired_differences, summarize_records
from src.simulation.core.model import DEFAULT_MODEL_PATH, load_model
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

    def test_closed_wheel_transform_is_non_destructive_and_complete(self) -> None:
        source = DEFAULT_MODEL_PATH.read_text(encoding="utf-8")
        root = ET.fromstring(source)
        replace_open_arcs_with_closed_wheels(root)
        transformed = {
            geom.get("name"): geom
            for geom in root.findall(".//geom")
            if geom.get("name") in TIRE_GEOM_NAMES
        }
        self.assertEqual(set(transformed), set(TIRE_GEOM_NAMES))
        self.assertTrue(all(geom.get("type") == "cylinder" for geom in transformed.values()))
        self.assertTrue(all(geom.get("mesh") is None for geom in transformed.values()))
        self.assertEqual(DEFAULT_MODEL_PATH.read_text(encoding="utf-8"), source)

    def test_closed_wheel_model_compiles_without_changing_mass_or_inertia(self) -> None:
        open_model = load_model(floating_base=True)
        closed_model = load_model(
            floating_base=True,
            xml_transform=transform_for_contact_geometry("closed-wheel"),
        )
        np.testing.assert_allclose(open_model.body_mass, closed_model.body_mass)
        np.testing.assert_allclose(open_model.body_inertia, closed_model.body_inertia)
        for name in TIRE_GEOM_NAMES:
            geom_id = mujoco.mj_name2id(
                closed_model,
                mujoco.mjtObj.mjOBJ_GEOM,
                name,
            )
            self.assertEqual(
                int(closed_model.geom_type[geom_id]),
                int(mujoco.mjtGeom.mjGEOM_CYLINDER),
            )

    def test_paired_perturbations_are_deterministic(self) -> None:
        first = sample_paired_perturbations(seed=42, count=4)
        second = sample_paired_perturbations(seed=42, count=4)
        different = sample_paired_perturbations(seed=43, count=4)
        self.assertEqual(first, second)
        self.assertNotEqual(first, different)

    def test_short_matched_closed_wheel_trial_is_serializable(self) -> None:
        record = run_flat_trial(
            "matched-coordinated",
            (0.10, 0.0, 0.0),
            command_name="forward-0.10",
            contact_geometry="closed-wheel",
            perturbation=Perturbation(gait_phase=0.25),
            config=BenchmarkConfig(settle_seconds=0.02, measure_seconds=0.06),
        )
        self.assertEqual(record["contact_geometry"], "closed-wheel")
        self.assertTrue(math.isfinite(float(record["mean_vx_mps"])))
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

    def test_bootstrap_and_paired_summaries_are_deterministic(self) -> None:
        records = []
        for pair_id, closed, opened in (("a", 0.1, 0.2), ("b", 0.2, 0.4)):
            records.extend(
                (
                    {
                        "controller": "matched-coordinated",
                        "command_name": "forward",
                        "pair_id": pair_id,
                        "contact_geometry": "closed-wheel",
                        "mean_vx_mps": closed,
                    },
                    {
                        "controller": "matched-coordinated",
                        "command_name": "forward",
                        "pair_id": pair_id,
                        "contact_geometry": "open-arc",
                        "mean_vx_mps": opened,
                    },
                )
            )
        first = summarize_paired_differences(
            records,
            condition_field="contact_geometry",
            reference="closed-wheel",
            candidate="open-arc",
            group_by=("controller", "command_name"),
            metrics=("mean_vx_mps",),
            bootstrap_seed=7,
            bootstrap_samples=200,
        )
        second = summarize_paired_differences(
            records,
            condition_field="contact_geometry",
            reference="closed-wheel",
            candidate="open-arc",
            group_by=("controller", "command_name"),
            metrics=("mean_vx_mps",),
            bootstrap_seed=7,
            bootstrap_samples=200,
        )
        self.assertEqual(first, second)
        self.assertEqual(first[0]["paired_N"], 2)
        self.assertAlmostEqual(first[0]["mean_vx_mps_mean_difference"], 0.15)

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
