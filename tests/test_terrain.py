from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from types import SimpleNamespace
from unittest.mock import patch

import mujoco
import numpy as np

from src.simulation import build_terrain_xml, load_model
from src.simulation.core.cli_bridge import SimulationControl
from src.simulation.core.viewer import configure_simulation_viewer
from src.simulation.core.simulator_cli import (
    select_rl_checkpoint,
    select_simulation_control,
    select_stair_demo_strategy,
    select_stair_terrain,
    select_terrain,
)
from src.simulation.core.stair_demo import StairDemoStrategy
from src.simulation.terrain import (
    SLOPE_PRESETS,
    STAIR_PRESETS,
    TerrainType,
)


class TerrainTests(unittest.TestCase):
    def test_launcher_control_picker_returns_selected_controller(self) -> None:
        prompt = SimpleNamespace(execute=lambda: SimulationControl.RL)

        with patch("InquirerPy.inquirer.select", return_value=prompt):
            self.assertIs(select_simulation_control(), SimulationControl.RL)

    def test_launcher_checkpoint_picker_uses_local_models(self) -> None:
        from src.rl.inquiry import PROJECT_ROOT

        checkpoint = PROJECT_ROOT / "runs" / "example" / "policy.zip"
        prompt = SimpleNamespace(execute=lambda: checkpoint)
        with (
            patch("src.rl.inquiry.local_model_files", return_value=[checkpoint]),
            patch("InquirerPy.inquirer.select", return_value=prompt),
        ):
            self.assertEqual(select_rl_checkpoint(), checkpoint)

    def test_launcher_terrain_picker_uses_inquirer_selection(self) -> None:
        prompt = SimpleNamespace(execute=lambda: TerrainType.MIXED)

        with patch("InquirerPy.inquirer.select", return_value=prompt):
            self.assertIs(select_terrain(), TerrainType.MIXED)

    def test_launcher_stair_demo_pickers_return_selected_values(self) -> None:
        prompts = iter(
            (
                SimpleNamespace(execute=lambda: StairDemoStrategy.IMPROVED),
                SimpleNamespace(execute=lambda: TerrainType.STAIRS_3),
            )
        )
        with patch("InquirerPy.inquirer.select", side_effect=lambda **_: next(prompts)):
            self.assertIs(
                select_stair_demo_strategy(),
                StairDemoStrategy.IMPROVED,
            )
            self.assertIs(select_stair_terrain(), TerrainType.STAIRS_3)

    def test_every_preset_compiles_with_all_actuators(self) -> None:
        for terrain in TerrainType:
            with self.subTest(terrain=terrain.value):
                model = load_model(terrain=terrain)
                self.assertEqual(model.nu, 18)
                generated = [
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                    for geom_id in range(model.ngeom)
                ]
                generated = [
                    name for name in generated if name and name.startswith("terrain_")
                ]
                if terrain is TerrainType.FLAT:
                    self.assertEqual(generated, [])
                else:
                    self.assertTrue(generated)

    def test_generated_terrain_uses_default_visible_geom_group(self) -> None:
        model = load_model(terrain=TerrainType.MIXED)
        terrain_ids = [
            geom_id
            for geom_id in range(model.ngeom)
            if (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                or ""
            ).startswith("terrain_")
        ]

        self.assertTrue(terrain_ids)
        self.assertTrue(
            all(model.geom_group[geom_id] == 0 for geom_id in terrain_ids)
        )
        self.assertTrue(
            all(model.geom_rgba[geom_id, 3] == 1.0 for geom_id in terrain_ids)
        )

    def test_viewer_tracks_robot_without_zooming_to_whole_mixed_course(self) -> None:
        model = load_model(terrain=TerrainType.MIXED)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        root_body_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "UPPER_BODY_1",
        )
        viewer = SimpleNamespace(
            cam=SimpleNamespace(
                type=None,
                trackbodyid=-1,
                lookat=np.zeros(3),
                distance=0.0,
                azimuth=0.0,
                elevation=0.0,
            ),
            opt=SimpleNamespace(geomgroup=np.zeros(6, dtype=np.uint8)),
        )

        configure_simulation_viewer(
            viewer,
            model,
            data,
            tracking_body_id=root_body_id,
        )

        self.assertEqual(viewer.cam.type, mujoco.mjtCamera.mjCAMERA_TRACKING)
        self.assertEqual(viewer.cam.trackbodyid, root_body_id)
        self.assertLessEqual(viewer.cam.distance, 3.0)
        self.assertEqual(viewer.opt.geomgroup[0], 1)

    def test_fixed_base_removes_root_freejoint(self) -> None:
        model = load_model(floating_base=False, terrain=TerrainType.UNEVEN)

        root_freejoint = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "root_freejoint",
        )
        self.assertEqual(root_freejoint, -1)

    def test_uneven_seed_is_reproducible(self) -> None:
        xml_a, result_a = build_terrain_xml(terrain=TerrainType.UNEVEN, terrain_seed=9)
        xml_b, result_b = build_terrain_xml(terrain=TerrainType.UNEVEN, terrain_seed=9)
        xml_c, _ = build_terrain_xml(terrain=TerrainType.UNEVEN, terrain_seed=10)

        self.assertEqual(xml_a, xml_b)
        self.assertEqual(result_a, result_b)
        self.assertNotEqual(xml_a, xml_c)

    def test_stair_steps_have_individual_dimensions(self) -> None:
        xml, result = build_terrain_xml(terrain=TerrainType.STAIRS_1)
        root = ET.fromstring(xml)
        steps = [
            geom
            for geom in root.findall("./worldbody/geom")
            if (geom.get("name") or "").startswith("terrain_stairs_1_up_")
        ]

        self.assertEqual(len(steps), len(STAIR_PRESETS[TerrainType.STAIRS_1].rises))
        sizes = [tuple(float(value) for value in step.get("size", "").split()) for step in steps]
        self.assertEqual(len({size[0] for size in sizes}), len(sizes))
        self.assertEqual(len({size[1] for size in sizes}), len(sizes))
        self.assertAlmostEqual(
            result.max_height,
            STAIR_PRESETS[TerrainType.STAIRS_1].total_height,
        )

    def test_stair_presets_use_requested_fixed_riser_heights(self) -> None:
        expected = {
            TerrainType.STAIRS_1: (0.10, 0.10, 0.10),
            TerrainType.STAIRS_2: (0.15, 0.15, 0.15),
            TerrainType.STAIRS_3: (0.20, 0.20, 0.20),
        }

        for terrain, rises in expected.items():
            with self.subTest(terrain=terrain.value):
                self.assertEqual(STAIR_PRESETS[terrain].rises, rises)

        self.assertEqual(
            STAIR_PRESETS[TerrainType.STAIRS_3].tread_depths,
            (0.35, 0.35, 0.35),
        )

    def test_slope_presets_use_three_distinct_angles(self) -> None:
        angles = {
            profile.angle_degrees for profile in SLOPE_PRESETS.values()
        }
        self.assertEqual(len(angles), 3)

    def test_mixed_contains_every_requested_family(self) -> None:
        _, result = build_terrain_xml(terrain=TerrainType.MIXED)
        names = result.geom_names
        required = (
            "terrain_mixed_uneven",
            "terrain_mixed_stairs_1",
            "terrain_mixed_stairs_2",
            "terrain_mixed_stairs_3",
            "terrain_mixed_slope_1",
            "terrain_mixed_slope_2",
            "terrain_mixed_slope_3",
        )
        for prefix in required:
            self.assertTrue(any(name.startswith(prefix) for name in names), prefix)
        self.assertGreater(result.end_y, result.start_y)


if __name__ == "__main__":
    unittest.main()
