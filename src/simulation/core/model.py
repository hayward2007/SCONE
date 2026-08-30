"""MuJoCo model loading utilities owned by :mod:`simulation.core`."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from ..terrain import TerrainBuildResult, TerrainType, add_terrain


DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[2] / "assets" / "model.xml"


def _mesh_assets(model_path: Path, root: ET.Element) -> dict[str, bytes]:
    assets: dict[str, bytes] = {}
    for mesh in root.findall("./asset/mesh"):
        filename = mesh.get("file")
        if filename:
            assets[filename] = (model_path.parent / filename).resolve().read_bytes()
    return assets


def _contact_mesh_floor_height(model: mujoco.MjModel) -> float:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lowest = np.inf
    for geom_id in range(model.ngeom):
        if model.geom_contype[geom_id] == 0:
            continue
        mesh_id = int(model.geom_dataid[geom_id])
        if mesh_id < 0 or model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        address = int(model.mesh_vertadr[mesh_id])
        count = int(model.mesh_vertnum[mesh_id])
        vertices = model.mesh_vert[address : address + count]
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        world_vertices = vertices @ rotation.T + data.geom_xpos[geom_id]
        lowest = min(lowest, float(world_vertices[:, 2].min()))
    if not np.isfinite(lowest):
        raise ValueError("cannot create a floor: model has no contact mesh geoms")
    return lowest - 0.001


def load_model(
    model_path: str | Path = DEFAULT_MODEL_PATH,
    *,
    floating_base: bool = True,
    terrain: TerrainType | str = TerrainType.FLAT,
    terrain_seed: int = 7,
) -> mujoco.MjModel:
    """Load MJCF and inject the requested procedural terrain.

    Terrain is generated from MuJoCo primitive geoms at load time. The source
    robot MJCF therefore remains a robot asset rather than accumulating test
    course geometry.
    """

    path = Path(model_path).expanduser().resolve()
    fixed_model = mujoco.MjModel.from_xml_path(str(path))
    selected_terrain = TerrainType.parse(terrain)

    root = ET.parse(path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF is missing <worldbody>")
    root_body = worldbody.find("body")
    if root_body is None:
        raise ValueError("MJCF is missing its root robot body")
    if floating_base and root_body.find("freejoint") is None:
        root_body.insert(0, ET.Element("freejoint", {"name": "root_freejoint"}))
    if not floating_base:
        for freejoint in tuple(root_body.findall("freejoint")):
            root_body.remove(freejoint)

    floor = next(
        (
            geom
            for geom in worldbody.findall("geom")
            if geom.get("name") == "simulation_floor"
        ),
        None,
    )
    if floor is None:
        floor_z = _contact_mesh_floor_height(fixed_model)
        floor = ET.Element(
            "geom",
            {
                "name": "simulation_floor",
                "type": "plane",
                "pos": f"0 0 {floor_z:.9g}",
                "size": "3 3 0.1",
                "rgba": "0.22 0.24 0.27 1",
                "friction": "1.0 0.005 0.0005",
                "condim": "6",
            },
        )
        worldbody.insert(1, floor)
    else:
        position = [float(value) for value in floor.get("pos", "0 0 0").split()]
        if len(position) != 3:
            raise ValueError("simulation_floor pos must contain three values")
        floor_z = position[2]

    if selected_terrain is not TerrainType.FLAT:
        add_terrain(
            worldbody,
            selected_terrain,
            floor_z=floor_z,
            seed=terrain_seed,
        )
    xml = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml, _mesh_assets(path, root))


def build_terrain_xml(
    model_path: str | Path = DEFAULT_MODEL_PATH,
    *,
    terrain: TerrainType | str,
    terrain_seed: int = 7,
) -> tuple[str, TerrainBuildResult]:
    """Return inspectable MJCF text plus terrain metadata without compiling it."""

    path = Path(model_path).expanduser().resolve()
    fixed_model = mujoco.MjModel.from_xml_path(str(path))
    root = ET.parse(path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF is missing <worldbody>")
    floor = next(
        (
            geom
            for geom in worldbody.findall("geom")
            if geom.get("name") == "simulation_floor"
        ),
        None,
    )
    if floor is None:
        floor_z = _contact_mesh_floor_height(fixed_model)
    else:
        position = [float(value) for value in floor.get("pos", "0 0 0").split()]
        if len(position) != 3:
            raise ValueError("simulation_floor pos must contain three values")
        floor_z = position[2]
    result = add_terrain(
        worldbody,
        terrain,
        floor_z=floor_z,
        seed=terrain_seed,
    )
    return ET.tostring(root, encoding="unicode"), result


__all__ = ["DEFAULT_MODEL_PATH", "build_terrain_xml", "load_model"]
