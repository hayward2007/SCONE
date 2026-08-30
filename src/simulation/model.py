"""MuJoCo model loading utilities; no input or motion policy lives here."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[1] / "assets" / "model.xml"


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
) -> mujoco.MjModel:
    """Load MJCF, adding a runtime freejoint/floor only when absent."""

    path = Path(model_path).expanduser().resolve()
    fixed_model = mujoco.MjModel.from_xml_path(str(path))
    if not floating_base:
        return fixed_model

    root = ET.parse(path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF is missing <worldbody>")
    root_body = worldbody.find("body")
    if root_body is None:
        raise ValueError("MJCF is missing its root robot body")
    if root_body.find("freejoint") is None:
        root_body.insert(0, ET.Element("freejoint", {"name": "root_freejoint"}))

    if not any(
        geom.get("name") == "simulation_floor" for geom in worldbody.findall("geom")
    ):
        floor = ET.Element(
            "geom",
            {
                "name": "simulation_floor",
                "type": "plane",
                "pos": f"0 0 {_contact_mesh_floor_height(fixed_model):.9g}",
                "size": "3 3 0.1",
                "rgba": "0.22 0.24 0.27 1",
                "friction": "1.0 0.005 0.0005",
                "condim": "6",
            },
        )
        worldbody.insert(1, floor)
    xml = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml, _mesh_assets(path, root))


__all__ = ["DEFAULT_MODEL_PATH", "load_model"]
