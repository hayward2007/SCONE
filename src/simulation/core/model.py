"""MuJoCo model loading utilities owned by :mod:`simulation.core`."""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Callable

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


ACTUATED_JOINT_PATTERN = re.compile(r"M(0[1-9]|1[0-8])_\w+")


# Gear backlash from the ROBOTIS e-Manual, in arcminutes of output-shaft play.
# Modelled as a free joint in series with each actuated joint, limited to half
# the total play either side of zero -- the technique Open Duck Mini v2 uses.
# At the 0.1225 m arc radius, the XM430's 15 arcmin is 0.53 mm of contact
# position, which is not negligible for a paper about edge engagement.
BACKLASH_ARCMIN = {"mx28at": 20.0, "xm430": 15.0}
BACKLASH_SUFFIX = "_backlash"
# Small but non-zero: the free member has to have some inertia and dissipation
# or the dead-band rattles at the integrator's resolution.
BACKLASH_ARMATURE = 1.0e-5
BACKLASH_DAMPING = 0.01
# MuJoCo's default limit is soft enough that a working torque pushes several
# times past the stop, which would model far more play than the datasheet
# allows. Pull the constraint time constant down to a timestep and raise the
# impedance so the dead-band edge behaves like a tooth flank.
BACKLASH_SOLREFLIMIT = "0.002 1"
BACKLASH_SOLIMPLIMIT = "0.95 0.99 0.0005 0.5 2"
TIRE_GEOM_NAMES = tuple(f"TIRE_{leg}_geom" for leg in range(1, 7))
ACTUATED_JOINT_PATTERN = re.compile(r"M(0[1-9]|1[0-8])_\w+")

# The exported TIRE mesh is 44 mm wide along its axis. MuJoCo recenters mesh
# vertices and stores the inverse recentering transform in ``geom_pos/quat``.
# Fitting the outer tenth of the compiled vertices gives a 122.5 mm radius and,
# after applying that transform, the following centre in the TIRE body frame.
# The primitive replaces collision geometry only; explicit body mass and inertia
# remain untouched so the comparison is not confounded by MuJoCo auto-inertia.
CLOSED_WHEEL_RADIUS_M = 0.1225
CLOSED_WHEEL_HALF_WIDTH_M = 0.022
CLOSED_WHEEL_CENTER = (0.124, 0.24349991, -0.21650636)


def _find_tire_geoms(root: ET.Element) -> list[ET.Element]:
    by_name = {
        geom.get("name"): geom
        for geom in root.findall(".//geom")
        if geom.get("name") in TIRE_GEOM_NAMES
    }
    missing = [name for name in TIRE_GEOM_NAMES if name not in by_name]
    if missing:
        raise ValueError(f"MJCF is missing tire collision geoms: {missing}")
    return [by_name[name] for name in TIRE_GEOM_NAMES]


def replace_open_arcs_with_closed_wheels(root: ET.Element) -> None:
    """Replace six tire collision meshes by same-envelope closed cylinders."""

    for geom in _find_tire_geoms(root):
        geom.attrib.pop("mesh", None)
        geom.set("type", "cylinder")
        geom.set(
            "size",
            f"{CLOSED_WHEEL_RADIUS_M:.9g} {CLOSED_WHEEL_HALF_WIDTH_M:.9g}",
        )
        geom.set("pos", " ".join(f"{value:.9g}" for value in CLOSED_WHEEL_CENTER))
        geom.set("euler", "0 1.5707963267948966 0")



def _backlash_half_range_rad(joint_name: str) -> float:
    """Half the total play, in radians, for the actuator driving this joint."""

    index = int(joint_name[1:3])
    family = "mx28at" if index <= 6 else "xm430"
    return math.radians(BACKLASH_ARCMIN[family] / 60.0) / 2.0


def add_joint_backlash(root: ET.Element) -> None:
    """Put a limited free joint in series with every actuated joint.

    MuJoCo composes the joints of one body in order, so a second hinge on the
    same axis makes the body angle the sum of the driven angle and the play.
    The actuator still targets the driven joint; the play is unactuated. A real
    X-series encoder sits on the output shaft, so a faithful reader of joint
    position must sum the pair (see ``ACTUATED_JOINT_PATTERN`` consumers).
    """

    compiler = root.find("compiler")
    # MuJoCo defaults to degrees when <compiler angle=...> is absent, and a
    # radian range emitted into such a model shrinks the dead band 57-fold.
    angle = "degree" if compiler is None else compiler.get("angle", "degree")
    for body in root.iter("body"):
        for joint in list(body.findall("joint")):
            name = joint.get("name") or ""
            if not ACTUATED_JOINT_PATTERN.fullmatch(name):
                continue
            half = _backlash_half_range_rad(name)
            if angle == "degree":
                half = math.degrees(half)
            play = ET.Element("joint", {
                "name": f"{name}{BACKLASH_SUFFIX}",
                "type": "hinge",
                "axis": joint.get("axis", "0 0 1"),
                "pos": joint.get("pos", "0 0 0"),
                "limited": "true",
                "range": f"{-half:.9g} {half:.9g}",
                "armature": f"{BACKLASH_ARMATURE:g}",
                "damping": f"{BACKLASH_DAMPING:g}",
                "frictionloss": "0",
                "stiffness": "0",
                "solreflimit": BACKLASH_SOLREFLIMIT,
                "solimplimit": BACKLASH_SOLIMPLIMIT,
            })
            body.insert(list(body).index(joint) + 1, play)


def compose(*transforms: Callable[[ET.Element], None] | None):
    """Chain MJCF transforms, ignoring the ones that are None."""

    active = [transform for transform in transforms if transform is not None]
    if not active:
        return None

    def apply(root: ET.Element) -> None:
        for transform in active:
            transform(root)

    return apply



def load_model(
    model_path: str | Path = DEFAULT_MODEL_PATH,
    *,
    floating_base: bool = True,
    terrain: TerrainType | str = TerrainType.FLAT,
    terrain_seed: int = 7,
    xml_transform: Callable[[ET.Element], None] | None = None,
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
    if xml_transform is not None:
        xml_transform(root)

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
    xml_transform: Callable[[ET.Element], None] | None = None,
) -> tuple[str, TerrainBuildResult]:
    """Return inspectable MJCF text plus terrain metadata without compiling it."""

    path = Path(model_path).expanduser().resolve()
    fixed_model = mujoco.MjModel.from_xml_path(str(path))
    root = ET.parse(path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF is missing <worldbody>")
    if xml_transform is not None:
        xml_transform(root)
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
