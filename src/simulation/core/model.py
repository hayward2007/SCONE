"""MuJoCo model loading utilities owned by :mod:`simulation.core`."""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Callable, Iterable, Sequence

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


# --------------------------------------------------------------------------
# Leg loss, for the fail-safe policy (src/rl/walk_failsafe.py).
#
# Both modes deliberately keep every body, joint and actuator in place, so
# nq/nv/nu and the 18-actuator ordering are identical to the nominal robot.
# A fault is a change of dynamics, never a change of model layout: one policy
# and one checkpoint therefore cover the healthy robot and every fault case.
# --------------------------------------------------------------------------

LEG_INDICES = (1, 2, 3, 4, 5, 6)
LEG_ROOT_BODY = "BODY_ACTUATOR_{leg}"
FAILURE_MODES = ("detached", "limp")

# "detached": the whole leg module is unbolted from the chassis, so its mass,
# its collision and its drawing all go. Mass is scaled rather than zeroed
# because MuJoCo rejects a jointed body whose inertia falls below mjMINVAL.
# What is left is ~0.1% of the leg, which is three orders of magnitude below
# the joint armature that actually conditions these DOFs (docs/18), so the
# remaining stub neither carries load nor stiffens the solver.
DETACHED_MASS_SCALE = 1.0e-3
DETACHED_RGBA = "0 0 0 0"               # fully transparent: the leg is not drawn

# "limp": a DYNAMIXEL whose torque is disabled is high impedance, not floppy.
# Through a 193:1 (MX-28AT) or 353.5:1 (XM430) gearbox the reflected static
# friction is what holds the joint, so an unpowered joint is modelled as zero
# terminal voltage plus a Coulomb friction torque. The fraction below is an
# assumption to be replaced by a back-drive measurement on hardware.
# A degenerate control range is rejected by the compiler, so the dead motor
# gets a range that is symmetric, legal, and six orders of magnitude below the
# volt-scale command the live actuators receive.
LIMP_CTRLRANGE = "-1e-6 1e-6"
LIMP_FRICTIONLOSS_STALL_FRACTION = 0.30
LIMP_STALL_TORQUE = {"mx28at": 2.5, "xm430_w350": 4.1, "xm430_w210": 3.0}


def _scale_inertial(inertial: ET.Element, scale: float) -> None:
    """Shrink one <inertial> uniformly, keeping its principal-axis shape."""

    mass = inertial.get("mass")
    if mass is not None:
        inertial.set("mass", f"{float(mass) * scale:.9g}")
    for key in ("fullinertia", "diaginertia"):
        values = inertial.get(key)
        if values is None:
            continue
        inertial.set(
            key,
            " ".join(f"{float(value) * scale:.9g}" for value in values.split()),
        )


def _leg_of_actuated_joint(joint_name: str) -> int | None:
    """Return the leg a joint belongs to, from its ``..._L<n>`` suffix."""

    if not ACTUATED_JOINT_PATTERN.fullmatch(joint_name):
        return None
    _, _, suffix = joint_name.rpartition("_L")
    return int(suffix) if suffix.isdigit() else None


def _motor_family(joint_name: str) -> str:
    index = int(joint_name[1:3])
    if index <= 6:
        return "mx28at"
    return "xm430_w350" if index <= 12 else "xm430_w210"


def _leg_root_bodies(root: ET.Element, legs: Sequence[int]) -> list[ET.Element]:
    wanted = {LEG_ROOT_BODY.format(leg=leg) for leg in legs}
    found = {
        body.get("name"): body
        for body in root.iter("body")
        if body.get("name") in wanted
    }
    missing = sorted(wanted - set(found))
    if missing:
        raise ValueError(f"MJCF is missing leg root bodies: {missing}")
    return [found[name] for name in sorted(wanted)]


def validate_failed_legs(legs: Iterable[int]) -> tuple[int, ...]:
    """Normalise a leg-loss set, rejecting anything that is not a SCONE leg."""

    ordered = tuple(sorted({int(leg) for leg in legs}))
    unknown = [leg for leg in ordered if leg not in LEG_INDICES]
    if unknown:
        raise ValueError(f"unknown leg numbers {unknown}; expected {LEG_INDICES}")
    return ordered


def amputate_legs(
    legs: Iterable[int],
    mode: str = "detached",
) -> Callable[[ET.Element], None] | None:
    """Build an MJCF transform that disables the given legs.

    ``detached`` removes the leg's mass, collision and visual presence, which
    is what the robot looks like after a leg breaks off at the hip. ``limp``
    keeps the leg attached and lets it drag: the motors lose power and only
    gearbox friction resists motion.
    """

    failed = validate_failed_legs(legs)
    if mode not in FAILURE_MODES:
        raise ValueError(f"unknown failure mode {mode!r}; expected {FAILURE_MODES}")
    if not failed:
        return None

    def apply(root: ET.Element) -> None:
        if mode == "detached":
            for body in _leg_root_bodies(root, failed):
                for geom in body.iter("geom"):
                    geom.set("contype", "0")
                    geom.set("conaffinity", "0")
                    geom.set("rgba", DETACHED_RGBA)
                for inertial in body.iter("inertial"):
                    _scale_inertial(inertial, DETACHED_MASS_SCALE)
            return

        # limp: cut the drive, leave the mass and the collision alone.
        for body in root.iter("body"):
            for joint in body.findall("joint"):
                name = joint.get("name") or ""
                if _leg_of_actuated_joint(name) not in failed:
                    continue
                stall = LIMP_STALL_TORQUE[_motor_family(name)]
                joint.set(
                    "frictionloss",
                    f"{stall * LIMP_FRICTIONLOSS_STALL_FRACTION:.9g}",
                )
        actuators = root.find("actuator")
        if actuators is None:
            raise ValueError("MJCF is missing <actuator>")
        for actuator in actuators:
            joint_name = actuator.get("joint") or ""
            if _leg_of_actuated_joint(joint_name) in failed:
                actuator.set("ctrllimited", "true")
                actuator.set("ctrlrange", LIMP_CTRLRANGE)

    return apply


class LegFailureRuntime:
    """Apply and undo leg failures on a compiled model, without recompiling.

    :func:`amputate_legs` is the reference: it edits the MJCF and lets the
    compiler produce the model.  That is the right thing for a viewer, a
    benchmark, or anything that picks one robot and keeps it, but an RL episode
    draws a new failure every reset and recompiling six meshes each time is
    far too slow.

    Every quantity those transforms change is a plain writable field of
    ``MjModel``, so the same result can be reached by assignment.  This class
    owns that mapping, keeps the pristine values so a failure can be lifted,
    and is verified against the compiler in ``tests/test_walk_failsafe.py``:
    if the two ever diverge, the compiled model is right and this is wrong.
    """

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self._geom_ids: dict[int, tuple[int, ...]] = {}
        self._body_ids: dict[int, tuple[int, ...]] = {}
        self._actuator_ids: dict[int, tuple[int, ...]] = {}
        self._dof_ids: dict[int, tuple[int, ...]] = {}
        for leg in LEG_INDICES:
            bodies = self._subtree_body_ids(model, LEG_ROOT_BODY.format(leg=leg))
            self._body_ids[leg] = bodies
            self._geom_ids[leg] = tuple(
                geom
                for geom in range(model.ngeom)
                if int(model.geom_bodyid[geom]) in set(bodies)
            )
            actuators: list[int] = []
            dofs: list[int] = []
            for actuator in range(model.nu):
                joint = int(model.actuator_trnid[actuator, 0])
                if joint < 0:
                    continue
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
                if _leg_of_actuated_joint(name or "") != leg:
                    continue
                actuators.append(actuator)
                dofs.append(int(model.jnt_dofadr[joint]))
            self._actuator_ids[leg] = tuple(actuators)
            self._dof_ids[leg] = tuple(dofs)

        self._pristine = {
            "geom_contype": model.geom_contype.copy(),
            "geom_conaffinity": model.geom_conaffinity.copy(),
            "geom_rgba": model.geom_rgba.copy(),
            "body_mass": model.body_mass.copy(),
            "body_inertia": model.body_inertia.copy(),
            "actuator_ctrlrange": model.actuator_ctrlrange.copy(),
            "actuator_ctrllimited": model.actuator_ctrllimited.copy(),
            "dof_frictionloss": model.dof_frictionloss.copy(),
        }
        self.failed_legs: tuple[int, ...] = ()
        self.mode = "detached"

    @staticmethod
    def _subtree_body_ids(model: mujoco.MjModel, root_name: str) -> tuple[int, ...]:
        root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, root_name)
        if root < 0:
            raise ValueError(f"model has no body named {root_name!r}")
        found = [root]
        # Bodies are stored parent-before-child, so one forward pass closes
        # the subtree.
        members = {root}
        for body in range(root + 1, model.nbody):
            if int(model.body_parentid[body]) in members:
                members.add(body)
                found.append(body)
        return tuple(found)

    def apply(self, legs: Iterable[int], mode: str = "detached") -> tuple[int, ...]:
        """Fail exactly these legs, restoring any that were failed before."""

        failed = validate_failed_legs(legs)
        if mode not in FAILURE_MODES:
            raise ValueError(f"unknown failure mode {mode!r}; expected {FAILURE_MODES}")
        self.restore()
        self.failed_legs = failed
        self.mode = mode
        model = self.model
        for leg in failed:
            if mode == "detached":
                geoms = list(self._geom_ids[leg])
                model.geom_contype[geoms] = 0
                model.geom_conaffinity[geoms] = 0
                # DETACHED_RGBA zeroes all four channels, not just alpha, so
                # this matches what the compiler produces bit for bit.
                model.geom_rgba[geoms] = [
                    float(value) for value in DETACHED_RGBA.split()
                ]
                bodies = list(self._body_ids[leg])
                model.body_mass[bodies] *= DETACHED_MASS_SCALE
                model.body_inertia[bodies] *= DETACHED_MASS_SCALE
            else:
                actuators = list(self._actuator_ids[leg])
                low, high = (float(value) for value in LIMP_CTRLRANGE.split())
                model.actuator_ctrllimited[actuators] = 1
                model.actuator_ctrlrange[actuators, 0] = low
                model.actuator_ctrlrange[actuators, 1] = high
                for actuator, dof in zip(actuators, self._dof_ids[leg]):
                    joint = int(model.actuator_trnid[actuator, 0])
                    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
                    stall = LIMP_STALL_TORQUE[_motor_family(name or "M01_x_L1")]
                    model.dof_frictionloss[dof] = (
                        stall * LIMP_FRICTIONLOSS_STALL_FRACTION
                    )
        return failed

    def restore(self) -> None:
        """Return every mutated field to the value the compiler produced."""

        for field, values in self._pristine.items():
            getattr(self.model, field)[:] = values
        self.failed_legs = ()


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
