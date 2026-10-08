"""Non-destructive MJCF contact-geometry variants for controlled ablations."""

from __future__ import annotations

import hashlib
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path
from typing import Callable

from src.simulation.core.model import (
    BACKLASH_ARCMIN,
    BACKLASH_SUFFIX,
    DEFAULT_MODEL_PATH,
    add_joint_backlash,
    compose,
)


CONTACT_GEOMETRIES = ("open-arc", "closed-wheel")
TIRE_GEOM_NAMES = tuple(f"TIRE_{leg}_geom" for leg in range(1, 7))

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


def transform_for_contact_geometry(
    geometry: str,
) -> Callable[[ET.Element], None] | None:
    if geometry == "open-arc":
        return None
    if geometry == "closed-wheel":
        return replace_open_arcs_with_closed_wheels
    raise ValueError(
        f"unknown contact geometry {geometry!r}; choose from {CONTACT_GEOMETRIES}"
    )


def transform_for_variant(
    geometry: str = "open-arc",
    *,
    backlash: bool = False,
) -> Callable[[ET.Element], None] | None:
    """Combine the contact-geometry and gear-backlash axes."""

    return compose(
        transform_for_contact_geometry(geometry),
        add_joint_backlash if backlash else None,
    )


@lru_cache(maxsize=8)
def model_fingerprint(model_path: str | Path = DEFAULT_MODEL_PATH) -> str:
    """Hash the source MJCF and every referenced mesh in deterministic order."""

    path = Path(model_path).expanduser().resolve()
    root = ET.parse(path).getroot()
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    filenames = sorted(
        {
            mesh.get("file")
            for mesh in root.findall("./asset/mesh")
            if mesh.get("file")
        }
    )
    for filename in filenames:
        digest.update(filename.encode("utf-8"))
        digest.update((path.parent / filename).resolve().read_bytes())
    return digest.hexdigest()


__all__ = [
    "BACKLASH_ARCMIN",
    "BACKLASH_SUFFIX",
    "CLOSED_WHEEL_CENTER",
    "CLOSED_WHEEL_HALF_WIDTH_M",
    "CLOSED_WHEEL_RADIUS_M",
    "CONTACT_GEOMETRIES",
    "TIRE_GEOM_NAMES",
    "add_joint_backlash",
    "compose",
    "model_fingerprint",
    "replace_open_arcs_with_closed_wheels",
    "transform_for_contact_geometry",
    "transform_for_variant",
]
