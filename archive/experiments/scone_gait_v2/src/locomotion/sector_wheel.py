"""Model-derived rolling geometry for SCONE's sector end frames.

``TripodGait`` treats every grounded sector tip as one fixed point.  The tip is
in fact a 225-degree TPU arc whose centre of curvature is the stage-2 joint
axis, which makes it a wheel with a gap rather than a foot.  Two consequences
follow, and both are measured here rather than assumed.

* Rotating stage-2 inside the arc does not move the geometric contact point.
  Over a 200-degree sweep the contact stays within 0.7 mm horizontally and
  0.05 mm vertically.  A sector rotation may therefore be *added* to an IK
  solution without invalidating it, but must never be blended into one.
* The rolling direction is not the direction the contact point drifts.  It is
  ``axis x radius`` at the contact, which for SCONE's mounting points is
  radially outward from the chassis.  The upper joint is an exact vertical-axis
  steering column that rotates this direction with unit gain.

``docs/28-scone-gait-v2-role-split-rolling.md`` records the measurements and
the reasoning that produced this module.  ``src/locomotion/scone_gait.py``
predates it and keeps its own, different, contact-difference estimate for
checkpoint compatibility; this module is not a drop-in replacement for that.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np
from numpy.typing import ArrayLike, NDArray

from src.kinematics import RobotKinematics


Vector2 = NDArray[np.float64]
Vector3 = NDArray[np.float64]

# Measuring the arc window costs a few hundred mesh transforms per leg, and it
# depends only on the model file and the calibration pose.  Benchmarks and
# tests build many gaits against the same pair, so keep the result.
_GEOMETRY_CACHE: dict[tuple[str, bytes], dict[int, "WheelGeometry"]] = {}

# Average the vertices in the lowest 0.1 mm of the sector tip, matching
# ``TripodGait.SUPPORT_PATCH_DEPTH``.  One absolute-lowest vertex locks onto
# either lateral edge of the 44 mm wide TPU band.
SUPPORT_PATCH_DEPTH = 1e-4


def active_support_point(
    kinematics: RobotKinematics,
    leg: int,
    motor_degrees: ArrayLike,
    *,
    patch_depth: float = SUPPORT_PATCH_DEPTH,
) -> Vector3:
    """Return the lowest sector-patch centre of one leg in the body frame."""

    kinematics.forward_motor_degrees(motor_degrees, frame="body")
    model = kinematics.model
    data = kinematics.data
    geom_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        f"TIRE_{leg}_geom",
    )
    if geom_id < 0:
        raise ValueError(f"model is missing TIRE_{leg}_geom")
    mesh_id = int(model.geom_dataid[geom_id])
    if mesh_id < 0 or model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
        raise ValueError(f"TIRE_{leg}_geom must be a mesh geom")

    address = int(model.mesh_vertadr[mesh_id])
    count = int(model.mesh_vertnum[mesh_id])
    local_vertices = model.mesh_vert[address : address + count]
    world_vertices = (
        local_vertices @ data.geom_xmat[geom_id].reshape(3, 3).T
        + data.geom_xpos[geom_id]
    )
    root_id = kinematics.legs[leg].root_body_id
    body_vertices = (
        world_vertices - data.xpos[root_id]
    ) @ data.xmat[root_id].reshape(3, 3)
    lowest = float(np.min(body_vertices[:, 2]))
    patch = body_vertices[body_vertices[:, 2] <= lowest + patch_depth]
    return np.mean(patch, axis=0)


def wrap_degrees(angle: float) -> float:
    """Fold an angle into ``[-180, 180)``."""

    return (float(angle) + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class WheelGeometry:
    """One leg's measured rolling geometry, in the body frame.

    ``heading_degrees`` is the direction the ground contact travels, relative
    to the body, for a *positive* stage-2 rate at zero steering.  The body then
    travels the other way.  ``arc_min_degrees`` and ``arc_max_degrees`` bound
    the stage-2 offsets, relative to the calibration pose, over which the
    sector still presents its arc to the ground.
    """

    leg: int
    heading_degrees: float
    roll_radius: float
    steering_gain: float
    arc_min_degrees: float
    arc_max_degrees: float
    hip: Vector2
    contact: Vector2

    @property
    def arc_span_degrees(self) -> float:
        return self.arc_max_degrees - self.arc_min_degrees


@dataclass(frozen=True)
class RollSolution:
    """How well one leg can roll a requested contact travel, and at what cost."""

    steering_degrees: float
    direction_degrees: float
    rate_sign: float
    alignment: float

    @property
    def usable(self) -> bool:
        return self.alignment > 0.0


class SectorWheelModel:
    """Measure, once per stance pose, how each sector frame can roll."""

    def __init__(
        self,
        kinematics: RobotKinematics,
        nominal_motor_degrees: ArrayLike,
        *,
        arc_lift_tolerance: float = 0.002,
        arc_scan_step: float = 2.0,
        arc_scan_limit: float = 280.0,
        steering_probe_degrees: float = 10.0,
        use_cache: bool = True,
    ) -> None:
        if arc_lift_tolerance <= 0.0:
            raise ValueError("arc_lift_tolerance must be positive")
        if not 0.0 < arc_scan_step <= 10.0:
            raise ValueError("arc_scan_step must be in (0, 10]")
        if arc_scan_limit <= arc_scan_step:
            raise ValueError("arc_scan_limit must exceed arc_scan_step")
        if not 0.0 < steering_probe_degrees <= 45.0:
            raise ValueError("steering_probe_degrees must be in (0, 45]")

        self.kinematics = kinematics
        self.nominal_motor_degrees = np.asarray(
            nominal_motor_degrees, dtype=np.float64
        ).copy()
        if self.nominal_motor_degrees.shape != (18,):
            raise ValueError("nominal_motor_degrees must contain actuator IDs 1..18")
        self.arc_lift_tolerance = float(arc_lift_tolerance)
        self.arc_scan_step = float(arc_scan_step)
        self.arc_scan_limit = float(arc_scan_limit)

        key = (
            f"{kinematics.model_path}|{self.arc_lift_tolerance}"
            f"|{self.arc_scan_step}|{self.arc_scan_limit}|{steering_probe_degrees}",
            np.round(self.nominal_motor_degrees, 6).tobytes(),
        )
        cached = _GEOMETRY_CACHE.get(key) if use_cache else None
        if cached is None:
            cached = {
                leg: self._measure(leg, steering_probe_degrees)
                for leg in range(1, 7)
            }
            if use_cache:
                _GEOMETRY_CACHE[key] = cached
        self.geometry: dict[int, WheelGeometry] = cached
        self.kinematics.forward_motor_degrees(
            self.nominal_motor_degrees, frame="body"
        )

    # -- measurement -----------------------------------------------------

    def _body_frame_axis(self, joint_id: int) -> tuple[Vector3, Vector3]:
        data = self.kinematics.data
        root_id = self.kinematics.legs[1].root_body_id
        world_from_body = data.xmat[root_id].reshape(3, 3)
        anchor = (data.xanchor[joint_id] - data.xpos[root_id]) @ world_from_body
        axis = data.xaxis[joint_id] @ world_from_body
        return anchor, axis

    def _rolling_vector(self, leg: int, motor_degrees: ArrayLike) -> Vector3:
        """Return ``axis x radius`` at the contact, in the body frame."""

        contact = active_support_point(self.kinematics, leg, motor_degrees)
        joint_id = int(self.kinematics.legs[leg].joint_ids[2])
        anchor, axis = self._body_frame_axis(joint_id)
        return np.cross(axis, contact - anchor)

    def _heading_and_radius(
        self,
        leg: int,
        motor_degrees: ArrayLike,
    ) -> tuple[float, float]:
        rolling = self._rolling_vector(leg, motor_degrees)
        radius = float(np.linalg.norm(rolling[:2]))
        if radius <= 1e-9:
            raise ValueError(
                f"leg {leg} sector axis is vertical; it cannot roll on the floor"
            )
        heading = math.degrees(math.atan2(float(rolling[1]), float(rolling[0])))
        return heading, radius

    def _arc_window(self, leg: int) -> tuple[float, float]:
        """Sweep stage-2 until the sector's open end lifts the contact."""

        nominal_height = float(
            active_support_point(self.kinematics, leg, self.nominal_motor_degrees)[2]
        )
        window: list[float] = []
        for direction in (-1.0, 1.0):
            reached = 0.0
            offset = self.arc_scan_step
            while offset <= self.arc_scan_limit:
                probe = self.nominal_motor_degrees.copy()
                probe[leg + 11] += direction * offset
                height = float(active_support_point(self.kinematics, leg, probe)[2])
                if height > nominal_height + self.arc_lift_tolerance:
                    break
                reached = offset
                offset += self.arc_scan_step
            window.append(direction * reached)
        return min(window), max(window)

    def _measure(self, leg: int, probe_degrees: float) -> WheelGeometry:
        heading, radius = self._heading_and_radius(leg, self.nominal_motor_degrees)

        steered = self.nominal_motor_degrees.copy()
        steered[leg - 1] += probe_degrees
        steered_heading, _ = self._heading_and_radius(leg, steered)
        gain = wrap_degrees(steered_heading - heading) / probe_degrees
        if abs(abs(gain) - 1.0) > 0.05:
            raise ValueError(
                f"leg {leg} upper joint is not a unit-gain steering column "
                f"(measured {gain:+.3f}); the model changed"
            )

        arc_min, arc_max = self._arc_window(leg)
        joint_id = int(self.kinematics.legs[leg].joint_ids[0])
        hip, _ = self._body_frame_axis(joint_id)
        contact = active_support_point(
            self.kinematics, leg, self.nominal_motor_degrees
        )
        return WheelGeometry(
            leg=leg,
            heading_degrees=heading,
            roll_radius=radius,
            steering_gain=float(np.sign(gain)),
            arc_min_degrees=arc_min,
            arc_max_degrees=arc_max,
            hip=np.asarray(hip[:2], dtype=np.float64),
            contact=np.asarray(contact[:2], dtype=np.float64),
        )

    # -- planning --------------------------------------------------------

    def solve(
        self,
        leg: int,
        travel: ArrayLike,
        *,
        max_steering_degrees: float,
    ) -> RollSolution:
        """Return the best steering, rate sign and alignment for one leg.

        ``travel`` is the body-frame velocity the ground contact has to have
        in order to satisfy the commanded body twist.  Rolling can only deliver
        it along the steered wheel heading, so the returned ``alignment`` is
        the cosine of whatever misalignment the steering limit leaves behind.
        """

        if max_steering_degrees < 0.0:
            raise ValueError("max_steering_degrees cannot be negative")
        vector = np.asarray(travel, dtype=np.float64)[:2]
        geometry = self.geometry[leg]
        speed = float(np.linalg.norm(vector))
        if speed <= 0.0:
            return RollSolution(0.0, geometry.heading_degrees, 0.0, 0.0)

        desired = math.degrees(math.atan2(float(vector[1]), float(vector[0])))
        best: RollSolution | None = None
        for rate_sign in (1.0, -1.0):
            base = geometry.heading_degrees + (0.0 if rate_sign > 0.0 else 180.0)
            requested = wrap_degrees(desired - base) / geometry.steering_gain
            applied = float(
                np.clip(requested, -max_steering_degrees, max_steering_degrees)
            )
            achieved = base + geometry.steering_gain * applied
            alignment = math.cos(math.radians(wrap_degrees(desired - achieved)))
            candidate = RollSolution(
                steering_degrees=applied,
                direction_degrees=wrap_degrees(achieved),
                rate_sign=rate_sign,
                alignment=max(0.0, alignment),
            )
            if (
                best is None
                or candidate.alignment > best.alignment + 1e-12
                or (
                    abs(candidate.alignment - best.alignment) <= 1e-12
                    and abs(candidate.steering_degrees) < abs(best.steering_degrees)
                )
            ):
                best = candidate
        assert best is not None
        return best

    def direction_unit(self, solution: RollSolution) -> Vector2:
        """Unit vector of the contact travel the solved roll actually produces."""

        angle = math.radians(solution.direction_degrees)
        return np.array([math.cos(angle), math.sin(angle)], dtype=np.float64)


__all__ = [
    "RollSolution",
    "SUPPORT_PATCH_DEPTH",
    "SectorWheelModel",
    "WheelGeometry",
    "active_support_point",
    "wrap_degrees",
]
