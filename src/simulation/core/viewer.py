"""Shared MuJoCo viewer presentation helpers."""

from __future__ import annotations

from typing import Any

import mujoco


def configure_simulation_viewer(
    viewer: Any,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    tracking_body_id: int,
) -> None:
    """Show terrain clearly while keeping the robot at a useful scale.

    Long courses, especially ``mixed``, make ``model.stat.extent`` much larger
    than the robot. Capping the initial distance prevents the viewer from
    zooming out until both the robot and obstacles become tiny. Tracking mode
    then follows the robot through the course without changing the physics.
    """

    if not 0 <= tracking_body_id < model.nbody:
        raise ValueError(f"invalid tracking body id: {tracking_body_id}")

    viewer.opt.geomgroup[0] = 1
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = tracking_body_id
    viewer.cam.lookat[:] = data.xpos[tracking_body_id]
    viewer.cam.distance = min(max(float(model.stat.extent) * 2.2, 2.2), 3.0)
    viewer.cam.azimuth = 135.0
    viewer.cam.elevation = -30.0


__all__ = ["configure_simulation_viewer"]
