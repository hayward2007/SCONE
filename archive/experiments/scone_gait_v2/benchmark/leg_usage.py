"""Per-leg contact accounting: who actually carries and pushes the robot.

The flat benchmark reports whole-robot outcomes.  This one answers a different
question -- how the work is divided between the six legs -- because gait design
choices are often made on a claim about one leg pulling its weight or not.

For every physics step it resolves each tire/floor contact into a world force,
orients it so the floor is pushing the robot up, and accumulates three things
per leg: how long the foot was down, the vertical impulse it carried, and the
fore-aft impulse it delivered.  Support share comes from the vertical impulse;
propulsion share comes from the positive part of the fore-aft impulse, with the
braking part reported separately, since a leg can be busy and still be a drag.

Fore-aft is the chassis's own forward axis, resolved every step, not a world
axis: the model's body frame is not aligned with the world frame, and a
controller that veers would otherwise have its propulsion credited to the wrong
component.

    python -m benchmark.leg_usage --controller articulated-walk
    python -m benchmark.leg_usage --controller role-split-scone --command forward
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Sequence

import mujoco
import numpy as np

from src.locomotion import VelocityCommand
from src.simulation.terrain import TerrainType

from .common import (
    BenchmarkConfig,
    COMMANDS,
    Perturbation,
    SimulationTrial,
)
from .controllers import CONTROLLER_CHOICES, make_controller


@dataclass(frozen=True)
class LegUsage:
    """One leg's share of the work over a measurement window."""

    leg: int
    contact_fraction: float
    vertical_impulse_ns: float
    forward_impulse_ns: float
    braking_impulse_ns: float

    @property
    def net_forward_impulse_ns(self) -> float:
        return self.forward_impulse_ns - self.braking_impulse_ns


class LegContactRecorder:
    """Accumulate per-leg contact time and impulse, one physics step at a time.

    ``SimulationTrial.advance`` calls ``sample(dt)`` after every step, which is
    the only hook into the physics loop, so this deliberately matches the
    ``MetricsRecorder`` interface rather than introducing another one.
    """

    def __init__(self, trial: SimulationTrial) -> None:
        self.trial = trial
        self.model = trial.model
        self.data = trial.data
        self.elapsed = 0.0
        self.contact_time = np.zeros(6, dtype=np.float64)
        self.vertical_impulse = np.zeros(6, dtype=np.float64)
        self.forward_impulse = np.zeros(6, dtype=np.float64)
        self.braking_impulse = np.zeros(6, dtype=np.float64)
        self._geom_leg: dict[int, int] = {}
        for leg in range(1, 7):
            geom_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                f"TIRE_{leg}_geom",
            )
            if geom_id < 0:
                raise ValueError(f"model is missing TIRE_{leg}_geom")
            self._geom_leg[geom_id] = leg
        self._force = np.zeros(6, dtype=np.float64)

    def _forward_axis(self) -> np.ndarray:
        rotation = self.data.xmat[self.trial.root_body_id].reshape(3, 3)
        axis = rotation[:, 0].copy()
        axis[2] = 0.0
        norm = float(np.linalg.norm(axis))
        if norm <= 1e-9:
            raise ValueError("chassis forward axis is vertical")
        return axis / norm

    def sample(self, dt: float) -> None:
        self.elapsed += dt
        forward = self._forward_axis()
        touched = set()
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            leg = self._geom_leg.get(int(contact.geom1))
            if leg is None:
                leg = self._geom_leg.get(int(contact.geom2))
            if leg is None:
                continue
            mujoco.mj_contactForce(self.model, self.data, index, self._force)
            frame = np.asarray(contact.frame, dtype=np.float64).reshape(3, 3)
            world = frame.T @ self._force[:3]
            # mj_contactForce orients the normal by geom order, which varies.
            # The floor can only push the robot up, so use that to fix the sign
            # rather than tracking which geom landed first.
            if world[2] < 0.0:
                world = -world
            touched.add(leg)
            self.vertical_impulse[leg - 1] += float(world[2]) * dt
            longitudinal = float(np.dot(world[:2], forward[:2])) * dt
            if longitudinal >= 0.0:
                self.forward_impulse[leg - 1] += longitudinal
            else:
                self.braking_impulse[leg - 1] -= longitudinal
        for leg in touched:
            self.contact_time[leg - 1] += dt

    def finalize(self) -> tuple[LegUsage, ...]:
        window = max(self.elapsed, 1e-12)
        return tuple(
            LegUsage(
                leg=leg,
                contact_fraction=float(self.contact_time[leg - 1] / window),
                vertical_impulse_ns=float(self.vertical_impulse[leg - 1]),
                forward_impulse_ns=float(self.forward_impulse[leg - 1]),
                braking_impulse_ns=float(self.braking_impulse[leg - 1]),
            )
            for leg in range(1, 7)
        )


def measure_leg_usage(
    controller_name: str,
    command: Sequence[float],
    *,
    terrain: TerrainType | str = TerrainType.FLAT,
    terrain_seed: int = 7,
    perturbation: Perturbation | None = None,
    config: BenchmarkConfig | None = None,
    contact_geometry: str = "open-arc",
) -> dict[str, object]:
    """Run one headless trial and return the per-leg division of labour."""

    selected = config or BenchmarkConfig()
    parsed = np.asarray(command, dtype=np.float64)
    if parsed.shape != (3,):
        raise ValueError("command must contain vx, vy, yaw_rate")

    with SimulationTrial(
        terrain=terrain,
        terrain_seed=terrain_seed,
        perturbation=perturbation or Perturbation(),
        contact_geometry=contact_geometry,
    ) as trial:
        trial.model.opt.timestep = (
            selected.physics_dt
            if selected.physics_dt is not None
            else min(float(trial.model.opt.timestep), selected.control_dt)
        )
        trial.initialize()
        controller = make_controller(
            controller_name,
            trial,
            phase=(perturbation or Perturbation()).gait_phase,
        )
        controller.prepare(trial)

        neutral = VelocityCommand()
        for _ in range(round(selected.settle_seconds / selected.control_dt)):
            controller.update(neutral, selected.control_dt)
            trial.advance(selected.control_dt)

        qpos_address = int(trial.model.jnt_qposadr[trial.root_joint_id])
        start = trial.data.qpos[qpos_address : qpos_address + 3].copy()
        start_yaw = _yaw(trial)
        start_rotation = trial.data.xmat[trial.root_body_id].reshape(3, 3).copy()

        recorder = LegContactRecorder(trial)
        velocity = VelocityCommand.from_array(parsed)
        for _ in range(round(selected.measure_seconds / selected.control_dt)):
            controller.update(velocity, selected.control_dt)
            trial.advance(selected.control_dt, recorder)
        controller.stop()

        # Report travel in the chassis frame the run started in, so "forward"
        # means the same thing as the command did.
        displacement = start_rotation.T @ (
            trial.data.qpos[qpos_address : qpos_address + 3].copy() - start
        )
        yaw_change = math.degrees(_yaw(trial) - start_yaw)
        usage = recorder.finalize()

    return {
        "controller": controller_name,
        "command": parsed.tolist(),
        "duration_s": recorder.elapsed,
        "displacement_x_m": float(displacement[0]),
        "displacement_y_m": float(displacement[1]),
        "yaw_change_degrees": yaw_change,
        "legs": usage,
    }


def _yaw(trial: SimulationTrial) -> float:
    rotation = trial.data.xmat[trial.root_body_id].reshape(3, 3)
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def format_report(record: dict[str, object]) -> str:
    legs: tuple[LegUsage, ...] = record["legs"]  # type: ignore[assignment]
    vertical_total = sum(leg.vertical_impulse_ns for leg in legs) or 1.0
    forward_total = sum(leg.forward_impulse_ns for leg in legs) or 1.0
    lines = [
        f"{record['controller']}  command={record['command']}  "
        f"{record['duration_s']:.1f} s  "
        f"forward={record['displacement_x_m']:+.3f} m  "
        f"lateral={record['displacement_y_m']:+.3f} m  "
        f"yaw={record['yaw_change_degrees']:+.1f} deg",
        f"{'leg':>4} {'contact':>9} {'support':>9} {'push':>9} {'brake':>9} "
        f"{'net push':>10}",
    ]
    for leg in legs:
        lines.append(
            f"{leg.leg:>4} {leg.contact_fraction * 100:>8.1f}% "
            f"{leg.vertical_impulse_ns / vertical_total * 100:>8.1f}% "
            f"{leg.forward_impulse_ns / forward_total * 100:>8.1f}% "
            f"{leg.braking_impulse_ns:>8.3f}N "
            f"{leg.net_forward_impulse_ns:>9.3f}N"
        )
    return "\n".join(lines)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="report how six legs divide support and propulsion",
    )
    parser.add_argument(
        "--controller",
        choices=CONTROLLER_CHOICES,
        default="articulated-walk",
    )
    parser.add_argument("--command", choices=tuple(COMMANDS), default="forward")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--gait-phase", type=float, default=0.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _build_parser().parse_args(argv)
    record = measure_leg_usage(
        arguments.controller,
        COMMANDS[arguments.command],
        perturbation=Perturbation(gait_phase=arguments.gait_phase),
        config=BenchmarkConfig(measure_seconds=arguments.seconds),
    )
    print(format_report(record))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "LegContactRecorder",
    "LegUsage",
    "format_report",
    "measure_leg_usage",
]
