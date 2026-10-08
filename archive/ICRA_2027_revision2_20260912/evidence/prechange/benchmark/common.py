"""Shared simulation trial, metrics, perturbation, and output utilities."""

from __future__ import annotations

import csv
import json
import math
import subprocess
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterator, Mapping, Sequence
from unittest.mock import patch

import mujoco
import numpy as np

from src.hardware import Actuator
from src.main import SCONE
from src.simulation.core.controller import MuJoCoController
from src.simulation.core.model import DEFAULT_MODEL_PATH, load_model
from src.simulation.core.pid import spec_for_motor_id
from src.simulation.terrain import TerrainType

from .model_variants import (
    CONTACT_GEOMETRIES,
    model_fingerprint,
    transform_for_contact_geometry,
)


SCHEMA_VERSION = 1
COMMANDS: dict[str, tuple[float, float, float]] = {
    "idle": (0.0, 0.0, 0.0),
    "forward": (0.18, 0.0, 0.0),
    "reverse": (-0.18, 0.0, 0.0),
    "left": (0.0, 0.10, 0.0),
    "right": (0.0, -0.10, 0.0),
    "yaw-left": (0.0, 0.0, 0.60),
    "yaw-right": (0.0, 0.0, -0.60),
    "forward-turn": (0.14, 0.0, 0.45),
}


@dataclass(frozen=True)
class BenchmarkConfig:
    """Numerical protocol shared by flat and transition benchmarks."""

    control_dt: float = 0.02
    settle_seconds: float = 0.50
    measure_seconds: float = 8.0
    max_tilt_degrees: float = 60.0
    contact_force_threshold: float = 1.0
    physics_dt: float | None = None

    def __post_init__(self) -> None:
        if min(self.control_dt, self.measure_seconds) <= 0.0:
            raise ValueError("control_dt and measure_seconds must be positive")
        if self.settle_seconds < 0.0:
            raise ValueError("settle_seconds cannot be negative")
        if not 0.0 < self.max_tilt_degrees < 90.0:
            raise ValueError("max_tilt_degrees must be between 0 and 90")
        if self.contact_force_threshold < 0.0:
            raise ValueError("contact_force_threshold cannot be negative")
        if self.physics_dt is not None:
            if self.physics_dt <= 0.0:
                raise ValueError("physics_dt must be positive")
            if self.physics_dt > self.control_dt:
                raise ValueError("physics_dt cannot exceed control_dt")


@dataclass(frozen=True)
class Perturbation:
    """Physically interpretable model and initial-pose perturbations."""

    mass_scale: float = 1.0
    friction_scale: float = 1.0
    actuator_strength_scale: float = 1.0
    initial_x_m: float = 0.0
    initial_y_m: float = 0.0
    initial_yaw_degrees: float = 0.0
    gait_phase: float = 0.0

    def __post_init__(self) -> None:
        if min(
            self.mass_scale,
            self.friction_scale,
            self.actuator_strength_scale,
        ) <= 0.0:
            raise ValueError("mass, friction, and actuator scales must be positive")
        if not 0.0 <= self.gait_phase < 1.0:
            raise ValueError("gait_phase must be in [0, 1)")


@dataclass(frozen=True)
class TrialMetrics:
    """Controller-independent quantities measured over one trial window."""

    duration_s: float
    completed: bool
    termination_reason: str | None
    displacement_x_m: float
    displacement_y_m: float
    displacement_z_m: float
    yaw_change_degrees: float
    mean_vx_mps: float
    mean_vy_mps: float
    mean_yaw_rate_rps: float
    velocity_rmse_mps: float
    yaw_rate_rmse_rps: float
    lateral_drift_m: float
    backward_distance_m: float
    root_z_min_delta_m: float
    root_z_max_delta_m: float
    root_z_rms_delta_m: float
    roll_rms_degrees: float
    pitch_rms_degrees: float
    maximum_abs_roll_degrees: float
    maximum_abs_pitch_degrees: float
    minimum_upright: float
    absolute_mechanical_work_j: float
    estimated_absolute_electrical_energy_j: float
    mechanical_cost_of_transport: float | None
    peak_actuator_torque_nm: float
    peak_estimated_current_a: float
    peak_contact_force_n: float
    slip_distance_m: float
    mean_stance_contacts: float
    forbidden_collision_steps: int
    ik_failure_frames: int
    mean_stride_clip_fraction: float
    minimum_ik_backoff_scale: float


def source_revision() -> dict[str, str | bool | None]:
    """Return lightweight Git provenance without requiring a clean tree."""

    try:
        revision = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        )
        return {"git_revision": revision, "git_dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"git_revision": None, "git_dirty": None}


def apply_model_perturbation(
    model: mujoco.MjModel,
    perturbation: Perturbation,
) -> None:
    """Apply sensitivity parameters before ``MjData`` is constructed."""

    model.body_mass[1:] *= perturbation.mass_scale
    model.body_inertia[1:] *= perturbation.mass_scale
    model.geom_friction[:, 0] *= perturbation.friction_scale
    model.actuator_gainprm[:, 0] *= perturbation.actuator_strength_scale
    limited = np.asarray(model.actuator_forcelimited, dtype=bool)
    model.actuator_forcerange[limited] *= perturbation.actuator_strength_scale


class SimulationTrial:
    """Own one fresh MuJoCo model/data/controller/robot instance."""

    def __init__(
        self,
        *,
        terrain: TerrainType | str = TerrainType.FLAT,
        terrain_seed: int = 7,
        perturbation: Perturbation | None = None,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        profile: str = "standard",
        contact_geometry: str = "open-arc",
    ) -> None:
        self.terrain = TerrainType.parse(terrain)
        self.terrain_seed = int(terrain_seed)
        self.perturbation = perturbation or Perturbation()
        if contact_geometry not in CONTACT_GEOMETRIES:
            raise ValueError(
                f"unknown contact geometry {contact_geometry!r}; "
                f"choose from {CONTACT_GEOMETRIES}"
            )
        self.contact_geometry = contact_geometry
        self.model_path = Path(model_path).expanduser().resolve()
        self.model = load_model(
            self.model_path,
            floating_base=True,
            terrain=self.terrain,
            terrain_seed=self.terrain_seed,
            xml_transform=transform_for_contact_geometry(contact_geometry),
        )
        apply_model_perturbation(self.model, self.perturbation)
        self.data = mujoco.MjData(self.model)
        self.controller = MuJoCoController(self.model, self.data, verbose=False)
        self.robot = SCONE(self.controller, profile=profile)
        self.root_joint_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "root_freejoint",
        )
        self.root_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "UPPER_BODY_1",
        )
        if self.root_joint_id < 0 or self.root_body_id < 0:
            raise ValueError("benchmark requires root_freejoint and UPPER_BODY_1")
        self.elapsed = 0.0

    def close(self) -> None:
        self.controller.close()

    def __enter__(self) -> "SimulationTrial":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def initialize(self) -> None:
        """Run the real initialization sequence in simulation time."""

        with patch("time.sleep", side_effect=self.advance):
            self.robot.initialize()
        self._apply_initial_pose()
        self.advance(0.10)

    def _apply_initial_pose(self) -> None:
        qpos_address = int(self.model.jnt_qposadr[self.root_joint_id])
        self.data.qpos[qpos_address] += self.perturbation.initial_x_m
        self.data.qpos[qpos_address + 1] += self.perturbation.initial_y_m
        yaw = math.radians(self.perturbation.initial_yaw_degrees)
        if yaw != 0.0:
            yaw_quaternion = np.array(
                [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
                dtype=np.float64,
            )
            current = self.data.qpos[qpos_address + 3 : qpos_address + 7].copy()
            rotated = np.empty(4, dtype=np.float64)
            mujoco.mju_mulQuat(rotated, yaw_quaternion, current)
            self.data.qpos[qpos_address + 3 : qpos_address + 7] = rotated
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def advance(
        self,
        seconds: float,
        recorder: "MetricsRecorder | None" = None,
    ) -> None:
        if seconds < 0.0:
            raise ValueError("advance duration cannot be negative")
        steps = max(1, int(math.ceil(seconds / self.model.opt.timestep)))
        dt = float(self.model.opt.timestep)
        for _ in range(steps):
            self.controller.update(dt)
            mujoco.mj_step(self.model, self.data)
            self.elapsed += dt
            if recorder is not None:
                recorder.sample(dt)

    def wait_until_raw_positions(
        self,
        positions: Mapping[int, int],
        *,
        tolerance: int = 96,
        timeout: float = 4.0,
        recorder: "MetricsRecorder | None" = None,
    ) -> bool:
        steps = max(1, int(math.ceil(timeout / self.model.opt.timestep)))
        for _ in range(steps):
            reached = all(
                abs(self.controller.get_position(motor_id) - target) <= tolerance
                for motor_id, target in positions.items()
            )
            if reached:
                return True
            self.advance(self.model.opt.timestep, recorder)
        return False


class MetricsRecorder:
    """Accumulate paper-oriented state/contact/energy metrics."""

    def __init__(
        self,
        trial: SimulationTrial,
        command: Sequence[float],
        *,
        max_tilt_degrees: float = 60.0,
        contact_force_threshold: float = 1.0,
    ) -> None:
        parsed_command = np.asarray(command, dtype=np.float64)
        if parsed_command.shape != (3,):
            raise ValueError("command must contain vx, vy, yaw_rate")
        self.trial = trial
        self.model = trial.model
        self.data = trial.data
        self.controller = trial.controller
        self.command = parsed_command
        self.max_tilt_cosine = math.cos(math.radians(max_tilt_degrees))
        self.contact_force_threshold = contact_force_threshold
        self.start_position = self.data.xpos[trial.root_body_id].copy()
        self.start_rotation = self.data.xmat[trial.root_body_id].reshape(3, 3).copy()
        self.previous_position = self.start_position.copy()
        self.previous_yaw = self._yaw(self.start_rotation)
        self.elapsed = 0.0
        self.yaw_change = 0.0
        self.velocity_error_squared = 0.0
        self.yaw_error_squared = 0.0
        self.root_z_squared = 0.0
        self.roll_squared = 0.0
        self.pitch_squared = 0.0
        self.sample_count = 0
        self.minimum_z_delta = 0.0
        self.maximum_z_delta = 0.0
        self.minimum_upright = 1.0
        self.maximum_abs_roll = 0.0
        self.maximum_abs_pitch = 0.0
        self.backward_distance = 0.0
        self.absolute_mechanical_work = 0.0
        self.estimated_absolute_electrical_energy = 0.0
        self.peak_actuator_torque = 0.0
        self.peak_estimated_current = 0.0
        self.peak_contact_force = 0.0
        self.slip_distance = 0.0
        self.stance_contact_sum = 0.0
        self.forbidden_collision_steps = 0
        self.ik_failure_frames = 0
        self.stride_clip_sum = 0.0
        self.control_frame_count = 0
        self.minimum_ik_backoff_scale = 1.0
        self.termination_reason: str | None = None
        self._contact_force = np.zeros(6, dtype=np.float64)
        self._jacobian_position = np.zeros((3, self.model.nv), dtype=np.float64)
        self._jacobian_rotation = np.zeros((3, self.model.nv), dtype=np.float64)
        self.ground_geom_ids = {
            geom_id
            for geom_id in range(self.model.ngeom)
            if self._is_ground_geom(geom_id)
        }
        self.tire_geom_ids = {
            mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                f"TIRE_{leg}_geom",
            )
            for leg in range(1, 7)
        }
        self.tire_geom_ids.discard(-1)

    def _is_ground_geom(self, geom_id: int) -> bool:
        name = mujoco.mj_id2name(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            geom_id,
        )
        return bool(
            int(self.model.geom_bodyid[geom_id]) == 0
            and name
            and (name == "simulation_floor" or name.startswith("terrain_"))
        )

    @staticmethod
    def _yaw(rotation: np.ndarray) -> float:
        return float(math.atan2(rotation[1, 0], rotation[0, 0]))

    @staticmethod
    def _roll_pitch(rotation: np.ndarray) -> tuple[float, float]:
        pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        return roll, pitch

    @staticmethod
    def _wrapped_delta(current: float, previous: float) -> float:
        return math.atan2(math.sin(current - previous), math.cos(current - previous))

    def record_control(
        self,
        *,
        converged: bool = True,
        stride_clip_fraction: float = 0.0,
        ik_backoff_scale: float = 1.0,
    ) -> None:
        self.control_frame_count += 1
        if not converged:
            self.ik_failure_frames += 1
        self.stride_clip_sum += float(stride_clip_fraction)
        self.minimum_ik_backoff_scale = min(
            self.minimum_ik_backoff_scale,
            float(ik_backoff_scale),
        )

    def sample(self, dt: float) -> None:
        position = self.data.xpos[self.trial.root_body_id].copy()
        rotation = self.data.xmat[self.trial.root_body_id].reshape(3, 3).copy()
        delta_world = position - self.previous_position
        body_velocity = rotation.T @ (delta_world / dt)
        forward_delta = float((self.start_rotation.T @ delta_world)[0])
        self.backward_distance += max(0.0, -forward_delta)

        yaw = self._yaw(rotation)
        yaw_delta = self._wrapped_delta(yaw, self.previous_yaw)
        yaw_rate = yaw_delta / dt
        self.yaw_change += yaw_delta
        self.previous_yaw = yaw
        self.previous_position = position

        velocity_error = body_velocity[:2] - self.command[:2]
        self.velocity_error_squared += float(velocity_error @ velocity_error)
        self.yaw_error_squared += float((yaw_rate - self.command[2]) ** 2)

        relative_rotation = self.start_rotation.T @ rotation
        roll, pitch = self._roll_pitch(relative_rotation)
        z_delta = float(position[2] - self.start_position[2])
        upright = float(rotation[2, 2])
        self.minimum_z_delta = min(self.minimum_z_delta, z_delta)
        self.maximum_z_delta = max(self.maximum_z_delta, z_delta)
        self.minimum_upright = min(self.minimum_upright, upright)
        self.maximum_abs_roll = max(self.maximum_abs_roll, abs(roll))
        self.maximum_abs_pitch = max(self.maximum_abs_pitch, abs(pitch))
        self.root_z_squared += z_delta * z_delta
        self.roll_squared += roll * roll
        self.pitch_squared += pitch * pitch

        for motor_id in Actuator.Index.ALL:
            actuator_id = int(self.controller._actuator_ids[motor_id])
            dof_id = int(self.controller._dof_addresses[motor_id])
            torque = float(self.data.actuator_force[actuator_id])
            velocity = float(self.data.qvel[dof_id])
            voltage = float(self.data.ctrl[actuator_id])
            spec = spec_for_motor_id(motor_id)
            current = (voltage - spec.K * velocity) / spec.R
            self.absolute_mechanical_work += abs(torque * velocity) * dt
            self.estimated_absolute_electrical_energy += abs(voltage * current) * dt
            self.peak_actuator_torque = max(self.peak_actuator_torque, abs(torque))
            self.peak_estimated_current = max(
                self.peak_estimated_current,
                abs(current),
            )

        contacts, slip_speed, forbidden = self._contact_metrics()
        self.stance_contact_sum += contacts
        self.slip_distance += slip_speed * dt
        if forbidden:
            self.forbidden_collision_steps += 1

        if self.termination_reason is None:
            if not (
                np.all(np.isfinite(self.data.qpos))
                and np.all(np.isfinite(self.data.qvel))
            ):
                self.termination_reason = "non-finite-state"
            elif upright < self.max_tilt_cosine:
                self.termination_reason = "tilt-limit"

        self.sample_count += 1
        self.elapsed += dt

    def _contact_metrics(self) -> tuple[int, float, bool]:
        loaded_tire_bodies: set[int] = set()
        tangential_speeds: list[float] = []
        forbidden = False
        for contact_id in range(self.data.ncon):
            contact = self.data.contact[contact_id]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            ground = geom1 if geom1 in self.ground_geom_ids else geom2
            if ground not in self.ground_geom_ids:
                continue
            robot_geom = geom2 if ground == geom1 else geom1
            mujoco.mj_contactForce(
                self.model,
                self.data,
                contact_id,
                self._contact_force,
            )
            force = float(np.linalg.norm(self._contact_force[:3]))
            self.peak_contact_force = max(self.peak_contact_force, force)
            if robot_geom not in self.tire_geom_ids:
                if int(self.model.geom_bodyid[robot_geom]) != 0:
                    forbidden = True
                continue
            if abs(float(self._contact_force[0])) < self.contact_force_threshold:
                continue
            body_id = int(self.model.geom_bodyid[robot_geom])
            loaded_tire_bodies.add(body_id)
            mujoco.mj_jac(
                self.model,
                self.data,
                self._jacobian_position,
                self._jacobian_rotation,
                contact.pos,
                body_id,
            )
            point_velocity = self._jacobian_position @ self.data.qvel
            normal = np.asarray(contact.frame[:3], dtype=np.float64)
            tangent = point_velocity - normal * float(point_velocity @ normal)
            tangential_speeds.append(float(np.linalg.norm(tangent)))
        mean_slip = float(np.mean(tangential_speeds)) if tangential_speeds else 0.0
        return len(loaded_tire_bodies), mean_slip, forbidden

    def finalize(self) -> TrialMetrics:
        if self.sample_count == 0 or self.elapsed <= 0.0:
            raise RuntimeError("cannot finalize an empty benchmark window")
        final_position = self.data.xpos[self.trial.root_body_id]
        displacement = self.start_rotation.T @ (final_position - self.start_position)
        horizontal_distance = float(np.linalg.norm(displacement[:2]))
        mass = float(np.sum(self.model.body_mass))
        cost = (
            self.absolute_mechanical_work / (mass * 9.81 * horizontal_distance)
            if horizontal_distance > 1e-3
            else None
        )
        return TrialMetrics(
            duration_s=self.elapsed,
            completed=self.termination_reason is None,
            termination_reason=self.termination_reason,
            displacement_x_m=float(displacement[0]),
            displacement_y_m=float(displacement[1]),
            displacement_z_m=float(displacement[2]),
            yaw_change_degrees=math.degrees(self.yaw_change),
            mean_vx_mps=float(displacement[0]) / self.elapsed,
            mean_vy_mps=float(displacement[1]) / self.elapsed,
            mean_yaw_rate_rps=self.yaw_change / self.elapsed,
            velocity_rmse_mps=math.sqrt(
                self.velocity_error_squared / self.sample_count
            ),
            yaw_rate_rmse_rps=math.sqrt(self.yaw_error_squared / self.sample_count),
            lateral_drift_m=abs(float(displacement[1])),
            backward_distance_m=self.backward_distance,
            root_z_min_delta_m=self.minimum_z_delta,
            root_z_max_delta_m=self.maximum_z_delta,
            root_z_rms_delta_m=math.sqrt(self.root_z_squared / self.sample_count),
            roll_rms_degrees=math.degrees(
                math.sqrt(self.roll_squared / self.sample_count)
            ),
            pitch_rms_degrees=math.degrees(
                math.sqrt(self.pitch_squared / self.sample_count)
            ),
            maximum_abs_roll_degrees=math.degrees(self.maximum_abs_roll),
            maximum_abs_pitch_degrees=math.degrees(self.maximum_abs_pitch),
            minimum_upright=self.minimum_upright,
            absolute_mechanical_work_j=self.absolute_mechanical_work,
            estimated_absolute_electrical_energy_j=(
                self.estimated_absolute_electrical_energy
            ),
            mechanical_cost_of_transport=cost,
            peak_actuator_torque_nm=self.peak_actuator_torque,
            peak_estimated_current_a=self.peak_estimated_current,
            peak_contact_force_n=self.peak_contact_force,
            slip_distance_m=self.slip_distance,
            mean_stance_contacts=self.stance_contact_sum / self.sample_count,
            forbidden_collision_steps=self.forbidden_collision_steps,
            ik_failure_frames=self.ik_failure_frames,
            mean_stride_clip_fraction=(
                self.stride_clip_sum / self.control_frame_count
                if self.control_frame_count
                else 0.0
            ),
            minimum_ik_backoff_scale=self.minimum_ik_backoff_scale,
        )


def make_record(
    *,
    benchmark: str,
    controller: str,
    command_name: str,
    command: Sequence[float],
    trial_index: int,
    seed: int,
    terrain: str,
    perturbation: Perturbation,
    metrics: TrialMetrics,
    extra: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    record: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "benchmark": benchmark,
        "controller": controller,
        "command_name": command_name,
        "command_vx_mps": float(command[0]),
        "command_vy_mps": float(command[1]),
        "command_yaw_rate_rps": float(command[2]),
        "trial_index": int(trial_index),
        "seed": int(seed),
        "terrain": terrain,
        **asdict(perturbation),
        **asdict(metrics),
        **source_revision(),
    }
    if extra:
        record.update(extra)
    return record


def simulation_provenance(
    *,
    model_path: str | Path = DEFAULT_MODEL_PATH,
    protocol_version: str | None = None,
) -> dict[str, Any]:
    """Return immutable model/runtime identifiers for publication records."""

    provenance: dict[str, Any] = {
        "model_sha256": model_fingerprint(Path(model_path).expanduser().resolve()),
        "mujoco_version": getattr(mujoco, "__version__", "unknown"),
    }
    if protocol_version is not None:
        provenance["protocol_version"] = protocol_version
    return provenance


def write_records(records: Sequence[Mapping[str, Any]], output: str | Path) -> Path:
    path = Path(output).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix.lower() == ".csv":
        fieldnames = sorted({key for record in records for key in record})
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(records)
    else:
        with path.open("w", encoding="utf-8") as stream:
            for record in records:
                stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True))
                stream.write("\n")
    return path


def print_summary(records: Sequence[Mapping[str, Any]]) -> None:
    grouped: dict[tuple[str, str], list[Mapping[str, Any]]] = {}
    for record in records:
        key = (str(record["controller"]), str(record["command_name"]))
        grouped.setdefault(key, []).append(record)
    print("controller,command,N,success,mean_vx,mean_work,mean_upright")
    for (controller, command_name), rows in sorted(grouped.items()):
        success = np.mean([bool(row.get("completed", row.get("top_reached"))) for row in rows])
        velocities = [float(row["mean_vx_mps"]) for row in rows if "mean_vx_mps" in row]
        work = [
            float(row[key])
            for row in rows
            for key in ("absolute_mechanical_work_j", "work_to_top_j")
            if row.get(key) is not None
        ]
        upright = [
            float(row[key])
            for row in rows
            for key in ("minimum_upright", "minimum_upright_to_top")
            if row.get(key) is not None
        ]
        print(
            f"{controller},{command_name},{len(rows)},{success:.3f},"
            f"{np.mean(velocities) if velocities else math.nan:.5f},"
            f"{np.mean(work) if work else math.nan:.3f},"
            f"{np.mean(upright) if upright else math.nan:.4f}"
        )


@contextmanager
def temporary_stair_profile(
    terrain: TerrainType,
    profile: Any,
) -> Iterator[None]:
    """Temporarily replace one preset dictionary entry for a custom sweep."""

    from src.simulation.terrain import STAIR_PRESETS

    previous = STAIR_PRESETS[terrain]
    STAIR_PRESETS[terrain] = profile
    try:
        yield
    finally:
        STAIR_PRESETS[terrain] = previous


__all__ = [
    "BenchmarkConfig",
    "COMMANDS",
    "MetricsRecorder",
    "Perturbation",
    "SCHEMA_VERSION",
    "SimulationTrial",
    "TrialMetrics",
    "apply_model_perturbation",
    "make_record",
    "print_summary",
    "source_revision",
    "simulation_provenance",
    "temporary_stair_profile",
    "write_records",
]
