"""Opt-in simulation target governor; never changes the hardware control path.

Limits are based on 12 V no-load data, not identified loaded capabilities.
Integer position counts and the inner profile limit are both accounted for.
"""
from __future__ import annotations
import math
import numpy as np
from src.simulation.core.controller import MuJoCoController
from src.simulation.core.pid import spec_for_motor_id

DEGREES_PER_COUNT = 360.0 / 4096.0
NO_LOAD_DPS = np.array([math.degrees(spec_for_motor_id(i).no_load_speed)
                        for i in range(1, 19)])


class JointTargetGovernor:
    """Move along each requested joint segment without exceeding count budgets.

The caller may hold the gait clock until ``at_target``. Rate-only mode sends
new requested targets every wall-clock tick. Neither mode is a dynamics or
contact-feasibility controller; profile and tracking errors remain measurable.
    """
    def __init__(self, initial_raw, scale: float, dt: float = .02):
        if not np.isfinite(scale) or not 0 < scale <= 1:
            raise ValueError('scale must be in (0,1]')
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be positive')
        self.dt = dt
        self.limits_dps = NO_LOAD_DPS * scale
        self.count_budget = np.floor(self.limits_dps * dt / DEGREES_PER_COUNT + 1e-10).astype(int)
        if np.any(self.count_budget < 1):
            raise ValueError('dt and scale do not allow one encoder count per update')
        self.raw = self._validate_raw(initial_raw).copy()
        self.at_target = True
        self.last_alpha = 1.0

    @staticmethod
    def _validate_raw(raw):
        arr = np.asarray(raw)
        if arr.shape != (18,) or not np.isfinite(arr).all():
            raise ValueError('expected 18 finite raw positions')
        if np.any(arr != np.floor(arr)) or np.any(arr < 0) or np.any(arr > 4096):
            raise ValueError('raw positions must be integers in [0,4096]')
        return arr.astype(int)

    def advance(self, desired_raw):
        desired = self._validate_raw(desired_raw)
        delta = desired - self.raw
        moving = np.abs(delta) > 0
        self.last_alpha = min(1.0, float(np.min(self.count_budget[moving] / np.abs(delta[moving])))) if moving.any() else 1.0
        if self.last_alpha == 1.0:
            increment = delta
        else:
            increment = np.sign(delta) * np.floor(np.abs(delta) * self.last_alpha + 1e-10).astype(int)
        self.raw += increment
        self.at_target = bool(np.array_equal(self.raw, desired))
        return self.raw.copy()

    def configure_inner_profile(self, controller):
        """Bound the simulated 2 ms setpoint as well as the 20 ms goal.

Uses public profile setters; zero is prohibited because it means unlimited.
        """
        effective = []
        for i, limit in enumerate(self.limits_dps, 1):
            unit = math.degrees(controller._speed_to_radians_per_second(i, 1))
            raw_speed = int(math.floor(limit / unit + 1e-10))
            if raw_speed < 1:
                raise ValueError('profile speed quantizes to unlimited zero')
            controller.set_speed(i, raw_speed)
            effective.append(raw_speed * unit)
        return np.array(effective)


def degrees_to_counts(degrees):
    values = np.asarray(degrees, dtype=float)
    if values.shape != (18,) or not np.isfinite(values).all() or np.any(values < 0) or np.any(values > 360):
        raise ValueError('expected 18 finite motor angles in [0,360]')
    return np.array([MuJoCoController.degrees_to_raw(i, value) for i, value in enumerate(values, 1)])
