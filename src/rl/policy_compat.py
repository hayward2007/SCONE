"""Compatibility helpers for replaying saved SCONE PPO policies.

Training must keep using an environment with an exactly matching observation
space.  Replay is less restrictive: the two heading values added after the
original 68-value policy input live at the end of the current observation, so
old policies can safely receive the original prefix when they are inspected.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3 import PPO


LEGACY_OBSERVATION_SHAPE = (68,)
CURRENT_OBSERVATION_SHAPE = (70,)
# src.rl.walk_v2 adds six foot-contact flags and six normalised normal forces.
# Its policies cannot be replayed against a walk_learn environment, so the
# viewer picks the environment from the checkpoint rather than rejecting it.
V2_OBSERVATION_SHAPE = (82,)


def load_compatible_policy(
    path: Path | str,
    env: Any,
    device: str,
) -> PPO:
    """Load a policy for replay without binding its old space to ``env``."""

    policy = PPO.load(path, device=device)
    observation_shape = policy.observation_space.shape
    supported_shapes = {
        env.observation_space.shape,
        LEGACY_OBSERVATION_SHAPE,
    }
    if observation_shape not in supported_shapes:
        raise ValueError(
            "unsupported checkpoint observation shape: "
            f"{observation_shape}; expected one of {sorted(supported_shapes)}"
        )
    if policy.action_space.shape != env.action_space.shape:
        raise ValueError(
            "checkpoint action shape does not match this walk_learn.py: "
            f"{policy.action_space.shape} != {env.action_space.shape}"
        )
    return policy


def checkpoint_observation_shape(
    path: Path | str,
    device: str = "cpu",
) -> tuple[int, ...]:
    """Read a checkpoint's observation shape without binding it to an env."""

    return tuple(PPO.load(path, device=device).observation_space.shape)


def is_v2_checkpoint(shape: tuple[int, ...]) -> bool:
    return tuple(shape) == V2_OBSERVATION_SHAPE


def observation_for_policy(policy: PPO, observation: np.ndarray) -> np.ndarray:
    """Adapt the current observation to a replay-only legacy policy input."""

    expected_shape = policy.observation_space.shape
    if expected_shape == observation.shape:
        return observation
    if (
        expected_shape == LEGACY_OBSERVATION_SHAPE
        and observation.shape == CURRENT_OBSERVATION_SHAPE
    ):
        return observation[: LEGACY_OBSERVATION_SHAPE[0]]
    raise ValueError(
        f"cannot adapt observation {observation.shape} to policy {expected_shape}"
    )
