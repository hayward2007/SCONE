"""Reinforcement-learning API, loaded lazily to keep core imports light."""

__all__ = [
    "CURRICULUM_RANGES",
    "DEFAULT_MODEL_PATH",
    "OBSERVATION_COMMAND_SCALE",
    "RewardConfig",
    "SconeWalkEnv",
    "WalkConfig",
]


def __getattr__(name: str):
    if name in __all__:
        from . import walk_learn

        return getattr(walk_learn, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
