"""Conditional diffusion training for preprocessed gesture windows."""

__all__ = [
    "ConditionalMotionDenoiser",
    "DiffusionSchedule",
    "PreprocessedGestureDataset",
]


def __getattr__(name: str):
    if name == "PreprocessedGestureDataset":
        from .dataset import PreprocessedGestureDataset

        return PreprocessedGestureDataset
    if name in {"ConditionalMotionDenoiser", "DiffusionSchedule"}:
        from .model import ConditionalMotionDenoiser, DiffusionSchedule

        return {
            "ConditionalMotionDenoiser": ConditionalMotionDenoiser,
            "DiffusionSchedule": DiffusionSchedule,
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
