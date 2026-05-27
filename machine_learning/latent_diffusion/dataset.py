"""Dataset adapters for latent diffusion training."""

from __future__ import annotations

from pathlib import Path

from machine_learning.diffusion.dataset import PreprocessedGestureDataset


class LatentGestureDataset(PreprocessedGestureDataset):
    """Read latent LMDB windows written by ``build_latent_dataset.py``."""

    def __init__(self, data_dir: str | Path):
        super().__init__(data_dir)
        representation = self.metadata.get("target_representation")
        if representation != "gesture_latent":
            raise ValueError(
                f"Expected target_representation='gesture_latent', got {representation!r}"
            )


__all__ = ["LatentGestureDataset", "PreprocessedGestureDataset"]
