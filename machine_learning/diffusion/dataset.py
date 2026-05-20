"""
Dataset adapters for preprocessed gesture training data.

The current preprocessing pipeline writes LMDB windows with keys containing
``x`` conditioning features and ``y`` motion targets. Older notes in the repo
also mention ``window_*_x.pt`` / ``window_*_y.pt`` files, so this module
supports both layouts behind one Dataset API.
"""

from __future__ import annotations

import json
import pickle
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.utils.data import Dataset


DEFAULT_METADATA = {
    "input_dim": 1536,
    "target_shape": [12, 3],
    "target_type": "genea_root_relative_xyz",
}


def _as_float_tensor(value: Any) -> torch.Tensor:
    if isinstance(value, torch.Tensor):
        return value.float()
    if isinstance(value, np.ndarray):
        return torch.from_numpy(value.astype(np.float32, copy=False))
    return torch.as_tensor(value, dtype=torch.float32)


class PreprocessedGestureDataset(Dataset):
    """Read preprocessed gesture windows from LMDB or legacy paired PT files."""

    def __init__(self, data_dir: str | Path):
        self.data_dir = Path(data_dir)
        if not self.data_dir.exists():
            raise FileNotFoundError(f"Dataset path does not exist: {self.data_dir}")

        self.kind = "lmdb" if self._looks_like_lmdb(self.data_dir) else "pt"
        self._env = None
        self.metadata = dict(DEFAULT_METADATA)

        if self.kind == "lmdb":
            self._init_lmdb()
        else:
            self._init_pt_files()

    @staticmethod
    def _looks_like_lmdb(path: Path) -> bool:
        if not path.is_dir():
            return False
        if (path / "data.mdb").exists() or (path / "lock.mdb").exists():
            return True
        return path.suffix == ".lmdb"

    def _open_lmdb(self):
        try:
            import lmdb
        except ImportError as exc:
            raise ImportError(
                "The lmdb package is required to read preprocessed LMDB data."
            ) from exc
        return lmdb.open(
            str(self.data_dir),
            readonly=True,
            lock=False,
            readahead=False,
            meminit=False,
            max_readers=2048,
        )

    def _init_lmdb(self):
        env = self._open_lmdb()
        with env.begin() as txn:
            length_bytes = txn.get(b"__len__")
            if length_bytes is None:
                raise ValueError(f"No b'__len__' key in LMDB at {self.data_dir}")
            self.length = int(pickle.loads(length_bytes))

            metadata_bytes = txn.get(b"__metadata__")
            if metadata_bytes is not None:
                self.metadata.update(pickle.loads(metadata_bytes))

            if metadata_bytes is None:
                sidecar = self.data_dir.parent / "metadata.json"
                if sidecar.is_file():
                    with open(sidecar, "r") as f:
                        self.metadata.update(json.load(f))

            first = txn.get(b"00000000")
            if first is not None and self.length > 0:
                raw = pickle.loads(first)
                self.metadata.update({
                    "input_dim": int(raw["x"].shape[-1]),
                    "target_shape": list(raw["y"].shape[1:]),
                    "window_frames": int(raw["x"].shape[0]),
                })
        env.close()

    def _init_pt_files(self):
        self.x_files = sorted(self.data_dir.glob("window_*_x.pt"))
        if not self.x_files:
            raise ValueError(
                f"{self.data_dir} is neither an LMDB dataset nor a legacy PT window directory"
            )

        self.y_files = []
        for x_path in self.x_files:
            y_path = x_path.with_name(x_path.name.replace("_x.pt", "_y.pt"))
            if not y_path.is_file():
                raise FileNotFoundError(f"Missing target file for {x_path.name}: {y_path}")
            self.y_files.append(y_path)

        self.length = len(self.x_files)
        x0 = _as_float_tensor(torch.load(self.x_files[0], map_location="cpu"))
        y0 = _as_float_tensor(torch.load(self.y_files[0], map_location="cpu"))
        self.metadata.update({
            "input_dim": int(x0.shape[-1]),
            "target_shape": list(y0.shape[1:]),
            "window_frames": int(x0.shape[0]),
            "target_type": "legacy_pt_windows",
        })

    def _get_env(self):
        if self._env is None:
            self._env = self._open_lmdb()
        return self._env

    def __len__(self) -> int:
        return self.length

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor]:
        if self.kind == "lmdb":
            with self._get_env().begin() as txn:
                value = txn.get(f"{idx:08d}".encode())
            if value is None:
                raise KeyError(f"Key {idx:08d} not found in {self.data_dir}")
            raw = pickle.loads(value)
            return _as_float_tensor(raw["x"]), _as_float_tensor(raw["y"])

        return (
            _as_float_tensor(torch.load(self.x_files[idx], map_location="cpu")),
            _as_float_tensor(torch.load(self.y_files[idx], map_location="cpu")),
        )
