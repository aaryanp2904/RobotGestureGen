"""Dataset adapters for latent diffusion training."""

from __future__ import annotations

import bisect
import json
import pickle
import shutil
from pathlib import Path
from typing import Any

import numpy as np
import torch

from machine_learning.diffusion.dataset import PreprocessedGestureDataset


def _as_float_tensor(value: Any) -> torch.Tensor:
    if isinstance(value, torch.Tensor):
        return value.float()
    if isinstance(value, np.ndarray):
        return torch.from_numpy(value.astype(np.float32, copy=False))
    return torch.as_tensor(value, dtype=torch.float32)


def _metadata_list(value: Any) -> list:
    if value is None:
        return []
    return list(value)


def validate_source_metadata(metadata: dict, source: PreprocessedGestureDataset, length: int, data_dir: Path):
    if len(source) != length:
        raise ValueError(
            f"Source dataset {source.data_dir} has {len(source)} samples, "
            f"but latent dataset {data_dir} has {length}"
        )
    checks = [
        ("input_dim", int(metadata.get("input_dim", 0) or 0), int(source.metadata.get("input_dim", 0) or 0)),
        (
            "prosody_dim",
            int(metadata.get("prosody_dim", 0) or 0),
            int(source.metadata.get("prosody_dim", 0) or 0),
        ),
        (
            "wavlm_dim",
            int(metadata.get("wavlm_dim", 0) or 0),
            int(source.metadata.get("wavlm_dim", 0) or 0),
        ),
        (
            "text_dim",
            int(metadata.get("text_dim", 0) or 0),
            int(source.metadata.get("text_dim", 0) or 0),
        ),
        (
            "gesture_energy_dim",
            int(metadata.get("gesture_energy_dim", 0) or 0),
            int(source.metadata.get("gesture_energy_dim", 0) or 0),
        ),
        (
            "speaker_dim",
            int(metadata.get("speaker_dim", 0) or 0),
            int(source.metadata.get("speaker_dim", 0) or 0),
        ),
        (
            "window_frames",
            int(metadata.get("window_frames", 0) or 0),
            int(source.metadata.get("window_frames", 0) or 0),
        ),
    ]
    for name, expected, actual in checks:
        if expected and actual and expected != actual:
            raise ValueError(
                f"Source dataset {source.data_dir} {name}={actual} does not match "
                f"latent dataset {data_dir} {name}={expected}"
            )

    source_target_shape = _metadata_list(metadata.get("source_target_shape"))
    actual_target_shape = _metadata_list(source.metadata.get("target_shape"))
    if source_target_shape and actual_target_shape and source_target_shape != actual_target_shape:
        raise ValueError(
            f"Source dataset {source.data_dir} target_shape={actual_target_shape} does not match "
            f"latent source_target_shape={source_target_shape}"
        )

    source_target_mode = metadata.get("source_target_mode")
    actual_target_mode = source.metadata.get("target_mode")
    if source_target_mode and actual_target_mode and source_target_mode != actual_target_mode:
        raise ValueError(
            f"Source dataset {source.data_dir} target_mode={actual_target_mode!r} does not match "
            f"latent source_target_mode={source_target_mode!r}"
        )

    source_target_representation = metadata.get("source_target_representation")
    actual_target_representation = source.metadata.get("target_representation")
    if (
        source_target_representation
        and actual_target_representation
        and source_target_representation != actual_target_representation
    ):
        raise ValueError(
            f"Source dataset {source.data_dir} target_representation={actual_target_representation!r} "
            f"does not match latent source_target_representation={source_target_representation!r}"
        )

    feature_names = _metadata_list(metadata.get("feature_names"))
    source_feature_names = _metadata_list(source.metadata.get("feature_names"))
    if feature_names and source_feature_names and feature_names != source_feature_names:
        raise ValueError(f"Source dataset {source.data_dir} feature_names do not match latent metadata")

    for key in (
        "conditioning_parts",
        "gesture_energy_names",
        "gesture_energy_feature_names",
        "gesture_energy_thresholds",
        "gesture_energy_audio_thresholds",
    ):
        expected = _metadata_list(metadata.get(key))
        actual = _metadata_list(source.metadata.get(key))
        if expected and actual and expected != actual:
            raise ValueError(f"Source dataset {source.data_dir} {key} does not match latent metadata")


class LatentGestureDataset(torch.utils.data.Dataset):
    """Read latent LMDB windows written by ``build_latent_dataset.py``.

    Current datasets store only latents plus a ``source_idx`` pointing back to
    the preprocessed LMDB. Older datasets that duplicated conditioning fields
    are still supported for compatibility.
    """

    def __init__(self, data_dir: str | Path, source_data_dir: str | Path | None = None):
        self.data_dir = Path(data_dir)
        if not self.data_dir.exists():
            raise FileNotFoundError(f"Dataset path does not exist: {self.data_dir}")
        self._env = None
        self.source = None

        env = self._open_lmdb()
        try:
            with env.begin() as txn:
                length_bytes = txn.get(b"__len__")
                if length_bytes is None:
                    raise ValueError(f"No b'__len__' key in latent LMDB at {self.data_dir}")
                self.length = int(pickle.loads(length_bytes))

                metadata_bytes = txn.get(b"__metadata__")
                self.metadata = pickle.loads(metadata_bytes) if metadata_bytes is not None else {}

                first = txn.get(b"00000000")
                if first is not None and self.length > 0:
                    raw = pickle.loads(first)
                    self._compact = bool(self.metadata.get("compact", "x" not in raw))
                    self.metadata.update({
                        "target_shape": list(raw["y"].shape[1:]),
                        "latent_dim": int(raw["y"].shape[-1]),
                        "window_frames": int(raw["y"].shape[0]),
                    })
                else:
                    self._compact = bool(
                        self.metadata.get(
                            "compact",
                            self.metadata.get("stores_conditioning") is False,
                        )
                    )
        finally:
            env.close()

        representation = self.metadata.get("target_representation")
        if representation != "gesture_latent":
            raise ValueError(
                f"Expected target_representation='gesture_latent', got {representation!r}"
            )

        if self._compact:
            source_path = source_data_dir or self.metadata.get("source_data_dir")
            if not source_path:
                raise ValueError(
                    f"Compact latent dataset {self.data_dir} is missing source_data_dir metadata"
                )
            self.source = PreprocessedGestureDataset(source_path)
            validate_source_metadata(self.metadata, self.source, self.length, self.data_dir)
            self.metadata.setdefault("input_dim", self.source.metadata.get("input_dim"))
            self.metadata.setdefault("speaker_dim", self.source.metadata.get("speaker_dim", 0))
            self.metadata.setdefault("has_valid_mask", self.source.metadata.get("has_valid_mask", False))

    def _open_lmdb(self):
        try:
            import lmdb
        except ImportError as exc:
            raise ImportError("The lmdb package is required to read latent LMDB data.") from exc
        return lmdb.open(
            str(self.data_dir),
            readonly=True,
            lock=False,
            readahead=False,
            meminit=False,
            max_readers=2048,
        )

    def _get_env(self):
        if self._env is None:
            self._env = self._open_lmdb()
        return self._env

    def __len__(self) -> int:
        return self.length

    def _read_record(self, idx: int) -> dict:
        if idx < 0 or idx >= self.length:
            raise IndexError(idx)
        with self._get_env().begin() as txn:
            value = txn.get(f"{idx:08d}".encode())
        if value is None:
            raise KeyError(f"Key {idx:08d} not found in {self.data_dir}")
        return pickle.loads(value)

    def __getitem__(self, idx: int):
        raw = self._read_record(idx)
        latents = _as_float_tensor(raw["y"])

        if "x" in raw:
            speaker = raw.get("speaker")
            if speaker is None:
                speaker = np.zeros((int(self.metadata.get("speaker_dim", 0)),), dtype=np.float32)
            valid_mask = raw.get("valid_mask")
            if valid_mask is None:
                valid_mask = np.ones((latents.shape[0], 1), dtype=np.float32)
            return (
                _as_float_tensor(raw["x"]),
                latents,
                _as_float_tensor(speaker),
                _as_float_tensor(valid_mask),
            )

        if self.source is None:
            raise ValueError(f"Compact latent dataset {self.data_dir} has no source dataset")
        source_idx = int(raw.get("source_idx", idx))
        if source_idx < 0 or source_idx >= len(self.source):
            raise IndexError(
                f"Latent record {idx} references source_idx={source_idx}, "
                f"but source dataset length is {len(self.source)}"
            )
        x, _motion, speaker, valid_mask = self.source[source_idx]
        return x, latents, speaker, valid_mask


class ShardedLatentGestureDataset(torch.utils.data.Dataset):
    """Read latent windows from compact shard files.

    This format is friendlier to slow shared storage than LMDB random reads. When
    ``cache_dir`` is set, only the currently used shard is copied to local disk.
    """

    def __init__(
        self,
        data_dir: str | Path,
        cache_dir: str | Path | None = None,
        source_data_dir: str | Path | None = None,
    ):
        self.data_dir = Path(data_dir)
        manifest_path = self.data_dir / "manifest.json"
        if not manifest_path.is_file():
            raise FileNotFoundError(f"Missing sharded latent manifest: {manifest_path}")
        with open(manifest_path, "r") as f:
            manifest = json.load(f)
        self.metadata = manifest["metadata"]
        if self.metadata.get("target_representation") != "gesture_latent":
            raise ValueError("Sharded dataset metadata is not gesture_latent")
        self._compact = bool(manifest.get("compact", False))
        self.source = None
        if self._compact:
            source_path = source_data_dir or self.metadata.get("source_data_dir")
            if not source_path:
                raise ValueError(
                    f"Compact sharded dataset {self.data_dir} is missing source_data_dir metadata"
                )
            self.source = PreprocessedGestureDataset(source_path)
            validate_source_metadata(self.metadata, self.source, int(manifest["num_samples"]), self.data_dir)
        self.shards = manifest["shards"]
        self.length = int(manifest["num_samples"])
        self.ends = []
        total = 0
        for shard in self.shards:
            total += int(shard["length"])
            self.ends.append(total)
        if total != self.length:
            raise ValueError(f"Shard lengths sum to {total}, manifest says {self.length}")

        self.cache_dir = Path(cache_dir) if cache_dir else None
        if self.cache_dir is not None:
            self.cache_dir.mkdir(parents=True, exist_ok=True)
        self._loaded_shard_idx = None
        self._loaded_shard = None
        self._cached_path = None

    def __len__(self) -> int:
        return self.length

    def _copy_to_cache(self, source: Path) -> Path:
        if self.cache_dir is None:
            return source
        target = self.cache_dir / source.name
        if not target.exists() or target.stat().st_size != source.stat().st_size:
            tmp_target = target.with_suffix(target.suffix + ".tmp")
            if tmp_target.exists():
                tmp_target.unlink()
            shutil.copy2(source, tmp_target)
            tmp_target.replace(target)
        if self._cached_path is not None and self._cached_path != target and self._cached_path.exists():
            self._cached_path.unlink()
        self._cached_path = target
        return target

    def _load_shard(self, shard_idx: int):
        if self._loaded_shard_idx == shard_idx:
            return self._loaded_shard
        shard_path = self.data_dir / self.shards[shard_idx]["file"]
        load_path = self._copy_to_cache(shard_path)
        self._loaded_shard = torch.load(load_path, map_location="cpu")
        self._loaded_shard_idx = shard_idx
        return self._loaded_shard

    def __getitem__(self, idx: int):
        if idx < 0 or idx >= self.length:
            raise IndexError(idx)
        shard_idx = bisect.bisect_right(self.ends, idx)
        start = 0 if shard_idx == 0 else self.ends[shard_idx - 1]
        local_idx = idx - start
        shard = self._load_shard(shard_idx)
        latents = shard["y"][local_idx].float()
        if "x" not in shard:
            if self.source is None:
                raise ValueError(f"Compact sharded dataset {self.data_dir} has no source dataset")
            source_idx = int(shard["source_idx"][local_idx]) if "source_idx" in shard else idx
            if source_idx < 0 or source_idx >= len(self.source):
                raise IndexError(
                    f"Shard sample {idx} references source_idx={source_idx}, "
                    f"but source dataset length is {len(self.source)}"
                )
            x, _motion, speaker, valid_mask = self.source[source_idx]
            return x, latents, speaker, valid_mask
        return (
            shard["x"][local_idx].float(),
            latents,
            shard["speaker"][local_idx].float(),
            shard["valid_mask"][local_idx].float(),
        )


def make_latent_dataset(
    data_dir: str | Path,
    cache_dir: str | Path | None = None,
    source_data_dir: str | Path | None = None,
):
    data_dir = Path(data_dir)
    if (data_dir / "manifest.json").is_file():
        return ShardedLatentGestureDataset(data_dir, cache_dir=cache_dir, source_data_dir=source_data_dir)
    return LatentGestureDataset(data_dir, source_data_dir=source_data_dir)


__all__ = [
    "LatentGestureDataset",
    "PreprocessedGestureDataset",
    "ShardedLatentGestureDataset",
    "make_latent_dataset",
]
