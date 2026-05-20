#!/usr/bin/env python3
"""Evaluate a trained GestureTransformer directly on preprocessed LMDB windows."""

import argparse
import pickle
from pathlib import Path

import lmdb
import numpy as np
import torch
from torch.utils.data import DataLoader

from pos_end import GestureTransformer
from train import MODEL_INIT_KEYS, PreprocessedLMDBDataset


def model_init_kwargs(model_config: dict) -> dict:
    return {key: value for key, value in model_config.items() if key in MODEL_INIT_KEYS}


def load_checkpoint(path: Path, device: torch.device):
    checkpoint = torch.load(path, map_location=device)
    if not isinstance(checkpoint, dict) or "model_state_dict" not in checkpoint:
        raise ValueError("Expected checkpoint dict with model_state_dict and model_config")
    return checkpoint["model_state_dict"], checkpoint.get("model_config", {})


def denormalize_targets(values: np.ndarray, metadata: dict) -> np.ndarray:
    target_mode = metadata.get("target_mode", "angle")
    if target_mode == "angle":
        mean = np.asarray(metadata["nao_mean"], dtype=np.float32)
        std = np.asarray(metadata["nao_std"], dtype=np.float32)
        return values * std + mean
    if target_mode == "delta":
        mean = np.asarray(metadata["nao_vel_mean"], dtype=np.float32)
        std = np.asarray(metadata["nao_vel_std"], dtype=np.float32)
        deltas = values * std + mean
        if deltas.shape[1] > 0:
            deltas[:, 0] = 0.0
        initial = np.asarray(metadata["nao_mean"], dtype=np.float32)
        return initial + np.cumsum(deltas, axis=1)
    raise ValueError(f"Unsupported target_mode: {target_mode}")


def motion_stats(values: np.ndarray) -> dict:
    # values: (N, T, J)
    ranges = values.max(axis=1) - values.min(axis=1)
    stds = values.std(axis=1)
    if values.shape[1] > 1:
        velocity = values[:, 1:] - values[:, :-1]
        vel_abs = np.abs(velocity)
    else:
        vel_abs = np.zeros_like(values)
    return {
        "mean_range": float(ranges.mean()),
        "per_joint_range": ranges.mean(axis=0),
        "mean_std": float(stds.mean()),
        "mean_abs_velocity": float(vel_abs.mean()),
    }


def print_stats(name: str, stats: dict):
    print(f"\n[{name}]")
    print(f"  mean_range:        {stats['mean_range']:.6f}")
    print(f"  mean_std:          {stats['mean_std']:.6f}")
    print(f"  mean_abs_velocity: {stats['mean_abs_velocity']:.6f}")
    print(f"  per_joint_range:   {np.array2string(stats['per_joint_range'], precision=4)}")


def read_metadata(lmdb_dir: Path) -> dict:
    env = lmdb.open(str(lmdb_dir), readonly=True, lock=False, readahead=False, meminit=False)
    with env.begin() as txn:
        metadata = pickle.loads(txn.get(b"__metadata__"))
    env.close()
    return metadata


def main():
    parser = argparse.ArgumentParser(description="Evaluate checkpoint output motion on LMDB")
    parser.add_argument("--data-dir", required=True, help="Path to train/val/test .lmdb")
    parser.add_argument("--checkpoint", required=True, help="Path to model checkpoint")
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--max-batches", type=int, default=20)
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    dataset = PreprocessedLMDBDataset(args.data_dir)
    metadata = read_metadata(Path(args.data_dir))
    state_dict, model_config = load_checkpoint(Path(args.checkpoint), device)

    if model_config.get("target_mode") != metadata.get("target_mode"):
        raise ValueError(
            f"Checkpoint target_mode {model_config.get('target_mode')} does not match "
            f"LMDB target_mode {metadata.get('target_mode')}"
        )

    model = GestureTransformer(**model_init_kwargs(model_config)).to(device)
    model.load_state_dict(state_dict)
    model.eval()

    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=False, num_workers=0)
    preds = []
    targets = []
    mse_values = []

    with torch.no_grad():
        for batch_idx, (x, y, _y_vel) in enumerate(loader):
            x = x.to(device)
            y = y.to(device)
            pred = model(x)
            mse_values.append(torch.mean((pred - y) ** 2).item())
            preds.append(pred.cpu().numpy())
            targets.append(y.cpu().numpy())
            if batch_idx + 1 >= args.max_batches:
                break

    pred_norm = np.concatenate(preds, axis=0)
    target_norm = np.concatenate(targets, axis=0)
    pred_raw = denormalize_targets(pred_norm, metadata)
    target_raw = denormalize_targets(target_norm, metadata)

    print(f"[LOAD] device:       {device}")
    print(f"[LOAD] target_mode:  {metadata.get('target_mode')}")
    print(f"[LOAD] evaluated:    {pred_raw.shape[0]} windows")
    print(f"[LOSS] norm_mse:     {float(np.mean(mse_values)):.6f}")
    print(f"[LOSS] raw_mse:      {float(np.mean((pred_raw - target_raw) ** 2)):.6f}")

    pred_stats = motion_stats(pred_raw)
    target_stats = motion_stats(target_raw)
    print_stats("PRED", pred_stats)
    print_stats("TARGET", target_stats)
    ratio = pred_stats["mean_range"] / max(target_stats["mean_range"], 1e-8)
    print(f"\n[RATIO] pred/target mean_range: {ratio:.3f}")


if __name__ == "__main__":
    main()
