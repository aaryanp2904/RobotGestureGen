"""Train a simple audio/text-to-motion regression baseline."""

from __future__ import annotations

import argparse
import json
import random
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader

try:
    from .dataset import PreprocessedGestureDataset
except ImportError:
    from dataset import PreprocessedGestureDataset


class TemporalRegressionBaseline(nn.Module):
    """Small Transformer baseline trained with direct L2 motion loss."""

    def __init__(
        self,
        input_dim: int,
        target_shape: tuple[int, ...],
        hidden_dim: int = 256,
        num_heads: int = 8,
        num_layers: int = 3,
        dropout: float = 0.1,
    ):
        super().__init__()
        self.target_shape = tuple(target_shape)
        self.motion_dim = int(np.prod(self.target_shape))
        self.input_projection = nn.Sequential(
            nn.LayerNorm(input_dim),
            nn.Linear(input_dim, hidden_dim),
            nn.GELU(),
        )
        layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 4,
            dropout=dropout,
            batch_first=True,
            norm_first=True,
        )
        self.encoder = nn.TransformerEncoder(layer, num_layers=num_layers)
        self.output_projection = nn.Linear(hidden_dim, self.motion_dim)

    def forward(self, conditioning: torch.Tensor) -> torch.Tensor:
        hidden = self.encoder(self.input_projection(conditioning))
        out = self.output_projection(hidden)
        return out.reshape(conditioning.shape[0], conditioning.shape[1], *self.target_shape)


def set_seed(seed: int):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def masked_mse(prediction: torch.Tensor, target: torch.Tensor, valid_mask: torch.Tensor) -> torch.Tensor:
    while valid_mask.ndim < prediction.ndim:
        valid_mask = valid_mask.unsqueeze(-1)
    valid_mask = valid_mask.to(dtype=prediction.dtype, device=prediction.device)
    squared = torch.square(prediction - target)
    return (squared * valid_mask).sum() / valid_mask.expand_as(squared).sum().clamp(min=1.0)


def make_loader(path: str, args, shuffle: bool) -> tuple[PreprocessedGestureDataset, DataLoader]:
    dataset = PreprocessedGestureDataset(path)
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=shuffle,
        num_workers=args.num_workers,
        pin_memory=torch.cuda.is_available() and not args.cpu,
        persistent_workers=args.num_workers > 0,
    )
    return dataset, loader


def validate_args(args):
    if args.epochs <= 0:
        raise ValueError("--epochs must be positive")
    if args.batch_size <= 0:
        raise ValueError("--batch-size must be positive")
    if args.num_workers < 0:
        raise ValueError("--num-workers cannot be negative")
    if args.learning_rate <= 0:
        raise ValueError("--learning-rate must be positive")
    if args.weight_decay < 0:
        raise ValueError("--weight-decay cannot be negative")
    if args.hidden_dim <= 0:
        raise ValueError("--hidden-dim must be positive")
    if args.num_heads <= 0:
        raise ValueError("--num-heads must be positive")
    if args.num_layers <= 0:
        raise ValueError("--num-layers must be positive")
    if args.hidden_dim % args.num_heads != 0:
        raise ValueError("--hidden-dim must be divisible by --num-heads")
    if not 0.0 <= args.dropout < 1.0:
        raise ValueError("--dropout must be in [0, 1)")


def run_epoch(model, loader, device, optimizer=None) -> float:
    training = optimizer is not None
    model.train(training)
    total = 0.0
    for conditioning, target, _, valid_mask in loader:
        conditioning = conditioning.to(device, non_blocking=True)
        target = target.to(device, non_blocking=True)
        valid_mask = valid_mask.to(device, non_blocking=True)
        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training):
            loss = masked_mse(model(conditioning), target, valid_mask)
        if training:
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
        total += float(loss.item())
    return total / max(len(loader), 1)


def train(args):
    validate_args(args)
    set_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")
    train_dataset, train_loader = make_loader(args.data_dir, args, shuffle=True)
    val_loader = None
    if args.val_data_dir:
        _, val_loader = make_loader(args.val_data_dir, args, shuffle=False)

    model_config = {
        "input_dim": int(train_dataset.metadata["input_dim"]),
        "target_shape": tuple(train_dataset.metadata["target_shape"]),
        "hidden_dim": args.hidden_dim,
        "num_heads": args.num_heads,
        "num_layers": args.num_layers,
        "dropout": args.dropout,
    }
    model = TemporalRegressionBaseline(**model_config).to(device)
    optimizer = optim.AdamW(model.parameters(), lr=args.learning_rate, weight_decay=args.weight_decay)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    best = float("inf")
    for epoch in range(1, args.epochs + 1):
        start = time.time()
        train_loss = run_epoch(model, train_loader, device, optimizer=optimizer)
        val_loss = run_epoch(model, val_loader, device) if val_loader is not None else None
        score = val_loss if val_loss is not None else train_loss
        checkpoint = {
            "model_state_dict": model.state_dict(),
            "model_config": {**model_config, "target_shape": list(model_config["target_shape"])},
            "dataset_metadata": train_dataset.metadata,
            "epoch": epoch,
            "train_loss": train_loss,
            "val_loss": val_loss,
            "baseline_type": "temporal_regression",
        }
        torch.save(checkpoint, output_dir / "regression_latest.pth")
        if score < best:
            best = score
            torch.save(checkpoint, output_dir / "regression_best.pth")
        print(
            f"[BASELINE] Epoch {epoch}/{args.epochs} train={train_loss:.6f} "
            f"val={val_loss if val_loss is not None else 'n/a'} time={time.time() - start:.1f}s",
            flush=True,
        )
    with open(output_dir / "regression_training_config.json", "w") as f:
        json.dump({"model_config": checkpoint["model_config"], "args": vars(args)}, f, indent=2)


def main():
    parser = argparse.ArgumentParser(description="Train direct regression gesture baseline")
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--val-data-dir", default=None)
    parser.add_argument("--output-dir", default="regression_baseline_checkpoints")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--hidden-dim", type=int, default=256)
    parser.add_argument("--num-heads", type=int, default=8)
    parser.add_argument("--num-layers", type=int, default=3)
    parser.add_argument("--dropout", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--cpu", action="store_true")
    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()
