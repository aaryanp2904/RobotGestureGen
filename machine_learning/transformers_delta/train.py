#!/usr/bin/env python3
"""Train the BEAT2/NAO GestureTransformer on preprocessed delta LMDB windows."""

from __future__ import annotations

import argparse
import math
import pickle
import time
from pathlib import Path

import lmdb
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset

try:
    from .model import GestureTransformer, model_init_kwargs
except ImportError:
    from model import GestureTransformer, model_init_kwargs


class PreprocessedLMDBDataset(Dataset):
    """Lazy LMDB dataset for transformer windows."""

    def __init__(self, lmdb_dir: str | Path):
        self.lmdb_dir = str(Path(lmdb_dir))
        self._env = None
        env = lmdb.open(
            self.lmdb_dir,
            readonly=True,
            lock=False,
            readahead=False,
            meminit=False,
            max_readers=2048,
        )
        with env.begin() as txn:
            length_bytes = txn.get(b"__len__")
            metadata_bytes = txn.get(b"__metadata__")
            if length_bytes is None:
                raise ValueError(f"No b'__len__' key in LMDB at {self.lmdb_dir}")
            if metadata_bytes is None:
                raise ValueError(f"No b'__metadata__' key in LMDB at {self.lmdb_dir}")
            self.length = int(pickle.loads(length_bytes))
            self.metadata = pickle.loads(metadata_bytes)
            validate_transformer_metadata(self.metadata, self.lmdb_dir)
        env.close()

        print(f"[DATASET] {self.lmdb_dir}")
        print(f"[DATASET] windows={self.length}")
        print(
            "[DATASET] "
            f"input_dim={self.metadata['input_dim']} "
            f"target_shape={self.metadata['target_shape']} "
            f"target_mode={self.metadata.get('target_mode', 'delta')}"
        )

    def _get_env(self):
        if self._env is None:
            self._env = lmdb.open(
                self.lmdb_dir,
                readonly=True,
                lock=False,
                readahead=False,
                meminit=False,
                max_readers=2048,
            )
        return self._env

    @staticmethod
    def _to_f32(value) -> torch.Tensor:
        if isinstance(value, np.ndarray):
            return torch.from_numpy(value.astype(np.float32, copy=False))
        return value.float()

    def __len__(self) -> int:
        return self.length

    def __getitem__(self, idx: int):
        with self._get_env().begin() as txn:
            value = txn.get(f"{idx:08d}".encode())
        if value is None:
            raise KeyError(f"Key {idx:08d} not found in LMDB")
        raw = pickle.loads(value)
        y_vel = raw.get("y_vel")
        if y_vel is None:
            y_vel = np.zeros_like(raw["y"], dtype=np.float32)
        valid_mask = raw.get("valid_mask")
        if valid_mask is None:
            valid_mask = np.ones((raw["y"].shape[0], 1), dtype=np.float32)
        return (
            self._to_f32(raw["x"]),
            self._to_f32(raw["y"]),
            self._to_f32(y_vel),
            self._to_f32(valid_mask),
        )


def validate_transformer_metadata(metadata: dict, source: str) -> None:
    """Ensure the LMDB matches this energy-free delta-transformer pipeline."""
    gesture_energy_dim = int(metadata.get("gesture_energy_dim", 0) or 0)
    if gesture_energy_dim != 0:
        raise ValueError(
            f"{source} has gesture_energy_dim={gesture_energy_dim}; "
            "rerun machine_learning.transformers_delta.preprocessing so delta-transformer "
            "training uses prosody + WavLM (+ optional text) only."
        )
    input_dim = int(metadata.get("input_dim", 0) or 0)
    expected_input_dim = (
        int(metadata.get("prosody_dim", 0) or 0)
        + int(metadata.get("wavlm_dim", 0) or 0)
        + int(metadata.get("text_dim", 0) or 0)
    )
    if input_dim <= 0 or input_dim != expected_input_dim:
        raise ValueError(
            f"{source} input_dim={input_dim} does not match "
            f"prosody+wavlm+text dims ({expected_input_dim})"
        )
    target_shape = tuple(metadata.get("target_shape", ()))
    if len(target_shape) != 1 or int(target_shape[0]) <= 0:
        raise ValueError(f"{source} target_shape must be a 1-D NAO angle shape, got {target_shape}")
    target_mode = metadata.get("target_mode")
    if target_mode != "delta":
        raise ValueError(
            f"{source} target_mode={target_mode!r}; transformers_delta expects per-frame deltas. "
            "Rerun machine_learning.transformers_delta.preprocessing."
        )


def important_metadata(metadata: dict) -> dict:
    keys = [
        "fps",
        "input_dim",
        "target_shape",
        "target_mode",
        "target_representation",
        "conditioning_parts",
        "feature_names",
        "target_names",
        "prosody_dim",
        "wavlm_dim",
        "wavlm_model",
        "text_dim",
        "gesture_energy_dim",
    ]
    return {key: metadata.get(key) for key in keys}


def validate_metadata_match(train_metadata: dict, val_metadata: dict) -> None:
    train_contract = important_metadata(train_metadata)
    val_contract = important_metadata(val_metadata)
    if train_contract != val_contract:
        raise ValueError(
            "Validation LMDB metadata does not match training LMDB:\n"
            f"train={train_contract}\nval={val_contract}"
        )


def build_model_config(metadata: dict, args: argparse.Namespace) -> dict:
    return {
        "input_dim": int(metadata["input_dim"]),
        "hidden_dim": args.hidden_dim,
        "num_heads": args.num_heads,
        "num_layers": args.num_layers,
        "dropout": args.dropout,
        "target_shape": tuple(metadata["target_shape"]),
        "target_mode": metadata.get("target_mode", "delta"),
        "target_representation": metadata.get("target_representation", "nao_joint_deltas"),
        "target_type": metadata.get("target_type", "nao_joint_deltas"),
        "nao_std": metadata.get("nao_std"),
        "nao_vel_mean": metadata.get("nao_vel_mean"),
        "nao_vel_std": metadata.get("nao_vel_std"),
    }


def masked_mse(pred: torch.Tensor, target: torch.Tensor, mask: torch.Tensor | None) -> torch.Tensor:
    if mask is None:
        return torch.mean((pred - target) ** 2)
    weight = mask
    while weight.ndim < pred.ndim:
        weight = weight.unsqueeze(-1)
    per_frame_dims = math.prod(pred.shape[2:]) if pred.ndim > 2 else 1
    denom = weight.sum().clamp(min=1.0) * per_frame_dims
    return torch.sum((pred - target) ** 2 * weight) / denom


class MotionLoss(nn.Module):
    """Pose MSE plus optional temporal regularization losses."""

    def __init__(
        self,
        velocity_weight: float = 0.0,
        acceleration_weight: float = 0.0,
        velocity_target_weight: float = 0.0,
        target_mode: str = "delta",
        angle_std=None,
        vel_mean=None,
        vel_std=None,
    ):
        super().__init__()
        self.velocity_weight = float(velocity_weight)
        self.acceleration_weight = float(acceleration_weight)
        self.velocity_target_weight = float(velocity_target_weight)
        self.target_mode = target_mode
        self.register_buffer(
            "angle_std",
            torch.as_tensor(angle_std if angle_std is not None else [1.0], dtype=torch.float32),
        )
        self.register_buffer(
            "vel_mean",
            torch.as_tensor(vel_mean if vel_mean is not None else [0.0], dtype=torch.float32),
        )
        self.register_buffer(
            "vel_std",
            torch.as_tensor(vel_std if vel_std is not None else [1.0], dtype=torch.float32),
        )

    @staticmethod
    def pair_mask(mask: torch.Tensor | None) -> torch.Tensor | None:
        if mask is None or mask.shape[1] <= 1:
            return None
        return mask[:, 1:] * mask[:, :-1]

    @staticmethod
    def triplet_mask(mask: torch.Tensor | None) -> torch.Tensor | None:
        if mask is None or mask.shape[1] <= 2:
            return None
        return mask[:, 2:] * mask[:, 1:-1] * mask[:, :-2]

    def forward(
        self,
        pred: torch.Tensor,
        target: torch.Tensor,
        target_velocity: torch.Tensor | None = None,
        valid_mask: torch.Tensor | None = None,
    ) -> torch.Tensor:
        total = masked_mse(pred, target, valid_mask)

        if self.velocity_weight > 0 and self.target_mode == "angle" and pred.shape[1] > 1:
            pred_vel = pred[:, 1:] - pred[:, :-1]
            target_vel = target[:, 1:] - target[:, :-1]
            total = total + self.velocity_weight * masked_mse(
                pred_vel, target_vel, self.pair_mask(valid_mask)
            )

        if self.acceleration_weight > 0 and self.target_mode == "angle" and pred.shape[1] > 2:
            pred_vel = pred[:, 1:] - pred[:, :-1]
            target_vel = target[:, 1:] - target[:, :-1]
            pred_acc = pred_vel[:, 1:] - pred_vel[:, :-1]
            target_acc = target_vel[:, 1:] - target_vel[:, :-1]
            total = total + self.acceleration_weight * masked_mse(
                pred_acc, target_acc, self.triplet_mask(valid_mask)
            )

        if (
            self.velocity_target_weight > 0
            and self.target_mode == "angle"
            and target_velocity is not None
            and pred.shape[1] > 1
        ):
            shape = (1, 1) + tuple(self.angle_std.shape)
            angle_std = self.angle_std.view(shape)
            vel_mean = self.vel_mean.view(shape)
            vel_std = self.vel_std.view(shape)
            pred_delta = (pred[:, 1:] - pred[:, :-1]) * angle_std
            pred_velocity_norm = (pred_delta - vel_mean) / vel_std
            total = total + self.velocity_target_weight * masked_mse(
                pred_velocity_norm,
                target_velocity[:, 1:],
                valid_mask[:, 1:] if valid_mask is not None else None,
            )

        return total


def autocast_context(device: torch.device):
    return torch.amp.autocast("cuda", enabled=(device.type == "cuda"))


def make_grad_scaler(device: torch.device):
    return torch.amp.GradScaler("cuda", enabled=(device.type == "cuda"))


def run_epoch(
    model: nn.Module,
    criterion: MotionLoss,
    loader: DataLoader,
    device: torch.device,
    optimizer=None,
    scaler=None,
    log_every: int = 50,
) -> float:
    training = optimizer is not None
    model.train(training)
    total_loss = 0.0
    total_batches = 0

    for batch_idx, (x, y, y_vel, valid_mask) in enumerate(loader, start=1):
        x = x.to(device, non_blocking=True)
        y = y.to(device, non_blocking=True)
        y_vel = y_vel.to(device, non_blocking=True)
        valid_mask = valid_mask.to(device, non_blocking=True)

        if training:
            optimizer.zero_grad(set_to_none=True)

        with torch.set_grad_enabled(training):
            with autocast_context(device):
                loss = criterion(model(x), y, y_vel, valid_mask)

        if training:
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            scaler.step(optimizer)
            scaler.update()
            if batch_idx == 1 or batch_idx % log_every == 0:
                print(f"  batch {batch_idx}/{len(loader)} loss={loss.item():.6f}", flush=True)

        total_loss += loss.item()
        total_batches += 1

    if total_batches == 0:
        raise RuntimeError("DataLoader produced no batches")
    return total_loss / total_batches


def save_checkpoint(
    path: Path,
    model: nn.Module,
    model_config: dict,
    dataset_metadata: dict,
    epoch: int,
    train_loss: float,
    val_loss: float | None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "model_config": model_config,
            "dataset_metadata": dataset_metadata,
            "epoch": epoch,
            "train_loss": train_loss,
            "val_loss": val_loss,
        },
        path,
    )
    print(f"[CHECKPOINT] {path}")


def sanity_check(
    model: nn.Module,
    criterion: MotionLoss,
    loader: DataLoader,
    device: torch.device,
    steps: int,
) -> None:
    optimizer = optim.AdamW(model.parameters(), lr=1e-3)
    scaler = make_grad_scaler(device)
    x, y, y_vel, valid_mask = next(iter(loader))
    x = x.to(device)
    y = y.to(device)
    y_vel = y_vel.to(device)
    valid_mask = valid_mask.to(device)
    print(f"[SANITY] Overfitting one batch for {steps} steps")
    print(f"[SANITY] x={tuple(x.shape)} y={tuple(y.shape)}")
    for step in range(1, steps + 1):
        model.train()
        optimizer.zero_grad(set_to_none=True)
        with autocast_context(device):
            loss = criterion(model(x), y, y_vel, valid_mask)
        scaler.scale(loss).backward()
        scaler.unscale_(optimizer)
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        scaler.step(optimizer)
        scaler.update()
        if step == 1 or step % 50 == 0 or step == steps:
            print(f"[SANITY] {step}/{steps} loss={loss.item():.8f}")


def train(args: argparse.Namespace) -> None:
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")
    print(f"[TRAIN] device={device}")
    if device.type == "cuda":
        print(f"[TRAIN] gpu={torch.cuda.get_device_name(0)}")

    train_dataset = PreprocessedLMDBDataset(args.data_dir)
    val_dataset = None
    val_data_dir = args.val_data_dir
    if val_data_dir is None:
        candidate = Path(args.data_dir).parent / "val.lmdb"
        if candidate.is_dir():
            val_data_dir = str(candidate)
    if val_data_dir and not args.sanity_check:
        val_dataset = PreprocessedLMDBDataset(val_data_dir)
        validate_metadata_match(train_dataset.metadata, val_dataset.metadata)

    model_config = build_model_config(train_dataset.metadata, args)
    if model_config.get("target_mode") == "delta" and any(
        weight > 0
        for weight in (
            args.velocity_loss_weight,
            args.acceleration_loss_weight,
            args.velocity_target_loss_weight,
        )
    ):
        raise ValueError(
            "The copied temporal loss flags operate on absolute angle targets and are disabled "
            "for transformers_delta. Leave velocity/acceleration loss weights at 0."
        )
    if args.velocity_target_loss_weight > 0 and any(
        model_config.get(key) is None for key in ("nao_std", "nao_vel_mean", "nao_vel_std")
    ):
        raise ValueError("--velocity-target-loss-weight requires nao_std/nao_vel stats in metadata")

    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=(device.type == "cuda"),
        persistent_workers=args.num_workers > 0,
    )
    val_loader = None
    if val_dataset is not None:
        val_loader = DataLoader(
            val_dataset,
            batch_size=args.batch_size,
            shuffle=False,
            num_workers=args.num_workers,
            pin_memory=(device.type == "cuda"),
            persistent_workers=args.num_workers > 0,
        )

    model = GestureTransformer(**model_init_kwargs(model_config)).to(device)
    criterion = MotionLoss(
        velocity_weight=args.velocity_loss_weight,
        acceleration_weight=args.acceleration_loss_weight,
        velocity_target_weight=args.velocity_target_loss_weight,
        target_mode=model_config.get("target_mode", "delta"),
        angle_std=model_config.get("nao_std"),
        vel_mean=model_config.get("nao_vel_mean"),
        vel_std=model_config.get("nao_vel_std"),
    ).to(device)

    if args.sanity_check:
        sanity_check(model, criterion, train_loader, device, args.sanity_steps)
        return

    optimizer = optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    scaler = make_grad_scaler(device)
    output_dir = Path(args.output_dir)
    best_metric = float("inf")
    best_name = "val" if val_loader is not None else "train"

    for epoch in range(1, args.epochs + 1):
        start_time = time.time()
        print(f"\n[TRAIN] epoch {epoch}/{args.epochs}")
        train_loss = run_epoch(
            model,
            criterion,
            train_loader,
            device,
            optimizer=optimizer,
            scaler=scaler,
            log_every=args.log_every,
        )
        val_loss = None
        if val_loader is not None:
            with torch.no_grad():
                val_loss = run_epoch(model, criterion, val_loader, device)

        elapsed = time.time() - start_time
        if val_loss is None:
            print(f"[TRAIN] epoch={epoch} train_loss={train_loss:.6f} time={elapsed:.1f}s")
        else:
            print(
                f"[TRAIN] epoch={epoch} train_loss={train_loss:.6f} "
                f"val_loss={val_loss:.6f} time={elapsed:.1f}s"
            )

        save_checkpoint(
            output_dir / "gesture_transformer_latest.pth",
            model,
            model_config,
            train_dataset.metadata,
            epoch,
            train_loss,
            val_loss,
        )
        metric = val_loss if val_loss is not None else train_loss
        if metric < best_metric:
            best_metric = metric
            save_checkpoint(
                output_dir / "gesture_transformer_best.pth",
                model,
                model_config,
                train_dataset.metadata,
                epoch,
                train_loss,
                val_loss,
            )
            print(f"[TRAIN] new best {best_name}_loss={best_metric:.6f}")

    save_checkpoint(
        output_dir / "gesture_transformer_full_trained.pth",
        model,
        model_config,
        train_dataset.metadata,
        args.epochs,
        train_loss,
        val_loss,
    )
    print(f"[TRAIN] complete: {output_dir}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Train GestureTransformer on BEAT2 delta LMDB windows")
    parser.add_argument("--data-dir", required=True, help="Path to train.lmdb")
    parser.add_argument("--val-data-dir", default=None, help="Optional path to val.lmdb")
    parser.add_argument("--output-dir", required=True, help="Directory for checkpoints")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-2)
    parser.add_argument("--hidden-dim", type=int, default=256)
    parser.add_argument("--num-heads", type=int, default=8)
    parser.add_argument("--num-layers", type=int, default=4)
    parser.add_argument("--dropout", type=float, default=0.1)
    parser.add_argument("--velocity-loss-weight", type=float, default=0.0)
    parser.add_argument("--acceleration-loss-weight", type=float, default=0.0)
    parser.add_argument("--velocity-target-loss-weight", type=float, default=0.0)
    parser.add_argument("--log-every", type=int, default=50)
    parser.add_argument("--sanity-check", action="store_true")
    parser.add_argument("--sanity-steps", type=int, default=500)
    parser.add_argument("--cpu", action="store_true")
    args = parser.parse_args()

    if args.epochs <= 0:
        raise ValueError("--epochs must be positive")
    if args.batch_size <= 0:
        raise ValueError("--batch-size must be positive")
    if args.num_workers < 0:
        raise ValueError("--num-workers cannot be negative")
    if args.lr <= 0:
        raise ValueError("--lr must be positive")
    if args.hidden_dim <= 0:
        raise ValueError("--hidden-dim must be positive")
    if args.num_heads <= 0:
        raise ValueError("--num-heads must be positive")
    if args.hidden_dim % args.num_heads != 0:
        raise ValueError("--hidden-dim must be divisible by --num-heads")
    if args.num_layers <= 0:
        raise ValueError("--num-layers must be positive")
    if not 0 <= args.dropout < 1:
        raise ValueError("--dropout must be in [0, 1)")
    if args.log_every <= 0:
        raise ValueError("--log-every must be positive")
    if args.sanity_steps <= 0:
        raise ValueError("--sanity-steps must be positive")

    train(args)


if __name__ == "__main__":
    main()
