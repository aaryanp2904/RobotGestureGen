"""
Train a conditional diffusion model on preprocessed gesture windows.

Examples:
    python train.py --data-dir /path/to/train.lmdb --epochs 100
    python train.py --data-dir /path/to/train.lmdb --val-data-dir /path/to/val.lmdb
    python train.py --data-dir /path/to/train.lmdb --sanity-check
"""

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
    from .model import ConditionalMotionDenoiser, DiffusionSchedule
except ImportError:
    from dataset import PreprocessedGestureDataset
    from model import ConditionalMotionDenoiser, DiffusionSchedule


def set_seed(seed: int):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def metadata_model_config(metadata: dict, args) -> dict:
    return {
        "input_dim": int(metadata.get("input_dim", 1536)),
        "target_shape": tuple(metadata.get("target_shape", [12, 3])),
        "hidden_dim": args.hidden_dim,
        "num_heads": args.num_heads,
        "num_layers": args.num_layers,
        "dropout": args.dropout,
        "max_frames": args.max_frames,
    }


def make_dataloader(dataset, args, shuffle: bool) -> DataLoader:
    return DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=shuffle,
        num_workers=args.num_workers,
        pin_memory=torch.cuda.is_available() and not args.cpu,
        persistent_workers=args.num_workers > 0,
    )


def dataset_window_frames(dataset: PreprocessedGestureDataset) -> int:
    for key in ("window_frames", "window_size_frames"):
        if key in dataset.metadata:
            return int(dataset.metadata[key])
    sample_x, _ = dataset[0]
    return int(sample_x.shape[0])


def validate_args_and_data(args, dataset: PreprocessedGestureDataset, label: str = "training"):
    if len(dataset) == 0:
        raise ValueError(f"No {label} windows found in {dataset.data_dir}")
    if not args.sanity_check and args.epochs <= 0:
        raise ValueError("--epochs must be positive unless --sanity-check is used")
    if args.batch_size <= 0:
        raise ValueError("--batch-size must be positive")
    if args.num_workers < 0:
        raise ValueError("--num-workers cannot be negative")
    if args.log_every <= 0:
        raise ValueError("--log-every must be positive")
    if args.sanity_check and args.sanity_steps <= 0:
        raise ValueError("--sanity-steps must be positive")
    if args.learning_rate <= 0:
        raise ValueError("--learning-rate must be positive")
    if args.weight_decay < 0:
        raise ValueError("--weight-decay cannot be negative")
    window_frames = dataset_window_frames(dataset)
    if args.max_frames < window_frames:
        raise ValueError(
            f"--max-frames={args.max_frames} is shorter than the dataset window "
            f"length ({window_frames} frames)"
        )


def diffusion_loss(
    model: nn.Module,
    schedule: DiffusionSchedule,
    conditioning: torch.Tensor,
    clean_motion: torch.Tensor,
) -> torch.Tensor:
    batch_size = clean_motion.shape[0]
    timesteps = torch.randint(
        0,
        schedule.timesteps,
        (batch_size,),
        device=clean_motion.device,
        dtype=torch.long,
    )
    noise = torch.randn_like(clean_motion)
    noisy_motion = schedule.q_sample(clean_motion, timesteps, noise)
    predicted_noise = model(noisy_motion, conditioning, timesteps)
    return torch.nn.functional.mse_loss(predicted_noise, noise)


def run_epoch(
    model: nn.Module,
    schedule: DiffusionSchedule,
    dataloader: DataLoader,
    device: torch.device,
    optimizer=None,
    scaler=None,
    log_every: int = 25,
    epoch: int = 0,
) -> float:
    training = optimizer is not None
    model.train(training)
    total_loss = 0.0

    for batch_idx, (batch_x, batch_y) in enumerate(dataloader):
        batch_x = batch_x.to(device, non_blocking=True)
        batch_y = batch_y.to(device, non_blocking=True)

        if training:
            optimizer.zero_grad(set_to_none=True)

        with torch.set_grad_enabled(training):
            with torch.amp.autocast("cuda", enabled=(device.type == "cuda")):
                loss = diffusion_loss(model, schedule, batch_x, batch_y)

        if training:
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            scaler.step(optimizer)
            scaler.update()

        total_loss += loss.item()
        if training and ((batch_idx + 1) == 1 or (batch_idx + 1) % log_every == 0):
            print(
                f"  Epoch {epoch} | Batch {batch_idx + 1}/{len(dataloader)} "
                f"| Noise MSE: {loss.item():.6f}",
                flush=True,
            )

    return total_loss / max(len(dataloader), 1)


def save_checkpoint(
    path: Path,
    model: nn.Module,
    optimizer,
    model_config: dict,
    diffusion_config: dict,
    metadata: dict,
    epoch: int,
    train_loss: float,
    val_loss: float | None,
):
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "optimizer_state_dict": optimizer.state_dict() if optimizer is not None else None,
            "model_config": model_config,
            "diffusion_config": diffusion_config,
            "dataset_metadata": metadata,
            "epoch": epoch,
            "train_loss": train_loss,
            "val_loss": val_loss,
        },
        path,
    )


def sanity_check(model, schedule, dataloader, device, args):
    print("[SANITY] Overfitting one batch with diffusion noise prediction", flush=True)
    optimizer = optim.AdamW(model.parameters(), lr=args.learning_rate)
    scaler = torch.amp.GradScaler("cuda", enabled=(device.type == "cuda"))
    batch_x, batch_y = next(iter(dataloader))
    batch_x = batch_x.to(device)
    batch_y = batch_y.to(device)
    print(f"[SANITY] X={tuple(batch_x.shape)} Y={tuple(batch_y.shape)}", flush=True)

    for step in range(args.sanity_steps):
        optimizer.zero_grad(set_to_none=True)
        with torch.amp.autocast("cuda", enabled=(device.type == "cuda")):
            loss = diffusion_loss(model, schedule, batch_x, batch_y)
        scaler.scale(loss).backward()
        scaler.unscale_(optimizer)
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        scaler.step(optimizer)
        scaler.update()
        if step == 0 or (step + 1) % 25 == 0:
            print(f"[SANITY] {step + 1}/{args.sanity_steps}  Loss: {loss.item():.6f}")


def train(args):
    set_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")
    print(f"[TRAIN] Device: {device}", flush=True)
    if device.type == "cuda":
        print(f"[TRAIN] GPU: {torch.cuda.get_device_name(0)}", flush=True)

    train_dataset = PreprocessedGestureDataset(args.data_dir)
    validate_args_and_data(args, train_dataset, label="training")
    model_config = metadata_model_config(train_dataset.metadata, args)
    diffusion_config = {
        "timesteps": args.diffusion_steps,
        "beta_start": args.beta_start,
        "beta_end": args.beta_end,
    }
    print(
        f"[DATA] {len(train_dataset)} windows | input_dim={model_config['input_dim']} "
        f"| target_shape={model_config['target_shape']}",
        flush=True,
    )

    train_loader = make_dataloader(train_dataset, args, shuffle=True)
    val_loader = None
    val_data_dir = args.val_data_dir
    if val_data_dir is None:
        candidate = Path(args.data_dir).parent / "val.lmdb"
        if candidate.is_dir():
            val_data_dir = str(candidate)
            print(f"[VAL] Auto-detected validation LMDB: {val_data_dir}", flush=True)

    if val_data_dir and not args.sanity_check:
        val_dataset = PreprocessedGestureDataset(val_data_dir)
        validate_args_and_data(args, val_dataset, label="validation")
        val_config = metadata_model_config(val_dataset.metadata, args)
        if val_config["input_dim"] != model_config["input_dim"] or (
            val_config["target_shape"] != model_config["target_shape"]
        ):
            raise ValueError(f"Validation metadata does not match train metadata: {val_config}")
        val_loader = make_dataloader(val_dataset, args, shuffle=False)
        print(f"[VAL] {len(val_dataset)} validation windows", flush=True)

    model = ConditionalMotionDenoiser(**model_config).to(device)
    schedule = DiffusionSchedule(**diffusion_config).to(device)

    if args.sanity_check:
        sanity_check(model, schedule, train_loader, device, args)
        return

    optimizer = optim.AdamW(model.parameters(), lr=args.learning_rate, weight_decay=args.weight_decay)
    scaler = torch.amp.GradScaler("cuda", enabled=(device.type == "cuda"))
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with open(output_dir / "diffusion_training_config.json", "w") as f:
        json.dump(
            {
                "model_config": {**model_config, "target_shape": list(model_config["target_shape"])},
                "diffusion_config": diffusion_config,
                "dataset_metadata": train_dataset.metadata,
                "args": vars(args),
            },
            f,
            indent=2,
        )

    best_val_loss = float("inf")
    best_train_loss = float("inf")
    for epoch in range(1, args.epochs + 1):
        start = time.time()
        print(f"\n[TRAIN] Epoch {epoch}/{args.epochs}", flush=True)
        train_loss = run_epoch(
            model,
            schedule,
            train_loader,
            device,
            optimizer=optimizer,
            scaler=scaler,
            log_every=args.log_every,
            epoch=epoch,
        )
        val_loss = None
        if val_loader is not None:
            val_loss = run_epoch(model, schedule, val_loader, device)

        elapsed = time.time() - start
        if val_loss is None:
            print(f"[TRAIN] Epoch {epoch} | Loss: {train_loss:.6f} | {elapsed:.1f}s")
        else:
            print(
                f"[TRAIN] Epoch {epoch} | Train: {train_loss:.6f} "
                f"| Val: {val_loss:.6f} | {elapsed:.1f}s"
            )

        save_checkpoint(
            output_dir / "diffusion_latest.pth",
            model,
            optimizer,
            model_config,
            diffusion_config,
            train_dataset.metadata,
            epoch,
            train_loss,
            val_loss,
        )

        is_best = False
        if val_loss is not None and val_loss < best_val_loss:
            best_val_loss = val_loss
            is_best = True
        elif val_loss is None and train_loss < best_train_loss:
            best_train_loss = train_loss
            is_best = True
        if is_best:
            save_checkpoint(
                output_dir / "diffusion_best.pth",
                model,
                optimizer,
                model_config,
                diffusion_config,
                train_dataset.metadata,
                epoch,
                train_loss,
                val_loss,
            )
            print("[CHECKPOINT] Saved new best model", flush=True)

    save_checkpoint(
        output_dir / "diffusion_final.pth",
        model,
        optimizer,
        model_config,
        diffusion_config,
        train_dataset.metadata,
        args.epochs,
        train_loss,
        val_loss,
    )
    print(f"[TRAIN] Complete. Checkpoints saved to {output_dir}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Train conditional gesture diffusion")
    parser.add_argument("--data-dir", type=str, required=True, help="Path to train .lmdb or PT window dir")
    parser.add_argument("--val-data-dir", type=str, default=None, help="Optional validation dataset")
    parser.add_argument("--output-dir", type=str, default="diffusion_checkpoints")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--hidden-dim", type=int, default=256)
    parser.add_argument("--num-heads", type=int, default=8)
    parser.add_argument("--num-layers", type=int, default=6)
    parser.add_argument("--dropout", type=float, default=0.1)
    parser.add_argument("--max-frames", type=int, default=512)
    parser.add_argument("--diffusion-steps", type=int, default=1000)
    parser.add_argument("--beta-start", type=float, default=1e-4)
    parser.add_argument("--beta-end", type=float, default=0.02)
    parser.add_argument("--log-every", type=int, default=25)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--cpu", action="store_true", help="Force CPU training")
    parser.add_argument("--sanity-check", action="store_true", help="Overfit one batch")
    parser.add_argument("--sanity-steps", type=int, default=200)
    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()
