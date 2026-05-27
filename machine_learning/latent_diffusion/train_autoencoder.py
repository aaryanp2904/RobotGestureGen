"""Train a temporal motion autoencoder on preprocessed gesture windows."""

from __future__ import annotations

import argparse
import json
import random
import time
from pathlib import Path

import numpy as np
import torch
import torch.optim as optim
from torch.utils.data import DataLoader

from .dataset import PreprocessedGestureDataset
from .model import MotionAutoencoder


def set_seed(seed: int):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def masked_mse(values: torch.Tensor, target: torch.Tensor, valid_mask: torch.Tensor | None) -> torch.Tensor:
    squared_error = torch.square(values - target)
    if valid_mask is None:
        return squared_error.mean()
    while valid_mask.ndim < squared_error.ndim:
        valid_mask = valid_mask.unsqueeze(-1)
    valid_mask = valid_mask.to(device=squared_error.device, dtype=squared_error.dtype)
    return (squared_error * valid_mask).sum() / valid_mask.expand_as(squared_error).sum().clamp(min=1.0)


def reconstruction_loss(
    reconstruction: torch.Tensor,
    target: torch.Tensor,
    valid_mask: torch.Tensor,
    velocity_weight: float,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    rec = masked_mse(reconstruction, target, valid_mask)
    vel = reconstruction.new_zeros(())
    if velocity_weight > 0 and reconstruction.shape[1] > 1:
        pred_vel = reconstruction[:, 1:] - reconstruction[:, :-1]
        true_vel = target[:, 1:] - target[:, :-1]
        vel_mask = valid_mask[:, 1:] * valid_mask[:, :-1]
        vel = masked_mse(pred_vel, true_vel, vel_mask)
    return rec + velocity_weight * vel, rec, vel


def make_config(metadata: dict, args) -> dict:
    return {
        "target_shape": tuple(metadata.get("target_shape", [10])),
        "latent_dim": args.latent_dim,
        "hidden_dim": args.hidden_dim,
        "num_heads": args.num_heads,
        "num_encoder_layers": args.num_encoder_layers,
        "num_decoder_layers": args.num_decoder_layers,
        "dropout": args.dropout,
        "max_frames": args.max_frames,
    }


def make_loader(dataset, args, shuffle: bool) -> DataLoader:
    return DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=shuffle,
        num_workers=args.num_workers,
        pin_memory=torch.cuda.is_available() and not args.cpu,
        persistent_workers=args.num_workers > 0,
    )


def run_epoch(model, loader, device, args, optimizer=None, scaler=None, epoch: int = 0):
    training = optimizer is not None
    model.train(training)
    total = 0.0
    total_rec = 0.0
    total_vel = 0.0
    for batch_idx, (_, motion, _, valid_mask) in enumerate(loader):
        motion = motion.to(device, non_blocking=True)
        valid_mask = valid_mask.to(device, non_blocking=True)
        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training):
            with torch.amp.autocast("cuda", enabled=(device.type == "cuda")):
                reconstruction, _ = model(motion)
                loss, rec, vel = reconstruction_loss(
                    reconstruction, motion, valid_mask, args.velocity_loss_weight
                )
        if training:
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), args.grad_clip)
            scaler.step(optimizer)
            scaler.update()
        total += loss.item()
        total_rec += rec.item()
        total_vel += vel.item()
        if training and (batch_idx == 0 or (batch_idx + 1) % args.log_every == 0):
            print(
                f"  Epoch {epoch} | Batch {batch_idx + 1}/{len(loader)} "
                f"| loss={loss.item():.6f} rec={rec.item():.6f} vel={vel.item():.6f}",
                flush=True,
            )
    count = max(len(loader), 1)
    return total / count, total_rec / count, total_vel / count


def save_checkpoint(path: Path, model, optimizer, autoencoder_config, metadata, epoch, train_loss, val_loss):
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "autoencoder_state_dict": model.state_dict(),
            "optimizer_state_dict": optimizer.state_dict() if optimizer is not None else None,
            "autoencoder_config": {
                **autoencoder_config,
                "target_shape": list(autoencoder_config["target_shape"]),
            },
            "dataset_metadata": metadata,
            "epoch": epoch,
            "train_loss": train_loss,
            "val_loss": val_loss,
        },
        path,
    )


def train(args):
    set_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")
    print(f"[TRAIN-AE] Device: {device}", flush=True)

    train_dataset = PreprocessedGestureDataset(args.data_dir)
    if len(train_dataset) == 0:
        raise ValueError(f"No training windows found in {args.data_dir}")
    config = make_config(train_dataset.metadata, args)
    if args.max_frames < int(train_dataset.metadata.get("window_frames", 0)):
        raise ValueError("--max-frames is shorter than the dataset window length")
    train_loader = make_loader(train_dataset, args, shuffle=True)

    val_loader = None
    val_data_dir = args.val_data_dir
    if val_data_dir is None:
        candidate = Path(args.data_dir).parent / "val.lmdb"
        if candidate.is_dir():
            val_data_dir = str(candidate)
            print(f"[VAL] Auto-detected validation LMDB: {val_data_dir}", flush=True)
    if val_data_dir:
        val_dataset = PreprocessedGestureDataset(val_data_dir)
        if tuple(val_dataset.metadata.get("target_shape", [])) != tuple(config["target_shape"]):
            raise ValueError("Validation target_shape does not match training target_shape")
        val_loader = make_loader(val_dataset, args, shuffle=False)

    model = MotionAutoencoder(**config).to(device)
    optimizer = optim.AdamW(model.parameters(), lr=args.learning_rate, weight_decay=args.weight_decay)
    scaler = torch.amp.GradScaler("cuda", enabled=(device.type == "cuda"))
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with open(output_dir / "autoencoder_training_config.json", "w") as f:
        json.dump(
            {
                "autoencoder_config": {**config, "target_shape": list(config["target_shape"])},
                "dataset_metadata": train_dataset.metadata,
                "args": vars(args),
            },
            f,
            indent=2,
        )

    best = float("inf")
    final_train = None
    final_val = None
    for epoch in range(1, args.epochs + 1):
        start = time.time()
        print(f"\n[TRAIN-AE] Epoch {epoch}/{args.epochs}", flush=True)
        train_loss, train_rec, train_vel = run_epoch(
            model, train_loader, device, args, optimizer=optimizer, scaler=scaler, epoch=epoch
        )
        val_loss = None
        if val_loader is not None:
            val_loss, val_rec, val_vel = run_epoch(model, val_loader, device, args)
            print(
                f"[TRAIN-AE] Train={train_loss:.6f} rec={train_rec:.6f} vel={train_vel:.6f} "
                f"| Val={val_loss:.6f} rec={val_rec:.6f} vel={val_vel:.6f} | {time.time() - start:.1f}s"
            )
        else:
            print(
                f"[TRAIN-AE] Loss={train_loss:.6f} rec={train_rec:.6f} "
                f"vel={train_vel:.6f} | {time.time() - start:.1f}s"
            )

        final_train = train_loss
        final_val = val_loss
        save_checkpoint(
            output_dir / "autoencoder_latest.pth",
            model,
            optimizer,
            config,
            train_dataset.metadata,
            epoch,
            train_loss,
            val_loss,
        )
        score = val_loss if val_loss is not None else train_loss
        if score < best:
            best = score
            save_checkpoint(
                output_dir / "autoencoder_best.pth",
                model,
                optimizer,
                config,
                train_dataset.metadata,
                epoch,
                train_loss,
                val_loss,
            )
            print("[CHECKPOINT] Saved new best autoencoder", flush=True)

    save_checkpoint(
        output_dir / "autoencoder_final.pth",
        model,
        optimizer,
        config,
        train_dataset.metadata,
        args.epochs,
        final_train,
        final_val,
    )
    print(f"[TRAIN-AE] Complete. Checkpoints saved to {output_dir}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Train latent motion autoencoder")
    parser.add_argument("--data-dir", required=True, help="Path to train .lmdb")
    parser.add_argument("--val-data-dir", default=None, help="Optional validation .lmdb")
    parser.add_argument("--output-dir", default="latent_autoencoder_checkpoints")
    parser.add_argument("--epochs", type=int, default=75)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--latent-dim", type=int, default=32)
    parser.add_argument("--hidden-dim", type=int, default=256)
    parser.add_argument("--num-heads", type=int, default=8)
    parser.add_argument("--num-encoder-layers", type=int, default=3)
    parser.add_argument("--num-decoder-layers", type=int, default=3)
    parser.add_argument("--dropout", type=float, default=0.1)
    parser.add_argument("--max-frames", type=int, default=512)
    parser.add_argument("--velocity-loss-weight", type=float, default=0.1)
    parser.add_argument("--grad-clip", type=float, default=1.0)
    parser.add_argument("--log-every", type=int, default=25)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--cpu", action="store_true")
    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()
