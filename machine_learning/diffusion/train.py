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
        "speaker_dim": int(metadata.get("speaker_dim", 0)),
        "seed_conditioning": args.seed_frames > 0,
        "seed_frames": args.seed_frames,
        "conditioning_layers": args.conditioning_layers,
        "conditioning_encoder": "transformer",
        "cross_attention": True,
        "cond_drop_prob": args.cond_drop_prob,
        "target_mode": metadata.get("target_mode", "angle"),
        "target_representation": metadata.get("target_representation", "unknown"),
    }


def metadata_training_contract(metadata: dict) -> dict:
    """Fields that must agree between train/validation datasets and inference stats."""
    return {
        "input_dim": int(metadata.get("input_dim", 1536)),
        "target_shape": tuple(metadata.get("target_shape", [12, 3])),
        "target_mode": metadata.get("target_mode", "angle"),
        "target_type": metadata.get("target_type", "unknown"),
        "target_representation": metadata.get("target_representation", "unknown"),
        "feature_names": list(metadata.get("feature_names", [])),
        "target_names": list(metadata.get("target_names", [])),
        "prosody_dim": metadata.get("prosody_dim"),
        "wavlm_dim": metadata.get("wavlm_dim"),
        "wavlm_model": metadata.get("wavlm_model"),
        "text_dim": metadata.get("text_dim"),
        "speaker_dim": metadata.get("speaker_dim", 0),
        "speaker_id_map": dict(metadata.get("speaker_id_map", {})),
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
    sample_x = dataset[0][0]
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
    if args.conditioning_layers < 0:
        raise ValueError("--conditioning-layers cannot be negative")
    if args.log_every <= 0:
        raise ValueError("--log-every must be positive")
    if args.sanity_check and args.sanity_steps <= 0:
        raise ValueError("--sanity-steps must be positive")
    if args.learning_rate <= 0:
        raise ValueError("--learning-rate must be positive")
    if args.weight_decay < 0:
        raise ValueError("--weight-decay cannot be negative")
    if not 0.0 <= args.cond_drop_prob < 1.0:
        raise ValueError("--cond-drop-prob must be in [0, 1)")
    if args.velocity_loss_weight < 0:
        raise ValueError("--velocity-loss-weight cannot be negative")
    if args.acceleration_loss_weight < 0:
        raise ValueError("--acceleration-loss-weight cannot be negative")
    if args.seed_frames < 0:
        raise ValueError("--seed-frames cannot be negative")
    window_frames = dataset_window_frames(dataset)
    if args.seed_frames >= window_frames:
        raise ValueError(
            f"--seed-frames={args.seed_frames} must be smaller than the dataset window "
            f"length ({window_frames} frames)"
        )
    if args.max_frames < window_frames:
        raise ValueError(
            f"--max-frames={args.max_frames} is shorter than the dataset window "
            f"length ({window_frames} frames)"
        )


def validate_pose_target_metadata(metadata: dict, source: str) -> None:
    """Diffusion is trained on normalized pose targets, never frame deltas."""
    target_mode = metadata.get("target_mode", "angle")
    target_type = str(metadata.get("target_type", ""))
    if target_mode != "angle" or "delta" in target_type:
        raise ValueError(
            f"{source} uses target_mode={target_mode!r}, target_type={target_type!r}. "
            "Conditional diffusion must train directly on normalized poses; "
            "rerun machine_learning.diffusion.preprocessing without --target-mode delta."
        )


def masked_mse(values: torch.Tensor, target: torch.Tensor, valid_mask: torch.Tensor | None) -> torch.Tensor:
    squared_error = torch.square(values - target)
    if valid_mask is None:
        return squared_error.mean()
    while valid_mask.ndim < squared_error.ndim:
        valid_mask = valid_mask.unsqueeze(-1)
    valid_mask = valid_mask.to(device=squared_error.device, dtype=squared_error.dtype)
    return (squared_error * valid_mask).sum() / valid_mask.expand_as(squared_error).sum().clamp(min=1.0)


def weighted_mse(
    values: torch.Tensor,
    target: torch.Tensor,
    valid_mask: torch.Tensor | None,
    weights: torch.Tensor | None,
) -> torch.Tensor:
    """MSE where weights reduce contribution without being normalized away."""
    squared_error = torch.square(values - target)
    if valid_mask is None:
        valid_mask = torch.ones_like(squared_error[..., :1])
    while valid_mask.ndim < squared_error.ndim:
        valid_mask = valid_mask.unsqueeze(-1)
    valid_mask = valid_mask.to(device=squared_error.device, dtype=squared_error.dtype)
    if weights is None:
        weights = torch.ones_like(valid_mask)
    while weights.ndim < squared_error.ndim:
        weights = weights.unsqueeze(-1)
    weights = weights.to(device=squared_error.device, dtype=squared_error.dtype)
    numerator = (squared_error * valid_mask * weights).sum()
    denominator = valid_mask.expand_as(squared_error).sum().clamp(min=1.0)
    return numerator / denominator


def reconstruct_x0(
    schedule: DiffusionSchedule,
    noisy_motion: torch.Tensor,
    model_prediction: torch.Tensor,
    timesteps: torch.Tensor,
) -> torch.Tensor:
    """Convert an x0/epsilon model output into a clean-motion estimate."""
    if schedule.prediction_type == "x0":
        return model_prediction
    sqrt_alpha = schedule._extract(schedule.sqrt_alpha_cumprod, timesteps, noisy_motion.shape)
    sqrt_one_minus = schedule._extract(
        schedule.sqrt_one_minus_alpha_cumprod, timesteps, noisy_motion.shape
    )
    return (noisy_motion - sqrt_one_minus * model_prediction) / sqrt_alpha.clamp(min=1e-8)


def motion_smoothness_loss(
    predicted_x0: torch.Tensor,
    clean_motion: torch.Tensor,
    valid_mask: torch.Tensor | None,
    timestep_weight: torch.Tensor | None,
    velocity_weight: float,
    acceleration_weight: float,
) -> torch.Tensor:
    """Penalize jitter in the predicted clean motion with finite differences."""
    loss = predicted_x0.new_zeros(())
    if velocity_weight > 0 and predicted_x0.shape[1] > 1:
        pred_vel = predicted_x0[:, 1:] - predicted_x0[:, :-1]
        true_vel = clean_motion[:, 1:] - clean_motion[:, :-1]
        vel_mask = None
        if valid_mask is not None:
            vel_mask = valid_mask[:, 1:] * valid_mask[:, :-1]
        vel_weight = timestep_weight
        if vel_weight is not None:
            vel_weight = vel_weight[:, 1:] * vel_weight[:, :-1]
        loss = loss + velocity_weight * weighted_mse(pred_vel, true_vel, vel_mask, vel_weight)
    if acceleration_weight > 0 and predicted_x0.shape[1] > 2:
        pred_vel = predicted_x0[:, 1:] - predicted_x0[:, :-1]
        true_vel = clean_motion[:, 1:] - clean_motion[:, :-1]
        pred_acc = pred_vel[:, 1:] - pred_vel[:, :-1]
        true_acc = true_vel[:, 1:] - true_vel[:, :-1]
        acc_mask = None
        if valid_mask is not None:
            acc_mask = valid_mask[:, 2:] * valid_mask[:, 1:-1] * valid_mask[:, :-2]
        acc_weight = timestep_weight
        if acc_weight is not None:
            acc_weight = acc_weight[:, 2:] * acc_weight[:, 1:-1] * acc_weight[:, :-2]
        loss = loss + acceleration_weight * weighted_mse(pred_acc, true_acc, acc_mask, acc_weight)
    return loss


def diffusion_loss(
    model: nn.Module,
    schedule: DiffusionSchedule,
    conditioning: torch.Tensor,
    clean_motion: torch.Tensor,
    speaker: torch.Tensor | None = None,
    valid_mask: torch.Tensor | None = None,
    seed_frames: int = 0,
    cond_drop_prob: float = 0.0,
    timesteps: torch.Tensor | None = None,
    noise: torch.Tensor | None = None,
    velocity_loss_weight: float = 0.0,
    acceleration_loss_weight: float = 0.0,
) -> torch.Tensor:
    if conditioning.ndim != 3:
        raise ValueError(f"Expected conditioning shape (B, T, C), got {tuple(conditioning.shape)}")
    if conditioning.shape[-1] != model.input_dim:
        raise ValueError(
            f"Expected conditioning dim {model.input_dim}, got {conditioning.shape[-1]}"
        )
    if tuple(clean_motion.shape[1:]) != (conditioning.shape[1], *model.target_shape):
        raise ValueError(
            f"Expected clean motion shape (B, {conditioning.shape[1]}, {model.target_shape}), "
            f"got {tuple(clean_motion.shape)}"
        )

    batch_size = clean_motion.shape[0]
    if timesteps is None:
        timesteps = torch.randint(
            0,
            schedule.timesteps,
            (batch_size,),
            device=clean_motion.device,
            dtype=torch.long,
        )
    elif timesteps.shape != (batch_size,):
        raise ValueError(f"Expected timesteps shape ({batch_size},), got {tuple(timesteps.shape)}")
    if noise is None:
        noise = torch.randn_like(clean_motion)
    elif tuple(noise.shape) != tuple(clean_motion.shape):
        raise ValueError(f"Expected noise shape {tuple(clean_motion.shape)}, got {tuple(noise.shape)}")
    noisy_motion = schedule.q_sample(clean_motion, timesteps, noise)
    if cond_drop_prob > 0.0:
        keep = (
            torch.rand((batch_size, 1, 1), device=conditioning.device)
            >= cond_drop_prob
        ).to(conditioning.dtype)
        conditioning = conditioning * keep
        if speaker is not None:
            speaker = speaker * keep.reshape(batch_size, 1)
    seed_motion = None
    seed_mask = None
    if getattr(model, "seed_conditioning", False) and seed_frames > 0:
        seed_frames = min(seed_frames, clean_motion.shape[1])
        seed_motion = torch.zeros_like(clean_motion)
        seed_motion[:, :seed_frames] = clean_motion[:, :seed_frames]
        seed_mask = torch.zeros(
            (clean_motion.shape[0], clean_motion.shape[1], 1),
            device=clean_motion.device,
            dtype=clean_motion.dtype,
        )
        seed_mask[:, :seed_frames] = 1.0
    prediction = model(
        noisy_motion,
        conditioning,
        timesteps,
        seed_motion=seed_motion,
        seed_mask=seed_mask,
        speaker=speaker,
    )
    target = clean_motion if schedule.prediction_type == "x0" else noise
    loss = masked_mse(prediction, target, valid_mask)
    if velocity_loss_weight > 0 or acceleration_loss_weight > 0:
        predicted_x0 = reconstruct_x0(schedule, noisy_motion, prediction, timesteps)
        timestep_weight = None
        if schedule.prediction_type == "epsilon":
            # x0 estimates are unreliable at high-noise timesteps early in training.
            # Down-weight the auxiliary motion loss when little signal remains.
            timestep_weight = schedule._extract(
                schedule.alpha_cumprod, timesteps, clean_motion.shape
            ).detach().expand(clean_motion.shape[0], clean_motion.shape[1], 1)
        loss = loss + motion_smoothness_loss(
            predicted_x0,
            clean_motion,
            valid_mask,
            timestep_weight,
            velocity_loss_weight,
            acceleration_loss_weight,
        )
    return loss


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

    for batch_idx, (batch_x, batch_y, batch_speaker, batch_mask) in enumerate(dataloader):
        batch_x = batch_x.to(device, non_blocking=True)
        batch_y = batch_y.to(device, non_blocking=True)
        batch_speaker = batch_speaker.to(device, non_blocking=True)
        batch_mask = batch_mask.to(device, non_blocking=True)

        if training:
            optimizer.zero_grad(set_to_none=True)

        with torch.set_grad_enabled(training):
            with torch.amp.autocast("cuda", enabled=(device.type == "cuda")):
                loss = diffusion_loss(
                    model,
                    schedule,
                    batch_x,
                    batch_y,
                    speaker=batch_speaker,
                    valid_mask=batch_mask,
                    seed_frames=getattr(model, "training_seed_frames", 0),
                    cond_drop_prob=getattr(model, "cond_drop_prob", 0.0) if training else 0.0,
                    velocity_loss_weight=getattr(model, "velocity_loss_weight", 0.0),
                    acceleration_loss_weight=getattr(model, "acceleration_loss_weight", 0.0),
                )

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
                f"| Diffusion loss: {loss.item():.6f}",
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
    print(
        f"[SANITY] Overfitting one batch with diffusion {schedule.prediction_type} prediction",
        flush=True,
    )
    optimizer = optim.AdamW(model.parameters(), lr=args.learning_rate)
    scaler = torch.amp.GradScaler("cuda", enabled=(device.type == "cuda"))
    batch_x, batch_y, batch_speaker, batch_mask = next(iter(dataloader))
    batch_x = batch_x.to(device)
    batch_y = batch_y.to(device)
    batch_speaker = batch_speaker.to(device)
    batch_mask = batch_mask.to(device)
    print(
        f"[SANITY] X={tuple(batch_x.shape)} Y={tuple(batch_y.shape)} "
        f"Speaker={tuple(batch_speaker.shape)}",
        flush=True,
    )
    fixed_timesteps = torch.randint(
        0,
        schedule.timesteps,
        (batch_y.shape[0],),
        device=device,
        dtype=torch.long,
    )
    fixed_noise = torch.randn_like(batch_y)
    previous_training_mode = model.training
    model.eval()
    print(
        "[SANITY] Using fixed diffusion noise/timesteps, cond_drop_prob=0, and dropout disabled",
        flush=True,
    )

    for step in range(args.sanity_steps):
        optimizer.zero_grad(set_to_none=True)
        with torch.amp.autocast("cuda", enabled=(device.type == "cuda")):
            loss = diffusion_loss(
                model,
                schedule,
                batch_x,
                batch_y,
                speaker=batch_speaker,
                valid_mask=batch_mask,
                seed_frames=args.seed_frames,
                cond_drop_prob=0.0,
                timesteps=fixed_timesteps,
                noise=fixed_noise,
            )
        scaler.scale(loss).backward()
        scaler.unscale_(optimizer)
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        scaler.step(optimizer)
        scaler.update()
        if step == 0 or (step + 1) % 25 == 0:
            print(f"[SANITY] {step + 1}/{args.sanity_steps}  Loss: {loss.item():.6f}")
    model.train(previous_training_mode)


def train(args):
    set_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")
    print(f"[TRAIN] Device: {device}", flush=True)
    if device.type == "cuda":
        print(f"[TRAIN] GPU: {torch.cuda.get_device_name(0)}", flush=True)

    train_dataset = PreprocessedGestureDataset(args.data_dir)
    validate_args_and_data(args, train_dataset, label="training")
    validate_pose_target_metadata(train_dataset.metadata, str(args.data_dir))
    model_config = metadata_model_config(train_dataset.metadata, args)
    training_contract = metadata_training_contract(train_dataset.metadata)
    diffusion_config = {
        "timesteps": args.diffusion_steps,
        "beta_start": args.beta_start,
        "beta_end": args.beta_end,
        "prediction_type": args.prediction_type,
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
        validate_pose_target_metadata(val_dataset.metadata, str(val_data_dir))
        val_contract = metadata_training_contract(val_dataset.metadata)
        if val_contract != training_contract:
            raise ValueError(
                f"Validation metadata does not match training metadata: "
                f"{val_contract} vs {training_contract}"
            )
        val_loader = make_dataloader(val_dataset, args, shuffle=False)
        print(f"[VAL] {len(val_dataset)} validation windows", flush=True)

    model = ConditionalMotionDenoiser(**model_config).to(device)
    model.training_seed_frames = args.seed_frames
    model.cond_drop_prob = args.cond_drop_prob
    model.velocity_loss_weight = args.velocity_loss_weight
    model.acceleration_loss_weight = args.acceleration_loss_weight
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
    parser.add_argument("--conditioning-layers", type=int, default=2,
                        help="Number of Transformer layers in the audio/text conditioning encoder")
    parser.add_argument("--dropout", type=float, default=0.1)
    parser.add_argument("--max-frames", type=int, default=512)
    parser.add_argument("--diffusion-steps", type=int, default=1000)
    parser.add_argument("--beta-start", type=float, default=1e-4)
    parser.add_argument("--beta-end", type=float, default=0.02)
    parser.add_argument("--prediction-type", choices=["x0", "epsilon"], default="epsilon",
                        help="Train the denoiser to predict clean motion x0 or diffusion noise")
    parser.add_argument("--cond-drop-prob", type=float, default=0.1,
                        help="Probability of dropping conditioning during training for classifier-free guidance")
    parser.add_argument("--velocity-loss-weight", type=float, default=0.005,
                        help="Auxiliary clean-motion velocity loss weight to reduce frame-to-frame jitter")
    parser.add_argument("--acceleration-loss-weight", type=float, default=0.001,
                        help="Auxiliary clean-motion acceleration loss weight to reduce twitchy motion")
    parser.add_argument("--seed-frames", type=int, default=15,
                        help="Number of leading ground-truth frames used as seed gesture conditioning")
    parser.add_argument("--log-every", type=int, default=25)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--cpu", action="store_true", help="Force CPU training")
    parser.add_argument("--sanity-check", action="store_true", help="Overfit one batch")
    parser.add_argument("--sanity-steps", type=int, default=200)
    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()
