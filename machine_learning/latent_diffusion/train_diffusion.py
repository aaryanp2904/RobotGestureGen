"""Train conditional diffusion on precomputed gesture latents."""

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

from machine_learning.diffusion.train import diffusion_loss

from .dataset import make_latent_dataset
from .model import DiffusionSchedule, LatentDenoiser


def set_seed(seed: int):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def load_autoencoder_payload(path: str | None) -> tuple[dict | None, dict | None]:
    if path is None:
        return None, None
    checkpoint = torch.load(path, map_location="cpu")
    config = checkpoint.get("autoencoder_config")
    state = checkpoint.get("autoencoder_state_dict")
    if config is None or state is None:
        raise ValueError("--autoencoder must point to an autoencoder checkpoint")
    return config, state


def make_config(metadata: dict, args) -> dict:
    latent_dim = int(metadata.get("latent_dim") or metadata.get("target_shape", [args.latent_dim])[0])
    return {
        "input_dim": int(metadata["input_dim"]),
        "latent_dim": latent_dim,
        "target_shape": (latent_dim,),
        "hidden_dim": args.hidden_dim,
        "num_heads": args.num_heads,
        "num_layers": args.num_layers,
        "dropout": args.dropout,
        "max_frames": args.max_frames,
        "speaker_dim": int(metadata.get("speaker_dim", 0) or 0),
        "seed_conditioning": args.seed_frames > 0,
        "seed_frames": args.seed_frames,
        "conditioning_layers": args.conditioning_layers,
        "cond_drop_prob": args.cond_drop_prob,
        "target_mode": "latent",
        "target_representation": "gesture_latent",
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


def validate_dataset(dataset, args, label: str):
    if len(dataset) == 0:
        raise ValueError(f"No {label} latent windows found in {dataset.data_dir}")
    window_frames = int(dataset.metadata.get("window_frames", dataset[0][0].shape[0]))
    if args.max_frames < window_frames:
        raise ValueError(
            f"--max-frames={args.max_frames} is shorter than {label} window length {window_frames}"
        )
    if args.seed_frames >= window_frames:
        raise ValueError("--seed-frames must be smaller than the dataset window length")


def metadata_contract(metadata: dict) -> dict:
    return {
        "input_dim": int(metadata.get("input_dim", 0)),
        "target_shape": tuple(metadata.get("target_shape", [])),
        "target_representation": metadata.get("target_representation"),
        "speaker_dim": int(metadata.get("speaker_dim", 0) or 0),
        "feature_names": list(metadata.get("feature_names", [])),
        "conditioning_parts": list(metadata.get("conditioning_parts", [])),
        "prosody_dim": int(metadata.get("prosody_dim", 0) or 0),
        "wavlm_dim": int(metadata.get("wavlm_dim", 0) or 0),
        "text_dim": int(metadata.get("text_dim", 0) or 0),
        "gesture_energy_dim": int(metadata.get("gesture_energy_dim", 0) or 0),
        "gesture_energy_names": list(metadata.get("gesture_energy_names", [])),
        "gesture_energy_feature_names": list(metadata.get("gesture_energy_feature_names", [])),
        "gesture_energy_thresholds": list(metadata.get("gesture_energy_thresholds", [])),
        "gesture_energy_audio_thresholds": list(metadata.get("gesture_energy_audio_thresholds", [])),
        "latent_dim": int(metadata.get("latent_dim", 0) or 0),
        "source_target_shape": tuple(metadata.get("source_target_shape", [])),
        "source_target_mode": metadata.get("source_target_mode"),
        "source_target_representation": metadata.get("source_target_representation"),
    }


def run_epoch(model, schedule, loader, device, args, optimizer=None, scaler=None, epoch: int = 0):
    training = optimizer is not None
    model.train(training)
    total = 0.0
    for batch_idx, (x, latents, speaker, valid_mask) in enumerate(loader):
        x = x.to(device, non_blocking=True)
        latents = latents.to(device, non_blocking=True)
        speaker = speaker.to(device, non_blocking=True)
        valid_mask = valid_mask.to(device, non_blocking=True)
        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training):
            with torch.amp.autocast("cuda", enabled=(device.type == "cuda")):
                loss = diffusion_loss(
                    model,
                    schedule,
                    x,
                    latents,
                    speaker=speaker,
                    valid_mask=valid_mask,
                    seed_frames=args.seed_frames,
                    cond_drop_prob=args.cond_drop_prob if training else 0.0,
                    velocity_loss_weight=args.velocity_loss_weight,
                    acceleration_loss_weight=args.acceleration_loss_weight,
                )
        if training:
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), args.grad_clip)
            scaler.step(optimizer)
            scaler.update()
        total += loss.item()
        if training and (batch_idx == 0 or (batch_idx + 1) % args.log_every == 0):
            print(
                f"  Epoch {epoch} | Batch {batch_idx + 1}/{len(loader)} "
                f"| latent diffusion loss={loss.item():.6f}",
                flush=True,
            )
    return total / max(len(loader), 1)


def save_checkpoint(
    path: Path,
    model,
    optimizer,
    model_config,
    diffusion_config,
    dataset_metadata,
    autoencoder_config,
    autoencoder_state_dict,
    epoch,
    train_loss,
    val_loss,
):
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "optimizer_state_dict": optimizer.state_dict() if optimizer is not None else None,
            "model_config": {**model_config, "target_shape": list(model_config["target_shape"])},
            "diffusion_config": diffusion_config,
            "dataset_metadata": dataset_metadata,
            "autoencoder_config": autoencoder_config,
            "autoencoder_state_dict": autoencoder_state_dict,
            "model_family": "latent_diffusion",
            "epoch": epoch,
            "train_loss": train_loss,
            "val_loss": val_loss,
        },
        path,
    )


def train(args):
    set_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")
    print(f"[TRAIN-LD] Device: {device}", flush=True)

    train_dataset = make_latent_dataset(
        args.data_dir,
        cache_dir=args.cache_dir,
        source_data_dir=args.source_data_dir,
    )
    validate_dataset(train_dataset, args, "training")
    model_config = make_config(train_dataset.metadata, args)
    diffusion_config = {
        "timesteps": args.diffusion_steps,
        "beta_start": args.beta_start,
        "beta_end": args.beta_end,
        "prediction_type": "x0",
    }
    autoencoder_config, autoencoder_state = load_autoencoder_payload(args.autoencoder)
    if autoencoder_config is None:
        autoencoder_config = train_dataset.metadata.get("autoencoder_config")
    if autoencoder_state is None:
        raise ValueError("Pass --autoencoder so inference checkpoints include the decoder weights")

    train_loader = make_loader(train_dataset, args, shuffle=not args.no_shuffle)
    val_loader = None
    val_data_dir = args.val_data_dir
    if val_data_dir is None:
        candidate = Path(args.data_dir).parent / "val_latent.lmdb"
        if candidate.is_dir():
            val_data_dir = str(candidate)
            print(f"[VAL] Auto-detected validation latent LMDB: {val_data_dir}", flush=True)
    if val_data_dir:
        val_dataset = make_latent_dataset(
            val_data_dir,
            cache_dir=args.cache_dir,
            source_data_dir=args.val_source_data_dir,
        )
        validate_dataset(val_dataset, args, "validation")
        if metadata_contract(val_dataset.metadata) != metadata_contract(train_dataset.metadata):
            raise ValueError("Validation latent metadata does not match training latent metadata")
        val_loader = make_loader(val_dataset, args, shuffle=False)

    model = LatentDenoiser(**model_config).to(device)
    schedule = DiffusionSchedule(**diffusion_config).to(device)
    optimizer = optim.AdamW(model.parameters(), lr=args.learning_rate, weight_decay=args.weight_decay)
    scaler = torch.amp.GradScaler("cuda", enabled=(device.type == "cuda"))
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with open(output_dir / "latent_diffusion_training_config.json", "w") as f:
        json.dump(
            {
                "model_config": {**model_config, "target_shape": list(model_config["target_shape"])},
                "diffusion_config": diffusion_config,
                "autoencoder_config": autoencoder_config,
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
        print(f"\n[TRAIN-LD] Epoch {epoch}/{args.epochs}", flush=True)
        train_loss = run_epoch(
            model, schedule, train_loader, device, args, optimizer=optimizer, scaler=scaler, epoch=epoch
        )
        val_loss = run_epoch(model, schedule, val_loader, device, args) if val_loader is not None else None
        final_train = train_loss
        final_val = val_loss
        if val_loss is None:
            print(f"[TRAIN-LD] Loss={train_loss:.6f} | {time.time() - start:.1f}s")
        else:
            print(
                f"[TRAIN-LD] Train={train_loss:.6f} | Val={val_loss:.6f} | {time.time() - start:.1f}s"
            )
        save_checkpoint(
            output_dir / "latent_diffusion_latest.pth",
            model,
            optimizer,
            model_config,
            diffusion_config,
            train_dataset.metadata,
            autoencoder_config,
            autoencoder_state,
            epoch,
            train_loss,
            val_loss,
        )
        score = val_loss if val_loss is not None else train_loss
        if score < best:
            best = score
            save_checkpoint(
                output_dir / "latent_diffusion_best.pth",
                model,
                optimizer,
                model_config,
                diffusion_config,
                train_dataset.metadata,
                autoencoder_config,
                autoencoder_state,
                epoch,
                train_loss,
                val_loss,
            )
            print("[CHECKPOINT] Saved new best latent diffusion model", flush=True)

    save_checkpoint(
        output_dir / "latent_diffusion_final.pth",
        model,
        optimizer,
        model_config,
        diffusion_config,
        train_dataset.metadata,
        autoencoder_config,
        autoencoder_state,
        args.epochs,
        final_train,
        final_val,
    )
    print(f"[TRAIN-LD] Complete. Checkpoints saved to {output_dir}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Train latent conditional gesture diffusion")
    parser.add_argument("--data-dir", required=True, help="Path to train_latent.lmdb")
    parser.add_argument("--val-data-dir", default=None, help="Optional val_latent.lmdb")
    parser.add_argument("--autoencoder", required=True, help="Path to autoencoder_best.pth")
    parser.add_argument("--output-dir", default="latent_diffusion_checkpoints")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--cache-dir", default=None,
                        help="Optional local cache for one sharded dataset file at a time")
    parser.add_argument("--source-data-dir", default=None,
                        help="Override source preprocessed LMDB for compact training latents")
    parser.add_argument("--val-source-data-dir", default=None,
                        help="Override source preprocessed LMDB for compact validation latents")
    parser.add_argument("--no-shuffle", action="store_true",
                        help="Read training samples sequentially; recommended for sharded datasets on slow storage")
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--latent-dim", type=int, default=32)
    parser.add_argument("--hidden-dim", type=int, default=256)
    parser.add_argument("--num-heads", type=int, default=8)
    parser.add_argument("--num-layers", type=int, default=6)
    parser.add_argument("--conditioning-layers", type=int, default=2)
    parser.add_argument("--dropout", type=float, default=0.1)
    parser.add_argument("--max-frames", type=int, default=512)
    parser.add_argument("--diffusion-steps", type=int, default=100)
    parser.add_argument("--beta-start", type=float, default=1e-4)
    parser.add_argument("--beta-end", type=float, default=0.02)
    parser.add_argument("--cond-drop-prob", type=float, default=0.1)
    parser.add_argument("--velocity-loss-weight", type=float, default=0.0)
    parser.add_argument("--acceleration-loss-weight", type=float, default=0.0)
    parser.add_argument("--seed-frames", type=int, default=0)
    parser.add_argument("--grad-clip", type=float, default=1.0)
    parser.add_argument("--log-every", type=int, default=25)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--cpu", action="store_true")
    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()
