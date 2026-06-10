#!/usr/bin/env python3
"""Generate NAO joint-angle gestures with a latent diffusion checkpoint."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from beat2_nao.infer_nao import (  # noqa: E402
    build_features,
    build_speaker_condition,
    clamp_nao_angles,
    infer_energy_features_from_prosody,
    limit_nao_velocity,
    load_words_from_textgrid,
    overlap_weights,
    reconstruct_nao_angles,
    resolve_textgrid,
    resolve_wav,
    save_metadata,
    smooth_nao_angles,
    speaker_id_from_clip_id,
    window_starts,
)
from beat2_nao.config import AUDIO_DIR, MOCAP_FPS, TEXTGRID_DIR  # noqa: E402
from beat2_nao.nao_constants import NAO_JOINTS  # noqa: E402
from beat2_nao.preprocess_nao import PROSODY_FEATURE_NAMES  # noqa: E402

from .model import DiffusionSchedule, LatentDenoiser, MotionAutoencoder  # noqa: E402


LATENT_DENOISER_KEYS = {
    "input_dim",
    "latent_dim",
    "hidden_dim",
    "num_heads",
    "num_layers",
    "dropout",
    "max_frames",
    "speaker_dim",
    "seed_conditioning",
    "seed_frames",
    "conditioning_layers",
    "cond_drop_prob",
    "target_shape",
    "target_mode",
    "target_representation",
}


def latent_model_kwargs(model_config: dict) -> dict:
    kwargs = {key: value for key, value in model_config.items() if key in LATENT_DENOISER_KEYS}
    if "target_shape" in kwargs:
        kwargs["target_shape"] = tuple(kwargs["target_shape"])
    if "latent_dim" in kwargs:
        kwargs["latent_dim"] = int(kwargs["latent_dim"])
    return kwargs


def load_checkpoint(path: Path) -> dict:
    checkpoint = torch.load(path, map_location="cpu")
    required = ["model_state_dict", "model_config", "diffusion_config", "autoencoder_state_dict", "autoencoder_config"]
    missing = [key for key in required if key not in checkpoint]
    if missing:
        raise ValueError(f"Latent diffusion checkpoint is missing required keys: {missing}")
    return checkpoint


def validate_latent_contract(model_config: dict, autoencoder_config: dict, stats: dict, metadata: dict) -> str:
    if int(model_config.get("input_dim", 0)) <= 0:
        raise ValueError("Checkpoint model_config is missing input_dim")
    if int(model_config.get("latent_dim", 0)) <= 0:
        raise ValueError("Checkpoint model_config is missing latent_dim")
    if tuple(model_config.get("target_shape", ())) != (int(model_config["latent_dim"]),):
        raise ValueError(
            f"Checkpoint latent target_shape {model_config.get('target_shape')} does not match "
            f"latent_dim={model_config['latent_dim']}"
        )
    if int(autoencoder_config.get("latent_dim", 0)) != int(model_config["latent_dim"]):
        raise ValueError(
            f"Autoencoder latent_dim={autoencoder_config.get('latent_dim')} does not match "
            f"denoiser latent_dim={model_config['latent_dim']}"
        )
    if metadata.get("target_representation") != "gesture_latent":
        raise ValueError("Checkpoint dataset_metadata must describe gesture_latent targets")
    source_representation = (
        metadata.get("source_target_representation")
        or stats.get("target_representation")
        or "nao_angles"
    )
    if source_representation != "nao_angles":
        raise ValueError(
            "This inference path only supports latent checkpoints whose decoder outputs "
            f"NAO angles, got source_target_representation={source_representation!r}"
        )
    source_shape = tuple(metadata.get("source_target_shape") or autoencoder_config.get("target_shape", []))
    if source_shape != (len(NAO_JOINTS),):
        raise ValueError(f"Expected decoded NAO shape ({len(NAO_JOINTS)},), got {source_shape}")
    for key in ("prosody_mean", "prosody_std", "nao_mean", "nao_std"):
        if key not in stats:
            raise ValueError(f"Stats file is missing required key: {key}")
    wavlm_dim = int(metadata.get("wavlm_dim", stats.get("wavlm_dim", 0)) or 0)
    if wavlm_dim > 0:
        for key in ("wavlm_mean", "wavlm_std"):
            if key not in stats:
                raise ValueError(f"Stats file is missing required key for WavLM inference: {key}")
    if int(metadata.get("gesture_energy_dim", 0) or 0) > 0 and "gesture_energy_audio_thresholds" not in stats:
        raise ValueError("Stats file is missing gesture_energy_audio_thresholds")
    stats_joint_names = stats.get("nao_joint_names")
    if stats_joint_names is not None and list(stats_joint_names) != list(NAO_JOINTS):
        raise ValueError("Stats NAO joint order does not match inference order")
    return metadata.get("source_target_mode") or stats.get("target_mode") or "angle"


def energy_labels_for_features(features: np.ndarray, metadata: dict, stats: dict) -> np.ndarray | None:
    dim = int(metadata.get("gesture_energy_dim", 0) or 0)
    if dim <= 0:
        return None
    prosody = features[:, :len(PROSODY_FEATURE_NAMES)]
    _energy_features, labels = infer_energy_features_from_prosody(prosody, stats, metadata)
    return labels


def apply_energy_rest_gate(
    pred_norm: np.ndarray,
    energy_labels: np.ndarray | None,
    enabled: bool,
    rest_blend: float,
) -> np.ndarray:
    if not enabled or energy_labels is None or rest_blend <= 0.0:
        return pred_norm
    if len(energy_labels) != len(pred_norm):
        raise ValueError("Energy labels length does not match prediction length")
    out = pred_norm.copy()
    low_mask = energy_labels == 0
    out[low_mask] = (1.0 - rest_blend) * out[low_mask]
    return out.astype(np.float32)


@torch.no_grad()
def run_latent_diffusion_inference(
    model: LatentDenoiser,
    autoencoder: MotionAutoencoder,
    schedule: DiffusionSchedule,
    features: np.ndarray,
    window_frames: int,
    stride_frames: int,
    device: torch.device,
    deterministic: bool = False,
    seed_frames: int = 0,
    speaker: torch.Tensor | None = None,
    sampler: str = "ddim",
    sample_steps: int | None = 50,
    guidance_scale: float = 1.0,
) -> np.ndarray:
    total_frames = features.shape[0]
    motion_shape = tuple(autoencoder.target_shape)
    latent_shape = (model.latent_dim,)
    pred_sum = torch.zeros((total_frames, *motion_shape), dtype=torch.float32)
    pred_count = torch.zeros((total_frames,) + (1,) * len(motion_shape), dtype=torch.float32)
    latent_sum = torch.zeros((total_frames, *latent_shape), dtype=torch.float32)
    latent_count = torch.zeros((total_frames, 1), dtype=torch.float32)

    padded_features = features
    if total_frames < window_frames:
        pad = np.repeat(features[-1:], window_frames - total_frames, axis=0)
        padded_features = np.concatenate([features, pad], axis=0)

    starts = window_starts(total_frames, window_frames, stride_frames)
    weights = overlap_weights(window_frames)
    motion_weights = weights.reshape(window_frames, *((1,) * len(motion_shape)))
    latent_weights = weights.reshape(window_frames, 1)
    active_steps = (
        min(sample_steps, schedule.timesteps)
        if sampler == "ddim" and sample_steps
        else schedule.timesteps
    )
    print(f"[LATENT] Sampling {len(starts)} windows x {active_steps} {sampler.upper()} steps")

    model.eval()
    autoencoder.eval()
    for window_idx, start in enumerate(starts, start=1):
        end = start + window_frames
        conditioning = torch.from_numpy(padded_features[start:end]).unsqueeze(0).to(device)
        seed_latents = None
        seed_mask = None
        active_seed = min(seed_frames, window_frames)
        if getattr(model, "seed_conditioning", False) and active_seed > 0:
            seed_latents = torch.zeros((1, window_frames, model.latent_dim), dtype=torch.float32)
            seed_mask = torch.zeros((1, window_frames, 1), dtype=torch.float32)
            if start > 0:
                seed_end = min(start + active_seed, total_frames)
                valid_seed = seed_end - start
                if valid_seed > 0:
                    known_count = latent_count[start:seed_end]
                    known_mask = (known_count > 0).reshape(valid_seed)
                    if known_mask.any():
                        known = latent_sum[start:seed_end] / known_count.clamp(min=1)
                        known_idx = torch.nonzero(known_mask, as_tuple=False).flatten()
                        seed_latents[0, known_idx] = known[known_idx]
                        seed_mask[0, known_idx, 0] = 1.0
            seed_latents = seed_latents.to(device)
            seed_mask = seed_mask.to(device)

        latents = schedule.sample(
            model,
            conditioning,
            latent_shape,
            seed_motion=seed_latents,
            seed_mask=seed_mask,
            speaker=speaker,
            add_noise=not deterministic,
            sampler=sampler,
            sample_steps=sample_steps,
            guidance_scale=guidance_scale,
        )
        if seed_latents is not None and start > 0 and seed_mask is not None:
            mask = seed_mask.bool().squeeze(0).squeeze(-1)
            latents_cpu = latents.squeeze(0).cpu()
            latents_cpu[mask.cpu()] = seed_latents.squeeze(0).cpu()[mask.cpu()]
            latents = latents_cpu.unsqueeze(0).to(device)

        decoded = autoencoder.decode(latents).squeeze(0).cpu()
        latents_cpu = latents.squeeze(0).cpu()
        valid_end = min(end, total_frames)
        valid_len = valid_end - start
        pred_sum[start:valid_end] += decoded[:valid_len] * motion_weights[:valid_len]
        pred_count[start:valid_end] += motion_weights[:valid_len]
        latent_sum[start:valid_end] += latents_cpu[:valid_len] * latent_weights[:valid_len]
        latent_count[start:valid_end] += latent_weights[:valid_len]
        if window_idx == 1 or window_idx == len(starts) or window_idx % 10 == 0:
            print(f"[LATENT] Window {window_idx}/{len(starts)}")

    return (pred_sum / pred_count.clamp(min=1)).numpy()


def main():
    parser = argparse.ArgumentParser(description="Generate NAO gestures from latent diffusion")
    parser.add_argument("--checkpoint", required=True, help="Path to latent_diffusion_best.pth")
    parser.add_argument("--stats", required=True, help="Path to normalization_stats.json")
    parser.add_argument("--output", required=True, help="Output .npy path")
    parser.add_argument("--wav", default=None, help="Input WAV path")
    parser.add_argument("--textgrid", default=None, help="Optional TextGrid path")
    parser.add_argument("--clip-id", default=None, help="Resolve WAV/TextGrid from BEAT2 directories")
    parser.add_argument("--speaker-id", default=None)
    parser.add_argument("--audio-dir", default=str(AUDIO_DIR))
    parser.add_argument("--textgrid-dir", default=str(TEXTGRID_DIR))
    parser.add_argument("--fps", type=int, default=MOCAP_FPS)
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--diffusion-deterministic", action="store_true")
    parser.add_argument("--sampler", choices=["ddpm", "ddim"], default="ddim")
    parser.add_argument("--sample-steps", type=int, default=50)
    parser.add_argument("--guidance-scale", type=float, default=1.0)
    parser.add_argument("--diffusion-seed-frames", type=int, default=None)
    parser.add_argument("--smooth-window", type=int, default=1)
    parser.add_argument("--velocity-limit", action="store_true")
    parser.add_argument("--velocity-scale", type=float, default=1.0)
    parser.add_argument("--energy-gate", dest="energy_gate", action="store_true", default=True,
                        help="Blend low-energy regions toward the normalized rest pose (default)")
    parser.add_argument("--disable-energy-gate", dest="energy_gate", action="store_false",
                        help="Disable low-energy rest blending")
    parser.add_argument("--rest-blend", type=float, default=0.5,
                        help="Blend strength toward normalized rest pose for low-energy frames")
    parser.add_argument("--text-cpu", action="store_true")
    parser.add_argument("--wavlm-cpu", action="store_true")
    args = parser.parse_args()

    generate(args)


def generate(args: argparse.Namespace) -> None:
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if args.seed is not None:
        torch.manual_seed(args.seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(args.seed)

    wav_path = resolve_wav(args)
    textgrid_path = resolve_textgrid(args)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(args.stats, "r") as f:
        stats = json.load(f)

    checkpoint = load_checkpoint(Path(args.checkpoint))
    model_config = checkpoint["model_config"]
    autoencoder_config = dict(checkpoint["autoencoder_config"])
    autoencoder_config["target_shape"] = tuple(autoencoder_config["target_shape"])
    dataset_metadata = checkpoint.get("dataset_metadata") or {}
    target_mode = validate_latent_contract(model_config, autoencoder_config, stats, dataset_metadata)

    print(f"[LOAD] WAV:        {wav_path}")
    print(f"[LOAD] TextGrid:   {textgrid_path if textgrid_path else 'NONE'}")
    print(f"[LOAD] Checkpoint: {args.checkpoint}")
    print(f"[LOAD] Device:     {device}")

    words = load_words_from_textgrid(textgrid_path)
    features = build_features(
        wav_path,
        words,
        model_config,
        stats,
        args.fps,
        device,
        args.text_cpu,
        args.wavlm_cpu,
        dataset_metadata=dataset_metadata,
    )

    window_frames = int(round(args.window_size * args.fps))
    stride_frames = int(round(args.stride * args.fps))
    if window_frames <= 0 or stride_frames <= 0:
        raise ValueError("--window-size and --stride must produce at least one frame")
    if args.sample_steps <= 0:
        raise ValueError("--sample-steps must be positive")
    if args.guidance_scale < 0:
        raise ValueError("--guidance-scale cannot be negative")
    if args.smooth_window < 1:
        raise ValueError("--smooth-window must be at least 1")
    if args.velocity_scale <= 0:
        raise ValueError("--velocity-scale must be positive")
    if not 0.0 <= args.rest_blend <= 1.0:
        raise ValueError("--rest-blend must be in [0, 1]")

    model = LatentDenoiser(**latent_model_kwargs(model_config)).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    autoencoder = MotionAutoencoder(**autoencoder_config).to(device)
    autoencoder.load_state_dict(checkpoint["autoencoder_state_dict"])
    schedule = DiffusionSchedule(**checkpoint["diffusion_config"]).to(device)
    speaker = build_speaker_condition(args, model_config, dataset_metadata, stats, device)

    seed_frames = args.diffusion_seed_frames
    if seed_frames is None:
        seed_frames = int(model_config.get("seed_frames", 0) or 0)
    if seed_frames < 0:
        raise ValueError("--diffusion-seed-frames cannot be negative")
    if seed_frames >= window_frames:
        raise ValueError("--diffusion-seed-frames must be smaller than the inference window")

    pred_norm = run_latent_diffusion_inference(
        model,
        autoencoder,
        schedule,
        features,
        window_frames,
        stride_frames,
        device,
        deterministic=args.diffusion_deterministic,
        seed_frames=seed_frames,
        speaker=speaker,
        sampler=args.sampler,
        sample_steps=args.sample_steps,
        guidance_scale=args.guidance_scale,
    )
    energy_labels = energy_labels_for_features(features, dataset_metadata, stats)
    pred_norm = apply_energy_rest_gate(
        pred_norm,
        energy_labels,
        enabled=args.energy_gate,
        rest_blend=args.rest_blend,
    )
    pred_angles = reconstruct_nao_angles(pred_norm, stats, target_mode).astype(np.float32)
    pred_angles = smooth_nao_angles(pred_angles, args.smooth_window).astype(np.float32)
    pred_angles = clamp_nao_angles(pred_angles).astype(np.float32)
    if args.velocity_limit:
        pred_angles = limit_nao_velocity(pred_angles, args.fps, scale=args.velocity_scale).astype(np.float32)
        pred_angles = clamp_nao_angles(pred_angles).astype(np.float32)

    np.save(output_path, pred_angles)
    print(f"[OUT] Saved:       {output_path}")
    print(f"[OUT] Shape:       {pred_angles.shape}")
    print(f"[OUT] Joints:      {NAO_JOINTS}")
    save_metadata(
        output_path,
        {
            "wav": str(wav_path),
            "textgrid": str(textgrid_path) if textgrid_path else None,
            "checkpoint": str(args.checkpoint),
            "stats": str(args.stats),
            "fps": args.fps,
            "window_size": args.window_size,
            "stride": args.stride,
            "model_type": "latent_diffusion",
            "seed": args.seed,
            "diffusion_deterministic": args.diffusion_deterministic,
            "sampler": args.sampler,
            "sample_steps": args.sample_steps,
            "guidance_scale": args.guidance_scale,
            "smooth_window": args.smooth_window,
            "velocity_limit": args.velocity_limit,
            "velocity_scale": args.velocity_scale,
            "energy_gate": args.energy_gate,
            "rest_blend": args.rest_blend,
            "energy_label_counts": (
                np.bincount(energy_labels, minlength=3).astype(int).tolist()
                if energy_labels is not None
                else None
            ),
            "speaker_id": args.speaker_id or (speaker_id_from_clip_id(args.clip_id) if args.clip_id else None),
            "diffusion_seed_frames": seed_frames,
            "num_frames": int(pred_angles.shape[0]),
            "duration_sec": float(pred_angles.shape[0] / args.fps),
            "nao_joint_names": NAO_JOINTS,
            "target_mode": target_mode,
            "model_config": model_config,
            "autoencoder_config": checkpoint["autoencoder_config"],
            "dataset_metadata": dataset_metadata,
            "diffusion_config": checkpoint["diffusion_config"],
        },
    )


if __name__ == "__main__":
    main()
