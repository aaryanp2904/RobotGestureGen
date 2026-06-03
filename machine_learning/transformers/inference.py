#!/usr/bin/env python3
"""Run BEAT2/NAO inference with a GestureTransformer checkpoint."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch
import torchaudio

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from BEATArc.config import AUDIO_DIR, AUDIO_SR, MOCAP_FPS, TEXTGRID_DIR  # noqa: E402
from BEATArc.nao_constants import NAO_JOINTS, NAO_LIMITS, NAO_MAX_VEL  # noqa: E402
from BEATArc.parse_annotations import parse_textgrid  # noqa: E402
from BEATArc.preprocess_nao import (  # noqa: E402
    DEFAULT_WAVLM_MODEL,
    PROSODY_FEATURE_NAMES,
    TEXT_EMBED_DIM,
    extract_prosody,
    extract_text_features,
    extract_wavlm_features,
    init_text_models,
    init_wavlm_model,
    speaker_id_from_clip_id,
)

try:
    from .model import GestureTransformer, model_init_kwargs
except ImportError:
    from model import GestureTransformer, model_init_kwargs


def resolve_wav(args: argparse.Namespace) -> Path:
    if args.wav:
        return Path(args.wav)
    if args.clip_id:
        return Path(args.audio_dir) / f"{args.clip_id}.wav"
    raise ValueError("Provide --wav or --clip-id")


def resolve_textgrid(args: argparse.Namespace) -> Path | None:
    if args.textgrid:
        return Path(args.textgrid)
    if args.clip_id:
        candidate = Path(args.textgrid_dir) / f"{args.clip_id}.TextGrid"
        return candidate if candidate.is_file() else None
    return None


def load_words_from_textgrid(textgrid_path: Path | None) -> list[dict]:
    if textgrid_path is None or not textgrid_path.is_file():
        return []
    return parse_textgrid(textgrid_path).get("words", [])


def load_checkpoint(checkpoint_path: Path) -> dict:
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    if isinstance(checkpoint, dict) and "model_state_dict" in checkpoint:
        return checkpoint
    return {"model_state_dict": checkpoint, "model_config": {}}


def resolve_target_mode(model_config: dict, dataset_metadata: dict, stats: dict) -> str:
    modes = [
        source.get("target_mode")
        for source in (model_config, dataset_metadata, stats)
        if source.get("target_mode") is not None
    ]
    unique = set(modes)
    if len(unique) > 1:
        raise ValueError(f"Conflicting target_mode values: {sorted(unique)}")
    return modes[0] if modes else "angle"


def feature_layout(model_config: dict, dataset_metadata: dict, stats: dict) -> tuple[int, int]:
    expected_input_dim = int(model_config.get("input_dim", 0) or 0)
    if expected_input_dim <= 0:
        raise ValueError("Checkpoint model_config must include a positive input_dim")

    energy_dim = int(dataset_metadata.get("gesture_energy_dim", stats.get("gesture_energy_dim", 0)) or 0)
    if energy_dim:
        raise ValueError(
            "This transformer inference script expects checkpoints preprocessed without "
            f"gesture-energy features, but checkpoint/stats report gesture_energy_dim={energy_dim}."
        )

    metadata_has_wavlm = dataset_metadata.get("wavlm_dim") is not None
    metadata_has_text = dataset_metadata.get("text_dim") is not None
    wavlm_dim = int(dataset_metadata.get("wavlm_dim", stats.get("wavlm_dim", 0)) or 0)
    text_dim = int(dataset_metadata.get("text_dim", stats.get("text_dim", 0)) or 0)

    if not metadata_has_wavlm and not metadata_has_text:
        remaining = expected_input_dim - len(PROSODY_FEATURE_NAMES)
        stats_wavlm_dim = int(stats.get("wavlm_dim", 0) or 0)
        if remaining == 0:
            wavlm_dim, text_dim = 0, 0
        elif remaining == stats_wavlm_dim:
            wavlm_dim, text_dim = stats_wavlm_dim, 0
        elif remaining == TEXT_EMBED_DIM:
            wavlm_dim, text_dim = 0, TEXT_EMBED_DIM
        elif remaining == stats_wavlm_dim + TEXT_EMBED_DIM:
            wavlm_dim, text_dim = stats_wavlm_dim, TEXT_EMBED_DIM
        else:
            raise ValueError(
                f"Cannot infer non-energy feature layout for input_dim={expected_input_dim}. "
                "Use a checkpoint with dataset_metadata or matching stats."
            )

    if text_dim not in (0, TEXT_EMBED_DIM):
        raise ValueError(f"Unsupported text_dim={text_dim}; expected 0 or {TEXT_EMBED_DIM}")
    actual_dim = len(PROSODY_FEATURE_NAMES) + wavlm_dim + text_dim
    if actual_dim != expected_input_dim:
        raise ValueError(
            f"Feature dims do not match checkpoint input_dim={expected_input_dim}: "
            f"prosody={len(PROSODY_FEATURE_NAMES)}, wavlm={wavlm_dim}, text={text_dim}"
        )
    return wavlm_dim, text_dim


def validate_contract(model_config: dict, dataset_metadata: dict, stats: dict) -> str:
    if not model_config:
        raise ValueError(
            "Checkpoint has no model_config. Use a checkpoint saved with input_dim and target_shape."
        )
    missing = [key for key in ("input_dim", "target_shape") if key not in model_config]
    if missing:
        raise ValueError(f"Checkpoint model_config is missing required keys: {missing}")

    required_stats = ["prosody_mean", "prosody_std", "nao_mean", "nao_std"]
    target_mode = resolve_target_mode(model_config, dataset_metadata, stats)
    if target_mode == "delta":
        required_stats.extend(["nao_vel_mean", "nao_vel_std"])
    wavlm_dim, _text_dim = feature_layout(model_config, dataset_metadata, stats)
    if wavlm_dim:
        required_stats.extend(["wavlm_mean", "wavlm_std"])

    missing_stats = [key for key in required_stats if key not in stats]
    if missing_stats:
        raise ValueError(f"Stats file is missing required keys: {missing_stats}")
    if tuple(model_config["target_shape"]) != (len(NAO_JOINTS),):
        raise ValueError(
            f"Expected target_shape ({len(NAO_JOINTS)},), got {model_config['target_shape']}"
        )

    stats_feature_names = stats.get("prosody_feature_names")
    if stats_feature_names is not None and list(stats_feature_names) != list(PROSODY_FEATURE_NAMES):
        raise ValueError("Stats prosody feature order does not match inference order")
    stats_joint_names = stats.get("nao_joint_names")
    if stats_joint_names is not None and list(stats_joint_names) != list(NAO_JOINTS):
        raise ValueError("Stats NAO joint order does not match inference order")
    metadata_features = dataset_metadata.get("feature_names")
    if metadata_features is not None and list(metadata_features) != stats.get("feature_names", metadata_features):
        raise ValueError("Checkpoint feature_names and stats feature_names do not match")
    return target_mode


def build_features(
    wav_path: Path,
    words: list[dict],
    model_config: dict,
    stats: dict,
    fps: int,
    device: torch.device,
    text_cpu: bool,
    wavlm_cpu: bool,
    dataset_metadata: dict,
) -> np.ndarray:
    waveform, sr = torchaudio.load(str(wav_path))
    target_frames = max(1, int(round((waveform.shape[1] / sr) * fps)))
    prosody = extract_prosody(wav_path, target_frames, fps=fps, sample_rate=AUDIO_SR)
    prosody_mean = np.asarray(stats["prosody_mean"], dtype=np.float32)
    prosody_std = np.asarray(stats["prosody_std"], dtype=np.float32)
    prosody = (prosody - prosody_mean) / prosody_std

    wavlm_dim, text_dim = feature_layout(model_config, dataset_metadata, stats)
    parts = [prosody.astype(np.float32)]

    if wavlm_dim:
        wavlm_mean = np.asarray(stats["wavlm_mean"], dtype=np.float32)
        wavlm_std = np.asarray(stats["wavlm_std"], dtype=np.float32)
        wavlm_device = torch.device("cpu") if wavlm_cpu else device
        wavlm_model_name = dataset_metadata.get("wavlm_model") or stats.get("wavlm_model") or DEFAULT_WAVLM_MODEL
        print(f"[WAVLM] Extracting {wavlm_model_name} features on {wavlm_device}")
        processor, wavlm_model, model_wavlm_dim = init_wavlm_model(wavlm_model_name, wavlm_device)
        if model_wavlm_dim != wavlm_dim:
            raise ValueError(
                f"WavLM model produces {model_wavlm_dim} dims, checkpoint expects {wavlm_dim}"
            )
        wavlm = extract_wavlm_features(wav_path, target_frames, processor, wavlm_model, wavlm_device)
        parts.append(((wavlm - wavlm_mean) / wavlm_std).astype(np.float32))

    if text_dim:
        if not words:
            print("[WARN] Model expects text features but no TextGrid words were found; using zeros.")
            text = np.zeros((target_frames, TEXT_EMBED_DIM), dtype=np.float32)
        else:
            text_device = torch.device("cpu") if text_cpu else device
            tokenizer, text_model = init_text_models(text_device)
            text = extract_text_features(words, target_frames, tokenizer, text_model, text_device, fps=fps)
        parts.append(text.astype(np.float32))

    features = np.concatenate(parts, axis=1).astype(np.float32)
    expected_input_dim = int(model_config["input_dim"])
    if features.shape[1] != expected_input_dim:
        raise ValueError(f"Built {features.shape[1]} input dims, checkpoint expects {expected_input_dim}")
    return features


def window_starts(total_frames: int, window_frames: int, stride_frames: int) -> list[int]:
    if total_frames <= window_frames:
        return [0]
    starts = list(range(0, total_frames - window_frames + 1, stride_frames))
    last = total_frames - window_frames
    if starts[-1] != last:
        starts.append(last)
    return starts


def overlap_weights(window_frames: int) -> torch.Tensor:
    if window_frames <= 2:
        return torch.ones((window_frames,), dtype=torch.float32)
    return torch.hann_window(window_frames, periodic=False, dtype=torch.float32).clamp(min=0.05)


@torch.no_grad()
def run_transformer_inference(
    model: GestureTransformer,
    features: np.ndarray,
    window_frames: int,
    stride_frames: int,
    device: torch.device,
) -> np.ndarray:
    total_frames = features.shape[0]
    target_shape = tuple(model.target_shape)
    pred_sum = torch.zeros((total_frames, *target_shape), dtype=torch.float32)
    pred_count = torch.zeros((total_frames,) + (1,) * len(target_shape), dtype=torch.float32)
    padded_features = features
    if total_frames < window_frames:
        pad = np.repeat(features[-1:], window_frames - total_frames, axis=0)
        padded_features = np.concatenate([features, pad], axis=0)

    weights = overlap_weights(window_frames).reshape(window_frames, *((1,) * len(target_shape)))
    starts = window_starts(total_frames, window_frames, stride_frames)
    print(f"[INFER] Running {len(starts)} windows")
    model.eval()
    for start in starts:
        end = start + window_frames
        x = torch.from_numpy(padded_features[start:end]).unsqueeze(0).to(device)
        pred = model(x).squeeze(0).cpu()
        valid_end = min(end, total_frames)
        valid_len = valid_end - start
        pred_sum[start:valid_end] += pred[:valid_len] * weights[:valid_len]
        pred_count[start:valid_end] += weights[:valid_len]
    return (pred_sum / pred_count.clamp(min=1)).numpy()


def reconstruct_nao_angles(pred_norm: np.ndarray, stats: dict, target_mode: str) -> np.ndarray:
    if target_mode == "angle":
        mean = np.asarray(stats["nao_mean"], dtype=np.float32)
        std = np.asarray(stats["nao_std"], dtype=np.float32)
        return pred_norm * std + mean
    if target_mode != "delta":
        raise ValueError(f"Unsupported target_mode={target_mode}")
    vel_mean = np.asarray(stats["nao_vel_mean"], dtype=np.float32)
    vel_std = np.asarray(stats["nao_vel_std"], dtype=np.float32)
    deltas = pred_norm * vel_std + vel_mean
    if len(deltas):
        deltas[0] = 0.0
    return np.asarray(stats["nao_mean"], dtype=np.float32) + np.cumsum(deltas, axis=0)


def clamp_nao_angles(angles: np.ndarray) -> np.ndarray:
    out = angles.copy()
    for joint_idx, name in enumerate(NAO_JOINTS):
        low, high = NAO_LIMITS[name]
        out[:, joint_idx] = np.clip(out[:, joint_idx], low, high)
    return out


def smooth_nao_angles(angles: np.ndarray, window_frames: int) -> np.ndarray:
    if window_frames <= 1 or len(angles) <= 2:
        return angles
    window_frames = min(int(window_frames), len(angles))
    kernel = np.ones(window_frames, dtype=np.float32) / float(window_frames)
    pad_left = window_frames // 2
    pad_right = window_frames - 1 - pad_left
    padded = np.pad(angles, ((pad_left, pad_right), (0, 0)), mode="edge")
    smoothed = np.empty_like(angles, dtype=np.float32)
    for joint_idx in range(angles.shape[1]):
        smoothed[:, joint_idx] = np.convolve(padded[:, joint_idx], kernel, mode="valid")
    return smoothed


def limit_nao_velocity(angles: np.ndarray, fps: int, scale: float) -> np.ndarray:
    if len(angles) <= 1:
        return angles
    out = angles.copy()
    frame_time = 1.0 / fps
    for frame_idx in range(1, len(out)):
        for joint_idx, name in enumerate(NAO_JOINTS):
            max_change = NAO_MAX_VEL.get(name, 5.0) * scale * frame_time
            diff = out[frame_idx, joint_idx] - out[frame_idx - 1, joint_idx]
            if abs(diff) > max_change:
                out[frame_idx, joint_idx] = out[frame_idx - 1, joint_idx] + np.sign(diff) * max_change
    return out


def save_metadata(output_path: Path, metadata: dict) -> None:
    meta_path = output_path.with_suffix(".json")
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"[OUT] Metadata: {meta_path}")


def generate(args: argparse.Namespace) -> np.ndarray:
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
    model_config = checkpoint.get("model_config") or {}
    dataset_metadata = checkpoint.get("dataset_metadata") or {}
    target_mode = validate_contract(model_config, dataset_metadata, stats)

    print(f"[LOAD] WAV:        {wav_path}")
    print(f"[LOAD] TextGrid:   {textgrid_path if textgrid_path else 'NONE'}")
    print(f"[LOAD] Checkpoint: {args.checkpoint}")
    print(f"[LOAD] Device:     {device}")
    print(f"[MODEL] Config:    {model_config}")

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
        dataset_metadata,
    )
    window_frames = int(round(args.window_size * args.fps))
    stride_frames = int(round(args.stride * args.fps))
    if window_frames <= 0 or stride_frames <= 0:
        raise ValueError("--window-size and --stride must produce at least one frame")
    if args.smooth_window < 1:
        raise ValueError("--smooth-window must be at least 1")
    if args.velocity_scale <= 0:
        raise ValueError("--velocity-scale must be positive")

    model = GestureTransformer(**model_init_kwargs(model_config)).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    pred_norm = run_transformer_inference(model, features, window_frames, stride_frames, device)
    pred_angles = reconstruct_nao_angles(pred_norm, stats, target_mode).astype(np.float32)
    pred_angles = smooth_nao_angles(pred_angles, args.smooth_window).astype(np.float32)
    pred_angles = clamp_nao_angles(pred_angles).astype(np.float32)
    if args.velocity_limit:
        pred_angles = limit_nao_velocity(pred_angles, args.fps, args.velocity_scale).astype(np.float32)
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
            "model_type": "transformer",
            "seed": args.seed,
            "smooth_window": args.smooth_window,
            "velocity_limit": args.velocity_limit,
            "velocity_scale": args.velocity_scale,
            "speaker_id": speaker_id_from_clip_id(args.clip_id) if args.clip_id else None,
            "num_frames": int(pred_angles.shape[0]),
            "duration_sec": float(pred_angles.shape[0] / args.fps),
            "nao_joint_names": NAO_JOINTS,
            "target_mode": target_mode,
            "model_config": model_config,
            "dataset_metadata": dataset_metadata,
            "feature_layout": {
                "prosody_dim": len(PROSODY_FEATURE_NAMES),
                "wavlm_dim": feature_layout(model_config, dataset_metadata, stats)[0],
                "text_dim": feature_layout(model_config, dataset_metadata, stats)[1],
                "gesture_energy_dim": 0,
            },
        },
    )
    return pred_angles


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate BEAT2 NAO gestures with GestureTransformer")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--stats", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--wav", default=None)
    parser.add_argument("--textgrid", default=None)
    parser.add_argument("--clip-id", default=None)
    parser.add_argument("--audio-dir", default=str(AUDIO_DIR))
    parser.add_argument("--textgrid-dir", default=str(TEXTGRID_DIR))
    parser.add_argument("--fps", type=int, default=MOCAP_FPS)
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--smooth-window", type=int, default=1)
    parser.add_argument("--velocity-limit", action="store_true")
    parser.add_argument("--velocity-scale", type=float, default=1.0)
    parser.add_argument("--text-cpu", action="store_true")
    parser.add_argument("--wavlm-cpu", action="store_true")
    args = parser.parse_args()

    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    if args.window_size <= 0 or args.stride <= 0:
        raise ValueError("--window-size and --stride must be positive")
    generate(args)


if __name__ == "__main__":
    main()
