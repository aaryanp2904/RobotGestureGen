#!/usr/bin/env python3
"""
Run inference with a BEAT2-trained NAO gesture model.

Input:
  - WAV audio
  - optional TextGrid words, if the model was trained with text embeddings

Output:
  - .npy array shaped (frames, 10), containing denormalized NAO joint angles
    in nao_constants.NAO_JOINTS order.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch
import torchaudio

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "machine_learning" / "transformers"))

from pos_end import GestureTransformer
from machine_learning.diffusion import ConditionalMotionDenoiser, DiffusionSchedule

MODEL_INIT_KEYS = {"input_dim", "hidden_dim", "num_heads", "num_layers", "num_joints", "target_shape"}
DIFFUSION_MODEL_INIT_KEYS = {
    "input_dim",
    "target_shape",
    "hidden_dim",
    "num_heads",
    "num_layers",
    "dropout",
    "max_frames",
}


def model_init_kwargs(model_config: dict) -> dict:
    return {key: value for key, value in model_config.items() if key in MODEL_INIT_KEYS}


def diffusion_model_init_kwargs(model_config: dict) -> dict:
    return {key: value for key, value in model_config.items() if key in DIFFUSION_MODEL_INIT_KEYS}


try:
    from .config import AUDIO_DIR, AUDIO_SR, MOCAP_FPS, TEXTGRID_DIR
    from .nao_constants import NAO_JOINTS, NAO_LIMITS
    from .parse_annotations import parse_textgrid
    from .preprocess_nao import (
        PROSODY_FEATURE_NAMES,
        TEXT_EMBED_DIM,
        extract_prosody,
        extract_text_features,
        init_text_models,
    )
except ImportError:
    from config import AUDIO_DIR, AUDIO_SR, MOCAP_FPS, TEXTGRID_DIR
    from nao_constants import NAO_JOINTS, NAO_LIMITS
    from parse_annotations import parse_textgrid
    from preprocess_nao import (
        PROSODY_FEATURE_NAMES,
        TEXT_EMBED_DIM,
        extract_prosody,
        extract_text_features,
        init_text_models,
    )


def resolve_wav(args) -> Path:
    if args.wav:
        return Path(args.wav)
    if args.clip_id:
        return Path(args.audio_dir) / f"{args.clip_id}.wav"
    raise ValueError("Provide --wav or --clip-id")


def resolve_textgrid(args) -> Path | None:
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


def load_checkpoint(checkpoint_path: Path, device: torch.device):
    checkpoint = torch.load(checkpoint_path, map_location=device)
    if isinstance(checkpoint, dict) and "model_state_dict" in checkpoint:
        return checkpoint
    return {"model_state_dict": checkpoint, "model_config": {}}


def resolve_target_mode(model_config: dict, dataset_metadata: dict, stats: dict) -> str:
    modes = [
        (name, source.get("target_mode"))
        for name, source in (
            ("checkpoint model_config", model_config),
            ("checkpoint dataset_metadata", dataset_metadata),
            ("stats", stats),
        )
        if source.get("target_mode") is not None
    ]
    unique_modes = {mode for _, mode in modes}
    if len(unique_modes) > 1:
        details = ", ".join(f"{name}={mode}" for name, mode in modes)
        raise ValueError(f"Conflicting target_mode values: {details}")
    return modes[0][1] if modes else "angle"


def validate_inference_contract(model_config: dict, stats: dict, dataset_metadata: dict):
    if not model_config:
        raise ValueError(
            "Checkpoint has no model_config. Use gesture_transformer_best.pth, "
            "gesture_transformer_latest.pth, or a newly saved full_trained checkpoint."
        )
    missing_config = [key for key in ("input_dim", "target_shape") if key not in model_config]
    if missing_config:
        raise ValueError(f"Checkpoint model_config is missing required keys: {missing_config}")

    required_stats = ["prosody_mean", "prosody_std", "nao_mean", "nao_std"]
    target_mode = resolve_target_mode(model_config, dataset_metadata, stats)
    if target_mode == "delta":
        required_stats.extend(["nao_vel_mean", "nao_vel_std"])

    missing = [key for key in required_stats if key not in stats]
    if missing:
        raise ValueError(f"Stats file is missing required keys: {missing}")

    if tuple(model_config.get("target_shape", ())) != (len(NAO_JOINTS),):
        raise ValueError(
            f"Expected NAO target_shape ({len(NAO_JOINTS)},), "
            f"got {model_config.get('target_shape')}"
        )
    if dataset_metadata:
        metadata_shape = tuple(dataset_metadata.get("target_shape", ()))
        if metadata_shape and metadata_shape != tuple(model_config["target_shape"]):
            raise ValueError(
                f"Checkpoint model_config target_shape {model_config['target_shape']} "
                f"does not match dataset_metadata target_shape {metadata_shape}"
            )
        metadata_input_dim = dataset_metadata.get("input_dim")
        if metadata_input_dim is not None and int(metadata_input_dim) != int(model_config["input_dim"]):
            raise ValueError(
                f"Checkpoint model_config input_dim {model_config['input_dim']} "
                f"does not match dataset_metadata input_dim {metadata_input_dim}"
            )
    return target_mode


def build_features(wav_path: Path, words: list[dict], model_config: dict,
                   stats: dict, fps: int, device: torch.device,
                   text_cpu: bool) -> np.ndarray:
    waveform, sr = torchaudio.load(str(wav_path))
    duration_sec = waveform.shape[1] / sr
    target_frames = max(1, int(round(duration_sec * fps)))

    prosody = extract_prosody(wav_path, target_frames, fps=fps, sample_rate=AUDIO_SR)
    prosody_mean = np.array(stats["prosody_mean"], dtype=np.float32)
    prosody_std = np.array(stats["prosody_std"], dtype=np.float32)
    prosody = (prosody - prosody_mean) / prosody_std

    expected_input_dim = int(model_config.get("input_dim", prosody.shape[1]))
    text_dim = expected_input_dim - len(PROSODY_FEATURE_NAMES)
    if text_dim == 0:
        return prosody.astype(np.float32)
    if text_dim != TEXT_EMBED_DIM:
        raise ValueError(
            f"Unsupported text feature dimension {text_dim}; expected 0 or {TEXT_EMBED_DIM}"
        )

    if not words:
        print("[WARN] Model expects text features but no TextGrid words were found; using zeros.")
        text_features = np.zeros((target_frames, TEXT_EMBED_DIM), dtype=np.float32)
    else:
        text_device = torch.device("cpu") if text_cpu else device
        tokenizer, text_model = init_text_models(text_device)
        text_features = extract_text_features(
            words, target_frames, tokenizer, text_model, text_device, fps=fps
        )

    return np.concatenate([prosody, text_features], axis=1).astype(np.float32)


def window_starts(total_frames: int, window_frames: int, stride_frames: int) -> list[int]:
    if total_frames <= window_frames:
        return [0]
    starts = list(range(0, total_frames - window_frames + 1, stride_frames))
    last = total_frames - window_frames
    if starts[-1] != last:
        starts.append(last)
    return starts


def run_inference(model, features: np.ndarray, window_frames: int,
                  stride_frames: int, device: torch.device) -> np.ndarray:
    total_frames = features.shape[0]
    target_shape = tuple(model.target_shape)
    pred_sum = torch.zeros((total_frames, *target_shape), dtype=torch.float32)
    pred_count = torch.zeros((total_frames,) + (1,) * len(target_shape), dtype=torch.float32)

    padded_features = features
    if total_frames < window_frames:
        pad = np.repeat(features[-1:], window_frames - total_frames, axis=0)
        padded_features = np.concatenate([features, pad], axis=0)

    model.eval()
    with torch.no_grad():
        for start in window_starts(total_frames, window_frames, stride_frames):
            end = start + window_frames
            x_np = padded_features[start:end]
            x = torch.from_numpy(x_np).unsqueeze(0).to(device)
            pred = model(x).squeeze(0).cpu()
            valid_end = min(end, total_frames)
            valid_len = valid_end - start
            pred_sum[start:valid_end] += pred[:valid_len]
            pred_count[start:valid_end] += 1

    return (pred_sum / pred_count.clamp(min=1)).numpy()


def run_diffusion_inference(model, schedule, features: np.ndarray, window_frames: int,
                            stride_frames: int, device: torch.device) -> np.ndarray:
    total_frames = features.shape[0]
    target_shape = tuple(model.target_shape)
    pred_sum = torch.zeros((total_frames, *target_shape), dtype=torch.float32)
    pred_count = torch.zeros((total_frames,) + (1,) * len(target_shape), dtype=torch.float32)

    padded_features = features
    if total_frames < window_frames:
        pad = np.repeat(features[-1:], window_frames - total_frames, axis=0)
        padded_features = np.concatenate([features, pad], axis=0)

    starts = window_starts(total_frames, window_frames, stride_frames)
    print(f"[DIFFUSION] Sampling {len(starts)} windows x {schedule.timesteps} steps")
    model.eval()
    with torch.no_grad():
        for window_idx, start in enumerate(starts, start=1):
            end = start + window_frames
            x_np = padded_features[start:end]
            conditioning = torch.from_numpy(x_np).unsqueeze(0).to(device)
            pred = schedule.sample(model, conditioning, target_shape).squeeze(0).cpu()
            valid_end = min(end, total_frames)
            valid_len = valid_end - start
            pred_sum[start:valid_end] += pred[:valid_len]
            pred_count[start:valid_end] += 1
            if window_idx == 1 or window_idx == len(starts) or window_idx % 10 == 0:
                print(f"[DIFFUSION] Window {window_idx}/{len(starts)}")

    return (pred_sum / pred_count.clamp(min=1)).numpy()


def denormalize_nao(pred_norm: np.ndarray, stats: dict) -> np.ndarray:
    mean = np.array(stats["nao_mean"], dtype=np.float32)
    std = np.array(stats["nao_std"], dtype=np.float32)
    return pred_norm * std + mean


def clamp_nao_angles(angles: np.ndarray) -> np.ndarray:
    out = angles.copy()
    for joint_idx, name in enumerate(NAO_JOINTS):
        low, high = NAO_LIMITS[name]
        out[:, joint_idx] = np.clip(out[:, joint_idx], low, high)
    return out


def reconstruct_nao_angles(pred_norm: np.ndarray, stats: dict, target_mode: str) -> np.ndarray:
    if target_mode == "angle":
        return denormalize_nao(pred_norm, stats)
    if target_mode != "delta":
        raise ValueError(f"Unsupported checkpoint target_mode: {target_mode}")

    vel_mean = np.array(stats["nao_vel_mean"], dtype=np.float32)
    vel_std = np.array(stats["nao_vel_std"], dtype=np.float32)
    deltas = pred_norm * vel_std + vel_mean
    if len(deltas) > 0:
        deltas[0] = 0.0
    initial_pose = np.array(stats["nao_mean"], dtype=np.float32)
    return initial_pose + np.cumsum(deltas, axis=0)


def save_metadata(output_path: Path, metadata: dict):
    meta_path = output_path.with_suffix(".json")
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"[OUT] Metadata: {meta_path}")


def main():
    parser = argparse.ArgumentParser(description="Generate NAO joint-angle gestures from WAV/TextGrid")
    parser.add_argument("--checkpoint", type=str, required=True,
                        help="Path to transformer or diffusion checkpoint")
    parser.add_argument("--stats", type=str, required=True,
                        help="Path to BEAT2_NAO_Preprocessed/normalization_stats.json")
    parser.add_argument("--output", type=str, required=True,
                        help="Output .npy path for predicted NAO angles")
    parser.add_argument("--model-type", choices=["auto", "transformer", "diffusion"], default="auto",
                        help="Inference model family for the checkpoint")
    parser.add_argument("--wav", type=str, default=None,
                        help="Input WAV path")
    parser.add_argument("--textgrid", type=str, default=None,
                        help="Optional TextGrid path")
    parser.add_argument("--clip-id", type=str, default=None,
                        help="Resolve WAV/TextGrid from BEAT2 directories by clip ID")
    parser.add_argument("--audio-dir", type=str, default=str(AUDIO_DIR))
    parser.add_argument("--textgrid-dir", type=str, default=str(TEXTGRID_DIR))
    parser.add_argument("--fps", type=int, default=MOCAP_FPS)
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=None,
                        help="Optional random seed for diffusion sampling")
    parser.add_argument("--text-cpu", action="store_true",
                        help="Run DistilBERT text embedding on CPU")
    args = parser.parse_args()

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

    checkpoint = load_checkpoint(Path(args.checkpoint), device)
    state_dict = checkpoint["model_state_dict"]
    model_config = checkpoint.get("model_config") or {}
    dataset_metadata = checkpoint.get("dataset_metadata") or {}
    target_mode = validate_inference_contract(model_config, stats, dataset_metadata)

    print(f"[LOAD] WAV:        {wav_path}")
    print(f"[LOAD] TextGrid:   {textgrid_path if textgrid_path else 'NONE'}")
    print(f"[LOAD] Checkpoint: {args.checkpoint}")
    print(f"[LOAD] Device:     {device}")
    model_type = args.model_type
    if model_type == "auto":
        model_type = "diffusion" if "diffusion_config" in checkpoint else "transformer"

    print(f"[MODEL] Type:      {model_type}")
    print(f"[MODEL] Config:    {model_config}")

    words = load_words_from_textgrid(textgrid_path)
    features = build_features(
        wav_path, words, model_config, stats, args.fps, device, args.text_cpu
    )

    window_frames = int(round(args.window_size * args.fps))
    stride_frames = int(round(args.stride * args.fps))
    if window_frames <= 0 or stride_frames <= 0:
        raise ValueError("--window-size and --stride must produce at least one frame")

    if model_type == "diffusion":
        diffusion_config = checkpoint.get("diffusion_config")
        if not diffusion_config:
            raise ValueError("Diffusion checkpoint is missing diffusion_config")
        model = ConditionalMotionDenoiser(**diffusion_model_init_kwargs(model_config)).to(device)
        model.load_state_dict(state_dict)
        schedule = DiffusionSchedule(**diffusion_config).to(device)
        pred_norm = run_diffusion_inference(
            model, schedule, features, window_frames, stride_frames, device
        )
    else:
        model = GestureTransformer(**model_init_kwargs(model_config)).to(device)
        model.load_state_dict(state_dict)
        pred_norm = run_inference(model, features, window_frames, stride_frames, device)

    pred_angles = reconstruct_nao_angles(pred_norm, stats, target_mode).astype(np.float32)
    pred_angles = clamp_nao_angles(pred_angles).astype(np.float32)

    np.save(output_path, pred_angles)
    print(f"[OUT] Saved:       {output_path}")
    print(f"[OUT] Shape:       {pred_angles.shape}")
    print(f"[OUT] Joints:      {NAO_JOINTS}")

    save_metadata(output_path, {
        "wav": str(wav_path),
        "textgrid": str(textgrid_path) if textgrid_path else None,
        "checkpoint": str(args.checkpoint),
        "stats": str(args.stats),
        "fps": args.fps,
        "window_size": args.window_size,
        "stride": args.stride,
        "model_type": model_type,
        "seed": args.seed,
        "num_frames": int(pred_angles.shape[0]),
        "duration_sec": float(pred_angles.shape[0] / args.fps),
        "nao_joint_names": NAO_JOINTS,
        "target_mode": target_mode,
        "model_config": model_config,
        "dataset_metadata": dataset_metadata,
        "diffusion_config": checkpoint.get("diffusion_config") if model_type == "diffusion" else None,
    })


if __name__ == "__main__":
    main()
