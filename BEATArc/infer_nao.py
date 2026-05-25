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
    "speaker_dim",
    "seed_conditioning",
    "seed_frames",
    "conditioning_layers",
    "conditioning_encoder",
    "cross_attention",
    "cond_drop_prob",
    "target_mode",
    "target_representation",
}


def model_init_kwargs(model_config: dict) -> dict:
    return {key: value for key, value in model_config.items() if key in MODEL_INIT_KEYS}


def diffusion_model_init_kwargs(model_config: dict) -> dict:
    return {key: value for key, value in model_config.items() if key in DIFFUSION_MODEL_INIT_KEYS}


try:
    from .config import AUDIO_DIR, AUDIO_SR, MOCAP_FPS, TEXTGRID_DIR
    from .nao_constants import NAO_JOINTS, NAO_LIMITS, NAO_MAX_VEL
    from .parse_annotations import parse_textgrid
    from .preprocess_nao import (
        PROSODY_FEATURE_NAMES,
        TEXT_EMBED_DIM,
        DEFAULT_WAVLM_MODEL,
        extract_prosody,
        extract_text_features,
        extract_wavlm_features,
        init_text_models,
        init_wavlm_model,
        speaker_id_from_clip_id,
    )
except ImportError:
    from config import AUDIO_DIR, AUDIO_SR, MOCAP_FPS, TEXTGRID_DIR
    from nao_constants import NAO_JOINTS, NAO_LIMITS, NAO_MAX_VEL
    from parse_annotations import parse_textgrid
    from preprocess_nao import (
        PROSODY_FEATURE_NAMES,
        TEXT_EMBED_DIM,
        DEFAULT_WAVLM_MODEL,
        extract_prosody,
        extract_text_features,
        extract_wavlm_features,
        init_text_models,
        init_wavlm_model,
        speaker_id_from_clip_id,
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
    target_representation = (
        model_config.get("target_representation")
        or dataset_metadata.get("target_representation")
        or stats.get("target_representation")
        or "nao_angles"
    )
    if target_representation != "nao_angles":
        raise ValueError(
            "This NAO inference path only supports target_representation='nao_angles', "
            f"got {target_representation!r}"
        )
    if target_mode == "delta":
        required_stats.extend(["nao_vel_mean", "nao_vel_std"])
    wavlm_dim = int(dataset_metadata.get("wavlm_dim", stats.get("wavlm_dim", 0)) or 0)
    if wavlm_dim > 0:
        required_stats.extend(["wavlm_mean", "wavlm_std"])

    missing = [key for key in required_stats if key not in stats]
    if missing:
        raise ValueError(f"Stats file is missing required keys: {missing}")

    stats_joint_names = stats.get("nao_joint_names")
    if stats_joint_names is not None and list(stats_joint_names) != list(NAO_JOINTS):
        raise ValueError(
            f"Stats NAO joint order does not match inference order: {stats_joint_names}"
        )
    stats_feature_names = stats.get("prosody_feature_names")
    if stats_feature_names is not None and list(stats_feature_names) != list(PROSODY_FEATURE_NAMES):
        raise ValueError(
            f"Stats prosody feature order does not match inference order: {stats_feature_names}"
        )

    if tuple(model_config.get("target_shape", ())) != (len(NAO_JOINTS),):
        raise ValueError(
            f"Expected NAO target_shape ({len(NAO_JOINTS)},), "
            f"got {model_config.get('target_shape')}"
        )
    if dataset_metadata:
        metadata_targets = dataset_metadata.get("target_names")
        if metadata_targets is not None and list(metadata_targets) != list(NAO_JOINTS):
            raise ValueError(
                f"Checkpoint target_names do not match inference order: {metadata_targets}"
            )
        metadata_features = dataset_metadata.get("feature_names")
        if metadata_features is not None and list(metadata_features)[:len(PROSODY_FEATURE_NAMES)] != list(PROSODY_FEATURE_NAMES):
            raise ValueError(
                f"Checkpoint prosody feature order does not match inference order: {metadata_features}"
            )
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
                   text_cpu: bool, wavlm_cpu: bool,
                   dataset_metadata: dict | None = None) -> np.ndarray:
    dataset_metadata = dataset_metadata or {}
    waveform, sr = torchaudio.load(str(wav_path))
    duration_sec = waveform.shape[1] / sr
    target_frames = max(1, int(round(duration_sec * fps)))

    prosody = extract_prosody(wav_path, target_frames, fps=fps, sample_rate=AUDIO_SR)
    prosody_mean = np.array(stats["prosody_mean"], dtype=np.float32)
    prosody_std = np.array(stats["prosody_std"], dtype=np.float32)
    if prosody_mean.shape != (len(PROSODY_FEATURE_NAMES),) or prosody_std.shape != (
        len(PROSODY_FEATURE_NAMES),
    ):
        raise ValueError(
            "Stats prosody mean/std shape does not match expected prosody feature count "
            f"({len(PROSODY_FEATURE_NAMES)})"
        )
    prosody = (prosody - prosody_mean) / prosody_std

    expected_input_dim = int(model_config.get("input_dim", prosody.shape[1]))
    wavlm_dim = int(dataset_metadata.get("wavlm_dim", stats.get("wavlm_dim", 0)) or 0)
    text_dim = dataset_metadata.get("text_dim")
    if text_dim is None:
        text_dim = expected_input_dim - len(PROSODY_FEATURE_NAMES) - wavlm_dim
    text_dim = int(text_dim)
    if wavlm_dim < 0 or text_dim < 0:
        raise ValueError(
            f"Checkpoint input_dim {expected_input_dim} is inconsistent with "
            f"prosody={len(PROSODY_FEATURE_NAMES)}, wavlm={wavlm_dim}, text={text_dim}"
        )
    expected_parts_dim = len(PROSODY_FEATURE_NAMES) + wavlm_dim + text_dim
    if expected_parts_dim != expected_input_dim:
        raise ValueError(
            f"Checkpoint input_dim {expected_input_dim} does not equal "
            f"prosody+wavlm+text dims ({expected_parts_dim})"
        )
    feature_parts = [prosody.astype(np.float32)]

    if wavlm_dim > 0:
        wavlm_mean = np.array(stats["wavlm_mean"], dtype=np.float32)
        wavlm_std = np.array(stats["wavlm_std"], dtype=np.float32)
        if wavlm_mean.shape != (wavlm_dim,) or wavlm_std.shape != (wavlm_dim,):
            raise ValueError(f"Stats WavLM mean/std shape does not match wavlm_dim={wavlm_dim}")
        wavlm_device = torch.device("cpu") if wavlm_cpu else device
        wavlm_model_name = (
            dataset_metadata.get("wavlm_model")
            or stats.get("wavlm_model")
            or DEFAULT_WAVLM_MODEL
        )
        print(f"[WAVLM] Extracting {wavlm_model_name} features on {wavlm_device}")
        wavlm_processor, wavlm_model, model_wavlm_dim = init_wavlm_model(wavlm_model_name, wavlm_device)
        if model_wavlm_dim != wavlm_dim:
            raise ValueError(
                f"WavLM model {wavlm_model_name} produces {model_wavlm_dim} dims, "
                f"but checkpoint expects {wavlm_dim}"
            )
        wavlm_features = extract_wavlm_features(
            wav_path, target_frames, wavlm_processor, wavlm_model, wavlm_device
        )
        feature_parts.append(((wavlm_features - wavlm_mean) / wavlm_std).astype(np.float32))

    if text_dim == 0:
        features = np.concatenate(feature_parts, axis=1).astype(np.float32)
        if features.shape[1] != expected_input_dim:
            raise ValueError(f"Built {features.shape[1]} input dims, checkpoint expects {expected_input_dim}")
        return features
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

    feature_parts.append(text_features.astype(np.float32))
    features = np.concatenate(feature_parts, axis=1).astype(np.float32)
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
    weights = torch.hann_window(window_frames, periodic=False, dtype=torch.float32)
    return weights.clamp(min=0.05)


def build_speaker_condition(args, model_config: dict, dataset_metadata: dict, stats: dict,
                            device: torch.device) -> torch.Tensor | None:
    speaker_dim = int(model_config.get("speaker_dim", dataset_metadata.get("speaker_dim", 0)) or 0)
    if speaker_dim <= 0:
        return None
    speaker_id_map = dataset_metadata.get("speaker_id_map") or stats.get("speaker_id_map") or {}
    speaker_id = args.speaker_id
    if speaker_id is None and args.clip_id:
        speaker_id = speaker_id_from_clip_id(args.clip_id)
    speaker = np.zeros((speaker_dim,), dtype=np.float32)
    if speaker_id is None:
        print("[WARN] Model expects speaker conditioning but no --speaker-id/--clip-id was provided; using zeros.")
    else:
        idx = speaker_id_map.get(str(speaker_id))
        if idx is None:
            print(f"[WARN] Speaker id {speaker_id} not in checkpoint speaker map; using zeros.")
        elif int(idx) >= speaker_dim:
            raise ValueError(f"Speaker id {speaker_id} maps to {idx}, outside speaker_dim={speaker_dim}")
        else:
            speaker[int(idx)] = 1.0
    return torch.from_numpy(speaker).unsqueeze(0).to(device)


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

    weights = overlap_weights(window_frames).reshape(window_frames, *((1,) * len(target_shape)))
    model.eval()
    with torch.no_grad():
        for start in window_starts(total_frames, window_frames, stride_frames):
            end = start + window_frames
            x_np = padded_features[start:end]
            x = torch.from_numpy(x_np).unsqueeze(0).to(device)
            pred = model(x).squeeze(0).cpu()
            valid_end = min(end, total_frames)
            valid_len = valid_end - start
            pred_sum[start:valid_end] += pred[:valid_len] * weights[:valid_len]
            pred_count[start:valid_end] += weights[:valid_len]

    return (pred_sum / pred_count.clamp(min=1)).numpy()


def run_diffusion_inference(model, schedule, features: np.ndarray, window_frames: int,
                            stride_frames: int, device: torch.device,
                            deterministic: bool = False,
                            seed_frames: int = 0,
                            speaker: torch.Tensor | None = None,
                            sampler: str = "ddpm",
                            sample_steps: int | None = None,
                            guidance_scale: float = 1.0) -> np.ndarray:
    total_frames = features.shape[0]
    target_shape = tuple(model.target_shape)
    pred_sum = torch.zeros((total_frames, *target_shape), dtype=torch.float32)
    pred_count = torch.zeros((total_frames,) + (1,) * len(target_shape), dtype=torch.float32)

    padded_features = features
    if total_frames < window_frames:
        pad = np.repeat(features[-1:], window_frames - total_frames, axis=0)
        padded_features = np.concatenate([features, pad], axis=0)

    starts = window_starts(total_frames, window_frames, stride_frames)
    weights = overlap_weights(window_frames).reshape(window_frames, *((1,) * len(target_shape)))
    active_steps = (
        min(sample_steps, schedule.timesteps)
        if sampler == "ddim" and sample_steps
        else schedule.timesteps
    )
    print(f"[DIFFUSION] Sampling {len(starts)} windows x {active_steps} {sampler.upper()} steps")
    model.eval()
    with torch.no_grad():
        for window_idx, start in enumerate(starts, start=1):
            end = start + window_frames
            x_np = padded_features[start:end]
            conditioning = torch.from_numpy(x_np).unsqueeze(0).to(device)
            seed_motion = None
            seed_mask = None
            active_seed = min(seed_frames, window_frames)
            if getattr(model, "seed_conditioning", False) and active_seed > 0:
                seed_motion = torch.zeros((1, window_frames, *target_shape), dtype=torch.float32)
                seed_mask = torch.zeros((1, window_frames, 1), dtype=torch.float32)
                if start > 0:
                    seed_end = min(start + active_seed, total_frames)
                    valid_seed = seed_end - start
                    if valid_seed > 0:
                        known_count = pred_count[start:seed_end]
                        known_mask = (known_count > 0).reshape(valid_seed)
                        if known_mask.any():
                            known = pred_sum[start:seed_end] / known_count.clamp(min=1)
                            known_idx = torch.nonzero(known_mask, as_tuple=False).flatten()
                            seed_motion[0, known_idx] = known[known_idx]
                            seed_mask[0, known_idx, 0] = 1.0
                seed_motion = seed_motion.to(device)
                seed_mask = seed_mask.to(device)
            pred = schedule.sample(
                model,
                conditioning,
                target_shape,
                seed_motion=seed_motion,
                seed_mask=seed_mask,
                speaker=speaker,
                add_noise=not deterministic,
                sampler=sampler,
                sample_steps=sample_steps,
                guidance_scale=guidance_scale,
            ).squeeze(0).cpu()
            if seed_motion is not None and start > 0 and seed_mask is not None:
                mask = seed_mask.squeeze(0).cpu().bool().squeeze(-1)
                pred[mask] = seed_motion.squeeze(0).cpu()[mask]
            valid_end = min(end, total_frames)
            valid_len = valid_end - start
            pred_sum[start:valid_end] += pred[:valid_len] * weights[:valid_len]
            pred_count[start:valid_end] += weights[:valid_len]
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


def smooth_nao_angles(angles: np.ndarray, window_frames: int) -> np.ndarray:
    if window_frames <= 1 or len(angles) <= 2:
        return angles
    window_frames = min(int(window_frames), len(angles))
    if window_frames <= 1:
        return angles

    kernel = np.ones(window_frames, dtype=np.float32) / float(window_frames)
    pad_left = window_frames // 2
    pad_right = window_frames - 1 - pad_left
    padded = np.pad(angles, ((pad_left, pad_right), (0, 0)), mode="edge")
    smoothed = np.empty_like(angles, dtype=np.float32)
    for joint_idx in range(angles.shape[1]):
        smoothed[:, joint_idx] = np.convolve(padded[:, joint_idx], kernel, mode="valid")
    return smoothed


def limit_nao_velocity(angles: np.ndarray, fps: int, scale: float = 1.0) -> np.ndarray:
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
    parser.add_argument("--speaker-id", type=str, default=None,
                        help="Speaker id for diffusion speaker conditioning; defaults to --clip-id prefix")
    parser.add_argument("--audio-dir", type=str, default=str(AUDIO_DIR))
    parser.add_argument("--textgrid-dir", type=str, default=str(TEXTGRID_DIR))
    parser.add_argument("--fps", type=int, default=MOCAP_FPS)
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=None,
                        help="Optional random seed for diffusion sampling")
    parser.add_argument("--diffusion-deterministic", action="store_true",
                        help="Use posterior means during diffusion sampling instead of adding reverse noise")
    parser.add_argument("--sampler", choices=["ddpm", "ddim"], default="ddpm",
                        help="Diffusion sampler used for generation")
    parser.add_argument("--sample-steps", type=int, default=50,
                        help="Number of DDIM sampling steps; DDPM always uses checkpoint diffusion steps")
    parser.add_argument("--guidance-scale", type=float, default=1.0,
                        help="Classifier-free guidance scale for diffusion sampling")
    parser.add_argument("--smooth-window", type=int, default=1,
                        help="Moving-average smoothing window in frames after denormalization; 1 disables")
    parser.add_argument("--velocity-limit", action="store_true",
                        help="Clamp frame-to-frame output changes to NAO joint velocity limits")
    parser.add_argument("--velocity-scale", type=float, default=1.0,
                        help="Scale applied to NAO velocity limits when --velocity-limit is used")
    parser.add_argument("--text-cpu", action="store_true",
                        help="Run DistilBERT text embedding on CPU")
    parser.add_argument("--wavlm-cpu", action="store_true",
                        help="Run WavLM feature extraction on CPU")
    parser.add_argument("--diffusion-seed-frames", type=int, default=None,
                        help="Leading generated frames reused as seed context for each diffusion window")
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
    if args.smooth_window < 1:
        raise ValueError("--smooth-window must be at least 1")
    if args.velocity_scale <= 0:
        raise ValueError("--velocity-scale must be positive")
    if args.sample_steps <= 0:
        raise ValueError("--sample-steps must be positive")
    if args.guidance_scale < 0:
        raise ValueError("--guidance-scale cannot be negative")

    if model_type == "diffusion":
        diffusion_config = checkpoint.get("diffusion_config")
        if not diffusion_config:
            raise ValueError("Diffusion checkpoint is missing diffusion_config")
        diffusion_config = dict(diffusion_config)
        diffusion_config.setdefault("prediction_type", "epsilon")
        model = ConditionalMotionDenoiser(**diffusion_model_init_kwargs(model_config)).to(device)
        model.load_state_dict(state_dict)
        schedule = DiffusionSchedule(**diffusion_config).to(device)
        speaker = build_speaker_condition(args, model_config, dataset_metadata, stats, device)
        diffusion_seed_frames = args.diffusion_seed_frames
        if diffusion_seed_frames is None:
            diffusion_seed_frames = int(model_config.get("seed_frames", 0) or 0)
        if diffusion_seed_frames < 0:
            raise ValueError("--diffusion-seed-frames cannot be negative")
        if diffusion_seed_frames >= window_frames:
            raise ValueError(
                f"--diffusion-seed-frames={diffusion_seed_frames} must be smaller than "
                f"the inference window length ({window_frames} frames)"
            )
        pred_norm = run_diffusion_inference(
            model,
            schedule,
            features,
            window_frames,
            stride_frames,
            device,
            deterministic=args.diffusion_deterministic,
            seed_frames=diffusion_seed_frames,
            speaker=speaker,
            sampler=args.sampler,
            sample_steps=args.sample_steps,
            guidance_scale=args.guidance_scale,
        )
    else:
        diffusion_seed_frames = None
        model = GestureTransformer(**model_init_kwargs(model_config)).to(device)
        model.load_state_dict(state_dict)
        pred_norm = run_inference(model, features, window_frames, stride_frames, device)

    pred_angles = reconstruct_nao_angles(pred_norm, stats, target_mode).astype(np.float32)
    pred_angles = smooth_nao_angles(pred_angles, args.smooth_window).astype(np.float32)
    pred_angles = clamp_nao_angles(pred_angles).astype(np.float32)
    if args.velocity_limit:
        pred_angles = limit_nao_velocity(
            pred_angles, args.fps, scale=args.velocity_scale
        ).astype(np.float32)
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
        "diffusion_deterministic": args.diffusion_deterministic,
        "sampler": args.sampler,
        "sample_steps": args.sample_steps,
        "guidance_scale": args.guidance_scale,
        "smooth_window": args.smooth_window,
        "velocity_limit": args.velocity_limit,
        "velocity_scale": args.velocity_scale,
        "speaker_id": args.speaker_id or (speaker_id_from_clip_id(args.clip_id) if args.clip_id else None),
        "diffusion_seed_frames": diffusion_seed_frames,
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
