#!/usr/bin/env python3
"""
Preprocess BEAT2 clips for direct NAO gesture training.

For every valid BEAT2 clip this script builds:
  - prosody features aligned to the 30 fps motion timeline
  - NAO joint-angle targets produced from SMPL-X poses via the BEATDemo IK path
  - optional sliding-window LMDB records for model training

The model target is intentionally robot-native: 10 NAO head/arm joints.
"""

import argparse
import csv
import json
import math
import pickle
import shutil
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
import torchaudio
from tqdm import tqdm

try:
    from .BEATDemo import map_smplx_to_nao
    from .config import (
        AUDIO_DIR,
        AUDIO_SR,
        MOCAP_FPS,
        MOTION_DIR,
        NAO_OUTPUT_DIR,
        SPLIT_CSV,
        TEXTGRID_DIR,
    )
    from .nao_constants import NAO_JOINTS, NAO_MAX_VEL
    from .parse_annotations import parse_textgrid
except ImportError:
    from BEATDemo import map_smplx_to_nao
    from config import (
        AUDIO_DIR,
        AUDIO_SR,
        MOCAP_FPS,
        MOTION_DIR,
        NAO_OUTPUT_DIR,
        SPLIT_CSV,
        TEXTGRID_DIR,
    )
    from nao_constants import NAO_JOINTS, NAO_MAX_VEL
    from parse_annotations import parse_textgrid


PROSODY_FEATURE_NAMES = [
    "log_f0",
    "f0_confidence",
    "voiced",
    "rms",
    "log_rms",
    "onset_strength",
    "spectral_centroid",
    "delta_log_f0",
    "delta_log_rms",
]

TEXT_EMBED_DIM = 768
DEFAULT_WAVLM_MODEL = "microsoft/wavlm-base-plus"
WAVLM_SAMPLE_RATE = 16000

class BatchedStats:
    """Streaming mean/std over arrays with shape (frames, feature_dim)."""

    def __init__(self, feature_dim: int):
        self.n = 0
        self.sum = np.zeros(feature_dim, dtype=np.float64)
        self.sum_sq = np.zeros(feature_dim, dtype=np.float64)

    def update(self, data: np.ndarray):
        if data.size == 0:
            return
        arr = data.astype(np.float64)
        self.n += arr.shape[0]
        self.sum += arr.sum(axis=0)
        self.sum_sq += np.square(arr).sum(axis=0)

    def finalise(self):
        if self.n == 0:
            return self.sum, np.ones_like(self.sum)
        mean = self.sum / self.n
        var = self.sum_sq / self.n - mean ** 2
        std = np.sqrt(np.maximum(var, 0.0))
        return mean, np.maximum(std, 1e-6)


def read_split_csv(path: Path, splits: set[str] | None = None) -> list[dict]:
    with open(path, "r", newline="") as f:
        rows = list(csv.DictReader(f))
    if splits:
        rows = [row for row in rows if row["type"] in splits]
    return rows


def _align_1d(values: torch.Tensor, target_frames: int, mode="linear") -> torch.Tensor:
    """Resize a 1-D feature track to exactly target_frames."""
    values = values.float().flatten()
    if target_frames <= 0:
        return torch.empty(0, dtype=torch.float32)
    if values.numel() == 0:
        return torch.zeros(target_frames, dtype=torch.float32)
    if values.numel() == target_frames:
        return values
    x = values.view(1, 1, -1)
    if mode == "linear":
        y = F.interpolate(x, size=target_frames, mode="linear", align_corners=False)
    else:
        y = F.interpolate(x, size=target_frames, mode="nearest")
    return y.view(-1)


def _align_2d(values: torch.Tensor, target_frames: int, mode="linear") -> torch.Tensor:
    """Resize a frame-feature matrix to the motion timeline."""
    values = values.float()
    if target_frames <= 0:
        return torch.empty((0, values.shape[-1] if values.ndim == 2 else 0), dtype=torch.float32)
    if values.numel() == 0:
        feature_dim = values.shape[-1] if values.ndim == 2 else 0
        return torch.zeros((target_frames, feature_dim), dtype=torch.float32)
    if values.ndim != 2:
        raise ValueError(f"Expected 2-D features, got shape {tuple(values.shape)}")
    if values.shape[0] == target_frames:
        return values
    x = values.T.unsqueeze(0)
    if mode == "linear":
        y = F.interpolate(x, size=target_frames, mode=mode, align_corners=False)
    else:
        y = F.interpolate(x, size=target_frames, mode=mode)
    return y.squeeze(0).T.contiguous()


def speaker_id_from_clip_id(clip_id: str) -> str:
    """BEAT2 clip ids are prefixed by numeric speaker id, e.g. 2_scott_..."""
    return clip_id.split("_", 1)[0]


def build_speaker_id_map(rows: list[dict]) -> dict[str, int]:
    speakers = sorted(
        {str(row["speaker_id"]) for row in rows},
        key=lambda item: (not item.isdigit(), int(item) if item.isdigit() else item),
    )
    return {speaker_id: idx for idx, speaker_id in enumerate(speakers)}


def load_audio_mono(wav_path: Path, sample_rate=AUDIO_SR) -> torch.Tensor:
    waveform, sr = torchaudio.load(str(wav_path))
    if sr != sample_rate:
        waveform = torchaudio.functional.resample(waveform, sr, sample_rate)
    return waveform.mean(dim=0).contiguous()


def init_wavlm_model(model_name: str, device: torch.device):
    """Load WavLM lazily because it is heavy and optional."""
    from transformers import AutoFeatureExtractor, AutoModel

    processor = AutoFeatureExtractor.from_pretrained(model_name)
    model = AutoModel.from_pretrained(model_name).eval().to(device)
    hidden_size = int(getattr(model.config, "hidden_size", 0))
    if hidden_size <= 0:
        raise ValueError(f"Could not determine WavLM hidden size for {model_name}")
    return processor, model, hidden_size


def extract_wavlm_features(
    wav_path: Path,
    target_frames: int,
    processor,
    model,
    device: torch.device,
    batch_seconds: float = 20.0,
) -> np.ndarray:
    """Extract WavLM frame features and align them to motion frames."""
    waveform = load_audio_mono(wav_path, sample_rate=WAVLM_SAMPLE_RATE)
    if waveform.numel() == 0:
        hidden_size = int(getattr(model.config, "hidden_size", 0))
        return np.zeros((target_frames, hidden_size), dtype=np.float32)

    max_samples = max(1, int(round(batch_seconds * WAVLM_SAMPLE_RATE)))
    chunks = []
    with torch.no_grad():
        for start in range(0, waveform.numel(), max_samples):
            chunk = waveform[start:start + max_samples]
            inputs = processor(
                chunk.cpu().numpy(),
                sampling_rate=WAVLM_SAMPLE_RATE,
                return_tensors="pt",
            )
            inputs = {key: value.to(device) for key, value in inputs.items()}
            hidden = model(**inputs).last_hidden_state.squeeze(0).cpu()
            chunks.append(hidden)
    features = torch.cat(chunks, dim=0) if chunks else torch.empty((0, model.config.hidden_size))
    features = _align_2d(features, target_frames)
    return features.numpy().astype(np.float32)


def cached_wavlm_features(
    wav_path: Path,
    clip_id: str,
    target_frames: int,
    processor,
    model,
    device: torch.device,
    cache_dir: Path,
    model_name: str,
) -> np.ndarray:
    """Load or compute WavLM features for one clip."""
    safe_model_name = model_name.replace("/", "__")
    model_cache_dir = cache_dir / safe_model_name
    model_cache_dir.mkdir(parents=True, exist_ok=True)
    cache_path = model_cache_dir / f"{clip_id}_{target_frames}.npy"
    if cache_path.is_file():
        cached = np.load(str(cache_path)).astype(np.float32)
        hidden_size = int(getattr(model.config, "hidden_size", cached.shape[-1] if cached.ndim == 2 else 0))
        if cached.shape == (target_frames, hidden_size):
            return cached
        print(f"[WARN] Ignoring stale WavLM cache for {clip_id}: shape={cached.shape}")

    features = extract_wavlm_features(wav_path, target_frames, processor, model, device)
    np.save(str(cache_path), features)
    return features


def extract_f0_pyin(waveform: torch.Tensor, sample_rate: int, hop_length: int,
                    target_frames: int) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Use librosa.pyin for speech F0 when available; fall back to torchaudio."""
    try:
        import librosa
    except ImportError:
        try:
            f0_track = torchaudio.functional.detect_pitch_frequency(
                waveform.unsqueeze(0),
                sample_rate,
                frame_time=hop_length / sample_rate,
                freq_low=50,
                freq_high=500,
            ).squeeze(0)
        except Exception:
            f0_track = torch.zeros(target_frames)
        f0 = _align_1d(f0_track, target_frames)
        voiced = (f0 >= 50.0).float()
        confidence = voiced.clone()
        return f0, voiced, confidence

    audio_np = waveform.cpu().numpy()
    f0_np, voiced_np, prob_np = librosa.pyin(
        audio_np,
        fmin=librosa.note_to_hz("C2"),
        fmax=librosa.note_to_hz("C5"),
        sr=sample_rate,
        frame_length=2048,
        hop_length=hop_length,
        center=True,
    )
    f0_np = np.nan_to_num(f0_np, nan=0.0).astype(np.float32)
    prob_np = np.nan_to_num(prob_np, nan=0.0).astype(np.float32)
    voiced_np = voiced_np.astype(np.float32)

    f0 = _align_1d(torch.from_numpy(f0_np), target_frames)
    voiced = _align_1d(torch.from_numpy(voiced_np), target_frames, mode="nearest")
    confidence = _align_1d(torch.from_numpy(prob_np), target_frames)
    voiced = ((voiced > 0.5) & (f0 >= 50.0) & (confidence >= 0.1)).float()
    return f0, voiced, confidence


def extract_prosody(wav_path: Path, target_frames: int, fps=MOCAP_FPS,
                    sample_rate=AUDIO_SR) -> np.ndarray:
    """
    Extract prosody-focused audio features aligned to motion frames.

    Features are deliberately low-level and rhythm/stress oriented:
    pitch/F0, voicing, RMS energy, onset strength, and deltas.
    """
    waveform = load_audio_mono(wav_path, sample_rate=sample_rate)
    if waveform.numel() == 0:
        return np.zeros((target_frames, len(PROSODY_FEATURE_NAMES)), dtype=np.float32)

    hop_length = max(1, int(round(sample_rate / fps)))
    frame_length = int(round(0.050 * sample_rate))  # 50 ms energy window

    padded = F.pad(waveform, (frame_length // 2, frame_length // 2))
    if padded.numel() >= frame_length:
        frames = padded.unfold(0, frame_length, hop_length)
        rms = torch.sqrt(torch.mean(frames ** 2, dim=1) + 1e-8)
    else:
        rms = torch.sqrt(torch.mean(waveform ** 2).view(1) + 1e-8)
    rms = _align_1d(rms, target_frames)
    log_rms = torch.log(rms + 1e-6)

    n_fft = 1024
    win_length = min(n_fft, max(32, waveform.numel()))
    window = torch.hann_window(win_length)
    spec = torch.stft(
        waveform,
        n_fft=n_fft,
        hop_length=hop_length,
        win_length=win_length,
        window=window,
        return_complex=True,
    ).abs()
    log_mag = torch.log1p(spec)
    if log_mag.shape[1] > 1:
        flux = torch.relu(log_mag[:, 1:] - log_mag[:, :-1]).mean(dim=0)
        onset = torch.cat([torch.zeros(1), flux])
    else:
        onset = torch.zeros(1)
    onset = _align_1d(onset, target_frames)

    freqs = torch.linspace(0.0, sample_rate / 2.0, spec.shape[0], dtype=spec.dtype)
    centroid = (spec * freqs[:, None]).sum(dim=0) / spec.sum(dim=0).clamp(min=1e-8)
    centroid = _align_1d(centroid, target_frames)

    f0, voiced, f0_confidence = extract_f0_pyin(
        waveform, sample_rate, hop_length, target_frames
    )
    log_f0 = torch.where(voiced > 0, torch.log(f0.clamp(min=50.0)), torch.zeros_like(f0))

    delta_log_f0 = torch.zeros_like(log_f0)
    delta_log_rms = torch.zeros_like(log_rms)
    if target_frames > 1:
        voiced_pairs = (voiced[1:] > 0) & (voiced[:-1] > 0)
        delta_log_f0[1:] = torch.where(
            voiced_pairs, log_f0[1:] - log_f0[:-1], torch.zeros_like(log_f0[1:])
        )
        delta_log_f0[0] = delta_log_f0[1]
        delta_log_rms[1:] = log_rms[1:] - log_rms[:-1]
        delta_log_rms[0] = delta_log_rms[1]

    features = torch.stack([
        log_f0,
        f0_confidence,
        voiced,
        rms,
        log_rms,
        onset,
        centroid,
        delta_log_f0,
        delta_log_rms,
    ], dim=1)
    return features.numpy().astype(np.float32)


def poses_to_nao_angles(poses: np.ndarray, fps=MOCAP_FPS,
                        velocity_limit=False) -> np.ndarray:
    """Map full SMPL-X pose frames to clamped NAO joint-angle targets."""
    frame_time = 1.0 / fps
    angles = np.zeros((poses.shape[0], len(NAO_JOINTS)), dtype=np.float32)
    last = None

    for i, frame in enumerate(poses):
        mapped = map_smplx_to_nao(frame)
        row = np.array([mapped[name] for name in NAO_JOINTS], dtype=np.float32)

        if velocity_limit and last is not None:
            for j, name in enumerate(NAO_JOINTS):
                max_change = NAO_MAX_VEL.get(name, 5.0) * frame_time
                diff = row[j] - last[j]
                if abs(diff) > max_change:
                    row[j] = last[j] + math.copysign(max_change, diff)
        last = row.copy()
        angles[i] = row

    return angles


def frame_velocity(values: np.ndarray) -> np.ndarray:
    vel = np.zeros_like(values, dtype=np.float32)
    if len(values) > 1:
        vel[1:] = values[1:] - values[:-1]
    return vel


def window_starts(total_frames: int, window_frames: int, stride_frames: int) -> list[int]:
    if total_frames <= window_frames:
        return [0]
    starts = list(range(0, total_frames - window_frames + 1, stride_frames))
    last = total_frames - window_frames
    if starts[-1] != last:
        starts.append(last)
    return starts


def fixed_length_window(values: np.ndarray, start: int, window_frames: int,
                        pad_mode: str = "repeat") -> np.ndarray:
    window = values[start:start + window_frames]
    if len(window) == window_frames:
        return window
    if len(window) == 0 or pad_mode == "zero":
        pad = np.zeros((window_frames, *values.shape[1:]), dtype=values.dtype)
        if len(window) == 0:
            return pad
        return np.concatenate([window, pad[:window_frames - len(window)]], axis=0)
    pad = np.repeat(window[-1:], window_frames - len(window), axis=0)
    return np.concatenate([window, pad], axis=0)


def fixed_length_valid_mask(total_frames: int, start: int, window_frames: int) -> np.ndarray:
    """Return 1 for real frames and 0 for padding in a fixed-length window."""
    valid_len = max(0, min(window_frames, total_frames - start))
    mask = np.zeros((window_frames, 1), dtype=np.float32)
    if valid_len > 0:
        mask[:valid_len, 0] = 1.0
    return mask


def select_training_target(data, stats: dict, target_mode: str) -> np.ndarray:
    """Return the normalized LMDB target for the selected training mode."""
    if target_mode == "angle":
        mean = np.array(stats["nao_mean"], dtype=np.float32)
        std = np.array(stats["nao_std"], dtype=np.float32)
        return (data["nao_angles"] - mean) / std
    if target_mode == "delta":
        mean = np.array(stats["nao_vel_mean"], dtype=np.float32)
        std = np.array(stats["nao_vel_std"], dtype=np.float32)
        return (data["nao_velocity"] - mean) / std
    raise ValueError(f"Unsupported target_mode: {target_mode}")


def load_words(clip_id: str) -> list[dict]:
    tg_path = TEXTGRID_DIR / f"{clip_id}.TextGrid"
    if not tg_path.is_file():
        return []
    try:
        return parse_textgrid(tg_path).get("words", [])
    except Exception as exc:
        print(f"[WARN] TextGrid parse failed for {clip_id}: {exc}")
        return []


def init_text_models(device: torch.device):
    """Load the text encoder lazily because prosody-only preprocessing is common."""
    from transformers import AutoModel, AutoTokenizer

    tokenizer = AutoTokenizer.from_pretrained("distilbert-base-uncased")
    model = AutoModel.from_pretrained("distilbert-base-uncased").eval().to(device)
    return tokenizer, model


def extract_text_features(words: list[dict], target_frames: int, tokenizer, model,
                          device: torch.device, fps=MOCAP_FPS,
                          context_radius_s=3.0, batch_size=32) -> np.ndarray:
    """
    Build per-frame contextual text embeddings from TextGrid word intervals.

    Each word is represented by a pooled DistilBERT embedding of a +/- context
    window around that word, then assigned to the frames covered by the word.
    Silence/non-word frames remain zero.
    """
    features = torch.zeros((target_frames, TEXT_EMBED_DIM), dtype=torch.float32)
    rows = []
    for word in words:
        text = word.get("text", "").strip()
        if not text:
            continue
        start = float(word["start"])
        end = float(word["end"])
        rows.append((start, end, 0.5 * (start + end), text))

    if not rows:
        return features.numpy()

    rows.sort(key=lambda item: item[2])
    mids = [row[2] for row in rows]
    contexts = []
    left = 0
    right = 0
    for anchor, mid in enumerate(mids):
        left_t = mid - context_radius_s
        right_t = mid + context_radius_s
        while left < len(rows) and mids[left] < left_t:
            left += 1
        if right < left:
            right = left
        while right < len(rows) and mids[right] <= right_t:
            right += 1
        sentence = " ".join(rows[i][3] for i in range(left, right))
        contexts.append((left, right, sentence))

    embeddings = []
    with torch.no_grad():
        for start in range(0, len(contexts), batch_size):
            batch_sentences = [ctx[2] for ctx in contexts[start:start + batch_size]]
            toks = tokenizer(
                batch_sentences,
                return_tensors="pt",
                padding=True,
                truncation=True,
                max_length=512,
            )
            toks = {key: value.to(device) for key, value in toks.items()}
            out = model(**toks).last_hidden_state
            mask = toks["attention_mask"].unsqueeze(-1)
            pooled = (out * mask).sum(dim=1) / mask.sum(dim=1).clamp(min=1)
            embeddings.append(pooled.cpu())

    embeddings = torch.cat(embeddings, dim=0)
    assigned = [False] * len(rows)
    for ctx_idx, (left_idx, right_idx, _) in enumerate(contexts):
        emb = embeddings[ctx_idx]
        for word_idx in range(left_idx, right_idx):
            if assigned[word_idx]:
                continue
            start_t, end_t, _, _ = rows[word_idx]
            start_f = max(0, int(start_t * fps))
            end_f = min(target_frames, max(start_f + 1, int(end_t * fps)))
            if start_f < target_frames:
                features[start_f:end_f] = emb
                assigned[word_idx] = True

    return features.numpy().astype(np.float32)


def save_stats(output_dir: Path, stats: dict):
    with open(output_dir / "normalization_stats.json", "w") as f:
        json.dump(stats, f, indent=2)
    arrays = {
        "prosody_mean": np.array(stats["prosody_mean"], dtype=np.float32),
        "prosody_std": np.array(stats["prosody_std"], dtype=np.float32),
        "nao_mean": np.array(stats["nao_mean"], dtype=np.float32),
        "nao_std": np.array(stats["nao_std"], dtype=np.float32),
        "motion_mean": np.array(stats["motion_mean"], dtype=np.float32),
        "motion_std": np.array(stats["motion_std"], dtype=np.float32),
        "nao_vel_mean": np.array(stats["nao_vel_mean"], dtype=np.float32),
        "nao_vel_std": np.array(stats["nao_vel_std"], dtype=np.float32),
        "motion_vel_mean": np.array(stats["motion_vel_mean"], dtype=np.float32),
        "motion_vel_std": np.array(stats["motion_vel_std"], dtype=np.float32),
    }
    if int(stats.get("wavlm_dim", 0)) > 0:
        arrays["wavlm_mean"] = np.array(stats["wavlm_mean"], dtype=np.float32)
        arrays["wavlm_std"] = np.array(stats["wavlm_std"], dtype=np.float32)
    np.savez(str(output_dir / "normalization_stats.npz"), **arrays)


def write_lmdb(output_dir: Path, manifest_rows: list[dict], stats: dict,
               window_frames: int, stride_frames: int, map_size_gb: int,
               text_dim: int = 0, wavlm_dim: int = 0, target_mode: str = "angle",
               speaker_id_map: dict[str, int] | None = None,
               target_representation: str = "nao_angles",
               conditioning_parts: list[str] | None = None,
               prediction_type: str = "epsilon",
               window_seconds: float | None = None,
               stride_seconds: float | None = None):
    try:
        import lmdb
    except ImportError:
        print("[WARN] lmdb is not installed; skipping dataset.lmdb creation.")
        return 0

    prosody_mean = np.array(stats["prosody_mean"], dtype=np.float32)
    prosody_std = np.array(stats["prosody_std"], dtype=np.float32)
    wavlm_mean = np.array(stats.get("wavlm_mean", []), dtype=np.float32)
    wavlm_std = np.array(stats.get("wavlm_std", []), dtype=np.float32)
    vel_mean = np.array(stats["nao_vel_mean"], dtype=np.float32)
    vel_std = np.array(stats["nao_vel_std"], dtype=np.float32)
    if prosody_mean.shape != (len(PROSODY_FEATURE_NAMES),) or prosody_std.shape != (len(PROSODY_FEATURE_NAMES),):
        raise ValueError("Prosody normalization stats do not match the configured prosody feature count")
    if wavlm_dim and (wavlm_mean.shape != (wavlm_dim,) or wavlm_std.shape != (wavlm_dim,)):
        raise ValueError(f"WavLM normalization stats do not match wavlm_dim={wavlm_dim}")
    if vel_mean.shape != (len(NAO_JOINTS),) or vel_std.shape != (len(NAO_JOINTS),):
        raise ValueError("NAO velocity stats do not match the configured joint count")
    speaker_id_map = speaker_id_map or {}
    conditioning_parts = conditioning_parts or ["prosody"]
    speaker_dim = len(speaker_id_map)
    clips_dir = output_dir / "clips"
    rows_by_split = {}
    for row in manifest_rows:
        rows_by_split.setdefault(row["split"], []).append(row)

    output_summary = {}
    total_windows = 0

    for split, rows in sorted(rows_by_split.items()):
        lmdb_path = output_dir / f"{split}.lmdb"
        if lmdb_path.exists():
            if lmdb_path.is_dir():
                shutil.rmtree(lmdb_path)
            else:
                lmdb_path.unlink()
        env = lmdb.open(str(lmdb_path), map_size=map_size_gb * 1024 ** 3)
        global_idx = 0
        file_window_map = {}
        txn = env.begin(write=True)

        for row in tqdm(rows, desc=f"Writing {split}.lmdb windows"):
            clip_id = row["id"]
            data = np.load(str(clips_dir / f"{clip_id}.npz"))
            prosody = (data["prosody"] - prosody_mean) / prosody_std
            if prosody.shape[1] != len(PROSODY_FEATURE_NAMES):
                raise ValueError(
                    f"{clip_id} prosody dim {prosody.shape[1]} does not match "
                    f"{len(PROSODY_FEATURE_NAMES)}"
                )
            if wavlm_dim:
                raw_wavlm = data["wavlm_features"]
                if raw_wavlm.shape[1] != wavlm_dim:
                    raise ValueError(
                        f"{clip_id} WavLM dim {raw_wavlm.shape[1]} does not match {wavlm_dim}"
                    )
                wavlm_features = (raw_wavlm - wavlm_mean) / wavlm_std
            else:
                wavlm_features = np.zeros((len(prosody), 0), dtype=np.float32)
            text_features = data["text_features"].astype(np.float32)
            if text_features.shape[1] != text_dim:
                raise ValueError(
                    f"{clip_id} text dim {text_features.shape[1]} does not match {text_dim}"
                )
            x = np.concatenate([prosody, wavlm_features, text_features], axis=1)
            y = select_training_target(data, stats, target_mode)
            if len(x) != len(y):
                raise ValueError(f"{clip_id} feature/target length mismatch: x={len(x)} y={len(y)}")
            velocity = (data["nao_velocity"] - vel_mean) / vel_std
            speaker_id = str(row.get("speaker_id", data["speaker_id"].item() if "speaker_id" in data else ""))
            speaker_idx = speaker_id_map.get(speaker_id, -1)
            speaker = np.zeros((speaker_dim,), dtype=np.float32)
            if speaker_idx >= 0:
                speaker[speaker_idx] = 1.0

            file_start = global_idx
            window_count = 0
            for start in window_starts(len(y), window_frames, stride_frames):
                y_pad_mode = "zero" if target_mode == "delta" else "repeat"
                value = pickle.dumps({
                    "x": fixed_length_window(x, start, window_frames).astype(np.float16),
                    "y": fixed_length_window(
                        y, start, window_frames, pad_mode=y_pad_mode
                    ).astype(np.float16),
                    "y_vel": fixed_length_window(
                        velocity, start, window_frames, pad_mode="zero"
                    ).astype(np.float16),
                    "speaker": speaker.astype(np.float16),
                    "speaker_id": speaker_id,
                    "clip_id": clip_id,
                    "split": split,
                    "start_frame": start,
                    "valid_mask": fixed_length_valid_mask(
                        len(y), start, window_frames
                    ).astype(np.float16),
                })
                txn.put(f"{global_idx:08d}".encode(), value)
                global_idx += 1
                window_count += 1

                if global_idx % 1000 == 0:
                    txn.commit()
                    txn = env.begin(write=True)

            file_window_map[clip_id] = {"start_idx": file_start, "count": window_count}

        metadata = {
            "split": split,
            "total_windows": global_idx,
            "window_frames": window_frames,
            "stride_frames": stride_frames,
            "fps": MOCAP_FPS,
            "input_dim": len(PROSODY_FEATURE_NAMES) + wavlm_dim + text_dim,
            "prosody_dim": len(PROSODY_FEATURE_NAMES),
            "wavlm_dim": wavlm_dim,
            "wavlm_model": stats.get("wavlm_model"),
            "text_dim": text_dim,
            "speaker_dim": speaker_dim,
            "speaker_id_map": speaker_id_map,
            "target_shape": [len(NAO_JOINTS)],
            "target_mode": target_mode,
            "target_representation": target_representation,
            "target_type": (
                "nao_joint_deltas" if target_mode == "delta" else "nao_joint_angles"
            ),
            "conditioning_parts": conditioning_parts,
            "prediction_type": prediction_type,
            "window_seconds": window_seconds,
            "stride_seconds": stride_seconds,
            "has_valid_mask": True,
            "feature_names": (
                PROSODY_FEATURE_NAMES
                + [f"wavlm_{idx:04d}" for idx in range(wavlm_dim)]
                + [f"text_{idx:04d}" for idx in range(text_dim)]
            ),
            "target_names": NAO_JOINTS,
            "nao_mean": stats["nao_mean"],
            "nao_std": stats["nao_std"],
            "motion_mean": stats["motion_mean"],
            "motion_std": stats["motion_std"],
            "nao_vel_mean": stats["nao_vel_mean"],
            "nao_vel_std": stats["nao_vel_std"],
            "motion_vel_mean": stats["motion_vel_mean"],
            "motion_vel_std": stats["motion_vel_std"],
        }

        txn.put(b"__len__", pickle.dumps(global_idx))
        txn.put(b"__metadata__", pickle.dumps(metadata))
        txn.commit()
        env.close()

        output_summary[split] = {
            **metadata,
            "lmdb_path": str(lmdb_path),
            "file_window_map": file_window_map,
        }
        total_windows += global_idx

    with open(output_dir / "lmdb_index.json", "w") as f:
        json.dump({
            "total_windows": total_windows,
            "lmdb_by_split": output_summary,
        }, f, indent=2)

    return total_windows


def preprocess(args):
    output_dir = Path(args.output_dir)
    clips_dir = output_dir / "clips"
    words_dir = output_dir / "words"
    cache_dir = output_dir / "feature_cache"
    clips_dir.mkdir(parents=True, exist_ok=True)
    words_dir.mkdir(parents=True, exist_ok=True)

    split_filter = set(args.splits.split(",")) if args.splits else None
    split_rows = read_split_csv(Path(args.split_csv), split_filter)
    if args.max_clips:
        split_rows = split_rows[:args.max_clips]

    prosody_stats = BatchedStats(len(PROSODY_FEATURE_NAMES))
    nao_stats = BatchedStats(len(NAO_JOINTS))
    vel_stats = BatchedStats(len(NAO_JOINTS))
    text_dim = TEXT_EMBED_DIM if args.include_text else 0
    wavlm_dim = 0
    wavlm_stats = None
    tokenizer = None
    text_model = None
    text_device = None
    if args.include_text:
        text_device = torch.device(
            "cuda" if torch.cuda.is_available() and not args.text_cpu else "cpu"
        )
        print(f"[TEXT] Loading DistilBERT on {text_device}")
        tokenizer, text_model = init_text_models(text_device)
    wavlm_processor = None
    wavlm_model = None
    wavlm_device = None
    if args.include_wavlm:
        wavlm_device = torch.device(
            "cuda" if torch.cuda.is_available() and not args.wavlm_cpu else "cpu"
        )
        print(f"[WAVLM] Loading {args.wavlm_model} on {wavlm_device}")
        wavlm_processor, wavlm_model, wavlm_dim = init_wavlm_model(args.wavlm_model, wavlm_device)
        wavlm_stats = BatchedStats(wavlm_dim)
    conditioning_parts = ["prosody"]
    if args.include_wavlm:
        conditioning_parts.append("wavlm")
    if args.include_text:
        conditioning_parts.append("text")

    processed = []
    dropped = {
        "missing_motion": [],
        "missing_audio": [],
        "bad_motion": [],
        "unsupported_fps": [],
        "failed": [],
    }
    total_frames = 0

    for row in tqdm(split_rows, desc="Preprocessing BEAT2 clips"):
        clip_id = row["id"]
        split = row["type"]
        npz_path = Path(args.motion_dir) / f"{clip_id}.npz"
        wav_path = Path(args.audio_dir) / f"{clip_id}.wav"

        if not npz_path.is_file():
            dropped["missing_motion"].append(clip_id)
            continue
        if not wav_path.is_file():
            dropped["missing_audio"].append(clip_id)
            continue

        try:
            speaker_id = speaker_id_from_clip_id(clip_id)
            data = np.load(str(npz_path), allow_pickle=True)
            poses = data["poses"]
            fps = int(data["mocap_frame_rate"]) if "mocap_frame_rate" in data else MOCAP_FPS
            if poses.ndim != 2 or poses.shape[1] != 165:
                dropped["bad_motion"].append(clip_id)
                continue
            if fps != MOCAP_FPS:
                print(
                    f"[WARN] Skipping {clip_id}: fps={fps}, expected {MOCAP_FPS}. "
                    "Fixed-size LMDB windows assume one dataset frame rate."
                )
                dropped["unsupported_fps"].append(clip_id)
                continue

            nao_angles = poses_to_nao_angles(
                poses, fps=fps, velocity_limit=args.velocity_limit
            )
            nao_velocity = frame_velocity(nao_angles)
            prosody = extract_prosody(wav_path, poses.shape[0], fps=fps)
            words = load_words(clip_id)
            if args.include_text:
                text_features = extract_text_features(
                    words, poses.shape[0], tokenizer, text_model, text_device, fps=fps
                )
            else:
                text_features = np.zeros((poses.shape[0], 0), dtype=np.float32)
            if args.include_wavlm:
                wavlm_features = cached_wavlm_features(
                    wav_path,
                    clip_id,
                    poses.shape[0],
                    wavlm_processor,
                    wavlm_model,
                    wavlm_device,
                    cache_dir,
                    args.wavlm_model,
                )
            else:
                wavlm_features = np.zeros((poses.shape[0], 0), dtype=np.float32)

            out_path = clips_dir / f"{clip_id}.npz"
            np.savez_compressed(
                str(out_path),
                prosody=prosody,
                wavlm_features=wavlm_features,
                text_features=text_features,
                nao_angles=nao_angles,
                nao_velocity=nao_velocity,
                fps=np.int32(fps),
                num_frames=np.int32(poses.shape[0]),
                clip_id=clip_id,
                speaker_id=speaker_id,
                split=split,
                target_representation=args.target_representation,
                prosody_feature_names=np.array(PROSODY_FEATURE_NAMES),
                wavlm_feature_dim=np.int32(wavlm_dim),
                text_feature_dim=np.int32(text_dim),
                nao_joint_names=np.array(NAO_JOINTS),
            )

            with open(words_dir / f"{clip_id}.json", "w") as f:
                json.dump({"id": clip_id, "words": words}, f, indent=2)

            manifest_row = {
                "id": clip_id,
                "split": split,
                "num_frames": str(poses.shape[0]),
                "duration_sec": f"{poses.shape[0] / fps:.6f}",
                "fps": str(fps),
                "speaker_id": speaker_id,
            }
            processed.append(manifest_row)
            total_frames += poses.shape[0]

            if split == "train":
                prosody_stats.update(prosody)
                if wavlm_stats is not None:
                    wavlm_stats.update(wavlm_features)
                nao_stats.update(nao_angles)
                vel_stats.update(nao_velocity)
        except Exception as exc:
            print(f"[WARN] Failed {clip_id}: {exc}")
            dropped["failed"].append(clip_id)

    manifest_path = output_dir / "manifest.csv"
    with open(manifest_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["id", "split", "num_frames", "duration_sec", "fps", "speaker_id"]
        )
        writer.writeheader()
        writer.writerows(processed)

    prosody_mean, prosody_std = prosody_stats.finalise()
    if wavlm_stats is not None:
        wavlm_mean, wavlm_std = wavlm_stats.finalise()
    else:
        wavlm_mean = np.zeros((0,), dtype=np.float32)
        wavlm_std = np.ones((0,), dtype=np.float32)
    nao_mean, nao_std = nao_stats.finalise()
    vel_mean, vel_std = vel_stats.finalise()
    if prosody_stats.n == 0:
        raise RuntimeError(
            "No training frames were found. Run preprocessing with the train split "
            "included so normalization statistics are valid."
        )
    train_rows = [row for row in processed if row["split"] == "train"]
    speaker_id_map = build_speaker_id_map(train_rows)
    stats = {
        "prosody_feature_names": PROSODY_FEATURE_NAMES,
        "nao_joint_names": NAO_JOINTS,
        "prosody_mean": prosody_mean.tolist(),
        "prosody_std": prosody_std.tolist(),
        "wavlm_mean": wavlm_mean.tolist(),
        "wavlm_std": wavlm_std.tolist(),
        "wavlm_dim": wavlm_dim,
        "wavlm_model": args.wavlm_model if args.include_wavlm else None,
        "nao_mean": nao_mean.tolist(),
        "nao_std": nao_std.tolist(),
        "motion_mean": nao_mean.tolist(),
        "motion_std": nao_std.tolist(),
        "nao_vel_mean": vel_mean.tolist(),
        "nao_vel_std": vel_std.tolist(),
        "motion_vel_mean": vel_mean.tolist(),
        "motion_vel_std": vel_std.tolist(),
        "speaker_id_map": speaker_id_map,
        "speaker_dim": len(speaker_id_map),
        "target_mode": args.target_mode,
        "target_representation": args.target_representation,
        "conditioning_parts": conditioning_parts,
        "prediction_type": args.prediction_type,
        "fps": MOCAP_FPS,
        "num_train_frames": prosody_stats.n,
    }
    save_stats(output_dir, stats)

    summary = {
        "processed": len(processed),
        "total_frames": total_frames,
        "total_hours": total_frames / MOCAP_FPS / 3600,
        "dropped": {k: len(v) for k, v in dropped.items()},
        "motion_dir": str(Path(args.motion_dir)),
        "audio_dir": str(Path(args.audio_dir)),
        "target_mode": args.target_mode,
        "target_representation": args.target_representation,
        "target": "nao_joint_deltas" if args.target_mode == "delta" else "nao_joint_angles",
        "conditioning_parts": conditioning_parts,
        "prosody_features": PROSODY_FEATURE_NAMES,
        "include_text": args.include_text,
        "text_feature_dim": text_dim,
        "include_wavlm": args.include_wavlm,
        "wavlm_model": args.wavlm_model if args.include_wavlm else None,
        "wavlm_feature_dim": wavlm_dim,
        "feature_cache_dir": str(cache_dir),
        "speaker_id_map": speaker_id_map,
        "window_seconds": args.window_size,
        "stride_seconds": args.stride,
        "prediction_type": args.prediction_type,
        "fps": MOCAP_FPS,
    }
    with open(output_dir / "preprocess_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    with open(output_dir / "dropped_clips.json", "w") as f:
        json.dump(dropped, f, indent=2)

    if not args.no_lmdb:
        window_frames = int(round(args.window_size * MOCAP_FPS))
        stride_frames = int(round(args.stride * MOCAP_FPS))
        windows = write_lmdb(
            output_dir, processed, stats, window_frames, stride_frames,
            args.map_size_gb, text_dim=text_dim, wavlm_dim=wavlm_dim,
            target_mode=args.target_mode, speaker_id_map=speaker_id_map,
            target_representation=args.target_representation,
            conditioning_parts=conditioning_parts,
            prediction_type=args.prediction_type,
            window_seconds=args.window_size,
            stride_seconds=args.stride,
        )
        summary["total_windows"] = windows
        with open(output_dir / "preprocess_summary.json", "w") as f:
            json.dump(summary, f, indent=2)

    print("\n" + "=" * 70)
    print("  BEAT2 NAO PREPROCESSING COMPLETE")
    print("=" * 70)
    print(f"  Processed clips: {len(processed)}")
    print(f"  Total frames:    {total_frames:,}")
    print(f"  Output:          {output_dir}")
    print(f"  Target joints:   {len(NAO_JOINTS)}")
    print(f"  Prosody dim:     {len(PROSODY_FEATURE_NAMES)}")
    print(f"  WavLM dim:       {wavlm_dim}")
    print(f"  Text dim:        {text_dim}")


def main():
    parser = argparse.ArgumentParser(description="Preprocess BEAT2 for NAO gesture training")
    parser.add_argument("--motion-dir", type=str, default=str(MOTION_DIR))
    parser.add_argument("--audio-dir", type=str, default=str(AUDIO_DIR))
    parser.add_argument("--split-csv", type=str, default=str(SPLIT_CSV))
    parser.add_argument("--output-dir", type=str, default=str(NAO_OUTPUT_DIR))
    parser.add_argument("--splits", type=str, default=None,
                        help="Comma-separated split filter, e.g. train,val")
    parser.add_argument("--max-clips", type=int, default=None)
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    parser.add_argument("--map-size-gb", type=int, default=100)
    parser.add_argument("--no-lmdb", action="store_true",
                        help="Only write per-clip .npz files and stats")
    parser.add_argument("--include-text", action="store_true",
                        help="Add per-frame DistilBERT TextGrid embeddings to LMDB inputs")
    parser.add_argument("--text-cpu", action="store_true",
                        help="Run DistilBERT text embedding on CPU even if CUDA is available")
    parser.add_argument("--include-wavlm", dest="include_wavlm", action="store_true",
                        default=True,
                        help="Add per-frame WavLM audio embeddings to LMDB inputs (default)")
    parser.add_argument("--no-wavlm", dest="include_wavlm", action="store_false",
                        help="Disable WavLM features for a fast prosody/text ablation")
    parser.add_argument("--wavlm-model", type=str, default=DEFAULT_WAVLM_MODEL,
                        help="Hugging Face WavLM model name")
    parser.add_argument("--wavlm-cpu", action="store_true",
                        help="Run WavLM feature extraction on CPU even if CUDA is available")
    parser.add_argument("--velocity-limit", action="store_true",
                        help="Apply NAO speed limiting to stored training targets")
    parser.add_argument("--target-mode", choices=["angle", "delta"], default="angle",
                        help="LMDB target: absolute NAO angles or frame-to-frame deltas")
    parser.add_argument("--target-representation", choices=["nao_angles"],
                        default="nao_angles",
                        help="Motion representation stored as the diffusion target")
    parser.add_argument("--prediction-type", choices=["x0", "epsilon"],
                        default="epsilon",
                        help="Default diffusion prediction target recorded in metadata")
    parser.add_argument("--disable-velocity-limit", action="store_true",
                        help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.disable_velocity_limit:
        args.velocity_limit = False
    if args.window_size <= 0:
        raise ValueError("--window-size must be positive")
    if args.stride <= 0:
        raise ValueError("--stride must be positive")
    if args.map_size_gb <= 0:
        raise ValueError("--map-size-gb must be positive")

    if not Path(args.split_csv).is_file():
        print(f"[ERROR] Split CSV not found: {args.split_csv}")
        sys.exit(1)
    preprocess(args)


if __name__ == "__main__":
    main()
