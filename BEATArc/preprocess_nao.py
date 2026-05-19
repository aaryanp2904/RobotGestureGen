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


def load_audio_mono(wav_path: Path, sample_rate=AUDIO_SR) -> torch.Tensor:
    waveform, sr = torchaudio.load(str(wav_path))
    if sr != sample_rate:
        waveform = torchaudio.functional.resample(waveform, sr, sample_rate)
    return waveform.mean(dim=0).contiguous()


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
        vel[0] = vel[1]
    return vel


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
    np.savez(
        str(output_dir / "normalization_stats.npz"),
        prosody_mean=np.array(stats["prosody_mean"], dtype=np.float32),
        prosody_std=np.array(stats["prosody_std"], dtype=np.float32),
        nao_mean=np.array(stats["nao_mean"], dtype=np.float32),
        nao_std=np.array(stats["nao_std"], dtype=np.float32),
        nao_vel_mean=np.array(stats["nao_vel_mean"], dtype=np.float32),
        nao_vel_std=np.array(stats["nao_vel_std"], dtype=np.float32),
    )


def write_lmdb(output_dir: Path, manifest_rows: list[dict], stats: dict,
               window_frames: int, stride_frames: int, map_size_gb: int,
               text_dim: int = 0):
    try:
        import lmdb
    except ImportError:
        print("[WARN] lmdb is not installed; skipping dataset.lmdb creation.")
        return 0

    prosody_mean = np.array(stats["prosody_mean"], dtype=np.float32)
    prosody_std = np.array(stats["prosody_std"], dtype=np.float32)
    nao_mean = np.array(stats["nao_mean"], dtype=np.float32)
    nao_std = np.array(stats["nao_std"], dtype=np.float32)
    vel_mean = np.array(stats["nao_vel_mean"], dtype=np.float32)
    vel_std = np.array(stats["nao_vel_std"], dtype=np.float32)
    clips_dir = output_dir / "clips"
    rows_by_split = {}
    for row in manifest_rows:
        rows_by_split.setdefault(row["split"], []).append(row)

    output_summary = {}
    total_windows = 0

    for split, rows in sorted(rows_by_split.items()):
        lmdb_path = output_dir / f"{split}.lmdb"
        env = lmdb.open(str(lmdb_path), map_size=map_size_gb * 1024 ** 3)
        global_idx = 0
        file_window_map = {}
        txn = env.begin(write=True)

        for row in tqdm(rows, desc=f"Writing {split}.lmdb windows"):
            clip_id = row["id"]
            data = np.load(str(clips_dir / f"{clip_id}.npz"))
            prosody = (data["prosody"] - prosody_mean) / prosody_std
            text_features = data["text_features"].astype(np.float32)
            x = np.concatenate([prosody, text_features], axis=1)
            angles = (data["nao_angles"] - nao_mean) / nao_std
            velocity = (data["nao_velocity"] - vel_mean) / vel_std

            file_start = global_idx
            window_count = 0
            for start in range(0, len(angles) - window_frames + 1, stride_frames):
                end = start + window_frames
                value = pickle.dumps({
                    "x": x[start:end].astype(np.float16),
                    "y": angles[start:end].astype(np.float16),
                    "y_vel": velocity[start:end].astype(np.float16),
                    "clip_id": clip_id,
                    "split": split,
                    "start_frame": start,
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
            "input_dim": len(PROSODY_FEATURE_NAMES) + text_dim,
            "prosody_dim": len(PROSODY_FEATURE_NAMES),
            "text_dim": text_dim,
            "target_shape": [len(NAO_JOINTS)],
            "target_type": "nao_joint_angles",
            "feature_names": PROSODY_FEATURE_NAMES,
            "target_names": NAO_JOINTS,
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
    tokenizer = None
    text_model = None
    text_device = None
    if args.include_text:
        text_device = torch.device(
            "cuda" if torch.cuda.is_available() and not args.text_cpu else "cpu"
        )
        print(f"[TEXT] Loading DistilBERT on {text_device}")
        tokenizer, text_model = init_text_models(text_device)

    processed = []
    dropped = {"missing_motion": [], "missing_audio": [], "bad_motion": [], "failed": []}
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
            data = np.load(str(npz_path), allow_pickle=True)
            poses = data["poses"]
            fps = int(data["mocap_frame_rate"]) if "mocap_frame_rate" in data else MOCAP_FPS
            if poses.ndim != 2 or poses.shape[1] != 165:
                dropped["bad_motion"].append(clip_id)
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

            out_path = clips_dir / f"{clip_id}.npz"
            np.savez_compressed(
                str(out_path),
                prosody=prosody,
                text_features=text_features,
                nao_angles=nao_angles,
                nao_velocity=nao_velocity,
                fps=np.int32(fps),
                num_frames=np.int32(poses.shape[0]),
                clip_id=clip_id,
                split=split,
                prosody_feature_names=np.array(PROSODY_FEATURE_NAMES),
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
            }
            processed.append(manifest_row)
            total_frames += poses.shape[0]

            if split == "train":
                prosody_stats.update(prosody)
                nao_stats.update(nao_angles)
                vel_stats.update(nao_velocity)
        except Exception as exc:
            print(f"[WARN] Failed {clip_id}: {exc}")
            dropped["failed"].append(clip_id)

    manifest_path = output_dir / "manifest.csv"
    with open(manifest_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["id", "split", "num_frames", "duration_sec", "fps"]
        )
        writer.writeheader()
        writer.writerows(processed)

    prosody_mean, prosody_std = prosody_stats.finalise()
    nao_mean, nao_std = nao_stats.finalise()
    vel_mean, vel_std = vel_stats.finalise()
    stats = {
        "prosody_feature_names": PROSODY_FEATURE_NAMES,
        "nao_joint_names": NAO_JOINTS,
        "prosody_mean": prosody_mean.tolist(),
        "prosody_std": prosody_std.tolist(),
        "nao_mean": nao_mean.tolist(),
        "nao_std": nao_std.tolist(),
        "nao_vel_mean": vel_mean.tolist(),
        "nao_vel_std": vel_std.tolist(),
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
        "target": "nao_joint_angles",
        "prosody_features": PROSODY_FEATURE_NAMES,
        "include_text": args.include_text,
        "text_feature_dim": text_dim,
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
            args.map_size_gb, text_dim=text_dim
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
    parser.add_argument("--velocity-limit", action="store_true",
                        help="Apply NAO speed limiting to stored training targets")
    parser.add_argument("--disable-velocity-limit", action="store_true",
                        help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.disable_velocity_limit:
        args.velocity_limit = False

    if not Path(args.split_csv).is_file():
        print(f"[ERROR] Split CSV not found: {args.split_csv}")
        sys.exit(1)
    preprocess(args)


if __name__ == "__main__":
    main()
