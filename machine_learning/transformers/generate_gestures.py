"""
Inference script for the gesture generation pipeline.

Takes raw WAV + TSV files, processes them identically to preprocessing.py,
runs the trained model on sliding windows, stitches results, and denormalizes
back to world-space XYZ coordinates.

Usage:
    python generate_gestures.py \
        --model gesture_transformer_full_trained.pth \
        --stats /path/to/normalization_stats.json \
        --wav /path/to/audio.wav \
        --tsv /path/to/text.tsv \
        --output predictions.npy
"""

import torch
import torchaudio
import numpy as np
import json
import argparse
from pathlib import Path
from transformers import AutoTokenizer, AutoModel, Wav2Vec2Processor, Wav2Vec2Model
import torch.nn.functional as F

from pos_end import GestureTransformer

# Must match preprocessing.py
FPS = 30
WAV2VEC_SR = 16_000
SELECTED_JOINTS = [
    "b_spine0", "b_spine3", "b_neck0", "b_head",
    "b_r_shoulder", "b_r_arm", "b_r_forearm", "b_r_wrist",
    "b_l_shoulder", "b_l_arm", "b_l_forearm", "b_l_wrist",
]


# ---------------------------------------------------------------------------
# Feature extraction (must match preprocessing.py exactly)
# ---------------------------------------------------------------------------

def process_audio(wav_path, target_frames, processor, model, device):
    """Extract Wav2Vec2 features and interpolate to target_frames."""
    waveform, sr = torchaudio.load(wav_path)
    if sr != WAV2VEC_SR:
        waveform = torchaudio.functional.resample(waveform, orig_freq=sr, new_freq=WAV2VEC_SR)
    waveform = waveform.mean(dim=0)  # mono

    chunk_samples = 30 * WAV2VEC_SR
    total_samples = waveform.shape[0]
    all_features = []

    model.eval()
    with torch.no_grad():
        for start in range(0, total_samples, chunk_samples):
            end = min(start + chunk_samples, total_samples)
            chunk = waveform[start:end]
            inputs = processor(chunk.numpy(), sampling_rate=WAV2VEC_SR, return_tensors="pt")
            inputs = {k: v.to(device) for k, v in inputs.items()}
            feats = model(**inputs).last_hidden_state.squeeze(0)
            all_features.append(feats)

        audio_feats = torch.cat(all_features, dim=0)
        audio_feats = audio_feats.unsqueeze(0).permute(0, 2, 1)
        aligned = F.interpolate(audio_feats, size=target_frames, mode="nearest-exact")
        aligned = aligned.squeeze(0).permute(1, 0)

    return aligned.cpu()


def process_text(tsv_path, target_frames, tokenizer, text_model, device,
                 fps=FPS, context_radius_s=3.0, batch_size=32):
    """Extract contextual DistilBERT features with ±3s sliding context windows."""
    text_features = torch.zeros((target_frames, 768), dtype=torch.float32)

    rows = []
    with open(tsv_path, "r", encoding="utf-8") as fh:
        for line in fh:
            parts = line.rstrip("\n").split("\t")
            if len(parts) < 3:
                continue
            try:
                start_t, end_t = float(parts[0]), float(parts[1])
            except ValueError:
                continue
            word = parts[2].strip()
            if not word:
                continue
            mid_t = 0.5 * (start_t + end_t)
            rows.append((start_t, end_t, mid_t, word))

    if not rows:
        return text_features

    rows.sort(key=lambda x: x[2])
    n = len(rows)
    mids = [r[2] for r in rows]

    # Build ±context_radius_s context windows via two-pointer sweep
    contexts = []
    L, R = 0, 0
    for anchor in range(n):
        anchor_mid = mids[anchor]
        left_t = anchor_mid - context_radius_s
        right_t = anchor_mid + context_radius_s
        while L < n and mids[L] < left_t:
            L += 1
        if R < L:
            R = L
        while R < n and mids[R] <= right_t:
            R += 1
        sentence = " ".join(rows[k][3] for k in range(L, R))
        contexts.append((L, R, sentence))

    # Batch encode all context sentences
    text_model.eval()
    sent_embs = []
    with torch.no_grad():
        for i in range(0, len(contexts), batch_size):
            batch_sentences = [c[2] for c in contexts[i: i + batch_size]]
            toks = tokenizer(
                batch_sentences, return_tensors="pt", padding=True,
                truncation=True, max_length=512,
            )
            toks = {k: v.to(device) for k, v in toks.items()}
            out = text_model(**toks).last_hidden_state
            attn = toks["attention_mask"].unsqueeze(-1)
            summed = (out * attn).sum(dim=1)
            denom = attn.sum(dim=1).clamp(min=1)
            sent_embs.append((summed / denom).cpu())

    sent_embs = torch.cat(sent_embs, dim=0)

    # Assign embeddings to frame spans
    assigned = [False] * n
    for ctx_idx, (L_idx, R_idx, _) in enumerate(contexts):
        emb = sent_embs[ctx_idx]
        for wi in range(L_idx, R_idx):
            if assigned[wi]:
                continue
            start_t, end_t, _, _ = rows[wi]
            start_f = int(start_t * fps)
            end_f = max(start_f + 1, int(end_t * fps))
            end_f = min(end_f, target_frames)
            if start_f < target_frames:
                text_features[start_f:end_f] = emb
                assigned[wi] = True

    return text_features


# ---------------------------------------------------------------------------
# Generator
# ---------------------------------------------------------------------------

class GestureGenerator:
    """
    Loads a trained model and generates denormalized gesture predictions
    from raw WAV + TSV input, processing the full duration via sliding windows.
    """

    def __init__(self, model_path, stats_path, device=None):
        if device is None:
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.device = device
        print(f"[GEN] Device: {device}", flush=True)

        # Model
        self.model = GestureTransformer().to(device)
        self.model.load_state_dict(
            torch.load(model_path, map_location=device, weights_only=True)
        )
        self.model.eval()
        print(f"[GEN] Model: {model_path}", flush=True)

        # Normalization stats
        with open(stats_path, "r") as f:
            stats = json.load(f)
        self.joint_mean = torch.tensor(stats["joint_mean"], dtype=torch.float32, device=device)
        self.joint_std = torch.tensor(stats["joint_std"], dtype=torch.float32, device=device)

        # Feature extractors
        print("[GEN] Loading feature extractors...", flush=True)
        self.tokenizer = AutoTokenizer.from_pretrained("distilbert-base-uncased")
        self.text_model = AutoModel.from_pretrained("distilbert-base-uncased").eval().to(device)
        self.wav_processor = Wav2Vec2Processor.from_pretrained("facebook/wav2vec2-base-960h")
        self.audio_model = Wav2Vec2Model.from_pretrained("facebook/wav2vec2-base-960h").eval().to(device)
        print("[GEN] Ready.", flush=True)

    def generate_from_raw(self, wav_path, tsv_path,
                          window_sec=2.0, stride_sec=0.5, fps=FPS):
        """
        Process full-length WAV + TSV, run model on sliding windows,
        stitch with overlap averaging, and denormalize.

        Returns:
            np.ndarray of shape (total_frames, 12, 3) — root-relative world XYZ
        """
        # Determine duration
        waveform_probe, sr_probe = torchaudio.load(wav_path)
        duration_sec = waveform_probe.shape[1] / sr_probe
        total_frames = int(duration_sec * fps)
        window_frames = int(window_sec * fps)
        stride_frames = int(stride_sec * fps)
        print(f"[GEN] {duration_sec:.1f}s → {total_frames} frames "
              f"(win={window_frames}, stride={stride_frames})", flush=True)

        # Extract features
        audio_feats = process_audio(wav_path, total_frames, self.wav_processor,
                                    self.audio_model, self.device)
        text_feats = process_text(tsv_path, total_frames, self.tokenizer,
                                  self.text_model, self.device, fps=fps)
        fused = torch.cat([audio_feats, text_feats], dim=-1)

        # Sliding window inference with overlap averaging
        sum_preds = torch.zeros(total_frames, 12, 3)
        counts = torch.zeros(total_frames, 1, 1)
        num_windows = 0

        for start in range(0, total_frames - window_frames + 1, stride_frames):
            end = start + window_frames
            x = fused[start:end].unsqueeze(0).to(self.device)
            with torch.no_grad():
                pred = self.model(x)
            sum_preds[start:end] += pred.squeeze(0).cpu()
            counts[start:end] += 1
            num_windows += 1

        avg_preds = sum_preds / counts.clamp(min=1)

        # Denormalize: reverse (x - mean) / std
        denorm = avg_preds * self.joint_std.cpu() + self.joint_mean.cpu()
        result = denorm.numpy()
        print(f"[GEN] ✓ {num_windows} windows → {result.shape}", flush=True)
        return result

    def save_predictions(self, predictions, output_path):
        """Save predictions as .npy file."""
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        np.save(output_path, predictions)
        print(f"[GEN] Saved {output_path} — {predictions.shape}", flush=True)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate gestures from raw WAV+TSV")
    parser.add_argument("--model", type=str, required=True, help="Path to .pth weights")
    parser.add_argument("--stats", type=str, required=True, help="Path to normalization_stats.json")
    parser.add_argument("--wav", type=str, required=True, help="Path to WAV file")
    parser.add_argument("--tsv", type=str, required=True, help="Path to TSV file")
    parser.add_argument("--output", type=str, default="predictions.npy")
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    args = parser.parse_args()

    gen = GestureGenerator(args.model, args.stats)
    preds = gen.generate_from_raw(
        args.wav, args.tsv,
        window_sec=args.window_size, stride_sec=args.stride,
    )
    gen.save_predictions(preds, args.output)
