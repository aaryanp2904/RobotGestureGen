"""
Preprocessing pipeline for GENEA 2022 dataset.

Produces an LMDB database with sliding windows of aligned (audio, text, motion) data.
Motion is upper-body only: 12 joints, world-space XYZ via batched PyTorch FK,
root-subtracted, normalised, and saved as fp16 numpy arrays.
"""

import sys
import torch
import torchaudio
import numpy as np
import json
import argparse
import lmdb
import pickle
from pathlib import Path
from collections import defaultdict
from transformers import (
    AutoTokenizer, AutoModel,
    Wav2Vec2Processor, Wav2Vec2Model,
)
import torch.nn.functional as F

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
FPS = 30
FRAME_TIME = 1.0 / FPS
WAV2VEC_SR = 16_000

SELECTED_JOINTS = [
    "b_spine0",      # 0  Spine base
    "b_spine3",      # 1  Upper spine
    "b_neck0",       # 2  Neck
    "b_head",        # 3  Head
    "b_r_shoulder",  # 4  Right shoulder
    "b_r_arm",       # 5  Right upper arm
    "b_r_forearm",   # 6  Right forearm
    "b_r_wrist",     # 7  Right wrist
    "b_l_shoulder",  # 8  Left shoulder
    "b_l_arm",       # 9  Left upper arm
    "b_l_forearm",   # 10 Left forearm
    "b_l_wrist",     # 11 Left wrist
]

ROOT_JOINT = "b_root"

# ---------------------------------------------------------------------------
# BVH Parser
# ---------------------------------------------------------------------------

class BVHJoint:
    __slots__ = ("name", "offset", "channels", "channel_start", "children", "parent")

    def __init__(self, name):
        self.name = name
        self.offset = np.zeros(3, dtype=np.float64)
        self.channels = []
        self.channel_start = 0
        self.children = []
        self.parent = None


def parse_bvh_hierarchy(filepath):
    with open(filepath, "r") as fh:
        lines = fh.readlines()

    joints = []
    joint_map = {}
    stack = []
    channel_offset = 0
    i = 0

    while i < len(lines):
        line = lines[i].strip()

        if line.startswith("ROOT") or line.startswith("JOINT"):
            name = line.split()[-1]
            joint = BVHJoint(name)
            if stack:
                parent = stack[-1]
                parent.children.append(joint)
                joint.parent = parent
            joints.append(joint)
            joint_map[name] = joint
            stack.append(joint)
        elif line == "{":
            pass
        elif line.startswith("OFFSET"):
            vals = line.split()[1:]
            offset = np.array([float(v) for v in vals], dtype=np.float64)
            if stack and isinstance(stack[-1], BVHJoint):
                stack[-1].offset = offset
        elif line.startswith("CHANNELS"):
            parts = line.split()
            n_ch = int(parts[1])
            ch_names = parts[2: 2 + n_ch]
            stack[-1].channels = ch_names
            stack[-1].channel_start = channel_offset
            channel_offset += n_ch
        elif line == "}":
            stack.pop()
        elif line.startswith("End Site"):
            stack.append("end_site")
        elif line == "MOTION":
            break
        i += 1

    return joints, joint_map, channel_offset


def parse_bvh_motion(filepath, num_channels):
    with open(filepath, "r") as fh:
        lines = fh.readlines()

    motion_idx = None
    for idx, line in enumerate(lines):
        if line.strip() == "MOTION":
            motion_idx = idx
            break

    num_frames = int(lines[motion_idx + 1].split(":")[1])
    frame_time = float(lines[motion_idx + 2].split(":")[1])

    data = np.zeros((num_frames, num_channels), dtype=np.float32)
    for f in range(num_frames):
        vals = lines[motion_idx + 3 + f].split()
        data[f] = [float(v) for v in vals[:num_channels]]

    return num_frames, frame_time, data

# ---------------------------------------------------------------------------
# GPU-Accelerated Batched Forward Kinematics
# ---------------------------------------------------------------------------

def batch_euler_to_matrix_pt(z_deg, x_deg, y_deg):
    """Batched ZXY Euler to Rotation Matrix on GPU. Inputs shape (N,)."""
    z = torch.deg2rad(z_deg)
    x = torch.deg2rad(x_deg)
    y = torch.deg2rad(y_deg)

    cz, sz = torch.cos(z), torch.sin(z)
    cx, sx = torch.cos(x), torch.sin(x)
    cy, sy = torch.cos(y), torch.sin(y)

    O = torch.zeros_like(z)
    I = torch.ones_like(z)

    Rz = torch.stack([
        cz, -sz, O,
        sz,  cz, O,
        O,   O,  I
    ], dim=1).reshape(-1, 3, 3)

    Rx = torch.stack([
        I,  O,   O,
        O, cx, -sx,
        O, sx,  cx
    ], dim=1).reshape(-1, 3, 3)

    Ry = torch.stack([
        cy,  O, sy,
        O,   I,  O,
       -sy,  O, cy
    ], dim=1).reshape(-1, 3, 3)

    return Rz @ Rx @ Ry


def compute_all_frames_fk_pt(joints, motion_data, selected_names, root_name, device):
    """
    Computes batched Forward Kinematics over ALL frames simultaneously on the GPU.
    motion_data: np.ndarray (N_frames, num_channels)
    Returns: positions (N_frames, num_selected, 3), root_pos (N_frames, 3) as CPU tensors.
    """
    data_pt = torch.tensor(motion_data, dtype=torch.float32, device=device)
    num_frames = data_pt.shape[0]

    world_transforms = {}

    def _compute(joint):
        ch = joint.channels
        cs = joint.channel_start
        
        # Build local transform (N, 4, 4)
        offset = torch.tensor(joint.offset, dtype=torch.float32, device=device).unsqueeze(0)
        local_t = offset + data_pt[:, cs:cs+3] # (N, 3)

        # R is (N, 3, 3)
        R = batch_euler_to_matrix_pt(data_pt[:, cs+3], data_pt[:, cs+4], data_pt[:, cs+5])

        # Pack into 4x4
        M = torch.eye(4, dtype=torch.float32, device=device).unsqueeze(0).repeat(num_frames, 1, 1)
        M[:, :3, :3] = R
        M[:, :3, 3] = local_t

        if joint.parent is not None:
            M = world_transforms[joint.parent.name] @ M

        world_transforms[joint.name] = M

        for child in joint.children:
            _compute(child)

    # Start from roots
    for j in joints:
        if j.parent is None:
            _compute(j)

    # Extract requested positions
    positions = torch.stack([world_transforms[name][:, :3, 3] for name in selected_names], dim=1)
    root_positions = world_transforms[root_name][:, :3, 3]

    return positions.cpu(), root_positions.cpu()

# ---------------------------------------------------------------------------
# Audio processing
# ---------------------------------------------------------------------------

def process_audio(wav_path, target_frames, processor, model, device):
    """Extract Wav2Vec2 features and interpolate (nearest) to target_frames."""
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

            feats = model(**inputs).last_hidden_state.squeeze(0)  # (T_chunk, 768)
            all_features.append(feats)

        audio_feats = torch.cat(all_features, dim=0)

        # Interpolation (Nearest is safer for latent embeddings than Linear)
        audio_feats = audio_feats.unsqueeze(0).permute(0, 2, 1)  # (1, 768, T_audio)
        aligned = F.interpolate(audio_feats, size=target_frames, mode="nearest-exact")
        aligned = aligned.squeeze(0).permute(1, 0)  # (target_frames, 768)

    return aligned.cpu()

def process_text(tsv_path, target_frames, tokenizer, text_model, device, fps=FPS, context_radius_s=3.0, batch_size=32):
    text_features = torch.zeros((target_frames, 768), dtype=torch.float32)

    rows = []
    with open(tsv_path, "r", encoding="utf-8") as fh:
        for line in fh:
            parts = line.rstrip("\n").split("\t")
            if len(parts) < 3: continue
            try:
                start_t, end_t = float(parts[0]), float(parts[1])
            except ValueError: continue
            word = parts[2].strip()
            if not word: continue
            mid_t = 0.5 * (start_t + end_t)
            rows.append((start_t, end_t, mid_t, word))

    if not rows: return text_features

    rows.sort(key=lambda x: x[2])
    n = len(rows)
    mids = [r[2] for r in rows]

    contexts = []
    L, R = 0, 0
    for anchor in range(n):
        anchor_mid = mids[anchor]
        left_t = anchor_mid - context_radius_s
        right_t = anchor_mid + context_radius_s

        while L < n and mids[L] < left_t: L += 1
        if R < L: R = L
        while R < n and mids[R] <= right_t: R += 1

        sentence = " ".join(rows[k][3] for k in range(L, R))
        contexts.append((L, R, sentence))

    text_model.eval()
    sent_embs = []
    with torch.no_grad():
        for i in range(0, len(contexts), batch_size):
            batch_sentences = [c[2] for c in contexts[i : i + batch_size]]
            toks = tokenizer(
                batch_sentences, return_tensors="pt", padding=True, truncation=True, max_length=512
            )
            toks = {k: v.to(device) for k, v in toks.items()}

            out = text_model(**toks).last_hidden_state  # (B, T, 768)
            attn = toks["attention_mask"].unsqueeze(-1)  # (B, T, 1)

            summed = (out * attn).sum(dim=1)
            denom = attn.sum(dim=1).clamp(min=1)
            pooled = (summed / denom).cpu()
            sent_embs.append(pooled)

    sent_embs = torch.cat(sent_embs, dim=0)

    assigned = [False] * n
    for ctx_idx, (L_idx, R_idx, _) in enumerate(contexts):
        emb = sent_embs[ctx_idx]
        for wi in range(L_idx, R_idx):
            if assigned[wi]: continue
            start_t, end_t, _, _ = rows[wi]
            start_f = int(start_t * fps)
            
            # Fix: Ensure at least a 1 frame window for very short words
            end_f = max(start_f + 1, int(end_t * fps)) 
            end_f = min(end_f, target_frames)
            
            if start_f < target_frames:
                text_features[start_f:end_f] = emb
                assigned[wi] = True

    return text_features

# ---------------------------------------------------------------------------
# Normalisation statistics
# ---------------------------------------------------------------------------

class BatchedStats:
    """Calculates global mean/std using highly optimized batched array sums."""
    def __init__(self, shape):
        self.n = 0
        self.sum = np.zeros(shape, dtype=np.float64)
        self.sum_sq = np.zeros(shape, dtype=np.float64)

    def update(self, data):
        # data is expected to be shape (N_frames, 12, 3)
        self.n += data.shape[0]
        self.sum += np.sum(data, axis=0, dtype=np.float64)
        self.sum_sq += np.sum(data**2, axis=0, dtype=np.float64)

    def finalise(self):
        mean = self.sum / max(self.n, 1)
        # Variance = E[X^2] - (E[X])^2
        var = (self.sum_sq / max(self.n, 1)) - (mean ** 2)
        var = np.maximum(var, 0.0) # Clamp at 0 to avoid tiny floating point negatives
        std = np.sqrt(var)
        return mean, std

# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def find_triplets(source_dir):
    source = Path(source_dir)
    files_by_name = defaultdict(dict)
    for ext, key in [(".bvh", "bvh"), (".wav", "wav"), (".tsv", "tsv")]:
        for fp in sorted(source.rglob(f"*{ext}")):
            files_by_name[fp.stem][key] = str(fp)

    triplets = [f for f in files_by_name.values() if all(k in f for k in ("bvh", "wav", "tsv"))]
    return triplets

def main():
    parser = argparse.ArgumentParser(description="Preprocess GENEA 2022 -> LMDB")
    parser.add_argument("--source", type=str, required=True)
    parser.add_argument("--output", type=str, required=True)
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--max-files", type=int, default=None)
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[MAIN] Device: {device}")

    triplets = find_triplets(args.source)
    if args.max_files: triplets = triplets[: args.max_files]
    
    if not triplets:
        print("[MAIN] ERROR: No complete triplets found.")
        return

    print("[SKELETON] Parsing BVH hierarchy ...")
    joints, joint_map, num_channels = parse_bvh_hierarchy(triplets[0]["bvh"])

    skeleton_info = {
        "all_joints": [j.name for j in joints],
        "selected_joints": SELECTED_JOINTS,
        "root_joint": ROOT_JOINT,
        "num_channels": num_channels,
        "hierarchy": { j.name: { "parent": j.parent.name if j.parent else None, "offset": j.offset.tolist()} for j in joints},
    }
    with open(output_dir / "skeleton_info.json", "w") as f:
        json.dump(skeleton_info, f, indent=2)

    # ------------------------------------------------------------------
    # PASS 1 - Computes global statistics (Positions & Velocity)
    # ------------------------------------------------------------------
    print("\n[PASS 1] Computing normalisation statistics ...")
    
    # Use the new batched stats
    joint_stats = BatchedStats((len(SELECTED_JOINTS), 3))
    vel_stats = BatchedStats((len(SELECTED_JOINTS), 3))

    for i, tri in enumerate(triplets):
        stem = Path(tri["bvh"]).stem
        print(f"  [{i+1}/{len(triplets)}] {stem}")
        
        _, _, motion_data = parse_bvh_motion(tri["bvh"], num_channels)
        positions, root_pos = compute_all_frames_fk_pt(joints, motion_data, SELECTED_JOINTS, ROOT_JOINT, device)
        
        # Raw Relative Position
        relative = positions - root_pos.unsqueeze(1) # (N, 12, 3)
        
        # Raw Velocity
        velocity = torch.zeros_like(relative)
        velocity[1:] = relative[1:] - relative[:-1]
        if len(velocity) > 1: velocity[0] = velocity[1] 

        joint_stats.update(relative.numpy())
        vel_stats.update(velocity.numpy())

    joint_mean, joint_std = joint_stats.finalise()
    vel_mean, vel_std = vel_stats.finalise()

    joint_std = np.maximum(joint_std, 1e-6)
    vel_std = np.maximum(vel_std, 1e-6)

    with open(output_dir / "normalization_stats.json", "w") as f:
        json.dump({"joint_mean": joint_mean.tolist(), "joint_std": joint_std.tolist(), 
                   "vel_mean": vel_mean.tolist(), "vel_std": vel_std.tolist()}, f, indent=2)

    jm = torch.tensor(joint_mean, dtype=torch.float32)
    js = torch.tensor(joint_std, dtype=torch.float32)
    vm = torch.tensor(vel_mean, dtype=torch.float32)
    vs = torch.tensor(vel_std, dtype=torch.float32)

    # ------------------------------------------------------------------
    # PASS 2 - Process modalities, window, and write LMDB
    # ------------------------------------------------------------------
    print("\n[PASS 2] Processing modalities and writing LMDB ...")
    tokenizer = AutoTokenizer.from_pretrained("distilbert-base-uncased")
    text_model = AutoModel.from_pretrained("distilbert-base-uncased").eval().to(device)
    wav_processor = Wav2Vec2Processor.from_pretrained("facebook/wav2vec2-base-960h")
    audio_model = Wav2Vec2Model.from_pretrained("facebook/wav2vec2-base-960h").eval().to(device)


    env = lmdb.open(str(output_dir / "dataset.lmdb"), map_size=50 * 1024**3)
    global_window_idx = 0
    file_window_map = {}
    
    fps = args.fps
    window_frames = int(args.window_size * fps)
    stride_frames = int(args.stride * fps)

    for i, tri in enumerate(triplets):
        stem = Path(tri["bvh"]).stem
        print(f"\n[FILE {i+1}/{len(triplets)}] {stem}")

        # Motion Processing
        num_frames_bvh, frame_time, motion_data = parse_bvh_motion(tri["bvh"], num_channels)
        positions, root_pos = compute_all_frames_fk_pt(joints, motion_data, SELECTED_JOINTS, ROOT_JOINT, device)
        
        # Proper Velocity Extraction
        relative = positions - root_pos.unsqueeze(1)
        velocity = torch.zeros_like(relative)
        velocity[1:] = relative[1:] - relative[:-1]
        if len(velocity) > 1: velocity[0] = velocity[1]

        # Independent Standardization
        normalised_pos = (relative - jm) / js 
        normalised_vel = (velocity - vm) / vs

        # Audio / Text processing
        audio_feats = process_audio(tri["wav"], num_frames_bvh, wav_processor, audio_model, device)
        text_feats = process_text(tri["tsv"], num_frames_bvh, tokenizer, text_model, device, fps=fps)

        fused_x = torch.cat([audio_feats, text_feats], dim=-1)

        file_start = global_window_idx
        window_count = 0
        txn = env.begin(write=True)

        for start in range(0, num_frames_bvh - window_frames + 1, stride_frames):
            end = start + window_frames
            w_x = fused_x[start:end]
            w_y = normalised_pos[start:end]
            w_v = normalised_vel[start:end]

            # Store as fp16 numpy arrays to minimize LMDB size
            value = pickle.dumps({
                "x": w_x.numpy().astype(np.float16),
                "y": w_y.numpy().astype(np.float16),
                "y_vel": w_v.numpy().astype(np.float16),
            })
            txn.put(f"{global_window_idx:08d}".encode(), value)

            global_window_idx += 1
            window_count += 1

        txn.commit()
        file_window_map[stem] = {"start_idx": file_start, "count": window_count}

    with env.begin(write=True) as txn:
        txn.put(b"__len__", pickle.dumps(global_window_idx))
    env.close()

    metadata = {
        "total_windows": global_window_idx,
        "window_size_frames": window_frames,
        "stride_frames": stride_frames,
        "input_dim": 1536,
        "output_shape": [window_frames, len(SELECTED_JOINTS), 3],
    }
    with open(output_dir / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)
    print("[MAIN] ✓ Preprocessing complete!")

if __name__ == "__main__":
    main()