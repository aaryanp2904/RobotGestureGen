#!/usr/bin/env python3
"""
Phase 1 orchestrator — runs all preprocessing steps in sequence.

Usage:
    python run_phase1.py [--output-dir /path/to/output] [--skip-positions]

Steps:
    1. validate_dataset  → manifest.csv + dropped_clips.json
    2. extract_motion    → motion/*.npz (upper-body axis-angle, 42 dims)
    3. parse_annotations → annotations/*.json (words, phones, gestures)
    4. extract_positions → positions/*.npz (world-space XYZ, optional)
    5. compute_stats     → normalization statistics for training
"""

import json
import argparse
import numpy as np
from pathlib import Path
from tqdm import tqdm

from config import OUTPUT_DIR, MOCAP_FPS, UPPER_BODY_JOINT_NAMES, EXTRACTED_POSE_DIM


def compute_normalization_stats(output_dir: Path):
    """
    Compute global mean/std over all training-split motion clips.

    Computes stats for:
      - axis-angle pose values (42 dims)
      - frame-to-frame velocity (42 dims)
      - root translation (3 dims)
      - root velocity (3 dims)
    """
    motion_dir = output_dir / "motion"
    manifest_path = output_dir / "manifest.csv"

    import csv
    with open(manifest_path, "r") as f:
        clips = list(csv.DictReader(f))

    # Only compute stats on TRAINING data to avoid data leakage
    train_clips = [c for c in clips if c["split"] == "train"]
    print(f"\n[STATS] Computing normalisation stats on {len(train_clips)} training clips")

    # Two-pass batched statistics (numerically stable enough for fp32 motion data,
    # and orders of magnitude faster than per-sample Welford)
    class BatchedStats:
        def __init__(self, shape):
            self.n = 0
            self.sum = np.zeros(shape, dtype=np.float64)
            self.sum_sq = np.zeros(shape, dtype=np.float64)

        def update_batch(self, data):
            """Update with a batch of data, shape (B, *shape)."""
            self.n += data.shape[0]
            self.sum += np.sum(data, axis=0, dtype=np.float64)
            self.sum_sq += np.sum(data.astype(np.float64) ** 2, axis=0)

        def finalise(self):
            if self.n < 2:
                return self.sum / max(self.n, 1), np.ones_like(self.sum)
            mean = self.sum / self.n
            var = (self.sum_sq / self.n) - (mean ** 2)
            std = np.sqrt(np.maximum(var, 0.0))
            return mean, std

    pose_stats = BatchedStats(EXTRACTED_POSE_DIM)
    vel_stats = BatchedStats(EXTRACTED_POSE_DIM)
    trans_stats = BatchedStats(3)
    trans_vel_stats = BatchedStats(3)

    total_frames = 0
    for clip in tqdm(train_clips, desc="Computing stats"):
        clip_id = clip["id"]
        motion_path = motion_dir / f"{clip_id}.npz"
        if not motion_path.is_file():
            continue

        data = np.load(str(motion_path))
        poses = data["upper_body_poses"]  # (T, 42)
        trans = data["trans"]             # (T, 3)
        T = poses.shape[0]

        # Velocity (frame-to-frame difference)
        vel = np.zeros_like(poses)
        if T > 1:
            vel[1:] = poses[1:] - poses[:-1]
            vel[0] = vel[1]

        trans_vel = np.zeros_like(trans)
        if T > 1:
            trans_vel[1:] = trans[1:] - trans[:-1]
            trans_vel[0] = trans_vel[1]

        pose_stats.update_batch(poses)
        vel_stats.update_batch(vel)
        trans_stats.update_batch(trans)
        trans_vel_stats.update_batch(trans_vel)
        total_frames += T

    pose_mean, pose_std = pose_stats.finalise()
    vel_mean, vel_std = vel_stats.finalise()
    trans_mean, trans_std = trans_stats.finalise()
    trans_vel_mean, trans_vel_std = trans_vel_stats.finalise()

    # Clamp minimum std to avoid division by zero
    pose_std = np.maximum(pose_std, 1e-6)
    vel_std = np.maximum(vel_std, 1e-6)
    trans_std = np.maximum(trans_std, 1e-6)
    trans_vel_std = np.maximum(trans_vel_std, 1e-6)

    # Save as JSON (human-readable) and NPZ (fast loading)
    stats = {
        "pose_mean": pose_mean.tolist(),
        "pose_std": pose_std.tolist(),
        "vel_mean": vel_mean.tolist(),
        "vel_std": vel_std.tolist(),
        "trans_mean": trans_mean.tolist(),
        "trans_std": trans_std.tolist(),
        "trans_vel_mean": trans_vel_mean.tolist(),
        "trans_vel_std": trans_vel_std.tolist(),
        "total_train_frames": total_frames,
        "total_train_hours": total_frames / MOCAP_FPS / 3600,
        "num_train_clips": len(train_clips),
        "joints": ["global_orient"] + UPPER_BODY_JOINT_NAMES,
        "pose_dim": EXTRACTED_POSE_DIM,
    }

    with open(output_dir / "normalization_stats.json", "w") as f:
        json.dump(stats, f, indent=2)

    np.savez(
        str(output_dir / "normalization_stats.npz"),
        pose_mean=pose_mean, pose_std=pose_std,
        vel_mean=vel_mean, vel_std=vel_std,
        trans_mean=trans_mean, trans_std=trans_std,
        trans_vel_mean=trans_vel_mean, trans_vel_std=trans_vel_std,
    )

    print(f"  Total training frames: {total_frames:,}")
    print(f"  Total training hours:  {total_frames / MOCAP_FPS / 3600:.2f}")
    print(f"  Pose mean range:  [{pose_mean.min():.4f}, {pose_mean.max():.4f}]")
    print(f"  Pose std range:   [{pose_std.min():.6f}, {pose_std.max():.4f}]")
    print(f"  Vel std range:    [{vel_std.min():.6f}, {vel_std.max():.4f}]")


def main():
    parser = argparse.ArgumentParser(description="Run Phase 1 preprocessing pipeline")
    parser.add_argument("--output-dir", type=str, default=str(OUTPUT_DIR))
    parser.add_argument("--skip-positions", action="store_true",
                        help="Skip SMPL-X FK position extraction (Step 4)")
    parser.add_argument("--smplx-model-dir", type=str, default=None,
                        help="Path to SMPL-X model directory (for Step 4)")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)

    # ---- Step 1: Validate ----
    print("\n" + "=" * 70)
    print("  STEP 1: VALIDATE DATASET")
    print("=" * 70)
    from validate_dataset import validate
    valid_clips = validate(output_dir)

    # ---- Step 2: Extract Motion ----
    print("\n" + "=" * 70)
    print("  STEP 2: EXTRACT UPPER-BODY MOTION")
    print("=" * 70)
    from extract_motion import extract_all
    extract_all(output_dir / "manifest.csv", output_dir)

    # ---- Step 3: Parse Annotations ----
    print("\n" + "=" * 70)
    print("  STEP 3: PARSE ANNOTATIONS")
    print("=" * 70)
    from parse_annotations import parse_all
    parse_all(output_dir / "manifest.csv", output_dir)

    # ---- Step 4: FK Position Extraction (optional) ----
    if not args.skip_positions and args.smplx_model_dir:
        print("\n" + "=" * 70)
        print("  STEP 4: SMPL-X FK POSITION EXTRACTION")
        print("=" * 70)
        from extract_positions import extract_positions_smplx
        extract_positions_smplx(
            output_dir / "manifest.csv", output_dir, Path(args.smplx_model_dir)
        )
    else:
        if args.skip_positions:
            print("\n[SKIP] Step 4: FK position extraction (--skip-positions)")
        else:
            print("\n[SKIP] Step 4: FK position extraction (no --smplx-model-dir provided)")

    # ---- Step 5: Compute Normalization Stats ----
    print("\n" + "=" * 70)
    print("  STEP 5: COMPUTE NORMALISATION STATISTICS")
    print("=" * 70)
    compute_normalization_stats(output_dir)

    # ---- Done ----
    print("\n" + "=" * 70)
    print("  PHASE 1 COMPLETE ✓")
    print("=" * 70)

    # Print output summary
    print(f"\n  Output directory: {output_dir}")
    print(f"  Contents:")
    for p in sorted(output_dir.iterdir()):
        if p.is_dir():
            count = sum(1 for _ in p.iterdir())
            print(f"    {p.name}/  ({count} files)")
        else:
            size_kb = p.stat().st_size / 1024
            print(f"    {p.name}  ({size_kb:.1f} KB)")


if __name__ == "__main__":
    main()
