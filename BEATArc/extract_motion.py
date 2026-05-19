#!/usr/bin/env python3
"""
Step 2 — Extract upper-body motion from BEAT2 SMPL-X .npz files.

For each validated clip, this script:
  1. Loads the full poses (T, 165) from the .npz
  2. Selectively extracts global orient + 13 upper-body joints → (T, 42)
  3. Saves the cleaned motion as a compact .npz alongside metadata

The output is axis-angle rotations. A separate FK step (extract_positions.py)
can convert these to world-space XYZ positions if an SMPL-X body model is
available.

Usage:
    python extract_motion.py [--output-dir /path/to/output] [--manifest /path/to/manifest.csv]
"""

import csv
import json
import argparse
import numpy as np
from pathlib import Path
from tqdm import tqdm

from config import (
    MOTION_DIR, OUTPUT_DIR, MOCAP_FPS,
    UPPER_BODY_JOINT_INDICES, UPPER_BODY_JOINT_NAMES, EXTRACTED_POSE_DIM,
)

# Pre-compute column indices for vectorized extraction (no Python loop)
_EXTRACT_COLS = np.array(
    list(range(3))  # global orient
    + [3 + idx * 3 + c for idx in UPPER_BODY_JOINT_INDICES for c in range(3)]
)


def extract_upper_body(poses: np.ndarray) -> np.ndarray:
    """
    Extract global orient + upper-body joints from full SMPL-X poses.
    
    Args:
        poses: (T, 165) full SMPL-X pose parameters in axis-angle.
        
    Returns:
        (T, 42) — [global_orient(3), spine1(3), spine2(3), ... right_wrist(3)]
    """
    return poses[:, _EXTRACT_COLS]


def extract_all(manifest_path: Path, output_dir: Path):
    motion_out = output_dir / "motion"
    motion_out.mkdir(parents=True, exist_ok=True)

    # Load manifest
    clips = []
    with open(manifest_path, "r") as f:
        reader = csv.DictReader(f)
        clips = list(reader)
    print(f"[EXTRACT] Processing {len(clips)} clips from manifest")

    # Track per-speaker betas for consistency check
    speaker_betas = {}
    stats = {"processed": 0, "skipped": 0, "total_frames": 0}
    duration_map = {}

    for clip in tqdm(clips, desc="Extracting motion"):
        clip_id = clip["id"]
        speaker_id = int(clip["speaker_id"])
        npz_path = MOTION_DIR / f"{clip_id}.npz"

        if not npz_path.is_file():
            stats["skipped"] += 1
            continue

        try:
            data = np.load(str(npz_path), allow_pickle=True)
        except Exception as e:
            print(f"  [WARN] Failed to load {clip_id}: {e}")
            stats["skipped"] += 1
            continue

        poses = data["poses"]          # (T, 165)
        trans = data["trans"]          # (T, 3)
        betas = data["betas"]          # (300,)

        # Validate shape
        if poses.ndim != 2 or poses.shape[1] != 165:
            print(f"  [WARN] Unexpected poses shape {poses.shape} in {clip_id}, skipping")
            stats["skipped"] += 1
            continue

        T = poses.shape[0]

        # ---- Extract upper-body ----
        upper_body = extract_upper_body(poses)  # (T, 42)
        assert upper_body.shape == (T, EXTRACTED_POSE_DIM), \
            f"Shape mismatch: {upper_body.shape} vs ({T}, {EXTRACTED_POSE_DIM})"

        # ---- Check betas consistency per speaker ----
        if speaker_id in speaker_betas:
            if not np.allclose(betas, speaker_betas[speaker_id], atol=1e-5):
                print(f"  [WARN] Betas differ for speaker {speaker_id} in {clip_id}")
        else:
            speaker_betas[speaker_id] = betas.copy()

        # ---- Save cleaned motion ----
        out_path = motion_out / f"{clip_id}.npz"
        np.savez_compressed(
            str(out_path),
            upper_body_poses=upper_body.astype(np.float32),  # (T, 42) axis-angle
            trans=trans.astype(np.float32),                   # (T, 3) root translation
            betas=betas.astype(np.float32),                   # (300,) body shape
            fps=np.int32(MOCAP_FPS),
            num_frames=np.int32(T),
            speaker_id=np.int32(speaker_id),
        )

        duration_map[clip_id] = T
        stats["processed"] += 1
        stats["total_frames"] += T

    # ---- Save speaker betas ----
    betas_out = output_dir / "speaker_betas"
    betas_out.mkdir(exist_ok=True)
    for sid, betas in speaker_betas.items():
        np.save(str(betas_out / f"speaker_{sid:02d}.npy"), betas)

    # ---- Save extraction report ----
    report = {
        "processed": stats["processed"],
        "skipped": stats["skipped"],
        "total_frames": stats["total_frames"],
        "total_duration_seconds": stats["total_frames"] / MOCAP_FPS,
        "total_duration_hours": stats["total_frames"] / MOCAP_FPS / 3600,
        "pose_dim": EXTRACTED_POSE_DIM,
        "joints": ["global_orient"] + UPPER_BODY_JOINT_NAMES,
        "fps": MOCAP_FPS,
        "num_speakers_with_betas": len(speaker_betas),
    }
    report_path = output_dir / "extraction_report.json"
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)

    # ---- Save duration map (needed for annotation alignment) ----
    duration_path = output_dir / "clip_durations.json"
    with open(duration_path, "w") as f:
        json.dump(duration_map, f, indent=2)

    print(f"\n{'='*60}")
    print(f"  EXTRACTION COMPLETE")
    print(f"{'='*60}")
    print(f"  Processed:     {stats['processed']} clips")
    print(f"  Skipped:       {stats['skipped']} clips")
    print(f"  Total frames:  {stats['total_frames']:,}")
    print(f"  Total hours:   {report['total_duration_hours']:.2f}")
    print(f"  Pose dim:      {EXTRACTED_POSE_DIM}")
    print(f"  Output:        {motion_out}")
    print(f"  Speaker betas: {betas_out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract upper-body motion from BEAT2")
    parser.add_argument("--output-dir", type=str, default=str(OUTPUT_DIR))
    parser.add_argument("--manifest", type=str, default=str(OUTPUT_DIR / "manifest.csv"))
    args = parser.parse_args()

    extract_all(Path(args.manifest), Path(args.output_dir))
