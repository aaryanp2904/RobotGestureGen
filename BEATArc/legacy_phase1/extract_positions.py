#!/usr/bin/env python3
"""
Step 4 (optional) — Convert axis-angle upper-body rotations to world-space XYZ
joint positions using SMPL-X forward kinematics.

This is needed to produce training data in the same representation as the GENEA
pipeline (root-relative 3D positions per joint per frame).

If the `smplx` package and body model are unavailable, this step can be skipped
and the model can be trained directly on axis-angle rotations instead.

Prerequisites:
    pip install smplx
    Download SMPL-X body model from https://smpl-x.is.tue.mpg.de/
    Set SMPLX_MODEL_DIR below to point to the extracted model directory.

Usage:
    python extract_positions.py --model-dir /path/to/smplx/models \
                                [--output-dir /path/to/output] \
                                [--manifest /path/to/manifest.csv]
"""

import csv
import json
import argparse
import numpy as np
from pathlib import Path
from tqdm import tqdm

from config import (
    MOTION_DIR, OUTPUT_DIR, MOCAP_FPS,
    UPPER_BODY_JOINT_INDICES, UPPER_BODY_JOINT_NAMES,
)

# SMPL-X body joint indices (in the full 55-joint SMPL-X skeleton) that
# correspond to our upper-body selection. The global orient contributes
# the root (pelvis) which we use for root subtraction.
#
# In SMPL-X, joint 0 = pelvis, then joints 1..21 = body, 22..24 = jaw+eyes,
# 25..39 = left hand, 40..54 = right hand.
#
# Our UPPER_BODY_JOINT_INDICES are body-block-relative (0..20), so the full
# skeleton indices are offset by +1 (since body block starts at joint 1).
SMPLX_FULL_UPPER_BODY_INDICES = [idx + 1 for idx in UPPER_BODY_JOINT_INDICES]


def extract_positions_smplx(manifest_path: Path, output_dir: Path, model_dir: Path):
    """Convert axis-angle rotations to world-space XYZ using the smplx package."""
    try:
        import smplx
        import torch
    except ImportError:
        print("[ERROR] 'smplx' and 'torch' packages are required.")
        print("        Install with: pip install smplx torch")
        return

    positions_out = output_dir / "positions"
    positions_out.mkdir(parents=True, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[FK] Device: {device}")

    # Load manifest
    with open(manifest_path, "r") as f:
        clips = list(csv.DictReader(f))
    print(f"[FK] Processing {len(clips)} clips")

    # Cache models per speaker (different betas → different joint locations)
    speaker_models = {}
    betas_dir = output_dir / "speaker_betas"

    stats = {"processed": 0, "skipped": 0, "total_frames": 0}

    for clip in tqdm(clips, desc="Computing FK positions"):
        clip_id = clip["id"]
        speaker_id = int(clip["speaker_id"])

        npz_path = MOTION_DIR / f"{clip_id}.npz"
        if not npz_path.is_file():
            stats["skipped"] += 1
            continue

        try:
            data = np.load(str(npz_path), allow_pickle=True)
        except Exception:
            stats["skipped"] += 1
            continue

        poses = data["poses"]             # (T, 165)
        trans = data["trans"]             # (T, 3)
        betas = data["betas"]             # (300,)
        expressions = data["expressions"] # (T, 100)
        T = poses.shape[0]

        # Get or create SMPL-X model for this speaker
        if speaker_id not in speaker_models:
            # Use per-speaker betas if available, otherwise from the clip
            betas_path = betas_dir / f"speaker_{speaker_id:02d}.npy"
            if betas_path.is_file():
                speaker_betas = np.load(str(betas_path))
            else:
                speaker_betas = betas

            # SMPL-X models typically support 10 betas; the dataset stores 300
            # but only the first 10 are meaningful for most body models.
            NUM_BETAS = 10
            NUM_EXPR = expressions.shape[1] if expressions.ndim == 2 else 10
            model = smplx.create(
                str(model_dir),
                model_type="smplx",
                gender="neutral",
                num_betas=NUM_BETAS,
                num_expression_coeffs=NUM_EXPR,
                use_pca=False,
                batch_size=1,
            ).to(device)
            speaker_models[speaker_id] = (model, speaker_betas[:NUM_BETAS])

        model, speaker_betas = speaker_models[speaker_id]

        # Process in batches to avoid OOM
        batch_size = 256
        all_positions = []

        for start in range(0, T, batch_size):
            end = min(start + batch_size, T)
            B = end - start

            # Prepare inputs — must pass ALL pose components with matching
            # batch size, otherwise the model's internal defaults (batch=1)
            # clash with our B-sized body_pose during concatenation.
            betas_t = torch.tensor(
                np.tile(speaker_betas[None, :], (B, 1)),  # (B, NUM_BETAS)
                dtype=torch.float32, device=device,
            )
            global_orient_t = torch.tensor(
                poses[start:end, :3], dtype=torch.float32, device=device,
            )
            body_pose_t = torch.tensor(
                poses[start:end, 3:66], dtype=torch.float32, device=device,
            )
            left_hand_t = torch.tensor(
                poses[start:end, 66:111], dtype=torch.float32, device=device,
            )
            right_hand_t = torch.tensor(
                poses[start:end, 111:156], dtype=torch.float32, device=device,
            )
            # jaw (3) + left_eye (3) + right_eye (3) = 9 dims
            jaw_t = torch.tensor(
                poses[start:end, 156:159], dtype=torch.float32, device=device,
            )
            leye_t = torch.tensor(
                poses[start:end, 159:162], dtype=torch.float32, device=device,
            )
            reye_t = torch.tensor(
                poses[start:end, 162:165], dtype=torch.float32, device=device,
            )
            transl_t = torch.tensor(
                trans[start:end], dtype=torch.float32, device=device,
            )
            expression_t = torch.tensor(
                expressions[start:end], dtype=torch.float32, device=device,
            )

            with torch.no_grad():
                output = model(
                    betas=betas_t,
                    global_orient=global_orient_t,
                    body_pose=body_pose_t,
                    left_hand_pose=left_hand_t,
                    right_hand_pose=right_hand_t,
                    jaw_pose=jaw_t,
                    leye_pose=leye_t,
                    reye_pose=reye_t,
                    transl=transl_t,
                    expression=expression_t,
                    return_verts=False,
                )

            # output.joints: (B, num_joints, 3)
            # Extract pelvis (joint 0) and our upper body joints
            joints = output.joints.cpu().numpy()
            pelvis = joints[:, 0:1, :]  # (B, 1, 3)
            upper = joints[:, SMPLX_FULL_UPPER_BODY_INDICES, :]  # (B, 13, 3)

            # Root-relative positions
            upper_relative = upper - pelvis  # (B, 13, 3)
            all_positions.append(upper_relative)

        positions = np.concatenate(all_positions, axis=0)  # (T, 13, 3)

        # Save
        out_path = positions_out / f"{clip_id}.npz"
        np.savez_compressed(
            str(out_path),
            positions=positions.astype(np.float32),       # (T, 13, 3) root-relative
            root_trans=trans.astype(np.float32),           # (T, 3) global root
            fps=np.int32(MOCAP_FPS),
            num_frames=np.int32(T),
            speaker_id=np.int32(speaker_id),
        )

        stats["processed"] += 1
        stats["total_frames"] += T

    # Report
    report = {
        "processed": stats["processed"],
        "skipped": stats["skipped"],
        "total_frames": stats["total_frames"],
        "total_duration_hours": stats["total_frames"] / MOCAP_FPS / 3600,
        "output_shape_per_frame": [len(UPPER_BODY_JOINT_INDICES), 3],
        "joints": UPPER_BODY_JOINT_NAMES,
        "representation": "root_relative_xyz",
    }
    with open(output_dir / "positions_report.json", "w") as f:
        json.dump(report, f, indent=2)

    print(f"\n{'='*60}")
    print(f"  FK POSITION EXTRACTION COMPLETE")
    print(f"{'='*60}")
    print(f"  Processed:    {stats['processed']} clips")
    print(f"  Total frames: {stats['total_frames']:,}")
    print(f"  Output shape: (T, {len(UPPER_BODY_JOINT_INDICES)}, 3)")
    print(f"  Output:       {positions_out}")


# ---------------------------------------------------------------------------
# Lightweight FK fallback (no smplx package needed)
# ---------------------------------------------------------------------------

# SMPL-X kinematic tree (body joints only, parent index in body block)
# This is a simplified version that only computes joint positions from
# axis-angle rotations using the standard kinematic chain.
SMPLX_BODY_PARENT = {
    # body_idx: parent_body_idx  (-1 = pelvis/root)
    0: -1,   # left_hip → pelvis
    1: -1,   # right_hip → pelvis
    2: -1,   # spine1 → pelvis
    3:  0,   # left_knee → left_hip
    4:  1,   # right_knee → right_hip
    5:  2,   # spine2 → spine1
    6:  3,   # left_ankle → left_knee
    7:  4,   # right_ankle → right_knee
    8:  5,   # spine3 → spine2
    9:  6,   # left_foot → left_ankle
    10: 7,   # right_foot → right_ankle
    11: 8,   # neck → spine3
    12: 8,   # left_collar → spine3
    13: 8,   # right_collar → spine3
    14: 11,  # head → neck
    15: 12,  # left_shoulder → left_collar
    16: 13,  # right_shoulder → right_collar
    17: 15,  # left_elbow → left_shoulder
    18: 16,  # right_elbow → right_shoulder
    19: 17,  # left_wrist → left_elbow
    20: 18,  # right_wrist → right_elbow
}


def axis_angle_to_matrix(aa: np.ndarray) -> np.ndarray:
    """Convert axis-angle (3,) to rotation matrix (3, 3) using Rodrigues."""
    angle = np.linalg.norm(aa)
    if angle < 1e-8:
        return np.eye(3, dtype=aa.dtype)
    k = aa / angle
    K = np.array([
        [0, -k[2], k[1]],
        [k[2], 0, -k[0]],
        [-k[1], k[0], 0]
    ], dtype=aa.dtype)
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def simple_fk(poses_frame: np.ndarray) -> np.ndarray:
    """
    Simple FK for a single frame — computes body joint positions from
    axis-angle rotations using a T-pose with unit bone lengths.

    This is a ROUGH approximation — for accurate positions, use the full
    smplx model. But it preserves the kinematic structure and relative
    motion patterns, which may be sufficient for gesture generation.

    Args:
        poses_frame: (165,) — one frame of SMPL-X poses

    Returns:
        (22, 3) — approximate positions for pelvis + 21 body joints
    """
    # Unit bone length offsets in T-pose (approximate human proportions)
    # These are rough estimates; the real offsets come from the SMPL-X model + betas
    T_POSE_OFFSETS = {
        -1: np.array([0, 0, 0]),          # pelvis is origin
        0:  np.array([0.09, -0.04, 0]),    # left_hip
        1:  np.array([-0.09, -0.04, 0]),   # right_hip
        2:  np.array([0, 0.07, 0]),        # spine1
        3:  np.array([0, -0.40, 0]),       # left_knee
        4:  np.array([0, -0.40, 0]),       # right_knee
        5:  np.array([0, 0.12, 0]),        # spine2
        6:  np.array([0, -0.40, 0]),       # left_ankle
        7:  np.array([0, -0.40, 0]),       # right_ankle
        8:  np.array([0, 0.12, 0]),        # spine3
        9:  np.array([0, -0.05, 0.10]),    # left_foot
        10: np.array([0, -0.05, 0.10]),    # right_foot
        11: np.array([0, 0.12, 0]),        # neck
        12: np.array([0.06, 0.06, 0]),     # left_collar
        13: np.array([-0.06, 0.06, 0]),    # right_collar
        14: np.array([0, 0.10, 0]),        # head
        15: np.array([0.12, 0, 0]),        # left_shoulder
        16: np.array([-0.12, 0, 0]),       # right_shoulder
        17: np.array([0.25, 0, 0]),        # left_elbow
        18: np.array([-0.25, 0, 0]),       # right_elbow
        19: np.array([0.25, 0, 0]),        # left_wrist
        20: np.array([-0.25, 0, 0]),       # right_wrist
    }

    # Global orientation (pelvis rotation)
    R_global = axis_angle_to_matrix(poses_frame[0:3])

    # Compute world-space rotation and position for each body joint
    world_R = {-1: R_global}
    world_pos = {-1: np.zeros(3)}

    for idx in range(21):
        parent = SMPLX_BODY_PARENT[idx]
        local_R = axis_angle_to_matrix(poses_frame[3 + idx * 3: 3 + (idx + 1) * 3])
        offset = T_POSE_OFFSETS[idx]

        world_R[idx] = world_R[parent] @ local_R
        world_pos[idx] = world_pos[parent] + world_R[parent] @ offset

    # Return pelvis + 21 body joints
    positions = np.zeros((22, 3))
    positions[0] = world_pos[-1]  # pelvis
    for idx in range(21):
        positions[idx + 1] = world_pos[idx]

    return positions


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert SMPL-X axis-angle to world-space XYZ positions"
    )
    parser.add_argument("--model-dir", type=str, required=True,
                        help="Path to SMPL-X model directory (contains smplx/ folder)")
    parser.add_argument("--output-dir", type=str, default=str(OUTPUT_DIR))
    parser.add_argument("--manifest", type=str, default=str(OUTPUT_DIR / "manifest.csv"))
    args = parser.parse_args()

    extract_positions_smplx(Path(args.manifest), Path(args.output_dir), Path(args.model_dir))
