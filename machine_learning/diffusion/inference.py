#!/usr/bin/env python3
"""Run BEAT2/NAO inference with a conditional diffusion checkpoint."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from beat2_nao.config import AUDIO_DIR, MOCAP_FPS, TEXTGRID_DIR  # noqa: E402
from beat2_nao import infer_nao  # noqa: E402


def _append_optional(command: list[str], flag: str, value) -> None:
    if value is not None:
        command.extend([flag, str(value)])


def build_infer_argv(args: argparse.Namespace) -> list[str]:
    command = [
        "beat2_nao/infer_nao.py",
        "--checkpoint",
        str(args.checkpoint),
        "--stats",
        str(args.stats),
        "--output",
        str(args.output),
        "--model-type",
        "diffusion",
        "--audio-dir",
        str(args.audio_dir),
        "--textgrid-dir",
        str(args.textgrid_dir),
        "--fps",
        str(args.fps),
        "--window-size",
        str(args.window_size),
        "--stride",
        str(args.stride),
        "--sampler",
        str(args.sampler),
        "--sample-steps",
        str(args.sample_steps),
        "--guidance-scale",
        str(args.guidance_scale),
        "--smooth-window",
        str(args.smooth_window),
        "--velocity-scale",
        str(args.velocity_scale),
    ]
    _append_optional(command, "--wav", args.wav)
    _append_optional(command, "--textgrid", args.textgrid)
    _append_optional(command, "--clip-id", args.clip_id)
    _append_optional(command, "--speaker-id", args.speaker_id)
    _append_optional(command, "--seed", args.seed)
    _append_optional(command, "--diffusion-seed-frames", args.diffusion_seed_frames)
    if args.diffusion_deterministic:
        command.append("--diffusion-deterministic")
    if args.velocity_limit:
        command.append("--velocity-limit")
    if args.text_cpu:
        command.append("--text-cpu")
    if args.wavlm_cpu:
        command.append("--wavlm-cpu")
    return command


def validate_pose_checkpoint(checkpoint_path: Path, stats_path: Path) -> None:
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    if not isinstance(checkpoint, dict):
        raise ValueError("Diffusion inference requires a checkpoint with model_config and diffusion_config")
    if "diffusion_config" not in checkpoint:
        raise ValueError("Diffusion inference requires a checkpoint containing diffusion_config")

    with open(stats_path, "r") as f:
        stats = json.load(f)
    sources = [
        ("checkpoint model_config", checkpoint.get("model_config") or {}),
        ("checkpoint dataset_metadata", checkpoint.get("dataset_metadata") or {}),
        ("stats", stats),
    ]
    for name, metadata in sources:
        target_mode = metadata.get("target_mode", "angle")
        target_type = str(metadata.get("target_type", ""))
        if target_mode != "angle" or "delta" in target_type:
            raise ValueError(
                f"{name} uses target_mode={target_mode!r}, target_type={target_type!r}. "
                "Diffusion inference expects a model trained directly on normalized poses, not deltas."
            )


def generate(args: argparse.Namespace) -> np.ndarray:
    """Generate a diffusion NAO angle sequence and return the saved array."""
    validate_pose_checkpoint(Path(args.checkpoint), Path(args.stats))
    old_argv = sys.argv
    try:
        sys.argv = build_infer_argv(args)
        infer_nao.main()
    finally:
        sys.argv = old_argv
    return np.load(str(args.output)).astype(np.float32)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate BEAT2 NAO gestures with conditional diffusion")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--stats", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--wav", default=None)
    parser.add_argument("--textgrid", default=None)
    parser.add_argument("--clip-id", default=None)
    parser.add_argument("--speaker-id", default=None)
    parser.add_argument("--audio-dir", default=str(AUDIO_DIR))
    parser.add_argument("--textgrid-dir", default=str(TEXTGRID_DIR))
    parser.add_argument("--fps", type=int, default=MOCAP_FPS)
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    parser.add_argument("--sampler", choices=["ddpm", "ddim"], default="ddpm")
    parser.add_argument("--sample-steps", type=int, default=50)
    parser.add_argument("--guidance-scale", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--diffusion-deterministic", action="store_true")
    parser.add_argument("--diffusion-seed-frames", type=int, default=None)
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
    if args.sample_steps <= 0:
        raise ValueError("--sample-steps must be positive")
    if args.guidance_scale < 0:
        raise ValueError("--guidance-scale cannot be negative")
    if args.diffusion_seed_frames is not None and args.diffusion_seed_frames < 0:
        raise ValueError("--diffusion-seed-frames cannot be negative")
    if args.smooth_window < 1:
        raise ValueError("--smooth-window must be at least 1")
    if args.velocity_scale <= 0:
        raise ValueError("--velocity-scale must be positive")
    generate(args)


if __name__ == "__main__":
    main()
