#!/usr/bin/env python3
"""Generate delta-transformer predictions for multiple BEAT2 clips."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from beat2_nao.config import AUDIO_DIR, SPLIT_CSV, TEXTGRID_DIR  # noqa: E402

try:
    from .inference import generate
except ImportError:
    from inference import generate


def read_clip_ids(args: argparse.Namespace) -> list[str]:
    clip_ids = list(args.clip_id or [])
    if args.clip_list:
        with open(args.clip_list, "r") as f:
            clip_ids.extend(line.strip() for line in f if line.strip() and not line.startswith("#"))
    if clip_ids:
        return list(dict.fromkeys(clip_ids))

    with open(args.split_csv, "r", newline="") as f:
        rows = list(csv.DictReader(f))
    rows = [row for row in rows if row.get("type") == args.split]
    ids = [row["id"] for row in rows]
    if args.max_clips is not None:
        ids = ids[: args.max_clips]
    return ids


def build_inference_args(args: argparse.Namespace, clip_id: str, output_path: Path) -> argparse.Namespace:
    return argparse.Namespace(
        checkpoint=args.checkpoint,
        stats=args.stats,
        output=str(output_path),
        wav=None,
        textgrid=None,
        clip_id=clip_id,
        audio_dir=args.audio_dir,
        textgrid_dir=args.textgrid_dir,
        fps=args.fps,
        window_size=args.window_size,
        stride=args.stride,
        seed=args.seed,
        smooth_window=args.smooth_window,
        velocity_limit=args.velocity_limit,
        velocity_scale=args.velocity_scale,
        text_cpu=args.text_cpu,
        wavlm_cpu=args.wavlm_cpu,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate delta-transformer predictions for multiple BEAT2 clips")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--stats", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--clip-id", action="append", help="Clip id to infer; repeat for multiple clips")
    parser.add_argument("--clip-list", help="Text file containing one clip id per line")
    parser.add_argument("--split-csv", default=str(SPLIT_CSV))
    parser.add_argument("--split", default="test")
    parser.add_argument("--max-clips", type=int, default=None)
    parser.add_argument("--audio-dir", default=str(AUDIO_DIR))
    parser.add_argument("--textgrid-dir", default=str(TEXTGRID_DIR))
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--smooth-window", type=int, default=1)
    parser.add_argument("--velocity-limit", action="store_true")
    parser.add_argument("--velocity-scale", type=float, default=1.0)
    parser.add_argument("--text-cpu", action="store_true")
    parser.add_argument("--wavlm-cpu", action="store_true")
    parser.add_argument("--continue-on-error", action="store_true")
    args = parser.parse_args()

    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    if args.window_size <= 0 or args.stride <= 0:
        raise ValueError("--window-size and --stride must be positive")
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    clip_ids = read_clip_ids(args)
    if not clip_ids:
        raise RuntimeError("No clip ids found for inference")

    print(f"[BATCH] Generating {len(clip_ids)} clips into {output_dir}")
    failures = []
    for idx, clip_id in enumerate(clip_ids, start=1):
        print("\n" + "=" * 70)
        print(f"[BATCH] {idx}/{len(clip_ids)} {clip_id}")
        print("=" * 70)
        try:
            generate(build_inference_args(args, clip_id, output_dir / f"{clip_id}.npy"))
        except Exception as exc:
            if not args.continue_on_error:
                raise
            print(f"[WARN] Failed {clip_id}: {exc}")
            failures.append({"clip_id": clip_id, "error": str(exc)})

    if failures:
        failure_path = output_dir / "failed_inferences.json"
        import json

        with open(failure_path, "w") as f:
            json.dump(failures, f, indent=2)
        print(f"[BATCH] Completed with {len(failures)} failures: {failure_path}")
    else:
        print("[BATCH] Completed without failures")


if __name__ == "__main__":
    main()
