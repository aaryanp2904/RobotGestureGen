"""
Batch inference: generate gesture predictions for all validation datapoints.

Usage (run from the transformers directory on GPU node):
    python batch_generate.py \
        --model gesture_transformer_full_trained.pth \
        --stats /vol/bitbucket/ap1922/PreprocessedGeneaDataset/normalization_stats.json \
        --val-dir /vol/bitbucket/ap1922/Genea2022/val \
        --output-dir /vol/bitbucket/ap1922/PreprocessedGeneaDataset/val_predictions
"""

import argparse
import time
from pathlib import Path
from generate_gestures import GestureGenerator


def main():
    parser = argparse.ArgumentParser(description="Batch generate gestures for validation set")
    parser.add_argument("--model", type=str, required=True)
    parser.add_argument("--stats", type=str, required=True)
    parser.add_argument("--val-dir", type=str, required=True,
                        help="Root of val set (contains wav/, tsv/ subdirs)")
    parser.add_argument("--output-dir", type=str, required=True)
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    args = parser.parse_args()

    val_dir = Path(args.val_dir)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    wav_dir = val_dir / "wav"
    tsv_dir = val_dir / "tsv"

    # Find all WAV files and match with TSV
    wav_files = sorted(wav_dir.glob("*.wav"))
    print(f"[BATCH] Found {len(wav_files)} WAV files in {wav_dir}")

    pairs = []
    for wav_path in wav_files:
        stem = wav_path.stem
        tsv_path = tsv_dir / f"{stem}.tsv"
        if tsv_path.exists():
            pairs.append((stem, str(wav_path), str(tsv_path)))
        else:
            print(f"[BATCH] WARNING: No TSV for {stem}, skipping")

    print(f"[BATCH] {len(pairs)} complete WAV+TSV pairs\n")

    if not pairs:
        print("[BATCH] No pairs found. Check directory structure.")
        return

    # Load model + feature extractors once
    gen = GestureGenerator(args.model, args.stats)

    total_start = time.time()
    for i, (stem, wav_path, tsv_path) in enumerate(pairs):
        print(f"\n{'='*60}")
        print(f"[BATCH] [{i+1}/{len(pairs)}] {stem}")
        print(f"{'='*60}")

        t0 = time.time()
        preds = gen.generate_from_raw(
            wav_path, tsv_path,
            window_sec=args.window_size, stride_sec=args.stride,
        )

        out_path = out_dir / f"{stem}.npy"
        gen.save_predictions(preds, str(out_path))

        elapsed = time.time() - t0
        print(f"[BATCH] {stem} done in {elapsed:.1f}s — {preds.shape}")

    total_elapsed = time.time() - total_start
    print(f"\n{'='*60}")
    print(f"[BATCH] ✓ All {len(pairs)} files processed in {total_elapsed:.1f}s")
    print(f"[BATCH] Predictions saved to {out_dir}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
