#!/usr/bin/env python3
"""
Step 1 — Validate the BEAT2 dataset and produce a clean manifest.

Checks that every clip ID in train_test_split.csv has all required files
(.npz, .wav, .TextGrid, .txt), flags missing files, and writes a validated
manifest CSV with per-clip metadata (speaker_id, speaker_name, emotion,
split, duration_frames).

Usage:
    python validate_dataset.py [--output-dir /path/to/output]
"""

import csv
import json
import argparse
import numpy as np
from pathlib import Path
from typing import Optional
from collections import defaultdict

from config import (
    BEAT2_EN, MOTION_DIR, AUDIO_DIR, SEM_DIR, TEXTGRID_DIR,
    SPLIT_CSV, OUTPUT_DIR, KNOWN_MISSING, SPEAKER_LUT, MOCAP_FPS,
)


def parse_clip_id(clip_id: str) -> dict:
    """
    Parse a BEAT2 clip ID like '10_kieks_0_103_103' into components.
    Format: {speaker_id}_{speaker_name}_{emotion}_{start}_{end}
    """
    parts = clip_id.split("_")
    speaker_id = int(parts[0])
    speaker_name = parts[1]
    emotion = int(parts[2])
    return {
        "speaker_id": speaker_id,
        "speaker_name": speaker_name,
        "emotion": emotion,
    }


def check_file_exists(directory: Path, clip_id: str, extension: str) -> bool:
    """Check if a file with the given clip ID and extension exists."""
    return (directory / f"{clip_id}{extension}").is_file()


def get_motion_duration(clip_id: str) -> Optional[int]:
    """Load the NPZ and return number of motion frames, or None if missing/corrupt."""
    npz_path = MOTION_DIR / f"{clip_id}.npz"
    if not npz_path.is_file():
        return None
    try:
        data = np.load(str(npz_path), allow_pickle=True)
        return data["poses"].shape[0]
    except Exception:
        return None


def validate(output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # 1. Load the split CSV
    # ------------------------------------------------------------------
    split_map = {}  # clip_id → split_type
    with open(SPLIT_CSV, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            split_map[row["id"]] = row["type"]

    print(f"[VALIDATE] Loaded {len(split_map)} clip IDs from train_test_split.csv")
    split_counts = defaultdict(int)
    for s in split_map.values():
        split_counts[s] += 1
    for k, v in sorted(split_counts.items()):
        print(f"           {k}: {v}")

    # ------------------------------------------------------------------
    # 2. Also discover files on disk not listed in the CSV
    # ------------------------------------------------------------------
    npz_on_disk = {p.stem for p in MOTION_DIR.glob("*.npz")}
    wav_on_disk = {p.stem for p in AUDIO_DIR.glob("*.wav")}
    sem_on_disk = {p.stem for p in SEM_DIR.glob("*.txt")}
    tg_on_disk  = {p.stem for p in TEXTGRID_DIR.glob("*.TextGrid")}

    csv_ids = set(split_map.keys())

    orphan_npz = npz_on_disk - csv_ids
    orphan_wav = wav_on_disk - csv_ids
    orphan_sem = sem_on_disk - csv_ids
    orphan_tg  = tg_on_disk  - csv_ids

    if orphan_npz:
        print(f"\n[WARN] {len(orphan_npz)} .npz files on disk but NOT in CSV (will be ignored)")
    if orphan_wav:
        print(f"[WARN] {len(orphan_wav)} .wav files on disk but NOT in CSV")
    if orphan_sem:
        print(f"[WARN] {len(orphan_sem)} .txt sem files on disk but NOT in CSV")

    # ------------------------------------------------------------------
    # 3. Validate each clip ID
    # ------------------------------------------------------------------
    valid_clips = []
    dropped = {"missing_npz": [], "missing_wav": [], "missing_textgrid": [],
               "missing_sem": [], "corrupt_npz": [], "known_missing": []}

    for clip_id, split_type in sorted(split_map.items()):
        # Check known missing first
        if clip_id in KNOWN_MISSING:
            dropped["known_missing"].append(clip_id)
            continue

        has_npz = check_file_exists(MOTION_DIR, clip_id, ".npz")
        has_wav = check_file_exists(AUDIO_DIR, clip_id, ".wav")
        has_tg  = check_file_exists(TEXTGRID_DIR, clip_id, ".TextGrid")
        has_sem = check_file_exists(SEM_DIR, clip_id, ".txt")

        if not has_npz:
            dropped["missing_npz"].append(clip_id)
            continue
        if not has_wav:
            dropped["missing_wav"].append(clip_id)
            continue
        if not has_tg:
            dropped["missing_textgrid"].append(clip_id)
            continue
        if not has_sem:
            dropped["missing_sem"].append(clip_id)
            continue

        # Parse metadata from filename
        meta = parse_clip_id(clip_id)

        valid_clips.append({
            "id": clip_id,
            "split": split_type,
            "speaker_id": meta["speaker_id"],
            "speaker_name": meta["speaker_name"],
            "emotion": meta["emotion"],
        })

    # ------------------------------------------------------------------
    # 4. Report
    # ------------------------------------------------------------------
    print(f"\n{'='*60}")
    print(f"  VALIDATION RESULTS")
    print(f"{'='*60}")
    print(f"  Total in CSV:        {len(split_map)}")
    print(f"  Valid (all files):    {len(valid_clips)}")
    total_dropped = sum(len(v) for v in dropped.values())
    print(f"  Dropped:             {total_dropped}")
    for reason, ids in dropped.items():
        if ids:
            print(f"    {reason}: {len(ids)}")
            for cid in ids[:5]:
                print(f"      - {cid}")
            if len(ids) > 5:
                print(f"      ... and {len(ids)-5} more")

    # Breakdown of valid clips by split
    valid_splits = defaultdict(int)
    for c in valid_clips:
        valid_splits[c["split"]] += 1
    print(f"\n  Valid clips by split:")
    for k, v in sorted(valid_splits.items()):
        print(f"    {k}: {v}")

    # Speaker distribution
    speaker_counts = defaultdict(int)
    for c in valid_clips:
        speaker_counts[c["speaker_id"]] += 1
    print(f"\n  Speakers: {len(speaker_counts)}")
    for sid in sorted(speaker_counts.keys()):
        name = SPEAKER_LUT.get(sid, "???")
        print(f"    {sid:3d} ({name:12s}): {speaker_counts[sid]} clips")

    # ------------------------------------------------------------------
    # 5. Write outputs
    # ------------------------------------------------------------------
    # Clean manifest CSV
    manifest_path = output_dir / "manifest.csv"
    with open(manifest_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "id", "split", "speaker_id", "speaker_name", "emotion",
        ])
        writer.writeheader()
        writer.writerows(valid_clips)
    print(f"\n[OUTPUT] Manifest written: {manifest_path} ({len(valid_clips)} clips)")

    # Dropped clips log
    dropped_path = output_dir / "dropped_clips.json"
    with open(dropped_path, "w") as f:
        json.dump(dropped, f, indent=2)
    print(f"[OUTPUT] Dropped clips log: {dropped_path}")

    # Summary JSON
    summary = {
        "total_in_csv": len(split_map),
        "valid_clips": len(valid_clips),
        "dropped_total": total_dropped,
        "dropped_breakdown": {k: len(v) for k, v in dropped.items()},
        "valid_by_split": dict(valid_splits),
        "num_speakers": len(speaker_counts),
    }
    summary_path = output_dir / "validation_summary.json"
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"[OUTPUT] Summary: {summary_path}")

    return valid_clips


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Validate BEAT2 dataset files")
    parser.add_argument("--output-dir", type=str, default=str(OUTPUT_DIR),
                        help="Directory to write manifest and reports")
    args = parser.parse_args()

    validate(Path(args.output_dir))
