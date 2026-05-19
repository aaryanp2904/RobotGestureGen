#!/usr/bin/env python3
"""
Step 3 — Parse TextGrid and semantic annotation files into unified JSON.

For each validated clip, produces a JSON file containing:
  - Word-level timing (from .TextGrid "words" tier)
  - Phoneme-level timing (from .TextGrid "phones" tier)
  - Semantic gesture labels (from .txt sem files)
  - Speaker / clip metadata

These JSONs are the canonical annotation format for downstream preprocessing.

Usage:
    python parse_annotations.py [--output-dir /path/to/output] [--manifest /path/to/manifest.csv]
"""

import csv
import re
import json
import argparse
from pathlib import Path
from tqdm import tqdm

from config import TEXTGRID_DIR, SEM_DIR, OUTPUT_DIR


# ---------------------------------------------------------------------------
# TextGrid parser (no external dependency required)
# ---------------------------------------------------------------------------

def parse_textgrid(filepath: Path) -> dict:
    """
    Parse a Praat TextGrid file (standard long format) into a dict of tiers.

    Returns:
        {
            "words": [{"start": float, "end": float, "text": str}, ...],
            "phones": [{"start": float, "end": float, "text": str}, ...],
        }
    """
    with open(filepath, "r", encoding="utf-8") as f:
        content = f.read()

    tiers = {}
    # Split into tier blocks
    tier_blocks = re.split(r'item\s*\[\d+\]\s*:', content)

    for block in tier_blocks[1:]:  # skip header
        # Extract tier name
        name_match = re.search(r'name\s*=\s*"([^"]*)"', block)
        if not name_match:
            continue
        tier_name = name_match.group(1)

        # Extract intervals
        intervals = []
        interval_blocks = re.split(r'intervals\s*\[\d+\]\s*:', block)

        for ib in interval_blocks[1:]:  # skip tier header
            xmin_m = re.search(r'xmin\s*=\s*([\d.]+)', ib)
            xmax_m = re.search(r'xmax\s*=\s*([\d.]+)', ib)
            text_m = re.search(r'text\s*=\s*"([^"]*)"', ib)

            if xmin_m and xmax_m and text_m:
                text = text_m.group(1).strip()
                if text:  # skip empty intervals (silence)
                    intervals.append({
                        "start": float(xmin_m.group(1)),
                        "end": float(xmax_m.group(1)),
                        "text": text,
                    })

        tiers[tier_name] = intervals

    return tiers


# ---------------------------------------------------------------------------
# Semantic annotation parser
# ---------------------------------------------------------------------------

def parse_sem_file(filepath: Path) -> list:
    """
    Parse a BEAT2 semantic annotation .txt file.

    Format (tab-separated):
        gesture_type  start_time  end_time  duration  weight  [keyword]

    Returns:
        [{"type": str, "start": float, "end": float,
          "duration": float, "weight": float, "keyword": str|None}, ...]
    """
    annotations = []
    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split("\t")
            if len(parts) < 5:
                continue
            try:
                entry = {
                    "type": parts[0],
                    "start": float(parts[1]),
                    "end": float(parts[2]),
                    "duration": float(parts[3]),
                    "weight": float(parts[4]),
                }
                if len(parts) >= 6 and parts[5].strip():
                    entry["keyword"] = parts[5].strip()
                else:
                    entry["keyword"] = None
                annotations.append(entry)
            except (ValueError, IndexError):
                continue

    return annotations


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_all(manifest_path: Path, output_dir: Path):
    annotations_out = output_dir / "annotations"
    annotations_out.mkdir(parents=True, exist_ok=True)

    # Load manifest
    with open(manifest_path, "r") as f:
        clips = list(csv.DictReader(f))
    print(f"[ANNOTATIONS] Processing {len(clips)} clips")

    stats = {
        "processed": 0, "textgrid_failed": 0, "sem_failed": 0,
        "total_words": 0, "total_phones": 0, "total_gestures": 0,
    }

    for clip in tqdm(clips, desc="Parsing annotations"):
        clip_id = clip["id"]
        tg_path = TEXTGRID_DIR / f"{clip_id}.TextGrid"
        sem_path = SEM_DIR / f"{clip_id}.txt"

        result = {
            "id": clip_id,
            "speaker_id": int(clip["speaker_id"]),
            "speaker_name": clip["speaker_name"],
            "split": clip["split"],
        }

        # Parse TextGrid
        if tg_path.is_file():
            try:
                tiers = parse_textgrid(tg_path)
                result["words"] = tiers.get("words", [])
                result["phones"] = tiers.get("phones", [])
                stats["total_words"] += len(result["words"])
                stats["total_phones"] += len(result["phones"])
            except Exception as e:
                print(f"  [WARN] TextGrid parse failed for {clip_id}: {e}")
                result["words"] = []
                result["phones"] = []
                stats["textgrid_failed"] += 1
        else:
            result["words"] = []
            result["phones"] = []

        # Parse semantic annotations
        if sem_path.is_file():
            try:
                result["gestures"] = parse_sem_file(sem_path)
                stats["total_gestures"] += len(result["gestures"])
            except Exception as e:
                print(f"  [WARN] Sem parse failed for {clip_id}: {e}")
                result["gestures"] = []
                stats["sem_failed"] += 1
        else:
            result["gestures"] = []

        # Write per-clip JSON
        out_path = annotations_out / f"{clip_id}.json"
        with open(out_path, "w") as f:
            json.dump(result, f, indent=2)

        stats["processed"] += 1

    # ---- Summary ----
    report = {
        "processed": stats["processed"],
        "textgrid_failures": stats["textgrid_failed"],
        "sem_failures": stats["sem_failed"],
        "total_words": stats["total_words"],
        "total_phones": stats["total_phones"],
        "total_gesture_annotations": stats["total_gestures"],
    }
    report_path = output_dir / "annotation_report.json"
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)

    print(f"\n{'='*60}")
    print(f"  ANNOTATION PARSING COMPLETE")
    print(f"{'='*60}")
    print(f"  Processed:      {stats['processed']} clips")
    print(f"  Total words:    {stats['total_words']:,}")
    print(f"  Total phones:   {stats['total_phones']:,}")
    print(f"  Total gestures: {stats['total_gestures']:,}")
    print(f"  TG failures:    {stats['textgrid_failed']}")
    print(f"  Sem failures:   {stats['sem_failed']}")
    print(f"  Output:         {annotations_out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parse BEAT2 TextGrid + semantic annotations")
    parser.add_argument("--output-dir", type=str, default=str(OUTPUT_DIR))
    parser.add_argument("--manifest", type=str, default=str(OUTPUT_DIR / "manifest.csv"))
    args = parser.parse_args()

    parse_all(Path(args.manifest), Path(args.output_dir))
