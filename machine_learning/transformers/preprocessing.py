#!/usr/bin/env python3
"""Preprocess BEAT2 for the transformer baseline without energy outputs.

The transformer baseline uses the same BEAT2/NAO contract as the main
preprocessor, but its conditioning is intentionally limited to:

    prosody + WavLM (+ optional text)

Gesture-energy features are not computed or written by this script.
"""

from __future__ import annotations

import argparse
import csv
import json
import pickle
import shutil
import sys
from pathlib import Path

import numpy as np
import torch
from tqdm import tqdm

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from BEATArc.config import AUDIO_DIR, MOCAP_FPS, MOTION_DIR, SPLIT_CSV, TEXTGRID_DIR  # noqa: E402
from BEATArc.nao_constants import NAO_JOINTS  # noqa: E402
from BEATArc.parse_annotations import parse_textgrid  # noqa: E402
from BEATArc.preprocess_nao import (  # noqa: E402
    DEFAULT_WAVLM_MODEL,
    PROSODY_FEATURE_NAMES,
    TEXT_EMBED_DIM,
    BatchedStats,
    build_speaker_id_map,
    cached_wavlm_features,
    extract_prosody,
    extract_text_features,
    fixed_length_valid_mask,
    fixed_length_window,
    frame_velocity,
    init_text_models,
    init_wavlm_model,
    poses_to_nao_angles,
    read_split_csv,
    save_stats,
    select_training_target,
    speaker_id_from_clip_id,
    window_starts,
)

DEFAULT_OUTPUT_DIR = "/vol/bitbucket/ap1922/BEAT2_NAO_Transformer_Preprocessed"


def load_words(clip_id: str, textgrid_dir: Path) -> list[dict]:
    textgrid_path = textgrid_dir / f"{clip_id}.TextGrid"
    if not textgrid_path.is_file():
        return []
    try:
        return parse_textgrid(textgrid_path).get("words", [])
    except Exception as exc:
        print(f"[WARN] TextGrid parse failed for {clip_id}: {exc}")
        return []


def write_manifest(path: Path, rows: list[dict]) -> None:
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["id", "split", "num_frames", "duration_sec", "fps", "speaker_id"],
        )
        writer.writeheader()
        writer.writerows(rows)


def write_transformer_lmdb(
    output_dir: Path,
    manifest_rows: list[dict],
    stats: dict,
    window_frames: int,
    stride_frames: int,
    map_size_gb: int,
    text_dim: int,
    wavlm_dim: int,
    target_mode: str,
    speaker_id_map: dict[str, int],
    window_seconds: float,
    stride_seconds: float,
) -> int:
    try:
        import lmdb
    except ImportError:
        print("[WARN] lmdb is not installed; skipping LMDB creation.")
        return 0

    clips_dir = output_dir / "clips"
    prosody_mean = np.asarray(stats["prosody_mean"], dtype=np.float32)
    prosody_std = np.asarray(stats["prosody_std"], dtype=np.float32)
    wavlm_mean = np.asarray(stats.get("wavlm_mean", []), dtype=np.float32)
    wavlm_std = np.asarray(stats.get("wavlm_std", []), dtype=np.float32)
    vel_mean = np.asarray(stats["nao_vel_mean"], dtype=np.float32)
    vel_std = np.asarray(stats["nao_vel_std"], dtype=np.float32)
    speaker_dim = len(speaker_id_map)

    rows_by_split: dict[str, list[dict]] = {}
    for row in manifest_rows:
        rows_by_split.setdefault(row["split"], []).append(row)

    total_windows = 0
    lmdb_index: dict[str, dict] = {}
    for split, rows in sorted(rows_by_split.items()):
        lmdb_path = output_dir / f"{split}.lmdb"
        if lmdb_path.exists():
            if lmdb_path.is_dir():
                shutil.rmtree(lmdb_path)
            else:
                lmdb_path.unlink()

        env = lmdb.open(str(lmdb_path), map_size=map_size_gb * 1024**3)
        txn = env.begin(write=True)
        global_idx = 0
        file_window_map = {}

        for row in tqdm(rows, desc=f"Writing {split}.lmdb windows"):
            clip_id = row["id"]
            with np.load(str(clips_dir / f"{clip_id}.npz"), allow_pickle=True) as data:
                prosody = (data["prosody"].astype(np.float32) - prosody_mean) / prosody_std
                if wavlm_dim:
                    wavlm = (
                        data["wavlm_features"].astype(np.float32) - wavlm_mean
                    ) / wavlm_std
                else:
                    wavlm = np.zeros((len(prosody), 0), dtype=np.float32)
                text = data["text_features"].astype(np.float32)
                y = select_training_target(data, stats, target_mode)
                velocity = (data["nao_velocity"].astype(np.float32) - vel_mean) / vel_std
                speaker_id = str(data["speaker_id"].item())

            if text.shape[1] != text_dim:
                raise ValueError(f"{clip_id} text dim {text.shape[1]} != expected {text_dim}")
            x = np.concatenate([prosody, wavlm, text], axis=1).astype(np.float32)
            if len(x) != len(y):
                raise ValueError(f"{clip_id} feature/target length mismatch: x={len(x)} y={len(y)}")

            speaker = np.zeros((speaker_dim,), dtype=np.float32)
            speaker_idx = speaker_id_map.get(speaker_id, -1)
            if speaker_idx >= 0:
                speaker[speaker_idx] = 1.0

            file_start = global_idx
            window_count = 0
            for start in window_starts(len(y), window_frames, stride_frames):
                y_pad_mode = "zero" if target_mode == "delta" else "repeat"
                value = pickle.dumps(
                    {
                        "x": fixed_length_window(x, start, window_frames).astype(np.float16),
                        "y": fixed_length_window(
                            y, start, window_frames, pad_mode=y_pad_mode
                        ).astype(np.float16),
                        "y_vel": fixed_length_window(
                            velocity, start, window_frames, pad_mode="zero"
                        ).astype(np.float16),
                        "speaker": speaker.astype(np.float16),
                        "speaker_id": speaker_id,
                        "clip_id": clip_id,
                        "split": split,
                        "start_frame": start,
                        "valid_mask": fixed_length_valid_mask(
                            len(y), start, window_frames
                        ).astype(np.float16),
                    }
                )
                txn.put(f"{global_idx:08d}".encode(), value)
                global_idx += 1
                window_count += 1
                if global_idx % 1000 == 0:
                    txn.commit()
                    txn = env.begin(write=True)

            file_window_map[clip_id] = {"start_idx": file_start, "count": window_count}

        metadata = {
            "split": split,
            "total_windows": global_idx,
            "window_frames": window_frames,
            "stride_frames": stride_frames,
            "fps": MOCAP_FPS,
            "input_dim": len(PROSODY_FEATURE_NAMES) + wavlm_dim + text_dim,
            "prosody_dim": len(PROSODY_FEATURE_NAMES),
            "wavlm_dim": wavlm_dim,
            "wavlm_model": stats.get("wavlm_model"),
            "include_wavlm": bool(wavlm_dim > 0),
            "include_text": bool(text_dim > 0),
            "text_dim": text_dim,
            "gesture_energy_dim": 0,
            "gesture_energy_names": [],
            "gesture_energy_feature_names": [],
            "gesture_energy_thresholds": [],
            "gesture_energy_audio_thresholds": [],
            "speaker_dim": speaker_dim,
            "speaker_id_map": speaker_id_map,
            "target_shape": [len(NAO_JOINTS)],
            "target_mode": target_mode,
            "target_representation": "nao_angles",
            "target_type": "nao_joint_deltas" if target_mode == "delta" else "nao_joint_angles",
            "conditioning_parts": stats["conditioning_parts"],
            "prediction_type": stats["prediction_type"],
            "window_seconds": window_seconds,
            "stride_seconds": stride_seconds,
            "has_valid_mask": True,
            "feature_names": stats["feature_names"],
            "target_names": NAO_JOINTS,
            "nao_mean": stats["nao_mean"],
            "nao_std": stats["nao_std"],
            "motion_mean": stats["motion_mean"],
            "motion_std": stats["motion_std"],
            "nao_vel_mean": stats["nao_vel_mean"],
            "nao_vel_std": stats["nao_vel_std"],
            "motion_vel_mean": stats["motion_vel_mean"],
            "motion_vel_std": stats["motion_vel_std"],
        }
        txn.put(b"__len__", pickle.dumps(global_idx))
        txn.put(b"__metadata__", pickle.dumps(metadata))
        txn.commit()
        env.close()

        lmdb_index[split] = {
            **metadata,
            "lmdb_path": str(lmdb_path),
            "file_window_map": file_window_map,
        }
        total_windows += global_idx

    with open(output_dir / "lmdb_index.json", "w") as f:
        json.dump({"total_windows": total_windows, "lmdb_by_split": lmdb_index}, f, indent=2)
    return total_windows


def preprocess(args: argparse.Namespace) -> None:
    output_dir = Path(args.output_dir)
    clips_dir = output_dir / "clips"
    words_dir = output_dir / "words"
    cache_dir = output_dir / "feature_cache"
    clips_dir.mkdir(parents=True, exist_ok=True)
    words_dir.mkdir(parents=True, exist_ok=True)

    split_filter = set(args.splits.split(",")) if args.splits else None
    split_rows = read_split_csv(Path(args.split_csv), split_filter)
    if args.max_clips:
        split_rows = split_rows[: args.max_clips]

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    text_dim = TEXT_EMBED_DIM if args.include_text else 0
    tokenizer = text_model = text_device = None
    if args.include_text:
        text_device = torch.device("cpu") if args.text_cpu else device
        print(f"[TEXT] Loading DistilBERT on {text_device}")
        tokenizer, text_model = init_text_models(text_device)

    wavlm_dim = 0
    wavlm_stats = None
    wavlm_processor = wavlm_model = wavlm_device = None
    if args.include_wavlm:
        wavlm_device = torch.device("cpu") if args.wavlm_cpu else device
        print(f"[WAVLM] Loading {args.wavlm_model} on {wavlm_device}")
        wavlm_processor, wavlm_model, wavlm_dim = init_wavlm_model(args.wavlm_model, wavlm_device)
        wavlm_stats = BatchedStats(wavlm_dim)

    prosody_stats = BatchedStats(len(PROSODY_FEATURE_NAMES))
    nao_stats = BatchedStats(len(NAO_JOINTS))
    vel_stats = BatchedStats(len(NAO_JOINTS))
    processed = []
    dropped = {
        "missing_motion": [],
        "missing_audio": [],
        "bad_motion": [],
        "unsupported_fps": [],
        "failed": [],
    }
    total_frames = 0

    motion_dir = Path(args.motion_dir)
    audio_dir = Path(args.audio_dir)
    textgrid_dir = Path(args.textgrid_dir)
    for row in tqdm(split_rows, desc="Preprocessing BEAT2 clips"):
        clip_id = row["id"]
        split = row["type"]
        npz_path = motion_dir / f"{clip_id}.npz"
        wav_path = audio_dir / f"{clip_id}.wav"

        if not npz_path.is_file():
            dropped["missing_motion"].append(clip_id)
            continue
        if not wav_path.is_file():
            dropped["missing_audio"].append(clip_id)
            continue

        try:
            data = np.load(str(npz_path), allow_pickle=True)
            poses = data["poses"]
            fps = int(data["mocap_frame_rate"]) if "mocap_frame_rate" in data else MOCAP_FPS
            if poses.ndim != 2 or poses.shape[1] != 165:
                dropped["bad_motion"].append(clip_id)
                continue
            if fps != MOCAP_FPS:
                dropped["unsupported_fps"].append(clip_id)
                continue

            speaker_id = speaker_id_from_clip_id(clip_id)
            nao_angles = poses_to_nao_angles(
                poses, fps=fps, velocity_limit=args.velocity_limit
            )
            nao_velocity = frame_velocity(nao_angles)
            prosody = extract_prosody(wav_path, poses.shape[0], fps=fps)
            words = load_words(clip_id, textgrid_dir)
            if args.include_text:
                text_features = extract_text_features(
                    words, poses.shape[0], tokenizer, text_model, text_device, fps=fps
                )
            else:
                text_features = np.zeros((poses.shape[0], 0), dtype=np.float32)
            if args.include_wavlm:
                wavlm_features = cached_wavlm_features(
                    wav_path,
                    clip_id,
                    poses.shape[0],
                    wavlm_processor,
                    wavlm_model,
                    wavlm_device,
                    cache_dir,
                    args.wavlm_model,
                )
            else:
                wavlm_features = np.zeros((poses.shape[0], 0), dtype=np.float32)

            np.savez_compressed(
                str(clips_dir / f"{clip_id}.npz"),
                prosody=prosody,
                wavlm_features=wavlm_features,
                text_features=text_features,
                nao_angles=nao_angles,
                nao_velocity=nao_velocity,
                fps=np.int32(fps),
                num_frames=np.int32(poses.shape[0]),
                clip_id=clip_id,
                speaker_id=speaker_id,
                split=split,
                target_representation="nao_angles",
                prosody_feature_names=np.array(PROSODY_FEATURE_NAMES),
                wavlm_feature_dim=np.int32(wavlm_dim),
                text_feature_dim=np.int32(text_dim),
                nao_joint_names=np.array(NAO_JOINTS),
            )
            with open(words_dir / f"{clip_id}.json", "w") as f:
                json.dump({"id": clip_id, "words": words}, f, indent=2)

            manifest_row = {
                "id": clip_id,
                "split": split,
                "num_frames": str(poses.shape[0]),
                "duration_sec": f"{poses.shape[0] / fps:.6f}",
                "fps": str(fps),
                "speaker_id": speaker_id,
            }
            processed.append(manifest_row)
            total_frames += poses.shape[0]

            if split == "train":
                prosody_stats.update(prosody)
                if wavlm_stats is not None:
                    wavlm_stats.update(wavlm_features)
                nao_stats.update(nao_angles)
                vel_stats.update(nao_velocity)
        except Exception as exc:
            print(f"[WARN] Failed {clip_id}: {exc}")
            dropped["failed"].append(clip_id)

    write_manifest(output_dir / "manifest.csv", processed)
    if prosody_stats.n == 0:
        raise RuntimeError("No training frames were found; include the train split.")

    prosody_mean, prosody_std = prosody_stats.finalise()
    if wavlm_stats is not None:
        wavlm_mean, wavlm_std = wavlm_stats.finalise()
    else:
        wavlm_mean = np.zeros((0,), dtype=np.float32)
        wavlm_std = np.ones((0,), dtype=np.float32)
    nao_mean, nao_std = nao_stats.finalise()
    vel_mean, vel_std = vel_stats.finalise()
    train_rows = [row for row in processed if row["split"] == "train"]
    speaker_id_map = build_speaker_id_map(train_rows)
    conditioning_parts = ["prosody"] + (["wavlm"] if args.include_wavlm else []) + (
        ["text"] if args.include_text else []
    )

    stats = {
        "prosody_feature_names": PROSODY_FEATURE_NAMES,
        "nao_joint_names": NAO_JOINTS,
        "prosody_mean": prosody_mean.tolist(),
        "prosody_std": prosody_std.tolist(),
        "wavlm_mean": wavlm_mean.tolist(),
        "wavlm_std": wavlm_std.tolist(),
        "wavlm_dim": wavlm_dim,
        "wavlm_model": args.wavlm_model if args.include_wavlm else None,
        "include_wavlm": args.include_wavlm,
        "include_text": args.include_text,
        "text_dim": text_dim,
        "input_dim": len(PROSODY_FEATURE_NAMES) + wavlm_dim + text_dim,
        "nao_mean": nao_mean.tolist(),
        "nao_std": nao_std.tolist(),
        "motion_mean": nao_mean.tolist(),
        "motion_std": nao_std.tolist(),
        "nao_vel_mean": vel_mean.tolist(),
        "nao_vel_std": vel_std.tolist(),
        "motion_vel_mean": vel_mean.tolist(),
        "motion_vel_std": vel_std.tolist(),
        "speaker_id_map": speaker_id_map,
        "speaker_dim": len(speaker_id_map),
        "target_mode": args.target_mode,
        "target_representation": "nao_angles",
        "target_shape": [len(NAO_JOINTS)],
        "target_names": NAO_JOINTS,
        "conditioning_parts": conditioning_parts,
        "feature_names": (
            PROSODY_FEATURE_NAMES
            + [f"wavlm_{idx:04d}" for idx in range(wavlm_dim)]
            + [f"text_{idx:04d}" for idx in range(text_dim)]
        ),
        "gesture_energy_dim": 0,
        "gesture_energy_names": [],
        "gesture_energy_feature_names": [],
        "gesture_energy_thresholds": [],
        "gesture_energy_audio_thresholds": [],
        "prediction_type": args.prediction_type,
        "fps": MOCAP_FPS,
        "num_train_frames": prosody_stats.n,
    }
    save_stats(output_dir, stats)

    summary = {
        "processed": len(processed),
        "total_frames": total_frames,
        "total_hours": total_frames / MOCAP_FPS / 3600,
        "dropped": {key: len(value) for key, value in dropped.items()},
        "motion_dir": str(motion_dir),
        "audio_dir": str(audio_dir),
        "textgrid_dir": str(textgrid_dir),
        "target_mode": args.target_mode,
        "conditioning_parts": conditioning_parts,
        "prosody_features": PROSODY_FEATURE_NAMES,
        "include_text": args.include_text,
        "text_feature_dim": text_dim,
        "include_wavlm": args.include_wavlm,
        "wavlm_model": args.wavlm_model if args.include_wavlm else None,
        "wavlm_feature_dim": wavlm_dim,
        "include_energy_clusters": False,
        "gesture_energy_dim": 0,
        "feature_cache_dir": str(cache_dir),
        "speaker_id_map": speaker_id_map,
        "window_seconds": args.window_size,
        "stride_seconds": args.stride,
        "prediction_type": args.prediction_type,
        "fps": MOCAP_FPS,
    }
    with open(output_dir / "preprocess_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    with open(output_dir / "dropped_clips.json", "w") as f:
        json.dump(dropped, f, indent=2)

    if not args.no_lmdb:
        window_frames = int(round(args.window_size * MOCAP_FPS))
        stride_frames = int(round(args.stride * MOCAP_FPS))
        windows = write_transformer_lmdb(
            output_dir,
            processed,
            stats,
            window_frames,
            stride_frames,
            args.map_size_gb,
            text_dim,
            wavlm_dim,
            args.target_mode,
            speaker_id_map,
            args.window_size,
            args.stride,
        )
        summary["total_windows"] = windows
        with open(output_dir / "preprocess_summary.json", "w") as f:
            json.dump(summary, f, indent=2)

    print("\n" + "=" * 70)
    print("  BEAT2 TRANSFORMER PREPROCESSING COMPLETE")
    print("=" * 70)
    print(f"  Processed clips: {len(processed)}")
    print(f"  Total frames:    {total_frames:,}")
    print(f"  Output:          {output_dir}")
    print(f"  Prosody dim:     {len(PROSODY_FEATURE_NAMES)}")
    print(f"  WavLM dim:       {wavlm_dim}")
    print(f"  Text dim:        {text_dim}")
    print("  Energy dim:      0")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Preprocess BEAT2 for the transformer baseline without energy outputs"
    )
    parser.add_argument("--motion-dir", default=str(MOTION_DIR))
    parser.add_argument("--audio-dir", default=str(AUDIO_DIR))
    parser.add_argument("--textgrid-dir", default=str(TEXTGRID_DIR))
    parser.add_argument("--split-csv", default=str(SPLIT_CSV))
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--splits", default=None, help="Comma-separated split filter, e.g. train,val")
    parser.add_argument("--max-clips", type=int, default=None)
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    parser.add_argument("--map-size-gb", type=int, default=100)
    parser.add_argument("--no-lmdb", action="store_true")
    parser.add_argument("--include-text", action="store_true")
    parser.add_argument("--text-cpu", action="store_true")
    parser.add_argument("--include-wavlm", dest="include_wavlm", action="store_true", default=True)
    parser.add_argument("--no-wavlm", dest="include_wavlm", action="store_false")
    parser.add_argument("--wavlm-model", default=DEFAULT_WAVLM_MODEL)
    parser.add_argument("--wavlm-cpu", action="store_true")
    parser.add_argument("--velocity-limit", action="store_true")
    parser.add_argument("--target-mode", choices=["angle", "delta"], default="angle")
    parser.add_argument("--prediction-type", choices=["x0", "epsilon"], default="epsilon")
    args = parser.parse_args()

    if args.window_size <= 0:
        raise ValueError("--window-size must be positive")
    if args.stride <= 0:
        raise ValueError("--stride must be positive")
    if args.map_size_gb <= 0:
        raise ValueError("--map-size-gb must be positive")
    if not Path(args.split_csv).is_file():
        raise FileNotFoundError(f"Split CSV not found: {args.split_csv}")

    preprocess(args)


if __name__ == "__main__":
    main()
