#!/usr/bin/env python3
"""Post-process saved latent-diffusion NAO angle predictions."""

from __future__ import annotations

import argparse
import json
import sys
from copy import deepcopy
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from beat2_nao.config import AUDIO_DIR, AUDIO_SR, MOCAP_FPS  # noqa: E402
from beat2_nao.infer_nao import (  # noqa: E402
    clamp_nao_angles,
    infer_energy_features_from_prosody,
    save_metadata,
    smooth_nao_angles,
)
from beat2_nao.nao_constants import NAO_JOINTS  # noqa: E402
from beat2_nao.preprocess_nao import extract_prosody  # noqa: E402

BITBUCKET_ROOT = Path("/vol/bitbucket/ap1922")
DEFAULT_INPUT_DIR = BITBUCKET_ROOT / "latent_diffusion_predictions"
DEFAULT_OUTPUT_DIR = BITBUCKET_ROOT / "latent_diffusion_predictions_smoothed"


def load_sidecar_metadata(pred_path: Path) -> dict:
    meta_path = pred_path.with_suffix(".json")
    if not meta_path.is_file():
        return {}
    with open(meta_path, "r") as handle:
        return json.load(handle)


def load_stats(stats_path: Path) -> dict:
    with open(stats_path, "r") as handle:
        return json.load(handle)


def resolve_wav_path(pred_path: Path, metadata: dict, audio_dir: Path) -> Path:
    wav_value = metadata.get("wav")
    if wav_value:
        wav_path = Path(wav_value)
        if wav_path.is_file():
            return wav_path
    clip_id = pred_path.stem
    candidate = audio_dir / f"{clip_id}.wav"
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(
        f"Could not resolve WAV for {pred_path.name}; "
        f"expected metadata wav or {candidate}"
    )


def energy_labels_for_wav(
    wav_path: Path,
    num_frames: int,
    stats: dict,
    dataset_metadata: dict,
    fps: int,
) -> np.ndarray:
    prosody = extract_prosody(wav_path, num_frames, fps=fps, sample_rate=AUDIO_SR)
    prosody_mean = np.array(stats["prosody_mean"], dtype=np.float32)
    prosody_std = np.array(stats["prosody_std"], dtype=np.float32)
    prosody = (prosody - prosody_mean) / prosody_std
    _energy_features, labels = infer_energy_features_from_prosody(
        prosody, stats, dataset_metadata
    )
    return labels.astype(np.int64)


def apply_rest_blend_angles(
    angles: np.ndarray,
    energy_labels: np.ndarray,
    stats: dict,
    rest_blend: float,
    *,
    energy_gate: bool,
    previous_rest_blend: float,
    previous_energy_gate: bool,
) -> np.ndarray:
    if not energy_gate or rest_blend <= 0.0:
        return angles.astype(np.float32)

    if len(energy_labels) != len(angles):
        raise ValueError(
            f"Energy labels length {len(energy_labels)} does not match "
            f"prediction length {len(angles)}"
        )

    mean = np.array(stats["nao_mean"], dtype=np.float32)
    std = np.array(stats["nao_std"], dtype=np.float32)
    norm = (angles - mean) / std
    low_mask = energy_labels == 0

    if previous_energy_gate and previous_rest_blend > 0.0:
        undo = 1.0 - previous_rest_blend
        if undo <= 0.0:
            raise ValueError(f"Cannot undo previous rest_blend={previous_rest_blend}")
        norm[low_mask] = norm[low_mask] / undo

    norm[low_mask] = (1.0 - rest_blend) * norm[low_mask]
    return (norm * std + mean).astype(np.float32)


def postprocess_prediction(
    pred_path: Path,
    output_path: Path,
    *,
    smooth_window: int,
    rest_blend: float,
    energy_gate: bool,
    audio_dir: Path,
    stats_cache: dict[str, dict],
) -> None:
    metadata = load_sidecar_metadata(pred_path)
    target_mode = metadata.get("target_mode", "angle")
    if target_mode != "angle":
        raise ValueError(
            f"{pred_path.name}: post-processing rest blend only supports target_mode='angle', "
            f"got {target_mode!r}"
        )

    stats_path = Path(metadata.get("stats", ""))
    if not stats_path.is_file():
        raise FileNotFoundError(f"{pred_path.name}: stats file not found: {stats_path}")
    stats_key = str(stats_path)
    if stats_key not in stats_cache:
        stats_cache[stats_key] = load_stats(stats_path)
    stats = stats_cache[stats_key]

    angles = np.load(pred_path).astype(np.float32)
    if angles.ndim != 2 or angles.shape[1] != len(NAO_JOINTS):
        raise ValueError(
            f"{pred_path.name}: expected shape (frames, {len(NAO_JOINTS)}), got {angles.shape}"
        )

    fps = int(metadata.get("fps", MOCAP_FPS))
    dataset_metadata = metadata.get("dataset_metadata") or {}
    wav_path = resolve_wav_path(pred_path, metadata, audio_dir)
    energy_labels = energy_labels_for_wav(
        wav_path, angles.shape[0], stats, dataset_metadata, fps
    )

    previous_energy_gate = bool(metadata.get("energy_gate", False))
    previous_rest_blend = float(metadata.get("rest_blend", 0.0) or 0.0)
    angles = apply_rest_blend_angles(
        angles,
        energy_labels,
        stats,
        rest_blend,
        energy_gate=energy_gate,
        previous_rest_blend=previous_rest_blend,
        previous_energy_gate=previous_energy_gate,
    )
    angles = smooth_nao_angles(angles, smooth_window).astype(np.float32)
    angles = clamp_nao_angles(angles).astype(np.float32)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(output_path, angles)

    updated_metadata = deepcopy(metadata) if metadata else {}
    updated_metadata.update(
        {
            "smooth_window": smooth_window,
            "rest_blend": rest_blend,
            "energy_gate": energy_gate,
            "energy_label_counts": np.bincount(energy_labels, minlength=3).astype(int).tolist(),
            "postprocessed_from": str(pred_path),
            "postprocess_source_dir": str(pred_path.parent),
            "num_frames": int(angles.shape[0]),
            "duration_sec": float(angles.shape[0] / fps),
        }
    )
    save_metadata(output_path, updated_metadata)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Apply rest blending and temporal smoothing to saved latent-diffusion "
            "NAO angle predictions."
        )
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=DEFAULT_INPUT_DIR,
        help="Directory containing source .npy/.json prediction pairs",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for post-processed predictions",
    )
    parser.add_argument("--smooth-window", type=int, default=9)
    parser.add_argument("--rest-blend", type=float, default=0.7)
    parser.add_argument(
        "--disable-energy-gate",
        action="store_true",
        help="Skip low-energy rest blending and only smooth",
    )
    parser.add_argument("--audio-dir", type=Path, default=AUDIO_DIR)
    parser.add_argument("--continue-on-error", action="store_true")
    args = parser.parse_args()

    if args.smooth_window < 1:
        raise ValueError("--smooth-window must be at least 1")
    if not 0.0 <= args.rest_blend <= 1.0:
        raise ValueError("--rest-blend must be in [0, 1]")

    input_dir = args.input_dir
    output_dir = args.output_dir
    if not input_dir.is_dir():
        raise FileNotFoundError(f"Input directory not found: {input_dir}")

    pred_paths = sorted(input_dir.glob("*.npy"))
    if not pred_paths:
        raise RuntimeError(f"No .npy prediction files found in {input_dir}")

    output_dir.mkdir(parents=True, exist_ok=True)
    energy_gate = not args.disable_energy_gate
    stats_cache: dict[str, dict] = {}
    failures: list[dict[str, str]] = []

    print(f"[POST] Input:         {input_dir}")
    print(f"[POST] Output:        {output_dir}")
    print(f"[POST] Smooth window: {args.smooth_window}")
    print(f"[POST] Rest blend:    {args.rest_blend} (energy_gate={energy_gate})")
    print(f"[POST] Files:         {len(pred_paths)}")

    for idx, pred_path in enumerate(pred_paths, start=1):
        output_path = output_dir / pred_path.name
        print(f"[POST] {idx}/{len(pred_paths)} {pred_path.name}")
        try:
            postprocess_prediction(
                pred_path,
                output_path,
                smooth_window=args.smooth_window,
                rest_blend=args.rest_blend,
                energy_gate=energy_gate,
                audio_dir=args.audio_dir,
                stats_cache=stats_cache,
            )
        except Exception as exc:
            if not args.continue_on_error:
                raise
            print(f"[WARN] Failed {pred_path.name}: {exc}")
            failures.append({"file": pred_path.name, "error": str(exc)})

    if failures:
        failure_path = output_dir / "failed_postprocess.json"
        with open(failure_path, "w") as handle:
            json.dump(failures, handle, indent=2)
        print(f"[POST] Completed with {len(failures)} failures: {failure_path}")
    else:
        print("[POST] Completed without failures")


if __name__ == "__main__":
    main()
