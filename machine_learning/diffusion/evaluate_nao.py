"""Evaluate generated NAO gesture sequences against ground truth."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def load_array(path: str | Path) -> np.ndarray:
    return np.load(str(path)).astype(np.float32)


def trim_pair(prediction: np.ndarray, ground_truth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    frames = min(len(prediction), len(ground_truth))
    if frames <= 0:
        raise ValueError("prediction and ground truth must contain at least one frame")
    return prediction[:frames], ground_truth[:frames]


def derivative(values: np.ndarray, order: int, fps: int) -> np.ndarray:
    out = values
    for _ in range(order):
        if len(out) <= 1:
            return np.zeros_like(out)
        out = np.diff(out, axis=0) * fps
    return out


def mean_l2(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    return float(np.linalg.norm(values.reshape(values.shape[0], -1), axis=1).mean())


def rhythm_correlation(motion: np.ndarray, onset_strength: np.ndarray) -> float | None:
    velocity = derivative(motion, order=1, fps=30)
    if len(velocity) == 0:
        return None
    motion_energy = np.linalg.norm(velocity.reshape(velocity.shape[0], -1), axis=1)
    frames = min(len(motion_energy), len(onset_strength))
    if frames < 2:
        return None
    a = motion_energy[:frames]
    b = onset_strength[:frames]
    if np.std(a) < 1e-8 or np.std(b) < 1e-8:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def load_onset_from_clip_npz(path: str | Path) -> np.ndarray:
    data = np.load(str(path), allow_pickle=True)
    names = [str(item) for item in data["prosody_feature_names"]]
    onset_idx = names.index("onset_strength")
    return data["prosody"][:, onset_idx].astype(np.float32)


def evaluate(prediction: np.ndarray, ground_truth: np.ndarray, fps: int,
             onset_strength: np.ndarray | None = None,
             samples: list[np.ndarray] | None = None) -> dict:
    prediction, ground_truth = trim_pair(prediction, ground_truth)
    velocity_pred = derivative(prediction, order=1, fps=fps)
    velocity_gt = derivative(ground_truth, order=1, fps=fps)

    results = {
        "frames": int(len(prediction)),
        "mse": float(np.mean(np.square(prediction - ground_truth))),
        "mae": float(np.mean(np.abs(prediction - ground_truth))),
        "velocity_error": float(np.mean(np.abs(velocity_pred - velocity_gt))) if len(velocity_pred) else 0.0,
        "mean_velocity": mean_l2(velocity_pred),
        "mean_acceleration": mean_l2(derivative(prediction, order=2, fps=fps)),
        "mean_jerk": mean_l2(derivative(prediction, order=3, fps=fps)),
    }
    if onset_strength is not None:
        results["rhythm_correlation"] = rhythm_correlation(prediction, onset_strength)
    if samples:
        aligned = [sample[:len(prediction)] for sample in samples if len(sample) >= len(prediction)]
        if len(aligned) >= 2:
            stacked = np.stack(aligned, axis=0)
            results["sample_diversity"] = float(np.mean(np.var(stacked, axis=0)))
    return results


def main():
    parser = argparse.ArgumentParser(description="Evaluate NAO gesture predictions")
    parser.add_argument("--prediction", required=True, help="Generated NAO angles .npy")
    parser.add_argument("--ground-truth", required=True, help="Ground-truth NAO angles .npy")
    parser.add_argument("--clip-npz", default=None,
                        help="Optional preprocessed clip .npz for onset-strength rhythm correlation")
    parser.add_argument("--sample", action="append", default=[],
                        help="Additional generated .npy sample for diversity; may be repeated")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--output", default=None, help="Optional JSON output path")
    args = parser.parse_args()

    onset = load_onset_from_clip_npz(args.clip_npz) if args.clip_npz else None
    samples = [load_array(path) for path in args.sample]
    results = evaluate(
        load_array(args.prediction),
        load_array(args.ground_truth),
        fps=args.fps,
        onset_strength=onset,
        samples=samples,
    )
    print(json.dumps(results, indent=2))
    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w") as f:
            json.dump(results, f, indent=2)


if __name__ == "__main__":
    main()
