#!/usr/bin/env python3
"""Plot movement statistics for generated gesture files."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from BEATArc.nao_constants import NAO_JOINTS  # noqa: E402


def load_gesture(path: Path) -> np.ndarray:
    values = np.load(str(path)).astype(np.float32)
    if values.ndim == 1:
        values = values[:, None]
    if values.ndim == 3:
        values = values.reshape(values.shape[0], -1)
    if values.ndim != 2:
        raise ValueError(f"Expected 2-D or 3-D gesture array, got {path}: {values.shape}")
    return values


def feature_names(dim: int) -> list[str]:
    if dim == len(NAO_JOINTS):
        return list(NAO_JOINTS)
    if dim in {2 * len(NAO_JOINTS), 3 * len(NAO_JOINTS)}:
        axes = ["x", "y", "z"][: dim // len(NAO_JOINTS)]
        return [f"{joint}_{axis}" for joint in NAO_JOINTS for axis in axes]
    return [f"feature_{idx}" for idx in range(dim)]


def movement_stats(values: np.ndarray, fps: int) -> dict:
    if len(values) > 1:
        velocity = np.diff(values, axis=0) * fps
        acceleration = np.diff(velocity, axis=0) * fps if len(velocity) > 1 else np.zeros_like(velocity)
    else:
        velocity = np.zeros((0, values.shape[1]), dtype=np.float32)
        acceleration = np.zeros((0, values.shape[1]), dtype=np.float32)

    ranges = values.max(axis=0) - values.min(axis=0)
    variances = values.var(axis=0)
    return {
        "frames": int(values.shape[0]),
        "dims": int(values.shape[1]),
        "duration_sec": float(values.shape[0] / fps),
        "mean_range": float(ranges.mean()),
        "max_range": float(ranges.max()) if ranges.size else 0.0,
        "mean_variance": float(variances.mean()),
        "max_variance": float(variances.max()) if variances.size else 0.0,
        "mean_abs_velocity": float(np.abs(velocity).mean()) if velocity.size else 0.0,
        "max_abs_velocity": float(np.abs(velocity).max()) if velocity.size else 0.0,
        "mean_abs_acceleration": float(np.abs(acceleration).mean()) if acceleration.size else 0.0,
        "per_feature_range": ranges.tolist(),
        "per_feature_variance": variances.tolist(),
    }


def save_bar(path: Path, names: list[str], values: np.ndarray, title: str, ylabel: str) -> None:
    width = max(8, min(22, 0.35 * len(values)))
    plt.figure(figsize=(width, 5))
    plt.bar(np.arange(len(values)), values)
    plt.xticks(np.arange(len(values)), names, rotation=75, ha="right", fontsize=8)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def save_clip_summary(path: Path, clip_names: list[str], stats: list[dict]) -> None:
    x = np.arange(len(clip_names))
    width = 0.28
    plt.figure(figsize=(max(10, 0.45 * len(clip_names)), 5))
    plt.bar(x - width, [item["mean_range"] for item in stats], width, label="Mean range")
    plt.bar(x, [item["mean_variance"] for item in stats], width, label="Mean variance")
    plt.bar(x + width, [item["mean_abs_velocity"] for item in stats], width, label="Mean abs velocity")
    plt.xticks(x, clip_names, rotation=75, ha="right", fontsize=8)
    plt.title("Movement Summary Per Clip")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def save_histogram(path: Path, values: np.ndarray, title: str, xlabel: str) -> None:
    plt.figure(figsize=(8, 5))
    plt.hist(values, bins=30)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel("Count")
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def save_example_trajectories(path: Path, clip_name: str, values: np.ndarray, names: list[str], top_n: int) -> None:
    variances = values.var(axis=0)
    top_idx = np.argsort(variances)[-min(top_n, values.shape[1]) :][::-1]
    plt.figure(figsize=(12, 6))
    for idx in top_idx:
        plt.plot(values[:, idx], label=names[idx])
    plt.title(f"Highest-Variance Features: {clip_name}")
    plt.xlabel("Frame")
    plt.ylabel("Value")
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def compare_to_ground_truth(pred: np.ndarray, truth: np.ndarray, fps: int) -> dict:
    length = min(len(pred), len(truth))
    pred = pred[:length]
    truth = truth[:length]
    pred_stats = movement_stats(pred, fps)
    truth_stats = movement_stats(truth, fps)
    return {
        "frames_compared": int(length),
        "mse": float(np.mean((pred - truth) ** 2)),
        "mean_abs_error": float(np.mean(np.abs(pred - truth))),
        "range_ratio": pred_stats["mean_range"] / max(truth_stats["mean_range"], 1e-8),
        "velocity_ratio": pred_stats["mean_abs_velocity"] / max(truth_stats["mean_abs_velocity"], 1e-8),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot movement variance/range/velocity for gesture .npy files")
    parser.add_argument("--input-dir", required=True, help="Directory containing generated .npy gestures")
    parser.add_argument("--output-dir", required=True, help="Directory for plots and summary JSON")
    parser.add_argument("--pattern", default="*.npy")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--ground-truth-dir", default=None, help="Optional folder with matching .npy ground truth")
    parser.add_argument("--top-n", type=int, default=6, help="Number of high-variance features in trajectory plot")
    args = parser.parse_args()

    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    files = sorted(input_dir.glob(args.pattern))
    if not files:
        raise RuntimeError(f"No files matched {args.pattern!r} in {input_dir}")

    summary = {}
    clip_names = []
    stats_list = []
    all_ranges = []
    all_variances = []
    per_feature_ranges = []
    per_feature_variances = []
    gt_dir = Path(args.ground_truth_dir) if args.ground_truth_dir else None

    for path in files:
        values = load_gesture(path)
        names = feature_names(values.shape[1])
        stats = movement_stats(values, args.fps)
        clip_names.append(path.stem)
        stats_list.append(stats)
        all_ranges.extend(stats["per_feature_range"])
        all_variances.extend(stats["per_feature_variance"])
        per_feature_ranges.append(np.asarray(stats["per_feature_range"], dtype=np.float32))
        per_feature_variances.append(np.asarray(stats["per_feature_variance"], dtype=np.float32))
        summary[path.stem] = stats

        save_example_trajectories(
            output_dir / f"{path.stem}_top_trajectories.png",
            path.stem,
            values,
            names,
            args.top_n,
        )

        if gt_dir is not None:
            truth_path = gt_dir / path.name
            if truth_path.is_file():
                truth = load_gesture(truth_path)
                if truth.shape[1] == values.shape[1]:
                    summary[path.stem]["ground_truth_comparison"] = compare_to_ground_truth(values, truth, args.fps)
                else:
                    summary[path.stem]["ground_truth_comparison_error"] = (
                        f"shape mismatch pred={values.shape} truth={truth.shape}"
                    )

    save_clip_summary(output_dir / "movement_summary_by_clip.png", clip_names, stats_list)
    mean_feature_range = np.mean(np.stack(per_feature_ranges), axis=0)
    mean_feature_variance = np.mean(np.stack(per_feature_variances), axis=0)
    names = feature_names(len(mean_feature_range))
    save_bar(
        output_dir / "mean_range_by_feature.png",
        names,
        mean_feature_range,
        "Mean Movement Range By Feature",
        "Range",
    )
    save_bar(
        output_dir / "mean_variance_by_feature.png",
        names,
        mean_feature_variance,
        "Mean Movement Variance By Feature",
        "Variance",
    )
    save_histogram(
        output_dir / "feature_range_histogram.png",
        np.asarray(all_ranges, dtype=np.float32),
        "Feature Movement Range Distribution",
        "Range",
    )
    save_histogram(
        output_dir / "feature_variance_histogram.png",
        np.asarray(all_variances, dtype=np.float32),
        "Feature Movement Variance Distribution",
        "Variance",
    )

    aggregate = {
        "num_files": len(files),
        "mean_range": float(np.mean([item["mean_range"] for item in stats_list])),
        "mean_variance": float(np.mean([item["mean_variance"] for item in stats_list])),
        "mean_abs_velocity": float(np.mean([item["mean_abs_velocity"] for item in stats_list])),
        "max_abs_velocity": float(np.max([item["max_abs_velocity"] for item in stats_list])),
    }
    with open(output_dir / "movement_summary.json", "w") as f:
        json.dump({"aggregate": aggregate, "clips": summary}, f, indent=2)

    print(f"[PLOTS] Wrote plots and summary to {output_dir}")
    print(f"[PLOTS] Aggregate mean range: {aggregate['mean_range']:.6f}")
    print(f"[PLOTS] Aggregate mean abs velocity: {aggregate['mean_abs_velocity']:.6f}")


if __name__ == "__main__":
    main()
