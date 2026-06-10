#!/usr/bin/env python3
"""Fréchet gesture distance metrics and plotting helpers."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import linalg

from machine_learning import plot_style as ps
from machine_learning.transformers import plot_graphs as plots


@dataclass(frozen=True)
class ModelPlotConfig:
    model_label: str
    short_name: str
    color: str
    prediction_key: str


def discrete_frechet_distance(p: np.ndarray, q: np.ndarray) -> float:
    """Discrete Fréchet distance between two pose trajectories of shape (T, D)."""
    if len(p) == 0 or len(q) == 0:
        return float("nan")
    dist = np.linalg.norm(p[:, None, :] - q[None, :, :], axis=2)
    n, m = dist.shape
    ca = np.full((n, m), np.inf, dtype=np.float64)
    ca[0, 0] = dist[0, 0]
    for i in range(1, n):
        ca[i, 0] = max(ca[i - 1, 0], dist[i, 0])
    for j in range(1, m):
        ca[0, j] = max(ca[0, j - 1], dist[0, j])
    for i in range(1, n):
        for j in range(1, m):
            ca[i, j] = max(min(ca[i - 1, j], ca[i, j - 1], ca[i - 1, j - 1]), dist[i, j])
    return float(ca[n - 1, m - 1])


def frechet_gesture_distance(features_real: np.ndarray, features_generated: np.ndarray) -> float:
    """Distribution-level Fréchet Gesture Distance (Yoon et al., 2020)."""
    if len(features_real) < 2 or len(features_generated) < 2:
        raise ValueError("FGD requires at least two sequences per distribution")
    features_real = np.asarray(features_real, dtype=np.float64)
    features_generated = np.asarray(features_generated, dtype=np.float64)
    mu_r = np.mean(features_real, axis=0)
    mu_g = np.mean(features_generated, axis=0)
    sigma_r = np.cov(features_real, rowvar=False)
    sigma_g = np.cov(features_generated, rowvar=False)
    if sigma_r.ndim == 0:
        sigma_r = np.array([[float(sigma_r)]], dtype=np.float64)
    if sigma_g.ndim == 0:
        sigma_g = np.array([[float(sigma_g)]], dtype=np.float64)
    eps = 1e-6
    sigma_r = sigma_r + np.eye(sigma_r.shape[0]) * eps
    sigma_g = sigma_g + np.eye(sigma_g.shape[0]) * eps
    diff = mu_r - mu_g
    covmean = linalg.sqrtm(sigma_r @ sigma_g)
    if np.iscomplexobj(covmean):
        if not np.allclose(covmean.imag, 0.0, atol=1e-3):
            raise ValueError("FGD covariance square root has large imaginary component")
        covmean = covmean.real
    return float(diff @ diff + np.trace(sigma_r + sigma_g - 2.0 * covmean))


def subset_values(values: np.ndarray, arm_idx: list[int] | None) -> np.ndarray:
    if arm_idx is None:
        return values
    return values[:, arm_idx]


def resample_pair(
    ground_truth: np.ndarray,
    prediction: np.ndarray,
    target_frames: int,
) -> tuple[np.ndarray, np.ndarray]:
    min_len = min(len(ground_truth), len(prediction))
    if min_len < 2:
        raise ValueError("Matched sequences must contain at least two frames")
    ground_truth = ground_truth[:min_len]
    prediction = prediction[:min_len]
    return (
        plots.resample_sequence(ground_truth, target_frames),
        plots.resample_sequence(prediction, target_frames),
    )


def per_sequence_discrete_frechet(
    ground_truth: list[np.ndarray],
    predictions: list[np.ndarray],
    target_frames: int,
    arm_idx: list[int] | None = None,
) -> np.ndarray:
    distances = []
    for gt, pred in zip(ground_truth, predictions):
        gt_resampled, pred_resampled = resample_pair(gt, pred, target_frames)
        gt_resampled = subset_values(gt_resampled, arm_idx)
        pred_resampled = subset_values(pred_resampled, arm_idx)
        distances.append(discrete_frechet_distance(gt_resampled, pred_resampled))
    return np.asarray(distances, dtype=np.float64)


def flattened_sequence_features(
    sequences: list[np.ndarray],
    target_frames: int,
    arm_idx: list[int] | None = None,
) -> np.ndarray:
    rows = []
    for values in sequences:
        resampled = plots.resample_sequence(values, target_frames)
        resampled = subset_values(resampled, arm_idx)
        rows.append(resampled.reshape(-1))
    return np.stack(rows, axis=0)


def discover_matched_files(ground_truth_dir: Path, prediction_dir: Path, pattern: str) -> list[str]:
    gt_files = {path.name for path in ground_truth_dir.glob(pattern)}
    pred_files = {path.name for path in prediction_dir.glob(pattern)}
    matched = sorted(gt_files & pred_files)
    if not matched:
        raise RuntimeError("No matching files found across ground-truth and prediction folders.")
    missing = sorted(gt_files - pred_files)
    if missing:
        print(f"[WARN] {len(missing)} GT files missing from prediction folder")
    return matched


def load_matched_pairs(
    ground_truth_dir: Path,
    prediction_dir: Path,
    pattern: str,
) -> tuple[list[str], list[np.ndarray], list[np.ndarray]]:
    filenames = discover_matched_files(ground_truth_dir, prediction_dir, pattern)
    ground_truth = []
    predictions = []
    for filename in filenames:
        gt = plots.load_gesture(ground_truth_dir / filename)
        pred = plots.load_gesture(prediction_dir / filename)
        if gt.shape[1] != pred.shape[1]:
            raise ValueError(f"Feature dimension mismatch for {filename}: {gt.shape[1]} vs {pred.shape[1]}")
        ground_truth.append(gt)
        predictions.append(pred)
    return filenames, ground_truth, predictions


def load_autoencoder(checkpoint_path: Path, device):
    from machine_learning.latent_diffusion.build_latent_dataset import load_autoencoder as _load

    return _load(checkpoint_path, device)


def latent_sequence_features(
    autoencoder,
    sequences: list[np.ndarray],
    device,
    target_frames: int,
) -> np.ndarray:
    import torch

    rows = []
    with torch.no_grad():
        for values in sequences:
            resampled = plots.resample_sequence(values, target_frames)
            tensor = torch.from_numpy(resampled[None]).to(device)
            latents = autoencoder.encode(tensor)[0].mean(dim=0).cpu().numpy()
            rows.append(latents)
    return np.stack(rows, axis=0)


def summary_stats(values: np.ndarray) -> dict:
    return {
        "mean": float(np.mean(values)),
        "std": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
        "median": float(np.median(values)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "n": int(len(values)),
    }


def save_figure(output_dir: Path, stem: str) -> None:
    ps.save_figure(output_dir, stem)


def plot_per_sequence_boxplot(
    output_dir: Path,
    distances: np.ndarray,
    config: ModelPlotConfig,
) -> None:
    fig, ax = plt.subplots(figsize=(7.0, 6.8), facecolor=ps.FIGURE_FACECOLOR)
    ps.style_axes(ax)
    box = ax.boxplot(
        [distances],
        tick_labels=[f"{config.short_name}\n(n={len(distances)})"],
        patch_artist=True,
        showfliers=True,
        widths=0.52,
        medianprops={"linewidth": 0},
    )
    ps.style_boxplot(box, [config.color])
    ax.set_ylabel("Discrete Fréchet distance (rad)")
    ax.set_title(f"Per-Sequence Fréchet Distance: {config.model_label} vs Ground Truth", pad=14)
    plots.finalize_figure(
        fig,
        ax,
        note="Each point is the discrete Fréchet distance between one matched GT/prediction pair.",
        bottom=0.22,
    )
    save_figure(output_dir, "01_per_sequence_frechet_boxplot")


def plot_per_sequence_histogram(
    output_dir: Path,
    distances: np.ndarray,
    config: ModelPlotConfig,
) -> None:
    fig, ax = plt.subplots(figsize=(9.0, 6.8), facecolor=ps.FIGURE_FACECOLOR)
    ps.style_axes(ax)
    ax.hist(
        distances,
        bins=min(40, max(10, len(distances) // 3)),
        color=config.color,
        alpha=0.55,
        edgecolor=ps.darker_edge(config.color),
        linewidth=0.9,
    )
    ax.axvline(float(np.mean(distances)), color=ps.ACCENT_LINE, linestyle="--", linewidth=1.6, label="Mean")
    ax.axvline(float(np.median(distances)), color=ps.MUTED_TEXT, linestyle=":", linewidth=1.6, label="Median")
    ax.set_xlabel("Discrete Fréchet distance (rad)")
    ax.set_ylabel("Number of sequences")
    ax.set_title(f"Per-Sequence Fréchet Distance Distribution: {config.model_label}", pad=14)
    ax.legend(framealpha=0.95)
    plots.finalize_figure(fig, ax, bottom=0.18)
    save_figure(output_dir, "02_per_sequence_frechet_histogram")


def plot_per_sequence_cdf(
    output_dir: Path,
    distances: np.ndarray,
    config: ModelPlotConfig,
) -> None:
    sorted_values = np.sort(distances)
    y = np.linspace(0.0, 1.0, len(sorted_values), endpoint=True)
    fig, ax = plt.subplots(figsize=(9.0, 6.8), facecolor=ps.FIGURE_FACECOLOR)
    ps.style_axes(ax)
    ax.plot(sorted_values, y, linewidth=5.0, color=config.color, alpha=0.14, solid_capstyle="round", zorder=1)
    ax.plot(sorted_values, y, color=config.color, linewidth=2.6, label=config.short_name, zorder=2)
    ax.set_xlabel("Discrete Fréchet distance (rad)")
    ax.set_ylabel("Cumulative probability")
    ax.set_title(f"CDF of Per-Sequence Fréchet Distance: {config.model_label}", pad=14)
    ax.legend(framealpha=0.95)
    plots.finalize_figure(fig, ax, bottom=0.18)
    save_figure(output_dir, "03_per_sequence_frechet_cdf")


def plot_dataset_fgd_bar(
    output_dir: Path,
    fgd_values: dict[str, float],
    config: ModelPlotConfig,
) -> None:
    labels = list(fgd_values.keys())
    values = [fgd_values[label] for label in labels]
    bar_colors = [ps.COLOR_GROUND_TRUTH, config.color]
    fig, ax = plt.subplots(figsize=(8.0, 6.8), facecolor=ps.FIGURE_FACECOLOR)
    ps.style_axes(ax)
    x = np.arange(len(labels))
    for idx, (label, value, color) in enumerate(zip(labels, values, bar_colors[: len(labels)])):
        ax.bar(x[idx], value, width=0.58, **ps.bar_kwargs(color))
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=15, ha="right")
    ax.set_ylabel("Fréchet Gesture Distance")
    ax.set_title(f"Dataset-Level FGD: {config.model_label} vs Ground Truth", pad=14)
    for idx, value in enumerate(values):
        ax.text(idx, value, f"{value:.3f}", ha="center", va="bottom", fontsize=11, color=ps.MUTED_TEXT)
    plots.finalize_figure(
        fig,
        ax,
        note="Lower FGD indicates closer distributional match in the chosen feature space.",
        bottom=0.24,
    )
    save_figure(output_dir, "04_dataset_fgd")


def run_frechet_evaluation(
    *,
    config: ModelPlotConfig,
    ground_truth_dir: Path,
    prediction_dir: Path,
    output_dir: Path,
    pattern: str,
    resample_frames: int,
    autoencoder_path: Path | None,
    cpu: bool,
) -> dict:
    plots.configure_style()
    output_dir.mkdir(parents=True, exist_ok=True)

    filenames, ground_truth, predictions = load_matched_pairs(ground_truth_dir, prediction_dir, pattern)
    joint_names = plots.feature_names(ground_truth[0].shape[1])
    arm_idx = plots.arm_indices(joint_names)
    arm_only_idx = arm_idx if arm_idx else None

    per_sequence = per_sequence_discrete_frechet(
        ground_truth,
        predictions,
        target_frames=resample_frames,
        arm_idx=arm_only_idx,
    )
    raw_features_gt = flattened_sequence_features(ground_truth, resample_frames, arm_only_idx)
    raw_features_pred = flattened_sequence_features(predictions, resample_frames, arm_only_idx)
    fgd_raw = frechet_gesture_distance(raw_features_gt, raw_features_pred)

    results = {
        "model": config.model_label,
        "num_matched_sequences": len(filenames),
        "matched_filenames": filenames,
        "resample_frames": resample_frames,
        "arm_joint_names": [joint_names[i] for i in arm_idx] if arm_idx else joint_names,
        "per_sequence_discrete_frechet": {
            "values": per_sequence.tolist(),
            "summary": summary_stats(per_sequence),
        },
        "dataset_fgd_raw": float(fgd_raw),
    }

    fgd_for_plot = {"Raw motion space": float(fgd_raw)}

    if autoencoder_path is not None:
        import torch

        device = torch.device("cpu" if cpu or not torch.cuda.is_available() else "cuda")
        autoencoder, ae_config, _ = load_autoencoder(autoencoder_path, device)
        if tuple(ae_config["target_shape"]) != (ground_truth[0].shape[1],):
            print(
                "[WARN] Autoencoder target_shape does not match gesture dimension; "
                "skipping latent-space FGD."
            )
        else:
            latent_gt = latent_sequence_features(
                autoencoder, ground_truth, device, resample_frames
            )
            latent_pred = latent_sequence_features(
                autoencoder, predictions, device, resample_frames
            )
            fgd_latent = frechet_gesture_distance(latent_gt, latent_pred)
            results["dataset_fgd_latent"] = float(fgd_latent)
            fgd_for_plot["Latent space"] = float(fgd_latent)

    plot_per_sequence_boxplot(output_dir, per_sequence, config)
    plot_per_sequence_histogram(output_dir, per_sequence, config)
    plot_per_sequence_cdf(output_dir, per_sequence, config)
    plot_dataset_fgd_bar(output_dir, fgd_for_plot, config)

    results["figures"] = [
        "01_per_sequence_frechet_boxplot",
        "02_per_sequence_frechet_histogram",
        "03_per_sequence_frechet_cdf",
        "04_dataset_fgd",
    ]

    with open(output_dir / "frechet_gesture_summary.json", "w") as handle:
        json.dump(results, handle, indent=2)

    print(f"\n[FGD] {config.model_label}")
    print(f"  Matched sequences: {len(filenames)}")
    print(f"  Per-sequence discrete Fréchet: mean={results['per_sequence_discrete_frechet']['summary']['mean']:.4f}")
    print(f"  Dataset FGD (raw): {fgd_raw:.4f}")
    if "dataset_fgd_latent" in results:
        print(f"  Dataset FGD (latent): {results['dataset_fgd_latent']:.4f}")
    print(f"[FGD] Wrote figures to {output_dir}")
    return results


def build_arg_parser(description: str, prediction_flag: str, prediction_help: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--ground-truth-dir", required=True)
    parser.add_argument(prediction_flag, required=True, help=prediction_help)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--pattern", default="*.npy")
    parser.add_argument("--resample-frames", type=int, default=120)
    parser.add_argument(
        "--autoencoder",
        default=None,
        help="Optional motion autoencoder checkpoint for latent-space FGD",
    )
    parser.add_argument("--cpu", action="store_true", help="Force CPU when encoding latents")
    return parser
