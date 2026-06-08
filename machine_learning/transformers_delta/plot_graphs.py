#!/usr/bin/env python3
"""Evaluate delta-transformer gesture predictions against ground-truth motion.

The goal is to produce dissertation-quality evidence for whether the
Delta Transformer baseline collapses toward low-amplitude, low-diversity gestures.
All inputs are .npy files with matching filenames in the ground-truth and
Delta Transformer folders.
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from BEATArc.nao_constants import NAO_JOINTS  # noqa: E402

UNIT_ANGLE = "rad"
UNIT_ANGLE_SQ = "rad²"
UNIT_VELOCITY = "rad/s"
UNIT_ENERGY = "rad²/frame"

DATASET_ORDER = ["Ground Truth", "Delta Transformer"]
DATASET_KEYS = {"Ground Truth": "ground_truth", "Delta Transformer": "transformer_delta"}
COLORS = {"Ground Truth": "#2E86AB", "Delta Transformer": "#D1495B"}
DATASET_LEGEND_LABELS = {
    "Ground Truth": "Ground truth (reference motion from dataset)",
    "Delta Transformer": "Delta Transformer (model predictions)",
}
ARM_JOINTS = [
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
]


def legend_label(dataset: str, detail: str | None = None) -> str:
    base = DATASET_LEGEND_LABELS.get(dataset, dataset)
    if detail:
        return f"{base} — {detail}"
    return base


def condition_legend_handles(details: dict[str, str] | None = None) -> list:
    handles = []
    for dataset in DATASET_ORDER:
        detail = (details or {}).get(dataset)
        handles.append(
            Patch(
                facecolor=COLORS[dataset],
                edgecolor="black",
                linewidth=0.7,
                label=legend_label(dataset, detail),
            )
        )
    return handles


SHORT_CONDITION_NAMES = {
    "Ground Truth": "Ground truth",
    "Delta Transformer": "Delta Transformer",
}

FIGURE_NOTES = {
    "bar_sem": "Bars = mean per sequence; error bars = ±1 standard error of the mean (SEM).",
    "boxplot": "Box = IQR; line = median; whiskers = 1.5×IQR.",
    "joint_mean": "Bars = mean across matched sequences (no error bars).",
    "histogram": "Each sample is one frame-to-frame joint velocity magnitude.",
    "cdf": "Cumulative distribution of frame-to-frame joint velocity magnitudes.",
    "pca": "Each point is one joint pose; PCA fitted on combined samples from both conditions.",
}


def short_condition_name(dataset: str) -> str:
    return SHORT_CONDITION_NAMES.get(dataset, dataset)


def xtick_with_count(dataset: str, count: int, unit: str = "sequences") -> str:
    return f"{short_condition_name(dataset)}\n(n={count})"


def place_legend_below(
    ax,
    handles: list,
    *,
    ncol: int = 1,
    anchor_y: float = -0.12,
    fontsize: float = 10,
) -> None:
    ax.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.5, anchor_y),
        ncol=ncol,
        framealpha=0.95,
        fontsize=fontsize,
        title="Legend",
        borderaxespad=0.0,
    )


def finalize_figure(
    fig,
    ax,
    *,
    note: str | None = None,
    bottom: float = 0.28,
    top: float = 0.90,
) -> None:
    if note:
        fig.text(0.5, 0.01, note, ha="center", va="bottom", fontsize=9.5, color="#333333")
        bottom = max(bottom, 0.26)
    fig.subplots_adjust(left=0.14, right=0.98, top=top, bottom=bottom)


def color_legend_handles(*, include_sem: bool = False) -> list:
    handles = [
        Patch(
            facecolor=COLORS[dataset],
            edgecolor="black",
            linewidth=0.7,
            label=short_condition_name(dataset),
        )
        for dataset in DATASET_ORDER
    ]
    if include_sem:
        handles.append(sem_errorbar_legend_handle())
    return handles


def apply_figure_note(note: str, bottom_margin: float = 0.16) -> None:
    finalize_figure(plt.gcf(), plt.gca(), note=note, bottom=bottom_margin)


def sem_errorbar_legend_handle() -> Line2D:
    return Line2D(
        [0],
        [0],
        color="black",
        linewidth=1.2,
        marker="|",
        markersize=8,
        label="Error bars: ±1 SEM",
    )


def bar_chart_legend(ax) -> None:
    place_legend_below(ax, color_legend_handles(include_sem=True), ncol=1, anchor_y=-0.10)
    finalize_figure(ax.figure, ax, note=FIGURE_NOTES["bar_sem"], bottom=0.34)


def boxplot_legend(ax) -> None:
    handles = color_legend_handles()
    handles.extend(
        [
            Line2D([0], [0], color="black", linewidth=1.5, label="Line: median"),
            Line2D(
                [0],
                [0],
                color="black",
                linewidth=4,
                alpha=0.35,
                label="Box: IQR",
            ),
            Line2D(
                [0],
                [0],
                color="black",
                linewidth=1,
                linestyle="--",
                label="Whiskers: 1.5×IQR",
            ),
        ]
    )
    place_legend_below(ax, handles, ncol=2, anchor_y=-0.14, fontsize=9.5)
    finalize_figure(ax.figure, ax, note=FIGURE_NOTES["boxplot"], bottom=0.40)


def configure_style() -> None:
    plt.rcParams.update(
        {
            "figure.figsize": (10, 6),
            "figure.dpi": 140,
            "savefig.dpi": 300,
            "font.size": 14,
            "axes.titlesize": 17,
            "axes.labelsize": 15,
            "xtick.labelsize": 12,
            "ytick.labelsize": 12,
            "legend.fontsize": 12,
            "axes.grid": True,
            "grid.alpha": 0.25,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


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


def arm_indices(names: list[str]) -> list[int]:
    name_to_idx = {name: idx for idx, name in enumerate(names)}
    return [name_to_idx[name] for name in ARM_JOINTS if name in name_to_idx]


def velocity(values: np.ndarray) -> np.ndarray:
    if len(values) <= 1:
        return np.zeros((0, values.shape[1]), dtype=np.float32)
    return np.diff(values, axis=0).astype(np.float32)


def velocity_per_second(values: np.ndarray, fps: int) -> np.ndarray:
    return velocity(values) * fps


def motion_energy(values: np.ndarray) -> float:
    delta = velocity(values)
    if delta.size == 0:
        return 0.0
    return float(np.mean(np.sum(delta**2, axis=1)))


def sequence_metrics(values: np.ndarray, fps: int, indices: list[int] | None = None) -> dict[str, float]:
    subset = values[:, indices] if indices is not None else values
    vel = velocity_per_second(subset, fps)
    abs_vel = np.abs(vel)
    return {
        "joint_angle_range": float(np.mean(subset.max(axis=0) - subset.min(axis=0))),
        "mean_abs_joint_velocity": float(abs_vel.mean()) if abs_vel.size else 0.0,
        "peak_joint_velocity": float(abs_vel.max()) if abs_vel.size else 0.0,
        "motion_energy": motion_energy(subset),
        "average_trajectory_variance": float(np.mean(np.var(subset, axis=0))),
    }


def per_joint_arrays(values: np.ndarray, fps: int) -> dict[str, np.ndarray]:
    vel = velocity_per_second(values, fps)
    abs_vel = np.abs(vel)
    return {
        "range": values.max(axis=0) - values.min(axis=0),
        "variance": values.var(axis=0),
        "mean_abs_velocity": abs_vel.mean(axis=0) if abs_vel.size else np.zeros(values.shape[1]),
        "peak_velocity": abs_vel.max(axis=0) if abs_vel.size else np.zeros(values.shape[1]),
        "motion_energy": np.mean(velocity(values) ** 2, axis=0) if len(values) > 1 else np.zeros(values.shape[1]),
    }


def velocity_magnitudes(values: np.ndarray, indices: list[int] | None = None, fps: int = 30) -> np.ndarray:
    subset = values[:, indices] if indices is not None else values
    vel = np.abs(velocity_per_second(subset, fps))
    return vel.reshape(-1) if vel.size else np.zeros((0,), dtype=np.float32)


def resample_sequence(values: np.ndarray, target_frames: int) -> np.ndarray:
    if len(values) == target_frames:
        return values
    old_x = np.linspace(0.0, 1.0, len(values))
    new_x = np.linspace(0.0, 1.0, target_frames)
    out = np.empty((target_frames, values.shape[1]), dtype=np.float32)
    for dim in range(values.shape[1]):
        out[:, dim] = np.interp(new_x, old_x, values[:, dim])
    return out


def pairwise_sequence_distances(sequences: list[np.ndarray], target_frames: int = 120) -> np.ndarray:
    if len(sequences) < 2:
        return np.zeros((0,), dtype=np.float32)
    flattened = [resample_sequence(seq, target_frames).reshape(-1) for seq in sequences]
    distances = [
        float(np.sqrt(np.mean((a - b) ** 2)))
        for a, b in itertools.combinations(flattened, 2)
    ]
    return np.asarray(distances, dtype=np.float32)


def discover_matched_files(gt_dir: Path, transformer_dir: Path, pattern: str) -> list[str]:
    gt_files = {path.name for path in gt_dir.glob(pattern)}
    transformer_files = {path.name for path in transformer_dir.glob(pattern)}
    matched = sorted(gt_files & transformer_files)
    if not matched:
        raise RuntimeError("No matching files found across ground-truth and Delta Transformer folders.")
    missing_transformer = sorted(gt_files - transformer_files)
    if missing_transformer:
        print(f"[WARN] {len(missing_transformer)} GT files missing from transformer folder")
    return matched


def load_matched_dataset(args: argparse.Namespace) -> tuple[list[str], list[str], dict[str, list[np.ndarray]]]:
    dirs = {
        "Ground Truth": Path(args.ground_truth_dir),
        "Delta Transformer": Path(args.transformer_dir),
    }
    filenames = discover_matched_files(dirs["Ground Truth"], dirs["Delta Transformer"], args.pattern)
    data = {name: [] for name in DATASET_ORDER}
    names: list[str] | None = None

    for filename in filenames:
        loaded = {dataset: load_gesture(folder / filename) for dataset, folder in dirs.items()}
        dims = {dataset: values.shape[1] for dataset, values in loaded.items()}
        if len(set(dims.values())) != 1:
            raise ValueError(f"Feature dimension mismatch for {filename}: {dims}")
        min_len = min(values.shape[0] for values in loaded.values())
        if min_len < 2:
            raise ValueError(f"{filename} has fewer than 2 aligned frames")
        for dataset in DATASET_ORDER:
            data[dataset].append(loaded[dataset][:min_len])
        if names is None:
            names = feature_names(next(iter(loaded.values())).shape[1])

    return filenames, names or [], data


def save_figure(output_dir: Path, stem: str) -> None:
    fig = plt.gcf()
    fig.savefig(output_dir / f"{stem}.png", bbox_inches="tight", pad_inches=0.15)
    fig.savefig(output_dir / f"{stem}.pdf", bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)


def format_value(value: float) -> str:
    if value == 0:
        return "0"
    if abs(value) < 1e-3 or abs(value) >= 1e3:
        return f"{value:.2e}"
    return f"{value:.4f}".rstrip("0").rstrip(".")


def maybe_use_symlog_y(values: list[np.ndarray] | list[float]) -> None:
    finite = np.asarray(values, dtype=np.float64).reshape(-1)
    finite = finite[np.isfinite(finite)]
    positive = finite[finite > 0]
    if positive.size < 2:
        return
    dynamic_range = float(positive.max() / max(positive.min(), 1e-12))
    if dynamic_range < 100.0:
        return
    linthresh = max(float(positive.min()) / 10.0, 1e-9)
    plt.yscale("symlog", linthresh=linthresh)


def bar_with_error(
    output_dir: Path,
    stem: str,
    title: str,
    ylabel: str,
    values: dict[str, np.ndarray],
    sample_counts: dict[str, int],
) -> None:
    means = [float(np.mean(values[label])) for label in DATASET_ORDER]
    sems = [
        float(np.std(values[label], ddof=1) / np.sqrt(max(len(values[label]), 1)))
        if len(values[label]) > 1
        else 0.0
        for label in DATASET_ORDER
    ]
    x = np.arange(len(DATASET_ORDER))
    fig, ax = plt.subplots(figsize=(7.5, 6.8))
    bars = ax.bar(
        x,
        means,
        yerr=sems,
        capsize=6,
        color=[COLORS[label] for label in DATASET_ORDER],
        edgecolor="black",
        linewidth=0.7,
        error_kw={"elinewidth": 1.2, "ecolor": "black", "capthick": 1.2},
    )
    maybe_use_symlog_y(means)
    for bar, mean in zip(bars, means):
        y = bar.get_height()
        va = "bottom" if y >= 0 else "top"
        ax.annotate(
            format_value(mean),
            xy=(bar.get_x() + bar.get_width() / 2, y),
            xytext=(0, 5 if y >= 0 else -5),
            textcoords="offset points",
            ha="center",
            va=va,
            fontsize=10,
        )
    ax.set_xticks(x)
    ax.set_xticklabels([xtick_with_count(label, sample_counts[label]) for label in DATASET_ORDER])
    ax.set_ylabel(ylabel)
    ax.set_title(title, pad=14, wrap=True)
    ax.margins(x=0.15)
    bar_chart_legend(ax)
    save_figure(output_dir, stem)


def boxplot_metric(output_dir: Path, stem: str, title: str, ylabel: str, values: dict[str, np.ndarray]) -> None:
    fig, ax = plt.subplots(figsize=(7.5, 7.0))
    box = ax.boxplot(
        [values[label] for label in DATASET_ORDER],
        labels=[xtick_with_count(label, len(values[label])) for label in DATASET_ORDER],
        patch_artist=True,
        showfliers=False,
    )
    maybe_use_symlog_y([values[dataset] for dataset in DATASET_ORDER])
    for patch, label in zip(box["boxes"], DATASET_ORDER):
        patch.set_facecolor(COLORS[label])
        patch.set_alpha(0.65)
    ax.set_ylabel(ylabel)
    ax.set_title(title, pad=14, wrap=True)
    ax.margins(x=0.15)
    boxplot_legend(ax)
    save_figure(output_dir, stem)


def grouped_joint_plot(
    output_dir: Path,
    stem: str,
    title: str,
    ylabel: str,
    joint_names: list[str],
    values: dict[str, np.ndarray],
) -> None:
    x = np.arange(len(joint_names))
    width = 0.34
    fig, ax = plt.subplots(figsize=(max(11, 0.6 * len(joint_names)), 7.5))
    maybe_use_symlog_y([values[dataset] for dataset in DATASET_ORDER])
    for offset, dataset in zip([-width / 2, width / 2], DATASET_ORDER):
        ax.bar(
            x + offset,
            values[dataset],
            width,
            label=short_condition_name(dataset),
            color=COLORS[dataset],
            edgecolor="black",
            linewidth=0.4,
        )
    ax.set_xticks(x)
    ax.set_xticklabels(joint_names, rotation=45, ha="right")
    ax.set_xlabel("Joint")
    ax.set_ylabel(ylabel)
    ax.set_title(title, pad=14, wrap=True)
    place_legend_below(ax, color_legend_handles(), ncol=2, anchor_y=-0.12)
    finalize_figure(fig, ax, note=FIGURE_NOTES["joint_mean"], bottom=0.30)
    save_figure(output_dir, stem)


def overlaid_histogram(
    output_dir: Path,
    stem: str,
    title: str,
    xlabel: str,
    values: dict[str, np.ndarray],
    bins: int = 80,
) -> None:
    non_empty = [v for v in values.values() if v.size]
    if not non_empty:
        return
    upper = np.percentile(np.concatenate(non_empty), 99.5)
    hist_range = (0.0, max(float(upper), 1e-6))
    fig, ax = plt.subplots(figsize=(9, 7.0))
    for dataset in DATASET_ORDER:
        ax.hist(
            values[dataset],
            bins=bins,
            range=hist_range,
            density=True,
            histtype="step",
            linewidth=2.4,
            color=COLORS[dataset],
            label=short_condition_name(dataset),
        )
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Probability density")
    ax.set_title(title, pad=14)
    place_legend_below(ax, color_legend_handles(), ncol=2, anchor_y=-0.10)
    finalize_figure(fig, ax, note=FIGURE_NOTES["histogram"], bottom=0.30)
    save_figure(output_dir, stem)


def cdf_plot(output_dir: Path, stem: str, title: str, xlabel: str, values: dict[str, np.ndarray]) -> None:
    fig, ax = plt.subplots(figsize=(9, 7.0))
    for dataset in DATASET_ORDER:
        arr = np.sort(values[dataset])
        y = np.linspace(0.0, 1.0, len(arr), endpoint=True)
        ax.plot(arr, y, linewidth=2.4, color=COLORS[dataset], label=short_condition_name(dataset))
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Cumulative probability")
    ax.set_title(title, pad=14)
    place_legend_below(ax, color_legend_handles(), ncol=2, anchor_y=-0.10)
    finalize_figure(fig, ax, note=FIGURE_NOTES["cdf"], bottom=0.30)
    save_figure(output_dir, stem)


def pca_2d(values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    centered = values - values.mean(axis=0, keepdims=True)
    _, singular_values, vt = np.linalg.svd(centered, full_matrices=False)
    components = vt[:2].T
    explained = (singular_values[:2] ** 2) / np.maximum(np.sum(singular_values**2), 1e-12)
    return centered @ components, explained


def subsample_rows(values: np.ndarray, max_rows: int, rng: np.random.Generator) -> np.ndarray:
    if len(values) <= max_rows:
        return values
    idx = rng.choice(len(values), size=max_rows, replace=False)
    return values[idx]


def pca_plot(output_dir: Path, poses: dict[str, np.ndarray], max_points: int, seed: int) -> None:
    rng = np.random.default_rng(seed)
    sampled = {dataset: subsample_rows(poses[dataset], max_points, rng) for dataset in DATASET_ORDER}
    combined = np.concatenate([sampled[dataset] for dataset in DATASET_ORDER], axis=0)
    projected, explained = pca_2d(combined)
    split = len(sampled["Ground Truth"])
    chunks = {
        "Ground Truth": projected[:split],
        "Delta Transformer": projected[split:],
    }
    fig, ax = plt.subplots(figsize=(9, 8.0))
    for dataset in DATASET_ORDER:
        chunk = chunks[dataset]
        ax.scatter(
            chunk[:, 0],
            chunk[:, 1],
            s=7,
            alpha=0.24,
            color=COLORS[dataset],
            label=short_condition_name(dataset),
            rasterized=True,
        )
    ax.set_xlabel(f"PC1 ({100 * explained[0]:.1f}% variance explained)")
    ax.set_ylabel(f"PC2 ({100 * explained[1]:.1f}% variance explained)")
    ax.set_title("PCA Motion Coverage: Ground Truth vs Delta Transformer", pad=14)
    place_legend_below(ax, color_legend_handles(), ncol=2, anchor_y=-0.10)
    finalize_figure(fig, ax, note=FIGURE_NOTES["pca"], bottom=0.30)
    save_figure(output_dir, "10_pca_motion_coverage")


def try_welch_t(a: np.ndarray, b: np.ndarray) -> tuple[float | None, float | None]:
    try:
        from scipy import stats

        result = stats.ttest_ind(a, b, equal_var=False, nan_policy="omit")
        return float(result.statistic), float(result.pvalue)
    except Exception:
        return None, None


def try_mann_whitney(a: np.ndarray, b: np.ndarray) -> tuple[float | None, float | None]:
    try:
        from scipy import stats

        result = stats.mannwhitneyu(a, b, alternative="two-sided")
        return float(result.statistic), float(result.pvalue)
    except Exception:
        return None, None


def cohens_d(a: np.ndarray, b: np.ndarray) -> float:
    if len(a) < 2 or len(b) < 2:
        return 0.0
    pooled = np.sqrt(
        ((len(a) - 1) * np.var(a, ddof=1) + (len(b) - 1) * np.var(b, ddof=1))
        / max(len(a) + len(b) - 2, 1)
    )
    if pooled <= 1e-12:
        return 0.0
    return float((np.mean(a) - np.mean(b)) / pooled)


def statistical_tests(metrics: dict[str, dict[str, np.ndarray]]) -> dict:
    results = {}
    for metric_name, by_dataset in metrics.items():
        gt = np.asarray(by_dataset["Ground Truth"], dtype=np.float64)
        transformer = np.asarray(by_dataset["Delta Transformer"], dtype=np.float64)
        t_stat, t_p = try_welch_t(gt, transformer)
        u_stat, u_p = try_mann_whitney(gt, transformer)
        results[metric_name] = {
            "ground_truth_vs_transformer": {
                "n_ground_truth": int(len(gt)),
                "n_transformer": int(len(transformer)),
                "ground_truth_mean": float(np.mean(gt)) if len(gt) else None,
                "transformer_mean": float(np.mean(transformer)) if len(transformer) else None,
                "ground_truth_median": float(np.median(gt)) if len(gt) else None,
                "transformer_median": float(np.median(transformer)) if len(transformer) else None,
                "welch_t_statistic": t_stat,
                "welch_t_pvalue": t_p,
                "mann_whitney_u_statistic": u_stat,
                "mann_whitney_u_pvalue": u_p,
                "cohens_d_gt_minus_transformer": cohens_d(gt, transformer),
            }
        }
    return results


def metric_units() -> dict[str, str]:
    return {
        "joint_angle_range": UNIT_ANGLE,
        "mean_abs_joint_velocity": UNIT_VELOCITY,
        "peak_joint_velocity": UNIT_VELOCITY,
        "motion_energy": UNIT_ENERGY,
        "average_trajectory_variance": UNIT_ANGLE_SQ,
        "arm_joint_angle_range": UNIT_ANGLE,
        "arm_mean_abs_joint_velocity": UNIT_VELOCITY,
        "arm_motion_energy": UNIT_ENERGY,
        "pairwise_sequence_distance": UNIT_ANGLE,
        "distance_to_mean_pose": UNIT_ANGLE,
    }


def serialise_metrics(metrics: dict[str, dict[str, np.ndarray]]) -> dict:
    return {
        metric: {
            dataset: {
                "mean": float(np.mean(values)) if len(values) else None,
                "std": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
                "median": float(np.median(values)) if len(values) else None,
                "n": int(len(values)),
                "values": np.asarray(values, dtype=float).tolist(),
            }
            for dataset, values in by_dataset.items()
        }
        for metric, by_dataset in metrics.items()
    }


def print_summary_table(metrics: dict[str, dict[str, np.ndarray]], tests: dict) -> None:
    print("\n[SUMMARY] Ground Truth vs Delta Transformer")
    header = f"{'Metric':<32} {'GT mean':>12} {'Delta Tx':>12} {'Welch p':>12} {'Cohen d':>10}"
    print(header)
    print("-" * len(header))
    for metric_name, by_dataset in metrics.items():
        gt = np.mean(by_dataset["Ground Truth"])
        tr = np.mean(by_dataset["Delta Transformer"])
        row = tests[metric_name]["ground_truth_vs_transformer"]
        p_value = row.get("welch_t_pvalue")
        p_text = f"{p_value:.2e}" if p_value is not None else "n/a"
        print(
            f"{metric_name:<32} {format_value(float(gt)):>12} "
            f"{format_value(float(tr)):>12} {p_text:>12} "
            f"{row['cohens_d_gt_minus_transformer']:10.3f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate Delta Transformer gestures against ground truth")
    parser.add_argument("--ground-truth-dir", required=True)
    parser.add_argument("--transformer-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--pattern", default="*.npy")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--pca-max-points", type=int, default=25000)
    parser.add_argument("--pca-seed", type=int, default=7)
    parser.add_argument("--pairwise-resample-frames", type=int, default=120)
    args = parser.parse_args()

    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    if args.pca_max_points <= 0:
        raise ValueError("--pca-max-points must be positive")
    if args.pairwise_resample_frames <= 1:
        raise ValueError("--pairwise-resample-frames must be greater than 1")

    configure_style()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    filenames, joint_names, data = load_matched_dataset(args)
    arm_idx = arm_indices(joint_names)
    if not arm_idx:
        print("[WARN] No named NAO arm joints found; arm analysis will use all joints.")
        arm_idx = list(range(len(joint_names)))
    sample_counts = {dataset: len(data[dataset]) for dataset in DATASET_ORDER}

    seq_metrics = {dataset: {} for dataset in DATASET_ORDER}
    per_joint = {dataset: [] for dataset in DATASET_ORDER}
    arm_per_joint = {dataset: [] for dataset in DATASET_ORDER}
    velocity_values = {}
    poses = {}
    pairwise_distances = {}

    for dataset in DATASET_ORDER:
        for values in data[dataset]:
            stats = sequence_metrics(values, args.fps)
            arm_stats = sequence_metrics(values, args.fps, arm_idx)
            for key, value in stats.items():
                seq_metrics[dataset].setdefault(key, []).append(value)
            seq_metrics[dataset].setdefault("arm_joint_angle_range", []).append(arm_stats["joint_angle_range"])
            seq_metrics[dataset].setdefault("arm_mean_abs_joint_velocity", []).append(
                arm_stats["mean_abs_joint_velocity"]
            )
            seq_metrics[dataset].setdefault("arm_motion_energy", []).append(arm_stats["motion_energy"])
            per_joint[dataset].append(per_joint_arrays(values, args.fps))
            arm_per_joint[dataset].append(per_joint_arrays(values[:, arm_idx], args.fps))

        velocity_values[dataset] = np.concatenate(
            [velocity_magnitudes(values, fps=args.fps) for values in data[dataset]]
        )
        poses[dataset] = np.concatenate(data[dataset], axis=0)
        pairwise_distances[dataset] = pairwise_sequence_distances(data[dataset], args.pairwise_resample_frames)
        seq_metrics[dataset]["pairwise_sequence_distance"] = pairwise_distances[dataset].tolist()

    # Calculate Distance to Dataset Mean Pose (Mode collapse proof)
    global_mean_pose = np.mean(poses["Ground Truth"], axis=0)
    for dataset in DATASET_ORDER:
        seq_metrics[dataset]["distance_to_mean_pose"] = []
        for values in data[dataset]:
            dist = float(np.mean(np.sqrt(np.mean((values - global_mean_pose)**2, axis=1))))
            seq_metrics[dataset]["distance_to_mean_pose"].append(dist)

    metrics_for_tests = {
        metric: {
            dataset: np.asarray(seq_metrics[dataset][metric], dtype=np.float32)
            for dataset in DATASET_ORDER
        }
        for metric in seq_metrics["Ground Truth"]
    }
    tests = statistical_tests(metrics_for_tests)
    print_summary_table(metrics_for_tests, tests)

    with open(output_dir / "statistical_tests.json", "w") as f:
        json.dump(
            {
                "units": metric_units(),
                "num_matched_sequences": len(filenames),
                "matched_filenames": filenames,
                "metrics": serialise_metrics(metrics_for_tests),
                "tests": tests,
            },
            f,
            indent=2,
        )

    bar_with_error(
        output_dir,
        "01_mean_motion_energy",
        "Mean Motion Energy: Ground Truth vs Delta Transformer",
        f"Motion energy ({UNIT_ENERGY})",
        metrics_for_tests["motion_energy"],
        sample_counts,
    )
    boxplot_metric(
        output_dir,
        "02_motion_energy_boxplot",
        "Per-Sequence Motion Energy Distribution",
        f"Motion energy ({UNIT_ENERGY})",
        metrics_for_tests["motion_energy"],
    )

    joint_mean_range = {
        dataset: np.mean(np.stack([item["range"] for item in per_joint[dataset]]), axis=0)
        for dataset in DATASET_ORDER
    }
    joint_mean_velocity = {
        dataset: np.mean(np.stack([item["mean_abs_velocity"] for item in per_joint[dataset]]), axis=0)
        for dataset in DATASET_ORDER
    }
    grouped_joint_plot(
        output_dir,
        "03_jointwise_range_comparison",
        "Joint-Wise Range of Motion: Ground Truth vs Delta Transformer",
        f"Range ({UNIT_ANGLE})",
        joint_names,
        joint_mean_range,
    )
    grouped_joint_plot(
        output_dir,
        "04_jointwise_velocity_comparison",
        "Joint-Wise Mean Absolute Velocity: Ground Truth vs Delta Transformer",
        f"Mean |velocity| ({UNIT_VELOCITY})",
        joint_names,
        joint_mean_velocity,
    )

    overlaid_histogram(
        output_dir,
        "05_velocity_distribution_histogram",
        "Frame-to-Frame Joint Velocity Distribution",
        f"|joint velocity| ({UNIT_VELOCITY})",
        velocity_values,
    )
    cdf_plot(
        output_dir,
        "06_velocity_distribution_cdf",
        "CDF of Frame-to-Frame Joint Velocity Magnitudes",
        f"|joint velocity| ({UNIT_VELOCITY})",
        velocity_values,
    )

    # Diversity metrics split into their own plots for proper scale comparison
    bar_with_error(
        output_dir,
        "07_trajectory_variance",
        "Average Trajectory Variance",
        f"Variance ({UNIT_ANGLE_SQ})",
        metrics_for_tests["average_trajectory_variance"],
        sample_counts,
    )
    
    bar_with_error(
        output_dir,
        "08_pairwise_sequence_distance",
        "Cross-Sequence Diversity (Pairwise Distance)",
        f"Distance ({UNIT_ANGLE})",
        metrics_for_tests["pairwise_sequence_distance"],
        sample_counts,
    )

    bar_with_error(
        output_dir,
        "09_distance_to_mean_pose",
        "Distance to Dataset Mean Pose (Mode Collapse Proof)",
        f"Distance ({UNIT_ANGLE})",
        metrics_for_tests["distance_to_mean_pose"],
        sample_counts,
    )

    pca_plot(output_dir, poses, max_points=args.pca_max_points, seed=args.pca_seed)

    arm_joint_names = [joint_names[i] for i in arm_idx]
    arm_range = {
        dataset: np.mean(np.stack([item["range"] for item in arm_per_joint[dataset]]), axis=0)
        for dataset in DATASET_ORDER
    }
    arm_velocity = {
        dataset: np.mean(np.stack([item["mean_abs_velocity"] for item in arm_per_joint[dataset]]), axis=0)
        for dataset in DATASET_ORDER
    }
    arm_energy = {
        dataset: np.mean(np.stack([item["motion_energy"] for item in arm_per_joint[dataset]]), axis=0)
        for dataset in DATASET_ORDER
    }
    grouped_joint_plot(
        output_dir,
        "11_arm_range_of_motion",
        "Arm Joint Range of Motion",
        f"Range ({UNIT_ANGLE})",
        arm_joint_names,
        arm_range,
    )
    grouped_joint_plot(
        output_dir,
        "12_arm_joint_velocity",
        "Arm Joint Mean Absolute Velocity",
        f"Mean |velocity| ({UNIT_VELOCITY})",
        arm_joint_names,
        arm_velocity,
    )
    grouped_joint_plot(
        output_dir,
        "13_arm_motion_energy",
        "Arm Joint Motion Energy",
        f"Motion energy ({UNIT_ENERGY})",
        arm_joint_names,
        arm_energy,
    )

    with open(output_dir / "evaluation_summary.json", "w") as f:
        json.dump(
            {
                "num_matched_sequences": len(filenames),
                "fps": args.fps,
                "units": metric_units(),
                "joint_names": joint_names,
                "arm_joint_names": arm_joint_names,
                "matched_filenames": filenames,
                "figures": [
                    "01_mean_motion_energy",
                    "02_motion_energy_boxplot",
                    "03_jointwise_range_comparison",
                    "04_jointwise_velocity_comparison",
                    "05_velocity_distribution_histogram",
                    "06_velocity_distribution_cdf",
                    "07_trajectory_variance",
                    "08_pairwise_sequence_distance",
                    "09_distance_to_mean_pose",
                    "10_pca_motion_coverage",
                    "11_arm_range_of_motion",
                    "12_arm_joint_velocity",
                    "13_arm_motion_energy",
                ],
            },
            f,
            indent=2,
        )

    print(f"\n[PLOTS] Wrote PNG and PDF figures to {output_dir}")
    print(f"[PLOTS] Matched sequences: {len(filenames)}")
    print(f"[PLOTS] Statistical tests: {output_dir / 'statistical_tests.json'}")


if __name__ == "__main__":
    main()