#!/usr/bin/env python3
"""Evaluate diffusion gesture predictions against ground-truth motion."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from machine_learning.transformers import plot_graphs as plots

DATASET_ORDER = ["Ground Truth", "Diffusion"]
COLORS = {"Ground Truth": "#2E86AB", "Diffusion": "#7B2CBF"}


def load_matched_dataset(args: argparse.Namespace) -> tuple[list[str], list[str], dict[str, list[np.ndarray]]]:
    gt_dir = Path(args.ground_truth_dir)
    diffusion_dir = Path(args.diffusion_dir)
    gt_files = {path.name for path in gt_dir.glob(args.pattern)}
    diffusion_files = {path.name for path in diffusion_dir.glob(args.pattern)}
    filenames = sorted(gt_files & diffusion_files)
    if not filenames:
        raise RuntimeError("No matching files found across ground-truth and diffusion folders.")
    missing_diffusion = sorted(gt_files - diffusion_files)
    if missing_diffusion:
        print(f"[WARN] {len(missing_diffusion)} GT files missing from diffusion folder")

    data = {name: [] for name in DATASET_ORDER}
    names: list[str] | None = None
    for filename in filenames:
        loaded = {
            "Ground Truth": plots.load_gesture(gt_dir / filename),
            "Diffusion": plots.load_gesture(diffusion_dir / filename),
        }
        dims = {dataset: values.shape[1] for dataset, values in loaded.items()}
        if len(set(dims.values())) != 1:
            raise ValueError(f"Feature dimension mismatch for {filename}: {dims}")
        min_len = min(values.shape[0] for values in loaded.values())
        if min_len < 2:
            raise ValueError(f"{filename} has fewer than 2 aligned frames")
        for dataset in DATASET_ORDER:
            data[dataset].append(loaded[dataset][:min_len])
        if names is None:
            names = plots.feature_names(next(iter(loaded.values())).shape[1])
    return filenames, names or [], data


def statistical_tests(metrics: dict[str, dict[str, np.ndarray]]) -> dict:
    results = {}
    for metric_name, by_dataset in metrics.items():
        gt = np.asarray(by_dataset["Ground Truth"], dtype=np.float64)
        diffusion = np.asarray(by_dataset["Diffusion"], dtype=np.float64)
        t_stat, t_p = plots.try_welch_t(gt, diffusion)
        u_stat, u_p = plots.try_mann_whitney(gt, diffusion)
        results[metric_name] = {
            "ground_truth_vs_diffusion": {
                "n_ground_truth": int(len(gt)),
                "n_diffusion": int(len(diffusion)),
                "ground_truth_mean": float(np.mean(gt)) if len(gt) else None,
                "diffusion_mean": float(np.mean(diffusion)) if len(diffusion) else None,
                "ground_truth_median": float(np.median(gt)) if len(gt) else None,
                "diffusion_median": float(np.median(diffusion)) if len(diffusion) else None,
                "welch_t_statistic": t_stat,
                "welch_t_pvalue": t_p,
                "mann_whitney_u_statistic": u_stat,
                "mann_whitney_u_pvalue": u_p,
                "cohens_d_gt_minus_diffusion": plots.cohens_d(gt, diffusion),
            }
        }
    return results


def print_summary_table(metrics: dict[str, dict[str, np.ndarray]], tests: dict) -> None:
    print("\n[SUMMARY] Ground Truth vs Diffusion")
    header = f"{'Metric':<32} {'GT mean':>12} {'Diffusion':>12} {'Welch p':>12} {'Cohen d':>10}"
    print(header)
    print("-" * len(header))
    for metric_name, by_dataset in metrics.items():
        gt = np.mean(by_dataset["Ground Truth"])
        diffusion = np.mean(by_dataset["Diffusion"])
        row = tests[metric_name]["ground_truth_vs_diffusion"]
        p_value = row.get("welch_t_pvalue")
        p_text = f"{p_value:.2e}" if p_value is not None else "n/a"
        print(
            f"{metric_name:<32} {gt:12.4f} {diffusion:12.4f} "
            f"{p_text:>12} {row['cohens_d_gt_minus_diffusion']:10.3f}"
        )


def pca_plot(output_dir: Path, poses: dict[str, np.ndarray], max_points: int, seed: int) -> None:
    rng = np.random.default_rng(seed)
    sampled = {dataset: plots.subsample_rows(poses[dataset], max_points, rng) for dataset in DATASET_ORDER}
    combined = np.concatenate([sampled[dataset] for dataset in DATASET_ORDER], axis=0)
    projected, explained = plots.pca_2d(combined)
    split = len(sampled["Ground Truth"])
    chunks = {
        "Ground Truth": projected[:split],
        "Diffusion": projected[split:],
    }
    plots.plt.figure(figsize=(9, 7))
    for dataset in DATASET_ORDER:
        chunk = chunks[dataset]
        plots.plt.scatter(
            chunk[:, 0],
            chunk[:, 1],
            s=7,
            alpha=0.24,
            color=COLORS[dataset],
            label=f"{dataset} (poses={len(chunk):,})",
            rasterized=True,
        )
    plots.plt.xlabel(f"PC1 ({100 * explained[0]:.1f}% variance)")
    plots.plt.ylabel(f"PC2 ({100 * explained[1]:.1f}% variance)")
    plots.plt.title("PCA Motion Coverage: Ground Truth vs Diffusion")
    plots.plt.legend(markerscale=2)
    plots.save_figure(output_dir, "10_pca_motion_coverage")


def use_diffusion_plot_labels() -> None:
    plots.DATASET_ORDER = DATASET_ORDER
    plots.DATASET_KEYS = {"Ground Truth": "ground_truth", "Diffusion": "diffusion"}
    plots.COLORS = COLORS


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate diffusion gestures against ground truth")
    parser.add_argument("--ground-truth-dir", required=True)
    parser.add_argument("--diffusion-dir", required=True)
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

    use_diffusion_plot_labels()
    plots.configure_style()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    filenames, joint_names, data = load_matched_dataset(args)
    arm_idx = plots.arm_indices(joint_names)
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
            stats = plots.sequence_metrics(values, args.fps)
            arm_stats = plots.sequence_metrics(values, args.fps, arm_idx)
            for key, value in stats.items():
                seq_metrics[dataset].setdefault(key, []).append(value)
            seq_metrics[dataset].setdefault("arm_joint_angle_range", []).append(arm_stats["joint_angle_range"])
            seq_metrics[dataset].setdefault("arm_mean_abs_joint_velocity", []).append(
                arm_stats["mean_abs_joint_velocity"]
            )
            seq_metrics[dataset].setdefault("arm_motion_energy", []).append(arm_stats["motion_energy"])
            per_joint[dataset].append(plots.per_joint_arrays(values, args.fps))
            arm_per_joint[dataset].append(plots.per_joint_arrays(values[:, arm_idx], args.fps))

        velocity_values[dataset] = np.concatenate(
            [plots.velocity_magnitudes(values, fps=args.fps) for values in data[dataset]]
        )
        poses[dataset] = np.concatenate(data[dataset], axis=0)
        pairwise_distances[dataset] = plots.pairwise_sequence_distances(
            data[dataset], args.pairwise_resample_frames
        )
        seq_metrics[dataset]["pairwise_sequence_distance"] = pairwise_distances[dataset].tolist()

    global_mean_pose = np.mean(poses["Ground Truth"], axis=0)
    for dataset in DATASET_ORDER:
        seq_metrics[dataset]["distance_to_mean_pose"] = []
        for values in data[dataset]:
            dist = float(np.mean(np.sqrt(np.mean((values - global_mean_pose) ** 2, axis=1))))
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
                "units": plots.metric_units(),
                "num_matched_sequences": len(filenames),
                "matched_filenames": filenames,
                "metrics": plots.serialise_metrics(metrics_for_tests),
                "tests": tests,
            },
            f,
            indent=2,
        )

    plots.bar_with_error(
        output_dir,
        "01_mean_motion_energy",
        "Mean Motion Energy: Ground Truth vs Diffusion",
        f"Motion energy ({plots.UNIT_ENERGY})",
        metrics_for_tests["motion_energy"],
        sample_counts,
    )
    plots.boxplot_metric(
        output_dir,
        "02_motion_energy_boxplot",
        "Per-Sequence Motion Energy Distribution",
        f"Motion energy ({plots.UNIT_ENERGY})",
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
    plots.grouped_joint_plot(
        output_dir,
        "03_jointwise_range_comparison",
        "Joint-Wise Range of Motion: Ground Truth vs Diffusion",
        f"Range ({plots.UNIT_ANGLE})",
        joint_names,
        joint_mean_range,
    )
    plots.grouped_joint_plot(
        output_dir,
        "04_jointwise_velocity_comparison",
        "Joint-Wise Mean Absolute Velocity: Ground Truth vs Diffusion",
        f"Mean |velocity| ({plots.UNIT_VELOCITY})",
        joint_names,
        joint_mean_velocity,
    )

    plots.overlaid_histogram(
        output_dir,
        "05_velocity_distribution_histogram",
        "Frame-to-Frame Joint Velocity Distribution",
        f"|joint velocity| ({plots.UNIT_VELOCITY})",
        velocity_values,
    )
    plots.cdf_plot(
        output_dir,
        "06_velocity_distribution_cdf",
        "CDF of Frame-to-Frame Joint Velocity Magnitudes",
        f"|joint velocity| ({plots.UNIT_VELOCITY})",
        velocity_values,
    )

    plots.bar_with_error(
        output_dir,
        "07_trajectory_variance",
        "Average Trajectory Variance",
        f"Variance ({plots.UNIT_ANGLE_SQ})",
        metrics_for_tests["average_trajectory_variance"],
        sample_counts,
    )
    plots.bar_with_error(
        output_dir,
        "08_pairwise_sequence_distance",
        "Cross-Sequence Diversity (Pairwise Distance)",
        f"Distance ({plots.UNIT_ANGLE})",
        metrics_for_tests["pairwise_sequence_distance"],
        sample_counts,
    )
    plots.bar_with_error(
        output_dir,
        "09_distance_to_mean_pose",
        "Distance to Dataset Mean Pose (Mode Collapse Proof)",
        f"Distance ({plots.UNIT_ANGLE})",
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
    plots.grouped_joint_plot(
        output_dir,
        "11_arm_range_of_motion",
        "Arm Joint Range of Motion",
        f"Range ({plots.UNIT_ANGLE})",
        arm_joint_names,
        arm_range,
    )
    plots.grouped_joint_plot(
        output_dir,
        "12_arm_joint_velocity",
        "Arm Joint Mean Absolute Velocity",
        f"Mean |velocity| ({plots.UNIT_VELOCITY})",
        arm_joint_names,
        arm_velocity,
    )
    plots.grouped_joint_plot(
        output_dir,
        "13_arm_motion_energy",
        "Arm Joint Motion Energy",
        f"Motion energy ({plots.UNIT_ENERGY})",
        arm_joint_names,
        arm_energy,
    )

    with open(output_dir / "evaluation_summary.json", "w") as f:
        json.dump(
            {
                "num_matched_sequences": len(filenames),
                "fps": args.fps,
                "units": plots.metric_units(),
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
    print(f"[PLOTS] Summary: {output_dir / 'evaluation_summary.json'}")
    print(f"[PLOTS] Statistical tests: {output_dir / 'statistical_tests.json'}")


if __name__ == "__main__":
    main()
