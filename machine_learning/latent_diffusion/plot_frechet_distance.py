#!/usr/bin/env python3
"""Plot Fréchet gesture distance metrics for latent diffusion predictions."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from machine_learning import plot_style as ps  # noqa: E402
from machine_learning.frechet_gesture import (  # noqa: E402
    ModelPlotConfig,
    build_arg_parser,
    run_frechet_evaluation,
)

CONFIG = ModelPlotConfig(
    model_label="Latent Diffusion",
    short_name="Latent diffusion",
    color=ps.COLOR_LATENT_DIFFUSION,
    prediction_key="latent_diffusion",
)


def main() -> None:
    parser = build_arg_parser(
        "Plot Fréchet gesture distance for latent diffusion predictions vs ground truth",
        "--latent-diffusion-dir",
        "Directory containing latent diffusion prediction .npy files",
    )
    args = parser.parse_args()
    if args.resample_frames <= 1:
        raise ValueError("--resample-frames must be greater than 1")

    run_frechet_evaluation(
        config=CONFIG,
        ground_truth_dir=Path(args.ground_truth_dir),
        prediction_dir=Path(args.latent_diffusion_dir),
        output_dir=Path(args.output_dir),
        pattern=args.pattern,
        resample_frames=args.resample_frames,
        autoencoder_path=Path(args.autoencoder) if args.autoencoder else None,
        cpu=args.cpu,
    )


if __name__ == "__main__":
    main()
