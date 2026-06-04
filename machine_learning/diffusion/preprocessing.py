#!/usr/bin/env python3
"""Preprocess BEAT2/NAO windows for conditional diffusion training.

This intentionally reuses the transformer baseline preprocessing contract:
prosody + WavLM (+ optional text) conditioning, no gesture-energy features,
and LMDB windows containing ``x``, ``y``, ``speaker`` and ``valid_mask``.
The diffusion trainer consumes that same dataset layout.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from machine_learning.transformers import preprocessing as transformer_preprocessing

DEFAULT_OUTPUT_DIR = "/vol/bitbucket/ap1922/BEAT2_NAO_Diffusion_Preprocessed"

preprocess = transformer_preprocessing.preprocess
write_transformer_lmdb = transformer_preprocessing.write_transformer_lmdb
write_manifest = transformer_preprocessing.write_manifest
load_words = transformer_preprocessing.load_words


def ensure_pose_targets(argv: list[str]) -> list[str]:
    """Force diffusion preprocessing to write normalized pose targets, not deltas."""
    target_mode = None
    cleaned = [argv[0]]
    idx = 1
    while idx < len(argv):
        item = argv[idx]
        if item == "--target-mode":
            if idx + 1 >= len(argv):
                raise ValueError("--target-mode requires a value")
            target_mode = argv[idx + 1]
            idx += 2
            continue
        if item.startswith("--target-mode="):
            target_mode = item.split("=", 1)[1]
            idx += 1
            continue
        cleaned.append(item)
        idx += 1

    if target_mode is not None and target_mode != "angle":
        raise ValueError(
            "Diffusion preprocessing must use direct pose targets. "
            "Do not pass --target-mode delta; use --target-mode angle or omit it."
        )
    cleaned.extend(["--target-mode", "angle"])
    return cleaned


def main() -> None:
    transformer_preprocessing.DEFAULT_OUTPUT_DIR = DEFAULT_OUTPUT_DIR
    old_argv = sys.argv
    try:
        sys.argv = ensure_pose_targets(sys.argv)
        transformer_preprocessing.main()
    finally:
        sys.argv = old_argv


if __name__ == "__main__":
    main()
