#!/usr/bin/env python3
"""Pull a BEAT2 datapoint with ground truth and four model predictions for local playback.

Transfers, for one clip ID:
  - WAV and TextGrid sidecars
  - ground_truth.npy (from preprocessed NAO angles)
  - diffusion.npy, latent_diffusion.npy, transformers.npy, transformers_delta.npy

Run from your local machine. Files are copied from the remote GPU host via SSH/scp.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import posixpath
import random
import shlex
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent

from machine_learning.diffusion.remote_infer_pull import (  # noqa: E402
    DEFAULT_JUMP_HOST,
    DEFAULT_REMOTE,
    DEFAULT_REMOTE_BEAT,
    DEFAULT_REMOTE_PRED_DIR as DIFFUSION_PRED_DIR,
)
from machine_learning.latent_diffusion.remote_infer_pull import (  # noqa: E402
    DEFAULT_REMOTE_PRED_DIR as LATENT_DIFFUSION_PRED_DIR,
)
from machine_learning.transformers.remote_infer_pull import (  # noqa: E402
    DEFAULT_REMOTE_PRED_DIR as TRANSFORMER_PRED_DIR,
)
from machine_learning.transformers_delta.remote_infer_pull import (  # noqa: E402
    DEFAULT_REMOTE_PRED_DIR as TRANSFORMER_DELTA_PRED_DIR,
)

DEFAULT_REMOTE_PREPROCESSED = "/vol/bitbucket/ap1922/BEAT2_NAO_Preprocessed"
DEFAULT_LOCAL_DIR = "gesture_datapoints"
DEFAULT_SSH_CONTROL_PATH = "~/.ssh/robotgesturegen-%C"
DEFAULT_SSH_CONTROL_PERSIST = "10m"
DEFAULT_SERVER = "http://localhost:8000"


@dataclass(frozen=True)
class GestureSource:
    local_name: str
    remote_dir: str


GESTURE_SOURCES = (
    GestureSource("ground_truth.npy", f"{DEFAULT_REMOTE_PREPROCESSED}/clips"),
    GestureSource("diffusion.npy", DIFFUSION_PRED_DIR),
    GestureSource("latent_diffusion.npy", LATENT_DIFFUSION_PRED_DIR),
    GestureSource("transformers.npy", TRANSFORMER_PRED_DIR),
    GestureSource("transformers_delta.npy", TRANSFORMER_DELTA_PRED_DIR),
)


def printable_command(command: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in command)


def run_command(command: list[str], dry_run: bool = False, required: bool = True) -> bool:
    print(f"[CMD] {printable_command(command)}")
    if dry_run:
        return True
    result = subprocess.run(command, check=False)
    if result.returncode == 0:
        return True
    if required:
        raise subprocess.CalledProcessError(result.returncode, command)
    return False


def remote_path(base: str, *parts: str) -> str:
    path = base.rstrip("/")
    for part in parts:
        if part:
            path = posixpath.join(path, part.strip("/"))
    return path


def ssh_reuse_options(args: argparse.Namespace) -> list[str]:
    if args.no_ssh_reuse or os.name == "nt":
        return []
    return [
        "-o",
        "ControlMaster=auto",
        "-o",
        f"ControlPersist={args.ssh_control_persist}",
        "-o",
        f"ControlPath={args.ssh_control_path}",
    ]


def ensure_ssh_control_dir(args: argparse.Namespace) -> None:
    if args.no_ssh_reuse or args.dry_run:
        return
    Path(args.ssh_control_path).expanduser().parent.mkdir(parents=True, exist_ok=True)


def ssh_command(args: argparse.Namespace, remote_command: str) -> list[str]:
    command = ["ssh"]
    command.extend(ssh_reuse_options(args))
    if args.jump_host:
        command.extend(["-J", args.jump_host])
    command.extend([args.remote, remote_command])
    return command


def scp_from_remote(
    args: argparse.Namespace,
    remote_path_str: str,
    local_path: Path,
    required: bool = True,
) -> bool:
    if local_path.is_file() and not args.overwrite:
        print(f"[SKIP] Local file exists: {local_path}")
        return True
    if not args.dry_run:
        local_path.parent.mkdir(parents=True, exist_ok=True)
    command = ["scp"]
    command.extend(ssh_reuse_options(args))
    if args.jump_host:
        command.extend(["-o", f"ProxyJump={args.jump_host}"])
    command.extend([f"{args.remote}:{remote_path_str}", str(local_path)])
    return run_command(command, dry_run=args.dry_run, required=required)


def remote_gesture_path(source: GestureSource, clip_id: str) -> str:
    if source.local_name == "ground_truth.npy":
        return remote_path(source.remote_dir, f"{clip_id}.npz")
    return remote_path(source.remote_dir, f"{clip_id}.npy")


def remote_asset_paths(args: argparse.Namespace, clip_id: str) -> dict[str, str]:
    beat_root = args.remote_beat_root.rstrip("/")
    paths = {
        "wav": remote_path(beat_root, "wave16k", f"{clip_id}.wav"),
        "textgrid": remote_path(beat_root, "textgrid", f"{clip_id}.TextGrid"),
    }
    for source in GESTURE_SOURCES:
        paths[source.local_name] = remote_gesture_path(source, clip_id)
    return paths


def remote_files_exist(args: argparse.Namespace, clip_id: str) -> bool:
    paths = list(remote_asset_paths(args, clip_id).values())
    tests = " && ".join(f"test -f {shlex.quote(path)}" for path in paths)
    command = ssh_command(args, tests)
    if args.dry_run:
        print(f"[DRY-RUN] Would verify remote assets for {clip_id}")
        return True
    result = subprocess.run(command, check=False)
    return result.returncode == 0


def fetch_split_csv(args: argparse.Namespace) -> Path:
    split_csv = Path(args.split_csv) if args.split_csv else None
    if split_csv and split_csv.is_file():
        return split_csv

    if not args.remote_split_csv:
        beat_root = args.remote_beat_root.rstrip("/")
        args.remote_split_csv = remote_path(beat_root, "train_test_split.csv")

    local_csv = Path(tempfile.gettempdir()) / "beat2_train_test_split.csv"
    scp_from_remote(args, args.remote_split_csv, local_csv, required=True)
    return local_csv


def read_clip_ids(split_csv: Path, split: str) -> list[str]:
    with open(split_csv, "r", newline="") as handle:
        rows = csv.DictReader(handle)
        clip_ids = [row["id"] for row in rows if row.get("type") == split and row.get("id")]
    if not clip_ids:
        raise RuntimeError(f"No {split!r} clips found in {split_csv}")
    return clip_ids


def choose_random_clip_id(args: argparse.Namespace) -> str:
    split_csv = fetch_split_csv(args)
    clip_ids = read_clip_ids(split_csv, args.split)
    rng = random.Random(args.seed)
    rng.shuffle(clip_ids)

    attempts = min(len(clip_ids), args.random_attempts)
    for clip_id in clip_ids[:attempts]:
        if remote_files_exist(args, clip_id):
            print(f"[RANDOM] Selected clip with complete remote assets: {clip_id}")
            return clip_id
        print(f"[RANDOM] Skipping incomplete clip: {clip_id}")

    raise RuntimeError(
        f"Could not find a {args.split!r} clip with all required remote files "
        f"after checking {attempts} candidates."
    )


def resolve_clip_id(args: argparse.Namespace) -> str:
    if args.random:
        if args.clip_id:
            raise ValueError("Pass either a clip ID or --random, not both.")
        return choose_random_clip_id(args)
    if not args.clip_id:
        raise ValueError("Provide a clip ID or use --random.")
    return Path(args.clip_id).stem


def export_ground_truth(npz_path: Path, output_path: Path) -> None:
    data = np.load(str(npz_path), allow_pickle=True)
    if "nao_angles" not in data.files:
        raise ValueError(
            f"Expected 'nao_angles' in {npz_path}, found keys: {sorted(data.files)}"
        )
    angles = np.asarray(data["nao_angles"], dtype=np.float32)
    if angles.ndim != 2 or angles.shape[1] != 10:
        raise ValueError(f"Expected ground-truth angles shaped (frames, 10), got {angles.shape}")
    np.save(output_path, angles)
    print(f"[OUT] Ground truth: {output_path} ({angles.shape})")


def copy_gesture_file(
    args: argparse.Namespace,
    clip_id: str,
    source: GestureSource,
    clip_dir: Path,
) -> None:
    local_path = clip_dir / source.local_name
    if local_path.is_file() and not args.overwrite:
        print(f"[SKIP] Local gesture exists: {local_path}")
        return

    remote_src = remote_gesture_path(source, clip_id)
    if source.local_name == "ground_truth.npy":
        temp_npz = clip_dir / f"{clip_id}.npz"
        scp_from_remote(args, remote_src, temp_npz, required=True)
        if args.dry_run:
            print(f"[DRY-RUN] Would export ground truth from {temp_npz}")
            return
        export_ground_truth(temp_npz, local_path)
        if not args.keep_preprocessed_npz:
            temp_npz.unlink(missing_ok=True)
        return

    scp_from_remote(args, remote_src, local_path, required=True)


def write_manifest(args: argparse.Namespace, clip_id: str, clip_dir: Path) -> None:
    manifest = {
        "clip_id": clip_id,
        "remote": args.remote,
        "remote_beat_root": args.remote_beat_root,
        "split": args.split,
        "files": {
            "wav": str(clip_dir / f"{clip_id}.wav"),
            "textgrid": str(clip_dir / f"{clip_id}.TextGrid"),
            **{source.local_name: str(clip_dir / source.local_name) for source in GESTURE_SOURCES},
        },
        "gesture_sources": {
            source.local_name: remote_gesture_path(source, clip_id) for source in GESTURE_SOURCES
        },
    }
    manifest_path = clip_dir / "manifest.json"
    if args.dry_run:
        print(f"[DRY-RUN] Would write manifest: {manifest_path}")
        return
    with open(manifest_path, "w") as handle:
        json.dump(manifest, handle, indent=2)
    print(f"[OUT] Manifest: {manifest_path}")


def build_playback_command(
    gesture_path: Path,
    wav_path: Path,
    textgrid_path: Path,
    server: str,
    include_textgrid: bool = True,
) -> list[str]:
    command = [
        sys.executable,
        str(REPO_ROOT / "beat2_nao" / "play_nao_predictions.py"),
        str(gesture_path),
        "--server",
        server,
        "--wav",
        str(wav_path),
    ]
    if include_textgrid:
        command.extend(["--textgrid", str(textgrid_path)])
    return command


def print_playback_commands(
    clip_dir: Path,
    clip_id: str,
    server: str,
    include_textgrid: bool = True,
) -> None:
    wav_path = clip_dir / f"{clip_id}.wav"
    textgrid_path = clip_dir / f"{clip_id}.TextGrid"
    print("\n[LOCAL] Playback commands:")
    for source in GESTURE_SOURCES:
        gesture_path = clip_dir / source.local_name
        label = source.local_name.removesuffix(".npy")
        command = build_playback_command(
            gesture_path,
            wav_path,
            textgrid_path,
            server,
            include_textgrid=include_textgrid,
        )
        print(f"  {label}:")
        print(f"    {printable_command(command)}")


def fetch_datapoint(args: argparse.Namespace, clip_id: str) -> tuple[Path, bool]:
    clip_dir = Path(args.output_dir) / clip_id
    if not args.dry_run:
        clip_dir.mkdir(parents=True, exist_ok=True)

    print(f"[FETCH] Clip:      {clip_id}")
    print(f"[FETCH] Remote:    {args.remote}")
    print(f"[FETCH] Local dir: {clip_dir.resolve()}")

    if not args.random and not args.dry_run and not remote_files_exist(args, clip_id):
        missing = []
        for label, remote_file in remote_asset_paths(args, clip_id).items():
            missing.append(f"  {label}: {remote_file}")
        raise FileNotFoundError(
            "One or more required remote files are missing:\n" + "\n".join(missing)
        )

    beat_root = args.remote_beat_root.rstrip("/")
    scp_from_remote(args, remote_path(beat_root, "wave16k", f"{clip_id}.wav"), clip_dir / f"{clip_id}.wav")
    scp_from_remote(
        args,
        remote_path(beat_root, "textgrid", f"{clip_id}.TextGrid"),
        clip_dir / f"{clip_id}.TextGrid",
        required=not args.allow_missing_textgrid,
    )

    for source in GESTURE_SOURCES:
        copy_gesture_file(args, clip_id, source, clip_dir)

    write_manifest(args, clip_id, clip_dir)
    return clip_dir, not args.allow_missing_textgrid


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Pull WAV, TextGrid, ground truth, and four model gesture predictions "
            "for one BEAT2 clip to your local machine."
        )
    )
    parser.add_argument(
        "clip_id",
        nargs="?",
        help="BEAT2 clip ID, e.g. 10_kieks_0_103_103",
    )
    parser.add_argument(
        "--random",
        action="store_true",
        help="Choose a random clip from the selected split with all remote assets present",
    )
    parser.add_argument("--split", default="test", help="Split used with --random (default: test)")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for --random")
    parser.add_argument(
        "--random-attempts",
        type=int,
        default=50,
        help="How many random candidates to check before giving up",
    )
    parser.add_argument("--split-csv", default=None, help="Local train_test_split.csv path")
    parser.add_argument(
        "--remote-split-csv",
        default=None,
        help="Remote train_test_split.csv path (default: <remote-beat-root>/train_test_split.csv)",
    )

    parser.add_argument("--remote", default=DEFAULT_REMOTE)
    parser.add_argument("--jump-host", default=DEFAULT_JUMP_HOST)
    parser.add_argument("--remote-beat-root", default=DEFAULT_REMOTE_BEAT)
    parser.add_argument("--output-dir", default=DEFAULT_LOCAL_DIR)
    parser.add_argument("--server", default=DEFAULT_SERVER, help="NAO server URL for printed playback commands")

    parser.add_argument("--overwrite", action="store_true", help="Re-copy files even if they already exist locally")
    parser.add_argument("--allow-missing-textgrid", action="store_true")
    parser.add_argument(
        "--keep-preprocessed-npz",
        action="store_true",
        help="Keep the temporary preprocessed .npz after exporting ground_truth.npy",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-ssh-reuse", action="store_true")
    parser.add_argument("--ssh-control-path", default=DEFAULT_SSH_CONTROL_PATH)
    parser.add_argument("--ssh-control-persist", default=DEFAULT_SSH_CONTROL_PERSIST)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.random_attempts <= 0:
        raise ValueError("--random-attempts must be positive")

    ensure_ssh_control_dir(args)
    clip_id = resolve_clip_id(args)
    clip_dir, include_textgrid = fetch_datapoint(args, clip_id)
    print_playback_commands(clip_dir, clip_id, args.server, include_textgrid=include_textgrid)
    print(f"\n[DONE] Datapoint copied to: {clip_dir.resolve()}")


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        print(f"[ERROR] Command failed with exit code {exc.returncode}", file=sys.stderr)
        sys.exit(exc.returncode)
