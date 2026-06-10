#!/usr/bin/env python3
"""Pull a BEAT2 datapoint with ground truth and four model predictions for local playback.

Transfers, for one clip ID:
  - WAV and TextGrid sidecars
  - ground_truth.npy (from preprocessed NAO angles, or raw SMPL-X motion)
  - diffusion.npy, latent_diffusion.npy, transformers.npy, transformers_delta.npy

Run from your local machine. Files are staged on the remote host in one SSH call,
then copied locally in one scp transfer.
"""

from __future__ import annotations

import argparse
import json
import os
import posixpath
import random
import shlex
import subprocess
import sys
import textwrap
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO_ROOT))

from beat2_nao.fetch_nao_datapoint import load_motion_as_nao_angles  # noqa: E402

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

BITBUCKET_ROOT = "/vol/bitbucket/ap1922"
DEFAULT_REMOTE_PREPROCESSED = f"{BITBUCKET_ROOT}/BEAT2_NAO_Preprocessed"
DEFAULT_REMOTE_BUNDLE_DIR = f"{BITBUCKET_ROOT}/gesture_questions"
DEFAULT_LOCAL_DIR = "gesture_datapoints"
DEFAULT_SSH_CONTROL_PATH = "~/.ssh/robotgesturegen-%C"
DEFAULT_SSH_CONTROL_PERSIST = "10m"
DEFAULT_SERVER = "http://localhost:8000"

_REMOTE_CLIP_CACHE: dict[tuple[str, str, str, str, str], dict[str, list[str]]] = {}


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


def run_ssh(args: argparse.Namespace, remote_command: str, required: bool = True) -> subprocess.CompletedProcess:
    command = ssh_command(args, remote_command)
    print(f"[CMD] {printable_command(command)}")
    if args.dry_run:
        return subprocess.CompletedProcess(command, 0, stdout=b"[]", stderr=b"")
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    if result.returncode != 0 and required:
        stderr = result.stderr.strip() or result.stdout.strip() or "unknown error"
        raise subprocess.CalledProcessError(result.returncode, command, output=result.stdout, stderr=stderr)
    return result


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


def scp_dir_from_remote(
    args: argparse.Namespace,
    remote_dir: str,
    local_dir: Path,
    required: bool = True,
) -> bool:
    if not args.dry_run:
        local_dir.parent.mkdir(parents=True, exist_ok=True)
    command = ["scp", "-r"]
    command.extend(ssh_reuse_options(args))
    if args.jump_host:
        command.extend(["-o", f"ProxyJump={args.jump_host}"])
    command.extend([f"{args.remote}:{remote_dir.rstrip('/')}", str(local_dir)])
    return run_command(command, dry_run=args.dry_run, required=required)


def remote_gesture_path(source: GestureSource, clip_id: str) -> str:
    if source.local_name == "ground_truth.npy":
        return remote_path(source.remote_dir, f"{clip_id}.npz")
    return remote_path(source.remote_dir, f"{clip_id}.npy")


def remote_motion_dir(args: argparse.Namespace) -> str:
    if args.remote_motion_dir:
        return args.remote_motion_dir.rstrip("/")
    return remote_path(args.remote_beat_root, "smplxflame_30")


def build_remote_list_script(args: argparse.Namespace) -> str:
    beat_root = args.remote_beat_root.rstrip("/")
    bundle_dir = (args.remote_bundle_dir or "").rstrip("/")
    motion_dir = remote_motion_dir(args)
    return f"""import csv, json, os
from pathlib import Path

bitbucket = Path({BITBUCKET_ROOT!r})
beat = Path({beat_root!r})
bundle_root = Path({bundle_dir!r})
preprocessed = Path({args.remote_preprocessed_dir!r}) / "clips"
motion = Path({motion_dir!r})
pred_dirs = [
    bitbucket / "diffusion_predictions",
    bitbucket / "latent_diffusion_predictions",
    bitbucket / "transformer_predictions",
    bitbucket / "transformer_delta_predictions",
]

def stems(directory, suffix):
    if not directory.is_dir():
        return set()
    return {{path.stem for path in directory.glob(f"*{{suffix}}")}}

complete = None
for directory in pred_dirs:
    ids = stems(directory, ".npy")
    complete = ids if complete is None else complete & ids

ground_truth_ids = set()
if preprocessed.is_dir():
    ground_truth_ids |= stems(preprocessed, ".npz")
if motion.is_dir():
    ground_truth_ids |= stems(motion, ".npz")
if ground_truth_ids:
    complete &= ground_truth_ids

split = {args.split!r}
if split:
    split_csv = beat / "train_test_split.csv"
    if split_csv.is_file():
        with split_csv.open(newline="") as handle:
            allowed = {{
                row["id"]
                for row in csv.DictReader(handle)
                if row.get("type") == split and row.get("id")
            }}
        complete &= allowed

bundle_ids = set()
if bundle_root.is_dir():
    for entry in bundle_root.iterdir():
        if not entry.is_dir():
            continue
        clip_id = entry.name
        expected = [
            entry / f"{{clip_id}}.wav",
            entry / "diffusion.npy",
            entry / "latent_diffusion.npy",
            entry / "transformers.npy",
            entry / "transformers_delta.npy",
        ]
        if all(path.is_file() for path in expected):
            bundle_ids.add(clip_id)

print(json.dumps({{
    "complete": sorted(complete or []),
    "bundle": sorted(bundle_ids),
}}))
"""


def list_remote_clip_ids(args: argparse.Namespace) -> dict[str, list[str]]:
    cache_key = (
        args.remote,
        args.split,
        args.remote_bundle_dir or "",
        args.remote_preprocessed_dir,
        remote_motion_dir(args),
    )
    if cache_key in _REMOTE_CLIP_CACHE:
        return _REMOTE_CLIP_CACHE[cache_key]

    script = build_remote_list_script(args)
    quoted_script = shlex.quote(script)
    result = run_ssh(args, f"python3 -c {quoted_script}")
    try:
        payload = json.loads(result.stdout.strip() or "{}")
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Could not parse remote clip list: {result.stdout!r}") from exc
    remote_ids = {
        "complete": payload.get("complete") or [],
        "bundle": payload.get("bundle") or [],
    }
    _REMOTE_CLIP_CACHE[cache_key] = remote_ids
    return remote_ids


def choose_random_clip_id(args: argparse.Namespace) -> str:
    print("[RANDOM] Listing clips that already exist on bitbucket (one SSH call)...")
    remote_ids = list_remote_clip_ids(args)
    bundle_ids = set(remote_ids["bundle"])
    if args.prefer_bundle and bundle_ids:
        pool = sorted(bundle_ids)
        source = f"bundle dir {args.remote_bundle_dir}"
    else:
        pool = remote_ids["complete"]
        source = "prediction directories on bitbucket"

    if not pool:
        raise RuntimeError(
            f"No {args.split!r} clips with all four model predictions were found on the remote. "
            "Run inference first, or use create_gesture_questions to populate "
            f"{args.remote_bundle_dir}."
        )

    rng = random.Random(args.seed)
    clip_id = rng.choice(pool)
    print(f"[RANDOM] Selected {clip_id} from {len(pool)} ready clips in {source}")
    return clip_id


def resolve_clip_id(args: argparse.Namespace) -> str:
    if args.random:
        if args.clip_id:
            raise ValueError("Pass either a clip ID or --random, not both.")
        return choose_random_clip_id(args)
    if not args.clip_id:
        raise ValueError("Provide a clip ID or use --random.")
    return Path(args.clip_id).stem


def export_ground_truth_from_preprocessed(npz_path: Path, output_path: Path) -> None:
    data = np.load(str(npz_path), allow_pickle=True)
    if "nao_angles" not in data.files:
        raise ValueError(
            f"Expected 'nao_angles' in {npz_path}, found keys: {sorted(data.files)}"
        )
    angles = np.asarray(data["nao_angles"], dtype=np.float32)
    if angles.ndim != 2 or angles.shape[1] != 10:
        raise ValueError(f"Expected ground-truth angles shaped (frames, 10), got {angles.shape}")
    np.save(output_path, angles)
    print(f"[OUT] Ground truth from preprocessed angles: {output_path} ({angles.shape})")


def export_ground_truth_from_motion(npz_path: Path, output_path: Path, clip_id: str) -> None:
    conv_args = SimpleNamespace(
        tag=clip_id,
        no_velocity_limit=False,
        start_frame=None,
        start_time=None,
        num_frames=None,
        duration=None,
    )
    angles, metadata = load_motion_as_nao_angles(npz_path, conv_args)
    np.save(output_path, angles)
    print(
        f"[OUT] Ground truth from {metadata['source_format']}: "
        f"{output_path} ({angles.shape})"
    )


def remote_bundle_dir(args: argparse.Namespace, clip_id: str) -> str:
    return remote_path(args.remote_bundle_dir, clip_id)


def bundle_is_available(args: argparse.Namespace, clip_id: str) -> bool:
    if not args.remote_bundle_dir:
        return False
    if args.dry_run:
        return args.prefer_bundle
    remote_ids = list_remote_clip_ids(args)
    return clip_id in set(remote_ids["bundle"])


def fetch_bundle_assets(args: argparse.Namespace, clip_id: str, clip_dir: Path) -> bool:
    remote_dir = remote_bundle_dir(args, clip_id)
    print(f"[FETCH] Using existing bundle: {remote_dir}")
    scp_dir_from_remote(args, remote_dir, clip_dir)
    return True


def ensure_ground_truth_source(args: argparse.Namespace, clip_id: str, clip_dir: Path) -> None:
    """Fetch a preprocessed or raw motion .npz when the bundle path omitted ground truth."""
    if args.dry_run:
        print(f"[DRY-RUN] Would fetch ground-truth source for {clip_id}")
        return
    if (clip_dir / f"{clip_id}.npz").is_file() or (clip_dir / "motion.npz").is_file():
        return

    preprocessed_npz = clip_dir / f"{clip_id}.npz"
    motion_npz = clip_dir / "motion.npz"
    remote_preprocessed = remote_path(args.remote_preprocessed_dir, "clips", f"{clip_id}.npz")
    remote_motion = remote_path(remote_motion_dir(args), f"{clip_id}.npz")

    # Try preprocessed first, then raw BEAT2 motion.
    if scp_from_remote(args, remote_preprocessed, preprocessed_npz, required=False):
        return
    scp_from_remote(args, remote_motion, motion_npz, required=True)


def finalize_ground_truth(args: argparse.Namespace, clip_id: str, clip_dir: Path) -> None:
    local_path = clip_dir / "ground_truth.npy"
    if local_path.is_file() and not args.overwrite:
        print(f"[SKIP] Local gesture exists: {local_path}")
        return
    if args.dry_run:
        print(f"[DRY-RUN] Would finalize ground truth at {local_path}")
        return

    preprocessed_npz = clip_dir / f"{clip_id}.npz"
    motion_npz = clip_dir / "motion.npz"
    if preprocessed_npz.is_file():
        export_ground_truth_from_preprocessed(preprocessed_npz, local_path)
        if not args.keep_preprocessed_npz:
            preprocessed_npz.unlink(missing_ok=True)
    elif motion_npz.is_file():
        export_ground_truth_from_motion(motion_npz, local_path, clip_id)
        if not args.keep_motion_npz:
            motion_npz.unlink(missing_ok=True)
    elif not local_path.is_file():
        raise FileNotFoundError(
            f"No ground-truth source arrived for {clip_id}. "
            "Expected either a preprocessed .npz or raw motion.npz in the staged files."
        )


def build_remote_stage_script(args: argparse.Namespace, clip_id: str) -> str:
    beat_root = args.remote_beat_root.rstrip("/")
    motion_dir = remote_motion_dir(args)
    preprocessed_npz = remote_path(args.remote_preprocessed_dir, "clips", f"{clip_id}.npz")
    stage = f"/tmp/gesture_fetch_{clip_id}"
    return textwrap.dedent(
        f"""
        import shutil
        from pathlib import Path

        clip_id = {clip_id!r}
        stage = Path({stage!r})
        if stage.exists():
            shutil.rmtree(stage)
        stage.mkdir(parents=True)

        files = {{
            "wav": Path({remote_path(beat_root, "wave16k", f"{clip_id}.wav")!r}),
            "textgrid": Path({remote_path(beat_root, "textgrid", f"{clip_id}.TextGrid")!r}),
            "diffusion.npy": Path({remote_path(DIFFUSION_PRED_DIR, f"{clip_id}.npy")!r}),
            "latent_diffusion.npy": Path({remote_path(LATENT_DIFFUSION_PRED_DIR, f"{clip_id}.npy")!r}),
            "transformers.npy": Path({remote_path(TRANSFORMER_PRED_DIR, f"{clip_id}.npy")!r}),
            "transformers_delta.npy": Path({remote_path(TRANSFORMER_DELTA_PRED_DIR, f"{clip_id}.npy")!r}),
        }}

        for name, src in files.items():
            if not src.is_file():
                raise FileNotFoundError(f"Missing remote asset: {{src}}")
            dst = stage / (f"{{clip_id}}.wav" if name == "wav" else f"{{clip_id}}.TextGrid" if name == "textgrid" else name)
            shutil.copy2(src, dst)

        preprocessed = Path({preprocessed_npz!r})
        motion = Path({remote_path(motion_dir, f"{clip_id}.npz")!r})
        if preprocessed.is_file():
            shutil.copy2(preprocessed, stage / f"{{clip_id}}.npz")
        elif motion.is_file():
            shutil.copy2(motion, stage / "motion.npz")
        else:
            raise FileNotFoundError(
                "No ground-truth source found. Checked:\\n"
                f"  {{preprocessed}}\\n"
                f"  {{motion}}"
            )

        print(stage)
        """
    ).strip()


def stage_remote_clip(args: argparse.Namespace, clip_id: str) -> str:
    if args.dry_run:
        print(f"[DRY-RUN] Would stage remote clip assets for {clip_id}")
        return f"/tmp/gesture_fetch_{clip_id}"
    script = build_remote_stage_script(args, clip_id)
    quoted_script = shlex.quote(script)
    result = run_ssh(args, f"python3 -c {quoted_script}")
    stage_dir = result.stdout.strip().splitlines()[-1]
    if not stage_dir:
        raise RuntimeError("Remote staging did not return a stage directory path")
    print(f"[REMOTE] Staged clip at {stage_dir}")
    return stage_dir


def cleanup_remote_stage(args: argparse.Namespace, stage_dir: str) -> None:
    if args.dry_run or args.keep_remote_stage:
        return
    run_ssh(args, f"rm -rf {shlex.quote(stage_dir)}", required=False)


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
    if args.overwrite and clip_dir.exists() and not args.dry_run:
        import shutil

        shutil.rmtree(clip_dir)

    print(f"[FETCH] Clip:      {clip_id}")
    print(f"[FETCH] Remote:    {args.remote}")
    print(f"[FETCH] Local dir: {clip_dir.resolve()}")

    stage_dir = None
    try:
        if args.prefer_bundle and bundle_is_available(args, clip_id):
            fetch_bundle_assets(args, clip_id, clip_dir)
            ensure_ground_truth_source(args, clip_id, clip_dir)
        else:
            stage_dir = stage_remote_clip(args, clip_id)
            scp_dir_from_remote(args, stage_dir, clip_dir)
        finalize_ground_truth(args, clip_id, clip_dir)
    finally:
        if stage_dir:
            cleanup_remote_stage(args, stage_dir)

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
        help=(
            "Choose a random clip that already has all four model predictions on bitbucket "
            "(discovered in a single SSH call)"
        ),
    )
    parser.add_argument("--split", default="test", help="Split used with --random (default: test)")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for --random")
    parser.add_argument("--remote", default=DEFAULT_REMOTE)
    parser.add_argument("--jump-host", default=DEFAULT_JUMP_HOST)
    parser.add_argument("--remote-beat-root", default=DEFAULT_REMOTE_BEAT)
    parser.add_argument(
        "--remote-bundle-dir",
        default=DEFAULT_REMOTE_BUNDLE_DIR,
        help=(
            "Remote folder with pre-built per-clip bundles (wav, textgrid, four model .npy files). "
            "Set to '' to disable bundle copy."
        ),
    )
    parser.add_argument(
        "--remote-preprocessed-dir",
        default=DEFAULT_REMOTE_PREPROCESSED,
        help="Remote BEAT2_NAO_Preprocessed root used for ground_truth.npy export",
    )
    parser.add_argument(
        "--remote-motion-dir",
        default=None,
        help="Remote SMPL-X motion directory (default: <remote-beat-root>/smplxflame_30)",
    )
    parser.add_argument(
        "--no-prefer-bundle",
        dest="prefer_bundle",
        action="store_false",
        help="Copy wav/textgrid/predictions individually instead of using --remote-bundle-dir",
    )
    parser.add_argument("--output-dir", default=DEFAULT_LOCAL_DIR)
    parser.add_argument("--server", default=DEFAULT_SERVER, help="NAO server URL for printed playback commands")

    parser.add_argument("--overwrite", action="store_true", help="Re-copy files even if they already exist locally")
    parser.add_argument("--allow-missing-textgrid", action="store_true")
    parser.add_argument(
        "--keep-preprocessed-npz",
        action="store_true",
        help="Keep the temporary preprocessed .npz after exporting ground_truth.npy",
    )
    parser.add_argument(
        "--keep-motion-npz",
        action="store_true",
        help="Keep the temporary raw motion.npz after exporting ground_truth.npy",
    )
    parser.add_argument(
        "--keep-remote-stage",
        action="store_true",
        help="Do not delete the temporary staging directory on the remote host",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-ssh-reuse", action="store_true")
    parser.add_argument("--ssh-control-path", default=DEFAULT_SSH_CONTROL_PATH)
    parser.add_argument("--ssh-control-persist", default=DEFAULT_SSH_CONTROL_PERSIST)
    parser.set_defaults(prefer_bundle=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.remote_bundle_dir:
        args.prefer_bundle = False

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
