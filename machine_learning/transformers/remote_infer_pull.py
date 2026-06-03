#!/usr/bin/env python3
"""Run transformer inference on a remote GPU machine and pull outputs locally."""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path

DEFAULT_REMOTE = "ap1922@gpu32.doc.ic.ac.uk"
DEFAULT_JUMP_HOST = "shell1.doc.ic.ac.uk"
DEFAULT_REMOTE_REPO = "/homes/ap1922/Documents/ForthYear/RobotGestureGen"
DEFAULT_REMOTE_BEAT = "/vol/bitbucket/ap1922/BEAT2/beat_english_v2.0.0"
DEFAULT_REMOTE_PRED_DIR = "/vol/bitbucket/ap1922/transformer_predictions"
DEFAULT_CHECKPOINT = "/vol/bitbucket/ap1922/fyp_dataset/BEAT2_NAO_Transformer_Checkpoints/gesture_transformer_best.pth"
DEFAULT_STATS = "/vol/bitbucket/ap1922/fyp_dataset/BEAT2_NAO_Transformer_Preprocessed/normalization_stats.json"
DEFAULT_SSH_CONTROL_PATH = "~/.ssh/robotgesturegen-%C"
DEFAULT_SSH_CONTROL_PERSIST = "10m"
REPO_ROOT = Path(__file__).resolve().parents[2]


def run_command(command: list[str], dry_run: bool = False) -> None:
    printable = " ".join(shlex.quote(part) for part in command)
    print(f"[CMD] {printable}")
    if not dry_run:
        subprocess.run(command, check=True)


def remote_quote(value: str) -> str:
    return shlex.quote(value)


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


def ssh_command(remote: str, remote_command: str, jump_host: str, ssh_options: list[str]) -> list[str]:
    command = ["ssh"]
    command.extend(ssh_options)
    if jump_host:
        command.extend(["-J", jump_host])
    command.extend([remote, remote_command])
    return command


def build_remote_infer_command(args: argparse.Namespace, remote_output: str) -> str:
    audio_dir = f"{args.remote_beat_root}/wave16k"
    textgrid_dir = f"{args.remote_beat_root}/textgrid"
    setup = f"{args.remote_setup} && " if args.remote_setup else ""
    command = (
        f"cd {remote_quote(args.remote_repo)} && "
        f"{setup}"
        f"mkdir -p {remote_quote(args.remote_pred_dir)} && "
        f"{remote_quote(args.remote_python)} -m machine_learning.transformers.inference "
        f"--checkpoint {remote_quote(args.checkpoint)} "
        f"--stats {remote_quote(args.stats)} "
        f"--clip-id {remote_quote(args.clip_id)} "
        f"--audio-dir {remote_quote(audio_dir)} "
        f"--textgrid-dir {remote_quote(textgrid_dir)} "
        f"--fps {remote_quote(str(args.fps))} "
        f"--window-size {remote_quote(str(args.window_size))} "
        f"--stride {remote_quote(str(args.stride))} "
        f"--smooth-window {remote_quote(str(args.smooth_window))} "
        f"--output {remote_quote(remote_output)}"
    )
    if args.seed is not None:
        command += f" --seed {remote_quote(str(args.seed))}"
    if args.velocity_limit:
        command += " --velocity-limit"
    if args.velocity_scale != 1.0:
        command += f" --velocity-scale {remote_quote(str(args.velocity_scale))}"
    if args.text_cpu:
        command += " --text-cpu"
    if args.wavlm_cpu:
        command += " --wavlm-cpu"
    return command


def scp_from_remote(args: argparse.Namespace, remote_path: str, local_dir: Path) -> None:
    command = ["scp"]
    command.extend(ssh_reuse_options(args))
    if args.jump_host:
        command.extend(["-o", f"ProxyJump={args.jump_host}"])
    command.extend([f"{args.remote}:{remote_path}", str(local_dir)])
    run_command(command, dry_run=args.dry_run)


def build_playback_command(local_dir: Path, clip_id: str, server: str, include_assets=True, dry_run=False) -> list[str]:
    command = [
        sys.executable,
        str(REPO_ROOT / "BEATArc" / "play_nao_predictions.py"),
        str(local_dir / f"{clip_id}.npy"),
        "--server",
        server,
    ]
    if include_assets:
        command.extend(["--wav", str(local_dir / f"{clip_id}.wav"), "--textgrid", str(local_dir / f"{clip_id}.TextGrid")])
    if dry_run:
        command.append("--dry-run")
    return command


def print_playback_command(local_dir: Path, clip_id: str, server: str, include_assets=True) -> None:
    dry_run = build_playback_command(local_dir, clip_id, server, include_assets=include_assets, dry_run=True)
    play = build_playback_command(local_dir, clip_id, server, include_assets=include_assets, dry_run=False)
    print("\n[LOCAL] Dry-run playback:")
    print(" ".join(shlex.quote(part) for part in dry_run))
    print("\n[LOCAL] Play on NAO server:")
    print(" ".join(shlex.quote(part) for part in play))


def main() -> None:
    parser = argparse.ArgumentParser(description="Run remote transformer inference and pull files locally")
    parser.add_argument("clip_id", help="BEAT2 clip ID, e.g. 10_kieks_0_103_103")
    parser.add_argument("--remote", default=DEFAULT_REMOTE)
    parser.add_argument("--jump-host", default=DEFAULT_JUMP_HOST)
    parser.add_argument("--remote-repo", default=DEFAULT_REMOTE_REPO)
    parser.add_argument("--remote-beat-root", default=DEFAULT_REMOTE_BEAT)
    parser.add_argument("--remote-pred-dir", default=DEFAULT_REMOTE_PRED_DIR)
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument("--stats", default=DEFAULT_STATS)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--window-size", type=float, default=2.0)
    parser.add_argument("--stride", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--smooth-window", type=int, default=1)
    parser.add_argument("--velocity-limit", action="store_true")
    parser.add_argument("--velocity-scale", type=float, default=1.0)
    parser.add_argument("--local-dir", default="transformer_predictions")
    parser.add_argument("--remote-python", default="python")
    parser.add_argument("--remote-setup", default="")
    parser.add_argument("--server", default="http://localhost:8000")
    parser.add_argument("--no-assets", action="store_true")
    parser.add_argument("--play", action="store_true")
    parser.add_argument("--text-cpu", action="store_true")
    parser.add_argument("--wavlm-cpu", action="store_true")
    parser.add_argument("--no-ssh-reuse", action="store_true")
    parser.add_argument("--ssh-control-path", default=DEFAULT_SSH_CONTROL_PATH)
    parser.add_argument("--ssh-control-persist", default=DEFAULT_SSH_CONTROL_PERSIST)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    if args.window_size <= 0 or args.stride <= 0:
        raise ValueError("--window-size and --stride must be positive")
    if args.smooth_window < 1:
        raise ValueError("--smooth-window must be at least 1")
    if args.velocity_scale <= 0:
        raise ValueError("--velocity-scale must be positive")

    ensure_ssh_control_dir(args)
    local_dir = Path(args.local_dir)
    local_dir.mkdir(parents=True, exist_ok=True)
    remote_output = f"{args.remote_pred_dir}/{args.clip_id}.npy"
    remote_command = build_remote_infer_command(args, remote_output)
    run_command(
        ssh_command(args.remote, remote_command, args.jump_host, ssh_reuse_options(args)),
        dry_run=args.dry_run,
    )

    scp_from_remote(args, remote_output, local_dir)
    scp_from_remote(args, f"{args.remote_pred_dir}/{args.clip_id}.json", local_dir)
    if not args.no_assets:
        scp_from_remote(args, f"{args.remote_beat_root}/wave16k/{args.clip_id}.wav", local_dir)
        scp_from_remote(args, f"{args.remote_beat_root}/textgrid/{args.clip_id}.TextGrid", local_dir)

    print(f"\n[DONE] Files copied to: {local_dir.resolve()}")
    print_playback_command(local_dir, args.clip_id, args.server, include_assets=not args.no_assets)
    if args.play:
        print("\n[LOCAL] Starting playback...")
        run_command(
            build_playback_command(local_dir, args.clip_id, args.server, include_assets=not args.no_assets),
            dry_run=args.dry_run,
        )


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        print(f"[ERROR] Command failed with exit code {exc.returncode}", file=sys.stderr)
        sys.exit(exc.returncode)
