#!/usr/bin/env python3
"""
Run NAO gesture inference on a remote GPU machine and pull the results locally.

Run this script from your local machine, inside the repository checkout that can
reach your local NAO server/simulator. It will:

  1. SSH to the remote GPU host.
  2. Run beat2_nao/infer_nao.py remotely.
  3. Copy the predicted .npy/.json and optional WAV/TextGrid files locally.
  4. Print the local playback command.

Requires local `ssh` and `scp` commands.
"""

import argparse
import shlex
import subprocess
import sys
from pathlib import Path


DEFAULT_REMOTE = "ap1922@oak21.doc.ic.ac.uk"
DEFAULT_JUMP_HOST = "shell1.doc.ic.ac.uk"
DEFAULT_REMOTE_REPO = "/homes/ap1922/Documents/ForthYear/RobotGestureGen"
DEFAULT_REMOTE_BEAT = "/vol/bitbucket/ap1922/BEAT2/beat_english_v2.0.0"
DEFAULT_REMOTE_PRED_DIR = "/vol/bitbucket/ap1922/nao_predictions"
DEFAULT_CHECKPOINT = "/vol/bitbucket/ap1922/BEAT2_NAO_Checkpoints/gesture_transformer_best.pth"
DEFAULT_DIFFUSION_CHECKPOINT = "/vol/bitbucket/ap1922/diffusion_checkpoints/diffusion_best.pth"
DEFAULT_STATS = "/vol/bitbucket/ap1922/BEAT2_NAO_Preprocessed/normalization_stats.json"
DEFAULT_SSH_CONTROL_PATH = "~/.ssh/robotgesturegen-%C"
DEFAULT_SSH_CONTROL_PERSIST = "10m"
REPO_ROOT = Path(__file__).resolve().parent.parent


def run_command(command: list[str], dry_run=False):
    printable = " ".join(shlex.quote(part) for part in command)
    print(f"[CMD] {printable}")
    if dry_run:
        return
    subprocess.run(command, check=True)


def remote_quote(value: str) -> str:
    return shlex.quote(value)


def ssh_reuse_options(args) -> list[str]:
    if args.no_ssh_reuse:
        return []
    return [
        "-o", "ControlMaster=auto",
        "-o", f"ControlPersist={args.ssh_control_persist}",
        "-o", f"ControlPath={args.ssh_control_path}",
    ]


def ensure_ssh_control_dir(args):
    if args.no_ssh_reuse or args.dry_run:
        return
    control_path = Path(args.ssh_control_path).expanduser()
    control_path.parent.mkdir(parents=True, exist_ok=True)


def ssh_command(
    remote: str,
    remote_command: str,
    jump_host: str = "",
    ssh_options: list[str] | None = None,
) -> list[str]:
    command = ["ssh"]
    if ssh_options:
        command.extend(ssh_options)
    if jump_host:
        command.extend(["-J", jump_host])
    command.extend([remote, remote_command])
    return command


def build_remote_infer_command(args, remote_output: str) -> str:
    audio_dir = f"{args.remote_beat_root}/wave16k"
    textgrid_dir = f"{args.remote_beat_root}/textgrid"
    setup = f"{args.remote_setup} && " if args.remote_setup else ""
    command = (
        f"cd {remote_quote(args.remote_repo)} && "
        f"{setup}"
        f"mkdir -p {remote_quote(args.remote_pred_dir)} && "
        f"{remote_quote(args.remote_python)} beat2_nao/infer_nao.py "
        f"--checkpoint {remote_quote(args.checkpoint)} "
        f"--stats {remote_quote(args.stats)} "
        f"--clip-id {remote_quote(args.clip_id)} "
        f"--audio-dir {remote_quote(audio_dir)} "
        f"--textgrid-dir {remote_quote(textgrid_dir)} "
        f"--fps {remote_quote(str(args.fps))} "
        f"--window-size {remote_quote(str(args.window_size))} "
        f"--stride {remote_quote(str(args.stride))} "
        f"--output {remote_quote(remote_output)}"
    )
    if args.model_type != "auto":
        command += f" --model-type {remote_quote(args.model_type)}"
    if args.speaker_id is not None:
        command += f" --speaker-id {remote_quote(args.speaker_id)}"
    if args.seed is not None:
        command += f" --seed {remote_quote(str(args.seed))}"
    if args.diffusion_deterministic:
        command += " --diffusion-deterministic"
    if args.diffusion_seed_frames is not None:
        command += f" --diffusion-seed-frames {remote_quote(str(args.diffusion_seed_frames))}"
    if args.smooth_window != 1:
        command += f" --smooth-window {remote_quote(str(args.smooth_window))}"
    if args.velocity_limit:
        command += " --velocity-limit"
    if args.velocity_scale != 1.0:
        command += f" --velocity-scale {remote_quote(str(args.velocity_scale))}"
    if args.text_cpu:
        command += " --text-cpu"
    if args.wavlm_cpu:
        command += " --wavlm-cpu"
    return command


def scp_from_remote(
    remote: str,
    remote_path: str,
    local_dir: Path,
    jump_host: str = "",
    ssh_options: list[str] | None = None,
    dry_run=False,
):
    command = ["scp"]
    if ssh_options:
        command.extend(ssh_options)
    if jump_host:
        command.extend(["-o", f"ProxyJump={jump_host}"])
    command.extend([f"{remote}:{remote_path}", str(local_dir)])
    run_command(command, dry_run=dry_run)


def copy_from_remote(args, remote_path: str, local_dir: Path):
    scp_from_remote(
        args.remote,
        remote_path,
        local_dir,
        jump_host=args.jump_host,
        ssh_options=ssh_reuse_options(args),
        dry_run=args.dry_run,
    )


def run_remote_command(args, remote_command: str):
    run_command(
        ssh_command(
            args.remote,
            remote_command,
            jump_host=args.jump_host,
            ssh_options=ssh_reuse_options(args),
        ),
        dry_run=args.dry_run,
    )


def build_playback_command(local_dir: Path, clip_id: str, server: str,
                           include_assets=True, dry_run=False) -> list[str]:
    pred = local_dir / f"{clip_id}.npy"
    wav = local_dir / f"{clip_id}.wav"
    textgrid = local_dir / f"{clip_id}.TextGrid"
    command = [
        sys.executable,
        str(REPO_ROOT / "beat2_nao" / "play_nao_predictions.py"),
        str(pred),
        "--server",
        server,
    ]
    if include_assets:
        command.extend(["--wav", str(wav), "--textgrid", str(textgrid)])
    if dry_run:
        command.append("--dry-run")
    return command


def print_playback_command(local_dir: Path, clip_id: str, server: str, include_assets=True):
    dry_run_command = build_playback_command(
        local_dir, clip_id, server, include_assets=include_assets, dry_run=True
    )
    play_command = build_playback_command(
        local_dir, clip_id, server, include_assets=include_assets, dry_run=False
    )
    print("\n[LOCAL] Dry-run playback:")
    print(" ".join(shlex.quote(part) for part in dry_run_command))
    print("\n[LOCAL] Play on NAO server:")
    print(" ".join(shlex.quote(part) for part in play_command))


def main():
    parser = argparse.ArgumentParser(
        description="Run remote BEAT/NAO inference and pull files locally"
    )
    parser.add_argument("clip_id", help="BEAT2 clip ID, e.g. 10_kieks_0_103_103")
    parser.add_argument("--remote", default=DEFAULT_REMOTE,
                        help="SSH target, e.g. ap1922@oak21.doc.ic.ac.uk")
    parser.add_argument("--jump-host", default=DEFAULT_JUMP_HOST,
                        help="SSH jump host. Set to '' to connect directly")
    parser.add_argument("--remote-repo", default=DEFAULT_REMOTE_REPO,
                        help="Repository path on the remote machine")
    parser.add_argument("--remote-beat-root", default=DEFAULT_REMOTE_BEAT,
                        help="Remote beat_english_v2.0.0 directory")
    parser.add_argument("--remote-pred-dir", default=DEFAULT_REMOTE_PRED_DIR,
                        help="Remote directory for generated prediction files")
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT,
                        help="Remote model checkpoint path")
    parser.add_argument("--diffusion-checkpoint", default=DEFAULT_DIFFUSION_CHECKPOINT,
                        help="Remote diffusion checkpoint path used when --model-type diffusion and --checkpoint is not set")
    parser.add_argument("--stats", default=DEFAULT_STATS,
                        help="Remote normalization_stats.json path")
    parser.add_argument("--model-type", choices=["auto", "transformer", "diffusion"],
                        default="auto",
                        help="Remote inference model family. Use diffusion for diffusion checkpoints")
    parser.add_argument("--fps", type=int, default=30,
                        help="Remote inference frame rate")
    parser.add_argument("--window-size", type=float, default=2.0,
                        help="Inference window size in seconds")
    parser.add_argument("--stride", type=float, default=0.5,
                        help="Inference stride in seconds")
    parser.add_argument("--seed", type=int, default=None,
                        help="Optional random seed for remote diffusion sampling")
    parser.add_argument("--diffusion-deterministic", action="store_true",
                        help="Use posterior means during remote diffusion sampling")
    parser.add_argument("--diffusion-seed-frames", type=int, default=None,
                        help="Leading generated frames reused as seed context for each diffusion window")
    parser.add_argument("--speaker-id", default=None,
                        help="Speaker id for diffusion speaker conditioning; defaults to clip id prefix remotely")
    parser.add_argument("--smooth-window", type=int, default=1,
                        help="Remote output moving-average smoothing window in frames")
    parser.add_argument("--velocity-limit", action="store_true",
                        help="Apply NAO joint velocity limiting to remote output")
    parser.add_argument("--velocity-scale", type=float, default=1.0,
                        help="Scale applied to NAO velocity limits")
    parser.add_argument("--local-dir", default="nao_predictions",
                        help="Local destination directory")
    parser.add_argument("--remote-python", default="python",
                        help="Python command on remote, e.g. python or /path/to/python")
    parser.add_argument("--remote-setup", default="",
                        help="Optional remote setup command before inference, e.g. "
                             "'source .venv/bin/activate'")
    parser.add_argument("--server", default="http://localhost:8000",
                        help="Local NAO XML-RPC server URL for printed playback command")
    parser.add_argument("--no-assets", action="store_true",
                        help="Only copy prediction .npy/.json, not WAV/TextGrid")
    parser.add_argument("--play", action="store_true",
                        help="After copying files, run local playback immediately")
    parser.add_argument("--text-cpu", action="store_true",
                        help="Run remote text embedding on CPU")
    parser.add_argument("--wavlm-cpu", action="store_true",
                        help="Run remote WavLM embedding on CPU")
    parser.add_argument("--no-ssh-reuse", action="store_true",
                        help="Disable SSH connection reuse between remote commands")
    parser.add_argument("--ssh-control-path", default=DEFAULT_SSH_CONTROL_PATH,
                        help="OpenSSH ControlPath used for connection reuse")
    parser.add_argument("--ssh-control-persist", default=DEFAULT_SSH_CONTROL_PERSIST,
                        help="How long the reusable SSH connection stays open after use")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print ssh/scp commands without running them")
    args = parser.parse_args()
    if args.model_type == "diffusion" and args.checkpoint == DEFAULT_CHECKPOINT:
        args.checkpoint = args.diffusion_checkpoint
    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    if args.window_size <= 0:
        raise ValueError("--window-size must be positive")
    if args.stride <= 0:
        raise ValueError("--stride must be positive")
    if args.diffusion_seed_frames is not None and args.diffusion_seed_frames < 0:
        raise ValueError("--diffusion-seed-frames cannot be negative")
    if args.smooth_window < 1:
        raise ValueError("--smooth-window must be at least 1")
    if args.velocity_scale <= 0:
        raise ValueError("--velocity-scale must be positive")
    ensure_ssh_control_dir(args)

    local_dir = Path(args.local_dir)
    local_dir.mkdir(parents=True, exist_ok=True)

    remote_output = f"{args.remote_pred_dir}/{args.clip_id}.npy"
    remote_command = build_remote_infer_command(args, remote_output)
    run_remote_command(args, remote_command)

    copy_from_remote(args, remote_output, local_dir)
    copy_from_remote(args, f"{args.remote_pred_dir}/{args.clip_id}.json", local_dir)

    if not args.no_assets:
        copy_from_remote(
            args,
            f"{args.remote_beat_root}/wave16k/{args.clip_id}.wav",
            local_dir,
        )
        copy_from_remote(
            args,
            f"{args.remote_beat_root}/textgrid/{args.clip_id}.TextGrid",
            local_dir,
        )

    print(f"\n[DONE] Files copied to: {local_dir.resolve()}")
    print_playback_command(
        local_dir, args.clip_id, args.server, include_assets=not args.no_assets
    )

    if args.play:
        print("\n[LOCAL] Starting playback...")
        run_command(
            build_playback_command(
                local_dir, args.clip_id, args.server, include_assets=not args.no_assets
            ),
            dry_run=args.dry_run,
        )


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        print(f"[ERROR] Command failed with exit code {exc.returncode}", file=sys.stderr)
        sys.exit(exc.returncode)
