#!/usr/bin/env python3
"""
Fetch a BEAT2 datapoint from the remote dataset and export it in NAO space.

Given a datapoint tag/clip ID, this script copies the remote motion, WAV, and
optional TextGrid files to the local machine, converts the motion into the
shared NAO_JOINTS angle order, and can play it through the local NAO server.
"""

import argparse
import json
import math
import shlex
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent

try:
    from .BEATDemo import map_smplx_to_nao, resolve_frame_window
    from .config import MOCAP_FPS
    from .nao_constants import NAO_JOINTS, NAO_LIMITS, NAO_MAX_VEL
except ImportError:
    sys.path.insert(0, str(REPO_ROOT))
    from BEATArc.BEATDemo import map_smplx_to_nao, resolve_frame_window
    from BEATArc.config import MOCAP_FPS
    from BEATArc.nao_constants import NAO_JOINTS, NAO_LIMITS, NAO_MAX_VEL


DEFAULT_REMOTE = "ap1922@oak21.doc.ic.ac.uk"
DEFAULT_JUMP_HOST = "shell1.doc.ic.ac.uk"
DEFAULT_REMOTE_BEAT = "/vol/bitbucket/ap1922/BEAT2/beat_english_v2.0.0"
DEFAULT_LOCAL_DIR = "nao_datapoints"


def printable_command(command: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in command)


def run_command(command: list[str], dry_run=False, required=True) -> bool:
    print(f"[CMD] {printable_command(command)}")
    if dry_run:
        return True
    result = subprocess.run(command, check=False)
    if result.returncode == 0:
        return True
    if required:
        raise subprocess.CalledProcessError(result.returncode, command)
    return False


def scp_from_remote(remote: str, remote_path: str, local_path: Path,
                    jump_host: str = "", dry_run=False, required=True) -> bool:
    command = ["scp"]
    if jump_host:
        command.extend(["-o", f"ProxyJump={jump_host}"])
    command.extend([f"{remote}:{remote_path}", str(local_path)])
    return run_command(command, dry_run=dry_run, required=required)


def copy_remote_asset(args, remote_path: str, local_path: Path, required=True) -> bool:
    if local_path.is_file() and not args.overwrite:
        print(f"[SKIP] Local file exists: {local_path}")
        return True
    if not args.dry_run:
        local_path.parent.mkdir(parents=True, exist_ok=True)
    copied = scp_from_remote(
        args.remote,
        remote_path,
        local_path,
        jump_host=args.jump_host,
        dry_run=args.dry_run,
        required=required,
    )
    if not copied:
        print(f"[WARN] Optional remote file unavailable: {remote_path}")
    return copied


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_nao_angles(angles: np.ndarray) -> np.ndarray:
    out = angles.copy()
    for joint_idx, name in enumerate(NAO_JOINTS):
        low, high = NAO_LIMITS[name]
        out[:, joint_idx] = np.clip(out[:, joint_idx], low, high)
    return out


def limit_nao_velocity(angles: np.ndarray, fps: float) -> np.ndarray:
    if len(angles) <= 1:
        return angles
    out = angles.copy()
    frame_time = 1.0 / fps
    for frame_idx in range(1, len(out)):
        for joint_idx, name in enumerate(NAO_JOINTS):
            max_change = NAO_MAX_VEL.get(name, 5.0) * frame_time
            diff = out[frame_idx, joint_idx] - out[frame_idx - 1, joint_idx]
            if abs(diff) > max_change:
                out[frame_idx, joint_idx] = (
                    out[frame_idx - 1, joint_idx] + math.copysign(max_change, diff)
                )
    return out


def smplx_poses_to_nao_angles(poses: np.ndarray, fps: float,
                              velocity_limit=True) -> np.ndarray:
    angles = np.zeros((poses.shape[0], len(NAO_JOINTS)), dtype=np.float32)
    last = None
    frame_time = 1.0 / fps

    print(f"[CONVERT] SMPL-X poses -> NAO angles: {poses.shape[0]} frames @ {fps:g} fps")
    for frame_idx, pose_frame in enumerate(poses):
        mapped = map_smplx_to_nao(pose_frame)
        row = np.array([mapped[name] for name in NAO_JOINTS], dtype=np.float32)
        if velocity_limit and last is not None:
            for joint_idx, name in enumerate(NAO_JOINTS):
                max_change = NAO_MAX_VEL.get(name, 5.0) * frame_time
                diff = row[joint_idx] - last[joint_idx]
                if abs(diff) > max_change:
                    row[joint_idx] = last[joint_idx] + math.copysign(max_change, diff)
        angles[frame_idx] = row
        last = row.copy()

    return clamp_nao_angles(angles).astype(np.float32)


def get_scalar(data, key: str, default):
    if key not in data:
        return default
    value = data[key]
    if hasattr(value, "item"):
        return value.item()
    return value


def load_motion_as_nao_angles(motion_path: Path, args) -> tuple[np.ndarray, dict]:
    data = np.load(str(motion_path), allow_pickle=True)
    keys = set(data.files)

    if "nao_angles" in keys:
        fps = float(get_scalar(data, "fps", MOCAP_FPS))
        angles = np.asarray(data["nao_angles"], dtype=np.float32)
        source_format = "nao_preprocessed_npz"
        print(f"[LOAD] Found preprocessed NAO angles: {angles.shape}")
    elif "poses" in keys:
        fps = float(get_scalar(data, "mocap_frame_rate", MOCAP_FPS))
        poses = np.asarray(data["poses"], dtype=np.float32)
        if poses.ndim != 2 or poses.shape[1] != 165:
            raise ValueError(f"Expected SMPL-X poses shaped (frames, 165), got {poses.shape}")
        source_format = "beat2_smplx_npz"
        angles = smplx_poses_to_nao_angles(
            poses, fps, velocity_limit=not args.no_velocity_limit
        )
    else:
        raise ValueError(
            f"Unsupported motion file {motion_path}; expected 'poses' or 'nao_angles'. "
            f"Available keys: {sorted(keys)}"
        )

    if angles.ndim != 2 or angles.shape[1] != len(NAO_JOINTS):
        raise ValueError(
            f"Expected NAO angles shaped (frames, {len(NAO_JOINTS)}), got {angles.shape}"
        )

    start_frame, end_frame = resolve_frame_window(
        angles.shape[0],
        fps,
        start_frame=args.start_frame,
        start_time=args.start_time,
        num_frames_arg=args.num_frames,
        duration=args.duration,
    )
    if start_frame or end_frame != angles.shape[0]:
        print(f"[WINDOW] Frames {start_frame}:{end_frame} "
              f"({start_frame / fps:.2f}s -> {end_frame / fps:.2f}s)")
        angles = angles[start_frame:end_frame]

    angles = clamp_nao_angles(angles).astype(np.float32)
    if not args.no_velocity_limit and source_format == "nao_preprocessed_npz":
        angles = limit_nao_velocity(angles, fps).astype(np.float32)

    metadata = {
        "tag": args.tag,
        "motion_source": str(motion_path),
        "source_format": source_format,
        "fps": fps,
        "start_frame": start_frame,
        "end_frame": end_frame,
        "num_frames": int(angles.shape[0]),
        "duration_sec": float(angles.shape[0] / fps),
        "nao_joint_names": NAO_JOINTS,
        "velocity_limit": not args.no_velocity_limit,
    }
    return angles, metadata


def write_outputs(args, local_paths: dict[str, Path]) -> dict:
    angles, metadata = load_motion_as_nao_angles(local_paths["motion"], args)
    output_path = local_paths["nao"]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(output_path, angles)

    metadata["wav"] = str(local_paths["wav"]) if local_paths["wav"].is_file() else None
    metadata["textgrid"] = (
        str(local_paths["textgrid"]) if local_paths["textgrid"].is_file() else None
    )
    metadata_path = output_path.with_suffix(".json")
    with open(metadata_path, "w") as f:
        json.dump(metadata, f, indent=2)

    print(f"[OUT] NAO angles: {output_path}")
    print(f"[OUT] Metadata:   {metadata_path}")
    print(f"[OUT] Shape:      {angles.shape}")
    print(f"[OUT] WAV:        {metadata['wav'] or 'NONE'}")
    print(f"[OUT] TextGrid:   {metadata['textgrid'] or 'NONE'}")
    return metadata


def build_playback_command(args, local_paths: dict[str, Path], metadata: dict) -> list[str]:
    command = [
        sys.executable,
        str(REPO_ROOT / "BEATArc" / "play_nao_predictions.py"),
        str(local_paths["nao"]),
        "--server",
        args.server,
    ]
    if args.playback_fps is not None:
        command.extend(["--fps", str(args.playback_fps)])
    if local_paths["wav"].is_file():
        command.extend(["--wav", str(local_paths["wav"])])
    if not args.no_textgrid and local_paths["textgrid"].is_file():
        command.extend(["--textgrid", str(local_paths["textgrid"])])
    if args.start_frame is not None:
        fps = float(metadata.get("fps", MOCAP_FPS))
        command.extend(["--time-offset", str(args.start_frame / fps)])
    elif args.start_time is not None:
        command.extend(["--time-offset", str(args.start_time)])
    if args.no_audio:
        command.append("--no-audio")
    if args.no_velocity_limit:
        command.append("--no-velocity-limit")
    return command


def resolve_paths(args) -> dict[str, Path]:
    tag = Path(args.tag).stem
    out_dir = Path(args.local_dir) / tag
    return {
        "dir": out_dir,
        "motion": out_dir / f"{tag}.npz",
        "wav": out_dir / f"{tag}.wav",
        "textgrid": out_dir / f"{tag}.TextGrid",
        "nao": out_dir / f"{tag}.npy",
    }


def remote_dirs(args) -> tuple[str, str, str]:
    motion_dir = args.remote_motion_dir or f"{args.remote_beat_root}/smplxflame_30"
    audio_dir = args.remote_audio_dir or f"{args.remote_beat_root}/wave16k"
    textgrid_dir = args.remote_textgrid_dir or f"{args.remote_beat_root}/textgrid"
    return motion_dir.rstrip("/"), audio_dir.rstrip("/"), textgrid_dir.rstrip("/")


def main():
    parser = argparse.ArgumentParser(
        description="Fetch a remote BEAT2 datapoint and convert its motion to NAO space"
    )
    parser.add_argument("tag", help="Datapoint tag / BEAT2 clip ID, e.g. 10_kieks_0_103_103")
    parser.add_argument("--remote", default=DEFAULT_REMOTE,
                        help="SSH target, e.g. ap1922@oak21.doc.ic.ac.uk")
    parser.add_argument("--jump-host", default=DEFAULT_JUMP_HOST,
                        help="SSH jump host. Set to '' to connect directly")
    parser.add_argument("--remote-beat-root", default=DEFAULT_REMOTE_BEAT,
                        help="Remote beat_english_v2.0.0 root")
    parser.add_argument("--remote-motion-dir", default=None,
                        help="Remote directory containing motion .npz files")
    parser.add_argument("--remote-audio-dir", default=None,
                        help="Remote directory containing WAV files")
    parser.add_argument("--remote-textgrid-dir", default=None,
                        help="Remote directory containing TextGrid files")
    parser.add_argument("--local-dir", default=DEFAULT_LOCAL_DIR,
                        help="Local output directory")
    parser.add_argument("--server", default="http://localhost:8000",
                        help="Local NAO XML-RPC server URL")
    parser.add_argument("--start-frame", type=int, default=None)
    parser.add_argument("--start-time", type=float, default=None)
    parser.add_argument("--num-frames", type=int, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--playback-fps", type=float, default=None,
                        help="Override playback FPS; defaults to exported metadata")
    parser.add_argument("--no-textgrid", action="store_true",
                        help="Do not copy the TextGrid sidecar")
    parser.add_argument("--no-audio", action="store_true",
                        help="Fetch the WAV but do not play audio when --play is used")
    parser.add_argument("--no-velocity-limit", action="store_true",
                        help="Do not apply NAO velocity limiting during export/playback")
    parser.add_argument("--overwrite", action="store_true",
                        help="Re-copy remote files even if local copies exist")
    parser.add_argument("--play", action="store_true",
                        help="Run the converted datapoint on the local NAO server")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print scp/playback commands without running them")
    args = parser.parse_args()

    local_paths = resolve_paths(args)
    motion_dir, audio_dir, textgrid_dir = remote_dirs(args)
    tag = Path(args.tag).stem

    print(f"[FETCH] Tag:        {tag}")
    print(f"[FETCH] Remote:     {args.remote}")
    print(f"[FETCH] Local dir:  {local_paths['dir']}")

    copy_remote_asset(args, f"{motion_dir}/{tag}.npz", local_paths["motion"], required=True)
    copy_remote_asset(args, f"{audio_dir}/{tag}.wav", local_paths["wav"], required=True)
    if not args.no_textgrid:
        copy_remote_asset(
            args,
            f"{textgrid_dir}/{tag}.TextGrid",
            local_paths["textgrid"],
            required=False,
        )

    if args.dry_run:
        print("[DRY-RUN] Skipping conversion because remote files were not copied.")
        return

    metadata = write_outputs(args, local_paths)

    playback_command = build_playback_command(args, local_paths, metadata)
    print("\n[LOCAL] Playback command:")
    print(printable_command(playback_command))

    if args.play:
        if (args.start_frame is not None or args.start_time is not None) and not args.no_audio:
            print("[WARN] Subclip playback uses the full WAV; pass --no-audio or provide a clipped WAV "
                  "if exact audio sync matters.")
        run_command(playback_command, dry_run=False, required=True)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        print(f"[ERROR] Command failed with exit code {exc.returncode}", file=sys.stderr)
        sys.exit(exc.returncode)
