#!/usr/bin/env python3
"""
BEATDemo.py — Play BEAT2 dataset gestures on NAO robot.

Loads SMPL-X motion data, converts to NAO joint angles via FK+IK,
and streams the motion to NAO while playing the corresponding audio.

Usage:
    python BEATDemo.py 10_kieks_0_1_1          # specific clip
    python BEATDemo.py --random                 # random clip
    python BEATDemo.py --random --no-audio      # without audio
"""

import sys
import os
import math
import argparse
import random
import time
import threading
import csv
import numpy as np
import xmlrpc.client
from pathlib import Path

# ---------------------------------------------------------------------------
# Import reusable code from existing modules
# ---------------------------------------------------------------------------
sys.path.insert(0, str(Path(__file__).parent.parent / "motion_logic"))

import main_ik_client
from main_ik_client import (
    clamp, normalize, cross_product,
    bvh_to_nao_space, solve_nao_arm_ik,
    play_audio_in_thread,
)
try:
    from .config import MOTION_DIR, AUDIO_DIR, TEXTGRID_DIR, SPLIT_CSV, MOCAP_FPS
    from .nao_constants import NAO_JOINTS, NAO_MAX_VEL
    from .parse_annotations import parse_textgrid
    from .smplx_utils import axis_angle_to_matrix
except ImportError:
    from config import MOTION_DIR, AUDIO_DIR, TEXTGRID_DIR, SPLIT_CSV, MOCAP_FPS
    from nao_constants import NAO_JOINTS, NAO_MAX_VEL
    from parse_annotations import parse_textgrid
    from smplx_utils import axis_angle_to_matrix

# ---------------------------------------------------------------------------
# SMPL-X pose slices (body joint index → poses array column)
# Body joint i → poses[3 + i*3 : 3 + (i+1)*3]
# SMPL-X coordinate system: X=Left, Y=Up, Z=Forward (same as BVH)
# ---------------------------------------------------------------------------
_S = lambda idx: slice(3 + idx * 3, 3 + (idx + 1) * 3)

POSE_SLICES = {
    "global":          slice(0, 3),
    "spine1":          _S(2),      # poses[9:12]
    "spine2":          _S(5),      # poses[18:21]
    "spine3":          _S(8),      # poses[27:30]
    "neck":            _S(11),     # poses[36:39]
    "left_collar":     _S(12),     # poses[39:42]
    "right_collar":    _S(13),     # poses[42:45]
    "head":            _S(14),     # poses[45:48]
    "left_shoulder":   _S(15),     # poses[48:51]
    "right_shoulder":  _S(16),     # poses[51:54]
    "left_elbow":      _S(17),     # poses[54:57]
    "right_elbow":     _S(18),     # poses[57:60]
}

# T-pose bone direction vectors in SMPL-X space (X=Left, Y=Up, Z=Forward)
# Left arm extends in +X, right arm in -X
BONE_UPPER_LEFT  = np.array([0.25, 0.0, 0.0])   # shoulder → elbow
BONE_LOWER_LEFT  = np.array([0.25, 0.0, 0.0])   # elbow → wrist
BONE_UPPER_RIGHT = np.array([-0.25, 0.0, 0.0])  # shoulder → elbow
BONE_LOWER_RIGHT = np.array([-0.25, 0.0, 0.0])  # elbow → wrist


def _R(frame, joint_name):
    """Get the rotation matrix for a joint from one SMPL-X pose frame."""
    return axis_angle_to_matrix(frame[POSE_SLICES[joint_name]])


def map_smplx_to_nao(poses_frame):
    """
    Convert a single SMPL-X pose frame (165,) to NAO joint angles.

    Mirrors the logic from main_ik_client.map_bvh_to_nao() but uses
    SMPL-X axis-angle rotations and kinematic chain instead of BVH Euler.
    """
    # --- HEAD & SPINE FORWARD KINEMATICS ---
    # Chain: global → spine1 → spine2 → spine3 → neck → head
    R_torso = _R(poses_frame, "global") @ _R(poses_frame, "spine1") \
              @ _R(poses_frame, "spine2") @ _R(poses_frame, "spine3")
    R_head_global = R_torso @ _R(poses_frame, "neck") @ _R(poses_frame, "head")

    # SMPL-X forward is +Z (same as BVH). Extract gaze vector.
    v_head_fwd = R_head_global @ np.array([0.0, 0.0, 1.0])
    v_head_nao = bvh_to_nao_space(v_head_fwd.tolist())

    # --- HEAD INVERSE KINEMATICS ---
    head_yaw = math.atan2(v_head_nao[1], v_head_nao[0])
    head_pitch = -math.asin(clamp(v_head_nao[2], -1.0, 1.0))

    # --- RIGHT ARM FORWARD KINEMATICS ---
    # Chain: torso → right_collar → right_shoulder
    R_upper_r = R_torso @ _R(poses_frame, "right_collar") \
                @ _R(poses_frame, "right_shoulder")
    v_upper_r = (R_upper_r @ BONE_UPPER_RIGHT).tolist()

    # Chain: upper → right_elbow
    R_lower_r = R_upper_r @ _R(poses_frame, "right_elbow")
    v_lower_r = (R_lower_r @ BONE_LOWER_RIGHT).tolist()

    # --- LEFT ARM FORWARD KINEMATICS ---
    R_upper_l = R_torso @ _R(poses_frame, "left_collar") \
                @ _R(poses_frame, "left_shoulder")
    v_upper_l = (R_upper_l @ BONE_UPPER_LEFT).tolist()

    R_lower_l = R_upper_l @ _R(poses_frame, "left_elbow")
    v_lower_l = (R_lower_l @ BONE_LOWER_LEFT).tolist()

    # --- ARM INVERSE KINEMATICS (reuse from main_ik_client) ---
    r_p, r_r, r_y, r_er = solve_nao_arm_ik(
        bvh_to_nao_space(v_upper_r), bvh_to_nao_space(v_lower_r), is_left=False
    )
    l_p, l_r, l_y, l_er = solve_nao_arm_ik(
        bvh_to_nao_space(v_upper_l), bvh_to_nao_space(v_lower_l), is_left=True
    )

    return {
        "HeadYaw":        clamp(head_yaw, -2.08, 2.08),
        "HeadPitch":      clamp(head_pitch, -0.67, 0.51),
        "RShoulderPitch": clamp(r_p, -2.08, 2.08),
        "RShoulderRoll":  clamp(r_r, -1.32, 0.31),
        "RElbowYaw":      clamp(r_y, -2.08, 2.08),
        "RElbowRoll":     clamp(r_er, 0.03, 1.54),
        "LShoulderPitch": clamp(l_p, -2.08, 2.08),
        "LShoulderRoll":  clamp(l_r, -0.31, 1.32),
        "LElbowYaw":      clamp(l_y, -2.08, 2.08),
        "LElbowRoll":     clamp(l_er, -1.54, -0.03),
    }


# ---------------------------------------------------------------------------
# Text from TextGrid (equivalent to TSVParser.get_text_for_frame_time)
# ---------------------------------------------------------------------------

def build_word_lookup(textgrid_path):
    """Parse a TextGrid and return a sorted list of (start, end, word) tuples."""
    if not textgrid_path.is_file():
        return []
    try:
        tiers = parse_textgrid(textgrid_path)
        words = tiers.get("words", [])
        return [(w["start"], w["end"], w["text"]) for w in words]
    except Exception as e:
        print(f"[WARN] Failed to parse TextGrid: {e}")
        return []


def get_text_at_time(word_list, t):
    """Return the word active at time t, or empty string."""
    for start, end, text in word_list:
        if start <= t < end:
            return text
    return ""


# ---------------------------------------------------------------------------
# Payload builder (mirrors main_ik_client.build_full_payload)
# ---------------------------------------------------------------------------

def build_payload(poses, word_list, fps=MOCAP_FPS, text_time_offset=0.0):
    """
    Pre-compute all NAO joint angles from SMPL-X poses and attach speech text.

    Args:
        poses: (T, 165) SMPL-X pose array
        word_list: sorted list of (start, end, word) tuples
        fps: frame rate of the motion data

    Returns:
        (all_times, all_angles, frame_texts) ready for XML-RPC
    """
    frame_time = 1.0 / fps
    num_frames = poses.shape[0]

    all_times = [[] for _ in NAO_JOINTS]
    all_angles = [[] for _ in NAO_JOINTS]
    frame_texts = []
    last_angles = [None] * len(NAO_JOINTS)

    print(f"[→] Pre-computing {num_frames} frames of SMPL-X → NAO IK ({fps} fps)...")

    current_time = frame_time  # start > 0 for NAO angleInterpolation
    for i in range(num_frames):
        mapped = map_smplx_to_nao(poses[i])

        # Per-joint speed limiting (same as main_ik_client)
        for j, name in enumerate(NAO_JOINTS):
            max_vel = NAO_MAX_VEL.get(name, 5.0)
            if last_angles[j] is not None:
                max_change = max_vel * frame_time
                diff = mapped[name] - last_angles[j]
                if abs(diff) > max_change:
                    mapped[name] = last_angles[j] + math.copysign(max_change, diff)
            last_angles[j] = mapped[name]

        for j, name in enumerate(NAO_JOINTS):
            all_times[j].append(current_time)
            all_angles[j].append(mapped[name])

        text = get_text_at_time(word_list, text_time_offset + i * frame_time)
        frame_texts.append(text)
        current_time += frame_time

    total_duration = all_times[0][-1] if num_frames > 0 else 0
    speech_frames = sum(1 for t in frame_texts if t)
    print(f"[✓] {num_frames} frames, {total_duration:.1f}s, {speech_frames} speech frames")

    return all_times, all_angles, frame_texts


def resolve_frame_window(num_frames, fps, start_frame=None, start_time=None,
                         num_frames_arg=None, duration=None):
    """
    Resolve optional frame/time window arguments into [start, end) frame indices.

    The returned window is clipped to the available motion range.
    """
    if start_frame is not None and start_time is not None:
        raise ValueError("Use either --start-frame or --start-time, not both")
    if num_frames_arg is not None and duration is not None:
        raise ValueError("Use either --num-frames/--max-frames or --duration, not both")

    start = int(round(start_time * fps)) if start_time is not None else (start_frame or 0)
    count = None
    if duration is not None:
        count = int(round(duration * fps))
    elif num_frames_arg is not None:
        count = num_frames_arg

    start = max(0, min(start, num_frames))
    end = num_frames if count is None else min(num_frames, start + max(0, count))
    return start, end


# ---------------------------------------------------------------------------
# Clip selection
# ---------------------------------------------------------------------------

def pick_random_clip(motion_dir=MOTION_DIR, audio_dir=AUDIO_DIR):
    """Pick a random clip ID from the BEAT2 English dataset."""
    clip_ids = []
    with open(SPLIT_CSV, "r") as f:
        for row in csv.DictReader(f):
            clip_id = row["id"]
            # Check both motion and audio exist
            if (motion_dir / f"{clip_id}.npz").is_file() and \
               (audio_dir / f"{clip_id}.wav").is_file():
                clip_ids.append(clip_id)

    if not clip_ids:
        print("[ERROR] No valid clips found with both motion and audio!")
        sys.exit(1)

    chosen = random.choice(clip_ids)
    print(f"[RANDOM] Selected: {chosen} (from {len(clip_ids)} available)")
    return chosen


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Play BEAT2 gestures on NAO")
    parser.add_argument("clip_id", nargs="?", default=None,
                        help="BEAT2 clip ID (e.g. 10_kieks_0_1_1)")
    parser.add_argument("--random", action="store_true",
                        help="Pick a random clip if no clip_id given")
    parser.add_argument("--no-audio", action="store_true",
                        help="Skip audio playback")
    parser.add_argument("--server", type=str, default="http://localhost:8000",
                        help="NAO XML-RPC server URL")
    parser.add_argument("--max-frames", type=int, default=None,
                        help="Deprecated alias for --num-frames")
    parser.add_argument("--start-frame", type=int, default=None,
                        help="Start playback from this motion frame")
    parser.add_argument("--start-time", type=float, default=None,
                        help="Start playback from this time in seconds")
    parser.add_argument("--num-frames", type=int, default=None,
                        help="Number of frames to play")
    parser.add_argument("--duration", type=float, default=None,
                        help="Duration to play in seconds")
    parser.add_argument("--motion-dir", type=str, default=str(MOTION_DIR),
                        help="Directory containing BEAT2 SMPL-X .npz files")
    parser.add_argument("--audio-dir", type=str, default=str(AUDIO_DIR),
                        help="Directory containing BEAT2 .wav files")
    parser.add_argument("--textgrid-dir", type=str, default=str(TEXTGRID_DIR),
                        help="Directory containing BEAT2 .TextGrid files")
    args = parser.parse_args()

    motion_dir = Path(args.motion_dir)
    audio_dir = Path(args.audio_dir)
    textgrid_dir = Path(args.textgrid_dir)

    # ---- Resolve clip ID ----
    if args.clip_id:
        clip_id = os.path.splitext(args.clip_id)[0]  # strip extension if given
    elif args.random:
        clip_id = pick_random_clip(motion_dir, audio_dir)
    else:
        parser.print_help()
        print("\nProvide a clip ID or use --random")
        sys.exit(1)

    # ---- Locate files ----
    npz_path = motion_dir / f"{clip_id}.npz"
    wav_path = audio_dir / f"{clip_id}.wav"
    tg_path = textgrid_dir / f"{clip_id}.TextGrid"

    if not npz_path.is_file():
        print(f"[ERROR] Motion file not found: {npz_path}")
        sys.exit(1)
    if not wav_path.is_file():
        print(f"[WARN] Audio file not found: {wav_path}")
        wav_path = None

    print(f"[LOAD] Clip: {clip_id}")
    print(f"       Motion: {npz_path.name}")
    print(f"       Audio:  {wav_path.name if wav_path else 'NONE'}")
    print(f"       TextGrid: {tg_path.name if tg_path.is_file() else 'NONE'}")

    # ---- Load motion data ----
    data = np.load(str(npz_path), allow_pickle=True)
    poses = data["poses"]  # (T, 165)
    fps = int(data["mocap_frame_rate"]) if "mocap_frame_rate" in data else MOCAP_FPS

    frame_count = args.num_frames if args.num_frames is not None else args.max_frames
    try:
        start_frame, end_frame = resolve_frame_window(
            poses.shape[0], fps,
            start_frame=args.start_frame,
            start_time=args.start_time,
            num_frames_arg=frame_count,
            duration=args.duration,
        )
    except ValueError as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    text_time_offset = start_frame / fps
    if start_frame or end_frame != poses.shape[0]:
        print(f"       Window: frames {start_frame}:{end_frame} "
              f"({text_time_offset:.2f}s → {end_frame / fps:.2f}s)")
        if wav_path and not args.no_audio:
            print("[WARN] Subclip playback does not trim audio; use --no-audio "
                  "or provide a matching clipped WAV for perfect sync.")
    poses = poses[start_frame:end_frame]

    print(f"       Frames: {poses.shape[0]} @ {fps} fps ({poses.shape[0]/fps:.1f}s)")

    # ---- Load word timings ----
    word_list = build_word_lookup(tg_path)
    print(f"       Words: {len(word_list)}")

    # ---- Compute IK payload ----
    all_times, all_angles, frame_texts = build_payload(
        poses, word_list, fps=fps, text_time_offset=text_time_offset
    )

    # ---- Connect to NAO ----
    print(f"\n[NAO] Connecting to {args.server}...")
    try:
        nao = xmlrpc.client.ServerProxy(args.server, allow_none=True)
    except Exception:
        print("[ERROR] Could not connect to NAO server. Is nao_server.py running?")
        return

    # ---- Send motion + play audio ----
    print("[→] Sending payload to NAO server...")

    if wav_path and not args.no_audio:
        play_audio_in_thread(str(wav_path))

    rpc_error = [None]

    def rpc_call():
        try:
            nao.play_motion_with_speech(
                NAO_JOINTS, all_angles, all_times, frame_texts
            )
        except Exception as e:
            rpc_error[0] = e

    rpc_thread = threading.Thread(target=rpc_call, daemon=True)
    rpc_thread.start()

    try:
        while rpc_thread.is_alive():
            rpc_thread.join(timeout=0.5)

        if rpc_error[0]:
            print(f"\n[ERROR] Playback error: {rpc_error[0]}")
        else:
            print("\n[✓] Animation complete. Putting robot to rest.")
            nao.rest()

    except KeyboardInterrupt:
        print("\n[!] Ctrl+C detected! Stopping...")
        main_ik_client._audio_stop = True
        try:
            nao2 = xmlrpc.client.ServerProxy(args.server, allow_none=True)
            nao2.stop()
        except Exception as e:
            print(f"[!] Could not send stop: {e}")


if __name__ == "__main__":
    main()
