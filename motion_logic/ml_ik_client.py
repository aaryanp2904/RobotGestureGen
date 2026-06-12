# ml_ik_client.py - Sends ML-generated gesture predictions to NAO robot
# Converts 12-joint XYZ positions → NAO joint angles via position-based IK
#
# Usage:
#   python ml_ik_client.py predictions.npy [--wav audio.wav] [--tsv text.tsv] [--fps 30]

import xmlrpc.client
import math
import threading
import time
import os
import wave
import sys
import numpy as np
import argparse

# Joint order in predictions.npy (must match preprocessing.py SELECTED_JOINTS)
JOINT_NAMES = [
    "b_spine0",      # 0
    "b_spine3",      # 1
    "b_neck0",       # 2
    "b_head",        # 3
    "b_r_shoulder",  # 4
    "b_r_arm",       # 5
    "b_r_forearm",   # 6
    "b_r_wrist",     # 7
    "b_l_shoulder",  # 8
    "b_l_arm",       # 9
    "b_l_forearm",   # 10
    "b_l_wrist",     # 11
]

# NAO joint names for angleInterpolation
NAO_JOINT_NAMES = [
    "HeadYaw", "HeadPitch",
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
]

_audio_stop = False

# --- Vector Math ---

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def normalize(v):
    length = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    return [x / length for x in v] if length > 1e-6 else [0, 0, 1]

def cross(a, b):
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ]

def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def mocap_to_nao_space(v):
    """Motion-capture (X=Left, Y=Up, Z=Forward) → NAO (X=Fwd, Y=Left, Z=Up)."""
    return [v[2], v[0], v[1]]

# --- Arm IK (reused from main_ik_client.py) ---

def solve_nao_arm_ik(V_nao, W_nao, is_left):
    V = normalize(V_nao)
    W = normalize(W_nao)

    pitch = math.atan2(-V[2], V[0])
    roll = math.asin(clamp(V[1], -1.0, 1.0))

    d = clamp(V[0]*W[0] + V[1]*W[1] + V[2]*W[2], -1.0, 1.0)
    elbow_roll_mag = math.acos(d)

    N = normalize(cross(V, W))

    My = -math.cos(pitch)*math.sin(roll)*N[0] + math.cos(roll)*N[1] + math.sin(pitch)*math.sin(roll)*N[2]
    Mz = math.sin(pitch)*N[0] + math.cos(pitch)*N[2]

    if is_left:
        elbow_yaw = math.atan2(My, -Mz)
        elbow_roll = -elbow_roll_mag
    else:
        elbow_yaw = math.atan2(-My, Mz)
        elbow_roll = elbow_roll_mag

    return pitch, roll, elbow_yaw, elbow_roll

# --- Position-based IK ---

def positions_to_nao_angles(p):
    """
    Convert 12-joint root-relative XYZ positions to NAO joint angles.

    p: (12, 3) array — BVH world space (root-relative)
    Indices: spine0=0, spine3=1, neck=2, head=3,
             r_shoulder=4, r_arm=5, r_forearm=6, r_wrist=7,
             l_shoulder=8, l_arm=9, l_forearm=10, l_wrist=11
    """
    # --- TORSO FRAME ---
    spine_up = normalize([p[1][i] - p[0][i] for i in range(3)])        # spine3 - spine0
    torso_right = normalize([p[4][i] - p[8][i] for i in range(3)])     # r_shoulder - l_shoulder
    torso_fwd = normalize(cross(torso_right, spine_up))
    torso_right = normalize(cross(spine_up, torso_fwd))  # re-orthogonalize

    # --- HEAD ---
    # Head bone direction in torso-local frame
    head_bone = normalize([p[3][i] - p[2][i] for i in range(3)])  # head - neck
    hfwd = dot(head_bone, torso_fwd)
    hright = dot(head_bone, torso_right)
    hup = dot(head_bone, spine_up)

    # Map to NAO head angles
    # yaw = lateral deviation of head bone from vertical (positive = look left)
    # pitch = forward tilt (positive = look down on NAO)
    head_yaw = math.atan2(-hright, hup)
    head_pitch = -math.atan2(hfwd, hup)

    # --- RIGHT ARM ---
    v_upper_r = [p[6][i] - p[5][i] for i in range(3)]   # forearm - arm
    v_lower_r = [p[7][i] - p[6][i] for i in range(3)]   # wrist - forearm
    r_p, r_r, r_y, r_er = solve_nao_arm_ik(
        mocap_to_nao_space(v_upper_r), mocap_to_nao_space(v_lower_r), is_left=False
    )

    # --- LEFT ARM ---
    v_upper_l = [p[10][i] - p[9][i] for i in range(3)]  # forearm - arm
    v_lower_l = [p[11][i] - p[10][i] for i in range(3)] # wrist - forearm
    l_p, l_r, l_y, l_er = solve_nao_arm_ik(
        mocap_to_nao_space(v_upper_l), mocap_to_nao_space(v_lower_l), is_left=True
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

# --- Audio ---

def play_audio_in_thread(wav_filepath):
    global _audio_stop
    if not wav_filepath or not os.path.exists(wav_filepath):
        print(f"[!] Audio file not found: {wav_filepath}")
        return

    def _play():
        global _audio_stop
        try:
            time.sleep(1.5)
            with wave.open(wav_filepath, 'rb') as wf:
                import pyaudio
                p = pyaudio.PyAudio()
                stream = p.open(
                    format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True,
                )
                chunk = 2048
                while not _audio_stop:
                    data = wf.readframes(chunk)
                    if not data:
                        break
                    stream.write(data)
                stream.stop_stream()
                stream.close()
                p.terminate()
        except Exception as e:
            print(f"[!] Audio error: {e}")

    _audio_stop = False
    t = threading.Thread(target=_play, daemon=True)
    t.start()
    print(f"[♪] Audio started: {os.path.basename(wav_filepath)}")

# --- TSV Parser ---

def parse_tsv(filepath):
    """Parse TSV file with start_time, end_time, text columns."""
    entries = []
    with open(filepath, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split('\t')
            if len(parts) >= 3:
                try:
                    entries.append({
                        'start': float(parts[0]),
                        'end': float(parts[1]),
                        'text': parts[2],
                    })
                except ValueError:
                    continue
    entries.sort(key=lambda x: x['start'])
    return entries


def get_text_for_time(entries, t):
    """Return the speech text active at time t, or empty string."""
    for e in entries:
        if e['start'] <= t < e['end']:
            return e['text']
    return ""


# --- Main ---

def main():
    parser = argparse.ArgumentParser(description="Send ML predictions to NAO robot")
    parser.add_argument("predictions", help="Path to predictions.npy (frames, 12, 3)")
    parser.add_argument("--wav", help="Optional WAV file for audio playback")
    parser.add_argument("--tsv", help="Optional TSV file for per-frame speech text")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--server", default="http://localhost:8000")
    args = parser.parse_args()

    # Load predictions
    print(f"[→] Loading predictions from {args.predictions}")
    preds = np.load(args.predictions)
    print(f"[✓] Shape: {preds.shape} ({preds.shape[0]} frames, {preds.shape[0]/args.fps:.1f}s)")

    if preds.shape[1:] != (12, 3):
        print(f"[!] ERROR: Expected shape (N, 12, 3), got {preds.shape}")
        sys.exit(1)

    # Connect to NAO bridge
    print(f"[→] Connecting to NAO bridge at {args.server}...")
    nao = xmlrpc.client.ServerProxy(args.server, allow_none=True)

    # Convert all frames to NAO angles
    frame_time = 1.0 / args.fps
    num_frames = preds.shape[0]

    # NAO velocity limits (rad/s) × 80% safety margin
    max_vel = {
        "HeadYaw": 6.62, "HeadPitch": 5.75,
        "RShoulderPitch": 6.62, "RShoulderRoll": 5.75,
        "RElbowYaw": 6.62, "RElbowRoll": 6.62,
        "LShoulderPitch": 6.62, "LShoulderRoll": 5.75,
        "LElbowYaw": 6.62, "LElbowRoll": 6.62,
    }

    all_times = [[] for _ in NAO_JOINT_NAMES]
    all_angles = [[] for _ in NAO_JOINT_NAMES]
    last_angles = [None] * len(NAO_JOINT_NAMES)

    print(f"[→] Computing IK for {num_frames} frames...")
    current_time = frame_time

    for i in range(num_frames):
        frame_pos = preds[i]  # (12, 3)
        mapped = positions_to_nao_angles(frame_pos.tolist())

        # Per-joint velocity limiting
        for j, name in enumerate(NAO_JOINT_NAMES):
            mv = max_vel.get(name, 5.0)
            if last_angles[j] is not None:
                max_change = mv * frame_time
                diff = mapped[name] - last_angles[j]
                if abs(diff) > max_change:
                    mapped[name] = last_angles[j] + math.copysign(max_change, diff)
            last_angles[j] = mapped[name]

            all_times[j].append(current_time)
            all_angles[j].append(mapped[name])

        current_time += frame_time

        if (i + 1) % 1000 == 0:
            print(f"[→]   Frame {i+1}/{num_frames}")

    total_duration = all_times[0][-1]
    print(f"[✓] IK complete. Duration: {total_duration:.1f}s")

    # Print sample angles for sanity check
    print("\n[DEBUG] Sample angles (frame 0):")
    for j, name in enumerate(NAO_JOINT_NAMES):
        print(f"  {name:20s}: {all_angles[j][0]:+.4f} rad ({math.degrees(all_angles[j][0]):+.1f}°)")

    # Per-frame speech text from TSV
    if args.tsv and os.path.exists(args.tsv):
        tsv_entries = parse_tsv(args.tsv)
        print(f"[✓] Loaded {len(tsv_entries)} speech entries from TSV")
        frame_texts = []
        t = frame_time
        for i in range(num_frames):
            frame_texts.append(get_text_for_time(tsv_entries, t))
            t += frame_time
        speech_frames = sum(1 for txt in frame_texts if txt)
        print(f"[✓] {speech_frames}/{num_frames} frames have speech text")
    else:
        frame_texts = [""] * num_frames

    # Start audio
    if args.wav:
        play_audio_in_thread(args.wav)

    # Send to NAO
    print("\n[→] Sending to NAO server...")
    rpc_error = [None]

    def rpc_call():
        try:
            nao.play_motion_with_speech(NAO_JOINT_NAMES, all_angles, all_times, frame_texts)
        except Exception as e:
            rpc_error[0] = e

    rpc_thread = threading.Thread(target=rpc_call, daemon=True)
    rpc_thread.start()

    try:
        while rpc_thread.is_alive():
            rpc_thread.join(timeout=0.5)
        if rpc_error[0]:
            print(f"\n[!] Error: {rpc_error[0]}")
        else:
            print("\n[✓] Animation complete.")
            nao.rest()
    except KeyboardInterrupt:
        global _audio_stop
        _audio_stop = True
        print("\n[!] Ctrl+C — stopping...")
        try:
            nao2 = xmlrpc.client.ServerProxy(args.server, allow_none=True)
            nao2.stop()
        except Exception:
            pass

if __name__ == "__main__":
    main()
