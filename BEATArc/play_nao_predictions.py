#!/usr/bin/env python3
"""
Play model-generated NAO joint-angle predictions on a local NAO server.

This script is intended for the local machine that can reach the NAO simulator
or robot server. It does not need GPU access or the BEAT2 dataset; it only needs
the predicted .npy file and optional WAV/TextGrid sidecar files.

Expected prediction shape:
  (frames, 10), columns in NAO_JOINTS order.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
import wave
import xmlrpc.client
from pathlib import Path

import numpy as np

try:
    from .nao_constants import NAO_JOINTS, NAO_LIMITS, NAO_MAX_VEL
    from .parse_annotations import parse_textgrid
except ImportError:
    from nao_constants import NAO_JOINTS, NAO_LIMITS, NAO_MAX_VEL
    from parse_annotations import parse_textgrid


_audio_stop = False


def clamp(value, low, high):
    return max(low, min(high, value))


def load_predictions(path: Path) -> np.ndarray:
    preds = np.load(str(path)).astype(np.float32)
    if preds.ndim != 2 or preds.shape[1] != len(NAO_JOINTS):
        raise ValueError(
            f"Expected predictions shaped (frames, {len(NAO_JOINTS)}), got {preds.shape}"
        )
    if preds.shape[0] == 0:
        raise ValueError("Prediction file contains zero frames")
    return preds


def build_word_lookup(textgrid_path: Path | None):
    if textgrid_path is None or not textgrid_path.is_file():
        return []
    try:
        tiers = parse_textgrid(textgrid_path)
        words = tiers.get("words", [])
        return [(word["start"], word["end"], word["text"]) for word in words]
    except Exception as exc:
        print(f"[WARN] Failed to parse TextGrid {textgrid_path}: {exc}")
        return []


def get_text_at_time(word_list, timestamp):
    for start, end, text in word_list:
        if start <= timestamp < end:
            return text
    return ""


def prepare_angles(preds: np.ndarray, fps: float, velocity_limit=True):
    """Clamp and optionally speed-limit direct NAO joint-angle predictions."""
    frame_time = 1.0 / fps
    all_times = [[] for _ in NAO_JOINTS]
    all_angles = [[] for _ in NAO_JOINTS]
    last_angles = [None] * len(NAO_JOINTS)

    current_time = frame_time
    for frame in preds:
        row = frame.copy()
        for joint_idx, name in enumerate(NAO_JOINTS):
            low, high = NAO_LIMITS[name]
            value = clamp(float(row[joint_idx]), low, high)

            if velocity_limit and last_angles[joint_idx] is not None:
                max_change = NAO_MAX_VEL.get(name, 5.0) * frame_time
                diff = value - last_angles[joint_idx]
                if abs(diff) > max_change:
                    value = last_angles[joint_idx] + math.copysign(max_change, diff)

            last_angles[joint_idx] = value
            all_times[joint_idx].append(current_time)
            all_angles[joint_idx].append(value)

        current_time += frame_time

    return all_times, all_angles


def build_frame_texts(num_frames: int, fps: float, word_list, time_offset=0.0):
    frame_time = 1.0 / fps
    frame_texts = []
    for frame_idx in range(num_frames):
        frame_texts.append(get_text_at_time(word_list, time_offset + frame_idx * frame_time))
    return frame_texts


def play_audio_in_thread(wav_path: Path | None, start_delay=1.5):
    global _audio_stop
    if wav_path is None:
        return
    if not wav_path.is_file():
        print(f"[WARN] WAV not found, skipping audio: {wav_path}")
        return

    def _play():
        global _audio_stop
        try:
            time.sleep(start_delay)
            with wave.open(str(wav_path), "rb") as wav_file:
                import pyaudio

                p = pyaudio.PyAudio()
                stream = p.open(
                    format=p.get_format_from_width(wav_file.getsampwidth()),
                    channels=wav_file.getnchannels(),
                    rate=wav_file.getframerate(),
                    output=True,
                )
                chunk = 2048
                while not _audio_stop:
                    data = wav_file.readframes(chunk)
                    if not data:
                        break
                    stream.write(data)
                stream.stop_stream()
                stream.close()
                p.terminate()
        except Exception as exc:
            print(f"[WARN] Audio playback failed: {exc}")

    _audio_stop = False
    thread = threading.Thread(target=_play, daemon=True)
    thread.start()
    print(f"[AUDIO] Started: {wav_path.name}")


def connect_nao(server_url: str):
    print(f"[NAO] Connecting to {server_url}")
    return xmlrpc.client.ServerProxy(server_url, allow_none=True)


def load_sidecar_metadata(pred_path: Path) -> dict:
    meta_path = pred_path.with_suffix(".json")
    if not meta_path.is_file():
        return {}
    try:
        with open(meta_path, "r") as f:
            return json.load(f)
    except Exception as exc:
        print(f"[WARN] Could not read metadata {meta_path}: {exc}")
        return {}


def main():
    parser = argparse.ArgumentParser(description="Play generated NAO joint-angle predictions")
    parser.add_argument("predictions", type=str,
                        help="Path to .npy shaped (frames, 10)")
    parser.add_argument("--wav", type=str, default=None,
                        help="Optional WAV to play alongside motion")
    parser.add_argument("--textgrid", type=str, default=None,
                        help="Optional TextGrid for NAO speech text events")
    parser.add_argument("--fps", type=float, default=None,
                        help="Playback FPS; defaults to prediction metadata or 30")
    parser.add_argument("--server", type=str, default="http://localhost:8000")
    parser.add_argument("--no-audio", action="store_true")
    parser.add_argument("--no-velocity-limit", action="store_true",
                        help="Do not smooth/speed-limit predictions before playback")
    parser.add_argument("--time-offset", type=float, default=0.0,
                        help="Offset into TextGrid timestamps if prediction is a subclip")
    parser.add_argument("--dry-run", action="store_true",
                        help="Build payload and print diagnostics without contacting NAO")
    args = parser.parse_args()

    pred_path = Path(args.predictions)
    wav_path = Path(args.wav) if args.wav else None
    textgrid_path = Path(args.textgrid) if args.textgrid else None
    metadata = load_sidecar_metadata(pred_path)
    fps = args.fps if args.fps is not None else float(metadata.get("fps", 30.0))

    preds = load_predictions(pred_path)
    duration = preds.shape[0] / fps
    print(f"[LOAD] Predictions: {pred_path}")
    print(f"[LOAD] Shape:       {preds.shape}")
    print(f"[LOAD] Duration:    {duration:.2f}s @ {fps:.1f} fps")
    print(f"[LOAD] Joint order: {NAO_JOINTS}")

    all_times, all_angles = prepare_angles(
        preds, fps, velocity_limit=not args.no_velocity_limit
    )
    word_list = build_word_lookup(textgrid_path)
    frame_texts = build_frame_texts(
        preds.shape[0], fps, word_list, time_offset=args.time_offset
    )
    speech_frames = sum(1 for text in frame_texts if text)
    print(f"[TEXT] Words:       {len(word_list)}")
    print(f"[TEXT] Speech frames: {speech_frames}/{len(frame_texts)}")

    print("[DEBUG] First-frame angles:")
    for joint_idx, name in enumerate(NAO_JOINTS):
        value = all_angles[joint_idx][0]
        print(f"  {name:18s} {value:+.4f} rad ({math.degrees(value):+.1f} deg)")

    if args.dry_run:
        print("[DRY-RUN] Payload built successfully; not contacting NAO.")
        return

    nao = connect_nao(args.server)
    if wav_path and not args.no_audio:
        play_audio_in_thread(wav_path)

    print("[NAO] Sending motion payload")
    rpc_error = [None]

    def _rpc_call():
        try:
            nao.play_motion_with_speech(NAO_JOINTS, all_angles, all_times, frame_texts)
        except Exception as exc:
            rpc_error[0] = exc

    rpc_thread = threading.Thread(target=_rpc_call, daemon=True)
    rpc_thread.start()

    try:
        while rpc_thread.is_alive():
            rpc_thread.join(timeout=0.5)
        if rpc_error[0]:
            print(f"[ERROR] Playback failed: {rpc_error[0]}")
            return
        print("[NAO] Playback complete. Resting robot.")
        nao.rest()
    except KeyboardInterrupt:
        global _audio_stop
        _audio_stop = True
        print("\n[STOP] Ctrl+C received; stopping NAO.")
        try:
            connect_nao(args.server).stop()
        except Exception as exc:
            print(f"[WARN] Could not send stop: {exc}")


if __name__ == "__main__":
    main()
