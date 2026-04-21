# main_ik_client.py - RUN IN PYTHON 3
import xmlrpc.client
import math
import threading
import time
import os
import wave
import sys

# ---- CHANGE THESE PATHS ----
BVH_DIR = r"C:\Users\aarya\Documents\Imperial College London\Year 4\FYP\Datasets\Genea2022\trn\trn\bvh"
TSV_DIR = r"C:\Users\aarya\Documents\Imperial College London\Year 4\FYP\Datasets\Genea2022\trn\trn\tsv"
WAV_DIR = r"C:\Users\aarya\Documents\Imperial College London\Year 4\FYP\Datasets\Genea2022\trn\trn\wav"

# Global flag to stop audio playback
_audio_stop = False

class BVHParser:
    def __init__(self, filepath):
        self.filepath = filepath
        self.joint_channels = {}
        self.frames = []
        self.frame_time = 0.0333333
        self.parse()

    def parse(self):
        with open(self.filepath, 'r') as f:
            lines = f.readlines()

        is_motion = False
        channel_index = 0
        joint_stack = []

        for line in lines:
            parts = line.strip().split()
            if not parts:
                continue

            if parts[0] == "MOTION":
                is_motion = True
                continue

            if not is_motion:
                if parts[0] in ["ROOT", "JOINT"]:
                    joint_stack.append(parts[1])
                elif parts[0] == "End":
                    joint_stack.append("EndSite")
                elif parts[0] == "}":
                    if joint_stack:
                        joint_stack.pop()
                elif parts[0] == "CHANNELS":
                    num_channels = int(parts[1])
                    current_joint = joint_stack[-1]
                    self.joint_channels[current_joint] = list(range(channel_index, channel_index + num_channels))
                    channel_index += num_channels
            else:
                if parts[0] == "Frames:":
                    continue
                elif parts[0] == "Frame" and len(parts) >= 3 and parts[1] == "Time:":
                    self.frame_time = float(parts[2])
                else:
                    self.frames.append([float(x) for x in parts])

class TSVParser:
    def __init__(self, filepath):
        self.filepath = filepath
        self.entries = []
        self.parse()
    
    def parse(self):
        """Parse TSV file with format: start_time<TAB>end_time<TAB>text"""
        with open(self.filepath, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split('\t')
                if len(parts) >= 3:
                    try:
                        start_time = float(parts[0])
                        end_time = float(parts[1])
                        text = parts[2]
                        self.entries.append({
                            'start': start_time,
                            'end': end_time,
                            'text': text
                        })
                    except ValueError:
                        print(f"[!] Skipping malformed TSV line: {line}")
                        continue
        
        # Sort by start time
        self.entries.sort(key=lambda x: x['start'])
        print(f"[✓] Loaded {len(self.entries)} speech entries from TSV.")
    
    def get_text_for_frame_time(self, current_time):
        """
        Get the speech text active at the given time.
        Returns the text string if a word's [start, end) interval covers current_time,
        or empty string if silence.
        """
        for entry in self.entries:
            if entry['start'] <= current_time < entry['end']:
                return entry['text']
        return ""

# --- Vector & Matrix Math for Kinematics ---

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def multiply_matrices(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def get_rotation_matrix(joint_name, bvh_frame, channels):
    if joint_name not in channels:
        return [[1,0,0], [0,1,0], [0,0,1]]
    
    idx = channels[joint_name][0]
    z = math.radians(bvh_frame[idx+3])
    x = math.radians(bvh_frame[idx+4])
    y = math.radians(bvh_frame[idx+5])
    
    cz, sz = math.cos(z), math.sin(z)
    cx, sx = math.cos(x), math.sin(x)
    cy, sy = math.cos(y), math.sin(y)
    
    Rz = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]
    Rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
    Ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
    
    return multiply_matrices(multiply_matrices(Rz, Rx), Ry)

def rotate_vector(R, v):
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]

def normalize(v):
    length = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    return [x/length for x in v] if length > 1e-6 else [1, 0, 0]

def cross_product(v, w):
    return [
        v[1]*w[2] - v[2]*w[1],
        v[2]*w[0] - v[0]*w[2],
        v[0]*w[1] - v[1]*w[0]
    ]

def bvh_to_nao_space(v):
    """Converts BVH space (X=Left, Y=Up, Z=Forward) to NAO space (X=Fwd, Y=Left, Z=Up)."""
    return [v[2], v[0], v[1]]

def solve_nao_arm_ik(V_nao, W_nao, is_left):
    V = normalize(V_nao)
    W = normalize(W_nao)
    
    pitch = math.atan2(-V[2], V[0])
    roll = math.asin(clamp(V[1], -1.0, 1.0))
    
    dot = clamp(V[0]*W[0] + V[1]*W[1] + V[2]*W[2], -1.0, 1.0)
    elbow_roll_mag = math.acos(dot)
    
    N = normalize(cross_product(V, W))
    
    My = -math.cos(pitch)*math.sin(roll)*N[0] + math.cos(roll)*N[1] + math.sin(pitch)*math.sin(roll)*N[2]
    Mz = math.sin(pitch)*N[0] + math.cos(pitch)*N[2]
    
    if is_left:
        elbow_yaw = math.atan2(My, -Mz)
        elbow_roll = -elbow_roll_mag 
    else:
        elbow_yaw = math.atan2(-My, Mz)
        elbow_roll = elbow_roll_mag  
        
    return pitch, roll, elbow_yaw, elbow_roll


def map_bvh_to_nao(bvh_frame, channels):
    # --- HEAD & SPINE FORWARD KINEMATICS ---
    # We multiply the entire spine chain so the robot compensates for human torso leaning
    R_spine0 = get_rotation_matrix("b_spine0", bvh_frame, channels)
    R_spine1 = get_rotation_matrix("b_spine1", bvh_frame, channels)
    R_spine2 = get_rotation_matrix("b_spine2", bvh_frame, channels)
    R_spine3 = get_rotation_matrix("b_spine3", bvh_frame, channels)
    R_neck0 = get_rotation_matrix("b_neck0", bvh_frame, channels)
    R_head = get_rotation_matrix("b_head", bvh_frame, channels)
    
    R_sp01 = multiply_matrices(R_spine0, R_spine1)
    R_sp23 = multiply_matrices(R_spine2, R_spine3)
    R_neck_head = multiply_matrices(R_neck0, R_head)
    
    R_torso = multiply_matrices(R_sp01, R_sp23)
    R_head_global = multiply_matrices(R_torso, R_neck_head)
    
    # BVH Default forward is usually +Z. Extract the human's absolute gaze vector.
    v_head_fwd = rotate_vector(R_head_global, [0, 0, 1])
    v_head_nao = bvh_to_nao_space(v_head_fwd)
    
    # --- HEAD INVERSE KINEMATICS ---
    # X is Forward, Y is Left, Z is Up in NAO space
    head_yaw = math.atan2(v_head_nao[1], v_head_nao[0])
    head_pitch = -math.asin(clamp(v_head_nao[2], -1.0, 1.0)) # Negative because +Pitch is looking down on NAO

    # --- RIGHT ARM FORWARD KINEMATICS ---
    R_r_shoulder = get_rotation_matrix("b_r_shoulder", bvh_frame, channels)
    R_r_scap = get_rotation_matrix("p_r_scap", bvh_frame, channels)
    R_r_arm = get_rotation_matrix("b_r_arm", bvh_frame, channels)
    
    R_upper_r = multiply_matrices(multiply_matrices(R_r_shoulder, R_r_scap), R_r_arm)
    v_upper_r = rotate_vector(R_upper_r, [-25.5811, -0.717611, -0.672458])
    
    R_r_arm_twist = get_rotation_matrix("b_r_arm_twist", bvh_frame, channels)
    R_r_forearm = get_rotation_matrix("b_r_forearm", bvh_frame, channels)
    
    R_lower_r = multiply_matrices(multiply_matrices(R_upper_r, R_r_arm_twist), R_r_forearm)
    v_lower_r = rotate_vector(R_lower_r, [-24.7817, -0.695186, -0.651444])

    # --- LEFT ARM FORWARD KINEMATICS ---
    R_l_shoulder = get_rotation_matrix("b_l_shoulder", bvh_frame, channels)
    R_l_scap = get_rotation_matrix("p_l_scap", bvh_frame, channels)
    R_l_arm = get_rotation_matrix("b_l_arm", bvh_frame, channels)
    
    R_upper_l = multiply_matrices(multiply_matrices(R_l_shoulder, R_l_scap), R_l_arm)
    v_upper_l = rotate_vector(R_upper_l, [25.5839, -0.593346, -0.688576])
    
    R_l_arm_twist = get_rotation_matrix("b_l_arm_twist", bvh_frame, channels)
    R_l_forearm = get_rotation_matrix("b_l_forearm", bvh_frame, channels)
    
    R_lower_l = multiply_matrices(multiply_matrices(R_upper_l, R_l_arm_twist), R_l_forearm)
    v_lower_l = rotate_vector(R_lower_l, [24.7844, -0.574804, -0.667058])

    # --- ARM INVERSE KINEMATICS ---
    r_p, r_r, r_y, r_er = solve_nao_arm_ik(bvh_to_nao_space(v_upper_r), bvh_to_nao_space(v_lower_r), is_left=False)
    l_p, l_r, l_y, l_er = solve_nao_arm_ik(bvh_to_nao_space(v_upper_l), bvh_to_nao_space(v_lower_l), is_left=True)

    return {
        "HeadYaw":        clamp(head_yaw, -2.08, 2.08),
        "HeadPitch":      clamp(head_pitch, -0.67, 0.51), # Asymmetrical hardware limits

        "RShoulderPitch": clamp(r_p, -2.08, 2.08),
        "RShoulderRoll":  clamp(r_r, -1.32, 0.31),
        "RElbowYaw":      clamp(r_y, -2.08, 2.08),
        "RElbowRoll":     clamp(r_er, 0.03, 1.54),

        "LShoulderPitch": clamp(l_p, -2.08, 2.08),
        "LShoulderRoll":  clamp(l_r, -0.31, 1.32),
        "LElbowYaw":      clamp(l_y, -2.08, 2.08),
        "LElbowRoll":     clamp(l_er, -1.54, -0.03)
    }

def get_wav_file(bvh_filepath):
    """
    Find the corresponding WAV file for a given BVH file.
    
    TODO: Implement this function to locate the audio file based on the BVH filepath.
    For example, you could:
    - Replace the extension: path.bvh -> path.wav
    - Search in a specific directory based on the BVH name
    - Check if the file exists before returning
    
    Args:
        bvh_filepath (str): Path to the BVH file
        
    Returns:
        str: Path to the corresponding WAV file, or None if not found
    """

    return r"C:\Users\aarya\Documents\Imperial College London\Year 4\FYP\Datasets\Genea2022\trn\trn\wav\trn_2022_v1_000.wav"
    # Placeholder implementation - replace with your logic
    wav_filepath = bvh_filepath.rsplit('.', 1)[0] + '.wav'
    if os.path.exists(wav_filepath):
        return wav_filepath
    return None

def play_audio_in_thread(wav_filepath):
    """
    Play a WAV file in a separate daemon thread without blocking the main thread.
    
    Args:
        wav_filepath (str): Path to the WAV file to play
    """
    if wav_filepath is None:
        print("[!] No audio file found, skipping audio playback.")
        return
    
    if not os.path.exists(wav_filepath):
        print(f"[!] Audio file not found: {wav_filepath}")
        return
    
    def _play_audio():
        global _audio_stop
        try:
            time.sleep(1.5)
            # Open and play the WAV file using the wave module
            with wave.open(wav_filepath, 'rb') as wav_file:
                import pyaudio
                print("playing audio")
                # Get audio parameters
                n_channels = wav_file.getnchannels()
                sample_width = wav_file.getsampwidth()
                framerate = wav_file.getframerate()
                
                # Initialize PyAudio
                p = pyaudio.PyAudio()
                
                # Open audio output stream
                stream = p.open(format=p.get_format_from_width(sample_width),
                                channels=n_channels,
                                rate=framerate,
                                output=True)
                
                # Read and play audio in chunks
                chunk = 2048
                while not _audio_stop:
                    data = wav_file.readframes(chunk)
                    if not data:
                        break
                    stream.write(data)
                
                # Clean up
                stream.stop_stream()
                stream.close()
                p.terminate()
                if not _audio_stop:
                    print("[✓] Audio playback completed.")
                else:
                    print("[✓] Audio playback stopped.")
                
        except Exception as e:
            print(f"[!] Error playing audio: {e}")
    
    _audio_stop = False
    # Start audio playback in a daemon thread (won't block main thread)
    audio_thread = threading.Thread(target=_play_audio, daemon=True)
    audio_thread.start()
    print(f"[♪] Audio playback started in background thread: {os.path.basename(wav_filepath)}")

def build_full_payload(bvh, tsv, joint_names):
    """
    Pre-compute ALL frame kinematics and attach per-frame speech text.
    Returns a payload ready to be sent in one XML-RPC call to the server.
    
    Each frame gets the speech text whose [start, end) interval covers
    the frame's timestamp. Empty string means silence at that frame.
    """
    time_delta = bvh.frame_time  # use native BVH frame rate (30fps)
    
    print(f"[→] Pre-computing all frame kinematics ({1.0/time_delta:.0f} fps)...")
    
    # Per-joint lists for angleInterpolation format
    all_times = [[] for _ in joint_names]
    all_angles = [[] for _ in joint_names]
    frame_texts = []
    
    # NAO per-joint max velocities (rad/s) — set to 80% of hardware max for safety
    nao_max_velocities = {
        "HeadYaw":        8.27 * 0.8,   # max 8.27
        "HeadPitch":      7.19 * 0.8,   # max 7.19
        "RShoulderPitch": 8.27 * 0.8,
        "RShoulderRoll":  7.19 * 0.8,   # max 7.19
        "RElbowYaw":      8.27 * 0.8,
        "RElbowRoll":     8.27 * 0.8,
        "LShoulderPitch": 8.27 * 0.8,
        "LShoulderRoll":  7.19 * 0.8,
        "LElbowYaw":      8.27 * 0.8,
        "LElbowRoll":     8.27 * 0.8,
    }
    
    # Start at one frame_time so first keyframe time is always > 0
    current_time = time_delta
    last_angles = [None] * len(joint_names)
    
    for i in range(len(bvh.frames)):
        frame_data = bvh.frames[i]
        mapped = map_bvh_to_nao(frame_data, bvh.joint_channels)
        
        # Apply per-joint speed limiting using NAO hardware velocity limits
        for j, name in enumerate(joint_names):
            max_vel = nao_max_velocities.get(name, 5.0)
            if last_angles[j] is not None:
                max_change = max_vel * time_delta
                diff = mapped[name] - last_angles[j]
                if abs(diff) > max_change:
                    mapped[name] = last_angles[j] + math.copysign(max_change, diff)
            last_angles[j] = mapped[name]
        
        # Build trajectory arrays
        for j, name in enumerate(joint_names):
            all_times[j].append(current_time)
            all_angles[j].append(mapped[name])
        
        # Attach speech text active at this frame's time
        text = tsv.get_text_for_frame_time(current_time)
        frame_texts.append(text)
        
        current_time += time_delta
    
    total_frames = len(frame_texts)
    total_duration = all_times[0][-1] if total_frames > 0 else 0
    speech_frames = sum(1 for t in frame_texts if t)
    
    print(f"[✓] Pre-computed {total_frames} frames. Total duration: {total_duration:.1f}s")
    print(f"[✓] {speech_frames} frames have speech text attached.\n")
    
    return all_times, all_angles, frame_texts

def main():
    # Parse command line arguments
    if len(sys.argv) < 2:
        print("Usage: python main_ik_client.py <filename>")
        print("Example: python main_ik_client.py trn_2022_v1_000")
        print("\nThe filename is used to find:")
        print(f"  BVH: {{BVH_DIR}}/<filename>.bvh")
        print(f"  TSV: {{TSV_DIR}}/<filename>.tsv")
        print(f"  WAV: {{WAV_DIR}}/<filename>.wav")
        sys.exit(1)
    
    filename = sys.argv[1]
    # Strip extension if user accidentally included one
    filename = os.path.splitext(filename)[0]
    
    bvh_filepath = os.path.join(BVH_DIR, filename + ".bvh")
    tsv_filepath = os.path.join(TSV_DIR, filename + ".tsv")
    wav_filepath = os.path.join(WAV_DIR, filename + ".wav")
    
    # Validate files exist
    if not os.path.exists(bvh_filepath):
        print(f"[!] BVH file not found: {bvh_filepath}")
        sys.exit(1)
    if not os.path.exists(tsv_filepath):
        print(f"[!] TSV file not found: {tsv_filepath}")
        sys.exit(1)
    if not os.path.exists(wav_filepath):
        print(f"[!] WAV file not found: {wav_filepath}")
        print("    (Audio will be skipped)")
        wav_filepath = None
    
    print("Connecting to Python 2 NAO Bridge on localhost:8000...")
    try:
        nao = xmlrpc.client.ServerProxy('http://localhost:8000', allow_none=True)
    except ConnectionRefusedError:
        print("ERROR: Could not connect to the Python 2 Server. Is nao_server.py running?")
        return

    print(f"Loading BVH: {os.path.basename(bvh_filepath)}")
    bvh = BVHParser(bvh_filepath)
    
    print(f"Loading TSV: {os.path.basename(tsv_filepath)}")
    tsv = TSVParser(tsv_filepath)
    
    # Joint names for NAO
    joint_names = [
        "HeadYaw", "HeadPitch",
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
        "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"
    ]
    
    # Pre-compute full payload (all frames + per-frame speech)
    print("Building full motion + speech payload...\n")
    all_times, all_angles, frame_texts = build_full_payload(bvh, tsv, joint_names)
    
    # Send everything in ONE XML-RPC call — server handles streaming
    print("[→] Sending full payload to NAO server (one RPC call)...")
    print("[⏳] Server is now streaming motion + speech to the robot...\n")
    
    # Start audio playback in background right as motion begins
    if wav_filepath:
        play_audio_in_thread(wav_filepath)
    
    # Run the RPC call in a background thread so the main thread
    # stays responsive to Ctrl+C (Windows blocks KeyboardInterrupt
    # during C-level socket reads)
    rpc_error = [None]
    def rpc_call():
        try:
            nao.play_motion_with_speech(joint_names, all_angles, all_times, frame_texts)
        except Exception as e:
            rpc_error[0] = e
    
    rpc_thread = threading.Thread(target=rpc_call)
    rpc_thread.daemon = True
    rpc_thread.start()
    
    try:
        # Poll with short timeout so Ctrl+C can be caught
        while rpc_thread.is_alive():
            rpc_thread.join(timeout=0.5)
        
        if rpc_error[0]:
            print(f"\n[!] Error during playback: {rpc_error[0]}")
        else:
            print("\n[✓] Animation complete. Putting robot to rest.")
            nao.rest()

    except KeyboardInterrupt:
        print("\n[!] Ctrl+C detected! Stopping audio and aborting robot motion...")
        _audio_stop = True
        try:
            # Create a FRESH connection since the original one may be broken
            nao2 = xmlrpc.client.ServerProxy('http://localhost:8000', allow_none=True)
            nao2.stop()
        except Exception as e:
            print(f"[!] Could not send stop to server: {e}")

if __name__ == "__main__":
    main()