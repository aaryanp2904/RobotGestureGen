# main_ik_client.py - RUN IN PYTHON 3
import xmlrpc.client
import math

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
                if parts[0] not in ["Frames:", "Frame"]:
                    self.frames.append([float(x) for x in parts])

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

def main():
    print("Connecting to Python 2 NAO Bridge on localhost:8000...")
    try:
        nao = xmlrpc.client.ServerProxy('http://localhost:8000', allow_none=True)
    except ConnectionRefusedError:
        print("ERROR: Could not connect to the Python 2 Server. Is nao_server.py running?")
        return

    print("Loading BVH and computing IK in Python 3...")
    bvh = BVHParser(r"C:\Users\aarya\Documents\Imperial College London\Year 4\FYP\Datasets\Genea2022\trn\trn\bvh\trn_2022_v1_000.bvh")
    
    # Added Head Joints
    joint_names = [
        "HeadYaw", "HeadPitch",
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
        "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"
    ]
    
    times = [[] for _ in joint_names]
    angles = [[] for _ in joint_names]
    
    current_time = 2.0  
    frame_step = 3      
    time_delta = bvh.frame_time * frame_step
    
    last_angles = [None] * len(joint_names)
    max_velocity = 5.0
    
    for i in range(0, len(bvh.frames), frame_step):
        frame_data = bvh.frames[i]
        mapped = map_bvh_to_nao(frame_data, bvh.joint_channels)
        
        for j, name in enumerate(joint_names):
            target_angle = mapped[name]
            
            # Software speed limit (smooths out BVH spikes)
            if last_angles[j] is not None:
                max_change = max_velocity * time_delta
                diff = target_angle - last_angles[j]
                if abs(diff) > max_change:
                    target_angle = last_angles[j] + math.copysign(max_change, diff)
            
            times[j].append(current_time)
            angles[j].append(target_angle)
            last_angles[j] = target_angle
            
        current_time += time_delta

    # Find out exactly how long the animation takes
    total_duration = max([t[-1] for t in times if t])

    print(f"Sending trajectory payload. Animation will run for {total_duration:.1f} seconds.")
    nao.play_trajectory(joint_names, angles, times)

    # Because the robot is now moving in the background, we must keep Python 3 
    # alive for the duration of the animation so it can listen for your Ctrl+C
    import time
    try:
        print("Robot is moving! Press Ctrl+C at any time to abort...")
        time.sleep(total_duration)
        
        # If it finishes naturally without you pressing Ctrl+C:
        print("Animation complete. Putting robot to rest.")
        nao.rest()

    except KeyboardInterrupt:
        # If you press Ctrl+C, this block catches it and sends the stop command!
        print("\n[!] Ctrl+C detected! Aborting robot motion...")
        nao.stop()

if __name__ == "__main__":
    main()