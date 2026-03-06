from naoqi import ALProxy
import time
import math

IP = "127.0.0.1"
PORT = 31559

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

# Keep it between the max and min vals
def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def multiply_matrices(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def get_rotation_matrix(joint_name, bvh_frame, channels):
    """Computes the local rotation matrix for a BVH joint (ZXY order)."""
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
    """
    Analytically solves for NAO shoulder and elbow angles given the 3D unit 
    vectors of the Upper Arm (V) and Forearm (W) in the Torso coordinate space.
    """
    V = normalize(V_nao)
    W = normalize(W_nao)
    
    # 1. Shoulder Pitch & Roll
    # NAO Pitch rotates around Y (left/right axis), Roll rotates around Z (forward axis)
    pitch = math.atan2(-V[2], V[0])
    roll = math.asin(clamp(V[1], -1.0, 1.0))
    
    # 2. Elbow Roll (Bending angle)
    dot = clamp(V[0]*W[0] + V[1]*W[1] + V[2]*W[2], -1.0, 1.0)
    elbow_roll_mag = math.acos(dot)
    
    # 3. Elbow Yaw (Twist of the bending plane)
    N = normalize(cross_product(V, W)) # Normal to the elbow bend plane
    
    # Transform the Normal back through the shoulder joints to isolate the Elbow Yaw
    My = -math.cos(pitch)*math.sin(roll)*N[0] + math.cos(roll)*N[1] + math.sin(pitch)*math.sin(roll)*N[2]
    Mz = math.sin(pitch)*N[0] + math.cos(pitch)*N[2]
    
    if is_left:
        elbow_yaw = math.atan2(My, -Mz)
        elbow_roll = -elbow_roll_mag # Left elbow roll is strictly negative on NAO
    else:
        elbow_yaw = math.atan2(-My, Mz)
        elbow_roll = elbow_roll_mag  # Right elbow roll is strictly positive
        
    return pitch, roll, elbow_yaw, elbow_roll


def map_bvh_to_nao(bvh_frame, channels):
    # --- RIGHT ARM FORWARD KINEMATICS ---
    R_r_shoulder = get_rotation_matrix("b_r_shoulder", bvh_frame, channels)
    R_r_scap = get_rotation_matrix("p_r_scap", bvh_frame, channels)
    R_r_arm = get_rotation_matrix("b_r_arm", bvh_frame, channels)
    
    # Upper arm frame
    R_upper_r = multiply_matrices(multiply_matrices(R_r_shoulder, R_r_scap), R_r_arm)
    # The BVH offset vector for the right forearm (length of upper arm)
    v_upper_r = rotate_vector(R_upper_r, [-25.5811, -0.717611, -0.672458])
    
    R_r_arm_twist = get_rotation_matrix("b_r_arm_twist", bvh_frame, channels)
    R_r_forearm = get_rotation_matrix("b_r_forearm", bvh_frame, channels)
    
    # Forearm frame
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

    # --- INVERSE KINEMATICS ---
    r_p, r_r, r_y, r_er = solve_nao_arm_ik(bvh_to_nao_space(v_upper_r), bvh_to_nao_space(v_lower_r), is_left=False)
    l_p, l_r, l_y, l_er = solve_nao_arm_ik(bvh_to_nao_space(v_upper_l), bvh_to_nao_space(v_lower_l), is_left=True)

    # Return clamped to NAO's physical hardware limits
    return {
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
    print("Connecting to NAO...")
    motion = ALProxy("ALMotion", IP, PORT)
    posture = ALProxy("ALRobotPosture", IP, PORT)
    life = ALProxy("ALAutonomousLife", IP, PORT)

    if life.getState() != "disabled":
        life.setState("disabled")

    motion.wakeUp()
    posture.goToPosture("StandInit", 0.5)

    print("Loading BVH...")
    bvh = BVHParser(r"C:\Users\aarya\Documents\Imperial College London\Year 4\FYP\Datasets\Genea2022\trn\trn\bvh\trn_2022_v1_000.bvh")
    
    joint_names = [
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
        "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"
    ]
    
    times = [[] for _ in joint_names]
    angles = [[] for _ in joint_names]
    
    current_time = 2.0  # Lead-in time for smooth starting transition
    frame_step = 3      # Skip frames for robot buffer stability
    time_delta = bvh.frame_time * frame_step
    
    # Software speed dampening to ensure hardware compatibility
    last_angles = [None] * len(joint_names)
    max_velocity = 5.0
    
    print("Computing Inverse Kinematics Trajectories...")
    for i in range(0, len(bvh.frames), frame_step):
        frame_data = bvh.frames[i]
        print("Processing frame {}/{}...".format(i, len(bvh.frames)), end="\r")
        mapped = map_bvh_to_nao(frame_data, bvh.joint_channels)
        
        for j, name in enumerate(joint_names):
            target_angle = mapped[name]
            if last_angles[j] is not None:
                max_change = max_velocity * time_delta
                diff = target_angle - last_angles[j]
                if abs(diff) > max_change:
                    target_angle = last_angles[j] + math.copysign(max_change, diff)
            
            times[j].append(current_time)
            angles[j].append(target_angle)
            last_angles[j] = target_angle
            
        current_time += time_delta

    print("Executing IK trajectory...")
    motion.angleInterpolation(joint_names, angles, times, True)

    print("Animation complete. Resting.")
    posture.goToPosture("StandInit", 0.5)
    motion.rest()

if __name__ == "__main__":
    main()