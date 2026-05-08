import numpy as np
import argparse
from pathlib import Path
import math

class BVHWriter:
    """
    Convert joint position predictions to BVH format.
    
    The model predicts 12 joints (upper body: spine, shoulders, arms, neck, head).
    These are mapped to the full NAO skeleton, with missing joints filled with
    identity rotations (0 0 0) to be ignored by IK solver.
    """
    
    # Full NAO skeleton hierarchy from the provided structure
    # Format: (name, parent_index, offset_x, offset_y, offset_z, channels, channel_names)
    # Most joints use 3 channels (Xrotation Yrotation Zrotation), root uses 6 (position + rotation)
    NAO_SKELETON_HIERARCHY = """
    ROOT body_world (0 0 0) [6 channels: Xpos, Ypos, Zpos, Zrot, Xrot, Yrot]
    ├─ Spine1 (-0.00 0.3635 -0.0605) [3 channels: Zrot, Xrot, Yrot]
    ├─ Spine2 (0.00 0.1096 -0.0016) [3 channels: Zrot, Xrot, Yrot]
    ├─ Spine3 (0.00 0.1496 -0.0016) [3 channels: Zrot, Xrot, Yrot]
    ├─ Spine4 (0.00 0.1496 -0.0016) [3 channels: Zrot, Xrot, Yrot]
    ├─ Spine5 (0.00 0.1096 -0.0016) [3 channels: Zrot, Xrot, Yrot]
    ├─ Neck (0.00 0.0705 -0.0115) [3 channels: Zrot, Xrot, Yrot]
    ├─ Head (0.00 0.0840 -0.0115) [3 channels: Zrot, Xrot, Yrot]
    ├─ LeftShoulder1 (-0.1069 0.0546 0.0546) [3 channels: Zrot, Xrot, Yrot]
    ├─ LeftArm1 (-0.0450 0.0000 0.0000) [3 channels: Zrot, Xrot, Yrot]
    ├─ LeftArm2 (-0.1050 0.0000 0.0000) [3 channels: Zrot, Xrot, Yrot]
    ├─ RightShoulder1 (0.1069 0.0546 0.0546) [3 channels: Zrot, Xrot, Yrot]
    ├─ RightArm1 (0.0450 0.0000 0.0000) [3 channels: Zrot, Xrot, Yrot]
    └─ RightArm2 (0.1050 0.0000 0.0000) [3 channels: Zrot, Xrot, Yrot]
    (Other joints: jaw, tongue, eyes, legs are present in full skeleton but set to identity rotations)
    """
    
    # Mapping: predicted joint index -> (bvh_joint_name, joint_order_in_bvh)
    # The 12 predicted joints are mapped to upper body joints
    PREDICTED_JOINT_MAPPING = {
        0: ("Spine1", 1),      # Spine/Torso joints
        1: ("Spine2", 2),
        2: ("Spine3", 3),
        3: ("Spine4", 4),
        4: ("Neck", 6),        # Neck
        5: ("Head", 7),        # Head
        6: ("LeftShoulder1", 8),   # Left arm
        7: ("LeftArm1", 9),
        8: ("LeftArm2", 10),
        9: ("RightShoulder1", 11),  # Right arm
        10: ("RightArm1", 12),
        11: ("RightArm2", 13),
    }
    
    # All BVH joint names in order with their parent relationships
    BVH_JOINT_ORDER = [
        ("body_world", None),     # 0 - ROOT
        ("Spine1", "body_world"),
        ("Spine2", "Spine1"),
        ("Spine3", "Spine2"),
        ("Spine4", "Spine3"),
        ("Spine5", "Spine4"),
        ("Neck", "Spine5"),
        ("Head", "Neck"),
        ("LeftShoulder1", "Spine3"),
        ("LeftArm1", "LeftShoulder1"),
        ("LeftArm2", "LeftArm1"),
        ("RightShoulder1", "Spine3"),
        ("RightArm1", "RightShoulder1"),
        ("RightArm2", "RightArm1"),
        # Additional joints (jaw, tongue, eyes, legs) would follow
        # For now, we include the 14 main upper-body joints
    ]
    
    def __init__(self, predicted_joints, fps=30):
        """
        Args:
            predicted_joints: numpy array of shape (Frames, 12, 3) - 12 predicted joint positions
            fps: frames per second
        """
        self.predicted_joints = predicted_joints  # (Frames, 12, 3)
        self.fps = fps
        self.num_frames = predicted_joints.shape[0]
        self.num_predicted = predicted_joints.shape[1]
        self.num_bvh_joints = len(self.BVH_JOINT_ORDER)
        
        if self.num_predicted != 12:
            raise ValueError(f"Expected 12 predicted joints, got {self.num_predicted}")
        
        print(f"[BVH] Initialized with {self.num_frames} frames, {self.num_predicted} predicted joints")
        print(f"[BVH] Full BVH will have {self.num_bvh_joints} joints")
    
    def positions_to_rotations(self):
        """
        Convert joint positions to Euler angles for BVH.
        
        Uses the child-to-parent vector to compute rotations. Missing joints
        (non-predicted) are set to identity rotations (0, 0, 0).
        
        Returns:
            rotations: (Frames, NumBVHJoints, 3) - Euler angles for each joint
        """
        print("[BVH] Converting joint positions to rotations...")
        
        rotations = np.zeros((self.num_frames, self.num_bvh_joints, 3))
        
        # First, expand predicted positions to full BVH joint space
        # joint_positions[frame, bvh_joint_idx] = position or [0,0,0] if not predicted
        full_positions = np.zeros((self.num_frames, self.num_bvh_joints, 3))
        
        for pred_idx, (joint_name, bvh_idx) in self.PREDICTED_JOINT_MAPPING.items():
            full_positions[:, bvh_idx, :] = self.predicted_joints[:, pred_idx, :]
        
        # Compute rotations for each frame
        for frame_idx in range(self.num_frames):
            positions = full_positions[frame_idx]  # (NumBVHJoints, 3)
            
            for joint_idx in range(self.num_bvh_joints):
                if joint_idx == 0:
                    # Root joint - no rotation (handled separately in BVH)
                    rotations[frame_idx, joint_idx] = [0, 0, 0]
                else:
                    # Get parent index
                    parent_name = self.BVH_JOINT_ORDER[joint_idx][1]
                    parent_idx = None
                    for i, (name, _) in enumerate(self.BVH_JOINT_ORDER):
                        if name == parent_name:
                            parent_idx = i
                            break
                    
                    if parent_idx is None:
                        rotations[frame_idx, joint_idx] = [0, 0, 0]
                    else:
                        # Vector from parent to this joint
                        parent_pos = positions[parent_idx]
                        joint_pos = positions[joint_idx]
                        vec = joint_pos - parent_pos
                        
                        # Compute Euler angles from vector
                        x, y, z = vec
                        
                        # Avoid division by zero
                        length = np.sqrt(x**2 + y**2 + z**2)
                        if length < 1e-6:
                            rotations[frame_idx, joint_idx] = [0, 0, 0]
                        else:
                            # Convert to angles
                            rx = np.arctan2(z, y) * 180 / np.pi
                            ry = np.arctan2(x, np.sqrt(y**2 + z**2)) * 180 / np.pi
                            rz = 0  # Simplified
                            
                            rotations[frame_idx, joint_idx] = [rx, ry, rz]
        
        print(f"[BVH] Generated rotations shape: {rotations.shape}")
        return rotations
    
    def write_bvh_minimal(self, output_path, rotations):
        """
        Write minimal BVH file with the upper-body skeleton and motion data.
        
        For now, write a simplified BVH with the 14 upper-body joints.
        Missing joints (legs, jaw, tongue, eyes) can be added later.
        
        Args:
            output_path: path to save BVH file
            rotations: (Frames, NumBVHJoints, 3) rotation data
        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        print(f"[BVH] Writing BVH file to {output_path}")
        
        with open(output_path, 'w') as f:
            # Write hierarchy
            f.write("HIERARCHY\n")
            f.write("ROOT body_world\n")
            f.write("{\n")
            f.write("  OFFSET 0 0 0\n")
            f.write("  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n")
            
            # Manually write hierarchy for upper body
            # Spine chain
            f.write("  JOINT Spine1\n")
            f.write("  {\n")
            f.write("    OFFSET 0 0.3635 -0.0605\n")
            f.write("    CHANNELS 3 Zrotation Xrotation Yrotation\n")
            
            f.write("    JOINT Spine2\n")
            f.write("    {\n")
            f.write("      OFFSET 0 0.1096 -0.0016\n")
            f.write("      CHANNELS 3 Zrotation Xrotation Yrotation\n")
            
            f.write("      JOINT Spine3\n")
            f.write("      {\n")
            f.write("        OFFSET 0 0.1496 -0.0016\n")
            f.write("        CHANNELS 3 Zrotation Xrotation Yrotation\n")
            
            f.write("        JOINT Spine4\n")
            f.write("        {\n")
            f.write("          OFFSET 0 0.1496 -0.0016\n")
            f.write("          CHANNELS 3 Zrotation Xrotation Yrotation\n")
            
            f.write("          JOINT Spine5\n")
            f.write("          {\n")
            f.write("            OFFSET 0 0.1096 -0.0016\n")
            f.write("            CHANNELS 3 Zrotation Xrotation Yrotation\n")
            
            f.write("            JOINT Neck\n")
            f.write("            {\n")
            f.write("              OFFSET 0 0.0705 -0.0115\n")
            f.write("              CHANNELS 3 Zrotation Xrotation Yrotation\n")
            
            f.write("              JOINT Head\n")
            f.write("              {\n")
            f.write("                OFFSET 0 0.0840 -0.0115\n")
            f.write("                CHANNELS 3 Zrotation Xrotation Yrotation\n")
            f.write("                End Site\n")
            f.write("                {\n")
            f.write("                  OFFSET 0 0.05 0\n")
            f.write("                }\n")
            f.write("              }\n")
            f.write("            }\n")
            
            # Left shoulder and arm
            f.write("            JOINT LeftShoulder1\n")
            f.write("            {\n")
            f.write("              OFFSET -0.1069 0.0546 0.0546\n")
            f.write("              CHANNELS 3 Zrotation Xrotation Yrotation\n")
            
            f.write("              JOINT LeftArm1\n")
            f.write("              {\n")
            f.write("                OFFSET -0.0450 0 0\n")
            f.write("                CHANNELS 3 Zrotation Xrotation Yrotation\n")
            
            f.write("                JOINT LeftArm2\n")
            f.write("                {\n")
            f.write("                  OFFSET -0.1050 0 0\n")
            f.write("                  CHANNELS 3 Zrotation Xrotation Yrotation\n")
            f.write("                  End Site\n")
            f.write("                  {\n")
            f.write("                    OFFSET -0.05 0 0\n")
            f.write("                  }\n")
            f.write("                }\n")
            f.write("              }\n")
            f.write("            }\n")
            
            # Right shoulder and arm
            f.write("            JOINT RightShoulder1\n")
            f.write("            {\n")
            f.write("              OFFSET 0.1069 0.0546 0.0546\n")
            f.write("              CHANNELS 3 Zrotation Xrotation Yrotation\n")
            
            f.write("              JOINT RightArm1\n")
            f.write("              {\n")
            f.write("                OFFSET 0.0450 0 0\n")
            f.write("                CHANNELS 3 Zrotation Xrotation Yrotation\n")
            
            f.write("                JOINT RightArm2\n")
            f.write("                {\n")
            f.write("                  OFFSET 0.1050 0 0\n")
            f.write("                  CHANNELS 3 Zrotation Xrotation Yrotation\n")
            f.write("                  End Site\n")
            f.write("                  {\n")
            f.write("                    OFFSET 0.05 0 0\n")
            f.write("                  }\n")
            f.write("                }\n")
            f.write("              }\n")
            f.write("            }\n")
            
            # Close spine hierarchy
            f.write("          }\n")  # Spine5
            f.write("        }\n")   # Spine4
            f.write("      }\n")     # Spine3
            f.write("    }\n")       # Spine2
            f.write("  }\n")         # Spine1
            f.write("}\n")           # body_world
            
            # Write motion data
            f.write("MOTION\n")
            f.write(f"Frames: {self.num_frames}\n")
            f.write(f"Frame Time: {1.0 / self.fps:.4f}\n")
            
            # Write frame data
            for frame_idx in range(self.num_frames):
                frame_data = []
                
                # Root position
                root_pos = self.predicted_joints[frame_idx, 0]  # First predicted joint is root reference
                frame_data.extend(root_pos)
                
                # Root rotation
                frame_data.extend(rotations[frame_idx, 0])
                
                # Other joints (rotation only)
                for joint_idx in range(1, self.num_bvh_joints):
                    frame_data.extend(rotations[frame_idx, joint_idx])
                
                # Write as space-separated values
                f.write(" ".join(f"{val:.6f}" for val in frame_data))
                f.write("\n")
        
        print(f"[BVH] BVH file saved successfully!")
        print(f"[BVH] Total frames: {self.num_frames}")
        print(f"[BVH] Frame rate: {self.fps} fps")
        print(f"[BVH] Duration: {self.num_frames / self.fps:.2f} seconds")
        print(f"[BVH] Joints included: {self.num_bvh_joints} (upper body only)")
        print(f"[BVH] Joint mapping: {self.PREDICTED_JOINT_MAPPING}")




if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Convert predicted joint positions to BVH format',
        epilog='The model predicts 12 upper-body joints. This script maps them to the full NAO skeleton.'
    )
    parser.add_argument('--input', type=str, required=True, help='Path to predictions (.npy file with shape (Frames, 12, 3))')
    parser.add_argument('--output', type=str, required=True, help='Output BVH file path')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second (default: 30)')
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("PREDICTIONS TO BVH CONVERTER")
    print("=" * 70)
    print(f"[INFO] Converting 12 predicted upper-body joints to full NAO skeleton BVH")
    print()
    
    # Load predictions
    print(f"[MAIN] Loading predictions from {args.input}")
    try:
        predictions = np.load(args.input)
        print(f"[MAIN] Predictions shape: {predictions.shape}")
    except Exception as e:
        print(f"[ERROR] Failed to load predictions: {e}")
        exit(1)
    
    # Validate shape
    if predictions.ndim != 3:
        print(f"[ERROR] Expected 3D array (Frames, 12, 3), got {predictions.ndim}D array")
        exit(1)
    
    if predictions.shape[1] != 12 or predictions.shape[2] != 3:
        print(f"[ERROR] Expected shape (Frames, 12, 3), got {predictions.shape}")
        exit(1)
    
    print(f"[MAIN] ✓ Shape valid: {predictions.ndim}D array with {predictions.shape[0]} frames, {predictions.shape[1]} joints, {predictions.shape[2]} coordinates")
    print()
    
    # Convert to BVH
    try:
        writer = BVHWriter(predictions, fps=args.fps)
        rotations = writer.positions_to_rotations()
        writer.write_bvh_minimal(args.output, rotations)
    except Exception as e:
        print(f"[ERROR] Failed to convert to BVH: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
    
    print()
    print("=" * 70)
    print("✓ Conversion complete!")
    print()
    print("Next steps:")
    print(f"  1. View BVH in Choregraphe: File → Load Motion → {args.output}")
    print("  2. Verify motion looks reasonable and joints are within limits")
    print("  3. Test with IK solver: python ../motion_logic/main_ik_client.py --bvh {args.output}")
    print("=" * 70)
