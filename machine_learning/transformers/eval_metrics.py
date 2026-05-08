import torch
import numpy as np
import argparse
from pathlib import Path
import json

# NAO Joint Limits (in radians)
NAO_JOINT_LIMITS = {
    'head_yaw': (-2.0857, 2.0857),
    'head_pitch': (-0.6458, 0.5149),
    'lshoulder_pitch': (-2.0857, 2.0857),
    'lshoulder_roll': (-0.3142, 1.3265),
    'lelbow_yaw': (-1.5446, 0.4363),
    'lelbow_roll': (-0.0698, 1.5446),
    'lhip_yaw_pitch': (-1.1459, 0.7505),
    'lhip_roll': (-0.3838, 0.7934),
    'lhip_pitch': (-1.5446, 1.0472),
    'lknee_pitch': (-0.0873, 2.0944),
    'lankle_pitch': (-1.1519, 0.9273),
    'lankle_roll': (-0.3842, 0.3900),
    # Right side mirrors left
    'rshoulder_pitch': (-2.0857, 2.0857),
    'rshoulder_roll': (-1.3265, 0.3142),
    'relbow_yaw': (-0.4363, 1.5446),
    'relbow_roll': (-1.5446, 0.0698),
    'rhip_yaw_pitch': (-1.1459, 0.7505),
    'rhip_roll': (-0.7934, 0.3838),
    'rhip_pitch': (-1.5446, 1.0472),
    'rknee_pitch': (-0.0873, 2.0944),
    'rankle_pitch': (-1.1519, 0.9273),
    'rankle_roll': (-0.3900, 0.3842),
}

class MotionEvaluator:
    """Compute motion quality metrics for gesture generation."""
    
    def __init__(self, fps=30):
        self.fps = fps
        self.joint_limits = NAO_JOINT_LIMITS
    
    def prediction_error(self, predictions, ground_truth):
        """
        Compute MSE and L1 prediction error.
        
        Args:
            predictions: (Frames, 12, 3) or (Frames, N_joints, 3)
            ground_truth: Same shape as predictions
        
        Returns:
            dict with mse and l1 errors
        """
        if not isinstance(predictions, torch.Tensor):
            predictions = torch.tensor(predictions, dtype=torch.float32)
        if not isinstance(ground_truth, torch.Tensor):
            ground_truth = torch.tensor(ground_truth, dtype=torch.float32)
        
        mse = torch.mean((predictions - ground_truth) ** 2).item()
        l1 = torch.mean(torch.abs(predictions - ground_truth)).item()
        
        # Per-joint errors for granularity
        per_joint_mse = torch.mean((predictions - ground_truth) ** 2, dim=(0, 2))  # (N_joints,)
        
        return {
            'mse': mse,
            'l1': l1,
            'per_joint_mse': per_joint_mse.cpu().numpy() if isinstance(per_joint_mse, torch.Tensor) else per_joint_mse
        }
    
    def smoothness(self, joint_angles):
        """
        Compute smoothness metrics: velocity and acceleration.
        Lower values = smoother motion.
        
        Args:
            joint_angles: (Frames, N_joints) - Joint angles in radians
        
        Returns:
            dict with velocity and acceleration metrics
        """
        if not isinstance(joint_angles, torch.Tensor):
            joint_angles = torch.tensor(joint_angles, dtype=torch.float32)
        
        # Velocity (first derivative)
        dt = 1.0 / self.fps
        velocity = torch.diff(joint_angles, dim=0) / dt  # (Frames-1, N_joints)
        velocity_magnitude = torch.sqrt(torch.sum(velocity ** 2, dim=-1))  # (Frames-1,)
        
        # Acceleration (second derivative)
        acceleration = torch.diff(velocity, dim=0) / dt  # (Frames-2, N_joints)
        acceleration_magnitude = torch.sqrt(torch.sum(acceleration ** 2, dim=-1))  # (Frames-2,)
        
        metrics = {
            'mean_velocity': torch.mean(velocity_magnitude).item(),
            'max_velocity': torch.max(velocity_magnitude).item(),
            'mean_acceleration': torch.mean(acceleration_magnitude).item(),
            'max_acceleration': torch.max(acceleration_magnitude).item(),
            # Jerk (3rd derivative) - how much acceleration changes
            'mean_jerk': torch.mean(torch.abs(torch.diff(acceleration_magnitude, dim=0))).item() if len(acceleration_magnitude) > 1 else 0.0
        }
        
        return metrics
    
    def joint_feasibility(self, joint_angles, joint_names=None):
        """
        Check if joint angles stay within NAO hardware limits.
        
        Args:
            joint_angles: (Frames, N_joints) - Joint angles in radians
            joint_names: Optional list of joint names for detailed reporting
        
        Returns:
            dict with feasibility metrics
        """
        if not isinstance(joint_angles, torch.Tensor):
            joint_angles = torch.tensor(joint_angles, dtype=torch.float32)
        
        frames, n_joints = joint_angles.shape
        
        # Use provided joint names or generic labels
        if joint_names is None:
            joint_names = [f'Joint_{i}' for i in range(n_joints)]
        
        # Check limits
        violations_per_frame = torch.zeros(frames)
        violations_per_joint = torch.zeros(n_joints)
        
        for j, joint_name in enumerate(joint_names):
            if joint_name in self.joint_limits:
                min_limit, max_limit = self.joint_limits[joint_name]
                
                # Find frames where this joint violates limits
                violations = (joint_angles[:, j] < min_limit) | (joint_angles[:, j] > max_limit)
                violations_per_frame += violations.float()
                violations_per_joint[j] = torch.sum(violations).item()
        
        # Compute statistics
        total_possible_violations = frames * n_joints
        total_violations = torch.sum(violations_per_joint).item()
        
        feasible_frames = torch.sum(violations_per_frame == 0).item()
        feasibility_percentage = (feasible_frames / frames) * 100
        
        return {
            'feasibility_percentage': feasibility_percentage,  # % of frames with NO violations
            'total_violations': int(total_violations),
            'violation_rate': (total_violations / total_possible_violations) * 100,  # % of joint-frames violated
            'violations_per_joint': violations_per_joint.cpu().numpy() if isinstance(violations_per_joint, torch.Tensor) else violations_per_joint
        }
    
    def evaluate(self, predictions, ground_truth, joint_angles_pred=None):
        """
        Complete evaluation with all metrics.
        
        Args:
            predictions: (Frames, 12, 3) - Predicted joint positions
            ground_truth: (Frames, 12, 3) - Ground truth joint positions
            joint_angles_pred: Optional (Frames, N_joints) - Predicted joint angles
        
        Returns:
            dict with all metrics
        """
        results = {}
        
        print("[EVAL] Computing prediction error...")
        results['prediction_error'] = self.prediction_error(predictions, ground_truth)
        
        print("[EVAL] Computing smoothness...")
        # For smoothness, we can use the position differences as proxy
        results['smoothness'] = self.smoothness(predictions.reshape(predictions.shape[0], -1))
        
        if joint_angles_pred is not None:
            print("[EVAL] Checking joint feasibility...")
            results['joint_feasibility'] = self.joint_feasibility(joint_angles_pred)
        else:
            print("[EVAL] WARNING: No joint angles provided for feasibility check")
        
        return results


def print_report(results):
    """Pretty-print evaluation results."""
    print("\n" + "=" * 70)
    print("MOTION EVALUATION REPORT")
    print("=" * 70)
    
    # Prediction Error
    print("\n[PREDICTION ERROR]")
    print(f"  MSE Loss:  {results['prediction_error']['mse']:.6f}")
    print(f"  L1 Loss:   {results['prediction_error']['l1']:.6f}")
    print(f"  Per-Joint MSE: {results['prediction_error']['per_joint_mse']}")
    
    # Smoothness
    print("\n[SMOOTHNESS]")
    for key, value in results['smoothness'].items():
        print(f"  {key.replace('_', ' ').title()}: {value:.6f}")
    
    # Joint Feasibility
    if 'joint_feasibility' in results:
        feas = results['joint_feasibility']
        print("\n[JOINT FEASIBILITY]")
        print(f"  Feasible Frames:       {feas['feasibility_percentage']:.1f}%")
        print(f"  Total Violations:      {feas['total_violations']}")
        print(f"  Violation Rate:        {feas['violation_rate']:.2f}%")
        print(f"  Per-Joint Violations:  {feas['violations_per_joint']}")
    
    print("\n" + "=" * 70)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Evaluate gesture generation quality')
    parser.add_argument('--predictions', type=str, required=True, help='Path to predicted joint positions (.npy or .pt)')
    parser.add_argument('--ground-truth', type=str, required=True, help='Path to ground truth joint positions (.npy or .pt)')
    parser.add_argument('--joint-angles', type=str, help='Path to predicted joint angles for feasibility check')
    parser.add_argument('--output', type=str, help='Save results to JSON file')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second')
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("LOADING DATA")
    print("=" * 70)
    
    # Load predictions
    pred_path = Path(args.predictions)
    if pred_path.suffix == '.pt':
        predictions = torch.load(args.predictions).cpu().numpy()
    else:
        predictions = np.load(args.predictions)
    print(f"[LOAD] Predictions: {predictions.shape}")
    
    # Load ground truth
    gt_path = Path(args.ground_truth)
    if gt_path.suffix == '.pt':
        ground_truth = torch.load(args.ground_truth).cpu().numpy()
    else:
        ground_truth = np.load(args.ground_truth)
    print(f"[LOAD] Ground truth: {ground_truth.shape}")
    
    # Load joint angles if provided
    joint_angles = None
    if args.joint_angles:
        ja_path = Path(args.joint_angles)
        if ja_path.suffix == '.pt':
            joint_angles = torch.load(args.joint_angles).cpu().numpy()
        else:
            joint_angles = np.load(args.joint_angles)
        print(f"[LOAD] Joint angles: {joint_angles.shape}")
    
    # Evaluate
    evaluator = MotionEvaluator(fps=args.fps)
    results = evaluator.evaluate(predictions, ground_truth, joint_angles)
    
    # Print report
    print_report(results)
    
    # Save results
    if args.output:
        output_data = {}
        for key, val in results.items():
            if isinstance(val, dict):
                output_data[key] = {k: float(v) if isinstance(v, (int, np.integer, float, np.floating)) else v.tolist() if hasattr(v, 'tolist') else str(v) for k, v in val.items()}
            else:
                output_data[key] = val
        
        with open(args.output, 'w') as f:
            json.dump(output_data, f, indent=2)
        print(f"\n[SAVE] Results saved to {args.output}")
