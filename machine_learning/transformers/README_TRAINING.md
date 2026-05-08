# GestureTransformer Training & Evaluation Pipeline

Complete workflow for training a transformer-based gesture generation model with positional encodings.

## Architecture Overview

```
Input (1536-dim audio+text features)
    ↓
[Linear Projection] → 256-dim
    ↓
[Positional Encoding] → Add temporal context
    ↓
[Transformer Encoder] → 8 heads, 4 layers, 256-dim
    ↓
[Linear Projection] → 36-dim (12 joints × 3 coords)
    ↓
Output (Frames, 12, 3) normalized relative coordinates
    ↓
[Denormalization] → Absolute XYZ space
    ↓
[IK Solver] → NAO joint angles
```

## Quick Start

### Step 0: Data Preprocessing (Already Done ✓)

Your preprocessed data is at: `/vol/bitbucket/ap1922/PreprocessedGenea/trn/`
- Contains window_XXXXXX_x.pt (input features) and window_XXXXXX_y.pt (motion targets)

### Step 1: Sanity Check (5-10 minutes)

Run the sanity check to verify the architecture can overfit a single batch:

```bash
cd /homes/ap1922/Documents/ForthYear/RobotGestureGen/machine_learning/transformers

python train.py --data-dir /vol/bitbucket/ap1922/PreprocessedGenea/trn --sanity-check
```

**Expected Output:**
```
[SANITY CHECK] Iteration 10/500 - Loss: 0.234567
[SANITY CHECK] Iteration 50/500 - Loss: 0.045678
[SANITY CHECK] Iteration 500/500 - Loss: 0.000123
[SANITY CHECK] ✓ Complete!
```

**Pass Criteria:** Loss should drop to ~0.0001 or lower. If not, your preprocessing has a bug.

### Step 2: Full Training (2-4 hours on GPU)

Once sanity check passes, run full training in a **tmux/screen session** (to survive SSH disconnects):

```bash
# Create a tmux session
tmux new-session -d -s training

# Inside the session
tmux send-keys -t training "cd /homes/ap1922/Documents/ForthYear/RobotGestureGen/machine_learning/transformers" C-m
tmux send-keys -t training "python train.py --data-dir /vol/bitbucket/ap1922/PreprocessedGenea/trn --epochs 50" C-m

# Check progress
tmux attach -t training
```

**Training Settings:**
- Learning Rate: 1e-4 (stable baseline)
- Optimizer: AdamW with gradient clipping (max_norm=1.0)
- Epochs: 50 (adjust based on loss plateau)

**Monitor Loss:**
- Loss should steadily decrease each epoch
- If it plateaus, you can stop early or adjust learning rate
- Output: `gesture_transformer_full_trained.pth` (saved after training)

### Step 3: Generate Predictions (While training, build this)

Create normalization statistics JSON from your preprocessing:

```json
{
  "mean": [array of means for 36 dims],
  "std": [array of stds for 36 dims],
  "root_mean": [x, y, z mean of root joint]
}
```

Then generate predictions on test data:

```bash
python generate_gestures.py \
  --model gesture_transformer_full_trained.pth \
  --stats normalization_stats.json \
  --input /path/to/test_window_x.pt \
  --output predictions.npy
```

**What happens inside:**
1. Loads trained model weights
2. Passes test sequence through model (forward pass)
3. Denormalizes: `xyz_relative = (prediction × σ) + μ`
4. Re-adds root: `xyz_absolute = xyz_relative + root_coords`
5. Saves to predictions.npy

### Step 4: Evaluate Motion Quality

Compute automated metrics:

```bash
python eval_metrics.py \
  --predictions predictions.npy \
  --ground-truth /path/to/test_window_y.pt \
  --joint-angles /path/to/ik_joint_angles.npy \
  --output evaluation_report.json
```

**Metrics Computed:**

1. **Prediction Error:**
   - MSE Loss (per-frame mean squared error)
   - L1 Loss (per-frame absolute error)
   - Per-joint MSE breakdown

2. **Smoothness:**
   - Mean/Max Velocity (how fast joints move)
   - Mean/Max Acceleration (how rapidly velocity changes)
   - Mean Jerk (3rd derivative - smoothness)
   
3. **Joint Feasibility:**
   - % of frames within NAO hardware limits
   - Total constraint violations
   - Per-joint violation breakdown

**Example Output:**
```
[PREDICTION ERROR]
  MSE Loss:  0.001234
  L1 Loss:   0.042567

[SMOOTHNESS]
  Mean Velocity:       0.342 rad/s
  Max Velocity:        2.145 rad/s
  Mean Acceleration:   0.078 rad/s²
  Max Acceleration:    0.567 rad/s²

[JOINT FEASIBILITY]
  Feasible Frames:       98.5%
  Total Violations:      45
  Violation Rate:        0.15%
```

### Step 5: Integration with IK Solver

The denormalized predictions (xyz_absolute) are ready for your IK solver:

```python
# In main_ik_client.py
xyz_coords = predictions  # Shape: (Frames, 12, 3) - absolute XYZ
joint_angles = ik_solver(xyz_coords)  # Your IK pipeline
send_to_nao(joint_angles)  # Send to nao_server.py
```

## File Descriptions

- **train.py**: Main training script with sanity check mode
- **generate_gestures.py**: Load model + denormalize predictions
- **eval_metrics.py**: Compute motion quality metrics with NAO limits
- **pos_end.py**: Model architecture (GestureTransformer + PositionalEncoding)

## Important Notes

### Learning Rate Strategy
- **Sanity Check**: 1e-3 (higher LR for quick convergence on single batch)
- **Full Training**: 1e-4 (stable baseline, prevents overfitting)

### Data Format
- **Input**: (Batch, Frames, 1536) - concatenated [audio_768d, text_768d]
- **Output**: (Batch, Frames, 12, 3) - relative coordinates of 12 NAO joints
- **After denormalization**: (Frames, 12, 3) - absolute XYZ coordinates

### Normalization Reversal Math
```
normalized = (xyz_relative - mean) / std
xyz_relative = normalized × std + mean
xyz_absolute = xyz_relative + root_coords
```

### GPU Memory Tips
- Batch size 32 uses ~4-6GB VRAM
- If OOM, reduce batch size: `--batch-size 16`
- Window size: 60 frames (2 seconds @ 30 fps)

## Troubleshooting

**Training loss not decreasing?**
- Check that preprocessing is correct (run sanity check first)
- Try reducing learning rate: `lr=1e-5`
- Increase epochs or check data quality

**Sanity check loss plateaus above 0.001?**
- Architecture is correct, but preprocessing might have scaling issues
- Verify mean/std normalization in your preprocessing

**Out of memory?**
- Reduce batch size: `--batch-size 16`
- Reduce sequence length in preprocessing

## Next Steps

After evaluation metrics confirm quality:
1. Integrate with main_ik_client.py for real-time inference
2. Deploy to NAO robot via nao_server.py (Python 2.7)
3. Visualize results in Choregraphe
4. Write up results for First-Class write-up

---

**Timeline:** 
- Sanity Check: ~10 min
- Full Training: ~2-4 hours (GPU)
- Inference + Eval: ~10-20 min per test set
