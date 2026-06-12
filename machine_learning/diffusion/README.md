# Gesture Diffusion Training

This folder trains a conditional DDPM denoiser on the preprocessed gesture
windows produced by the existing preprocessing scripts.

The expected training record is:

- `x`: per-frame conditioning features, such as prosody/audio/text embeddings.
- `y`: normalized motion targets, such as NAO joint angles or root-relative XYZ.

The loader supports both current LMDB datasets and the older
`window_*_x.pt` / `window_*_y.pt` window layout.

## Quick Start

From the repo root:

```bash
python -m machine_learning.diffusion.train \
  --data-dir /path/to/train.lmdb \
  --val-data-dir /path/to/val.lmdb \
  --output-dir diffusion_checkpoints \
  --epochs 100 \
  --batch-size 64
```

If `--val-data-dir` is omitted and a sibling `val.lmdb` exists beside the
training LMDB, it is used automatically.

Run a single-batch learning check before a long job:

```bash
python -m machine_learning.diffusion.train \
  --data-dir /path/to/train.lmdb \
  --sanity-check
```

## Outputs

Training writes:

- `diffusion_latest.pth`
- `diffusion_best.pth`
- `diffusion_final.pth`
- `diffusion_training_config.json`

Each checkpoint stores model config, diffusion schedule config, dataset
metadata, optimizer state, and loss values.

## Model

`ConditionalMotionDenoiser` predicts the Gaussian noise added to a motion
window at a randomly sampled diffusion step. It uses a transformer encoder over
time and conditions each frame on:

- the noised motion frame,
- the matching preprocessed feature frame,
- a sinusoidal diffusion timestep embedding.

The trained model can be sampled with `DiffusionSchedule.sample(...)` by passing
conditioning features and the checkpoint target shape.
