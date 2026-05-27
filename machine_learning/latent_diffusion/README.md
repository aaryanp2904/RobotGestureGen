# Latent Diffusion Gesture Training

This package implements the latent diffusion pipeline for BEAT2/NAO gesture
generation. It reuses the existing BEATArc preprocessing format:

- `x`: per-frame conditioning features, such as prosody, WavLM, and text.
- `y`: normalized NAO motion targets from `BEATArc/preprocess_nao.py`.
- `speaker`: optional one-hot speaker conditioning.
- `valid_mask`: valid frames in padded windows.

The model is trained in three stages:

1. Train a motion autoencoder on normalized motion windows.
2. Encode each motion window into compact latents and write latent LMDBs.
3. Train an audio-conditioned diffusion model that predicts clean latents.

Inference samples latents from audio, decodes them back to normalized NAO joint
angles, then uses the existing BEATArc denormalization, clamping, smoothing, and
velocity limiting.

## Do I Need To Preprocess Again?

No, not if you already have BEATArc preprocessed LMDBs and the matching
`normalization_stats.json`.

You can reuse existing preprocessing when you have:

- `train.lmdb`
- `val.lmdb` if you want validation
- `test.lmdb` if you want held-out evaluation
- `normalization_stats.json`

Preprocess again only if you changed the BEAT2 source data, changed the
conditioning features (`--include-wavlm`, `--include-text`, WavLM model, etc.),
changed target mode (`angle` vs `delta`), changed windowing, or lost the stats
file.

## Commands

From the repo root, with an existing preprocessed directory:

```bash
PRE=/vol/bitbucket/ap1922/BEAT2_NAO_Preprocessed
AE=/vol/bitbucket/ap1922/latent_autoencoder_checkpoints
LAT=/vol/bitbucket/ap1922/BEAT2_NAO_Latents
LD=/vol/bitbucket/ap1922/latent_diffusion_checkpoints
```

If preprocessing has not been run yet:

```bash
python -m machine_learning.latent_diffusion.preprocess \
  --output-dir "$PRE" \
  --include-wavlm \
  --target-mode angle \
  --window-size 2.0 \
  --stride 0.5
```

Train the motion autoencoder:

```bash
python -m machine_learning.latent_diffusion.train_autoencoder \
  --data-dir "$PRE/train.lmdb" \
  --val-data-dir "$PRE/val.lmdb" \
  --output-dir "$AE" \
  --epochs 75 \
  --batch-size 64 \
  --latent-dim 32 \
  --velocity-loss-weight 0.1
```

Build latent LMDBs:

```bash
python -m machine_learning.latent_diffusion.build_latent_dataset \
  --data-dir "$PRE/train.lmdb" \
  --autoencoder "$AE/autoencoder_best.pth" \
  --output-dir "$LAT/train_latent.lmdb"

python -m machine_learning.latent_diffusion.build_latent_dataset \
  --data-dir "$PRE/val.lmdb" \
  --autoencoder "$AE/autoencoder_best.pth" \
  --output-dir "$LAT/val_latent.lmdb"
```

Train latent diffusion:

```bash
python -m machine_learning.latent_diffusion.train_diffusion \
  --data-dir "$LAT/train_latent.lmdb" \
  --val-data-dir "$LAT/val_latent.lmdb" \
  --autoencoder "$AE/autoencoder_best.pth" \
  --output-dir "$LD" \
  --epochs 100 \
  --batch-size 64 \
  --diffusion-steps 100 \
  --latent-dim 32
```

Run local inference:

```bash
python -m machine_learning.latent_diffusion.infer_nao \
  --checkpoint "$LD/latent_diffusion_best.pth" \
  --stats "$PRE/normalization_stats.json" \
  --clip-id 10_kieks_0_103_103 \
  --audio-dir /vol/bitbucket/ap1922/BEAT2/beat_english_v2.0.0/wave16k \
  --textgrid-dir /vol/bitbucket/ap1922/BEAT2/beat_english_v2.0.0/textgrid \
  --output nao_predictions/10_kieks_0_103_103.npy \
  --sampler ddim \
  --sample-steps 50 \
  --velocity-limit
```

Run remote inference and pull the output:

```bash
python -m machine_learning.latent_diffusion.remote_infer_pull \
  10_kieks_0_103_103 \
  --checkpoint "$LD/latent_diffusion_best.pth" \
  --stats "$PRE/normalization_stats.json" \
  --sampler ddim \
  --sample-steps 50 \
  --velocity-limit
```

The remote helper prints the local `BEATArc/play_nao_predictions.py` command
after copying the `.npy`, metadata `.json`, WAV, and TextGrid files.
