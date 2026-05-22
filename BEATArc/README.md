# BEATArc

BEATArc contains the current BEAT2 to NAO workflow:

- inspect or play BEAT2 dataset motion on NAO with IK,
- preprocess BEAT2 audio/text/motion into NAO LMDB training data,
- run trained-model inference on a GPU machine,
- pull generated predictions back to the local machine and play them on NAO.

Older Phase 1 extraction scripts are archived in `legacy_phase1/`.

## Dataset Clip to NAO

Play a specific BEAT2 clip through the local NAO server:

```bash
python BEATArc/BEATDemo.py 10_kieks_0_103_103 --server http://localhost:8000
```

Pick a random clip:

```bash
python BEATArc/BEATDemo.py --random --server http://localhost:8000
```

Fetch a datapoint from the remote BEAT2 dataset, copy its WAV locally, and
export the original motion as NAO joint angles:

```bash
python BEATArc/fetch_nao_datapoint.py 10_kieks_0_103_103
```

Fetch, convert, and play it on the local NAO server:

```bash
python BEATArc/fetch_nao_datapoint.py 10_kieks_0_103_103 --play --server http://localhost:8000
```

Outputs are written under `nao_datapoints/<tag>/`:

- `<tag>.wav`: copied dataset audio,
- `<tag>.npz`: copied source motion,
- `<tag>.npy`: converted NAO angle sequence in `NAO_JOINTS` order,
- `<tag>.json`: metadata including FPS, frame window, duration, and joint order.

## NAO Training Preprocessing

Create split-specific LMDBs and normalization stats:

```bash
python BEATArc/preprocess_nao.py \
  --motion-dir /vol/bitbucket/ap1922/BEAT2/beat_english_v2.0.0/smplxflame_30 \
  --audio-dir /vol/bitbucket/ap1922/BEAT2/beat_english_v2.0.0/wave16k \
  --split-csv /vol/bitbucket/ap1922/BEAT2/beat_english_v2.0.0/train_test_split.csv \
  --output-dir /vol/bitbucket/ap1922/BEAT2_NAO_Preprocessed
```

Use environment variables from `config.py` for TextGrid and other default paths if
your dataset layout differs.

## Remote Inference

Run inference on `oak11.doc.ic.ac.uk` from your local machine and pull outputs:

```bash
python BEATArc/remote_infer_pull.py 10_kieks_0_103_103
```

Run inference, pull outputs, and start local playback:

```bash
python BEATArc/remote_infer_pull.py 10_kieks_0_103_103 --play
```

If the remote machine needs environment setup:

```bash
python BEATArc/remote_infer_pull.py 10_kieks_0_103_103 \
  --remote-setup "source .venv/bin/activate"
```

Run a diffusion checkpoint remotely and pull the generated files locally:

```bash
python BEATArc/remote_infer_pull.py 10_kieks_0_103_103 \
  --model-type diffusion \
  --checkpoint /vol/bitbucket/ap1922/path/to/diffusion_best.pth \
  --stats /vol/bitbucket/ap1922/BEAT2_NAO_Preprocessed/normalization_stats.json \
  --diffusion-deterministic \
  --smooth-window 9 \
  --velocity-limit \
  --seed 42
```

## Local Diffusion Inference

Run a diffusion checkpoint locally and write the prediction files directly:

```bash
python BEATArc/infer_nao.py \
  --checkpoint /path/to/diffusion_best.pth \
  --stats /path/to/BEAT2_NAO_Preprocessed/normalization_stats.json \
  --clip-id 10_kieks_0_103_103 \
  --audio-dir /path/to/beat_english_v2.0.0/wave16k \
  --textgrid-dir /path/to/beat_english_v2.0.0/textgrid \
  --output nao_predictions/10_kieks_0_103_103.npy \
  --diffusion-deterministic \
  --smooth-window 9 \
  --velocity-limit
```

`infer_nao.py` auto-detects diffusion checkpoints that contain
`diffusion_config`; pass `--seed 42` if you want reproducible sampling.

## Local Prediction Playback

Dry-run a generated prediction payload:

```bash
python BEATArc/play_nao_predictions.py \
  nao_predictions/10_kieks_0_103_103.npy \
  --wav nao_predictions/10_kieks_0_103_103.wav \
  --textgrid nao_predictions/10_kieks_0_103_103.TextGrid \
  --dry-run
```

Play it on the local NAO server:

```bash
python BEATArc/play_nao_predictions.py \
  nao_predictions/10_kieks_0_103_103.npy \
  --wav nao_predictions/10_kieks_0_103_103.wav \
  --textgrid nao_predictions/10_kieks_0_103_103.TextGrid \
  --server http://localhost:8000
```

## Active Files

- `BEATDemo.py`: map BEAT2 SMPL-X clips to NAO motion via IK and stream them.
- `fetch_nao_datapoint.py`: pull remote dataset clips locally and export/play original motion in NAO space.
- `preprocess_nao.py`: build prosody/text features and direct NAO joint targets.
- `infer_nao.py`: generate NAO joint-angle predictions from WAV/TextGrid input.
- `play_nao_predictions.py`: play generated predictions on the local NAO server.
- `remote_infer_pull.py`: local helper for remote inference and file transfer.
- `nao_constants.py`: shared NAO joint order, limits, and velocity caps.
- `smplx_utils.py`: small SMPL-X math helpers used by active scripts.
- `parse_annotations.py`: TextGrid and semantic annotation parsing.
- `config.py`: dataset paths and BEAT2 constants.
