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
- `preprocess_nao.py`: build prosody/text features and direct NAO joint targets.
- `infer_nao.py`: generate NAO joint-angle predictions from WAV/TextGrid input.
- `play_nao_predictions.py`: play generated predictions on the local NAO server.
- `remote_infer_pull.py`: local helper for remote inference and file transfer.
- `nao_constants.py`: shared NAO joint order, limits, and velocity caps.
- `smplx_utils.py`: small SMPL-X math helpers used by active scripts.
- `parse_annotations.py`: TextGrid and semantic annotation parsing.
- `config.py`: dataset paths and BEAT2 constants.
