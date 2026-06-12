# Legacy Phase 1 Scripts

These scripts are archived from an earlier BEAT2 preprocessing approach that
produced cleaned SMPL-X motion, parsed annotations, optional SMPL-X FK positions,
and normalization statistics.

The current NAO workflow uses `beat2_nao/preprocess_nao.py` instead. It extracts
prosody/text features and writes LMDB datasets with direct NAO joint-angle
targets.

Keep using the top-level wrapper paths if you need to run these older scripts:

```bash
python beat2_nao/run_phase1.py
python beat2_nao/validate_dataset.py
python beat2_nao/extract_motion.py
python beat2_nao/extract_positions.py
```
