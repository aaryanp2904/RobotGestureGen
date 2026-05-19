# Legacy Phase 1 Scripts

These scripts are archived from an earlier BEAT2 preprocessing approach that
produced cleaned SMPL-X motion, parsed annotations, optional SMPL-X FK positions,
and normalization statistics.

The current NAO workflow uses `BEATArc/preprocess_nao.py` instead. It extracts
prosody/text features and writes LMDB datasets with direct NAO joint-angle
targets.

Keep using the top-level wrapper paths if you need to run these older scripts:

```bash
python BEATArc/run_phase1.py
python BEATArc/validate_dataset.py
python BEATArc/extract_motion.py
python BEATArc/extract_positions.py
```
