#!/usr/bin/env python3
"""Compatibility wrapper for the archived Phase 1 SMPL-X position extractor."""

from pathlib import Path
import runpy
import sys


LEGACY_DIR = Path(__file__).resolve().parent / "legacy_phase1"

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.path.insert(0, str(LEGACY_DIR))
    runpy.run_path(str(LEGACY_DIR / "extract_positions.py"), run_name="__main__")
