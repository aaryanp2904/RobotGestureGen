"""Compatibility entry point for BEAT2/NAO preprocessing.

Latent diffusion intentionally reuses the existing BEATArc preprocessing
contract. This wrapper keeps the latent pipeline runnable from one package:

    python -m machine_learning.latent_diffusion.preprocess ...
"""

from __future__ import annotations

from BEATArc.preprocess_nao import main


if __name__ == "__main__":
    main()
