"""Encode preprocessed motion windows into latent LMDB datasets."""

from __future__ import annotations

import argparse
import json
import pickle
import shutil
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

from .dataset import PreprocessedGestureDataset
from .model import MotionAutoencoder


def load_autoencoder(checkpoint_path: Path, device: torch.device) -> tuple[MotionAutoencoder, dict, dict]:
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    config = checkpoint.get("autoencoder_config")
    if not config:
        raise ValueError("Autoencoder checkpoint is missing autoencoder_config")
    config = dict(config)
    config["target_shape"] = tuple(config["target_shape"])
    state_dict = checkpoint.get("autoencoder_state_dict")
    if state_dict is None:
        raise ValueError("Autoencoder checkpoint is missing autoencoder_state_dict")
    model = MotionAutoencoder(**config).to(device)
    model.load_state_dict(state_dict)
    model.eval()
    return model, config, checkpoint.get("dataset_metadata") or {}


def write_latent_lmdb(args):
    try:
        import lmdb
    except ImportError as exc:
        raise ImportError("The lmdb package is required to write latent datasets.") from exc

    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")
    source = PreprocessedGestureDataset(args.data_dir)
    model, ae_config, ae_metadata = load_autoencoder(Path(args.autoencoder), device)

    source_shape = tuple(source.metadata.get("target_shape", []))
    if source_shape != tuple(ae_config["target_shape"]):
        raise ValueError(
            f"Dataset target_shape {source_shape} does not match autoencoder "
            f"target_shape {ae_config['target_shape']}"
        )

    output_dir = Path(args.output_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        if not args.overwrite:
            raise FileExistsError(f"{output_dir} is not empty. Pass --overwrite to replace it.")
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    metadata = dict(source.metadata)
    metadata.update(
        {
            "target_shape": [int(ae_config["latent_dim"])],
            "target_type": "latent_motion",
            "target_mode": "latent",
            "target_representation": "gesture_latent",
            "latent_dim": int(ae_config["latent_dim"]),
            "source_target_shape": list(source_shape),
            "source_target_mode": source.metadata.get("target_mode", "angle"),
            "source_target_type": source.metadata.get("target_type"),
            "source_target_representation": source.metadata.get("target_representation"),
            "autoencoder_config": {
                **ae_config,
                "target_shape": list(ae_config["target_shape"]),
            },
            "autoencoder_checkpoint": str(Path(args.autoencoder)),
            "autoencoder_dataset_metadata": ae_metadata,
        }
    )

    loader = DataLoader(
        source,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=torch.cuda.is_available() and not args.cpu,
    )
    env = lmdb.open(str(output_dir), map_size=args.map_size, subdir=True, lock=True)
    index = 0
    print(f"[LATENTS] Encoding {len(source)} windows to {output_dir} on {device}", flush=True)
    try:
        with env.begin(write=True) as txn:
            txn.put(b"__len__", pickle.dumps(len(source)))
            txn.put(b"__metadata__", pickle.dumps(metadata))
        txn = env.begin(write=True)
        with torch.no_grad():
            for batch_idx, (x, motion, speaker, valid_mask) in enumerate(loader):
                motion = motion.to(device, non_blocking=True)
                latents = model.encode(motion).cpu().numpy().astype(np.float32)
                x_np = x.numpy().astype(np.float32)
                speaker_np = speaker.numpy().astype(np.float32)
                mask_np = valid_mask.numpy().astype(np.float32)
                for item_idx in range(latents.shape[0]):
                    record = {
                        "x": x_np[item_idx],
                        "y": latents[item_idx],
                        "speaker": speaker_np[item_idx],
                        "valid_mask": mask_np[item_idx],
                    }
                    txn.put(f"{index:08d}".encode(), pickle.dumps(record, protocol=pickle.HIGHEST_PROTOCOL))
                    index += 1
                    if index % args.commit_every == 0:
                        txn.commit()
                        txn = env.begin(write=True)
                if batch_idx == 0 or (batch_idx + 1) % args.log_every == 0:
                    print(f"[LATENTS] Encoded {index}/{len(source)} windows", flush=True)
        txn.commit()
        env.sync()
    finally:
        env.close()

    with open(output_dir / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"[LATENTS] Complete: {output_dir}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Build latent LMDB from preprocessed gesture LMDB")
    parser.add_argument("--data-dir", required=True, help="Source train/val/test .lmdb")
    parser.add_argument("--autoencoder", required=True, help="Path to autoencoder_best.pth")
    parser.add_argument("--output-dir", required=True, help="Output latent .lmdb directory")
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--map-size", type=int, default=64 * 1024**3)
    parser.add_argument("--commit-every", type=int, default=2048,
                        help="Commit the LMDB write transaction every N records")
    parser.add_argument("--log-every", type=int, default=25)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--cpu", action="store_true")
    args = parser.parse_args()
    if args.commit_every <= 0:
        raise ValueError("--commit-every must be positive")
    write_latent_lmdb(args)


if __name__ == "__main__":
    main()
