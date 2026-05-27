"""Convert latent LMDB datasets into sequential shard files."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from .dataset import LatentGestureDataset


def cast_tensor(tensor: torch.Tensor, dtype: str) -> torch.Tensor:
    if dtype == "float16":
        return tensor.half()
    if dtype == "float32":
        return tensor.float()
    raise ValueError("--dtype must be float16 or float32")


def build_shards(args):
    source = LatentGestureDataset(args.data_dir, source_data_dir=args.source_data_dir)
    output_dir = Path(args.output_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        if not args.overwrite:
            raise FileExistsError(f"{output_dir} is not empty. Pass --overwrite to replace it.")
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    shards = []
    total = 0
    print(f"[SHARDS] Converting {len(source)} windows from {args.data_dir} to {output_dir}")
    compact = bool(getattr(source, "_compact", False))

    if compact:
        for shard_idx, start in enumerate(range(0, len(source), args.shard_size)):
            end = min(start + args.shard_size, len(source))
            records = [source._read_record(idx) for idx in range(start, end)]
            y = torch.stack([torch.as_tensor(record["y"]) for record in records])
            source_idx = torch.tensor(
                [int(record.get("source_idx", start + offset)) for offset, record in enumerate(records)],
                dtype=torch.long,
            )
            shard_name = f"shard_{shard_idx:05d}.pt"
            shard_path = output_dir / shard_name
            payload = {
                "y": cast_tensor(y, args.dtype).contiguous(),
                "source_idx": source_idx.contiguous(),
            }
            torch.save(payload, shard_path)
            length = end - start
            total += length
            shards.append({"file": shard_name, "length": length})
            if shard_idx == 0 or (shard_idx + 1) % args.log_every == 0:
                print(f"[SHARDS] Wrote {total}/{len(source)} windows")
    else:
        loader = DataLoader(
            source,
            batch_size=args.shard_size,
            shuffle=False,
            num_workers=args.num_workers,
            pin_memory=False,
        )
        for shard_idx, (x, y, speaker, valid_mask) in enumerate(loader):
            shard_name = f"shard_{shard_idx:05d}.pt"
            shard_path = output_dir / shard_name
            payload = {
                "x": cast_tensor(x, args.dtype).contiguous(),
                "y": cast_tensor(y, args.dtype).contiguous(),
                "speaker": speaker.float().contiguous(),
                "valid_mask": valid_mask.float().contiguous(),
            }
            torch.save(payload, shard_path)
            length = int(y.shape[0])
            total += length
            shards.append({"file": shard_name, "length": length})
            if shard_idx == 0 or (shard_idx + 1) % args.log_every == 0:
                print(f"[SHARDS] Wrote {total}/{len(source)} windows")

    manifest = {
        "format": "latent_shards_v2" if compact else "latent_shards_v1",
        "compact": compact,
        "num_samples": total,
        "shard_size": args.shard_size,
        "dtype": args.dtype,
        "metadata": source.metadata,
        "shards": shards,
    }
    with open(output_dir / "manifest.json", "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"[SHARDS] Complete: {output_dir}")


def main():
    parser = argparse.ArgumentParser(description="Convert latent LMDB to compact shard files")
    parser.add_argument("--data-dir", required=True, help="Input latent .lmdb directory")
    parser.add_argument("--output-dir", required=True, help="Output sharded latent directory")
    parser.add_argument("--shard-size", type=int, default=512,
                        help="Number of windows per shard")
    parser.add_argument("--dtype", choices=["float16", "float32"], default="float16",
                        help="Storage dtype for x/y arrays")
    parser.add_argument("--source-data-dir", default=None,
                        help="Override source preprocessed LMDB for compact latent datasets")
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--log-every", type=int, default=10)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if args.shard_size <= 0:
        raise ValueError("--shard-size must be positive")
    build_shards(args)


if __name__ == "__main__":
    main()
