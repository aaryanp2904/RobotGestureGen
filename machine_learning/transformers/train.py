"""
Train a GestureTransformer model on a preprocessed LMDB gesture dataset.

Usage:
    python train.py --data-dir /path/to/dataset.lmdb --epochs 50
    python train.py --data-dir /path/to/train.lmdb --val-data-dir /path/to/val.lmdb
    python train.py --data-dir /path/to/dataset.lmdb --sanity-check
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from pathlib import Path
import argparse
import time
import lmdb
import pickle
import numpy as np

from pos_end import GestureTransformer


# ---------------------------------------------------------------------------
# Dataset
# ---------------------------------------------------------------------------

class PreprocessedLMDBDataset(Dataset):
    """
    Lazily reads sliding windows from an LMDB database.

    BEAT2 windows can be large, especially with text embeddings, so loading the
    whole database into RAM can be killed by the OS. Keep one read-only LMDB
    handle per process instead.
    """

    def __init__(self, lmdb_dir):
        self.lmdb_dir = str(Path(lmdb_dir))
        self._env = None
        env = lmdb.open(
            self.lmdb_dir, readonly=True, lock=False, readahead=False,
            meminit=False, max_readers=2048,
        )

        with env.begin() as txn:
            length_bytes = txn.get(b"__len__")
            if length_bytes is None:
                raise ValueError(f"No b'__len__' key in LMDB at {self.lmdb_dir}")
            self.length = pickle.loads(length_bytes)
            metadata_bytes = txn.get(b"__metadata__")
            self.metadata = pickle.loads(metadata_bytes) if metadata_bytes else {
                "input_dim": 1536,
                "target_shape": [12, 3],
                "target_type": "genea_root_relative_xyz",
            }

        env.close()

        print(f"[DATASET] Lazy LMDB: {self.lmdb_dir}", flush=True)
        print(f"[DATASET] Windows: {self.length}", flush=True)
        print(f"[DATASET] Metadata: input_dim={self.metadata['input_dim']}, "
              f"target_shape={self.metadata['target_shape']}", flush=True)

    def _get_env(self):
        if self._env is None:
            self._env = lmdb.open(
                self.lmdb_dir, readonly=True, lock=False, readahead=False,
                meminit=False, max_readers=2048,
            )
        return self._env

    @staticmethod
    def _to_f32(v):
        if isinstance(v, np.ndarray):
            return torch.from_numpy(v.astype(np.float32))
        return v.float()

    def __len__(self):
        return self.length

    def __getitem__(self, idx):
        with self._get_env().begin() as txn:
            value = txn.get(f"{idx:08d}".encode())
        if value is None:
            raise KeyError(f"Key {idx:08d} not found in LMDB")
        raw = pickle.loads(value)
        return self._to_f32(raw["x"]), self._to_f32(raw["y"])


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------

def train_model(dataloader, epochs=50, sanity_check=False, device=None,
                model_config=None, val_dataloader=None, checkpoint_dir="."):
    """Train or sanity-check the GestureTransformer."""
    if device is None:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[TRAIN] Device: {device}", flush=True)
    if device.type == "cuda":
        print(f"[TRAIN] GPU: {torch.cuda.get_device_name(0)}", flush=True)

    if model_config is None:
        model_config = {}
    model = GestureTransformer(**model_config).to(device)
    model.config = dict(model_config)
    criterion = nn.MSELoss()
    lr = 1e-3 if sanity_check else 1e-4
    optimizer = optim.AdamW(model.parameters(), lr=lr)
    scaler = torch.amp.GradScaler("cuda", enabled=(device.type == "cuda"))

    if sanity_check:
        _sanity_check(model, criterion, optimizer, scaler, dataloader, device)
    else:
        _full_train(
            model, criterion, optimizer, scaler, dataloader, epochs, device,
            val_dataloader=val_dataloader,
            checkpoint_dir=Path(checkpoint_dir),
        )

    return model


def _sanity_check(model, criterion, optimizer, scaler, dataloader, device):
    """Overfit a single batch 500 times to verify the model can learn."""
    print("\n[SANITY] Overfitting single batch × 500", flush=True)
    batch_x, batch_y = next(iter(dataloader))
    batch_x, batch_y = batch_x.to(device), batch_y.to(device)
    print(f"[SANITY] X: {batch_x.shape}, Y: {batch_y.shape}", flush=True)

    for i in range(500):
        model.train()
        optimizer.zero_grad(set_to_none=True)
        with torch.amp.autocast("cuda", enabled=(device.type == "cuda")):
            loss = criterion(model(batch_x), batch_y)
        scaler.scale(loss).backward()
        scaler.unscale_(optimizer)
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        scaler.step(optimizer)
        scaler.update()
        if (i + 1) % 50 == 0 or i == 0:
            print(f"[SANITY] {i + 1}/500  Loss: {loss.item():.8f}")

    print("[SANITY] ✓ Complete\n")


def _evaluate(model, criterion, dataloader, device):
    """Compute validation loss without gradient updates."""
    if dataloader is None:
        return None

    model.eval()
    total_loss = 0.0
    total_batches = 0
    with torch.no_grad():
        for bx, by in dataloader:
            bx = bx.to(device, non_blocking=True)
            by = by.to(device, non_blocking=True)
            with torch.amp.autocast("cuda", enabled=(device.type == "cuda")):
                loss = criterion(model(bx), by)
            total_loss += loss.item()
            total_batches += 1

    if total_batches == 0:
        return None
    return total_loss / total_batches


def _save_checkpoint(path, model, model_config, epoch, train_loss, val_loss=None):
    checkpoint = {
        "model_state_dict": model.state_dict(),
        "model_config": model_config,
        "epoch": epoch,
        "train_loss": train_loss,
        "val_loss": val_loss,
    }
    torch.save(checkpoint, path)


def _full_train(model, criterion, optimizer, scaler, dataloader, epochs, device,
                val_dataloader=None, checkpoint_dir=Path(".")):
    """Standard epoch-based training loop with AMP."""
    print(f"\n[TRAIN] {epochs} epochs, {len(dataloader)} batches/epoch\n", flush=True)
    if val_dataloader is not None:
        print(f"[VAL] {len(val_dataloader)} validation batches/epoch\n", flush=True)

    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    best_val_loss = float("inf")
    best_train_loss = float("inf")

    for epoch in range(epochs):
        model.train()
        total_loss = 0.0
        t0 = time.time()

        for batch_idx, (bx, by) in enumerate(dataloader):
            bx = bx.to(device, non_blocking=True)
            by = by.to(device, non_blocking=True)

            optimizer.zero_grad(set_to_none=True)
            with torch.amp.autocast("cuda", enabled=(device.type == "cuda")):
                loss = criterion(model(bx), by)
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            scaler.step(optimizer)
            scaler.update()

            total_loss += loss.item()
            if (batch_idx + 1) % 100 == 0:
                print(f"  Epoch {epoch+1} | Batch {batch_idx+1}/{len(dataloader)} | Loss: {loss.item():.6f}")

        avg = total_loss / len(dataloader)
        val_loss = _evaluate(model, criterion, val_dataloader, device)
        dt = time.time() - t0
        if val_loss is None:
            print(f"[TRAIN] Epoch {epoch+1}/{epochs} | Avg Loss: {avg:.6f} | {dt:.1f}s")
        else:
            print(
                f"[TRAIN] Epoch {epoch+1}/{epochs} | "
                f"Train Loss: {avg:.6f} | Val Loss: {val_loss:.6f} | {dt:.1f}s"
            )

        _save_checkpoint(
            checkpoint_dir / "gesture_transformer_latest.pth",
            model, getattr(model, "config", {}), epoch + 1, avg, val_loss,
        )

        is_best = False
        if val_loss is not None and val_loss < best_val_loss:
            best_val_loss = val_loss
            is_best = True
        elif val_loss is None and avg < best_train_loss:
            best_train_loss = avg
            is_best = True

        if is_best:
            _save_checkpoint(
                checkpoint_dir / "gesture_transformer_best.pth",
                model, getattr(model, "config", {}), epoch + 1, avg, val_loss,
            )
            metric_name = "val" if val_loss is not None else "train"
            metric_value = val_loss if val_loss is not None else avg
            print(f"[CHECKPOINT] ✓ New best {metric_name} loss: {metric_value:.6f}")

    torch.save(model.state_dict(), checkpoint_dir / "gesture_transformer_full_trained.pth")
    print(f"[TRAIN] ✓ Saved checkpoints to {checkpoint_dir}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train GestureTransformer")
    parser.add_argument("--data-dir", type=str, required=True, help="Path to .lmdb directory")
    parser.add_argument("--val-data-dir", type=str, default=None,
                        help="Optional path to validation .lmdb directory")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--output-dir", type=str, default=".",
                        help="Directory for latest/best/final checkpoints")
    parser.add_argument("--sanity-check", action="store_true",
                        help="Overfit single batch to verify model learns")
    args = parser.parse_args()

    dataset = PreprocessedLMDBDataset(args.data_dir)
    model_config = {
        "input_dim": int(dataset.metadata.get("input_dim", 1536)),
        "target_shape": tuple(dataset.metadata.get("target_shape", [12, 3])),
    }
    dataloader = DataLoader(
        dataset, batch_size=args.batch_size, shuffle=True,
        num_workers=0, pin_memory=True,
    )
    print(f"[MAIN] {len(dataloader)} batches (batch_size={args.batch_size})\n", flush=True)

    val_dataloader = None
    if not args.sanity_check:
        val_data_dir = args.val_data_dir
        if val_data_dir is None:
            candidate = Path(args.data_dir).parent / "val.lmdb"
            if candidate.is_dir():
                val_data_dir = str(candidate)
                print(f"[MAIN] Auto-detected validation LMDB: {val_data_dir}", flush=True)

        if val_data_dir:
            val_dataset = PreprocessedLMDBDataset(val_data_dir)
            val_model_config = {
                "input_dim": int(val_dataset.metadata.get("input_dim", 1536)),
                "target_shape": tuple(val_dataset.metadata.get("target_shape", [12, 3])),
            }
            if val_model_config != model_config:
                raise ValueError(
                    f"Validation LMDB shape metadata does not match training LMDB: "
                    f"{val_model_config} vs {model_config}"
                )
            val_dataloader = DataLoader(
                val_dataset, batch_size=args.batch_size, shuffle=False,
                num_workers=0, pin_memory=True,
            )
            print(f"[MAIN] {len(val_dataloader)} validation batches\n", flush=True)

    if args.sanity_check:
        train_model(dataloader, sanity_check=True, model_config=model_config)
    else:
        train_model(
            dataloader,
            epochs=args.epochs,
            model_config=model_config,
            val_dataloader=val_dataloader,
            checkpoint_dir=args.output_dir,
        )