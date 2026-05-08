"""
Train a GestureTransformer model on the preprocessed GENEA 2022 LMDB dataset.

Usage:
    python train.py --data-dir /path/to/dataset.lmdb --epochs 50
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
    Pre-loads all sliding windows from an LMDB database into RAM.

    This eliminates NFS I/O during training — each __getitem__ call
    is a simple list lookup. Suitable when the dataset fits in memory
    (e.g. ~1000 windows × 400 KB ≈ 400 MB).
    """

    def __init__(self, lmdb_dir):
        lmdb_dir = str(Path(lmdb_dir))
        env = lmdb.open(lmdb_dir, readonly=True, lock=False, readahead=True)

        with env.begin() as txn:
            length_bytes = txn.get(b"__len__")
            if length_bytes is None:
                raise ValueError(f"No b'__len__' key in LMDB at {lmdb_dir}")
            self.length = pickle.loads(length_bytes)

        print(f"[DATASET] Pre-loading {self.length} windows into RAM...", flush=True)
        self.data = []
        with env.begin() as txn:
            for idx in range(self.length):
                value = txn.get(f"{idx:08d}".encode())
                if value is None:
                    raise KeyError(f"Key {idx:08d} not found in LMDB")
                raw = pickle.loads(value)
                x = self._to_f32(raw["x"])
                y = self._to_f32(raw["y"])
                self.data.append((x, y))
                if (idx + 1) % 500 == 0:
                    print(f"[DATASET]   {idx + 1}/{self.length}", flush=True)

        env.close()
        print(f"[DATASET] ✓ {self.length} windows loaded", flush=True)

    @staticmethod
    def _to_f32(v):
        if isinstance(v, np.ndarray):
            return torch.from_numpy(v.astype(np.float32))
        return v.float()

    def __len__(self):
        return self.length

    def __getitem__(self, idx):
        return self.data[idx]


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------

def train_model(dataloader, epochs=50, sanity_check=False, device=None):
    """Train or sanity-check the GestureTransformer."""
    if device is None:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[TRAIN] Device: {device}", flush=True)
    if device.type == "cuda":
        print(f"[TRAIN] GPU: {torch.cuda.get_device_name(0)}", flush=True)

    model = GestureTransformer().to(device)
    criterion = nn.MSELoss()
    lr = 1e-3 if sanity_check else 1e-4
    optimizer = optim.AdamW(model.parameters(), lr=lr)
    scaler = torch.amp.GradScaler("cuda", enabled=(device.type == "cuda"))

    if sanity_check:
        _sanity_check(model, criterion, optimizer, scaler, dataloader, device)
    else:
        _full_train(model, criterion, optimizer, scaler, dataloader, epochs, device)

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


def _full_train(model, criterion, optimizer, scaler, dataloader, epochs, device):
    """Standard epoch-based training loop with AMP."""
    print(f"\n[TRAIN] {epochs} epochs, {len(dataloader)} batches/epoch\n", flush=True)

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
        dt = time.time() - t0
        print(f"[TRAIN] Epoch {epoch+1}/{epochs} | Avg Loss: {avg:.6f} | {dt:.1f}s")

    torch.save(model.state_dict(), "gesture_transformer_full_trained.pth")
    print("[TRAIN] ✓ Saved gesture_transformer_full_trained.pth")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train GestureTransformer")
    parser.add_argument("--data-dir", type=str, required=True, help="Path to .lmdb directory")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--sanity-check", action="store_true",
                        help="Overfit single batch to verify model learns")
    args = parser.parse_args()

    dataset = PreprocessedLMDBDataset(args.data_dir)
    dataloader = DataLoader(
        dataset, batch_size=args.batch_size, shuffle=True,
        num_workers=0, pin_memory=True,
    )
    print(f"[MAIN] {len(dataloader)} batches (batch_size={args.batch_size})\n", flush=True)

    if args.sanity_check:
        train_model(dataloader, sanity_check=True)
    else:
        train_model(dataloader, epochs=args.epochs)