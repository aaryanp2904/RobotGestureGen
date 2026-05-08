"""
GestureTransformer — Encoder-only Transformer for motion prediction.

Input:  (Batch, Frames, 1536)  — concatenated Wav2Vec2 + DistilBERT features
Output: (Batch, Frames, 12, 3) — root-relative XYZ positions for 12 upper-body joints
"""

import torch
import torch.nn as nn
import math


class PositionalEncoding(nn.Module):
    """Standard sinusoidal positional encoding."""

    def __init__(self, d_model, max_len=5000):
        super().__init__()
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        self.register_buffer("pe", pe.unsqueeze(0))

    def forward(self, x):
        return x + self.pe[:, : x.size(1), :]


class GestureTransformer(nn.Module):
    """
    Encoder-only Transformer that maps fused audio+text features
    to per-frame upper-body joint positions.

    Architecture:
        LayerNorm → Linear(1536→256) → GELU → LayerNorm
        → Positional Encoding
        → TransformerEncoder (4 layers, 8 heads, pre-norm)
        → Linear(256→36) → reshape to (B, T, 12, 3)
    """

    def __init__(self, input_dim=1536, hidden_dim=256, num_heads=8,
                 num_layers=4, num_joints=12):
        super().__init__()
        self.num_joints = num_joints

        self.input_norm = nn.LayerNorm(input_dim)
        self.input_projection = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.GELU(),
            nn.LayerNorm(hidden_dim),
        )
        self.pos_encoder = PositionalEncoding(hidden_dim)

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 4,
            batch_first=True,
            norm_first=True,
        )
        self.transformer_encoder = nn.TransformerEncoder(
            encoder_layer, num_layers=num_layers
        )
        self.output_projection = nn.Linear(hidden_dim, num_joints * 3)

    def forward(self, x):
        x = self.input_norm(x)
        x = self.input_projection(x)
        x = self.pos_encoder(x)
        x = self.transformer_encoder(x)
        x = self.output_projection(x)
        return x.view(x.size(0), x.size(1), self.num_joints, 3)