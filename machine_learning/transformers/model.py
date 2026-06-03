"""Transformer model used by the BEAT2/NAO gesture baseline."""

from __future__ import annotations

import math

import torch
import torch.nn as nn


MODEL_INIT_KEYS = {
    "input_dim",
    "hidden_dim",
    "num_heads",
    "num_layers",
    "num_joints",
    "target_shape",
    "dropout",
}


def model_init_kwargs(model_config: dict) -> dict:
    """Keep checkpoint metadata tolerant of training-only keys."""
    return {key: value for key, value in model_config.items() if key in MODEL_INIT_KEYS}


class PositionalEncoding(nn.Module):
    """Standard sinusoidal positional encoding."""

    def __init__(self, d_model: int, max_len: int = 5000):
        super().__init__()
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float32).unsqueeze(1)
        div_term = torch.exp(
            torch.arange(0, d_model, 2, dtype=torch.float32)
            * (-math.log(10000.0) / d_model)
        )
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term[: pe[:, 1::2].shape[1]])
        self.register_buffer("pe", pe.unsqueeze(0))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x + self.pe[:, : x.size(1), :]


class GestureTransformer(nn.Module):
    """Encoder-only Transformer that maps frame features to NAO joint angles."""

    def __init__(
        self,
        input_dim: int = 9,
        hidden_dim: int = 256,
        num_heads: int = 8,
        num_layers: int = 4,
        num_joints: int = 10,
        target_shape: tuple[int, ...] | list[int] | None = None,
        dropout: float = 0.1,
    ):
        super().__init__()
        if target_shape is None:
            target_shape = (num_joints,)
        self.target_shape = tuple(target_shape)
        output_dim = math.prod(self.target_shape)

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
            dropout=dropout,
            batch_first=True,
            norm_first=True,
        )
        self.transformer_encoder = nn.TransformerEncoder(
            encoder_layer, num_layers=num_layers
        )
        self.output_projection = nn.Linear(hidden_dim, output_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.input_norm(x)
        x = self.input_projection(x)
        x = self.pos_encoder(x)
        x = self.transformer_encoder(x)
        x = self.output_projection(x)
        return x.view(x.size(0), x.size(1), *self.target_shape)
