"""Latent autoencoder and conditional latent diffusion models."""

from __future__ import annotations

import math

import torch
import torch.nn as nn

from machine_learning.diffusion.model import (
    ConditionalMotionDenoiser,
    DiffusionSchedule,
    PositionalEncoding,
)


class MotionAutoencoder(nn.Module):
    """Temporal autoencoder that compresses motion frames into gesture latents."""

    def __init__(
        self,
        target_shape: tuple[int, ...],
        latent_dim: int = 32,
        hidden_dim: int = 256,
        num_heads: int = 8,
        num_encoder_layers: int = 3,
        num_decoder_layers: int = 3,
        dropout: float = 0.1,
        max_frames: int = 512,
    ):
        super().__init__()
        if not target_shape or any(dim <= 0 for dim in target_shape):
            raise ValueError("target_shape must contain positive dimensions")
        if latent_dim <= 0:
            raise ValueError("latent_dim must be positive")
        if hidden_dim <= 0:
            raise ValueError("hidden_dim must be positive")
        if hidden_dim % num_heads != 0:
            raise ValueError("hidden_dim must be divisible by num_heads")
        if num_encoder_layers <= 0 or num_decoder_layers <= 0:
            raise ValueError("encoder and decoder layer counts must be positive")

        self.target_shape = tuple(target_shape)
        self.motion_dim = math.prod(self.target_shape)
        self.latent_dim = int(latent_dim)
        self.hidden_dim = int(hidden_dim)
        self.max_frames = int(max_frames)

        self.input_norm = nn.LayerNorm(self.motion_dim)
        self.encoder_in = nn.Linear(self.motion_dim, hidden_dim)
        self.encoder_pos = PositionalEncoding(hidden_dim, max_len=max_frames)
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 4,
            dropout=dropout,
            batch_first=True,
            norm_first=True,
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_encoder_layers)
        self.latent_out = nn.Linear(hidden_dim, latent_dim)

        self.latent_norm = nn.LayerNorm(latent_dim)
        self.decoder_in = nn.Linear(latent_dim, hidden_dim)
        self.decoder_pos = PositionalEncoding(hidden_dim, max_len=max_frames)
        decoder_layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 4,
            dropout=dropout,
            batch_first=True,
            norm_first=True,
        )
        self.decoder = nn.TransformerEncoder(decoder_layer, num_layers=num_decoder_layers)
        self.output_norm = nn.LayerNorm(hidden_dim)
        self.output = nn.Linear(hidden_dim, self.motion_dim)

    def encode(self, motion: torch.Tensor) -> torch.Tensor:
        batch_size, frames = motion.shape[:2]
        if tuple(motion.shape[2:]) != self.target_shape:
            raise ValueError(
                f"Expected motion shape (B, T, {self.target_shape}), got {tuple(motion.shape)}"
            )
        motion_flat = motion.reshape(batch_size, frames, self.motion_dim)
        hidden = self.encoder_in(self.input_norm(motion_flat))
        hidden = self.encoder_pos(hidden)
        hidden = self.encoder(hidden)
        return self.latent_out(hidden)

    def decode(self, latents: torch.Tensor) -> torch.Tensor:
        if latents.ndim != 3 or latents.shape[-1] != self.latent_dim:
            raise ValueError(
                f"Expected latents shape (B, T, {self.latent_dim}), got {tuple(latents.shape)}"
            )
        batch_size, frames = latents.shape[:2]
        hidden = self.decoder_in(self.latent_norm(latents))
        hidden = self.decoder_pos(hidden)
        hidden = self.decoder(hidden)
        motion = self.output(self.output_norm(hidden))
        return motion.reshape(batch_size, frames, *self.target_shape)

    def forward(self, motion: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        latents = self.encode(motion)
        reconstruction = self.decode(latents)
        return reconstruction, latents


class LatentDenoiser(ConditionalMotionDenoiser):
    """Conditional denoiser that predicts clean gesture latents."""

    def __init__(
        self,
        input_dim: int,
        latent_dim: int = 32,
        hidden_dim: int = 256,
        num_heads: int = 8,
        num_layers: int = 6,
        dropout: float = 0.1,
        max_frames: int = 512,
        speaker_dim: int = 0,
        seed_conditioning: bool = False,
        seed_frames: int = 0,
        conditioning_layers: int = 2,
        cond_drop_prob: float = 0.0,
        target_shape: tuple[int, ...] | list[int] | None = None,
        target_mode: str | None = "latent",
        target_representation: str | None = "gesture_latent",
    ):
        if target_shape is not None:
            shape = tuple(target_shape)
            if shape != (latent_dim,):
                raise ValueError(
                    f"LatentDenoiser target_shape must be ({latent_dim},), got {shape}"
                )
        super().__init__(
            input_dim=input_dim,
            target_shape=(latent_dim,),
            hidden_dim=hidden_dim,
            num_heads=num_heads,
            num_layers=num_layers,
            dropout=dropout,
            max_frames=max_frames,
            speaker_dim=speaker_dim,
            seed_conditioning=seed_conditioning,
            seed_frames=seed_frames,
            conditioning_layers=conditioning_layers,
            conditioning_encoder="transformer",
            cross_attention=True,
            cond_drop_prob=cond_drop_prob,
            target_mode=target_mode,
            target_representation=target_representation,
        )
        self.latent_dim = int(latent_dim)


__all__ = ["DiffusionSchedule", "LatentDenoiser", "MotionAutoencoder"]
