"""Conditional diffusion model for gesture motion windows."""

from __future__ import annotations

import math

import torch
import torch.nn as nn


class SinusoidalPositionEmbeddings(nn.Module):
    """Embed integer diffusion timesteps with fixed sinusoidal features."""

    def __init__(self, dim: int):
        super().__init__()
        self.dim = dim

    def forward(self, timesteps: torch.Tensor) -> torch.Tensor:
        device = timesteps.device
        half_dim = self.dim // 2
        scale = math.log(10000.0) / max(half_dim - 1, 1)
        frequencies = torch.exp(torch.arange(half_dim, device=device) * -scale)
        args = timesteps.float().unsqueeze(1) * frequencies.unsqueeze(0)
        embedding = torch.cat([torch.sin(args), torch.cos(args)], dim=1)
        if self.dim % 2 == 1:
            embedding = torch.nn.functional.pad(embedding, (0, 1))
        return embedding


class PositionalEncoding(nn.Module):
    """Standard temporal sinusoidal positional encoding."""

    def __init__(self, d_model: int, max_len: int = 5000):
        super().__init__()
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(
            torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model)
        )
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term[: pe[:, 1::2].shape[1]])
        self.register_buffer("pe", pe.unsqueeze(0))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.size(1) > self.pe.size(1):
            raise ValueError(
                f"Sequence has {x.size(1)} frames, but positional encoding was "
                f"created for max_len={self.pe.size(1)}. Increase --max-frames."
            )
        return x + self.pe[:, : x.size(1), :]


class ConditionalMotionDenoiser(nn.Module):
    """
    Predict DDPM noise for a noised motion window conditioned on audio/text features.

    Inputs:
        noisy_motion: (batch, frames, *target_shape)
        conditioning: (batch, frames, input_dim)
        timesteps: (batch,)
    Output:
        Predicted noise with the same shape as ``noisy_motion``.
    """

    def __init__(
        self,
        input_dim: int,
        target_shape: tuple[int, ...],
        hidden_dim: int = 256,
        num_heads: int = 8,
        num_layers: int = 6,
        dropout: float = 0.1,
        max_frames: int = 512,
    ):
        super().__init__()
        if input_dim <= 0:
            raise ValueError("input_dim must be positive")
        if not target_shape or any(dim <= 0 for dim in target_shape):
            raise ValueError("target_shape must contain positive dimensions")
        if hidden_dim <= 0:
            raise ValueError("hidden_dim must be positive")
        if num_heads <= 0:
            raise ValueError("num_heads must be positive")
        if num_layers <= 0:
            raise ValueError("num_layers must be positive")
        if not 0.0 <= dropout < 1.0:
            raise ValueError("dropout must be in [0, 1)")
        if max_frames <= 0:
            raise ValueError("max_frames must be positive")
        if hidden_dim % num_heads != 0:
            raise ValueError(
                f"hidden_dim ({hidden_dim}) must be divisible by num_heads ({num_heads})"
            )
        self.input_dim = int(input_dim)
        self.target_shape = tuple(target_shape)
        self.motion_dim = math.prod(self.target_shape)

        self.timestep_mlp = nn.Sequential(
            SinusoidalPositionEmbeddings(hidden_dim),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
        )
        self.input_norm = nn.LayerNorm(self.input_dim)
        self.motion_norm = nn.LayerNorm(self.motion_dim)
        self.input_projection = nn.Sequential(
            nn.Linear(self.input_dim + self.motion_dim + hidden_dim, hidden_dim),
            nn.GELU(),
            nn.LayerNorm(hidden_dim),
        )
        self.position = PositionalEncoding(hidden_dim, max_len=max_frames)

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 4,
            dropout=dropout,
            batch_first=True,
            norm_first=True,
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        self.output_projection = nn.Linear(hidden_dim, self.motion_dim)

    def forward(
        self,
        noisy_motion: torch.Tensor,
        conditioning: torch.Tensor,
        timesteps: torch.Tensor,
    ) -> torch.Tensor:
        batch_size, frames = noisy_motion.shape[:2]
        if conditioning.shape[:2] != (batch_size, frames):
            raise ValueError(
                "conditioning and noisy_motion must have matching batch/frame dimensions"
            )
        if conditioning.shape[-1] != self.input_dim:
            raise ValueError(
                f"Expected conditioning dim {self.input_dim}, got {conditioning.shape[-1]}"
            )
        if tuple(noisy_motion.shape[2:]) != self.target_shape:
            raise ValueError(
                f"Expected motion shape {self.target_shape}, got {tuple(noisy_motion.shape[2:])}"
            )
        if timesteps.shape != (batch_size,):
            raise ValueError(f"Expected timesteps shape ({batch_size},), got {tuple(timesteps.shape)}")
        motion_flat = noisy_motion.reshape(batch_size, frames, self.motion_dim)
        motion_flat = self.motion_norm(motion_flat)
        conditioning = self.input_norm(conditioning)

        t_emb = self.timestep_mlp(timesteps).unsqueeze(1).expand(-1, frames, -1)
        hidden = torch.cat([motion_flat, conditioning, t_emb], dim=-1)
        hidden = self.input_projection(hidden)
        hidden = self.position(hidden)
        hidden = self.encoder(hidden)
        predicted = self.output_projection(hidden)
        return predicted.reshape(batch_size, frames, *self.target_shape)


class DiffusionSchedule(nn.Module):
    """DDPM forward and reverse coefficients."""

    def __init__(
        self,
        timesteps: int = 1000,
        beta_start: float = 1e-4,
        beta_end: float = 0.02,
    ):
        super().__init__()
        if timesteps <= 0:
            raise ValueError("timesteps must be positive")
        if not 0.0 < beta_start < 1.0:
            raise ValueError("beta_start must be in (0, 1)")
        if not 0.0 < beta_end < 1.0:
            raise ValueError("beta_end must be in (0, 1)")
        if beta_start >= beta_end:
            raise ValueError("beta_start must be smaller than beta_end")

        betas = torch.linspace(beta_start, beta_end, timesteps, dtype=torch.float32)
        alphas = 1.0 - betas
        alpha_cumprod = torch.cumprod(alphas, dim=0)
        alpha_cumprod_prev = torch.cat([torch.ones(1), alpha_cumprod[:-1]])

        self.timesteps = int(timesteps)
        self.register_buffer("betas", betas)
        self.register_buffer("alphas", alphas)
        self.register_buffer("alpha_cumprod", alpha_cumprod)
        self.register_buffer("alpha_cumprod_prev", alpha_cumprod_prev)
        self.register_buffer("sqrt_alpha_cumprod", torch.sqrt(alpha_cumprod))
        self.register_buffer("sqrt_one_minus_alpha_cumprod", torch.sqrt(1.0 - alpha_cumprod))
        posterior_variance = betas * (1.0 - alpha_cumprod_prev) / (1.0 - alpha_cumprod)
        self.register_buffer("posterior_variance", posterior_variance.clamp(min=1e-20))

    @staticmethod
    def _extract(values: torch.Tensor, timesteps: torch.Tensor, x_shape: torch.Size):
        out = values.gather(0, timesteps)
        return out.reshape(timesteps.shape[0], *((1,) * (len(x_shape) - 1)))

    def q_sample(
        self,
        clean_motion: torch.Tensor,
        timesteps: torch.Tensor,
        noise: torch.Tensor | None = None,
    ) -> torch.Tensor:
        if noise is None:
            noise = torch.randn_like(clean_motion)
        sqrt_alpha = self._extract(self.sqrt_alpha_cumprod, timesteps, clean_motion.shape)
        sqrt_one_minus = self._extract(
            self.sqrt_one_minus_alpha_cumprod, timesteps, clean_motion.shape
        )
        return sqrt_alpha * clean_motion + sqrt_one_minus * noise

    @torch.no_grad()
    def p_sample(
        self,
        model: nn.Module,
        noisy_motion: torch.Tensor,
        conditioning: torch.Tensor,
        timesteps: torch.Tensor,
    ) -> torch.Tensor:
        betas_t = self._extract(self.betas, timesteps, noisy_motion.shape)
        sqrt_one_minus = self._extract(
            self.sqrt_one_minus_alpha_cumprod, timesteps, noisy_motion.shape
        )
        sqrt_recip_alpha = self._extract(torch.sqrt(1.0 / self.alphas), timesteps, noisy_motion.shape)
        model_mean = sqrt_recip_alpha * (
            noisy_motion - betas_t * model(noisy_motion, conditioning, timesteps) / sqrt_one_minus
        )

        noise = torch.randn_like(noisy_motion)
        nonzero_mask = (timesteps != 0).float().reshape(
            timesteps.shape[0], *((1,) * (noisy_motion.dim() - 1))
        )
        variance = self._extract(self.posterior_variance, timesteps, noisy_motion.shape)
        return model_mean + nonzero_mask * torch.sqrt(variance) * noise

    @torch.no_grad()
    def sample(
        self,
        model: nn.Module,
        conditioning: torch.Tensor,
        target_shape: tuple[int, ...],
    ) -> torch.Tensor:
        model.eval()
        shape = (conditioning.shape[0], conditioning.shape[1], *target_shape)
        motion = torch.randn(shape, device=conditioning.device)
        for step in reversed(range(self.timesteps)):
            timesteps = torch.full(
                (conditioning.shape[0],),
                step,
                device=conditioning.device,
                dtype=torch.long,
            )
            motion = self.p_sample(model, motion, conditioning, timesteps)
        return motion
