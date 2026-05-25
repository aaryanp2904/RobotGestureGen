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


class ConditioningEncoder(nn.Module):
    """Project audio/text/prosody features into temporal conditioning tokens."""

    def __init__(
        self,
        input_dim: int,
        hidden_dim: int,
        num_heads: int,
        num_layers: int = 2,
        dropout: float = 0.1,
        max_frames: int = 512,
    ):
        super().__init__()
        self.input_norm = nn.LayerNorm(input_dim)
        self.input_projection = nn.Linear(input_dim, hidden_dim)
        self.position = PositionalEncoding(hidden_dim, max_len=max_frames)
        if num_layers > 0:
            layer = nn.TransformerEncoderLayer(
                d_model=hidden_dim,
                nhead=num_heads,
                dim_feedforward=hidden_dim * 4,
                dropout=dropout,
                batch_first=True,
                norm_first=True,
            )
            self.encoder = nn.TransformerEncoder(layer, num_layers=num_layers)
        else:
            self.encoder = nn.Identity()

    def forward(self, conditioning: torch.Tensor) -> torch.Tensor:
        hidden = self.input_projection(self.input_norm(conditioning))
        hidden = self.position(hidden)
        return self.encoder(hidden)


class CrossAttentionDenoiserBlock(nn.Module):
    """Transformer decoder-style block for noised motion tokens."""

    def __init__(self, hidden_dim: int, num_heads: int, dropout: float = 0.1):
        super().__init__()
        self.self_norm = nn.LayerNorm(hidden_dim)
        self.self_attn = nn.MultiheadAttention(
            hidden_dim, num_heads, dropout=dropout, batch_first=True
        )
        self.cross_norm = nn.LayerNorm(hidden_dim)
        self.cross_attn = nn.MultiheadAttention(
            hidden_dim, num_heads, dropout=dropout, batch_first=True
        )
        self.ff_norm = nn.LayerNorm(hidden_dim)
        self.ff = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim * 4),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim * 4, hidden_dim),
            nn.Dropout(dropout),
        )

    def forward(self, motion_tokens: torch.Tensor, conditioning_tokens: torch.Tensor) -> torch.Tensor:
        residual = motion_tokens
        hidden = self.self_norm(motion_tokens)
        hidden, _ = self.self_attn(hidden, hidden, hidden, need_weights=False)
        motion_tokens = residual + hidden

        residual = motion_tokens
        hidden = self.cross_norm(motion_tokens)
        hidden, _ = self.cross_attn(
            hidden, conditioning_tokens, conditioning_tokens, need_weights=False
        )
        motion_tokens = residual + hidden

        return motion_tokens + self.ff(self.ff_norm(motion_tokens))


class ConditionalMotionDenoiser(nn.Module):
    """
    Predict clean motion or DDPM noise for a noised motion window.

    Inputs:
        noisy_motion: (batch, frames, *target_shape)
        conditioning: (batch, frames, input_dim)
        timesteps: (batch,)
        seed_motion: optional (batch, frames, *target_shape)
        seed_mask: optional (batch, frames, 1), 1 where seed is known
        speaker: optional (batch, speaker_dim)
    Output:
        Prediction with the same shape as ``noisy_motion``.
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
        speaker_dim: int = 0,
        seed_conditioning: bool = False,
        seed_frames: int = 0,
        conditioning_layers: int = 2,
        conditioning_encoder: str = "transformer",
        cross_attention: bool = True,
        cond_drop_prob: float = 0.0,
        target_mode: str | None = None,
        target_representation: str | None = None,
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
        if speaker_dim < 0:
            raise ValueError("speaker_dim cannot be negative")
        if conditioning_encoder != "transformer":
            raise ValueError("conditioning_encoder must be 'transformer'")
        if not cross_attention:
            raise ValueError("ConditionalMotionDenoiser requires cross_attention=True")
        if not 0.0 <= cond_drop_prob < 1.0:
            raise ValueError("cond_drop_prob must be in [0, 1)")
        if hidden_dim % num_heads != 0:
            raise ValueError(
                f"hidden_dim ({hidden_dim}) must be divisible by num_heads ({num_heads})"
            )
        self.input_dim = int(input_dim)
        self.target_shape = tuple(target_shape)
        self.motion_dim = math.prod(self.target_shape)
        self.speaker_dim = int(speaker_dim)
        self.seed_conditioning = bool(seed_conditioning)
        self.seed_frames = int(seed_frames)
        self.conditioning_layers = int(conditioning_layers)
        self.conditioning_encoder_type = conditioning_encoder
        self.cross_attention = bool(cross_attention)
        self.cond_drop_prob = float(cond_drop_prob)
        self.target_mode = target_mode
        self.target_representation = target_representation

        self.timestep_mlp = nn.Sequential(
            SinusoidalPositionEmbeddings(hidden_dim),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
        )
        self.motion_norm = nn.LayerNorm(self.motion_dim)
        if self.seed_conditioning:
            self.seed_norm = nn.LayerNorm(self.motion_dim)
            seed_feature_dim = self.motion_dim + 1
        else:
            self.seed_norm = None
            seed_feature_dim = 0
        if self.speaker_dim > 0:
            self.speaker_projection = nn.Sequential(
                nn.Linear(self.speaker_dim, hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, hidden_dim),
            )
        else:
            self.speaker_projection = None
        self.conditioning_encoder = ConditioningEncoder(
            self.input_dim,
            hidden_dim,
            num_heads,
            num_layers=max(0, self.conditioning_layers),
            dropout=dropout,
            max_frames=max_frames,
        )
        self.motion_projection = nn.Linear(self.motion_dim + seed_feature_dim, hidden_dim)
        self.position = PositionalEncoding(hidden_dim, max_len=max_frames)
        self.blocks = nn.ModuleList(
            [
                CrossAttentionDenoiserBlock(hidden_dim, num_heads, dropout=dropout)
                for _ in range(num_layers)
            ]
        )
        self.output_norm = nn.LayerNorm(hidden_dim)
        self.output_projection = nn.Linear(hidden_dim, self.motion_dim)

    def forward(
        self,
        noisy_motion: torch.Tensor,
        conditioning: torch.Tensor,
        timesteps: torch.Tensor,
        seed_motion: torch.Tensor | None = None,
        seed_mask: torch.Tensor | None = None,
        speaker: torch.Tensor | None = None,
        force_unconditional: bool = False,
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
        if force_unconditional:
            conditioning = torch.zeros_like(conditioning)

        motion_parts = [motion_flat]
        if self.seed_conditioning:
            if seed_motion is None:
                seed_motion = torch.zeros_like(noisy_motion)
            if seed_mask is None:
                seed_mask = torch.zeros((batch_size, frames, 1), device=noisy_motion.device, dtype=noisy_motion.dtype)
            if tuple(seed_motion.shape) != tuple(noisy_motion.shape):
                raise ValueError(
                    f"Expected seed_motion shape {tuple(noisy_motion.shape)}, got {tuple(seed_motion.shape)}"
                )
            if seed_mask.shape != (batch_size, frames, 1):
                raise ValueError(f"Expected seed_mask shape ({batch_size}, {frames}, 1), got {tuple(seed_mask.shape)}")
            seed_flat = seed_motion.reshape(batch_size, frames, self.motion_dim)
            seed_flat = self.seed_norm(seed_flat)
            motion_parts.append(torch.cat([seed_flat, seed_mask.to(seed_flat.dtype)], dim=-1))
        motion_tokens = self.motion_projection(torch.cat(motion_parts, dim=-1))
        t_emb = self.timestep_mlp(timesteps).unsqueeze(1)
        motion_tokens = motion_tokens + t_emb
        if self.speaker_projection is not None:
            if speaker is None:
                speaker = torch.zeros((batch_size, self.speaker_dim), device=noisy_motion.device, dtype=noisy_motion.dtype)
            if speaker.shape != (batch_size, self.speaker_dim):
                raise ValueError(f"Expected speaker shape ({batch_size}, {self.speaker_dim}), got {tuple(speaker.shape)}")
            if force_unconditional:
                speaker = torch.zeros_like(speaker)
            motion_tokens = motion_tokens + self.speaker_projection(speaker).unsqueeze(1)
        motion_tokens = self.position(motion_tokens)
        conditioning_tokens = self.conditioning_encoder(conditioning)
        for block in self.blocks:
            motion_tokens = block(motion_tokens, conditioning_tokens)
        predicted = self.output_projection(self.output_norm(motion_tokens))
        return predicted.reshape(batch_size, frames, *self.target_shape)


class DiffusionSchedule(nn.Module):
    """DDPM forward and reverse coefficients."""

    def __init__(
        self,
        timesteps: int = 1000,
        beta_start: float = 1e-4,
        beta_end: float = 0.02,
        prediction_type: str = "epsilon",
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
        if prediction_type not in {"x0", "epsilon"}:
            raise ValueError("prediction_type must be 'x0' or 'epsilon'")
        self.prediction_type = prediction_type

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
        self.register_buffer(
            "posterior_mean_coef1",
            betas * torch.sqrt(alpha_cumprod_prev) / (1.0 - alpha_cumprod),
        )
        self.register_buffer(
            "posterior_mean_coef2",
            (1.0 - alpha_cumprod_prev) * torch.sqrt(alphas) / (1.0 - alpha_cumprod),
        )

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
        seed_motion: torch.Tensor | None = None,
        seed_mask: torch.Tensor | None = None,
        speaker: torch.Tensor | None = None,
        add_noise: bool = True,
        guidance_scale: float = 1.0,
    ) -> torch.Tensor:
        model_output = model(
            noisy_motion,
            conditioning,
            timesteps,
            seed_motion=seed_motion,
            seed_mask=seed_mask,
            speaker=speaker,
        )
        if guidance_scale != 1.0:
            uncond_output = model(
                noisy_motion,
                conditioning,
                timesteps,
                seed_motion=seed_motion,
                seed_mask=seed_mask,
                speaker=speaker,
                force_unconditional=True,
            )
            model_output = uncond_output + guidance_scale * (model_output - uncond_output)
        if self.prediction_type == "x0":
            pred_x0 = model_output
        else:
            sqrt_recip_alpha_bar = self._extract(
                torch.sqrt(1.0 / self.alpha_cumprod), timesteps, noisy_motion.shape
            )
            sqrt_recipm1_alpha_bar = self._extract(
                torch.sqrt(1.0 / self.alpha_cumprod - 1.0), timesteps, noisy_motion.shape
            )
            pred_x0 = sqrt_recip_alpha_bar * noisy_motion - sqrt_recipm1_alpha_bar * model_output
        coef1 = self._extract(self.posterior_mean_coef1, timesteps, noisy_motion.shape)
        coef2 = self._extract(self.posterior_mean_coef2, timesteps, noisy_motion.shape)
        model_mean = coef1 * pred_x0 + coef2 * noisy_motion

        noise = torch.randn_like(noisy_motion)
        nonzero_mask = (timesteps != 0).float().reshape(
            timesteps.shape[0], *((1,) * (noisy_motion.dim() - 1))
        )
        variance = self._extract(self.posterior_variance, timesteps, noisy_motion.shape)
        if not add_noise:
            return model_mean
        return model_mean + nonzero_mask * torch.sqrt(variance) * noise

    @torch.no_grad()
    def sample(
        self,
        model: nn.Module,
        conditioning: torch.Tensor,
        target_shape: tuple[int, ...],
        seed_motion: torch.Tensor | None = None,
        seed_mask: torch.Tensor | None = None,
        speaker: torch.Tensor | None = None,
        add_noise: bool = True,
        sampler: str = "ddpm",
        sample_steps: int | None = None,
        guidance_scale: float = 1.0,
    ) -> torch.Tensor:
        model.eval()
        shape = (conditioning.shape[0], conditioning.shape[1], *target_shape)
        motion = torch.randn(shape, device=conditioning.device)
        if sampler == "ddim":
            return self.ddim_sample(
                model,
                conditioning,
                target_shape,
                sample_steps=sample_steps or min(50, self.timesteps),
                seed_motion=seed_motion,
                seed_mask=seed_mask,
                speaker=speaker,
                guidance_scale=guidance_scale,
            )
        if sampler != "ddpm":
            raise ValueError("sampler must be 'ddpm' or 'ddim'")
        for step in reversed(range(self.timesteps)):
            timesteps = torch.full(
                (conditioning.shape[0],),
                step,
                device=conditioning.device,
                dtype=torch.long,
            )
            motion = self.p_sample(
                model,
                motion,
                conditioning,
                timesteps,
                seed_motion=seed_motion,
                seed_mask=seed_mask,
                speaker=speaker,
                add_noise=add_noise,
                guidance_scale=guidance_scale,
            )
        return motion

    @torch.no_grad()
    def ddim_sample(
        self,
        model: nn.Module,
        conditioning: torch.Tensor,
        target_shape: tuple[int, ...],
        sample_steps: int,
        seed_motion: torch.Tensor | None = None,
        seed_mask: torch.Tensor | None = None,
        speaker: torch.Tensor | None = None,
        guidance_scale: float = 1.0,
    ) -> torch.Tensor:
        if sample_steps <= 0:
            raise ValueError("sample_steps must be positive")
        model.eval()
        shape = (conditioning.shape[0], conditioning.shape[1], *target_shape)
        motion = torch.randn(shape, device=conditioning.device)
        steps = torch.linspace(
            self.timesteps - 1,
            0,
            min(sample_steps, self.timesteps),
            device=conditioning.device,
        ).long()
        for idx, step in enumerate(steps):
            timesteps = torch.full(
                (conditioning.shape[0],),
                int(step.item()),
                device=conditioning.device,
                dtype=torch.long,
            )
            model_output = model(
                motion,
                conditioning,
                timesteps,
                seed_motion=seed_motion,
                seed_mask=seed_mask,
                speaker=speaker,
            )
            if guidance_scale != 1.0:
                uncond_output = model(
                    motion,
                    conditioning,
                    timesteps,
                    seed_motion=seed_motion,
                    seed_mask=seed_mask,
                    speaker=speaker,
                    force_unconditional=True,
                )
                model_output = uncond_output + guidance_scale * (model_output - uncond_output)

            alpha_t = self._extract(self.alpha_cumprod, timesteps, motion.shape)
            if self.prediction_type == "epsilon":
                pred_noise = model_output
                pred_x0 = (
                    motion - torch.sqrt(1.0 - alpha_t) * pred_noise
                ) / torch.sqrt(alpha_t)
            else:
                pred_x0 = model_output
                pred_noise = (
                    motion - torch.sqrt(alpha_t) * pred_x0
                ) / torch.sqrt(1.0 - alpha_t).clamp(min=1e-8)

            if idx + 1 >= len(steps):
                motion = pred_x0
                continue
            next_step = torch.full(
                (conditioning.shape[0],),
                int(steps[idx + 1].item()),
                device=conditioning.device,
                dtype=torch.long,
            )
            alpha_next = self._extract(self.alpha_cumprod, next_step, motion.shape)
            motion = torch.sqrt(alpha_next) * pred_x0 + torch.sqrt(1.0 - alpha_next) * pred_noise
        return motion
