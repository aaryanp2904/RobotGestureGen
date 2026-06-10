"""Shared matplotlib styling for gesture evaluation figures."""

from __future__ import annotations

import matplotlib.pyplot as plt
from matplotlib import colors as mcolors
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

FIGURE_FACECOLOR = "#F7F9FC"
AXES_FACECOLOR = "#FFFFFF"
TEXT_COLOR = "#2D3748"
MUTED_TEXT = "#718096"
GRID_COLOR = "#E8EDF3"
SPINE_COLOR = "#C5CED8"
EDGE_COLOR = "#A8B4C0"
ACCENT_LINE = "#4A5568"

COLOR_GROUND_TRUTH = "#4A7C9B"
COLOR_TRANSFORMER = "#C97B84"
COLOR_TRANSFORMER_DELTA = "#9B7EBD"
COLOR_DIFFUSION = "#6B9E78"
COLOR_LATENT_DIFFUSION = "#D4A24E"

MODEL_COLORS = {
    "ground_truth": COLOR_GROUND_TRUTH,
    "transformer": COLOR_TRANSFORMER,
    "transformer_delta": COLOR_TRANSFORMER_DELTA,
    "diffusion": COLOR_DIFFUSION,
    "latent_diffusion": COLOR_LATENT_DIFFUSION,
}


def darker_edge(color: str, amount: float = 0.72) -> str:
    rgb = mcolors.to_rgb(color)
    return mcolors.to_hex(tuple(max(0.0, min(1.0, channel * amount)) for channel in rgb))


def configure_style() -> None:
    plt.rcParams.update(
        {
            "figure.figsize": (10, 6),
            "figure.dpi": 140,
            "figure.facecolor": FIGURE_FACECOLOR,
            "axes.facecolor": AXES_FACECOLOR,
            "savefig.dpi": 300,
            "savefig.facecolor": FIGURE_FACECOLOR,
            "savefig.edgecolor": FIGURE_FACECOLOR,
            "font.family": "sans-serif",
            "font.sans-serif": ["DejaVu Sans", "Helvetica", "Arial", "Liberation Sans"],
            "font.size": 13,
            "axes.titlesize": 16,
            "axes.titleweight": "600",
            "axes.labelsize": 14,
            "axes.labelcolor": TEXT_COLOR,
            "axes.titlecolor": TEXT_COLOR,
            "xtick.labelsize": 11.5,
            "ytick.labelsize": 11.5,
            "xtick.color": MUTED_TEXT,
            "ytick.color": MUTED_TEXT,
            "legend.fontsize": 11,
            "legend.title_fontsize": 11,
            "text.color": TEXT_COLOR,
            "axes.edgecolor": SPINE_COLOR,
            "axes.linewidth": 0.9,
            "axes.grid": False,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "grid.color": GRID_COLOR,
            "grid.linewidth": 0.8,
            "grid.alpha": 1.0,
            "lines.linewidth": 2.2,
            "patch.linewidth": 0.8,
        }
    )


def style_axes(ax, *, grid_axis: str = "y") -> None:
    ax.set_facecolor(AXES_FACECOLOR)
    ax.tick_params(axis="both", colors=MUTED_TEXT, width=0.8, length=4)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(SPINE_COLOR)
        ax.spines[spine].set_linewidth(0.9)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    if grid_axis == "y":
        ax.grid(True, axis="y", color=GRID_COLOR, linestyle="-", linewidth=0.8, alpha=1.0)
        ax.set_axisbelow(True)
    elif grid_axis == "both":
        ax.grid(True, which="both", axis="both", color=GRID_COLOR, linestyle="-", linewidth=0.7, alpha=1.0)
        ax.set_axisbelow(True)


def place_legend_below(
    ax,
    handles: list,
    *,
    ncol: int = 1,
    anchor_y: float = -0.12,
    fontsize: float = 10,
    title: str | None = None,
) -> None:
    legend = ax.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.5, anchor_y),
        ncol=ncol,
        frameon=True,
        fancybox=True,
        framealpha=0.97,
        facecolor=AXES_FACECOLOR,
        edgecolor=GRID_COLOR,
        fontsize=fontsize,
        title=title,
        borderaxespad=0.0,
    )
    if legend.get_title() is not None:
        legend.get_title().set_color(MUTED_TEXT)
        legend.get_title().set_fontweight("600")


def finalize_figure(
    fig,
    ax,
    *,
    note: str | None = None,
    bottom: float = 0.28,
    top: float = 0.90,
) -> None:
    style_axes(ax)
    if note:
        fig.text(0.5, 0.01, note, ha="center", va="bottom", fontsize=9.5, color=MUTED_TEXT)
        bottom = max(bottom, 0.26)
    fig.subplots_adjust(left=0.14, right=0.98, top=top, bottom=bottom)


def color_patch(color: str, label: str) -> Patch:
    return Patch(
        facecolor=color,
        edgecolor=darker_edge(color),
        linewidth=0.9,
        label=label,
        alpha=0.92,
    )


def sem_errorbar_legend_handle() -> Line2D:
    return Line2D(
        [0],
        [0],
        color=ACCENT_LINE,
        linewidth=1.4,
        marker="|",
        markersize=8,
        label="Error bars: ±1 SEM",
    )


def save_figure(output_dir, stem: str) -> None:
    from pathlib import Path

    fig = plt.gcf()
    for ax in fig.axes:
        style_axes(ax)
    out = Path(output_dir)
    fig.savefig(out / f"{stem}.png", bbox_inches="tight", pad_inches=0.18, facecolor=FIGURE_FACECOLOR)
    fig.savefig(out / f"{stem}.pdf", bbox_inches="tight", pad_inches=0.18, facecolor=FIGURE_FACECOLOR)
    plt.close(fig)


def bar_kwargs(color: str) -> dict:
    return {
        "color": color,
        "edgecolor": darker_edge(color),
        "linewidth": 0.9,
        "alpha": 0.9,
        "zorder": 3,
    }


def errorbar_kwargs() -> dict:
    return {
        "elinewidth": 1.4,
        "ecolor": ACCENT_LINE,
        "capthick": 1.4,
        "capsize": 5,
        "alpha": 0.85,
        "zorder": 4,
    }


def style_boxplot(box: dict, colors: list[str]) -> None:
    for patch, color in zip(box["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_edgecolor(darker_edge(color))
        patch.set_alpha(0.82)
        patch.set_linewidth(1.0)
    for element in ("whiskers", "caps"):
        for artist in box[element]:
            artist.set_color(SPINE_COLOR)
            artist.set_linewidth(1.0)
    for median in box["medians"]:
        median.set_color(ACCENT_LINE)
        median.set_linewidth(1.8)
    for flier in box.get("fliers", []):
        flier.set_markerfacecolor(MUTED_TEXT)
        flier.set_markeredgecolor("none")
        flier.set_alpha(0.45)
        flier.set_markersize(4)
