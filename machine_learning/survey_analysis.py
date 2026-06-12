#!/usr/bin/env python3
"""Statistical analysis of the Robot Gesture Generation Realism survey.

Loads the Google Forms response export (.xlsx), applies pre-defined exclusion
criteria, computes Likert and ranking statistics with non-parametric tests,
and emits the report figures plus JSON artefacts.

The raw spreadsheet is intentionally kept out of the repository because the
free-text comments contain personal names; point ``--xlsx`` at its location.
Free-text comments are printed to stdout for reference but never written to
the committed JSON artefacts.

Analysis sets
-------------
full     all submitted responses (n = 22)
primary  zero-variance straight-liners removed (pre-defined criterion)
strict   sensitivity check: primary minus perfectly inverted rankings
         submitted during the late-night burst
"""

from __future__ import annotations

import argparse
import json
import sys
from itertools import combinations
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import colors as mcolors
from matplotlib.patches import Patch
from scipy import stats

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from machine_learning import plot_style as ps  # noqa: E402

DEFAULT_XLSX = Path.home() / "Downloads" / "Robot Gesture Generation Realism Survey (Responses).xlsx"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "notes" / "figures" / "evaluation" / "survey"

SCALES = ["Naturalness", "Appropriateness", "Humanlikeness", "Timing", "Comfort"]
N_SCALES = len(SCALES)

# Survey clip order -> experimental condition.
CONDITIONS = ["ground_truth", "latent_diffusion", "latent_diffusion_smoothed", "transformer"]
CONDITION_LABELS = {
    "ground_truth": "Ground truth",
    "latent_diffusion": "Latent diff. (raw)",
    "latent_diffusion_smoothed": "Latent diff. (smoothed)",
    "transformer": "Transformer",
}

# Google Forms export layout: per clip, 5 Likert columns then 1 comment column.
LIKERT_BLOCK_STARTS = {clip: 1 + 6 * clip_idx for clip_idx, clip in enumerate(CONDITIONS)}
RANK_COL_START = 25


# ---------------------------------------------------------------------------
# Loading and cleaning
# ---------------------------------------------------------------------------

def load_responses(xlsx_path: Path) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    raw = pd.read_excel(xlsx_path)
    cols = list(raw.columns)

    likert = {}
    comments = {}
    for cond, start in LIKERT_BLOCK_STARTS.items():
        block = raw[cols[start : start + N_SCALES]].apply(pd.to_numeric, errors="coerce")
        block.columns = [f"{cond}:{scale}" for scale in SCALES]
        likert[cond] = block
        comments[cond] = raw[cols[start + N_SCALES]]

    likert_df = pd.concat(likert.values(), axis=1)
    ranks_df = raw[cols[RANK_COL_START : RANK_COL_START + 4]].apply(pd.to_numeric, errors="coerce")
    ranks_df.columns = CONDITIONS
    comments_df = pd.DataFrame(comments)
    timestamps = pd.to_datetime(raw[cols[0]])

    bad_rank_rows = [
        int(i) for i, row in ranks_df.iterrows() if sorted(row.dropna().tolist()) != [1.0, 2.0, 3.0, 4.0]
    ]
    if bad_rank_rows:
        print(f"[WARN] Non-permutation rankings dropped from rank analyses: rows {bad_rank_rows}")

    missing = likert_df.isna().sum()
    missing = missing[missing > 0]
    if not missing.empty:
        print("[WARN] Missing Likert cells (complete-case handling per test):")
        for col, count in missing.items():
            rows = likert_df.index[likert_df[col].isna()].tolist()
            print(f"        {col}: {count} missing (rows {rows})")

    return likert_df, ranks_df.astype("Int64"), comments_df, timestamps.to_frame(name="timestamp")


def respondent_flags(likert_df: pd.DataFrame, ranks_df: pd.DataFrame, timestamps: pd.DataFrame) -> pd.DataFrame:
    flags = pd.DataFrame(index=likert_df.index)
    flags["likert_sd"] = likert_df.std(axis=1)
    flags["straight_liner"] = flags["likert_sd"].fillna(0.0) == 0.0
    flags["inverted_ranking"] = ranks_df.apply(
        lambda r: r.dropna().tolist() == [4, 3, 2, 1], axis=1
    )
    gaps = timestamps["timestamp"].diff().dt.total_seconds()
    flags["burst_submission"] = gaps < 120  # < 2 min after the previous response

    # Within-respondent agreement between the forced ranking and the
    # respondent's own mean rating per clip (+1 = fully consistent).
    composite = pd.DataFrame({cond: likert_df[[f"{cond}:{s}" for s in SCALES]].mean(axis=1) for cond in CONDITIONS})
    consistency = []
    for i in flags.index:
        ratings = composite.loc[i]
        ranking = ranks_df.loc[i].astype(float)
        if ratings.isna().any() or ranking.isna().any() or ratings.nunique() == 1:
            consistency.append(np.nan)
        else:
            consistency.append(stats.spearmanr(ratings, -ranking).statistic)
    flags["rank_rating_consistency"] = consistency
    return flags


def build_analysis_sets(flags: pd.DataFrame) -> dict[str, pd.Index]:
    full = flags.index
    primary = full[~flags["straight_liner"]]
    strict = primary[~flags.loc[primary, "inverted_ranking"]]
    return {"full": full, "primary": primary, "strict": strict}


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def holm_correction(pvalues: list[float]) -> list[float]:
    m = len(pvalues)
    order = np.argsort(pvalues)
    adjusted = np.full(m, np.nan)
    running_max = 0.0
    for rank, idx in enumerate(order):
        p = pvalues[idx]
        if np.isnan(p):
            continue
        adj = min(1.0, (m - rank) * p)
        running_max = max(running_max, adj)
        adjusted[idx] = running_max
    return adjusted.tolist()


def friedman_with_w(samples: list[np.ndarray]) -> dict:
    n = len(samples[0])
    k = len(samples)
    result = stats.friedmanchisquare(*samples)
    return {
        "n": int(n),
        "chi2": float(result.statistic),
        "df": k - 1,
        "p": float(result.pvalue),
        "kendalls_w": float(result.statistic / (n * (k - 1))),
    }


def cronbach_alpha(items: pd.DataFrame) -> float:
    complete = items.dropna().astype(float)
    k = complete.shape[1]
    item_var = complete.var(axis=0, ddof=1).sum()
    total_var = complete.sum(axis=1).var(ddof=1)
    return float(k / (k - 1) * (1.0 - item_var / total_var))


def likert_statistics(likert_df: pd.DataFrame, idx: pd.Index) -> dict:
    sub = likert_df.loc[idx]
    per_scale: dict[str, dict] = {}
    raw_pvalues = []
    for s_i, scale in enumerate(SCALES):
        scale_cols = [f"{cond}:{scale}" for cond in CONDITIONS]
        descriptives = {}
        for cond in CONDITIONS:
            vals = sub[f"{cond}:{scale}"].dropna().astype(float)
            descriptives[cond] = {
                "n": int(len(vals)),
                "median": float(vals.median()),
                "q1": float(vals.quantile(0.25)),
                "q3": float(vals.quantile(0.75)),
                "mean": float(vals.mean()),
                "sd": float(vals.std(ddof=1)),
            }
        complete = sub[scale_cols].dropna().astype(float)
        test = friedman_with_w([complete[c].to_numpy() for c in scale_cols])
        raw_pvalues.append(test["p"])
        per_scale[scale] = {"descriptives": descriptives, "friedman": test}

    for scale, p_holm in zip(SCALES, holm_correction(raw_pvalues)):
        per_scale[scale]["friedman"]["p_holm"] = p_holm

    alphas = {
        cond: cronbach_alpha(sub[[f"{cond}:{scale}" for scale in SCALES]]) for cond in CONDITIONS
    }
    composite = pd.DataFrame({cond: sub[[f"{cond}:{s}" for s in SCALES]].mean(axis=1) for cond in CONDITIONS})
    composite_complete = composite.dropna()
    composite_test = friedman_with_w([composite_complete[c].to_numpy() for c in CONDITIONS])
    return {
        "per_scale": per_scale,
        "cronbach_alpha": alphas,
        "composite_means": {c: float(composite[c].mean()) for c in CONDITIONS},
        "composite_friedman": composite_test,
    }


def ranking_statistics(ranks_df: pd.DataFrame, idx: pd.Index) -> dict:
    sub = ranks_df.loc[idx].dropna().astype(int)
    n = len(sub)
    summary = {
        "n": n,
        "mean_ranks": {c: float(sub[c].mean()) for c in CONDITIONS},
        "rank_counts": {c: [int((sub[c] == r).sum()) for r in (1, 2, 3, 4)] for c in CONDITIONS},
        "pct_ranked_first": {c: float((sub[c] == 1).mean()) for c in CONDITIONS},
        "pct_ranked_last": {c: float((sub[c] == 4).mean()) for c in CONDITIONS},
        "friedman": friedman_with_w([sub[c].to_numpy() for c in CONDITIONS]),
    }
    pairs = list(combinations(CONDITIONS, 2))
    raw_pvalues = []
    pairwise = {}
    for a, b in pairs:
        test = stats.wilcoxon(sub[a], sub[b])
        pairwise[f"{a}_vs_{b}"] = {"statistic": float(test.statistic), "p": float(test.pvalue)}
        raw_pvalues.append(float(test.pvalue))
    for (a, b), p_holm in zip(pairs, holm_correction(raw_pvalues)):
        pairwise[f"{a}_vs_{b}"]["p_holm"] = p_holm
    summary["pairwise_wilcoxon"] = pairwise
    return summary


# ---------------------------------------------------------------------------
# Figures (generated from the primary analysis set)
# ---------------------------------------------------------------------------

def _likert_palette() -> list[str]:
    """Diverging 7-colour palette consistent with the report's muted scheme."""
    negative = ps.COLOR_TRANSFORMER  # muted red
    positive = ps.COLOR_GROUND_TRUTH  # muted blue
    neutral = "#D8DEE6"
    neg = [mcolors.to_hex(c) for c in mcolors.LinearSegmentedColormap.from_list("n", [negative, neutral])(np.linspace(0.0, 0.78, 3))]
    pos = [mcolors.to_hex(c) for c in mcolors.LinearSegmentedColormap.from_list("p", [neutral, positive])(np.linspace(0.22, 1.0, 3))]
    return neg + [neutral] + pos


def plot_likert_diverging(likert_df: pd.DataFrame, idx: pd.Index, output_dir: Path) -> None:
    sub = likert_df.loc[idx]
    palette = _likert_palette()
    fig, axes = plt.subplots(N_SCALES, 1, figsize=(9.0, 10.2), sharex=True)
    fig.set_facecolor(ps.FIGURE_FACECOLOR)

    for ax, scale in zip(axes, SCALES):
        for c_i, cond in enumerate(CONDITIONS):
            vals = sub[f"{cond}:{scale}"].dropna().astype(int)
            n = len(vals)
            counts = np.array([(vals == score).sum() for score in range(1, 8)], dtype=float)
            pct = 100.0 * counts / n
            # Centre each bar on the neutral midpoint (half of score 4).
            left = -(pct[:3].sum() + pct[3] / 2.0)
            y = len(CONDITIONS) - 1 - c_i
            for score_i in range(7):
                ax.barh(
                    y,
                    pct[score_i],
                    left=left,
                    height=0.66,
                    color=palette[score_i],
                    edgecolor=ps.darker_edge(palette[score_i], 0.85),
                    linewidth=0.6,
                    zorder=3,
                )
                left += pct[score_i]
        ax.axvline(0.0, color=ps.ACCENT_LINE, linewidth=1.0, alpha=0.55, zorder=4)
        ax.set_yticks(range(len(CONDITIONS)))
        ax.set_yticklabels([CONDITION_LABELS[c] for c in reversed(CONDITIONS)], fontsize=10.5)
        ax.set_title(scale, fontsize=12.5, loc="left", pad=4)
        ax.set_xlim(-100, 100)
        ps.style_axes(ax, grid_axis="y")
        ax.grid(False, axis="y")
        ax.grid(True, axis="x", color=ps.GRID_COLOR, linewidth=0.8)
        ax.set_axisbelow(True)

    axes[-1].set_xlabel("Percentage of respondents")
    axes[-1].set_xticks([-100, -50, 0, 50, 100])
    axes[-1].set_xticklabels(["100%", "50%", "0%", "50%", "100%"])

    legend_labels = [
        "1 - Strongly disagree", "2", "3", "4 - Neutral", "5", "6", "7 - Strongly agree",
    ]
    handles = [ps.color_patch(palette[i], legend_labels[i]) for i in range(7)]
    fig.legend(
        handles=handles,
        loc="lower center",
        bbox_to_anchor=(0.5, -0.005),
        ncol=4,
        frameon=True,
        framealpha=0.97,
        facecolor=ps.AXES_FACECOLOR,
        edgecolor=ps.GRID_COLOR,
        fontsize=9.5,
    )
    fig.suptitle("Likert ratings by condition", fontsize=15, fontweight="600", y=0.995)
    fig.tight_layout(rect=(0.0, 0.06, 1.0, 0.98))
    _save(fig, output_dir, "01_likert_diverging")


def plot_rank_distribution(ranks_df: pd.DataFrame, idx: pd.Index, output_dir: Path) -> None:
    sub = ranks_df.loc[idx].dropna().astype(int)
    n = len(sub)
    palette = [_likert_palette()[i] for i in (6, 4, 2, 0)]  # blue (best) -> red (worst)
    rank_labels = ["Ranked 1st (best)", "Ranked 2nd", "Ranked 3rd", "Ranked 4th (worst)"]

    fig, ax = plt.subplots(figsize=(9.0, 4.6))
    fig.set_facecolor(ps.FIGURE_FACECOLOR)
    for c_i, cond in enumerate(CONDITIONS):
        y = len(CONDITIONS) - 1 - c_i
        left = 0.0
        for rank in (1, 2, 3, 4):
            count = int((sub[cond] == rank).sum())
            if count:
                ax.barh(
                    y,
                    count,
                    left=left,
                    height=0.62,
                    color=palette[rank - 1],
                    edgecolor=ps.darker_edge(palette[rank - 1], 0.85),
                    linewidth=0.7,
                    zorder=3,
                )
                if count >= 2:
                    ax.text(
                        left + count / 2.0,
                        y,
                        str(count),
                        ha="center",
                        va="center",
                        fontsize=10,
                        color=ps.TEXT_COLOR,
                        zorder=5,
                    )
            left += count
        mean_rank = sub[cond].mean()
        ax.text(
            n + 0.35,
            y,
            f"mean rank {mean_rank:.2f}",
            ha="left",
            va="center",
            fontsize=10.5,
            color=ps.MUTED_TEXT,
        )

    ax.set_yticks(range(len(CONDITIONS)))
    ax.set_yticklabels([CONDITION_LABELS[c] for c in reversed(CONDITIONS)], fontsize=11)
    ax.set_xlim(0, n + 5.2)
    ax.set_xticks(range(0, n + 1, 5))
    ax.set_xlabel(f"Number of respondents (n = {n})")
    ax.set_title("Forced ranking: best (1) to worst (4)", pad=10)
    ps.style_axes(ax, grid_axis="y")
    ax.grid(False, axis="y")
    ax.grid(True, axis="x", color=ps.GRID_COLOR, linewidth=0.8)
    ax.set_axisbelow(True)
    handles = [ps.color_patch(palette[i], rank_labels[i]) for i in range(4)]
    ps.place_legend_below(ax, handles, ncol=4, anchor_y=-0.22, fontsize=9.5)
    fig.tight_layout(rect=(0.0, 0.05, 1.0, 1.0))
    _save(fig, output_dir, "02_rank_distribution")


def _save(fig, output_dir: Path, stem: str) -> None:
    for ext in ("png", "pdf"):
        fig.savefig(
            output_dir / f"{stem}.{ext}",
            bbox_inches="tight",
            pad_inches=0.18,
            facecolor=ps.FIGURE_FACECOLOR,
        )
    plt.close(fig)
    print(f"[FIG] {output_dir / stem}.png/.pdf")


# ---------------------------------------------------------------------------
# Reporting helpers
# ---------------------------------------------------------------------------

def _fmt_iqr(d: dict) -> str:
    return f"{d['median']:.1f} ({d['q1']:.3g}--{d['q3']:.3g})"


def print_latex_rows(likert: dict, ranking: dict) -> None:
    print("\n=== LaTeX table rows (primary set) ===")
    for scale in SCALES:
        entry = likert["per_scale"][scale]
        cells = " & ".join(_fmt_iqr(entry["descriptives"][cond]) for cond in CONDITIONS)
        fr = entry["friedman"]
        print(
            f"{scale} & {cells} & {fr['chi2']:.2f} & {fr['p_holm']:.3f} & {fr['kendalls_w']:.2f} \\\\"
        )
    fr = ranking["friedman"]
    rank_cells = " & ".join(f"{ranking['mean_ranks'][cond]:.2f}" for cond in CONDITIONS)
    print(f"Mean rank & {rank_cells} & {fr['chi2']:.2f} & {fr['p']:.4f} & {fr['kendalls_w']:.2f} \\\\")


def print_comments(comments_df: pd.DataFrame) -> None:
    print("\n=== Free-text comments (stdout only, not written to artefacts) ===")
    for cond in CONDITIONS:
        non_empty = comments_df[cond].dropna()
        non_empty = non_empty[non_empty.astype(str).str.strip() != ""]
        print(f"-- {CONDITION_LABELS[cond]} --")
        for i, text in non_empty.items():
            print(f"  r{i:02d}: {text}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--xlsx", type=Path, default=DEFAULT_XLSX, help="Path to the Google Forms responses export")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    likert_df, ranks_df, comments_df, timestamps = load_responses(args.xlsx)
    flags = respondent_flags(likert_df, ranks_df, timestamps)
    sets = build_analysis_sets(flags)

    print("=== Exclusion flags ===")
    print(
        flags.assign(likert_sd=flags["likert_sd"].round(2), rank_rating_consistency=flags["rank_rating_consistency"].round(2)).to_string()
    )
    for name, idx in sets.items():
        excluded = sorted(set(flags.index) - set(idx))
        print(f"set '{name}': n={len(idx)} excluded_rows={excluded}")

    results = {
        "metadata": {
            "n_total": int(len(likert_df)),
            "conditions": CONDITION_LABELS,
            "scales": SCALES,
            "exclusion_criteria": {
                "primary": "straight-liners: SD = 0 across all 20 Likert items",
                "strict": "primary plus responses whose ranking is the exact inversion (4,3,2,1) of the majority ordering",
            },
            "analysis_sets": {
                name: {"n": int(len(idx)), "excluded_rows": sorted(set(map(int, flags.index)) - set(map(int, idx)))}
                for name, idx in sets.items()
            },
            "respondent_flags": {
                str(i): {
                    "straight_liner": bool(flags.loc[i, "straight_liner"]),
                    "inverted_ranking": bool(flags.loc[i, "inverted_ranking"]),
                    "burst_submission": bool(flags.loc[i, "burst_submission"]),
                    "rank_rating_consistency": None
                    if pd.isna(flags.loc[i, "rank_rating_consistency"])
                    else round(float(flags.loc[i, "rank_rating_consistency"]), 3),
                }
                for i in flags.index
            },
        },
        "analyses": {},
    }

    for name, idx in sets.items():
        results["analyses"][name] = {
            "likert": likert_statistics(likert_df, idx),
            "ranking": ranking_statistics(ranks_df, idx),
        }

    with open(args.output_dir / "statistical_tests.json", "w", encoding="utf-8") as fh:
        json.dump(results, fh, indent=2, default=float)
    print(f"\n[JSON] {args.output_dir / 'statistical_tests.json'}")

    primary = results["analyses"]["primary"]
    summary = {
        "primary_n": primary["ranking"]["n"],
        "mean_ranks": primary["ranking"]["mean_ranks"],
        "pct_ranked_first": primary["ranking"]["pct_ranked_first"],
        "ranking_friedman": primary["ranking"]["friedman"],
        "cronbach_alpha": primary["likert"]["cronbach_alpha"],
        "composite_means": primary["likert"]["composite_means"],
    }
    with open(args.output_dir / "survey_summary.json", "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, default=float)
    print(f"[JSON] {args.output_dir / 'survey_summary.json'}")

    ps.configure_style()
    plot_likert_diverging(likert_df, sets["primary"], args.output_dir)
    plot_rank_distribution(ranks_df, sets["primary"], args.output_dir)

    print("\n=== Headline numbers ===")
    for name in ("full", "primary", "strict"):
        r = results["analyses"][name]["ranking"]
        print(
            f"{name:8s} n={r['n']:2d} mean_ranks="
            + ", ".join(f"{CONDITION_LABELS[c]}: {r['mean_ranks'][c]:.2f}" for c in CONDITIONS)
        )
        fr = r["friedman"]
        print(f"          Friedman chi2({fr['df']})={fr['chi2']:.2f} p={fr['p']:.5f} W={fr['kendalls_w']:.2f}")
        print("          pairwise (Holm): " + ", ".join(
            f"{k}: p={v['p_holm']:.4f}" for k, v in r["pairwise_wilcoxon"].items()
        ))

    print_latex_rows(primary["likert"], primary["ranking"])
    print_comments(comments_df)


if __name__ == "__main__":
    main()
