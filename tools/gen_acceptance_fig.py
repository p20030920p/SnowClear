#!/usr/bin/env python3
"""Fig. 5 - acceptance region of the released decision function, drawn analytically.

What the figure shows
---------------------
Two views of the same gate, both for the released configuration of
`src/snowclear_core/src/snow_detector.cpp` + `.../parameter_optimizer.cpp`:

* **(a)** the `(I/T, h_ag)` plane. The fused score is `C = 0.7*S + 0.15*h_ag` with the
  intensity score `S = (1 - I/T)^1.2`; the branch threshold is `theta = 0.75`, `0.90` for
  `S < 0.4` and `0.675` for `S > 0.7`. The shaded region is what survives `C > theta`, and
  its right edge is the score gate `S > 0.75` that those two facts imply.
* **(b)** the intensity ceiling the gate imposes once `T` is substituted. `T` depends on the
  point's own intensity through `h(I) = 1 - min(1, I/255)`, so the ceiling is the fixed point
  of `I = (sup I/T) * T(r, I)` and not a single number: it is drawn against range for the
  clamp ceiling `Tg = 8.0`, for the largest `Tg` actually observed (`7.20`) and for the clamp
  floor `Tg = 2.5`, where the family collapses onto the `T = 2.0` floor.

Why this is drawn analytically
------------------------------
Every curve is the released formula evaluated in closed form: no dataset, no ground truth,
no arguments. The dataset is not redistributable (`docs/DATASET.md`), and a figure of the
decision function must stay valid when the numbers of the day change. The two measured facts
it annotates - the largest intensity ever detected (`I = 1`, `docs/METHOD.md` section 2) and
the `Tg` distribution (`docs/OPTIMIZATION.md` section 4) - are labels on top of the analytic
curves, and the tool prints the values it drew so the prose can be checked against them.

    python3 tools/gen_acceptance_fig.py             # -> docs/figures/fig5_acceptance.png
    python3 tools/gen_acceptance_fig.py --lang zh   # -> docs/figures/fig5_acceptance_zh.png

One nuance worth stating: `METHOD.md` section 2's `I < 1.36` is the *supremum* over `(r, I)` -
it needs `alpha(r)*h(I) -> 0`. Solving the fixed point gives the lower curve of panel (b):
`0.95` at the `alpha` peak for `Tg = 8.0`, `0.43` flat when the `Tg = 2.5` floor pins the
threshold at `T = 2.0`. The bound is right, but it is a bound, not the operating point.
"""

from __future__ import annotations

import argparse
import math
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUTDIR = ROOT / "docs/figures"

CJK_REG = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
CJK_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

# House palette, identical to tools/gen_baseline_fig.py and tools/gen_algorithm_fig.py.
FG = "#1f2328"        # text
ACCENT = "#4a3b8f"    # the released rule
BLUE = "#2f6f9f"      # secondary curve
GREEN = "#2da44e"     # measured marker
MUTED = "#6e7781"     # annotations, ticks
RULE = "#d0d7de"      # spines, leader lines
GRID = "#eaeef2"      # grid
BAND = "#f6f8fa"      # shaded bands / fills

# ---- released constants -------------------------------------------------------------
# Sources: include/snowclear/system_config.hpp (score_threshold, weights,
# detector_min_intensity_score), include/snowclear/cloud_operations_types.hpp
# (idsor_*, T clamp, max_intensity) and src/parameter_optimizer.cpp
# (adaptive_intensity_threshold -> Tg clamp).
SCORE_THRESHOLD = 0.75     # feature_weights.score_threshold
SCORE_EXPONENT = 1.2       # pow() exponent of the intensity score
THETA_LOW = 1.2            # enable_local_threshold_adjustment: *1.2 when S < 0.4
THETA_HIGH = 0.9           # ... and *0.9 when S > 0.7
W_INTENSITY = 0.35         # intensity_weight
W_HEIGHT = 0.15            # height_weight
HEIGHT_SCORE = 0.5         # height_score = height_above_ground * 0.5
S_BRANCH_LOW = 0.4
S_BRANCH_HIGH = 0.7
IDSOR_SCALE = 0.8          # idsor_scale
IDSOR_RHO = 3.0            # idsor_rho
IDSOR_K = 2.15             # idsor_k
IDSOR_THETA = 2.38         # idsor_theta
MAX_INTENSITY = 255.0      # max_intensity
T_FLOOR, T_CEIL = 2.0, 20.0
TG_SCALE, TG_FLOOR, TG_CEIL = 0.8, 2.5, 8.0
ROI_RADIUS = 17.0          # xy_threshold, m
# largest Tg observed over the 406 measured frames of the reported set (OPTIMIZATION.md
# section 4: median 2.50, max 7.20, 69.7 % of the frames on the clamp floor)
TG_OBSERVED_MAX = 7.2
GAMMA_K = math.gamma(IDSOR_K)   # frame constant, as in RangeIntensityThreshold::build_lut

# ---- analytic consequences ----------------------------------------------------------
# The two live terms are normalised by their own weight sum (0.35 + 0.15 = 0.50), so the
# fused score is 0.7*S + 0.15*h_ag. The height term cannot exceed 0.15, which is what makes
# the low-S branches of theta unreachable and turns the score gate into S > 0.75.
WEIGHT_SUM = W_INTENSITY + W_HEIGHT
W_SCORE = W_INTENSITY / WEIGHT_SUM                       # 0.7
W_HAG = W_HEIGHT * HEIGHT_SCORE / WEIGHT_SUM             # 0.15
S_MIN_ACCEPT = (SCORE_THRESHOLD * THETA_HIGH - W_HAG) / W_SCORE        # 0.75 exactly
X_MAX = 1.0 - S_MIN_ACCEPT ** (1.0 / SCORE_EXPONENT)     # I/T < 0.21316
X_GROUND = 1.0 - (SCORE_THRESHOLD * THETA_HIGH / W_SCORE) ** (1.0 / SCORE_EXPONENT)
X_BRANCH = 1.0 - S_BRANCH_HIGH ** (1.0 / SCORE_EXPONENT)  # S = 0.7 boundary, I/T = 0.2571


def h_of_intensity(intensity):
    """h(I) = 1 - min(1, I/255): 1 at I = 0, 0 from I = 255 up."""
    return 1.0 - np.minimum(1.0, np.asarray(intensity, dtype=float) / MAX_INTENSITY)


def alpha_of_range(r):
    """alpha(r) = rho*f(r) / (rho*f(r) + 1), f = Gamma pdf(k = 2.15, theta = 2.38).

    The detector evaluates this through a 1 cm LUT (build_lut), so the shipped alpha is a
    step function of r; the figure draws the continuous form the LUT samples, which differs
    by at most one step (the LUT argument is truncated, never interpolated).
    """
    r = np.maximum(np.asarray(r, dtype=float), 0.0)
    pdf = np.where(r > 0.0,
                   (r / IDSOR_THETA) ** (IDSOR_K - 1.0) * np.exp(-r / IDSOR_THETA) /
                   (GAMMA_K * IDSOR_THETA),
                   0.0)
    return IDSOR_RHO * pdf / (IDSOR_RHO * pdf + 1.0)


def threshold(r, intensity, tg):
    """T(r, I) = clamp(0.8*Tg*(1 - alpha(r)*h(I)), 2.0, 20.0), slope = 0 in the release."""
    raw = IDSOR_SCALE * tg * (1.0 - alpha_of_range(r) * h_of_intensity(intensity))
    return np.clip(raw, T_FLOOR, T_CEIL)


def score(x):
    """S = (1 - I/T)^1.2 for I/T = x < 1; snow_detector.cpp leaves S = 0 above that."""
    x = np.asarray(x, dtype=float)
    return np.where(x < 1.0, np.power(np.clip(1.0 - x, 0.0, None), SCORE_EXPONENT), 0.0)


def branch_theta(s):
    """The three-branch threshold: 0.75, *1.2 below S = 0.4, *0.9 above S = 0.7."""
    s = np.asarray(s, dtype=float)
    theta = np.full(s.shape, SCORE_THRESHOLD)
    theta = np.where(s < S_BRANCH_LOW, SCORE_THRESHOLD * THETA_LOW, theta)
    theta = np.where(s > S_BRANCH_HIGH, SCORE_THRESHOLD * THETA_HIGH, theta)
    return theta


def h_min(x):
    """Smallest h_ag that passes C > theta, on the only reachable branch (S > 0.7)."""
    return np.clip((SCORE_THRESHOLD * THETA_HIGH - W_SCORE * score(x)) / W_HAG, 0.0, 1.0)


def ceiling(r, tg, iters=60):
    """Largest intensity the score gate accepts at range r, T substituted self-consistently.

    I - X_MAX*T(r, I) is strictly increasing in I (T grows with I through h), so bisection
    converges; 60 halvings of [0, 20] is far past float resolution and there is no
    closed form once the T clamp is in the way.
    """
    r = np.atleast_1d(np.asarray(r, dtype=float))
    out = np.empty_like(r)
    for i, ri in enumerate(r):
        lo, hi = 0.0, T_CEIL
        for _ in range(iters):
            mid = 0.5 * (lo + hi)
            if mid < X_MAX * float(threshold(ri, mid, tg)):
                lo = mid
            else:
                hi = mid
        out[i] = lo
    return out


TXT = {
    "en": dict(
        title="Acceptance region of the released decision function",
        sub="the score gate S > 0.75 that the two-term fused score implies, "
            "and the intensity ceiling it puts on the threshold",
        region="accepted as snow", outside="rejected",
        gate="S > 0.75  ⇒  I/T < 0.2132",
        boundary="C = θ(S)",
        ground="h_ag = 0 (on the local ground):\ngate closes at I/T = 0.0299",
        example="I = 1 at T = 5.12 needs\nh_ag > 0.904 (1.36 m up)",
        branches="θ = 0.75 / 0.90 branches:\nmax C = 0.64 < θ",
        xlabel="normalised intensity   I / T",
        ylabel="normalised height above local ground   h$_{ag}$",
        blabel="intensity ceiling  max I accepted by the score gate",
        curve_hi="Tg = 8.0 (clamp ceiling)",
        curve_obs="Tg = 7.20 (largest observed)",
        curve_lo="Tg = 2.5 (clamp floor, 69.7 % of frames)",
        docbound="METHOD.md §2 bound  0.2132·0.8·Tg = 1.36",
        measured="largest I ever detected = 1",
        flat="0.8·Tg = 2.0 = the T floor:\nthe curve is flat at 0.426",
        xlabel_b="horizontal range   r   [m]",
        roi="ROI edge 17 m",
        foot=("analytic — every curve is the released formula evaluated in closed form; "
              "no dataset, no ground truth, no arguments (tools/gen_acceptance_fig.py)\n"
              "score   C = 0.7·S + 0.15·h_ag > θ(S),  S = (1 − I/T)$^{1.2}$,  "
              "θ = 0.75 / 0.90 (S < 0.4) / 0.675 (S > 0.7)\n"
              "              boundary on the only reachable branch (S > 0.7):   "
              "h_ag = (0.675 − 0.7·S)/0.15   —   the height term adds at most 0.15, so the "
              "S ≤ 0.7 branches can never reach their θ\n"
              "threshold   T = clamp(0.8·Tg·(1 − α(r)·h(I)), 2.0, 20.0),  "
              "Tg = clamp(0.8·Q1, 2.5, 8.0),  α(r) = ρ·f(r)/(ρ·f(r)+1),\n"
              "                     f = Gamma pdf(k = 2.15, θ = 2.38), ρ = 3.0, "
              "h(I) = 1 − min(1, I/255)   —   panel (b) substitutes T(r, I) into the gate\n"
              "annotated measurements: max detected I = 1 (METHOD.md §2); "
              "Tg median 2.50 / max 7.20, 69.7 % of 406 frames on the floor (OPTIMIZATION.md §4)"),
    ),
    "zh": dict(
        title="发布版判决函数的接受域",
        sub="两项融合得分所蕴含的得分门限 S > 0.75，以及它给阈值带来的强度上界",
        region="判为雪点", outside="判为非雪点",
        gate="S > 0.75  ⇒  I/T < 0.2132",
        boundary="C = θ(S)",
        ground="h_ag = 0（贴地）：\n门限在 I/T = 0.0299 处关闭",
        example="T = 5.12 时 I = 1 需要\nh_ag > 0.904（离地 1.36 m）",
        branches="θ = 0.75 / 0.90 分支：\nC 最大 0.64 < θ",
        xlabel="归一化强度   I / T",
        ylabel="归一化离地高度   h$_{ag}$",
        blabel="强度上界：得分门限能接受的最大 I",
        curve_hi="Tg = 8.0（钳制上界）",
        curve_obs="Tg = 7.20（实测最大）",
        curve_lo="Tg = 2.5（钳制下界，占 69.7 % 帧）",
        docbound="METHOD.md §2 上界  0.2132·0.8·Tg = 1.36",
        measured="实测检出的最大 I = 1",
        flat="0.8·Tg = 2.0 即 T 下界：\n曲线在 0.426 处拉平",
        xlabel_b="水平距离   r   [m]",
        roi="ROI 边界 17 m",
        foot=("解析绘制 —— 每条曲线都是发布公式的闭式求值；不读数据集、不用真值、不带参数"
              "（tools/gen_acceptance_fig.py）\n"
              "得分   C = 0.7·S + 0.15·h_ag > θ(S)，  S = (1 − I/T)$^{1.2}$，  "
              "θ = 0.75 / 0.90 (S < 0.4) / 0.675 (S > 0.7)\n"
              "          唯一可达分支 (S > 0.7) 的边界：   h_ag = (0.675 − 0.7·S)/0.15   ——   "
              "高度项最多加 0.15，故 S ≤ 0.7 的两个分支永远够不到各自的 θ\n"
              "阈值   T = clamp(0.8·Tg·(1 − α(r)·h(I)), 2.0, 20.0)，  "
              "Tg = clamp(0.8·Q1, 2.5, 8.0)，  α(r) = ρ·f(r)/(ρ·f(r)+1)，\n"
              "                f = Gamma 概率密度(k = 2.15, θ = 2.38)，ρ = 3.0，"
              "h(I) = 1 − min(1, I/255)   ——   面板 (b) 把 T(r, I) 代回门限\n"
              "标注的实测值：最大检出 I = 1（METHOD.md §2）；"
              "Tg 中位数 2.50 / 最大 7.20，406 帧中 69.7 % 落在下界（OPTIMIZATION.md §4）"),
    ),
}


def draw(lang: str, out: pathlib.Path) -> None:
    # self-check of the two facts the shaded region rests on: the boundary is drawn on the
    # S > 0.7 branch (theta = 0.675), and the other two branches cannot be reached by any
    # h_ag <= 1 because the height term contributes at most 0.15
    s_edge = float(score(np.array([X_MAX * 0.999]))[0])
    assert float(branch_theta(np.array([s_edge]))[0]) == SCORE_THRESHOLD * THETA_HIGH
    assert W_SCORE * S_BRANCH_HIGH + W_HAG < SCORE_THRESHOLD
    assert W_SCORE * S_BRANCH_LOW + W_HAG < SCORE_THRESHOLD * THETA_LOW
    t = TXT[lang]
    if lang == "zh":
        font_manager.fontManager.addfont(CJK_REG)
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
        fp = font_manager.FontProperties(fname=CJK_REG)
        fp_bold = font_manager.FontProperties(fname=CJK_BOLD)
    else:
        plt.rcParams["font.family"] = "DejaVu Sans"
        fp = font_manager.FontProperties(family="DejaVu Sans")
        fp_bold = font_manager.FontProperties(family="DejaVu Sans", weight="bold")

    fig = plt.figure(figsize=(9.8, 5.0), dpi=200)
    fig.patch.set_facecolor("white")
    gs = fig.add_gridspec(1, 2, width_ratios=[1.1, 1.0],
                          left=0.082, right=0.985, top=0.788, bottom=0.255, wspace=0.235)
    ax = fig.add_subplot(gs[0, 0])
    bx = fig.add_subplot(gs[0, 1])

    # ---- (a) the (I/T, h_ag) plane ------------------------------------------------
    xs = np.linspace(0.0, X_MAX, 400)
    ax.fill_between(xs, h_min(xs), 1.0, color=ACCENT, alpha=0.13, linewidth=0, zorder=1)
    ax.plot(xs, h_min(xs), color=ACCENT, linewidth=2.0, zorder=4, solid_capstyle="round")
    # the two branches that exist but cannot be reached: left of X_BRANCH the fused score
    # can still clear 0.675, right of it the maximum reachable C is 0.64
    ax.axvspan(X_BRANCH, 0.32, color=BAND, zorder=0)
    ax.axvline(X_MAX, color=MUTED, linewidth=1.0, linestyle=(0, (5, 3)), zorder=3)
    ax.axhline(0.0, color=RULE, linewidth=1.0, zorder=2)

    ax.plot([X_MAX], [1.0], marker="o", markersize=4.2, color=ACCENT, zorder=5)
    ax.annotate(t["gate"], xy=(X_MAX, 0.60), xytext=(0.219, 0.90), fontproperties=fp,
                fontsize=8.4, color=FG, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                shrinkA=0, shrinkB=2))
    # the boundary is too steep for a leader line: its label sits on the curve at the local
    # tangent angle with an opaque box, otherwise the line reads as part of the fill
    ax.text(0.150, 0.628, t["boundary"], rotation=44, rotation_mode="anchor",
            fontproperties=fp, fontsize=8.4, color=FG, ha="center", va="center", zorder=6,
            bbox=dict(boxstyle="round,pad=0.16", facecolor="white", edgecolor="none"))
    ax.plot([X_GROUND], [0.0], marker="o", markersize=4.2, markerfacecolor="white",
            markeredgecolor=ACCENT, markeredgewidth=1.2, zorder=5)
    # opaque box: the boundary curve runs through this corner of the wedge
    ax.annotate(t["ground"], xy=(X_GROUND, 0.0), xytext=(0.004, 0.062),
                fontproperties=fp, fontsize=8.0, color=FG, ha="left", va="bottom",
                zorder=6, bbox=dict(boxstyle="round,pad=0.18", facecolor="white",
                                    edgecolor="none"),
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                shrinkA=0, shrinkB=3))
    ex_x, ex_h = 1.0 / 5.12, float(h_min(1.0 / 5.12))
    ax.plot([ex_x], [ex_h], marker="o", markersize=4.2, color=GREEN, zorder=5)
    ax.annotate(t["example"], xy=(ex_x, ex_h), xytext=(0.135, 0.720),
                fontproperties=fp, fontsize=8.0, color=FG, ha="right", va="center",
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                shrinkA=2, shrinkB=3))
    ax.text(0.317, 0.30, t["branches"], fontproperties=fp, fontsize=8.0, color=MUTED,
            ha="right", va="center", linespacing=1.45)
    ax.text(0.052, 0.930, t["region"], fontproperties=fp_bold, fontsize=9.4, color=ACCENT,
            ha="center", va="center")
    ax.text(0.145, 0.330, t["outside"], fontproperties=fp, fontsize=8.6, color=MUTED,
            ha="center", va="center")

    ax.set_xlim(-0.004, 0.32)
    ax.set_ylim(-0.035, 1.045)
    ax.set_xticks(np.arange(0.0, 0.31, 0.05))
    ax.set_yticks(np.arange(0.0, 1.01, 0.25))
    ax.set_xlabel(t["xlabel"], fontproperties=fp, fontsize=8.8, color=MUTED, labelpad=6)
    ax.set_ylabel(t["ylabel"], fontproperties=fp, fontsize=8.8, color=MUTED, labelpad=6)

    # ---- (b) the intensity ceiling -------------------------------------------------
    r = np.linspace(0.0, ROI_RADIUS, 341)
    series = [(TG_CEIL, t["curve_hi"], ACCENT, 2.0, "-"),
              (TG_OBSERVED_MAX, t["curve_obs"], BLUE, 1.6, "-"),
              (TG_FLOOR, t["curve_lo"], MUTED, 1.4, (0, (5, 3)))]
    for tg, label, colour, lw, style in series:
        bx.plot(r, ceiling(r, tg), color=colour, linewidth=lw, linestyle=style,
                label=label, zorder=3, solid_capstyle="round")
    loose = X_MAX * IDSOR_SCALE * TG_CEIL
    bx.axhline(loose, color=RULE, linewidth=1.0, linestyle=(0, (2, 2)), zorder=2)
    bx.axhline(1.0, color=GREEN, linewidth=1.0, linestyle=(0, (1.6, 2.2)), zorder=2)
    bx.axvline(ROI_RADIUS, color=RULE, linewidth=1.0, zorder=1)
    bx.annotate(t["docbound"], xy=(3.6, loose), xytext=(0.4, loose + 0.075),
                fontproperties=fp, fontsize=8.0, color=MUTED, ha="left", va="bottom")
    bx.annotate(t["measured"], xy=(16.4, 1.0), xytext=(16.9, 1.045),
                fontproperties=fp, fontsize=8.0, color=GREEN, ha="right", va="bottom")
    bx.annotate(t["flat"], xy=(4.0, ceiling(8.0, TG_FLOOR)[0]), xytext=(4.9, 0.60),
                fontproperties=fp, fontsize=8.0, color=FG, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                shrinkA=0, shrinkB=3))
    dip_r = (IDSOR_K - 1.0) * IDSOR_THETA
    bx.plot([dip_r], [ceiling(dip_r, TG_CEIL)[0]], marker="o", markersize=4.0,
            color=ACCENT, zorder=5)
    bx.annotate(f"α peak r = {dip_r:.2f} m  ⇒  {ceiling(dip_r, TG_CEIL)[0]:.3f}",
                xy=(dip_r + 0.12, ceiling(dip_r, TG_CEIL)[0]), xytext=(6.2, 0.76),
                fontproperties=fp, fontsize=8.0, color=FG, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                shrinkA=0, shrinkB=3))
    bx.text(16.85, 0.62, t["roi"], fontproperties=fp, fontsize=8.0, color=MUTED,
            ha="right", va="center")

    bx.set_xlim(0.0, 17.6)
    bx.set_ylim(0.0, 1.52)
    bx.set_xticks([0, 2.5, 5, 7.5, 10, 12.5, 15, 17.5])
    bx.set_yticks(np.arange(0.0, 1.51, 0.25))
    bx.set_xlabel(t["xlabel_b"], fontproperties=fp, fontsize=8.8, color=MUTED, labelpad=6)
    bx.set_ylabel(t["blabel"], fontproperties=fp, fontsize=8.8, color=MUTED, labelpad=6)
    # the legend lives under the flat Tg = 2.5 curve: the top-right corner is where the
    # two rising curves (1.35 / 1.23) would run through the labels
    bx.legend(loc="lower right", bbox_to_anchor=(1.005, 0.012), frameon=False, fontsize=8.0,
              prop=fp, handlelength=1.6, labelspacing=0.45)

    for a in (ax, bx):
        a.grid(color=GRID, linewidth=0.8)
        a.set_axisbelow(True)
        for side in ("top", "right"):
            a.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            a.spines[side].set_color(RULE)
        a.tick_params(labelsize=8.4, colors=MUTED, length=0)

    ax.text(0.0, 1.035, "(a)", transform=ax.transAxes, fontproperties=fp_bold, fontsize=10.0,
            color=FG, ha="left", va="bottom")
    bx.text(0.0, 1.035, "(b)", transform=bx.transAxes, fontproperties=fp_bold, fontsize=10.0,
            color=FG, ha="left", va="bottom")

    fig.text(0.5, 0.968, t["title"], ha="center", va="top", fontsize=11.6, color=FG,
             fontproperties=fp_bold)
    fig.text(0.5, 0.921, t["sub"], ha="center", va="top", fontsize=8.4, color=MUTED,
             fontproperties=fp)
    fig.text(0.012, 0.014, t["foot"], ha="left", va="bottom", fontsize=6.6, color=MUTED,
             fontproperties=fp, linespacing=1.5)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, facecolor="white")
    plt.close(fig)
    flatten(out)

    print(f"  score gate        S > {S_MIN_ACCEPT:.4f}  ⇒  I/T < {X_MAX:.5f}   "
          f"(branch boundary S = 0.7 at I/T = {X_BRANCH:.5f})")
    print(f"  h_ag = 0          gate closes at I/T = {X_GROUND:.5f}")
    print(f"  loose bound       I < {X_MAX:.5f}·0.8·Tg = "
          f"{X_MAX * IDSOR_SCALE * TG_CEIL:.4f} at Tg = {TG_CEIL} (METHOD.md §2), "
          f"{X_MAX * IDSOR_SCALE * TG_OBSERVED_MAX:.4f} at the observed Tg = {TG_OBSERVED_MAX}, "
          f"{X_MAX * IDSOR_SCALE * TG_FLOOR:.4f} at Tg = {TG_FLOOR}")
    print(f"  self-consistent   min over r: {ceiling(r, TG_CEIL).min():.4f} at "
          f"r = {r[int(np.argmin(ceiling(r, TG_CEIL)))]:.2f} m, "
          f"{ceiling(ROI_RADIUS, TG_CEIL)[0]:.4f} at the ROI edge, "
          f"flat {ceiling(1.0, TG_FLOOR)[0]:.4f} at Tg = {TG_FLOOR}")
    print(f"  example           I = 1 at T = 5.12 ⇔ I/T = {ex_x:.5f}, "
          f"h_ag > {ex_h:.4f} ({ex_h * 1.5:.2f} m above local ground)")
    print(f"  written {out}  {int(fig.get_size_inches()[0] * 200)}"
          f"x{int(fig.get_size_inches()[1] * 200)} px")


def flatten(path: pathlib.Path) -> None:
    """Drop the (fully opaque) alpha channel: the README asks for PNGs without transparency."""
    try:
        from PIL import Image
    except ImportError:      # matplotlib writes RGBA with alpha = 255; still opaque
        return
    with Image.open(path) as im:
        im.convert("RGB").save(path)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    ap.add_argument("--out", type=pathlib.Path, default=None,
                    help="default: docs/figures/fig5_acceptance{,_zh}.png")
    a = ap.parse_args()
    stem = "fig5_acceptance_zh" if a.lang == "zh" else "fig5_acceptance"
    draw(a.lang, a.out or OUTDIR / f"{stem}.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
