#!/usr/bin/env python3
"""Fig. 9 - the released range-intensity threshold `T(r, I)` and the `alpha(r)` shape.

What the figure shows
---------------------
The three released formulas of `docs/METHOD.md` section 3, in one frame:

* **(a)** `T(r, I)` over the ROI for several intensities, at the clamp ceiling `Tg = 8.0`.
  The shape is *not* monotonic in range: it is pressed down where `alpha(r)` peaks and
  returns to the geometric baseline `0.8*Tg` far away - and the press disappears entirely
  for bright points, because `alpha` is weighted by `h(I) = 1 - min(1, I/255)`.
* **(b)** `alpha(r) = rho*f(r)/(rho*f(r) + 1)`, the Gamma-shaped range prior that does the
  pressing, against its complement `1 - alpha(r)` - the factor that is left of `0.8*Tg` when
  `I = 0`. The gap between the two curves is the whole range dependence of the threshold.
* **(c)** `Tg = clamp(0.8*Q1, 2.5, 8.0)`, the per-frame base the family is scaled by, with
  both clamps marked. It is the reason panel (a) also carries the `T = 2.0` floor line: for
  every `Tg <= 2.5`, `0.8*Tg <= 2.0` equals the lower clamp, so the entire family collapses
  onto `T = 2.0` - which is what the majority of the measured frames do.

Why this is drawn analytically
------------------------------
`T(r, I)`, `alpha(r)` and the `Tg` clamp are closed-form expressions in
`src/snowclear_core/include/snowclear/cloud_operations_types.hpp` and
`src/snowclear_core/src/parameter_optimizer.cpp`; drawing them from a dataset would make the
figure depend on data that is not redistributable (`docs/DATASET.md`) and would hide the
shape behind the sampling. The tool reads nothing, takes no arguments and prints every value
it drew, so `METHOD.md` section 3 can be checked against it.

    python3 tools/gen_threshold_fig.py             # -> docs/figures/fig9_threshold_curve.png
    python3 tools/gen_threshold_fig.py --lang zh   # -> docs/figures/fig9_threshold_curve_zh.png

Two details of the released code the figure keeps visible rather than hiding: the detector
reads `alpha` from a 1 cm LUT (`build_lut`, `use_threshold_lut`), so the shipped curve is a
step function of range that the continuous curve here samples; and `Q1` is taken over the
post-ROI cloud, which is why the floor clamp, not the scene, sets `Tg` on most frames.
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
from matplotlib.colors import LinearSegmentedColormap

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUTDIR = ROOT / "docs/figures"

CJK_REG = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
CJK_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

# House palette, identical to tools/gen_baseline_fig.py and tools/gen_algorithm_fig.py.
FG = "#1f2328"
ACCENT = "#4a3b8f"
GREY = "#57606a"
MUTED = "#6e7781"
RULE = "#d0d7de"
GRID = "#eaeef2"
BAND = "#f6f8fa"
# intensity ramp for panel (a): the same indigo -> blue -> pale family, darkest = I = 0
RAMP = LinearSegmentedColormap.from_list(
    "snowclear_intensity", ["#4a3b8f", "#2f6f9f", "#7fa8c9", "#ccd8e2"])

# ---- released constants -------------------------------------------------------------
# Sources: include/snowclear/cloud_operations_types.hpp (idsor_*, smooth_slope/r0,
# max_intensity, T clamp, kLutStep/kLutSize), include/snowclear/system_config.hpp
# (idsor_k/idsor_theta/idsor_rho/idsor_scale/idsor_slope/idsor_r0) and
# src/parameter_optimizer.cpp (adaptive_intensity_threshold -> the Tg clamp).
IDSOR_SCALE = 0.8          # idsor_scale, the "s" of T = s*Tg*(1 - alpha*h)
IDSOR_RHO = 3.0            # idsor_rho
IDSOR_K = 2.15             # idsor_k
IDSOR_THETA = 2.38         # idsor_theta
IDSOR_SLOPE = 0.0          # idsor_slope  - the range-increasing term is off in the release
IDSOR_R0 = 5.0             # idsor_r0     - only used by that term
MAX_INTENSITY = 255.0      # max_intensity
T_FLOOR, T_CEIL = 2.0, 20.0
TG_SCALE, TG_FLOOR, TG_CEIL = 0.8, 2.5, 8.0
Q1_FLOOR = TG_FLOOR / TG_SCALE      # 3.125 - below this Q1 the clamp floor is in charge
Q1_CEIL = TG_CEIL / TG_SCALE        # 10.0  - above this Q1 the clamp ceiling is
ROI_RADIUS = 17.0          # xy_threshold, m
# largest Tg observed over the 406 measured frames of the reported set (OPTIMIZATION.md
# section 4: median 2.50, max 7.20, 69.7 % of the frames on the clamp floor)
TG_OBSERVED_MAX = 7.2
LUT_STEP, LUT_SIZE = 0.01, 4001     # 0-40 m at 1 cm, as built once per frame
GAMMA_K = math.gamma(IDSOR_K)       # frame constant, hoisted out of the point loop

ALPHA_PEAK_R = (IDSOR_K - 1.0) * IDSOR_THETA   # mode of the Gamma pdf, 2.737 m
# Intensities drawn in panel (a): 0 is the population that matters (85.6 % of ground truth is
# I = 0), 1 is the brightest point the released build ever detects (METHOD.md section 2), and
# 255 is h = 0, where the range term vanishes and T is exactly 0.8*Tg.
INTENSITIES = [0.0, 1.0, 4.0, 16.0, 64.0, 255.0]


def h_of_intensity(intensity):
    """h(I) = 1 - min(1, I/255): 1 at I = 0, 0 from I = 255 up."""
    return 1.0 - np.minimum(1.0, np.asarray(intensity, dtype=float) / MAX_INTENSITY)


def alpha_of_range(r):
    """alpha(r) = rho*f(r) / (rho*f(r) + 1) with f the Gamma pdf(k, theta).

    The detector uses the 1 cm LUT built by RangeIntensityThreshold::build_lut, which stores
    alpha(i * 0.01) and truncates the lookup index, so the shipped alpha is a step function;
    this is the continuous function that LUT samples.
    """
    r = np.maximum(np.asarray(r, dtype=float), 0.0)
    pdf = np.where(r > 0.0,
                   (r / IDSOR_THETA) ** (IDSOR_K - 1.0) * np.exp(-r / IDSOR_THETA) /
                   (GAMMA_K * IDSOR_THETA),
                   0.0)
    return IDSOR_RHO * pdf / (IDSOR_RHO * pdf + 1.0)


def threshold(r, intensity, tg):
    """T(r, I) = clamp(0.8*Tg*(1 - alpha(r)*h(I)) + slope*max(0, r - r0), 2.0, 20.0)."""
    raw = (IDSOR_SCALE * tg * (1.0 - alpha_of_range(r) * h_of_intensity(intensity)) +
           IDSOR_SLOPE * np.maximum(0.0, np.asarray(r, dtype=float) - IDSOR_R0))
    return np.clip(raw, T_FLOOR, T_CEIL)


def tg_of_q1(q1):
    """Tg = clamp(0.8*Q1, 2.5, 8.0) - the only scene-adaptive number in the release."""
    return np.clip(TG_SCALE * np.asarray(q1, dtype=float), TG_FLOOR, TG_CEIL)


TXT = {
    "en": dict(
        title="The released threshold T(r, I) and the α(r) Gamma shape that sets it",
        sub="T = clamp(0.8·Tg·(1 − α(r)·h(I)), 2.0, 20.0),  "
            "Tg = clamp(0.8·Q1, 2.5, 8.0),  h(I) = 1 − min(1, I/255)",
        panel_a="(a)  T(r, I) at the clamp ceiling Tg = 8.0",
        panel_b="(b)  α(r): the Gamma-shaped range weight",
        panel_c="(c)  the per-frame base   Tg = clamp(0.8·Q1, 2.5, 8.0)",
        ylabel_a="threshold   T(r, I)   [intensity]",
        xlabel_a="horizontal range   r   [m]",
        floor_label="T = 2.0 clamp floor — and the whole family at Tg = 2.5\n"
                    "(69.7 % of the measured frames, OPTIMIZATION.md §4)",
        peak_label="α peak r = 2.74 m:\nthe I = 0 curve is pressed to 4.45",
        roi="ROI edge\n17 m",
        legend_max="I ≥ 255  (h = 0)",
        xlabel_b="horizontal range   r   [m]",
        ylabel_b="α(r)      and      1 − α(r)      (I = 0)",
        legend_alpha="α(r)",
        legend_rest="1 − α(r)   (I = 0)",
        gap_label="this gap is the entire range dependence of T",
        alpha_edge="α(17 m) = 0.009 at the ROI edge:\nthe shape is spent well inside it",
        xlabel_c="scene intensity first quartile   Q1",
        ylabel_c="base threshold   Tg",
        unclamped="0.8·Q1 before the clamp",
        floor_note="Q1 ≤ 3.125  ⇒  Tg = 2.50\nand T(r, I) ≡ 2.0",
        ceil_note="0.8·Q1 ≥ 8.0  (Q1 ≥ 10)  ⇒  Tg = 8.00",
        measured="measured: median 2.50, max 7.20 — 69.7 % of 406 frames on the floor",
        footnote=("analytic — T(r, I), α(r) and the Tg clamp are the released formulas "
                  "evaluated in closed form; no dataset, no ground truth, no arguments "
                  "(tools/gen_threshold_fig.py)\n"
                  "threshold   T(r, I) = 0.8·Tg·(1 − α(r)·h(I)) + slope·max(0, r − r0), "
                  "clamped to [2.0, 20.0];  slope = 0.0 and r0 = 5.0 in the release, so the "
                  "whole range dependence is α(r)\n"
                  "range prior   α(r) = ρ·f(r)/(ρ·f(r) + 1),  f = Gamma pdf(k = 2.15, "
                  "θ = 2.38),  ρ = 3.0   —   built once per frame into a 4 001-entry LUT over "
                  "0–40 m at 1 cm (use_threshold_lut, METHOD.md §3)\n"
                  "base   Tg = clamp(0.8·Q1, 2.5, 8.0), Q1 = intensity first quartile of the "
                  "post-ROI cloud:  Tg ≤ 2.5 ⇒ 0.8·Tg ≤ 2.0 = the clamp floor, so the whole "
                  "family flattens onto T = 2.0\n"
                  "annotated measurements: Tg median 2.50 / max 7.20, 69.7 % of 406 frames on "
                  "the floor, Q1 = 0 on 47.0 % (OPTIMIZATION.md §4);  max detected I = 1 "
                  "(METHOD.md §2)"),
    ),
    "zh": dict(
        title="发布阈值 T(r, I) 与决定其形状的 α(r) Gamma 曲线",
        sub="T = clamp(0.8·Tg·(1 − α(r)·h(I)), 2.0, 20.0)，  "
            "Tg = clamp(0.8·Q1, 2.5, 8.0)，  h(I) = 1 − min(1, I/255)",
        panel_a="(a)  钳制上界 Tg = 8.0 时的 T(r, I)",
        panel_b="(b)  α(r)：Gamma 形状的距离权重",
        panel_c="(c)  每帧基准   Tg = clamp(0.8·Q1, 2.5, 8.0)",
        ylabel_a="阈值   T(r, I)   [强度]",
        xlabel_a="水平距离   r   [m]",
        floor_label="T = 2.0 钳制下界 —— 也是 Tg = 2.5 时的整个曲线族\n"
                    "（占实测帧的 69.7 %，OPTIMIZATION.md §4）",
        peak_label="α 峰值 r = 2.74 m：\nI = 0 曲线被压到 4.45",
        roi="ROI 边界\n17 m",
        legend_max="I ≥ 255（h = 0）",
        xlabel_b="水平距离   r   [m]",
        ylabel_b="α(r)      与      1 − α(r)      (I = 0)",
        legend_alpha="α(r)",
        legend_rest="1 − α(r)   （I = 0）",
        gap_label="这段间距就是 T 的全部距离依赖",
        alpha_edge="ROI 边界处 α(17 m) = 0.009：\n该形状在 ROI 内就已基本衰减完",
        xlabel_c="场景强度第一四分位数   Q1",
        ylabel_c="基准阈值   Tg",
        unclamped="钳制前的 0.8·Q1",
        floor_note="Q1 ≤ 3.125  ⇒  Tg = 2.50\n且 T(r, I) ≡ 2.0",
        ceil_note="0.8·Q1 ≥ 8.0（Q1 ≥ 10）⇒  Tg = 8.00",
        measured="实测：中位数 2.50，最大 7.20 —— 406 帧中 69.7 % 落在下界",
        footnote=("解析绘制 —— T(r, I)、α(r) 与 Tg 钳制均为发布公式的闭式求值；"
                  "不读数据集、不用真值、不带参数（tools/gen_threshold_fig.py）\n"
                  "阈值   T(r, I) = 0.8·Tg·(1 − α(r)·h(I)) + slope·max(0, r − r0)，"
                  "钳制到 [2.0, 20.0]；发布配置中 slope = 0.0、r0 = 5.0，"
                  "因此全部距离依赖都来自 α(r)\n"
                  "距离先验   α(r) = ρ·f(r)/(ρ·f(r) + 1)，  f = Gamma 概率密度(k = 2.15, "
                  "θ = 2.38)，  ρ = 3.0   ——   每帧建一次 4 001 项查找表，覆盖 0–40 m、"
                  "步长 1 cm（use_threshold_lut，METHOD.md §3）\n"
                  "基准   Tg = clamp(0.8·Q1, 2.5, 8.0)，Q1 = ROI 后点云的强度第一四分位数："
                  "Tg ≤ 2.5 ⇒ 0.8·Tg ≤ 2.0 = 钳制下界，整个曲线族被拉到 T = 2.0\n"
                  "标注的实测值：Tg 中位数 2.50 / 最大 7.20，406 帧中 69.7 % 落在下界，"
                  "47.0 % 的帧 Q1 = 0（OPTIMIZATION.md §4）；最大检出 I = 1（METHOD.md §2）"),
    ),
}


def draw(lang: str, out: pathlib.Path) -> None:
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

    fig = plt.figure(figsize=(10.2, 6.7), dpi=200)
    fig.patch.set_facecolor("white")
    gs = fig.add_gridspec(2, 2, height_ratios=[1.18, 1.0],
                          left=0.072, right=0.986, top=0.845, bottom=0.168,
                          hspace=0.52, wspace=0.20)
    ax = fig.add_subplot(gs[0, :])
    bx = fig.add_subplot(gs[1, 0])
    cx = fig.add_subplot(gs[1, 1])

    # ---- (a) T(r, I) ---------------------------------------------------------------
    r = np.linspace(0.0, 17.6, 353)
    colours = [RAMP(v) for v in np.linspace(0.0, 1.0, len(INTENSITIES))]
    for i, (intensity, colour) in enumerate(zip(INTENSITIES, colours)):
        label = f"I = {intensity:.0f}" if intensity < MAX_INTENSITY else t["legend_max"]
        ax.plot(r, threshold(r, intensity, TG_CEIL), color=colour,
                linewidth=2.2 if i == 0 else 1.5, zorder=3, label=label,
                solid_capstyle="round")
    # Tg = 2.5 pins 0.8*Tg at the lower clamp, so this one line is the whole family there
    ax.axhline(T_FLOOR, color=GREY, linewidth=1.2, linestyle=(0, (5, 3)), zorder=2)
    ax.axvline(ROI_RADIUS, color=RULE, linewidth=1.0, zorder=1)
    ax.axvline(ALPHA_PEAK_R, color=RULE, linewidth=1.0, zorder=1)
    dip = float(threshold(ALPHA_PEAK_R, 0.0, TG_CEIL))
    ax.plot([ALPHA_PEAK_R], [dip], marker="o", markersize=4.4, color=ACCENT, zorder=5)
    ax.annotate(t["peak_label"], xy=(ALPHA_PEAK_R, dip), xytext=(5.3, 3.30),
                fontproperties=fp, fontsize=8.2, color=FG, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                shrinkA=0, shrinkB=3))
    ax.annotate(t["floor_label"], xy=(8.6, T_FLOOR), xytext=(5.3, 2.35),
                fontproperties=fp, fontsize=8.2, color=FG, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                shrinkA=0, shrinkB=3))
    ax.text(ROI_RADIUS - 0.25, T_CEIL + 0.02, t["roi"], fontproperties=fp, fontsize=8.0,
            color=MUTED, ha="right", va="bottom", linespacing=1.4)
    ax.set_xlim(0.0, 17.8)
    ax.set_ylim(1.72, 6.82)
    ax.set_xticks([0, 2.5, 5, 7.5, 10, 12.5, 15, 17.5])
    ax.set_yticks([2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5])
    ax.set_xlabel(t["xlabel_a"], fontproperties=fp, fontsize=8.8, color=MUTED, labelpad=6)
    ax.set_ylabel(t["ylabel_a"], fontproperties=fp, fontsize=8.8, color=MUTED, labelpad=6)
    # the family is only spread out between the α peak and the ROI edge, and it converges
    # again at both ends, so the labels go in the empty lower-left corner instead of on the lines
    ax.legend(loc="lower left", bbox_to_anchor=(0.004, 0.008), frameon=False, fontsize=8.2,
              prop=fp, ncol=2, handlelength=1.6, columnspacing=1.4, labelspacing=0.45)

    # ---- (b) alpha(r) --------------------------------------------------------------
    rb = np.linspace(0.0, 20.0, 401)
    alpha_b = alpha_of_range(rb)
    bx.fill_between(rb, alpha_b, 1.0, color=ACCENT, alpha=0.07, linewidth=0, zorder=1)
    bx.plot(rb, alpha_b, color=ACCENT, linewidth=2.2, zorder=4, label=t["legend_alpha"])
    bx.plot(rb, 1.0 - alpha_b, color=GREY, linewidth=1.4, linestyle=(0, (5, 3)), zorder=3,
            label=t["legend_rest"])
    bx.axvline(ROI_RADIUS, color=RULE, linewidth=1.0, zorder=1)
    bx.plot([ALPHA_PEAK_R], [float(alpha_of_range(ALPHA_PEAK_R))], marker="o", markersize=4.4,
            color=ACCENT, zorder=5)
    bx.annotate(f"α$_{{max}}$ = {float(alpha_of_range(ALPHA_PEAK_R)):.3f}  "
                f"at r = (k − 1)·θ = {ALPHA_PEAK_R:.2f} m",
                xy=(ALPHA_PEAK_R, float(alpha_of_range(ALPHA_PEAK_R))), xytext=(5.1, 0.455),
                fontproperties=fp, fontsize=8.2, color=FG, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                shrinkA=0, shrinkB=3))
    bx.annotate(t["gap_label"], xy=(6.6, 0.80), xytext=(6.9, 0.965),
                fontproperties=fp, fontsize=8.2, color=FG, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                shrinkA=0, shrinkB=3))
    bx.annotate(t["alpha_edge"], xy=(ROI_RADIUS, float(alpha_of_range(ROI_RADIUS))),
                xytext=(17.3, 0.60), fontproperties=fp, fontsize=8.0, color=MUTED, ha="right",
                va="center", arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                             shrinkA=0, shrinkB=3))
    bx.set_xlim(0.0, 20.0)
    bx.set_ylim(0.0, 1.06)
    bx.set_xticks([0, 5, 10, 15, 20])
    bx.set_yticks(np.arange(0.0, 1.01, 0.25))
    bx.set_xlabel(t["xlabel_b"], fontproperties=fp, fontsize=8.8, color=MUTED, labelpad=6)
    bx.set_ylabel(t["ylabel_b"], fontproperties=fp, fontsize=8.8, color=MUTED, labelpad=6)
    bx.legend(loc="lower right", bbox_to_anchor=(1.004, 0.010), frameon=False, fontsize=8.0,
              prop=fp, ncol=2, handlelength=1.6, columnspacing=1.6, labelspacing=0.4)

    # ---- (c) Tg = clamp(0.8*Q1, 2.5, 8.0) ------------------------------------------
    q1 = np.linspace(0.0, 14.0, 281)
    cx.axvspan(0.0, Q1_FLOOR, color=BAND, zorder=0)
    cx.axvspan(Q1_CEIL, 14.0, color=BAND, zorder=0)
    cx.plot(q1, TG_SCALE * q1, color=GREY, linewidth=1.3, linestyle=(0, (5, 3)), zorder=2,
            label=t["unclamped"])
    cx.plot(q1, tg_of_q1(q1), color=ACCENT, linewidth=2.2, zorder=3,
            label="Tg = clamp(0.8·Q1, 2.5, 8.0)")
    for xq in (Q1_FLOOR, Q1_CEIL):
        cx.axvline(xq, color=RULE, linewidth=1.0, zorder=1)
    cx.axhline(TG_OBSERVED_MAX, color=MUTED, linewidth=0.9, linestyle=(0, (1.6, 2.2)),
               zorder=1)
    cx.annotate(t["floor_note"], xy=(2.4, TG_FLOOR), xytext=(0.35, 3.55),
                fontproperties=fp, fontsize=8.2, color=FG, ha="left", va="center",
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                shrinkA=0, shrinkB=3))
    cx.annotate(t["ceil_note"], xy=(11.6, TG_CEIL), xytext=(13.7, 6.35),
                fontproperties=fp, fontsize=8.2, color=FG, ha="right", va="center",
                arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.8,
                                shrinkA=0, shrinkB=3))
    cx.text(0.3, 7.34, t["measured"], fontproperties=fp, fontsize=7.8, color=MUTED,
            ha="left", va="bottom")
    cx.set_xlim(0.0, 14.0)
    cx.set_ylim(0.0, 9.7)
    cx.set_xticks([0, 2, 4, 6, 8, 10, 12, 14])
    cx.set_yticks([0, 2, 4, 6, 8])
    cx.set_xlabel(t["xlabel_c"], fontproperties=fp, fontsize=8.8, color=MUTED, labelpad=6)
    cx.set_ylabel(t["ylabel_c"], fontproperties=fp, fontsize=8.8, color=MUTED, labelpad=6)
    cx.legend(loc="lower right", bbox_to_anchor=(1.004, 0.012), frameon=False, fontsize=8.0,
              prop=fp, handlelength=1.7, labelspacing=0.4)

    for a in (ax, bx, cx):
        a.grid(color=GRID, linewidth=0.8)
        a.set_axisbelow(True)
        for side in ("top", "right"):
            a.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            a.spines[side].set_color(RULE)
        a.tick_params(labelsize=8.4, colors=MUTED, length=0)

    for a, label in ((ax, t["panel_a"]), (bx, t["panel_b"]), (cx, t["panel_c"])):
        a.text(0.0, 1.028, label, transform=a.transAxes, fontproperties=fp_bold, fontsize=9.6,
               color=FG, ha="left", va="bottom")

    fig.text(0.5, 0.972, t["title"], ha="center", va="top", fontsize=11.8, color=FG,
             fontproperties=fp_bold)
    fig.text(0.5, 0.931, t["sub"], ha="center", va="top", fontsize=8.4, color=MUTED,
             fontproperties=fp)
    fig.text(0.012, 0.012, t["footnote"], ha="left", va="bottom", fontsize=6.6, color=MUTED,
             fontproperties=fp, linespacing=1.5)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, facecolor="white")
    plt.close(fig)
    flatten(out)

    print(f"  α(r)              max {float(alpha_of_range(ALPHA_PEAK_R)):.4f} at "
          f"r = (k − 1)·θ = {ALPHA_PEAK_R:.3f} m, "
          f"{float(alpha_of_range(ROI_RADIUS)):.4f} at the ROI edge, "
          f"{float(alpha_of_range(40.0)):.2e} at the LUT end")
    print(f"  T(r, I) at Tg=8   I = 0: {dip:.3f} at the α peak … "
          f"{float(threshold(ROI_RADIUS, 0.0, TG_CEIL)):.3f} at 17 m;  "
          f"I ≥ 255: {float(threshold(0.0, MAX_INTENSITY, TG_CEIL)):.2f} flat;  "
          f"range of the family {float(threshold(0.0, 0.0, TG_CEIL)):.3f}-"
          f"{IDSOR_SCALE * TG_CEIL:.2f}")
    print(f"  T(r, I) at Tg=2.5 every (r, I) → {float(threshold(ALPHA_PEAK_R, 0.0, TG_FLOOR)):.2f} "
          f"(0.8·Tg = 2.0 = the clamp floor)")
    print(f"  Tg clamp          Q1 = {Q1_FLOOR:.3f} → Tg = {TG_FLOOR}, "
          f"Q1 = {Q1_CEIL:.1f} → Tg = {TG_CEIL};  "
          f"measured median {tg_of_q1(0.0):.2f}, max {TG_OBSERVED_MAX} "
          f"(OPTIMIZATION.md §4)")
    print(f"  LUT               {LUT_SIZE} entries over 0-{LUT_STEP * (LUT_SIZE - 1):.0f} m at "
          f"{LUT_STEP * 100:.0f} cm, α sampled, index truncated (no interpolation)")
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
                    help="default: docs/figures/fig9_threshold_curve{,_zh}.png")
    a = ap.parse_args()
    stem = "fig9_threshold_curve_zh" if a.lang == "zh" else "fig9_threshold_curve"
    draw(a.lang, a.out or OUTDIR / f"{stem}.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
