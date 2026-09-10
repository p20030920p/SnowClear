#!/usr/bin/env python3
"""Render Algorithm 1 as a publication-style figure (PNG + SVG, EN + ZH).

Run from anywhere:  python3 tools/gen_algorithm_fig.py

A fenced code block is at the mercy of whatever monospace font the reader's
browser picks — and a CJK annotation in the same line breaks the column grid.
Rendering the algorithm as a figure removes that dependency: the PNG renders
identically everywhere, and the SVG stores text as paths so it cannot reflow.
"""

from __future__ import annotations

import pathlib
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import FancyBboxPatch, Rectangle

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/figures"

MONO = "DejaVu Sans Mono"
FS = 11.0                     # base font size, points
LH = 1.52                     # line height multiplier
COL = 56                      # code column width, in monospace cells
GUT = 5                       # gutter for the line number ("  12 ")
CMT = COL + 2                 # comment starts here (cells)
PAD = 14.0                    # inner padding, points

CJK_REG = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
CJK_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

FG = "#1f2328"       # code
CM = "#6e7781"       # comment
NUM = "#8c959f"      # line number
RULE = "#d0d7de"     # rules / frame
BAR = "#f6f8fa"      # title bar

# ----------------------------------------------------------------- content
# (number, code, comment). "**word**" marks a bold keyword.
BODY = [
    (1,  "P' ← ∅ ;  map ← ∅", "▷ ROI gate + processed→original index map"),
    (2,  "**for** each p_i ∈ P **do**", "▷ data-parallel, order-independent"),
    (3,  "    **if**  z_i ∈ [−1.0, 2.6]  ∧  x_i² + y_i² ≤ 17²", ""),
    (4,  "        ∧  asin(z_i / |p_i|) ≥ −23°  **then**", ""),
    (5,  "        append p_i to P' ;  map(|P'|) ← i", ""),
    (None, "", ""),
    (7,  "H  ← 256-bin intensity histogram of P'", "▷ per-thread hists, summed as ints"),
    (8,  "Q1 ← first bin with CDF(H) ≥ 0.25·|P'|", ""),
    (9,  "Tg ← clamp(0.8·Q1, 2.5, 8.0)", ""),
    (10, "build α-LUT over r ∈ [0, 40] m at 1 cm:", "▷ Γ(2.15) is constant per frame"),
    (11, "    α(r) ← ρ·f(r) / (ρ·f(r) + 1)", "▷ f(r) = Gamma pdf(k = 2.15, θ = 2.38)"),
    (12, "ground ← 10th pct of z per 1 m × 1 m cell", "▷ ≥ 5 points; else global min z"),
    (13, "G  ← spatial hash of { p ∈ P' : I > 1.0 }", "▷ cell size 0.6 m"),
    (None, "", ""),
    (15, "S ← ∅", ""),
    (16, "**for** each p_j = (x, y, z, I) ∈ P' **do**", "▷ parallel; results merged + sorted"),
    (17, "    r ← √(x² + y²) ;  h ← 1 − min(1, I/255)", ""),
    (18, "    T ← clamp( 0.8·Tg·(1 − α(r)·h), 2.0, 20.0 )", "▷ smooth range–intensity threshold"),
    (19, "    **if**  I ≥ 1.2·T  **then continue**", "▷ early stop"),
    (20, "    s ← ( I < T ) ? (1 − I/T)^1.2 : 0", "▷ intensity score"),
    (21, "    **if**  I ≤ 0.5·I_min ∧ r > 7 ∧ ∃ support", ""),
    (22, "        point within 0.6 m in G **then continue**", "▷ zero-intensity surface veto"),
    (23, "    **if**  s < 0.25  **then continue**", "▷ pre-filter"),
    (24, "    hag ← clamp((z − ground(cell(x,y))) / 1.5, 0, 1)", ""),
    (25, "    C ← 0.7·s + 0.15·hag", "▷ fused score, released configuration"),
    (26, "    θ ← 0.75 ;  **if** s < 0.4 **then** θ ← 0.90", ""),
    (27, "        **else if** s > 0.7 **then** θ ← 0.675", ""),
    (28, "    **if**  C > θ  ∧  s > 0.3  **then**  S ← S ∪ { map(j) }", ""),
    (None, "", ""),
    (30, "**return**  S ,  P \\ S", ""),
]

ZH_BODY = [
    (1,  "P' ← ∅ ;  map ← ∅", "▷ ROI 门控 + 处理后→原始索引映射"),
    (2,  "**for** each p_i ∈ P **do**", "▷ 数据并行，与顺序无关"),
    (3,  "    **if**  z_i ∈ [−1.0, 2.6]  ∧  x_i² + y_i² ≤ 17²", ""),
    (4,  "        ∧  asin(z_i / |p_i|) ≥ −23°  **then**", ""),
    (5,  "        append p_i to P' ;  map(|P'|) ← i", ""),
    (None, "", ""),
    (7,  "H  ← 256-bin intensity histogram of P'", "▷ 每线程局部直方图，整数求和"),
    (8,  "Q1 ← first bin with CDF(H) ≥ 0.25·|P'|", ""),
    (9,  "Tg ← clamp(0.8·Q1, 2.5, 8.0)", ""),
    (10, "build α-LUT over r ∈ [0, 40] m at 1 cm:", "▷ Γ(2.15) 是帧常量"),
    (11, "    α(r) ← ρ·f(r) / (ρ·f(r) + 1)", "▷ f(r) = Gamma 概率密度(k=2.15, θ=2.38)"),
    (12, "ground ← 10th pct of z per 1 m × 1 m cell", "▷ 格内 ≥ 5 点；否则全局最小 z"),
    (13, "G  ← spatial hash of { p ∈ P' : I > 1.0 }", "▷ 支撑点：I > 1.0，cell 0.6 m"),
    (None, "", ""),
    (15, "S ← ∅", ""),
    (16, "**for** each p_j = (x, y, z, I) ∈ P' **do**", "▷ 并行；结果合并后排序"),
    (17, "    r ← √(x² + y²) ;  h ← 1 − min(1, I/255)", ""),
    (18, "    T ← clamp( 0.8·Tg·(1 − α(r)·h), 2.0, 20.0 )", "▷ 平滑距离–强度阈值"),
    (19, "    **if**  I ≥ 1.2·T  **then continue**", "▷ 早停"),
    (20, "    s ← ( I < T ) ? (1 − I/T)^1.2 : 0", "▷ 强度得分"),
    (21, "    **if**  I ≤ 0.5·I_min ∧ r > 7 ∧ ∃ support", ""),
    (22, "        point within 0.6 m in G **then continue**", "▷ 零强度表面抑制"),
    (23, "    **if**  s < 0.25  **then continue**", "▷ 预筛"),
    (24, "    hag ← clamp((z − ground(cell(x,y))) / 1.5, 0, 1)", ""),
    (25, "    C ← 0.7·s + 0.15·hag", "▷ 融合得分（发布配置）"),
    (26, "    θ ← 0.75 ;  **if** s < 0.4 **then** θ ← 0.90", ""),
    (27, "        **else if** s > 0.7 **then** θ ← 0.675", ""),
    (28, "    **if**  C > θ  ∧  s > 0.3  **then**  S ← S ∪ { map(j) }", ""),
    (None, "", ""),
    (30, "**return**  S ,  P \\ S", ""),
]

HEADER_EN = {
    "title": "Algorithm 1   SnowClear: per-frame snow-point detection (released configuration)",
    "head": [
        ("Input",  "raw scan  P = { p_i = (x_i, y_i, z_i, I_i) },   i = 1..N"),
        ("",       "released parameters Θ  (score_threshold 0.75, idsor_scale 0.8, ρ 3.0,"),
        ("",       "                        k 2.15, θ 2.38, support 0.6 m / I > 1.0 / r > 7 m, …)"),
        ("Output", "snow index set S ⊆ {1..N} in the input index space; de-snowed cloud P \\ S"),
    ],
}

HEADER_ZH = {
    "title": "算法 1   SnowClear：单帧雪点检测（出厂配置）",
    "head": [
        ("输入", "原始扫描  P = { p_i = (x_i, y_i, z_i, I_i) },   i = 1..N"),
        ("",     "发布参数  Θ（score_threshold 0.75、idsor_scale 0.8、ρ 3.0、"),
        ("",     "          k 2.15、θ 2.38、支撑 0.6 m / I > 1.0 / r > 7 m，…）"),
        ("输出", "雪点索引集合 S ⊆ {1..N}（输入索引空间）；去雪点云 P \\ S"),
    ],
}


def strips(markup: str) -> str:
    return markup.replace("**", "")


def runs(markup: str):
    """Split '**bold** plain' into [(text, bold), ...]."""
    out, bold = [], False
    for part in re.split(r"(\*\*)", markup):
        if part == "**":
            bold = not bold
            continue
        if part:
            out.append((part, bold))
    return out


def build(lang: str, body, header, stem: str) -> None:
    mono = fm.FontProperties(family=MONO, size=FS)
    mono_b = fm.FontProperties(family=MONO, size=FS, weight="bold")
    num_fp = fm.FontProperties(family=MONO, size=FS - 1)
    cjk_fp = fm.FontProperties(fname=CJK_BOLD if lang == "zh" else CJK_REG, size=FS - 0.5)
    cjk_b = fm.FontProperties(fname=CJK_BOLD, size=FS - 0.5)
    title_fp = fm.FontProperties(fname=CJK_BOLD, size=FS + 0.5) if lang == "zh" \
        else fm.FontProperties(family=MONO, size=FS + 0.5, weight="bold")

    # ---- geometry -----------------------------------------------------
    n_rows = len(header["head"]) + 1 + len(body)          # head + rule + body
    fig_w_pt = 900.0
    height_pt = PAD * 2 + 30.0 + n_rows * FS * LH
    fig = plt.figure(figsize=(fig_w_pt / 72, height_pt / 72), dpi=200)
    renderer = fig.canvas.get_renderer()

    def width(s: str, fp) -> float:
        t = fig.text(0, 0, s, fontproperties=fp)
        w = t.get_window_extent(renderer=renderer).width * 72.0 / fig.dpi
        t.remove()
        return w

    chw = width("M" * 40, mono) / 40.0                    # monospace advance, points
    width("M" * 40, mono_b)                               # (bold advance checked below)
    bold_chw = width("M" * 40, mono_b) / 40.0
    if abs(bold_chw - chw) > 1e-6:
        print(f"  ! bold advance differs ({bold_chw:.4f} vs {chw:.4f}) — grid would shift",
              file=sys.stderr)

    code_x = PAD + GUT * chw
    cmt_x = PAD + CMT * chw
    cmt_fp = cjk_fp if lang == "zh" else mono
    head_fp = cjk_fp if lang == "zh" else mono
    head_x = PAD + 8 * chw          # header text starts here, clear of the label
    header_w = max((head_x + width(t, head_fp) for _, t in header["head"]), default=0.0)
    body_w = max((code_x + len(strips(c)) * chw for _, c, _ in body), default=0.0)
    cmt_w = max((cmt_x + width(c, cmt_fp) for _, _, c in body if c), default=0.0)
    right = PAD + max(body_w, cmt_w, header_w) + PAD
    fig.set_size_inches(right / 72.0, height_pt / 72.0)

    # ---- frame + title bar -------------------------------------------
    fig.patches.append(FancyBboxPatch(
        (0.5 / right, 0.5 / height_pt), 1 - 1 / right, 1 - 1 / height_pt,
        boxstyle="round,pad=0,rounding_size=6", transform=fig.transFigure,
        linewidth=1.0, edgecolor=RULE, facecolor="white", mutation_aspect=right / height_pt))
    bar_h = 30.0
    fig.patches.append(Rectangle(
        (0, 1 - bar_h / height_pt), 1, bar_h / height_pt, transform=fig.transFigure,
        facecolor=BAR, edgecolor="none"))
    fig.lines.append(plt.Line2D([0, 1], [1 - bar_h / height_pt] * 2, transform=fig.transFigure,
                                color=RULE, linewidth=1.0))
    fig.text(PAD / right, 1 - (bar_h / 2) / height_pt, header["title"],
             fontproperties=title_fp, color=FG, va="center", ha="left")

    # ---- rows ---------------------------------------------------------
    y = height_pt - bar_h - PAD
    for label, text in header["head"]:
        y -= FS * LH
        if label:
            fig.text(PAD / right, y / height_pt, label,
                     fontproperties=cjk_b if lang == "zh" else mono_b,
                     color=FG, va="baseline", ha="left")
        fig.text(head_x / right, y / height_pt, text,
                 fontproperties=head_fp, color=FG, va="baseline", ha="left")
    y -= FS * 0.55
    fig.lines.append(plt.Line2D([PAD / right, 1 - PAD / right],
                                [y / height_pt] * 2, transform=fig.transFigure,
                                color=RULE, linewidth=1.0))
    y -= FS * 0.45

    for n, code, comment in body:
        y -= FS * LH
        if n is not None:
            fig.text((PAD + (GUT - 1) * chw) / right, y / height_pt, str(n),
                     fontproperties=num_fp, color=NUM, va="baseline", ha="right")
        cell = 0
        for text, bold in runs(code):
            fig.text((code_x + cell * chw) / right, y / height_pt, text,
                     fontproperties=mono_b if bold else mono,
                     color=FG, va="baseline", ha="left")
            cell += len(text)
        if comment:
            fig.text(cmt_x / right, y / height_pt, comment,
                     fontproperties=cjk_fp if lang == "zh" else mono,
                     color=CM, va="baseline", ha="left")

    OUT.mkdir(parents=True, exist_ok=True)
    png = OUT / f"{stem}.png"
    fig.savefig(png, dpi=200, facecolor="white")
    fig.savefig(OUT / f"{stem}.svg", facecolor="white", format="svg")
    plt.close(fig)

    over = [(n, len(strips(c))) for n, c, _ in body if len(strips(c)) > COL]
    print(f"  {png.name:26s} {right:.0f}x{height_pt:.0f}pt -> "
          f"{int(right * 200 / 72)}x{int(height_pt * 200 / 72)}px   "
          f"code col {COL}, over-width rows: {over or 'none'}")


def main() -> int:
    print("rendering Algorithm 1:")
    build("en", BODY, HEADER_EN, "algorithm1_en")
    build("zh", ZH_BODY, HEADER_ZH, "algorithm1_zh")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
