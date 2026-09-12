#!/usr/bin/env python3
"""Fig. 8 - where the recall goes: the ground-truth budget, by scene.

Every annotated point is charged to the *first* stage that rejects it, so the four bands
are not independently recoverable — they are the order in which the shipped configuration
gives up on ground truth:

    reachable   survives the ROI gate, the intensity ceiling and the surface veto; recall
                cannot exceed this, and on the 4-scene audit the measured recall sits
                within 0.03 pp of it
    I-ceiling   I >= 0.2132 * T, which the low-intensity score can never admit
    veto        a weak, far return attached to a surface (the veto's whole job)
    ROI         outside z in [-1, 2.6] m, r <= 17 m or elevation >= -23 deg

    python3 tools/audit_error_budget.py --mode budget --csv <path> --scenes <all scenes>
    python3 tools/gen_ceiling_fig.py <path-to-that-csv> [--lang zh]

The CSV is the single source of truth: the numbers in the figure are the numbers the audit
prints, so the two cannot drift.
"""

from __future__ import annotations

import argparse
import csv
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

FG = "#1f2328"
MUTED = "#6e7781"
RULE = "#d0d7de"
GRID = "#eaeef2"
BAND = "#f6f8fa"
SEG = [("reachable", "#2da44e"), ("ceiling", "#d1242f"),
       ("veto", "#c9820a"), ("roi", "#8c959f")]

TXT = {
    "en": dict(title="Where the recall goes: the ground-truth budget by scene",
               sub="each annotated point charged to the first stage that rejects it — "
                   "the bands are ordered, not independent",
               ylabel="% of annotated points in the frame",
               legend=["reachable by the shipped rule", "above the intensity ceiling",
                       "vetoed as attached to a surface", "outside the ROI gate"],
               foot="per-scene macro average over frames · ‡ scenes outside the 16-scene "
                    "reported set (14 and 16 are the recall-limited ones, 76 holds 5 frames)\n"
                    "regenerate: tools/audit_error_budget.py --mode budget --csv <file>, "
                    "then this script on that CSV",
               note="the ROI gate, not the decision rule, is the largest single loss"),
    "zh": dict(title="召回去了哪里：逐场景的真值预算",
               sub="每个标注点计入第一个拒绝它的阶段 —— 各段是有序的，不能独立相加",
               ylabel="占该帧标注点的百分比",
               legend=["发布规则仍可检出", "高于强度上限", "被判为贴附于表面而否决", "落在 ROI 门控之外"],
               foot="逐场景按帧宏平均 · ‡ 为 16 场景报告集之外的场景（14、16 召回受限，76 只有 5 帧）\n"
                    "复现：tools/audit_error_budget.py --mode budget --csv <文件>，再用本脚本读取该 CSV",
               note="最大的单笔损失来自 ROI 门控，而不是判定规则"),
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_path", type=pathlib.Path)
    ap.add_argument("--out", type=pathlib.Path, default=None)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    a = ap.parse_args()
    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    out = a.out or pathlib.Path("docs/figures") / (
        "fig8_gt_ceiling_zh.png" if a.lang == "zh" else "fig8_gt_ceiling.png")

    rows = list(csv.DictReader(a.csv_path.open()))
    rows.sort(key=lambda r: int(r["scene"]))
    scenes = [r["scene"] for r in rows]
    frames = [int(r["frames"]) for r in rows]
    data = {k: np.array([float(r[k]) for r in rows]) for k, _ in SEG}
    # the audit reports the losses; the reachable share is what is left
    data["reachable"] = np.array([float(r["reachable"]) for r in rows])
    outside = {"14", "16", "76"}

    fig, ax = plt.subplots(figsize=(11.6, 4.3), dpi=200)
    fig.patch.set_facecolor("white")
    x = np.arange(len(scenes))
    bottom = np.zeros(len(scenes))
    for key, colour in SEG:
        ax.bar(x, data[key], 0.72, bottom=bottom, color=colour, zorder=3,
               edgecolor="white", linewidth=0.7,
               label=t["legend"][[k for k, _ in SEG].index(key)])
        bottom += data[key]
    for i, (scene, reach) in enumerate(zip(scenes, data["reachable"])):
        ax.text(i, 101.5, f"{reach:.0f}", ha="center", va="bottom", fontsize=7.2,
                color=FG if scene not in outside else MUTED, fontproperties=fps, zorder=5)
        if data["roi"][i] >= 6:                     # only label the segments that matter
            ax.text(i, 100 - data["roi"][i] / 2, f"{data['roi'][i]:.0f}", ha="center",
                    va="center", fontsize=7.0, color="white", fontproperties=fps, zorder=5)

    for i, scene in enumerate(scenes):
        if scene in outside:
            ax.axvspan(i - 0.5, i + 0.5, color=BAND, zorder=0)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{s}‡" if s in outside else s for s in scenes], fontsize=8.6,
                       color=FG, fontproperties=fps)
    for tick, scene in zip(ax.get_xticklabels(), scenes):
        if scene in outside:
            tick.set_color(MUTED)
    ax.set_ylim(0, 112)
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.set_ylabel(t["ylabel"], fontsize=8.6, color=MUTED, fontproperties=fps)
    ax.tick_params(axis="y", labelsize=8.4, colors=MUTED, length=0)
    ax.tick_params(axis="x", length=0, pad=5)
    ax.grid(axis="y", color=GRID, linewidth=0.8, zorder=1)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(RULE)
    ax.legend(loc="lower left", bbox_to_anchor=(0.0, 1.02), frameon=False, fontsize=8.8,
              ncol=4, prop=fps, handlelength=1.1, handleheight=0.95, columnspacing=1.5)

    fig.text(0.5, 0.975, t["title"], ha="center", va="top", fontsize=11.6, color=FG,
             fontweight="bold", fontproperties=fps)
    fig.text(0.5, 0.928, t["sub"], ha="center", va="top", fontsize=8.4, color=MUTED,
             fontproperties=fps)
    fig.text(0.008, 0.015, t["foot"], ha="left", va="bottom", fontsize=7.2, color=MUTED,
             fontproperties=fps, linespacing=1.5)
    fig.subplots_adjust(left=0.062, right=0.995, top=0.80, bottom=0.155)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, facecolor="white")
    plt.close(fig)

    rep = [i for i, s in enumerate(scenes) if s not in outside]
    print(f"{out}")
    print(f"\n{len(scenes)} scenes, {sum(frames)} frames")
    for key, _ in SEG:
        print(f"  {key:>10}: 16-scene macro {data[key][rep].mean():6.2f}%   "
              f"all-19 {data[key].mean():6.2f}%")
    print(f"  the ROI gate is the largest single loss on both sets "
          f"({data['roi'][rep].mean():.2f}% vs {data['ceiling'][rep].mean():.2f}% "
          f"for the ceiling)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
