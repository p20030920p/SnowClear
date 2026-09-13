#!/usr/bin/env python3
"""Fig. 15 - where the ground truth goes, on the hard scenes and on the reported set.

One row per scene, one bar per row, four segments. Each annotated point is charged to the first
stage that rejects it, so the segments are ordered rather than independent: a wider green segment is
the only thing that can raise recall.

    python3 tools/audit_error_budget.py --mode budget --csv <audit.csv> --scenes 35 11 14 16
    python3 tools/audit_error_budget.py --mode budget --csv <all.csv> --scenes <all scenes>
    python3 tools/gen_budget_fig.py --audit-csv <audit.csv> --all-csv <all.csv> [--lang zh]

The reported-set bar is computed from the all-scenes CSV by excluding 14, 16 and 76, so it is the
same 16 scenes / 1 620 frames the tables use. Nothing here is typed in by hand.
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import statistics

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

FG = "#1f2328"
MUTED = "#6e7781"
RULE = "#d0d7de"
BAND = "#f6f8fa"
SEG = [("reachable", "#1a7f37"), ("roi", "#8c959f"),
       ("ceiling", "#cf222e"), ("veto", "#c9820a")]

TXT = {
    "en": dict(title="Where the ground truth goes",
               sub="each annotated point charged to the first stage that rejects it · the released "
                   "configuration, no parameters changed",
               xlabel="% of annotated points",
               legend=["reached by the decision", "outside the ROI gate",
                       "above the intensity ceiling", "vetoed as surface-attached"],
               foot="the green share is the recall ceiling: the measured recall sits within 0.1 pp "
                    "of it on the reported set and on the 4-scene audit\n"
                    "regenerate: tools/audit_error_budget.py --mode budget --csv <file>, then this "
                    "script on the two CSVs"),
    "zh": dict(title="真值去了哪里",
               sub="每个标注点计入第一个拒绝它的阶段 · 发布配置，未改动任何参数",
               xlabel="占标注点的百分比",
               legend=["通过判定被检出", "落在 ROI 门控之外", "高于强度上限", "被判为贴附表面而否决"],
               foot="绿色段即召回上限：在报告集与 4 场景审计上，实测召回与它的差距都在 0.1 pp 以内\n"
                    "复现：tools/audit_error_budget.py --mode budget --csv <文件>，再用本脚本读这两个 CSV"),
}


def load(path: pathlib.Path):
    return list(csv.DictReader(path.open()))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--audit-csv", type=pathlib.Path, required=True)
    ap.add_argument("--all-csv", type=pathlib.Path, default=None)
    ap.add_argument("--out", type=pathlib.Path, default=None)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    a = ap.parse_args()
    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    out = a.out or pathlib.Path("docs/figures") / (
        "fig15_budget_zh.png" if a.lang == "zh" else "fig15_budget.png")

    bars = []
    if a.all_csv:
        rows = [r for r in load(a.all_csv) if r["scene"] not in ("14", "16", "76")]
        if rows:
            bars.append(("16-scene reported set", {k: statistics.fmean(float(r[k]) for r in rows)
                                                   for k, _ in SEG}))
    for row in load(a.audit_csv):
        bars.append((f"scene {row['scene']}", {k: float(row[k]) for k, _ in SEG}))

    fig, ax = plt.subplots(figsize=(9.4, 0.95 + 0.62 * len(bars)), dpi=200)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    y = np.arange(len(bars))
    left = np.zeros(len(bars))
    for key, colour in SEG:
        vals = np.array([b[1][key] for b in bars])
        ax.barh(y, vals, 0.6, left=left, color=colour, zorder=3, edgecolor="white",
                linewidth=0.8, label=t["legend"][[k for k, _ in SEG].index(key)])
        for yy, v, l in zip(y, vals, left):
            if v >= 5.0:
                ax.text(l + v / 2, yy, f"{v:.1f}", ha="center", va="center",
                        fontsize=9.4 if key == "reachable" else 8.4, color="white",
                        fontweight="bold" if key == "reachable" else "normal",
                        fontproperties=fps)
        left += vals
    ax.axhspan(-0.5, 0.5, color=BAND, zorder=0)
    ax.set_yticks(y)
    ax.set_yticklabels([b[0] for b in bars], fontsize=9.0, color=FG, fontproperties=fps)
    ax.invert_yaxis()
    ax.set_xlim(0, 100)
    ax.set_xticks([0, 25, 50, 75, 100])
    ax.set_xlabel(t["xlabel"], fontsize=8.6, color=MUTED, fontproperties=fps)
    ax.tick_params(axis="both", length=0, labelsize=8.2, colors=MUTED)
    ax.grid(axis="x", color="#eaeef2", linewidth=0.8, zorder=1)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(RULE)
    ax.legend(loc="lower left", bbox_to_anchor=(0.0, 1.005), frameon=False, fontsize=8.6,
              ncol=2, prop=fps, handlelength=1.1, handleheight=0.95, columnspacing=1.4)
    fig.text(0.5, 0.975, t["title"], ha="center", va="top", fontsize=11.6, color=FG,
             fontweight="bold", fontproperties=fps)
    fig.text(0.5, 0.930, t["sub"], ha="center", va="top", fontsize=8.2, color=MUTED,
             fontproperties=fps)
    fig.text(0.008, 0.014, t["foot"], ha="left", va="bottom", fontsize=7.2, color=MUTED,
             fontproperties=fps, linespacing=1.5)
    fig.subplots_adjust(left=0.145, right=0.985, top=0.775, bottom=0.155)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, facecolor="white")
    plt.close(fig)
    print(f"{out}")
    for name, vals in bars:
        print(f"  {name:24s} reachable {vals['reachable']:6.2f}  roi {vals['roi']:6.2f}  "
              f"ceiling {vals['ceiling']:6.2f}  veto {vals['veto']:5.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
