#!/usr/bin/env python3
"""Fig. 16 - precision / recall / F1 and frame time, one bar group per method.

Separate from the panel board on purpose: a bar chart belongs next to the table it summarises, not
inside a grid of point clouds.

    tools/eval_crfor.py --crfor-dir <repo> --scenes 35 --out <dir>     # CRFOR's indices and CSV
    SCENES=35 bash tools/eval_baselines.sh <dir>                       # SnowClear + the four filters
    python3 tools/gen_score_fig.py --csv-dir <dir> --crfor-csv <dir>/crfor.csv [--lang zh]

Reads the per-scene CSVs those two commands write, so the numbers here are the numbers in the
table. Latency comes from the idle single-scene run when one is given, otherwise from the same CSV.
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
GRID = "#eaeef2"
BAR = {"precision": "#2f6f9f", "recall": "#2da44e", "f1": "#8250df"}

TXT = {
    "en": dict(xlabel="", ylabel="%",
               legend=["precision", "recall", "F1"],
               foot="scene 35, {frames} frames · in-ROI · ms/frame under each name\n"
                    "regenerate: SCENES=35 tools/eval_baselines.sh, tools/eval_crfor.py, then this "
                    "script"),
    "zh": dict(xlabel="", ylabel="%",
               legend=["精确率", "召回率", "F1"],
               foot="场景 35，{frames} 帧 · ROI 内 · 方法名下方为 ms/帧\n"
                    "复现：SCENES=35 tools/eval_baselines.sh、tools/eval_crfor.py，再用本脚本"),
}


def macro(path: pathlib.Path):
    rows = list(csv.DictReader(path.open()))
    if not rows:
        return None
    mean = lambda k: statistics.fmean(float(r[k]) for r in rows)
    return dict(p=mean("precision"), r=mean("recall"), f1=mean("f1"), ms=mean("time_ms"),
                n=len(rows))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv-dir", type=pathlib.Path, required=True)
    ap.add_argument("--crfor-csv", type=pathlib.Path, default=None)
    ap.add_argument("--latency-dir", type=pathlib.Path, default=None)
    ap.add_argument("--out", type=pathlib.Path, default=None)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    a = ap.parse_args()
    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    out = a.out or pathlib.Path("docs/figures") / (
        "fig16_scores_zh.png" if a.lang == "zh" else "fig16_scores.png")

    series = [("SnowClear", macro(a.csv_dir / "feature_fusion.csv"))]
    if a.crfor_csv and a.crfor_csv.exists():
        rows = list(csv.DictReader(a.crfor_csv.open()))
        series.append(("CRFOR", dict(
            p=statistics.fmean(float(r["precision_roi"]) for r in rows),
            r=statistics.fmean(float(r["recall_roi"]) for r in rows),
            f1=statistics.fmean(float(r["f1_roi"]) for r in rows),
            ms=statistics.fmean(float(r["ms"]) for r in rows), n=len(rows))))
    for key, label in (("dror", "DROR"), ("dsor", "DSOR"), ("sor", "SOR"), ("ror", "ROR")):
        series.append((label, macro(a.csv_dir / f"{key}.csv")))
    series = [(n, m) for n, m in series if m]
    if a.latency_dir:                      # the idle run, when one exists
        for i, (name, m) in enumerate(series):
            key = {"SnowClear": "feature_fusion", "CRFOR": None}.get(name, name.lower())
            idle = macro(a.latency_dir / f"{key}.csv") if key else None
            if idle:
                m["ms"] = idle["ms"]
    frames = max(m["n"] for _, m in series)

    fig, ax = plt.subplots(figsize=(9.6, 4.0), dpi=200)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    x = np.arange(len(series))
    width = 0.26
    for offset, (key, colour) in zip((-width, 0.0, width),
                                     zip(("p", "r", "f1"), BAR.values())):
        vals = [m[key] for _, m in series]
        ax.bar(x + offset, vals, width, color=colour, zorder=3)
        for xx, v in zip(x, vals):
            ax.text(xx + offset, v + 1.4, f"{v:.1f}", ha="center", va="bottom", fontsize=7.2,
                    color=FG, fontproperties=fps)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{n}\n{m['ms']:.0f} ms" for n, m in series], fontsize=8.8, color=FG,
                       fontproperties=fps)
    ax.set_ylim(0, 116)
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    ax.set_ylabel(t["ylabel"], fontsize=8.6, color=MUTED, fontproperties=fps)
    ax.tick_params(axis="both", length=0, labelsize=8.4, colors=MUTED)
    ax.grid(axis="y", color=GRID, linewidth=0.8, zorder=1)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(RULE)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in BAR.values()]
    ax.legend(handles, t["legend"], loc="upper right", frameon=False, fontsize=9.0, ncol=3,
              prop=fps, handlelength=1.1, handleheight=0.95, columnspacing=1.4)
    fig.text(0.008, 0.015, t["foot"].format(frames=frames), ha="left", va="bottom",
             fontsize=7.2, color=MUTED, fontproperties=fps, linespacing=1.5)
    fig.subplots_adjust(left=0.062, right=0.995, top=0.975, bottom=0.185)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, facecolor="white")
    plt.close(fig)
    print(f"{out}  ({frames} frames)")
    for name, m in series:
        print(f"  {name:10s} P {m['p']:6.2f}  R {m['r']:6.2f}  F1 {m['f1']:6.2f}  {m['ms']:9.1f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
