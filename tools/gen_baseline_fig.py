#!/usr/bin/env python3
"""Fig. 3 - SnowClear against the non-learned baselines on the reported set.

Reads the per-scene CSVs written by `tools/eval_baselines.sh` (one row per scene, the
columns `snowclear_runner` emits) and draws precision / recall / F1 as the macro average
over scenes - the aggregation README Table 1 uses - with the per-frame latency under each
method name.

    python3 tools/gen_baseline_fig.py --csv-dir experiments/baselines
    python3 tools/gen_baseline_fig.py --csv-dir experiments/baselines --lang zh

The script also prints the same numbers as a markdown table, so Table 2 in the READMEs is
filled from the measurement instead of being retyped by hand.
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

# Publication order; ours last, drawn with the emphasis the table gives it.
METHODS = [
    ("dror", "DROR", "Charron et al., CRV 2018"),
    ("dsor", "DSOR", "Kurup & Bos, 2021"),
    ("sor", "SOR", "Rusu et al., 2008"),
    ("ror", "ROR", "Rusu, 2009"),
    ("feature_fusion", "SnowClear (RITS)", "this work"),
]
METRICS = ["precision", "recall", "f1"]
SHADE = {"precision": "#2f6f9f", "recall": "#2da44e", "f1": "#8250df"}

FG = "#1f2328"
ACCENT = "#4a3b8f"
MUTED = "#6e7781"
RULE = "#d0d7de"
GRID = "#eaeef2"
BAND = "#f6f8fa"

TXT = {
    "en": dict(title="Reported set: 16 scenes / 1 620 frames of WADS",
               sub="macro average over scenes · identical ROI gate, index mapping and "
                   "evaluation path for every method",
               foot="Release build, OMP_NUM_THREADS=2 · latency from a single run on scene 35 "
                    "(101 frames), I/O and evaluation excluded — the protocol of Table 1\n"
                    "baselines: re-implemented in this repository, parameters at their "
                    "shipped defaults",
               legends=["Precision", "Recall", "F1"], unit="ms"),
    "zh": dict(title="报告集：WADS 16 个场景 / 1 620 帧",
               sub="按场景取宏平均 · 所有方法共用同一 ROI 门控、索引映射与评测路径",
               foot="Release 构建，OMP_NUM_THREADS=2 · 耗时在空闲状态下于场景 35（101 帧）"
                    "实测，不含 I/O 与评测\n基线：本仓库自实现，参数保持各自默认值",
               legends=["精确率", "召回率", "F1"], unit="ms"),
}


def load(csv_dir: pathlib.Path, method: str):
    path = csv_dir / f"{method}.csv"
    if not path.exists():
        return None
    rows = list(csv.DictReader(path.open()))
    if not rows:
        return None
    n = len(rows)
    out = {k: sum(float(r[k]) for r in rows) / n for k in
           ("precision", "recall", "f1", "time_ms")}
    out["scenes"] = n
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv-dir", type=pathlib.Path,
                    default=pathlib.Path("experiments/baselines"))
    ap.add_argument("--latency-csv-dir", type=pathlib.Path, default=None,
                    help="CSVs from a single-scene run, used for the ms/frame column")
    ap.add_argument("--out", type=pathlib.Path, default=None)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    a = ap.parse_args()
    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    out = a.out or pathlib.Path("docs/figures") / (
        "fig3_comparison_zh.png" if a.lang == "zh" else "fig3_comparison.png")

    data = []
    for key, name, ref in METHODS:
        got = load(a.csv_dir, key)
        if got is None:
            continue
        if a.latency_csv_dir:
            idle = load(a.latency_csv_dir, key)
            if idle is not None:
                got["time_ms"] = idle["time_ms"]
        data.append((key, name, ref, got))
    if not data:
        raise SystemExit(f"no CSVs in {a.csv_dir} - run tools/eval_baselines.sh first")

    x = np.arange(len(data))
    width = 0.26
    fig, ax = plt.subplots(figsize=(8.8, 4.15), dpi=200)
    fig.patch.set_facecolor("white")
    # our own group gets a band: when every bar is a percentage, the eye needs help
    # finding the row the table bolds
    ax.axvspan(len(data) - 1.5, len(data) - 0.5, color=BAND, zorder=0)

    for i, metric in enumerate(METRICS):
        vals = [d[3][metric] for d in data]
        bars = ax.bar(x + (i - 1) * width, vals, width * 0.92, color=SHADE[metric],
                      zorder=3, edgecolor="white", linewidth=0.6,
                      label=t["legends"][i])
        for bar, value in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 1.6, f"{value:.1f}",
                    ha="center", va="bottom", fontsize=7.4, color=FG,
                    fontproperties=fps, zorder=4)

    ax.set_xticks(x)
    ax.set_xticklabels([f"{d[1]}\n{d[3]['time_ms']:.0f} {t['unit']}" for d in data],
                       fontsize=9.6, color=FG, fontproperties=fps)
    for tick, d in zip(ax.get_xticklabels(), data):
        if d[0] == "feature_fusion":
            tick.set_fontweight("bold")
            tick.set_color(ACCENT)
    ax.set_ylim(0, 108)
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    ax.set_ylabel("%", fontsize=9, color=MUTED, fontproperties=fps)
    ax.tick_params(axis="y", labelsize=8.5, colors=MUTED, length=0)
    ax.tick_params(axis="x", length=0, pad=7)
    ax.grid(axis="y", color=GRID, linewidth=0.8, zorder=1)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(RULE)

    ax.legend(loc="upper left", bbox_to_anchor=(0.005, 0.995), frameon=False,
              fontsize=9, ncol=3, prop=fps, handlelength=1.1, handleheight=1.0,
              columnspacing=1.4)
    fig.text(0.5, 0.962, t["title"], ha="center", va="top", fontsize=11.4, color=FG,
             fontweight="bold", fontproperties=fps)
    fig.text(0.5, 0.916, t["sub"], ha="center", va="top", fontsize=8.2, color=MUTED,
             fontproperties=fps)
    fig.text(0.012, 0.022, t["foot"], ha="left", va="bottom", fontsize=7.4,
             color=MUTED, fontproperties=fps, linespacing=1.5)
    fig.subplots_adjust(left=0.058, right=0.995, top=0.845, bottom=0.235)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, facecolor="white")
    plt.close(fig)
    print(f"{out}")

    print("\n| Method | Precision | Recall | F1 | ms / frame |")
    print("|---|---:|---:|---:|---:|")
    for key, name, ref, d in data:
        bold = "**" if key == "feature_fusion" else ""
        print(f"| {bold}{name}{bold} | {bold}{d['precision']:.4f}{bold} | "
              f"{bold}{d['recall']:.4f}{bold} | {bold}{d['f1']:.4f}{bold} | "
              f"{bold}{d['time_ms']:.0f}{bold} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
