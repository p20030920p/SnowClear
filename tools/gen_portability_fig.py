#!/usr/bin/env python3
"""Fig. 7 - what the label-free self-calibration is worth when the sensor moves.

Reads the four runs `tools/measure_portability.sh` writes - {released, self-calibrated} x
{reference mirror, +0.9 m mount} - and draws each pair against the released configuration on the
reference sensor, so the reader sees the loss and the recovery in the same picture.

The third row of README Table 6 is the CADC dataset, which this repository does not ship. It is
drawn as an empty slot with that stated, rather than left out or filled with a number nobody here
can regenerate.

    SNOWCLEAR_DATA=<mirror> bash tools/measure_portability.sh <outdir> <scenes...>
    python3 tools/gen_portability_fig.py <outdir> [--lang zh]

The perturbation is a rigid translation of the clouds (`z += -0.9 m`, the simulated effect of
mounting the sensor 0.9 m higher on the same vehicle), produced by tools/make_derived_frames.py;
ground-truth indices carry over unchanged because the point order does.
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
RELEASED = "#2f6f9f"
SELFCAL = "#2da44e"
MISSING = "#c3cad3"

TXT = {
    "en": dict(title="Released constants versus the label-free self-calibration",
               sub="the same {frames} frames, {scenes} scenes: the mount moves, the method does "
                   "not get re-parameterised",
               legend=["released constants", "self-calibrated switches on"],
               groups=["reference\nmount", "+0.9 m mounting\nheight",
                       "CADC (VLP-32C)\n8-bit intensity"],
               missing="needs the CADC dataset,\nwhich this repository\ndoes not ship",
               recall="recall {r:.1f} %",
               delta_lost="−{d:.1f} pp vs reference", delta_kept="{d:+.1f} pp vs reference",
               foot="perturbation: z += −0.9 m on every cloud (tools/make_derived_frames.py) — a rigid\n"
                    "translation with a flat ground: a simulation of a higher mount, not a recording from\n"
                    "one. It breaks the four absolute constants of METHOD.md §6 — the z bounds, the radial\n"
                    "bound, the elevation gate and the support intensity floor.",
               ylabel="macro F1 (%)"),
    "zh": dict(title="发布常量 vs 无标注自标定",
               sub="同一批 {frames} 帧、{scenes} 个场景：安装方式变了，方法本身没有被重新调参",
               legend=["发布常量", "开启自标定开关"],
               groups=["参考\n安装高度", "安装高度\n+0.9 m", "CADC（VLP-32C）\n8 位强度"],
               missing="需要 CADC 数据集，\n本仓库不分发", recall="召回 {r:.1f} %",
               delta_lost="较参考低 {d:.1f} pp", delta_kept="较参考 {d:+.1f} pp",
               foot="扰动：所有点云 z += −0.9 m（tools/make_derived_frames.py）—— 地面平坦假设下的刚体平移，\n"
                    "是对更高安装的仿真而非实拍。被破坏的是 METHOD.md 第 6 节里那四个绝对常量：\n"
                    "z 上下界、半径上界、仰角门与支撑强度下限。",
               ylabel="宏平均 F1（%）"),
}


def load(directory: pathlib.Path, dataset: str, config: str):
    path = directory / f"{dataset}_{config}.csv"
    if not path.exists():
        return None
    rows = list(csv.DictReader(path.open()))
    if not rows:
        return None
    mean = lambda k: statistics.fmean(float(r[k]) for r in rows)
    return {"p": mean("precision"), "r": mean("recall"), "f1": mean("f1"),
            "scenes": len(rows), "frames": 0}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_dir", type=pathlib.Path, nargs="?",
                    default=pathlib.Path("experiments/portability"))
    ap.add_argument("--out", type=pathlib.Path, default=None)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    a = ap.parse_args()
    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    out = a.out or pathlib.Path("docs/figures") / (
        "fig7_cross_sensor_zh.png" if a.lang == "zh" else "fig7_cross_sensor.png")

    ref_rel = load(a.csv_dir, "reference", "released")
    ref_self = load(a.csv_dir, "reference", "selfcal")
    sh_rel = load(a.csv_dir, "shifted", "released")
    sh_self = load(a.csv_dir, "shifted", "selfcal")
    if ref_rel is None:
        raise SystemExit(f"no reference_released.csv in {a.csv_dir}; run "
                         f"tools/measure_portability.sh first")

    groups = [("reference mount", ref_rel, ref_self),
              ("+0.9 m mounting height", sh_rel, sh_self),
              ("CADC (VLP-32C), 8-bit intensity", None, None)]
    print(f"{'dataset':>26} {'config':>9} {'precision':>10} {'recall':>8} {'F1':>8}")
    for name, rel, self_ in groups:
        for label, d in (("released", rel), ("selfcal", self_)):
            if d is None:
                print(f"{name:>26} {label:>9}  (not measured here)")
            else:
                print(f"{name:>26} {label:>9} {d['p']:>10.2f} {d['r']:>8.2f} {d['f1']:>8.2f}")

    x = np.arange(len(groups))
    width = 0.34
    fig, ax = plt.subplots(figsize=(9.2, 4.4), dpi=200)
    fig.patch.set_facecolor("white")
    for i, (name, rel, self_) in enumerate(groups):
        for offset, d, colour in ((-width / 2, rel, RELEASED), (width / 2, self_, SELFCAL)):
            if d is None:
                continue                   # no bar to draw: the row is a gap, not a value
            ax.bar(i + offset, d["f1"], width, color=colour, zorder=3,
                   edgecolor="white", linewidth=0.7)
            ax.text(i + offset, d["f1"] + 1.2, f"{d['f1']:.1f}", ha="center", va="bottom",
                    fontsize=8.6, color=FG, fontproperties=fps)
            ax.text(i + offset, d["f1"] - 4.0, t["recall"].format(r=d["r"]), ha="center",
                    va="top", fontsize=7.6, color="white", fontproperties=fps, zorder=4)
        if rel is None:
            ax.text(i, ref_rel["f1"] * 0.52, t["missing"], ha="center", va="center",
                    fontsize=8.0, color=MUTED, fontproperties=fps, zorder=5)
            continue
        # the loss and the recovery, stated on the picture rather than in the prose
        if rel["f1"] < ref_rel["f1"] - 1.0:
            ax.annotate(t["delta_lost"].format(d=ref_rel["f1"] - rel["f1"]),
                        xy=(i - width / 2, rel["f1"] + 1.4), xytext=(i - width / 2, 30),
                        ha="center", fontsize=8.0, color=FG, fontproperties=fps,
                        arrowprops=dict(arrowstyle="->", color=MUTED, linewidth=0.9))
        if self_ is not None and abs(self_["f1"] - ref_rel["f1"]) < 2.0:
            ax.text(i + width / 2, self_["f1"] + 4.5,
                    t["delta_kept"].format(d=self_["f1"] - ref_rel["f1"]), ha="center",
                    va="bottom", fontsize=8.0, color=SELFCAL, fontproperties=fps)

    ax.axhline(ref_rel["f1"], color=RELEASED, linewidth=0.9, linestyle=(0, (5, 3)), zorder=2)
    ax.set_xlim(-0.58, 2.62)      # room for the labels of the last group
    ax.set_xticks(x)
    ax.set_xticklabels([t["groups"][i] for i in range(len(groups))], fontsize=8.8, color=FG,
                       fontproperties=fps)
    ax.set_ylim(0, 100)
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    ax.set_ylabel(t["ylabel"], fontsize=8.6, color=MUTED, fontproperties=fps)
    ax.tick_params(axis="y", labelsize=8.4, colors=MUTED, length=0)
    ax.tick_params(axis="x", length=0, pad=6)
    ax.grid(axis="y", color=GRID, linewidth=0.8, zorder=1)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(RULE)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in (RELEASED, SELFCAL)]
    ax.legend(handles, t["legend"], loc="lower left", bbox_to_anchor=(0.0, 1.01),
              frameon=False, fontsize=8.8, prop=fps, ncol=2, handlelength=1.1,
              handleheight=0.95, columnspacing=1.6)
    fig.text(0.5, 0.972, t["title"], ha="center", va="top", fontsize=11.4, color=FG,
             fontweight="bold", fontproperties=fps)
    fig.text(0.5, 0.928, t["sub"].format(frames=ref_rel["frames"] or "1 620",
                                         scenes=ref_rel["scenes"]), ha="center", va="top",
             fontsize=8.2, color=MUTED, fontproperties=fps)
    fig.text(0.008, 0.015, t["foot"], ha="left", va="bottom", fontsize=7.2, color=MUTED,
             fontproperties=fps, linespacing=1.5)
    fig.subplots_adjust(left=0.072, right=0.965, top=0.785, bottom=0.245)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, facecolor="white")
    plt.close(fig)
    print(f"\n{out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
