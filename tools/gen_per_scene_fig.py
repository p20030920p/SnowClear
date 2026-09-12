#!/usr/bin/env python3
"""Fig. 1 - per-scene precision / recall / F1 over the 19 mirrored WADS scenes.

Reads the per-scene CSV written by `snowclear_runner --mode eval_folders` with the
released configuration (`detector_type:=feature_fusion`) and draws one precision /
recall / F1 group per scene, in the style of `tools/gen_baseline_fig.py`.

Why per scene, and why one unweighted mean on top:

* `eval_folders` writes **one CSV row per scene directory**; each metric in that row is
  already the macro average of the per-frame values inside the scene (DATASET.md sections 3
  and 4). The bars therefore carry the same numbers the per-scene table in DATASET.md
  quotes, and the horizontal reference lines are the unweighted mean over the 16 reported
  scenes - the aggregation README Table 1 uses. Scenes are *not* pooled by frame, because
  the three scenes outside the reported set hold 208 of the 1 828 frames and would
  otherwise be invisible.
* The 16-scene reported set (every scene except 14, 16 and 76) is marked with a shaded
  band. This is the point of the figure: scenes 14 and 16 are recall-limited, and averaging
  them away hides the one failure mode (`docs/METHOD.md` section 2) the rest of the
  documentation is about.

Produce the CSV first (one run, all 19 scenes, ~50 s at OMP_NUM_THREADS=2):

    export SNOWCLEAR_DATA=<WADS mirror root>          # layout: docs/DATASET.md
    OUT=~/.cache/snowclear_eval/per_scene             # any scratch dir outside the repo
    mkdir -p "$OUT/pcd_root"
    for s in $(ls "$SNOWCLEAR_DATA/pcd_output"); do
      ln -sfn "$SNOWCLEAR_DATA/pcd_output/$s" "$OUT/pcd_root/$s"
    done
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    OMP_NUM_THREADS=2 OMP_DYNAMIC=false ros2 run snowclear_core snowclear_runner \
      --mode eval_folders --params src/snowclear_ros/config/snowclear_params.yaml \
      pcd_root:="$OUT/pcd_root" result_root:="$SNOWCLEAR_DATA/result" \
      csv_path:="$OUT/per_scene.csv" detector_type:=feature_fusion

Then:

    python3 tools/gen_per_scene_fig.py ~/.cache/snowclear_eval/per_scene
    python3 tools/gen_per_scene_fig.py --csv-dir ~/.cache/snowclear_eval/per_scene --lang zh

The script also prints the per-scene numbers and both macro averages as markdown, and
flags any value that disagrees with the ones DATASET.md section 3 publishes.
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

METRICS = ["precision", "recall", "f1"]
SHADE = {"precision": "#2f6f9f", "recall": "#2da44e", "f1": "#8250df"}

FG = "#1f2328"
MUTED = "#6e7781"
RULE = "#d0d7de"
GRID = "#eaeef2"
BAND = "#f6f8fa"

# Scenes outside the 16-scene reported set. They are in the mirror, evaluated and drawn
# here, but README Table 4 reports them separately (DATASET.md section 3): 14 and 16 are
# recall-limited, 76 holds 5 frames.
EXCLUDE = ["14", "16", "76"]

# DATASET.md section 3 - quoted here only so the script can flag a drift instead of
# silently disagreeing with the document.
DOCUMENTED_PER_SCENE = {  # scene -> (P, R, F1)
    "14": (93.96, 53.47, 67.34),
    "16": (95.82, 44.13, 59.74),
    "76": (97.85, 98.84, 98.34),
}
DOCUMENTED_MACRO = {  # (P, R, F1)
    "reported": (96.6934, 89.9765, 92.8229),
    "all19": (96.5654, 85.4256, 90.0295),
}
# Frame counts per scene (DATASET.md section 3), used only by the drift check below: the
# all-19 recall DATASET.md prints is the *frame-weighted* mean of these per-scene recalls,
# while its P and F1 are the unweighted scene mean.
DOCUMENTED_FRAMES = {
    "11": 102, "12": 101, "13": 101, "14": 101, "15": 102, "16": 102, "17": 101,
    "18": 101, "20": 101, "22": 101, "23": 101, "24": 101, "26": 101, "28": 102,
    "30": 101, "34": 101, "35": 101, "36": 102, "76": 5,
}

TXT = {
    "en": dict(
        title="Per-scene precision / recall / F1 across the 19 mirrored scenes",
        sub="shaded band: the 16-scene reported set (all but 14, 16, 76) · dashed lines: "
            "its macro average — {macro}",
        foot="Released configuration ({cfg}), one eval_folders run per scene · 1 828 frames "
             "in total\n"
             "metrics are computed per frame and averaged within a scene, then across the "
             "reported scenes (DATASET.md §4)\n"
             "‡ outside the reported set: scenes 14 and 16 are recall-limited and scene "
             "76 holds 5 frames — reported separately in Table 4",
        legends=["Precision", "Recall", "F1"],
        ylabel="%",
    ),
    "zh": dict(
        title="19 个镜像场景的逐场景精确率 / 召回率 / F1",
        sub="浅色带：16 场景报告集（除 14、16、76）· 虚线：该集合的宏平均 — {macro}",
        foot="发布配置（{cfg}），每个场景一次 eval_folders 运行，共 1 828 帧\n"
             "指标先按帧计算、在场景内取宏平均，再对报告集取宏平均（DATASET.md §4）\n"
             "‡ 不在报告集内：场景 14、16 受召回上限限制，场景 76 仅 5 帧，"
             "三者按 Table 4 单独报告",
        legends=["精确率", "召回率", "F1"],
        ylabel="%",
    ),
}

# Accepted detector configurations, so a typo in the CSV cannot be mistaken for data.
RELEASED_DETECTOR = "feature_fusion"


def eval_command(csv_dir: pathlib.Path) -> str:
    """The exact command that produces the CSV this script reads."""
    return f"""\
export SNOWCLEAR_DATA=<WADS mirror root>          # layout: docs/DATASET.md
OUT={csv_dir}                                     # scratch dir outside the repository
mkdir -p "$OUT/pcd_root"
for s in $(ls "$SNOWCLEAR_DATA/pcd_output"); do
  ln -sfn "$SNOWCLEAR_DATA/pcd_output/$s" "$OUT/pcd_root/$s"
done
source /opt/ros/jazzy/setup.bash && source install/setup.bash
OMP_NUM_THREADS=2 OMP_DYNAMIC=false ros2 run snowclear_core snowclear_runner \\
  --mode eval_folders --params src/snowclear_ros/config/snowclear_params.yaml \\
  pcd_root:="$OUT/pcd_root" result_root:="$SNOWCLEAR_DATA/result" \\
  csv_path:="$OUT/per_scene.csv" detector_type:={RELEASED_DETECTOR}"""


def pick_csv(csv_dir: pathlib.Path) -> pathlib.Path:
    """`per_scene.csv` if it is there, otherwise the only usable CSV in the directory."""
    preferred = csv_dir / "per_scene.csv"
    if preferred.exists():
        return preferred
    candidates = []
    for path in sorted(csv_dir.glob("*.csv")):
        with path.open() as handle:
            cols = next(csv.reader(handle), [])
        if {"folder", *METRICS} <= set(cols):
            candidates.append(path)
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise SystemExit(f"no per-scene CSV under {csv_dir}\n\n{eval_command(csv_dir)}")
    listed = "\n".join(f"  {p}" for p in candidates)
    raise SystemExit(f"several CSVs match under {csv_dir}; keep one or pass `--csv`:\n{listed}")


def load(path: pathlib.Path):
    rows = list(csv.DictReader(path.open()))
    scenes = [(r["folder"], {k: float(r[k]) for k in METRICS}) for r in rows]
    scenes.sort(key=lambda s: (int(s[0]) if s[0].isdigit() else 1 << 30, s[0]))
    return scenes


def macro(rows) -> dict:
    n = len(rows)
    return {k: sum(r[1][k] for r in rows) / n for k in METRICS} if n else {}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_dir", nargs="?", type=pathlib.Path,
                    help="directory holding the per-scene CSV")
    ap.add_argument("--csv-dir", dest="csv_dir_opt", type=pathlib.Path,
                    help="same as the positional argument")
    ap.add_argument("--csv", type=pathlib.Path, help="explicit CSV path")
    ap.add_argument("--out", type=pathlib.Path, default=None)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    ap.add_argument("--exclude", default=",".join(EXCLUDE),
                    help="comma-separated scenes outside the reported set")
    a = ap.parse_args()

    csv_dir = a.csv_dir_opt or a.csv_dir
    if a.csv is None and csv_dir is None:
        ap.error("give the CSV directory (positional or --csv-dir)")
    path = a.csv or pick_csv(csv_dir)
    if not path.exists():
        raise SystemExit(f"no such CSV: {path}\n\n{eval_command(csv_dir or path.parent)}")

    exclude = {s.strip() for s in a.exclude.split(",") if s.strip()}
    scenes = load(path)
    if not scenes:
        raise SystemExit(f"{path} has no data rows\n\n{eval_command(csv_dir or path.parent)}")
    if len(scenes) != 19:
        print(f"[WARN] {path} holds {len(scenes)} scenes; DATASET.md §3 documents 19",
              file=sys.stderr)

    reported = [s for s in scenes if s[0] not in exclude]
    macro_rep, macro_all = macro(reported), macro(scenes)

    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
    out = a.out or pathlib.Path("docs/figures") / (
        "fig1_per_scene_zh.png" if a.lang == "zh" else "fig1_per_scene.png")

    x = np.arange(len(scenes))
    width = 0.26
    fig, ax = plt.subplots(figsize=(11.6, 4.55), dpi=200)
    fig.patch.set_facecolor("white")

    # A band per reported scene: the eye finds the three scenes the average ignores
    # without a second axis or a split panel.
    for i, (scene, _) in enumerate(scenes):
        if scene not in exclude:
            ax.axvspan(i - 0.5, i + 0.5, color=BAND, zorder=0)

    for i, metric in enumerate(METRICS):
        vals = [m[metric] for _, m in scenes]
        bars = ax.bar(x + (i - 1) * width, vals, width * 0.92, color=SHADE[metric],
                      zorder=3, edgecolor="white", linewidth=0.5,
                      label=t["legends"][i])
        for bar, value in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 1.6, f"{value:.1f}",
                    ha="center", va="bottom", fontsize=6.2, color=FG,
                    fontproperties=fps, zorder=4)

    # Reported-set macro average. Drawn under the bars, so it reads as a level, not a bar.
    for metric in METRICS:
        ax.axhline(macro_rep[metric], color=SHADE[metric], linewidth=0.9,
                   linestyle=(0, (4, 3)), alpha=0.85, zorder=2)

    ax.set_xticks(x)
    ax.set_xticklabels([f"{scene}‡" if scene in exclude else scene for scene, _ in scenes],
                       fontsize=9.0, color=FG, fontproperties=fps)
    for tick, (scene, _) in zip(ax.get_xticklabels(), scenes):
        if scene in exclude:
            tick.set_color(MUTED)
            tick.set_fontstyle("italic")
    ax.set_xlim(-0.7, len(scenes) - 0.3)
    ax.set_ylim(0, 108)
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    ax.set_ylabel(t["ylabel"], fontsize=9, color=MUTED, fontproperties=fps)
    ax.tick_params(axis="y", labelsize=8.5, colors=MUTED, length=0)
    ax.tick_params(axis="x", length=0, pad=6)
    ax.grid(axis="y", color=GRID, linewidth=0.8, zorder=1)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(RULE)

    # Above the axes: 19 groups of value labels leave no clean corner inside the frame.
    ax.legend(loc="lower left", bbox_to_anchor=(0.0, 1.005), frameon=False,
              fontsize=9, ncol=3, prop=fps, handlelength=1.1, handleheight=1.0,
              columnspacing=1.4)
    fig.text(0.5, 0.965, t["title"], ha="center", va="top", fontsize=11.4, color=FG,
             fontweight="bold", fontproperties=fps)
    fig.text(0.5, 0.923, t["sub"].format(macro=" · ".join(
        f"{k[0].upper()} {macro_rep[k]:.2f}" for k in METRICS)),
        ha="center", va="top", fontsize=8.2, color=MUTED, fontproperties=fps)
    fig.text(0.012, 0.020, t["foot"].format(cfg=f"detector_type:={RELEASED_DETECTOR}"),
             ha="left", va="bottom", fontsize=7.4, color=MUTED, fontproperties=fps,
             linespacing=1.5)
    fig.subplots_adjust(left=0.052, right=0.995, top=0.795, bottom=0.225)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, facecolor="white", transparent=False)
    plt.close(fig)
    # The figure conventions ask for no transparency; matplotlib still writes an alpha
    # channel, so flatten it (the background is already white).
    Image.open(out).convert("RGB").save(out)
    print(f"{out}")

    # Table + drift check: the numbers below are the ones a write-up should quote.
    print("\n| Scene | Precision | Recall | F1 | Reported set |")
    print("|---|---:|---:|---:|---|")
    for scene, m in scenes:
        mark = "no" if scene in exclude else "yes"
        print(f"| {scene} | {m['precision']:.2f} | {m['recall']:.2f} | {m['f1']:.2f} | {mark} |")
    print(f"| **macro, {len(reported)}-scene reported set** | **{macro_rep['precision']:.4f}** | "
          f"**{macro_rep['recall']:.4f}** | **{macro_rep['f1']:.4f}** | — |")
    print(f"| macro, all {len(scenes)} scenes | {macro_all['precision']:.4f} | "
          f"{macro_all['recall']:.4f} | {macro_all['f1']:.4f} | — |")

    drift = []
    for scene, m in scenes:
        if scene in DOCUMENTED_PER_SCENE:
            doc = DOCUMENTED_PER_SCENE[scene]
            for k, d in zip(METRICS, doc):
                if abs(m[k] - d) > 0.05:
                    drift.append(f"scene {scene} {k}: measured {m[k]:.2f} vs "
                                 f"DATASET.md {d:.2f}")
    for name, measured, doc in (("reported", macro_rep, DOCUMENTED_MACRO["reported"]),
                                ("all19", macro_all, DOCUMENTED_MACRO["all19"])):
        for k, d in zip(METRICS, doc):
            if abs(measured[k] - d) > 0.05:
                drift.append(f"{name} macro {k}: measured {measured[k]:.4f} vs "
                             f"DATASET.md {d:.4f}")
    if drift:
        print("\n[WARN] disagrees with DATASET.md §3 (reported, not reconciled):")
        for d in drift:
            print(f"  - {d}")
        # Weighting is the one benign explanation, and it is checkable: DATASET.md's
        # all-19 row mixes an unweighted P / F1 with a frame-weighted recall. Say so
        # explicitly rather than leaving a bare warning.
        frames = sum(DOCUMENTED_FRAMES.get(s, 0) for s, _ in scenes)
        if frames:
            weighted = {k: sum(m[k] * DOCUMENTED_FRAMES.get(s, 0) for s, m in scenes) / frames
                        for k in METRICS}
            notes = [f"{k}: frame-weighted {weighted[k]:.4f} vs DATASET.md {d:.4f}"
                     for k, d in zip(METRICS, DOCUMENTED_MACRO["all19"])
                     if abs(weighted[k] - d) <= 0.05
                     and abs(macro_all[k] - d) > 0.05]
            if notes:
                print("  the documented all-19 value matches the frame-weighted mean of "
                      "the same per-scene rows, not the unweighted scene mean:")
                for n in notes:
                    print(f"    - {n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
