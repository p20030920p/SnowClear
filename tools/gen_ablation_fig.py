#!/usr/bin/env python3
"""Fig. 4 - module-wise ablation on the 4-scene subset: macro-F1 delta per switch.

Reads the CSVs written by `snowclear_runner --mode eval_folders` for the released
configuration and for one single-switch run per ablated module, all on the same four
scenes (35, 11, 14, 16 - the subset `docs/OPTIMIZATION.md` uses throughout), and draws
each switch's macro-F1 delta relative to the released configuration.

Why this aggregation:

* Macro F1, not pooled F1: `eval_folders` writes one row per scene, each row already a
  macro average of the per-frame metrics (DATASET.md section 4), and the ablation table in
  `docs/OPTIMIZATION.md` section 6 is the unweighted mean over those rows. Averaging the
  per-scene F1 keeps this figure directly comparable with the documented table.
* Delta, not absolute F1: the four runs differ in one switch only, so the difference is
  the module's contribution; the absolute value of every run is printed at the bar tip and
  in the markdown table the script emits, so nothing is hidden by the differencing.
* The **ROI + `I = 0` trivial baseline** (`docs/METHOD.md` section 2) is the released rule
  with the surface veto removed: on this subset the veto-off run *is* that rule, because
  `I = 0` points pass the score test unconditionally. Its measured macro F1 is therefore
  drawn as the reference line labelled "ROI + I = 0 only". `METHOD.md` quotes 90.12 for the
  same rule on the 16-scene set; the line here is the 4-scene measurement, since mixing the
  two sets would compare different populations.

Produce the CSVs first (six short runs, sequential; released plus one flip per switch):

    export SNOWCLEAR_DATA=<WADS mirror root>          # layout: docs/DATASET.md
    OUT=~/.cache/snowclear_eval/ablation              # scratch dir outside the repository
    mkdir -p "$OUT/pcd_root"
    for s in 35 11 14 16; do
      ln -sfn "$SNOWCLEAR_DATA/pcd_output/$s" "$OUT/pcd_root/$s"
    done
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    run() {  # <csv stem> [key:=value ...]
      OMP_NUM_THREADS=2 OMP_DYNAMIC=false ros2 run snowclear_core snowclear_runner \\
        --mode eval_folders --params src/snowclear_ros/config/snowclear_params.yaml \\
        pcd_root:="$OUT/pcd_root" result_root:="$SNOWCLEAR_DATA/result" \\
        csv_path:="$OUT/$1.csv" detector_type:=feature_fusion "${@:2}"
    }
    run released
    run enable_zero_intensity_surface_suppression_off \\
        enable_zero_intensity_surface_suppression:=false
    run enable_planarity_calculation_on enable_planarity_calculation:=true
    run enable_density_calculation_on enable_density_calculation:=true
    run enable_feature_entropy_on enable_feature_entropy:=true
    run enable_grid_search_optimization_on enable_grid_search_optimization:=true

Then:

    python3 tools/gen_ablation_fig.py ~/.cache/snowclear_eval/ablation
    python3 tools/gen_ablation_fig.py --csv-dir ~/.cache/snowclear_eval/ablation --lang zh

The file name of each run encodes the setting away from the release: `<switch>_on.csv`
means the run had that switch `true`, `<switch>_off.csv` means `false`. Any switch from
the registry below whose CSV is present is drawn, so extending the measurement needs no
edit here.

The script prints the measured P / R / F1 / ms-per-frame per run next to the values
`docs/OPTIMIZATION.md` section 6 documents, and flags any disagreement instead of hiding it.
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

FG = "#1f2328"
ACCENT = "#4a3b8f"
MUTED = "#6e7781"
RULE = "#d0d7de"
GRID = "#eaeef2"
BAND = "#f6f8fa"
GOOD = "#2da44e"   # a switch that earns its place
BAD = "#cf222e"    # a switch that costs F1

SCENES = ["35", "11", "14", "16"]   # the subset OPTIMIZATION.md uses (406 frames)

# Registry of ablated switches: parameter name -> (English label, Chinese label).
# The released value comes from src/snowclear_ros/config/snowclear_params.yaml and
# src/snowclear_core/include/snowclear/ablation_switches.hpp.
SWITCHES = [
    ("enable_zero_intensity_surface_suppression",
     "zero-intensity surface suppression",
     "零强度表面抑制"),
    ("enable_planarity_calculation",
     "planarity term",
     "平面度项"),
    ("enable_density_calculation",
     "density term",
     "密度项"),
    ("enable_feature_entropy",
     "feature entropy",
     "特征熵"),
    ("enable_grid_search_optimization",
     "grid-search optimiser",
     "网格搜索优化器"),
]
RELEASED = {  # switch -> released value
    "enable_zero_intensity_surface_suppression": True,
    "enable_planarity_calculation": False,
    "enable_density_calculation": False,
    "enable_feature_entropy": False,
    "enable_grid_search_optimization": False,
}
# The five switches the task/figure promise; a missing one is an error, not a silent row.
REQUIRED = [k for k, _, _ in SWITCHES]
TRIVIAL_SWITCH = "enable_zero_intensity_surface_suppression"

# docs/OPTIMIZATION.md section 6 (4-scene macro average). Quoted so a drift is reported.
# The last field is the documented ms/frame, which is load-dependent and therefore never
# flagged: only the deterministic P / R / F1 columns take part in the drift check.
DOCUMENTED = {  # switch -> (P, R, F1, dF1, ms/frame)
    "released": (95.47, 68.60, 77.56, 0.0, 8.1),
    "enable_zero_intensity_surface_suppression": (87.64, 69.01, 74.94, -2.62, 5.1),
    "enable_planarity_calculation": (94.93, 51.77, 65.17, -12.39, 22.7),
    "enable_density_calculation": (96.26, 41.02, 56.38, -21.18, 21.1),
    "enable_feature_entropy": (94.99, 68.61, 77.34, -0.22, 21.1),
    "enable_grid_search_optimization": (None, None, None, 0.0, None),  # Table 3: no-op
}
TOL = 0.05   # pp; the detector is deterministic, so F1 should reproduce exactly

TXT = {
    "en": dict(
        title="Single-switch ablation: macro-F1 delta per module",
        sub="scenes {scenes} ({frames} frames), one switch flipped away from the released "
            "configuration per run",
        foot="Released macro F1 {rel} on this subset · bars are Δ F1 in points; the label "
             "quotes the run's own macro F1 · ms/frame is the run's mean per-frame stage "
             "budget and moves with machine load\n"
             "“ROI + I = 0 only” is the measured released rule with the surface veto "
             "removed — the trivial baseline `METHOD.md` §2 documents at 90.12 on the "
             "16-scene set (same rule, different population)",
        legend_released="Released configuration",
        legend_trivial="ROI + I = 0 only (trivial baseline)",
        released_txt="released",
        unit="pp",
        xlabel="Δ macro F1 (pp)",
    ),
    "zh": dict(
        title="单开关消融：各模块的宏平均 F1 变化量",
        sub="场景 {scenes}（{frames} 帧），每次运行仅相对发布配置翻转一个开关",
        foot="该子集上的发布配置宏平均 F1 = {rel} · 条形为 Δ F1（百分点），标注为该次运行"
             "自身的宏平均 F1 · ms/帧 为该次运行的逐帧阶段耗时均值，随机器负载波动\n"
             "“仅 ROI + I = 0”为去掉表面抑制后的实测发布规则，即 `METHOD.md` §2 在 16 场景集上"
             "记录的 90.12 平凡基线（规则相同、总体不同）",
        legend_released="发布配置",
        legend_trivial="仅 ROI + I = 0（平凡基线）",
        released_txt="发布配置",
        unit="pp",
        xlabel="Δ 宏平均 F1（百分点）",
    ),
}

FRAMES = 406   # DATASET.md section 3: 35/101 + 11/102 + 14/101 + 16/102


def run_kind(key: str) -> tuple[str, bool, str]:
    """(csv stem, value the run sets, the override) for a switch, given its released value.

    The file name says what the run *is* (`<switch>_on.csv` = the switch was true); the
    override is what has to be passed to `snowclear_runner`, which is the negation of the
    released value because the ablation flips exactly one switch.
    """
    value = not RELEASED[key]
    return f"{key}_{'on' if value else 'off'}", value, f"{key}:={'true' if value else 'false'}"


def eval_command(missing: list[tuple[str, str]], csv_dir: pathlib.Path) -> str:
    """The exact commands that produce the CSVs this script is missing."""
    calls = "\n".join(f"run {stem} {override}".rstrip() for stem, override in missing)
    return f"""\
export SNOWCLEAR_DATA=<WADS mirror root>          # layout: docs/DATASET.md
OUT={csv_dir}                                     # scratch dir outside the repository
mkdir -p "$OUT/pcd_root"
for s in {' '.join(SCENES)}; do
  ln -sfn "$SNOWCLEAR_DATA/pcd_output/$s" "$OUT/pcd_root/$s"
done
source /opt/ros/jazzy/setup.bash && source install/setup.bash
run() {{  # <csv stem> [key:=value ...]
  OMP_NUM_THREADS=2 OMP_DYNAMIC=false ros2 run snowclear_core snowclear_runner \\
    --mode eval_folders --params src/snowclear_ros/config/snowclear_params.yaml \\
    pcd_root:="$OUT/pcd_root" result_root:="$SNOWCLEAR_DATA/result" \\
    csv_path:="$OUT/$1.csv" detector_type:=feature_fusion "${{@:2}}"
}}
{calls}"""


def load(path: pathlib.Path, key: str) -> dict:
    rows = list(csv.DictReader(path.open()))
    if not rows:
        raise SystemExit(f"{path} has no data rows")
    n = len(rows)
    out = {k: sum(float(r[k]) for r in rows) / n for k in (*METRICS, "time_ms")}
    out["key"] = key
    out["scenes"] = n
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_dir", nargs="?", type=pathlib.Path,
                    help="directory holding released.csv and one CSV per switch run")
    ap.add_argument("--csv-dir", dest="csv_dir_opt", type=pathlib.Path,
                    help="same as the positional argument")
    ap.add_argument("--out", type=pathlib.Path, default=None)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    a = ap.parse_args()

    csv_dir = a.csv_dir_opt or a.csv_dir
    if csv_dir is None:
        ap.error("give the CSV directory (positional or --csv-dir)")
    if not csv_dir.is_dir():
        raise SystemExit(f"no such directory: {csv_dir}\n\n"
                         f"{eval_command([('released', '')], csv_dir)}")

    # Which switch runs are on disk (and in which direction), and which are owed.
    wanted = {"released": ("released", None, "")}
    for key in REQUIRED:
        stem, value, override = run_kind(key)
        wanted[key] = (stem, value, override)
    missing = [(stem, override) for stem, _, override in wanted.values()
               if not (csv_dir / f"{stem}.csv").exists()]
    extra = []
    for key, _, _ in SWITCHES:                     # documented switch, measured but not required
        if key in REQUIRED:
            continue
        stem, value, _ = run_kind(key)
        if (csv_dir / f"{stem}.csv").exists():
            extra.append((key, stem, value))
    if missing:
        raise SystemExit("missing measurements: "
                         + ", ".join(f"{stem}.csv" for stem, _ in missing)
                         + f"\n\n{eval_command(missing, csv_dir)}")

    released = load(csv_dir / "released.csv", "released")
    runs = []
    for key, stem, value in ([(k, *wanted[k][:2]) for k in REQUIRED] + extra):
        m = load(csv_dir / f"{stem}.csv", key)
        m["value"] = value
        m["stem"] = stem
        m["df1"] = m["f1"] - released["f1"]
        runs.append(m)
    runs.sort(key=lambda r: r["df1"])              # most harmful first, i.e. on top

    trivial = next(r for r in runs if r["key"] == TRIVIAL_SWITCH)
    if trivial["scenes"] != released["scenes"]:
        print(f"[WARN] the trivial-baseline run holds {trivial['scenes']} scenes and the "
              f"released run {released['scenes']}", file=sys.stderr)

    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
    out = a.out or pathlib.Path("docs/figures") / (
        "fig4_ablation_zh.png" if a.lang == "zh" else "fig4_ablation.png")

    name = {k: (z if a.lang == "zh" else e) for k, e, z in SWITCHES}
    en = {k: e for k, e, _ in SWITCHES}   # the markdown table stays English, as in Fig. 3
    # The run's setting relative to the release, in the figure's language.
    flip = {"en": {True: "on", False: "off"}, "zh": {True: "开", False: "关"}}[a.lang]

    fig, ax = plt.subplots(figsize=(11.4, 4.3), dpi=200)
    fig.patch.set_facecolor("white")
    y = np.arange(len(runs))[::-1]                 # first switch on top

    # The negative half is the story, so it gets the only band in the figure.
    ax.axvspan(min(r["df1"] for r in runs) - 1.5, 0, color=BAND, zorder=0)
    ax.barh(y, [r["df1"] for r in runs], 0.52,
            color=[GOOD if r["df1"] > TOL else BAD if r["df1"] < -TOL else MUTED
                   for r in runs], zorder=3)

    for yy, r in zip(y, runs):
        # A no-op has a zero-length bar; a tick keeps the row from reading as missing data.
        if abs(r["df1"]) <= TOL:
            ax.plot([0.0], [yy], marker="|", color=MUTED, markersize=9,
                    markeredgewidth=1.8, zorder=4)
        # The absolute F1 of the run, not just the delta: differencing must not hide it.
        # A long bar carries its label inside (white), a short one beside its tip, so no
        # label can grow into the y tick labels.
        value = f"{r['df1']:+.2f} {t['unit']}  ({r['f1']:.2f})"
        if r["df1"] < -6.0:
            ax.text(r["df1"] + 0.35, yy, value, ha="left", va="center", fontsize=8.4,
                    color="white", fontproperties=fps, zorder=4)
        else:
            ax.text(r["df1"] - 0.35, yy, value, ha="right", va="center", fontsize=8.4,
                    color=FG, fontproperties=fps, zorder=4)

    ax.axvline(0.0, color=ACCENT, linewidth=1.1, zorder=5)
    ax.axvline(trivial["df1"], color=MUTED, linewidth=1.0, linestyle=(0, (4, 3)), zorder=2)

    ax.set_yticks(y)
    # Two lines per row: what was flipped, then its direction and the run's time budget.
    ax.set_yticklabels(
        [f"{name[r['key']]}\n{flip[r['value']]} · {r['time_ms']:.1f} ms/frame"
         for r in runs],
        fontsize=8.6, color=FG, fontproperties=fps, linespacing=1.55)
    for tick, r in zip(ax.get_yticklabels(), runs):
        if r["key"] == TRIVIAL_SWITCH:
            tick.set_color(ACCENT)

    ax.set_xlim(min(r["df1"] for r in runs) - 1.5, 2.0)
    ax.set_ylim(-0.62, len(runs) - 0.38)
    ax.set_xlabel(t["xlabel"], fontsize=9, color=MUTED, fontproperties=fps)
    ax.tick_params(axis="x", labelsize=8.5, colors=MUTED, length=0)
    ax.tick_params(axis="y", length=0, pad=6)
    ax.grid(axis="x", color=GRID, linewidth=0.8, zorder=1)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(RULE)

    handles = [
        plt.Line2D([], [], color=ACCENT, linewidth=1.6,
                   label=f"{t['legend_released']} ({released['f1']:.2f})"),
        plt.Line2D([], [], color=MUTED, linewidth=1.2, linestyle=(0, (4, 3)),
                   label=f"{t['legend_trivial']} ({trivial['f1']:.2f})"),
    ]
    ax.legend(handles=handles, loc="lower left", bbox_to_anchor=(0.0, 0.005),
              frameon=False, fontsize=8.6, prop=fps, handlelength=1.6)

    fig.text(0.5, 0.965, t["title"], ha="center", va="top", fontsize=11.4, color=FG,
             fontweight="bold", fontproperties=fps)
    fig.text(0.5, 0.921, t["sub"].format(scenes=" / ".join(SCENES), frames=FRAMES),
             ha="center", va="top", fontsize=8.2, color=MUTED, fontproperties=fps)
    fig.text(0.012, 0.024, t["foot"].format(rel=f"{released['f1']:.2f}"), ha="left",
             va="bottom", fontsize=7.4, color=MUTED, fontproperties=fps, linespacing=1.5)
    fig.subplots_adjust(left=0.285, right=0.985, top=0.845, bottom=0.245)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, facecolor="white", transparent=False)
    plt.close(fig)
    # The figure conventions ask for no transparency; matplotlib still writes an alpha
    # channel, so flatten it (the background is already white).
    Image.open(out).convert("RGB").save(out)
    print(f"{out}")

    print(f"\nreleased macro F1 on scenes {'/'.join(SCENES)}: {released['f1']:.4f} "
          f"(P {released['precision']:.4f}, R {released['recall']:.4f}, "
          f"{released['time_ms']:.1f} ms/frame)")
    print("\n| Configuration | Switch | P | R | F1 | Δ F1 | ms/frame | "
          "documented Δ F1 | documented ms/frame |")
    print("|---|---|---:|---:|---:|---:|---:|---:|---:|")
    rel_doc = DOCUMENTED["released"]
    print(f"| Released | — | {released['precision']:.2f} | {released['recall']:.2f} | "
          f"{released['f1']:.2f} | — | {released['time_ms']:.1f} | — | {rel_doc[4]} |")
    for r in sorted(runs, key=lambda r: -r["df1"]):
        name_s = en[r["key"]]
        doc = DOCUMENTED.get(r["key"], (None,) * 5)[3]
        doc_ms = DOCUMENTED.get(r["key"], (None,) * 5)[4]
        doc_s = "0 (no-op)" if doc == 0.0 else (f"{doc:+.2f}" if doc is not None else "—")
        print(f"| {name_s} | {r['stem'].rsplit('_', 1)[1]} | {r['precision']:.2f} | "
              f"{r['recall']:.2f} | {r['f1']:.2f} | {r['df1']:+.2f} | {r['time_ms']:.1f} | "
              f"{doc_s} | {doc_ms if doc_ms is not None else '—'} |")

    drift = []
    for r in runs:
        doc = DOCUMENTED.get(r["key"])
        if not doc:
            continue
        for metric, d in zip((*METRICS, "df1"), doc[:4]):
            if d is None:
                continue
            if abs(r[metric] - d) > TOL:
                drift.append(f"{r['key']} {metric}: measured {r[metric]:.2f} vs "
                             f"OPTIMIZATION.md §6 {d:+.2f}")
    for metric, d in zip(METRICS, DOCUMENTED["released"]):
        if abs(released[metric] - d) > TOL:
            drift.append(f"released {metric}: measured {released[metric]:.2f} vs "
                         f"OPTIMIZATION.md §6 {d:.2f}")
    if drift:
        print("\n[WARN] disagrees with docs/OPTIMIZATION.md §6 (reported, not reconciled):")
        for d in drift:
            print(f"  - {d}")

    # F1 is deterministic, the timing is not: say so when the machine was busy, so a slower
    # run is not read as a regression of the module.
    slow = [(en[r["key"]], r["time_ms"], DOCUMENTED[r["key"]][4]) for r in runs
            if DOCUMENTED.get(r["key"], (None,) * 5)[4] and
            r["time_ms"] > DOCUMENTED[r["key"]][4] * 1.25]
    if slow or released["time_ms"] > rel_doc[4] * 1.25:
        print(f"\n[NOTE] ms/frame is above the documented value "
              f"(released {released['time_ms']:.1f} vs {rel_doc[4]} ms/frame) — "
              f"expected on a loaded machine; F1 is unaffected:")
        for name_s, got, doc_ms in slow:
            print(f"  - {name_s}: {got:.1f} vs {doc_ms} ms/frame")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
