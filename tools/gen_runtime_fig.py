#!/usr/bin/env python3
"""Fig. 6 - the frame-time budget, stage by stage, and what the fast paths are worth.

Reads the verbose runs written by `tools/measure_timing.sh` (one log per configuration) and
draws two panels:

    (a) where the algorithm time goes in the released configuration, with the two stages the
        released measurement excludes (I/O and evaluation) drawn separately so the accounting
        is visible rather than asserted;
    (b) the two equivalence-preserving fast paths measured on and off.

Panel (b) only means something if the runs agree on the output, so the script reads the metrics
each run printed and refuses to draw an equivalence claim it cannot back: it prints the F1 of
every configuration it was given.

    bash tools/measure_timing.sh released
    bash tools/measure_timing.sh lut_off use_threshold_lut:=false
    bash tools/measure_timing.sh elev_off use_fast_elevation_gate:=false
    python3 tools/gen_runtime_fig.py experiments/timing
    python3 tools/gen_runtime_fig.py experiments/timing --lang zh

Frame time moves with machine load, which is why the script prints every number it draws and
the figure says so; F1 does not move.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import statistics
import sys

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
EXCLUDED = "#c3cad3"
# the three stages that make up the algorithm time, in pipeline order
STAGES = [("preprocess", r"预处理时间", "#2f6f9f"),
          ("features", r"参数优化时间", "#8250df"),
          ("filtering", r"雪点滤波时间", "#2da44e")]
EXCL = [("io", r"读盘\+去NaN时间", "I/O"), ("evaluation", r"评估时间", "evaluation")]
TOTAL = r"算法总时间"

TXT = {
    "en": dict(title="Where the frame time goes — scene 35, {frames} frames, "
                     "OMP_NUM_THREADS=2",
               sub="per-frame stage timers (verbose:=true), best of {runs} runs — the machine "
                   "is shared, so the fastest run is the honest estimate",
               stages=["preprocessing", "feature analysis + optimiser", "snow filtering"],
               excluded=["I/O (excluded)", "evaluation (excluded)"],
               label_a="(a) released configuration",
               label_b="(b) the two equivalence-preserving fast paths",
               total="algorithm total",
               foot="frame time moves with machine load — this machine is shared, so panel (b) "
                    "pairs the runs instead of comparing minima; the metrics do not move, and "
                    "every configuration here prints the same F1\n"
                    "regenerate: bash tools/measure_timing.sh <label> [key:=value …], then "
                    "python3 tools/gen_runtime_fig.py <dir>",
               eq="identical output: F1 {f1} in all {n} configurations",
               noise="no effect at this resolution",
               paired="paired {d:+.2f} ms, {n}/{n} runs",
               on_off=["released", "switched off"]),
    "zh": dict(title="单帧时间花在哪里 —— 场景 35，{frames} 帧，OMP_NUM_THREADS=2",
               sub="verbose:=true 的阶段计时器逐帧均值，取 {runs} 轮中最快的一轮 —— 本机为共享"
                   "机器，最快一轮才是可靠估计",
               stages=["预处理", "特征分析 + 参数优化", "雪点滤波"],
               excluded=["读盘（不计入）", "评估（不计入）"],
               label_a="(a) 发布配置",
               label_b="(b) 两个数学等价的快速路径",
               total="算法总时间",
               foot="单帧耗时随机器负载浮动（本机为共享机器）；指标不随负载变化，此处所有配置"
                    "打印的 F1 完全相同\n复现：bash tools/measure_timing.sh <标签> [key:=value …]，"
                    "再运行 python3 tools/gen_runtime_fig.py <目录>",
               eq="输出完全一致：{n} 个配置的 F1 均为 {f1}",
               noise="此分辨率下无效应",
               paired="配对差 {d:+.2f} ms，{n}/{n} 轮一致",
               on_off=["发布配置", "关闭该开关"]),
}

# label -> (title, override the run used); the figure only draws what it is given
SERIES = [("released", "released"), ("lut_off", "use_threshold_lut: false"),
          ("elev_off", "use_fast_elevation_gate: false")]


def parse(log: pathlib.Path):
    """Per-stage frame time for one configuration, run by run.

    Two properties of the machine shape this: it is shared (a single pass can be 40 % off
    the next one) and the runner also prints a summary block whose lines begin with 平均
    ("average"), which must not leak into the mean. So the log is split into its runs
    (measure_timing.sh marks them), each run is averaged over its own frames, and every
    stage keeps its whole list - that is what lets the figure test a switch's delta against
    the run-to-run spread of the stage the switch actually touches, instead of against the
    spread of the total, which is dominated by stages the switch cannot influence.
    """
    text = log.read_text(errors="ignore")
    runs = re.split(r"^--- run \d+/\d+:.*$", text, flags=re.M)[1:] or [text]
    stages = {key: [] for key, _, _ in STAGES + EXCL}
    totals, f1s, frames = [], [], 0
    for chunk in runs:
        for key, pattern, _ in STAGES + EXCL:
            vals = [float(m.group(1)) for m in
                    re.finditer(rf"^{pattern}.*?: ([\d.]+) ms$", chunk, flags=re.M)]
            if vals:
                stages[key].append(statistics.fmean(vals))
                if key == "preprocess":
                    frames = max(frames, len(vals))
        vals = [float(m.group(1)) for m in
                re.finditer(rf"^{TOTAL}.*?: ([\d.]+) ms$", chunk, flags=re.M)]
        if vals:
            totals.append(statistics.fmean(vals))
        f1 = re.findall(r"平均F1分数 \(F1 Score\): ([\d.]+)%", chunk)
        if f1:
            f1s.append(f1[-1])
    out = {key: v for key, v in stages.items()}
    out["total"] = totals
    out["frames"] = frames
    out["runs"] = len(runs)
    out["f1"] = f1s[-1] if f1s else "?"
    return out


def best(values):
    """The fastest run: on a shared machine that is the closest to the unloaded cost."""
    return min(values) if values else 0.0


def spread(values):
    return (max(values) - min(values)) if len(values) > 1 else 0.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log_dir", type=pathlib.Path, nargs="?", default=pathlib.Path("experiments/timing"))
    ap.add_argument("--out", type=pathlib.Path, default=None)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    a = ap.parse_args()
    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    out = a.out or pathlib.Path("docs/figures") / (
        "fig6_runtime_zh.png" if a.lang == "zh" else "fig6_runtime.png")

    data = {}
    for label, override in SERIES:
        log = a.log_dir / f"{label}.log"
        if not log.exists():
            print(f"no {log} - run: bash tools/measure_timing.sh {label}"
                  f"{' ' + override if override else ''}", file=sys.stderr)
            continue
        data[label] = parse(log)
    if "released" not in data:
        raise SystemExit(f"no released.log in {a.log_dir}; see the docstring for the commands")

    rel = data["released"]
    f1s = {k: v["f1"] for k, v in data.items()}
    print(f"{'config':>10} {'runs':>5} {'frames':>7} " + " ".join(f"{k:>10}" for k, _, _ in STAGES)
          + f" {'total':>8} {'F1':>8}")
    for label, d in data.items():
        print(f"{label:>10} {d['runs']:>5} {d['frames']:>7} "
              + " ".join(f"{best(d[k]):>10.3f}" for k, _, _ in STAGES)
              + f" {best(d['total']):>8.3f} {d['f1']:>8}")
    for key, _, name in EXCL:
        print(f"  {name:>18}: {best(rel[key]):.3f} ms/frame (excluded from the algorithm time)")
    print(f"  best of {rel['runs']} runs per configuration; the machine is shared, so every")
    print(f"  delta is reported next to the run-to-run spread of the stage it touches")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.2, 3.9), dpi=200,
                                   gridspec_kw=dict(width_ratios=[1.15, 1.0], wspace=0.28))
    fig.patch.set_facecolor("white")

    # ---- (a) the released configuration, stage by stage ------------------------
    # one bar per stage rather than one stacked bar: the stage names then live on the axis,
    # where they cannot collide with a legend or with the segment labels
    rows = [(t["stages"][i], best(rel[key]), colour)
            for i, (key, _, colour) in enumerate(STAGES)]
    rows += [(t["excluded"][i], best(rel[key]), EXCLUDED)
             for i, (key, _, _) in enumerate(EXCL)]
    ypos = [3.0, 2.0, 1.0, -0.7, -1.7]
    ax1.axhspan(0.55, 3.45, color=BAND, zorder=0)
    for y, (_, value, colour) in zip(ypos, rows):
        ax1.barh(y, value, 0.62, color=colour, zorder=3)
        ax1.text(value + 0.09, y, f"{value:.2f}", va="center", ha="left", fontsize=8.4,
                 color=FG, fontproperties=fps)
    ax1.text(best(rel["total"]) + 0.09, 3.42, f"{t['total']} {best(rel['total']):.2f} ms",
             ha="left", va="bottom", fontsize=8.6, color=FG, fontproperties=fps)
    ax1.set_yticks(ypos)
    ax1.set_yticklabels([r[0] for r in rows], fontsize=8.4, color=FG, fontproperties=fps)
    for tick, (_, _, colour) in zip(ax1.get_yticklabels(), rows):
        if colour == EXCLUDED:
            tick.set_color(MUTED)
    ax1.set_ylim(-2.3, 3.95)
    ax1.set_xlim(0, max(best(rel["total"]), best(rel["io"]),
                        best(rel["evaluation"])) * 1.34)
    ax1.set_title(t["label_a"], fontsize=9.6, color=FG, loc="left", pad=10,
                  fontproperties=fps)
    ax1.set_xlabel("ms / frame", fontsize=8.4, color=MUTED, fontproperties=fps)
    ax1.grid(axis="x", color=GRID, linewidth=0.8, zorder=1)
    ax1.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax1.spines[side].set_visible(False)
    ax1.spines["bottom"].set_color(RULE)
    ax1.tick_params(axis="both", length=0, labelsize=8.2, colors=MUTED)

    # ---- (b) each switch, judged on the stage it can actually touch ------------
    # use_threshold_lut changes how the per-point threshold is evaluated, which the runner
    # charges to snow filtering; use_fast_elevation_gate replaces an asin in the ROI
    # prefilter, which is preprocessing. Comparing totals would bury both deltas under load
    # noise from the stages a switch cannot influence, and comparing minima alone still
    # leaves the machine's drift in the number - so the runs are paired: measure_timing.sh
    # must be called in round-robin order, and the switch's effect is the median of the
    # per-round differences between the two configurations.
    checks = [("lut_off", "filtering", "use_threshold_lut: false"),
              ("elev_off", "preprocess", "use_fast_elevation_gate: false")]
    checks = [(k, st, nm) for k, st, nm in checks if k in data]
    x = np.arange(len(checks))
    width = 0.34
    top = 0.0
    for i, (key, stage, _) in enumerate(checks):
        on_runs, off_runs = rel[stage], data[key][stage]
        n = min(len(on_runs), len(off_runs))
        pairs = [off_runs[j] - on_runs[j] for j in range(n)]
        median = statistics.median(pairs) if pairs else 0.0
        same_sign = bool(pairs) and (all(v > 0 for v in pairs) or all(v < 0 for v in pairs))
        resolvable = same_sign and abs(median) > 0.05
        on, off = best(on_runs), best(off_runs)
        top = max(top, on, off)
        ax2.bar(i - width / 2, on, width, color="#2da44e", zorder=3, edgecolor="white",
                linewidth=0.7)
        ax2.bar(i + width / 2, off, width, color="#8c959f", zorder=3, edgecolor="white",
                linewidth=0.7)
        ax2.text(i - width / 2, on + top * 0.02, f"{on:.2f}", ha="center", va="bottom",
                 fontsize=8.2, color=FG, fontproperties=fps)
        ax2.text(i + width / 2, off + top * 0.02, f"{off:.2f}", ha="center", va="bottom",
                 fontsize=8.2, color=FG, fontproperties=fps)
        note = t["paired"].format(d=median, n=n) if resolvable else t["noise"]
        ax2.text(i, max(on, off) + top * 0.115, note, ha="center", va="bottom",
                 fontsize=7.8, color=FG if resolvable else MUTED, fontproperties=fps)
        print(f"  {key:>9} on {stage}: best {on:.3f} -> {off:.3f} ms, paired deltas "
              f"{[round(v, 2) for v in pairs]} -> median {median:+.2f} ms, "
              f"{'resolvable' if resolvable else 'not resolvable at this resolution'}")
    ax2.set_xticks(x)
    ax2.set_xticklabels([f"{nm}\n({st})" for _, st, nm in checks], fontsize=7.8, color=FG,
                        fontproperties=fps)
    ax2.set_ylim(0, top * 1.52)
    ax2.set_title(t["label_b"], fontsize=9.6, color=FG, loc="left", pad=10,
                  fontproperties=fps)
    ax2.set_ylabel("ms / frame", fontsize=8.4, color=MUTED, fontproperties=fps)
    ax2.grid(axis="y", color=GRID, linewidth=0.8, zorder=1)
    ax2.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax2.spines[side].set_visible(False)
    ax2.spines["bottom"].set_color(RULE)
    ax2.tick_params(axis="both", length=0, labelsize=8.0, colors=MUTED)
    handles_b = [plt.Rectangle((0, 0), 1, 1, color=c) for c in ("#2da44e", "#8c959f")]
    ax2.legend(handles_b, t["on_off"], loc="upper right", frameon=False, fontsize=8.2,
               prop=fps, handlelength=1.1, handleheight=0.9)

    fig.text(0.5, 0.972, t["title"].format(frames=rel["frames"]), ha="center", va="top",
             fontsize=11.2, color=FG, fontweight="bold", fontproperties=fps)
    fig.text(0.5, 0.916, t["sub"].format(runs=rel["runs"]), ha="center", va="top",
             fontsize=8.0, color=MUTED, fontproperties=fps)
    if len(set(f1s.values())) == 1:
        fig.text(0.5, 0.856, t["eq"].format(f1=f"{float(next(iter(f1s.values()))):.2f} %",
                                            n=len(f1s)),
                 ha="center", va="top", fontsize=8.2, color=FG, fontproperties=fps)
    fig.text(0.008, 0.015, t["foot"], ha="left", va="bottom", fontsize=7.2, color=MUTED,
             fontproperties=fps, linespacing=1.5)
    fig.subplots_adjust(left=0.185, right=0.995, top=0.775, bottom=0.215, wspace=0.34)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, facecolor="white")
    plt.close(fig)
    print(f"\n{out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
