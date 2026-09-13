#!/usr/bin/env python3
"""Fig. 14 - the weak-return test, and how much of the ground truth it can never reach.

The released rule admits a point only when its return is weak, so returns brighter than the
ceiling are invisible to it whatever the other parameters do. Two measurements, 12 frames per
scene:

    (a) the in-ROI intensity distribution of annotated returns against everything else: the two
        classes separate at `I = 0` rather than overlapping, which is why a weak-return test is
        the right shape of rule here at all
    (b) the share of annotated returns above the ceiling, per scene - the part of the story that
        is not uniform, and the reason scene 35 scores in the nineties while scene 16 does not

    SNOWCLEAR_DATA=<mirror> python3 tools/gen_intensity_fig.py --scenes 35 11 14 16
    SNOWCLEAR_DATA=<mirror> python3 tools/gen_intensity_fig.py --scenes 35 11 14 16 --lang zh

The per-scene numbers are recomputed from the clouds, so they can be checked against
docs/OPTIMIZATION.md section 2 line by line.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from pcd_common import load_indices, read_pcd   # noqa: E402

CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

FG = "#1f2328"
MUTED = "#6e7781"
RULE = "#d0d7de"
GRID = "#eaeef2"
SNOW = "#1a7f37"
OTHER = "#8c959f"
CEIL = "#cf222e"
BRIGHT = 2.0                 # the band the rule cannot admit, at any setting

TXT = {
    "en": dict(title="The weak-return test, and the ground truth it cannot reach",
               sub="in-ROI, {frames} frames per scene · (a) pooled over {scenes} scenes, "
                   "(b) per scene",
               xlabel="intensity of in-ROI returns", ylabel="points (log)",
               share="annotated returns above I = 2 (%)",
               legend=["annotated snow", "everything else"],
               foot="the rule admits a point only below the ceiling its threshold implies, so the "
                    "bar in (b) is a recall ceiling that no parameter setting can lift\n"
                    "regenerate: SNOWCLEAR_DATA=<mirror> python3 tools/gen_intensity_fig.py "
                    "--scenes 35 11 14 16"),
    "zh": dict(title="弱回波判据，以及它永远够不到的那部分真值",
               sub="ROI 内，每场景 {frames} 帧 · (a) 为 {scenes} 个场景合并统计，(b) 为逐场景",
               xlabel="ROI 内回波强度", ylabel="点数（对数）",
               share="被标注点中强度高于 I = 2 的比例（%）",
               legend=["被标注的雪", "其余全部"],
               foot="判定只接受低于阈值所隐含上限的点，因此 (b) 中的每根柱子都是任何参数设置都无法抬高的召回天花板\n"
                    "复现：SNOWCLEAR_DATA=<镜像> python3 tools/gen_intensity_fig.py --scenes 35 11 14 16"),
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", type=pathlib.Path,
                    default=pathlib.Path(os.environ.get("SNOWCLEAR_DATA", "./data")))
    ap.add_argument("--scenes", nargs="+", default=["35", "11", "14", "16"])
    ap.add_argument("--frames", type=int, default=12, help="frames sampled per scene")
    ap.add_argument("--roi", type=float, default=17.0)
    ap.add_argument("--bins", type=int, default=40)
    ap.add_argument("--out", type=pathlib.Path, default=None)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    a = ap.parse_args()
    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    out = a.out or pathlib.Path("docs/figures") / (
        "fig14_intensity_zh.png" if a.lang == "zh" else "fig14_intensity.png")

    snow_all, other_all, per_scene = [], [], []
    for scene in a.scenes:
        pcd_dir = a.data / "pcd_output" / scene / "velodyne"
        gt_dir = a.data / "result" / scene
        frames = sorted(pcd_dir.glob("*.pcd"))
        if not frames:
            print(f"  scene {scene}: no frames, skipped")
            continue
        step = max(1, len(frames) // a.frames)
        snow_s, other_s = [], []
        gt_in = bright = 0
        for frame in frames[::step][:a.frames]:
            gt = gt_dir / f"{frame.stem}.txt"
            if not gt.exists():
                continue
            pts = read_pcd(frame)
            x, y, z, inten = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
            r3 = np.sqrt(x * x + y * y + z * z)
            elev = np.degrees(np.arcsin(np.clip(z / np.maximum(r3, 1e-9), -1.0, 1.0)))
            roi = (z >= -1.0) & (z <= 2.6) & (x * x + y * y <= a.roi ** 2) & (elev >= -23.0)
            gset = np.zeros(pts.shape[0], dtype=bool)
            idx = load_indices(gt)
            gset[idx[idx < pts.shape[0]]] = True
            s = inten[roi & gset]
            snow_s.append(s); other_s.append(inten[roi & ~gset])
            gt_in += s.size
            bright += int((s >= BRIGHT).sum())
        if not snow_s:
            continue
        snow = np.concatenate(snow_s); other = np.concatenate(other_s)
        snow_all.append(snow); other_all.append(other)
        share = 100.0 * bright / max(gt_in, 1)
        per_scene.append((scene, share, gt_in))
        print(f"  scene {scene:>3}: {gt_in:>7} annotated in ROI, {share:5.2f} % above I = {BRIGHT:.0f}"
              f"   (zero-intensity {100.0 * (snow == 0).mean():5.2f} % of annotated, "
              f"{100.0 * (other == 0).mean():5.2f} % of the rest)")
    if not per_scene:
        raise SystemExit(f"no comparable frames under {a.data}")
    snow = np.concatenate(snow_all); other = np.concatenate(other_all)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.4, 3.8), dpi=200,
                                   gridspec_kw=dict(width_ratios=[1.55, 1.0], wspace=0.2))
    fig.patch.set_facecolor("white")
    bins = np.linspace(0, 6, a.bins)
    ax1.hist(other, bins=bins, color=OTHER, zorder=3, label=t["legend"][1])
    ax1.hist(snow, bins=bins, color=SNOW, zorder=4, alpha=0.92, label=t["legend"][0])
    ax1.set_yscale("log")
    ax1.axvline(BRIGHT, color=CEIL, linewidth=1.3, linestyle=(0, (4, 2)), zorder=5)
    ax1.text(BRIGHT + 0.08, ax1.get_ylim()[1] * 0.35,
             f"unreachable  I ≥ {BRIGHT:.0f}", color=CEIL, fontsize=8.2, va="top",
             fontproperties=fps)
    ax1.set_xlabel(t["xlabel"], fontsize=8.6, color=MUTED, fontproperties=fps)
    ax1.set_ylabel(t["ylabel"], fontsize=8.6, color=MUTED, fontproperties=fps)

    x = np.arange(len(per_scene))
    shares = [p[1] for p in per_scene]
    ax2.bar(x, shares, 0.55, color=CEIL, zorder=3)
    for i, (scene, share, _) in enumerate(per_scene):
        ax2.text(i, share + max(shares) * 0.03, f"{share:.2f}", ha="center", va="bottom",
                 fontsize=8.4, color=FG, fontproperties=fps)
    ax2.set_xticks(x)
    ax2.set_xticklabels([p[0] for p in per_scene], fontsize=8.8, color=FG, fontproperties=fps)
    ax2.set_ylabel(t["share"], fontsize=8.6, color=MUTED, fontproperties=fps)
    ax2.set_ylim(0, max(shares) * 1.30)
    for ax in (ax1, ax2):
        ax.set_facecolor("white")
        ax.grid(axis="y", color=GRID, linewidth=0.8, zorder=1)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.spines["left"].set_color(RULE)
        ax.spines["bottom"].set_color(RULE)
        ax.tick_params(length=0, labelsize=8.2, colors=MUTED)
    ax1.legend(loc="upper right", frameon=False, fontsize=8.6, prop=fps, handlelength=1.4)
    fig.text(0.5, 0.975, t["title"], ha="center", va="top", fontsize=11.4, color=FG,
             fontweight="bold", fontproperties=fps)
    fig.text(0.5, 0.925, t["sub"].format(frames=a.frames, scenes=len(per_scene)),
             ha="center", va="top", fontsize=8.2, color=MUTED, fontproperties=fps)
    fig.text(0.008, 0.015, t["foot"], ha="left", va="bottom", fontsize=7.2, color=MUTED,
             fontproperties=fps, linespacing=1.5)
    fig.subplots_adjust(left=0.068, right=0.995, top=0.795, bottom=0.255)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, facecolor="white")
    plt.close(fig)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
