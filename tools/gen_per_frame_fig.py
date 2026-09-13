#!/usr/bin/env python3
"""Fig. 13 - one scene, frame by frame: how stable is the detector, and what does it remove?

The animation at the top of the README shows 21 frames of one scene; this is all 101 of them,
so the reader can see whether what they watched was typical. Two questions in one figure:

    (a) precision / recall / F1 per frame, with the frames that fall off called out by name
    (b) how many points were annotated and how many the detector removed, per frame

(b) is the part that makes (a) readable: a recall dip on a frame with 30 annotated points is a
different event from the same dip on a frame with 3 000.

    python3 tools/gen_per_frame_fig.py --pcd-dir <mirror>/pcd_output/35/velodyne \\
        --gt-dir <mirror>/result/35 --detection-dir <dump>/velodyne --scene 35
    python3 tools/gen_per_frame_fig.py ... --lang zh

Metric definition, ROI gate and index space are the same ones the tables use, so a point counted
here is a point counted there.
"""

from __future__ import annotations

import argparse
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
BAND = "#f6f8fa"
PREC = "#2f6f9f"
REC = "#2da44e"
F1C = "#8250df"
COUNT_GT = "#c3cad3"
COUNT_DET = "#1a7f37"

TXT = {
    "en": dict(title="Scene {scene}, every frame: detection quality and how much snow there was",
               sub="per-frame, in-ROI · macro over the scene in the corner of each panel",
               ylabel_score="%", ylabel_count="points",
               legend=["precision", "recall", "F1", "annotated", "removed"],
               worst="lowest F1: {items}",
               foot="the same 101 frames behind the animation; the ground-truth count is what the "
                    "score is computed against, so a dip on a bare frame costs less than it looks\n"
                    "regenerate: python3 tools/gen_per_frame_fig.py --pcd-dir … --gt-dir … "
                    "--detection-dir …"),
    "zh": dict(title="场景 {scene} 的每一帧：检测质量，以及那帧到底有多少雪",
               sub="逐帧、ROI 内统计 · 各面板角落给出该场景的宏平均",
               ylabel_score="%", ylabel_count="点数",
               legend=["精确率", "召回率", "F1", "被标注", "被剔除"],
               worst="F1 最低的几帧：{items}",
               foot="即动图背后的同一批 101 帧；分数是相对真值算的，因此空帧上的回落没有看上去那么严重\n"
                    "复现：python3 tools/gen_per_frame_fig.py --pcd-dir … --gt-dir … --detection-dir …"),
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pcd-dir", type=pathlib.Path, required=True)
    ap.add_argument("--gt-dir", type=pathlib.Path, required=True)
    ap.add_argument("--detection-dir", type=pathlib.Path, required=True)
    ap.add_argument("--scene", default="")
    ap.add_argument("--roi", type=float, default=17.0)
    ap.add_argument("--out", type=pathlib.Path, default=None)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    a = ap.parse_args()
    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    out = a.out or pathlib.Path("docs/figures") / (
        "fig13_per_frame_zh.png" if a.lang == "zh" else "fig13_per_frame.png")

    rows = []
    for frame in sorted(a.pcd_dir.glob("*.pcd")):
        det = a.detection_dir / f"{frame.stem}.txt"
        gt = a.gt_dir / f"{frame.stem}.txt"
        if not det.exists() or not gt.exists():
            continue
        pts = read_pcd(frame)
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        r3 = np.sqrt(x * x + y * y + z * z)
        elev = np.degrees(np.arcsin(np.clip(z / np.maximum(r3, 1e-9), -1.0, 1.0)))
        roi = (z >= -1.0) & (z <= 2.6) & (x * x + y * y <= a.roi ** 2) & (elev >= -23.0)
        snow = np.zeros(pts.shape[0], dtype=bool)
        idx = load_indices(det); snow[idx[idx < pts.shape[0]]] = True
        gset = np.zeros(pts.shape[0], dtype=bool)
        idx = load_indices(gt); gset[idx[idx < pts.shape[0]]] = True
        tp = int((roi & snow & gset).sum())
        fp = int((roi & snow & ~gset).sum())
        fn = int((roi & gset & ~snow).sum())
        p = 100.0 * tp / max(tp + fp, 1)
        r = 100.0 * tp / max(tp + fn, 1)
        rows.append((frame.stem, p, r, 2 * p * r / (p + r) if p + r else 0.0,
                     int(gset.sum()), int(snow.sum())))
    if not rows:
        raise SystemExit(f"no comparable frames under {a.pcd_dir}")
    names = [r[0] for r in rows]
    p = np.array([r[1] for r in rows]); r = np.array([r[2] for r in rows])
    f1 = np.array([r[3] for r in rows])
    gt_n = np.array([r[4] for r in rows]); det_n = np.array([r[5] for r in rows])
    idx = np.arange(len(rows))
    print(f"{len(rows)} frames   macro P {p.mean():.2f}  R {r.mean():.2f}  F1 {f1.mean():.2f}")
    print(f"  annotated {gt_n.mean():.0f} / frame, removed {det_n.mean():.0f} / frame")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10.6, 5.4), dpi=190, sharex=True,
                                   gridspec_kw=dict(height_ratios=[1.35, 1.0], hspace=0.12))
    fig.patch.set_facecolor("white")
    for ax in (ax1, ax2):
        ax.set_facecolor("white")
        ax.grid(axis="y", color=GRID, linewidth=0.8, zorder=1)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.spines["left"].set_color(RULE)
        ax.spines["bottom"].set_color(RULE)
        ax.tick_params(length=0, labelsize=8.2, colors=MUTED)

    ax1.plot(idx, p, color=PREC, linewidth=1.0, alpha=0.85)
    ax1.plot(idx, r, color=REC, linewidth=1.0, alpha=0.85)
    ax1.plot(idx, f1, color=F1C, linewidth=1.9)
    ax1.axhline(f1.mean(), color=F1C, linewidth=0.9, linestyle=(0, (5, 3)), zorder=2)
    ax1.set_ylim(0, 104)
    ax1.set_ylabel(t["ylabel_score"], fontsize=8.6, color=MUTED, fontproperties=fps)
    handles = [plt.Line2D([], [], color=c, linewidth=2.0) for c in (PREC, REC, F1C)]
    ax1.legend(handles, t["legend"][:3], loc="lower left", bbox_to_anchor=(0.0, 1.01),
               frameon=False, fontsize=8.6, ncol=3, prop=fps, handlelength=1.4)
    worst = np.argsort(f1)[:3]
    # one arrow on the frame that actually falls, and the rest as a list: three arrows in the
    # same dip overlap into an unreadable knot
    i0 = int(worst[0])
    ax1.annotate(f"{names[i0]}  F1 {f1[i0]:.0f}", xy=(i0, f1[i0]), xytext=(i0, 55),
                 ha="center", fontsize=8.0, color=FG, fontproperties=fps,
                 arrowprops=dict(arrowstyle="->", color=MUTED, linewidth=0.8))
    ax1.text(0.995, 0.04, f"macro  P {p.mean():.1f} · R {r.mean():.1f} · F1 {f1.mean():.1f}",
             transform=ax1.transAxes, ha="right", va="bottom", fontsize=9.0, color=FG,
             fontproperties=fps)
    ax1.text(0.995, 0.16, t["worst"].format(items=" · ".join(
        f"{names[i]} {f1[i]:.0f}" for i in worst)), transform=ax1.transAxes, ha="right",
        va="bottom", fontsize=7.8, color=MUTED, fontproperties=fps)

    ax2.bar(idx, gt_n, 0.86, color=COUNT_GT, zorder=3, label=t["legend"][3])
    ax2.plot(idx, det_n, color=COUNT_DET, linewidth=1.4, zorder=4, label=t["legend"][4])
    ax2.set_ylabel(t["ylabel_count"], fontsize=8.6, color=MUTED, fontproperties=fps)
    ax2.set_xlabel("frame index", fontsize=8.6, color=MUTED, fontproperties=fps)
    ax2.legend(loc="lower left", bbox_to_anchor=(0.0, 1.01), frameon=False, fontsize=8.6,
               ncol=2, prop=fps, handlelength=1.4)

    fig.text(0.5, 0.975, t["title"].format(scene=a.scene or a.pcd_dir.parent.parent.name),
             ha="center", va="top", fontsize=11.6, color=FG, fontweight="bold",
             fontproperties=fps)
    fig.text(0.5, 0.930, t["sub"], ha="center", va="top", fontsize=8.2, color=MUTED,
             fontproperties=fps)
    fig.text(0.008, 0.015, t["foot"], ha="left", va="bottom", fontsize=7.2, color=MUTED,
             fontproperties=fps, linespacing=1.5)
    fig.subplots_adjust(left=0.062, right=0.995, top=0.845, bottom=0.145)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=190, facecolor="white")
    plt.close(fig)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
