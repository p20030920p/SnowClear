#!/usr/bin/env python3
"""One frame, five detectors, one camera: the qualitative companion to Table 2.

Every panel is drawn by `render_hero3d.draw_scene` with a single shared camera and the
same palette as the README hero, so the only thing that differs between panels is which
points each method filed under tp / fn / fp. Framing comes from the ground truth and the
ROI structure, never from a method's own output, so no panel can flatter itself by
zooming somewhere convenient.

    python3 tools/render_baseline_clouds.py --pcd 042126.pcd --gt result/35/042126.txt \\
        --series SnowClear=det/feature_fusion/042126.txt \\
        --series DROR=det/dror/042126.txt --series DSOR=det/dsor/042126.txt \\
        --series SOR=det/sor/042126.txt --series ROR=det/ror/042126.txt \\
        --out docs/figures/fig12_baselines.png

Panel numbers are per-frame and in-ROI; the 1 620-frame macro is Fig. 3 / Table 2.
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from pcd_common import read_pcd, load_indices                      # noqa: E402
from render_hero3d import (BG, FN_C, FP_C, STRUCT_HI, TP_C,        # noqa: E402
                           camera, draw_scene, project)

CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

TXT = {
    "en": dict(title="One frame, five detectors — frame {f}, scene 35",
               sub="identical ROI gate and index mapping; every panel rendered with the "
                   "same camera",
               gt="Ground truth", annotated="{n} annotated points inside the ROI",
               det="{d} detections · {a} of them annotated",
               foot="per-frame, in-ROI numbers; the 1 620-frame macro average is Fig. 3 / "
                    "Table 2",
               legend=["ROI structure", "detected & annotated — TP",
                       "annotated, missed — FN", "detected, not annotated — FP"]),
    "zh": dict(title="同一帧、五种检测器 —— 帧 {f}，场景 35",
               sub="共用同一 ROI 门控与索引映射；所有面板使用同一相机",
               gt="真值标注", annotated="ROI 内标注雪点 {n} 个",
               det="检出 {d} 点 · 其中 {a} 点被标注",
               foot="数值为单帧 ROI 内统计；1 620 帧宏平均见图 3 / 表 2",
               legend=["ROI 内结构", "检出且被标注 —— TP", "被标注但漏检 —— FN",
                       "检出但无标注 —— FP"]),
}


def classes(pts, roi, snow, gset):
    return {"structure": roi & ~snow & ~gset, "tp": roi & snow & gset,
            "fn": roi & gset & ~snow, "fp": roi & snow & ~gset}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pcd", type=pathlib.Path, required=True)
    ap.add_argument("--gt", type=pathlib.Path, required=True)
    ap.add_argument("--series", action="append", default=[],
                    help="Label=indices.txt, repeatable; the first series is drawn in the "
                         "ground-truth slot")
    ap.add_argument("--gt-label", default="", help="title for the ground-truth panel")
    ap.add_argument("--out", type=pathlib.Path,
                    default=pathlib.Path("docs/figures/fig12_baselines.png"))
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    ap.add_argument("--frame", default="")
    ap.add_argument("--roi", type=float, default=17.0)
    ap.add_argument("--azim", type=float, default=-58.0)
    ap.add_argument("--elev", type=float, default=30.0)
    ap.add_argument("--fog", type=float, default=0.55)
    ap.add_argument("--dpi", type=int, default=170)
    ap.add_argument("--width", type=float, default=11.4, help="figure width, inches")
    ap.add_argument("--height", type=float, default=5.1, help="figure height, inches")
    a = ap.parse_args()
    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]

    pts = read_pcd(a.pcd)
    if not np.isfinite(pts).all(axis=1).all():
        pts = pts[np.isfinite(pts).all(axis=1)]
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r3 = np.sqrt(x * x + y * y + z * z)
    elev = np.degrees(np.arcsin(np.clip(z / np.maximum(r3, 1e-9), -1.0, 1.0)))
    roi = (z >= -1.0) & (z <= 2.6) & (x * x + y * y <= a.roi ** 2) & (elev >= -23.0)
    gset = np.zeros(pts.shape[0], dtype=bool)
    gt = load_indices(a.gt)
    gset[gt[gt < pts.shape[0]]] = True

    panels = [(a.gt_label or t["gt"], None)]
    for item in a.series:
        label, _, path = item.partition("=")
        snow = np.zeros(pts.shape[0], dtype=bool)
        idx = load_indices(pathlib.Path(path))
        snow[idx[idx < pts.shape[0]]] = True
        panels.append((label, snow))

    # ---- one camera for every panel, framed by the ground truth -------------
    anchor = (roi & gset) | (roi & ~gset)
    span = max(float(np.ptp(x[roi & gset])), float(np.ptp(y[roi & gset])))
    target = np.array([float(np.median(x[roi & gset])), float(np.median(y[roi & gset])),
                       float(np.percentile(z[roi & gset], 60))])
    dist = 2.05 * max(span, 6.0)
    focal = 1.30
    frame = camera(a.azim, a.elev)
    sx, sy, _ = project(pts[anchor][:, :3], target, frame, dist, focal)
    x0, x1 = np.percentile(sx, [0.4, 99.6])
    y0, y1 = np.percentile(sy, [0.4, 99.6])
    mx, my = 0.05 * (x1 - x0), 0.05 * (y1 - y0)
    view = [x0 - mx, x1 + mx, y0 - my, y1 + my]

    cols = 3
    rows = int(np.ceil(len(panels) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(a.width, a.height), dpi=a.dpi,
                             facecolor=BG)
    fig.subplots_adjust(left=0.006, right=0.994, top=0.845, bottom=0.062,
                        wspace=0.015, hspace=0.03)
    flat = axes.ravel() if hasattr(axes, "ravel") else [axes]

    for ax, (label, snow) in zip(flat, panels):
        ax.set_facecolor(BG)
        if snow is None:                       # ground truth: annotated snow is the class
            cls = {"structure": roi & ~gset, "tp": roi & gset,
                   "fn": np.zeros_like(gset), "fp": np.zeros_like(gset)}
            n = int(cls["tp"].sum())
            note = t["annotated"].format(n=f"{n:,}".replace(",", " "))
        else:
            cls = classes(pts, roi, snow, gset)
            tp, fp, fn = (int(cls[k].sum()) for k in ("tp", "fp", "fn"))
            prec = 100.0 * tp / max(tp + fp, 1)
            rec = 100.0 * tp / max(tp + fn, 1)
            f1 = 2 * prec * rec / (prec + rec) if prec + rec else 0.0
            note = f"P {prec:.1f} · R {rec:.1f} · F1 {f1:.1f}\n" + \
                   t["det"].format(d=f"{tp + fp:,}".replace(",", " "),
                                   a=f"{tp:,}".replace(",", " "))
        draw_scene(ax, pts, cls, target, frame, dist, focal, view, a.fog)
        ax.set_xlim(view[0], view[1]); ax.set_ylim(view[2], view[3])
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_axis_off()
        # inside the panel, not under it: a caption below the axes lands on the next
        # row's title as soon as the rows are packed tightly enough to be readable
        plate = dict(boxstyle="round,pad=0.32", facecolor="#0b1017", edgecolor="#2b3846",
                     linewidth=0.8, alpha=0.82)
        ax.text(0.016, 0.965, label, transform=ax.transAxes, fontsize=11.0,
                color="#eef3f9", va="top", ha="left", fontproperties=fps,
                fontweight="bold", zorder=12, bbox=plate)
        ax.text(0.016, 0.815, note, transform=ax.transAxes, fontsize=8.0, va="top",
                ha="left", color="#a9b6c5", linespacing=1.55, fontproperties=fps,
                zorder=12, bbox=plate)

    for ax in flat[len(panels):]:
        ax.set_axis_off()

    handles = [Line2D([], [], marker="o", ls="", markersize=6.0, color=c,
                      markeredgecolor=BG, label=lab)
               for c, lab in zip(("#aab4c0", TP_C, FN_C, FP_C), t["legend"])]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.995), ncol=4,
               frameon=False, fontsize=9.0, prop=fps, handletextpad=0.4,
               columnspacing=1.6, labelcolor="#dce4ee")
    fig.text(0.5, 0.955, t["title"].format(f=a.frame), ha="center", va="top",
             fontsize=12.0, color="#f0f4f9", fontweight="bold", fontproperties=fps)
    fig.text(0.5, 0.915, t["sub"], ha="center", va="top", fontsize=8.2,
             color="#8b98a8", fontproperties=fps)
    fig.text(0.5, 0.012, t["foot"], ha="center", va="bottom", fontsize=7.6,
             color="#7d8794", fontproperties=fps)

    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=a.dpi, facecolor=BG)
    plt.close(fig)
    print(f"{a.out}  panels: {', '.join(p[0] for p in panels)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
