#!/usr/bin/env python3
"""The comparison board: one frame, every method, one camera.

Eight cells, all drawn by `bev_panel` - the same code that draws the animation at the top of the
README - so the board reads as one picture instead of six unrelated renderings:

    ground truth | SnowClear | CRFOR | DROR
    DSOR         | SOR       | ROR   | scoreboard

Each panel carries its own in-ROI precision / recall / F1 for that frame, computed by the same
function the tables use, and the scoreboard repeats them side by side so the visual and the numeric
comparison cannot drift apart.

    python3 tools/render_comparison_board.py --pcd <frame>.pcd --gt <gt>.txt \\
        --series SnowClear=det/feature_fusion.txt --series CRFOR=det/crfor.txt \\
        --series DROR=det/dror.txt --series DSOR=det/dsor.txt \\
        --series SOR=det/sor.txt --series ROR=det/ror.txt \\
        --out docs/figures/fig12_baselines.png --frame 042126

`--series` order is the drawing order after the ground-truth panel; pass `--skip` labels to leave a
method out. CRFOR's indices come from tools/eval_crfor.py, which runs the upstream implementation.
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
from pcd_common import load_indices, read_pcd                      # noqa: E402
from bev_panel import (BG, FG, FP, FN, MUTED, PANEL, TP, WINDOW_DISC,   # noqa: E402
                       draw_panel, grey_base, labels, roi_mask, stats)

CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
BAR = {"precision": "#2f6f9f", "recall": "#2da44e", "f1": "#8250df"}

TXT = {
    "en": dict(gt="Ground truth", annotated="annotated",
               scoreboard="P / R / F1",
               legend=["annotated", "TP", "FP", "FN"]),
    "zh": dict(gt="真值标注", annotated="标注",
               scoreboard="P / R / F1",
               legend=["被标注", "TP", "FP", "FN"]),
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pcd", type=pathlib.Path, required=True)
    ap.add_argument("--gt", type=pathlib.Path, required=True)
    ap.add_argument("--series", action="append", default=[],
                    help="Label=indices.txt, repeatable; drawn after the ground-truth panel")
    ap.add_argument("--frame", default="")
    ap.add_argument("--roi", type=float, default=17.0)
    ap.add_argument("--out", type=pathlib.Path, default=None)
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    ap.add_argument("--dpi", type=int, default=170)
    ap.add_argument("--width", type=float, default=11.4)
    ap.add_argument("--height", type=float, default=6.3)
    a = ap.parse_args()
    t = TXT[a.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if a.lang == "zh" else None)
    if a.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    out = a.out or pathlib.Path("docs/figures") / (
        "fig12_baselines_zh.png" if a.lang == "zh" else "fig12_baselines.png")

    pts = read_pcd(a.pcd)
    if not np.isfinite(pts).all(axis=1).all():
        pts = pts[np.isfinite(pts).all(axis=1)]
    base = grey_base(pts)
    roi = roi_mask(pts, a.roi)
    gset = np.zeros(pts.shape[0], dtype=bool)
    idx = load_indices(a.gt)
    gset[idx[idx < pts.shape[0]]] = True

    panels = [("gt", t["gt"], None)]
    for item in a.series:
        label, _, path = item.partition("=")
        snow = np.zeros(pts.shape[0], dtype=bool)
        idx = load_indices(pathlib.Path(path))
        snow[idx[idx < pts.shape[0]]] = True
        panels.append((label, label, snow))

    cols, rows = 4, 2
    need = cols * rows
    if len(panels) > need - 1:
        raise SystemExit(f"{len(panels)} panels do not fit {cols}x{rows} with a scoreboard")

    fig = plt.figure(figsize=(a.width, a.height), dpi=a.dpi, facecolor=BG)
    gs = fig.add_gridspec(rows, cols, left=0.008, right=0.992, top=0.925, bottom=0.080,
                          wspace=0.035, hspace=0.17)
    scores = []
    for k, (label, title, snow) in enumerate(panels):
        ax = fig.add_subplot(gs[k // cols, k % cols])
        if snow is None:
            draw_panel(ax, pts, base, np.zeros(pts.shape[0], dtype=bool),
                       [(roi & gset, TP, 1.8)], a.roi, WINDOW_DISC)
        else:
            m = stats(pts, gset, snow, a.roi)
            scores.append((title, m))
            draw_panel(ax, pts, base, np.zeros(pts.shape[0], dtype=bool),
                       [(roi & snow & gset, TP, 1.8), (roi & snow & ~gset, FP, 2.6),
                        (roi & gset & ~snow, FN, 3.4)], a.roi, WINDOW_DISC)
        pos = ax.get_position()
        fig.text(pos.x0, pos.y1 + 0.010, title, color=FG, fontsize=10.4, fontweight="bold",
                 ha="left", va="bottom", fontproperties=fps)

    ax = fig.add_subplot(gs[1, cols - 1])
    ax.set_facecolor(PANEL)
    x = np.arange(len(scores))
    width = 0.26
    for offset, key in ((-width, "precision"), (0.0, "recall"), (width, "f1")):
        vals = [m[key] for _, m in scores]
        ax.bar(x + offset, vals, width, color=BAR[key], zorder=3)
        if key == "f1":
            for xx, v in zip(x, vals):
                ax.text(xx + offset, v + 2.0, f"{v:.1f}", ha="center", va="bottom",
                        fontsize=7.0, color=FG, fontproperties=fps)
    ax.set_xticks(x)
    ax.set_xticklabels([n for n, _ in scores], fontsize=7.8, color=FG, rotation=28,
                       ha="right", fontproperties=fps)
    ax.set_ylim(0, 118)
    ax.set_yticks([0, 50, 100])
    ax.tick_params(length=0, labelsize=7.6, colors=MUTED)
    ax.grid(axis="y", color="#eaeef2", linewidth=0.8, zorder=1)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#d0d7de")
    ax.set_title(t["scoreboard"], fontsize=9.0, color=MUTED, loc="left", pad=5,
                 fontproperties=fps)

    handles = [plt.Line2D([], [], marker="o", ls="", markersize=5.5, color=c,
                          markeredgecolor="white")
               for c in ("#8c959f", TP, FP, FN)]
    # no title, subtitle or footer: the caption under the figure says all of that, and burned-in
    # prose is the first thing that makes a figure read as cluttered
    fig.legend(handles, t["legend"], loc="upper center", bbox_to_anchor=(0.5, 1.0), ncol=4,
               frameon=False, fontsize=9.4, prop=fps, handletextpad=0.35, columnspacing=2.0)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=a.dpi, facecolor=BG)
    plt.close(fig)
    print(f"{out}")
    for name, m in scores:
        print(f"  {name:12s} P {m['precision']:5.2f}  R {m['recall']:5.2f}  F1 {m['f1']:5.2f}  "
              f"(TP {m['tp']}, FP {m['fp']}, FN {m['fn']})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
