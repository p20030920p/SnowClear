#!/usr/bin/env python3
"""Render qualitative comparison figures for a LiDAR scan.

Produces the panel sequence a snow-removal paper needs:

    (a) raw scan                — BEV, intensity-shaded, no labels
    (b) ground truth            — the annotated snow points, everything else muted
    (c) detection outcome       — TP / FN / FP against that ground truth, with the
                                  per-frame precision, recall and F1 called out in
                                  the panel and leader lines at the densest error
                                  clusters

Optional fourth panel (`--detection2`) renders a second method on the same frame,
which is what a comparison figure needs; the layout then becomes 2x2.

Examples
    # the reference frame shipped with the repository
    python3 tools/render_qualitative.py \\
        --pcd  $SNOWCLEAR_DATA/pcd_output/35/velodyne/042126.pcd \\
        --gt   $SNOWCLEAR_DATA/result/35/042126.txt \\
        --detection /tmp/sc_verify/042126.txt \\
        --out docs/figures/fig2_qualitative.png

    # SnowClear against a non-learned baseline
    python3 tools/render_qualitative.py --pcd frame.pcd --gt gt.txt \\
        --detection snowclear.txt --label "SnowClear (released)" \\
        --detection2 sor.txt    --label2 "SOR (PCL)" \\
        --lang en --out comparison.png

Producing the detection files is one command each:

    ros2 run snowclear_core snowclear_cli pcd_file:=frame.pcd gt_folder:=<gt dir> \\
        save_results:=true output_dir:=/tmp/det detector_type:=sor

Index space: ground truth and detection files are both "index into the scan", so
they are directly comparable. If the scan contains non-finite points the offline
path removes them first and indices shift; the tool warns when it sees one.
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from pcd_common import read_pcd, load_indices, confusions   # noqa: E402

CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

BG = "#c9ced6"       # everything that is neither snow nor detected
RAW = LinearSegmentedColormap.from_list(
    "rawscan", ["#c3cad3", "#9aa2ac", "#5d656e", "#2a2f36"])
GT = "#d73027"       # annotated snow
TP = "#1a9850"       # detected and annotated
FN = "#2c7fb8"       # annotated, missed
FP = "#984ea3"       # detected, not annotated

TXT = {
    "en": dict(raw="(a) Raw scan", gt="(b) Ground truth (snow annotated)",
               out="({k}) {name}", other="other points", snow="annotated snow",
               tp="TP  detected snow", fn="FN  missed snow", fp="FP  false positive",
               missed="missed cluster", false="false positives",
               desnow="de-snowed cloud (result)", metrics="per-frame", scale="5 m"),
    "zh": dict(raw="(a) 原始点云", gt="(b) 真值标注（雪点）",
               out="({k}) {name}", other="其他点", snow="真值雪点",
               tp="TP  正确检出", fn="FN  漏检", fp="FP  误检",
               missed="漏检聚集区", false="误检聚集区",
               desnow="去雪后点云（结果）", metrics="本帧", scale="5 m"),
}


# --------------------------------------------------------------------- drawing
def _scatter(ax, pts, mask, color, size, alpha=1.0, zorder=2, edge=False):
    """One point class. `edge` adds a hairline so rare classes stay visible when
    the figure is scaled down (the README renders these at ~1/3 linear size)."""
    if mask is None or mask.any():
        sel = pts if mask is None else pts[mask]
        ax.scatter(sel[:, 0], sel[:, 1], s=size, c=color, alpha=alpha, marker=".",
                   rasterized=True, zorder=zorder,
                   **({"linewidths": 0.35, "edgecolors": "white"} if edge
                      else {"linewidths": 0}))


def _frame(ax, lim, title, t, fps):
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#b8bec8")
        s.set_linewidth(0.8)
    ax.set_title(title, fontsize=10.5, pad=5, fontproperties=fps)
    ax.set_facecolor("white")


def _metrics_box(ax, tp, fp, fn, p, r, f1, t, fps):
    txt = (f"TP {tp:>6d}   FP {fp:>5d}   FN {fn:>6d}\n"
           f"P {100*p:6.2f}%  R {100*r:6.2f}%  F1 {100*f1:6.2f}%")
    return ax.text(0.025, 0.975, txt, transform=ax.transAxes, va="top", ha="left",
                   fontsize=7.6, family="monospace", linespacing=1.45, zorder=6,
                   bbox=dict(boxstyle="round,pad=0.36", facecolor="white",
                             edgecolor="#9aa1ab", linewidth=0.7, alpha=0.94))


def _densest(xy, lim, bin_m=3.0, min_count=25):
    """Centre of the densest bin_m x bin_m cell, if it holds at least min_count."""
    if xy.shape[0] == 0:
        return None
    edges = np.arange(-lim, lim + bin_m, bin_m)
    h, _, _ = np.histogram2d(xy[:, 0], xy[:, 1], bins=[edges, edges])
    i, j = np.unravel_index(int(np.argmax(h)), h.shape)
    if h[i, j] < min_count:
        return None
    return (0.5 * (edges[i] + edges[i + 1]), 0.5 * (edges[j] + edges[j + 1]), int(h[i, j]))


def _axes_rect(ax, artist, pad=0.012):
    """An artist's bbox in axes fraction, so labels can avoid it exactly."""
    try:
        r = ax.figure.canvas.get_renderer()
        bb = artist.get_window_extent(renderer=r)
        (x0, y0), (x1, y1) = ax.transAxes.inverted().transform(
            [[bb.x0, bb.y0], [bb.x1, bb.y1]])
        return (x0 - pad, y0 - pad, x1 + pad, y1 + pad)
    except Exception:                                  # pragma: no cover
        return None


def _label_spot(cx, cy, lim, reserved=(), w=0.30, h=0.09):
    """Push a label outside the densest cluster, clear of RESERVED regions."""
    d = float(np.hypot(cx, cy)) or 1.0
    tx = float(np.clip(cx * (1.0 + 0.55 * lim / d), -0.86 * lim, 0.86 * lim))
    ty = float(np.clip(cy * (1.0 + 0.55 * lim / d), -0.86 * lim, 0.86 * lim))
    rects = [r for r in reserved if r]
    for _ in range(4):
        xf, yf = (tx / lim + 1.0) / 2.0, (ty / lim + 1.0) / 2.0
        hit = next(((x0, y0, x1, y1) for x0, y0, x1, y1 in rects
                    if xf - w / 2 < x1 and xf + w / 2 > x0
                    and yf - h / 2 < y1 and yf + h / 2 > y0), None)
        if hit is None:
            break
        if hit[2] + w / 2 + 0.02 <= 0.99:              # slide right of the obstacle
            xf = hit[2] + w / 2 + 0.02
        else:                                          # otherwise drop below it
            yf = max(0.05, hit[1] - h / 2 - 0.03)
        tx, ty = xf * 2 * lim - lim, yf * 2 * lim - lim
    return tx, ty


def _callout(ax, cluster, text, color, lim, reserved=()):
    """Label a cluster just outside the data; returns the region it occupies."""
    if cluster is None:
        return None
    cx, cy, n = cluster
    label = f"{text} ({n})"
    # CJK glyphs are roughly twice as wide as Latin ones; count them as two cells
    units = len(label) + sum(1 for ch in label if ord(ch) > 0x2E80)
    w = min(0.62, 0.05 + 0.014 * units)
    tx, ty = _label_spot(cx, cy, lim, reserved, w=w)
    ann = ax.annotate(label, xy=(cx, cy), xytext=(tx, ty),
                      fontsize=8.2, color=color, ha="center", va="center", zorder=9,
                      arrowprops=dict(arrowstyle="-|>", color=color, linewidth=1.1,
                                      shrinkA=1, shrinkB=4,
                                      connectionstyle="arc3,rad=0.12"))
    ann.set_path_effects([pe.withStroke(linewidth=2.6, foreground="white")])
    xf, yf = (tx / lim + 1.0) / 2.0, (ty / lim + 1.0) / 2.0
    return (xf - w / 2, yf - 0.045, xf + w / 2, yf + 0.045)


def _roi_circle(ax, roi, lim, fps):
    """The radius beyond which the detector never looks; only drawn when visible."""
    if not (0 < roi < lim - 0.5):
        return None
    ax.add_patch(Circle((0, 0), roi, fill=False, edgecolor="#6b7280",
                        linewidth=0.9, linestyle=(0, (5, 3)), zorder=6))
    return ax.text(roi * 0.7071, roi * 0.7071 + 0.035 * lim, f"ROI {roi:.0f} m",
                   fontsize=7.0, color="#4b5563", ha="center", va="bottom", zorder=9,
                   fontproperties=fps)


def _scale_bar(ax, lim, label, fps):
    x0, y0, L = -0.92 * lim, -0.92 * lim, 5.0
    ax.plot([x0, x0 + L], [y0, y0], color="#333333", linewidth=2.0, zorder=8,
            solid_capstyle="butt")
    ax.text(x0 + L / 2, y0 + 0.045 * lim, label, fontsize=7.5, ha="center",
            va="bottom", color="#333333", zorder=8, fontproperties=fps)


# ------------------------------------------------------------------------ main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pcd", type=pathlib.Path, required=True)
    ap.add_argument("--gt", type=pathlib.Path)
    ap.add_argument("--detection", type=pathlib.Path)
    ap.add_argument("--detection2", type=pathlib.Path)
    ap.add_argument("--label", default="SnowClear (released configuration)")
    ap.add_argument("--label2", default="baseline")
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    ap.add_argument("--out", type=pathlib.Path, required=True)
    ap.add_argument("--limits", type=float, default=17.0,
                    help="half-extent of the BEV window in metres (default: the ROI)")
    ap.add_argument("--roi", type=float, default=17.0,
                    help="draw this ROI radius; visible when it is inside --limits")
    ap.add_argument("--dpi", type=int, default=200)
    ap.add_argument("--svg", action="store_true", help="also write an SVG with text as paths")
    ap.add_argument("--desnowed", action="store_true",
                    help="append the de-snowed result (input minus everything detected)")
    ap.add_argument("--color", default="intensity", choices=["intensity", "height"])
    ap.add_argument("--title", default="", help="figure title; defaults to the file stem")
    ap.add_argument("--note", default="", help="footer note")
    args = ap.parse_args()

    t = TXT[args.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK)
           if args.lang == "zh" else None)
    if args.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False

    pts = read_pcd(args.pcd)
    finite = np.isfinite(pts).all(axis=1)
    if not finite.all():
        print(f"warning: {args.pcd.name} has {(~finite).sum()} non-finite points; the "
              f"offline path removes them before detecting, so indices shift",
              file=sys.stderr)
        pts = pts[finite]
    x, y, z, inten = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
    lim = args.limits
    inside = (x * x + y * y) <= lim * lim

    gt = load_indices(args.gt) if args.gt else None
    det = load_indices(args.detection) if args.detection else None
    det2 = load_indices(args.detection2) if args.detection2 else None

    # ---- panel plan -----------------------------------------------------
    panels: list[tuple[str, object]] = [("raw", None)]
    if gt is not None:
        panels.append(("gt", gt))
    for name, d in ((args.label, det), (args.label2, det2)):
        if d is not None:
            panels.append(("out", (name, d)))
    if args.desnowed and det is not None:
        panels.append(("desnow", (args.label, det)))
    n = len(panels)
    rows, cols = (1, n) if n <= 3 else ((2, 2) if n == 4 else (2, -(-n // 2)))
    fig, axes = plt.subplots(rows, cols, figsize=(3.25 * cols + 0.15, 3.35 * rows),
                             dpi=args.dpi, squeeze=False)
    flat = [ax for row in axes for ax in row]
    for ax in flat[n:]:
        ax.axis("off")

    # ---- (a) raw --------------------------------------------------------
    ax = flat[0]
    order = np.argsort(inten)
    if args.color == "intensity":
        ax.scatter(x[order], y[order], s=0.32, c=inten[order], cmap=RAW,
                   vmin=0, vmax=max(1.0, float(np.percentile(inten, 99))),
                   linewidths=0, marker=".", rasterized=True)
    else:
        ax.scatter(x[order], y[order], s=0.30, c=z[order], cmap="viridis",
                   linewidths=0, marker=".", rasterized=True)
    _scale_bar(ax, lim, t["scale"], fps)
    _frame(ax, lim, t["raw"], t, fps)

    # ---- (b) ground truth ----------------------------------------------
    if gt is not None:
        ax = flat[1]
        gmask = np.zeros(pts.shape[0], dtype=bool)
        gmask[gt[gt < pts.shape[0]]] = True
        _scatter(ax, pts, inside & ~gmask, BG, 0.22, 0.55, zorder=1)
        _scatter(ax, pts, gmask, GT, 0.85, 1.0, zorder=3)
        ax.legend(handles=[
            Line2D([], [], marker=".", ls="", color=GT, markersize=7, label=t["snow"]),
            Line2D([], [], marker=".", ls="", color=BG, markersize=7, label=t["other"]),
        ], loc="lower right", fontsize=7.5, framealpha=0.95, borderpad=0.4,
            handletextpad=0.35, prop=fps)
        _roi_circle(ax, args.roi, lim, fps)
        _frame(ax, lim, t["gt"], t, fps)

    # ---- (c..) outcomes -------------------------------------------------
    letters = "cdefg"
    for idx, (name, d) in enumerate([p[1] for p in panels if p[0] == "out"]):
        ax = flat[2 + idx]
        roi_artist = _roi_circle(ax, args.roi, lim, fps)
        d = d[d < pts.shape[0]]
        dset = np.zeros(pts.shape[0], dtype=bool)
        dset[d] = True
        if gt is not None:
            tp, fp, fn, p, r, f1 = confusions(d, gt)
            tpm = np.zeros(pts.shape[0], dtype=bool); tpm[tp[tp < pts.shape[0]]] = True
            fpm = np.zeros(pts.shape[0], dtype=bool); fpm[fp[fp < pts.shape[0]]] = True
            fnm = np.zeros(pts.shape[0], dtype=bool); fnm[fn[fn < pts.shape[0]]] = True
            _scatter(ax, pts, inside & ~(tpm | fpm | fnm), BG, 0.20, 0.5, zorder=1)
            _scatter(ax, pts, tpm, TP, 0.85, 1.0, zorder=3)
            _scatter(ax, pts, fpm, FP, 6.00, 1.0, zorder=4, edge=True)
            _scatter(ax, pts, fnm, FN, 5.00, 1.0, zorder=5, edge=True)
            leg = ax.legend(handles=[
                Line2D([], [], marker=".", ls="", color=TP, markersize=8,
                       label=f'{t["tp"]} ({tp.size})'),
                Line2D([], [], marker=".", ls="", color=FN, markersize=8,
                       label=f'{t["fn"]} ({fn.size})'),
                Line2D([], [], marker=".", ls="", color=FP, markersize=8,
                       label=f'{t["fp"]} ({fp.size})'),
            ], loc="lower right", fontsize=7.5, framealpha=0.95, borderpad=0.4,
                handletextpad=0.35, prop=fps)
            box = _metrics_box(ax, tp.size, fp.size, fn.size, p, r, f1, t, fps)
            reserved = [r for r in (_axes_rect(ax, box), _axes_rect(ax, leg),
                                    _axes_rect(ax, roi_artist)) if r]
            for cluster, label, colour in (
                    (_densest(np.column_stack([x[fnm], y[fnm]]), lim), t["missed"], FN),
                    (_densest(np.column_stack([x[fpm], y[fpm]]), lim), t["false"], FP)):
                rect = _callout(ax, cluster, label, colour, lim, reserved)
                if rect:
                    reserved.append(rect)
        else:
            _scatter(ax, pts, inside & ~dset, BG, 0.20, 0.5, zorder=1)
            _scatter(ax, pts, dset, FP, 0.80, 1.0, zorder=3)
        _frame(ax, lim, t["out"].format(k=letters[idx], name=name), t, fps)

    # ---- (d) de-snowed result -------------------------------------------
    if args.desnowed and det is not None:
        ax = flat[2 + len([p for p in panels if p[0] == "out"])]
        d = det[det < pts.shape[0]]
        dset = np.zeros(pts.shape[0], dtype=bool)
        dset[d] = True
        keep = inside & ~dset
        order = np.argsort(inten)
        sel = np.nonzero(keep[order])[0]
        ax.scatter(x[order][sel], y[order][sel], s=0.32, c=inten[order][sel], cmap=RAW,
                   vmin=0, vmax=max(1.0, float(np.percentile(inten, 99))),
                   linewidths=0, marker=".", rasterized=True)
        ax.text(0.025, 0.975, f"-{int(dset.sum())} points",
                transform=ax.transAxes, va="top", ha="left", fontsize=7.6,
                family="monospace", zorder=6,
                bbox=dict(boxstyle="round,pad=0.36", facecolor="white",
                          edgecolor="#9aa1ab", linewidth=0.7, alpha=0.94))
        _roi_circle(ax, args.roi, lim, fps)
        _frame(ax, lim, t["out"].format(k=letters[len([p for p in panels if p[0] == "out"])],
                                        name=t["desnow"]), t, fps)

    title = args.title or args.pcd.stem
    fig.suptitle(title, fontsize=11.5 if len(title) <= 78 else 9.8, y=0.995,
                 fontproperties=fps)
    note = args.note or ("BEV, sensor frame, ROI +/-%.0f m" % lim)
    fig.text(0.5, 0.005, note, ha="center", va="bottom", fontsize=7.5,
             color="#5b616b", fontproperties=fps)
    fig.tight_layout(rect=(0, 0.018, 1, 0.975))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=args.dpi, facecolor="white")
    if args.svg:
        with plt.rc_context({"svg.fonttype": "path"}):
            fig.savefig(args.out.with_suffix(".svg"), format="svg", facecolor="white")
    plt.close(fig)

    if gt is not None and det is not None:
        tp, fp, fn, p, r, f1 = confusions(det, gt)
        print(f"{args.out}  |  TP {tp.size}  FP {fp.size}  FN {fn.size}  "
              f"P {100*p:.2f}%  R {100*r:.2f}%  F1 {100*f1:.2f}%")
    else:
        print(f"{args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
