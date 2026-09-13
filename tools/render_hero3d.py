#!/usr/bin/env python3
"""Render the README hero: the detection outcome in true 3D perspective.

A live RViz screenshot is the honest way to show "this runs in ROS", and
`capture_rviz_screenshot.sh` keeps that path reproducible. It is a poor *figure*
though: RViz draws near-constant-size squares, so a 208 k-point scan reads as a spray
of dots and the three outcome classes vanish into it.

This renders the same four classes through a perspective camera we control:

    structure   ROI points that are neither annotated snow nor detected (intensity
                ramp, monochrome — no rainbow), so the scene keeps a surface to sit on
    TP / FN / FP  flat green / blue / red, size-attenuated, painted far-to-near

plus an RViz-style ground grid (the strongest 3D cue there is), aerial perspective so
depth survives without fogging the classes away, and leader lines that point at the
clusters the caption talks about.

    python3 tools/render_hero3d.py --pcd 042126.pcd \\
        --detection testdata/reference_042126.txt --gt $DATA/result/35/042126.txt \\
        --out docs/figures/fig0_hero.png
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from pcd_common import read_pcd, load_indices   # noqa: E402

CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
BG = "#ffffff"
FLOOR = -1.0                      # the ROI floor: the grid lives here
GRID_MINOR = "#eef1f6"
GRID_MAJOR = "#d5dde8"
STRUCT_LO = (0.80, 0.83, 0.88)    # I = 0    weak returns, pale on white
STRUCT_HI = (0.20, 0.24, 0.30)    # I >= 2   strong returns, dark on white
TP_C = "#1a7f37"
FN_C = "#0969da"
FP_C = "#cf222e"

TXT = {
    "en": dict(structure="Structure (ROI)", tp="Detected snow — TP",
               fn="Missed snow — FN", fp="False positives — FP",
               metrics="in-ROI   precision {p:.1f} %   recall {r:.1f} %",
               foot="frame {f} · {n} points · ROI ≤ {roi:.0f} m",
               fp_note="{n} false alarms", fn_note="{n} missed"),
    "zh": dict(structure="场景点（ROI 内）", tp="检出雪点 — TP",
               fn="漏检雪点 — FN", fp="误检点 — FP",
               metrics="ROI 内   精确率 {p:.1f} %   召回率 {r:.1f} %",
               foot="帧 {f} · {n} 点 · ROI ≤ {roi:.0f} m",
               fp_note="{n} 个误检", fn_note="{n} 个漏检"),
}


def camera(azim_deg: float, elev_deg: float):
    """World-from-camera frame (right, up, forward); camera sits at target - d*fwd."""
    a, e = math.radians(azim_deg), math.radians(elev_deg)
    fwd = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    fwd = -fwd
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right) + 1e-12
    up = np.cross(right, fwd)
    return right, up, fwd


def project(P, target, frame, dist, focal):
    """Perspective projection: screen x/y plus the depth along the view axis."""
    right, up, fwd = frame
    v = P - target
    z = dist + v @ fwd
    return (v @ right) * focal / z, (v @ up) * focal / z, z


def grid(x0, x1, y0, y1, step=1.0, major=5.0):
    """Ground-grid segments on the ROI floor, major lines every `major` metres."""
    segs, cols = [], []
    for axis, lo, hi in ((0, x0, x1), (1, y0, y1)):
        for t in np.arange(math.ceil(lo / step) * step, hi + 1e-9, step):
            p0 = [0.0, 0.0, FLOOR]
            p1 = [0.0, 0.0, FLOOR]
            p0[axis], p1[axis] = t, t
            p0[1 - axis], p1[1 - axis] = lo, hi
            segs.append([p0, p1])
            cols.append(GRID_MAJOR if abs(t / major - round(t / major)) < 1e-6
                        else GRID_MINOR)
    return np.asarray(segs), cols


def densest_cell(sx, sy, view, bins=18):
    """Screen-space densest cell — where a leader line should point."""
    h, xe, ye = np.histogram2d(
        sx, sy, bins=bins,
        range=[[view[0], view[1]], [view[2], view[3]]])
    i, j = np.unravel_index(np.argmax(h), h.shape)
    m = ((sx >= xe[i]) & (sx <= xe[i + 1]) & (sy >= ye[j]) & (sy <= ye[j + 1]))
    if not m.any():
        return float(np.mean(sx)), float(np.mean(sy))
    return float(np.median(sx[m])), float(np.median(sy[m]))


def draw_scene(ax, pts, cls, target, frame, dist, focal, view, fog) -> None:
    """Ground grid, intensity-ramped structure, then the three outcome classes.

    Split out of `render` so the baseline comparison draws every panel with exactly
    the code that draws the hero: same camera, same palette, same point sizes. Panels
    then differ only in which points each method filed under tp / fn / fp.
    """
    _, _, z = project(pts[cls["structure"] | cls["tp"] | cls["fn"] | cls["fp"]][:, :3],
                      target, frame, dist, focal)
    zlo, zhi = float(z.min()), float(z.max())
    span = max(zhi - zlo, 1e-6)

    # ---- ground grid ----------------------------------------------------
    gseg, gcol = grid(view[0] * 1.4, view[1] * 1.4, view[2] * 1.4, view[3] * 1.4)
    gx0, gy0, gz0 = project(gseg[:, 0, :], target, frame, dist, focal)
    gx1, gy1, gz1 = project(gseg[:, 1, :], target, frame, dist, focal)
    lines = np.stack([np.stack([gx0, gy0], axis=1),
                      np.stack([gx1, gy1], axis=1)], axis=1)
    zn = np.clip((0.5 * (gz0 + gz1) - zlo) / span, 0.0, 1.0)
    alpha = np.clip(0.95 - 0.62 * zn, 0.12, 0.95)
    rgb = np.array([matplotlib.colors.to_rgb(c) for c in gcol])
    ax.add_collection(LineCollection(lines, colors=np.c_[rgb, alpha],
                                     linewidths=0.7, zorder=1))

    # ---- structure: intensity ramp, monochrome --------------------------
    m = cls["structure"]
    px, py, pz = project(pts[m][:, :3], target, frame, dist, focal)
    inn = np.clip(pts[m][:, 3] / max(float(np.percentile(pts[:, 3], 99)), 1.0), 0, 1)
    base = np.array(STRUCT_LO)[None, :] * (1 - inn)[:, None] + \
        np.array(STRUCT_HI)[None, :] * inn[:, None]
    depth = np.clip((pz - zlo) / span, 0, 1)
    shade = fog * depth
    bgc = np.array(matplotlib.colors.to_rgb(BG))[None, :]
    rgb = base * (1 - shade)[:, None] + bgc * shade[:, None]
    o = np.argsort(-pz)
    ax.scatter(px[o], py[o], s=np.clip(3.4 * (dist / pz[o]) ** 2, 0.25, 6.0),
               c=rgb[o], linewidths=0, marker="o", rasterized=True, zorder=2)

    # ---- the three outcome classes --------------------------------------
    # a hairline of background keeps neighbouring flakes apart, so a class reads as
    # particles instead of one flat mass
    for key, colour, size, order in (("tp", TP_C, 4.4, 3), ("fp", FP_C, 5.2, 4),
                                     ("fn", FN_C, 13.5, 5)):
        mm = cls[key]
        if not mm.any():
            continue
        qx, qy, qz = project(pts[mm][:, :3], target, frame, dist, focal)
        o = np.argsort(-qz)
        ax.scatter(qx[o], qy[o],
                   s=np.clip(size * (dist / qz[o]) ** 2, 1.1, 26.0),
                   c=colour, linewidths=0.25, edgecolors=BG, marker="o",
                   rasterized=True, zorder=order)


def render(pts, cls, col, args) -> pathlib.Path:
    t = TXT[args.lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK)
           if args.lang == "zh" else None)
    if args.lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]

    hit = cls["tp"] | cls["fn"] | cls["fp"]              # where the action is
    span = max(float(np.ptp(pts[hit, 0])), float(np.ptp(pts[hit, 1])))
    target = np.array([float(np.median(pts[hit, 0])), float(np.median(pts[hit, 1])),
                       float(np.percentile(pts[hit, 2], 60))])
    dist = args.dist if args.dist > 0 else 2.05 * max(span, 6.0)
    focal = 1.30
    frame = camera(args.azim, args.elev)

    # frame the scene on the ROI structure: the classes then land wherever the scan
    # actually puts them, which is the point of the figure
    keep = cls["structure"] | hit
    sx, sy, z = project(pts[keep][:, :3], target, frame, dist, focal)
    x0, x1 = np.percentile(sx, [0.4, 99.6])
    y0, y1 = np.percentile(sy, [0.4, 99.6])
    mx, my = 0.045 * (x1 - x0), 0.045 * (y1 - y0)
    view = [x0 - mx, x1 + mx, y0 - my, y1 + my]

    fig = plt.figure(figsize=(args.width / args.dpi, args.height / args.dpi),
                     dpi=args.dpi)
    fig.patch.set_facecolor(BG)
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
    ax.set_facecolor(BG)
    # match the data window to the pixel aspect so the equal-aspect scatter is not
    # letterboxed: the axes fill the whole canvas here, so it is the canvas ratio
    want = args.width / args.height
    have = (view[1] - view[0]) / (view[3] - view[2])
    if have < want:                                   # too tall: widen
        grow = 0.5 * ((view[3] - view[2]) * want - (view[1] - view[0]))
        view[0] -= grow; view[1] += grow
    else:                                             # too wide: heighten
        grow = 0.5 * ((view[1] - view[0]) / want - (view[3] - view[2]))
        view[2] -= grow; view[3] += grow

    draw_scene(ax, pts, cls, target, frame, dist, focal, view, args.fog)

    n = {k: int(v.sum()) for k, v in cls.items()}
    prec = 100.0 * n["tp"] / max(n["tp"] + n["fp"], 1)
    rec = 100.0 * n["tp"] / max(n["tp"] + n["fn"], 1)

    # ---- leaders: name the clusters the caption talks about -------------
    # anchored to the densest screen cell of each class, labelled right next to it:
    # a leader that crosses half the figure reads as a scratch, not an annotation
    for key in ("fp", "fn"):
        mm = cls[key]
        if not mm.any():
            continue
        qx, qy, _ = project(pts[mm][:, :3], target, frame, dist, focal)
        cx, cy = densest_cell(qx, qy, view)
        fx = (cx - view[0]) / (view[1] - view[0])
        fy = (cy - view[2]) / (view[3] - view[2])
        up = fy < 0.30
        side = 0.055 if key == "fp" else -0.055     # keep the label off the busiest cell
        tx = float(np.clip(fx + side, 0.06, 0.88))
        ty = fy + 0.075 if up else fy - 0.075
        ax.annotate("", xy=(cx, cy), xytext=(tx, ty), textcoords="axes fraction",
                    arrowprops=dict(arrowstyle="-", color="#8c959f", linewidth=1.0,
                                    shrinkA=3, shrinkB=4, connectionstyle="arc3,rad=0.0"),
                    zorder=9)
        ax.text(tx, ty + (0.018 if up else -0.018), t[f"{key}_note"].format(n=n[key]),
                transform=ax.transAxes, fontsize=9.6, ha="center",
                va="bottom" if up else "top", color="#1f2328", fontproperties=fps,
                zorder=9,
                bbox=dict(boxstyle="round,pad=0.30", facecolor="#ffffff",
                          edgecolor="#d0d7de", linewidth=0.8, alpha=0.95))

    # ---- legend + numbers ----------------------------------------------
    order = (("structure", "#6b7280"), ("tp", TP_C), ("fp", FP_C), ("fn", FN_C))
    handles = [Line2D([], [], marker="o", ls="", markersize=6.2, color=c,
                      markeredgecolor=BG, label=f"{t[k]}  ({n[k]:,})".replace(",", " "))
               for k, c in order]
    leg = ax.legend(handles=handles, loc="upper left", fontsize=9.2,
                    framealpha=0.95, facecolor="#ffffff", edgecolor="#d0d7de",
                    borderpad=0.6, labelspacing=0.45, handletextpad=0.5,
                    prop=fps, bbox_to_anchor=(0.018, 0.975))
    leg.set_zorder(10)
    ax.text(0.022, 0.055, t["metrics"].format(p=prec, r=rec), transform=ax.transAxes,
            fontsize=10.0, color="#1f2328", fontproperties=fps, zorder=10)
    ax.text(0.978, 0.045, t["foot"].format(f=args.frame, n=pts.shape[0], roi=args.roi),
            transform=ax.transAxes, fontsize=8.4, color="#6e7781", ha="right",
            fontproperties=fps, zorder=10)

    ax.set_xlim(view[0], view[1]); ax.set_ylim(view[2], view[3])
    ax.set_aspect("equal", adjustable="box")
    ax.set_axis_off()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=args.dpi, facecolor=BG)
    plt.close(fig)
    print(f"{args.out}  TP {n['tp']}  FN {n['fn']}  FP {n['fp']}  "
          f"structure {n['structure']}  P {prec:.2f} %  R {rec:.2f} %")
    return args.out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pcd", type=pathlib.Path, required=True)
    ap.add_argument("--detection", type=pathlib.Path, required=True)
    ap.add_argument("--gt", type=pathlib.Path, required=True)
    ap.add_argument("--out", type=pathlib.Path,
                    default=pathlib.Path("docs/figures/fig0_hero.png"))
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    ap.add_argument("--frame", default="", help="frame id for the footer")
    ap.add_argument("--roi", type=float, default=17.0)
    ap.add_argument("--azim", type=float, default=-58.0, help="camera azimuth, degrees")
    ap.add_argument("--elev", type=float, default=24.0, help="camera elevation, degrees")
    ap.add_argument("--dist", type=float, default=0.0, help="camera distance (0 = auto)")
    ap.add_argument("--fog", type=float, default=0.55, help="aerial perspective strength")
    ap.add_argument("--dpi", type=int, default=160)
    ap.add_argument("--width", type=int, default=1600)
    ap.add_argument("--height", type=int, default=900)
    a = ap.parse_args()

    pts = read_pcd(a.pcd)
    if not np.isfinite(pts).all(axis=1).all():
        pts = pts[np.isfinite(pts).all(axis=1)]
    snow = np.zeros(pts.shape[0], dtype=bool)
    det = load_indices(a.detection)
    snow[det[det < pts.shape[0]]] = True
    gset = np.zeros(pts.shape[0], dtype=bool)
    gt = load_indices(a.gt)
    gset[gt[gt < pts.shape[0]]] = True

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r3 = np.sqrt(x * x + y * y + z * z)
    elev = np.degrees(np.arcsin(np.clip(z / np.maximum(r3, 1e-9), -1.0, 1.0)))
    roi = (z >= -1.0) & (z <= 2.6) & (x * x + y * y <= a.roi ** 2) & (elev >= -23.0)
    cls = {"structure": roi & ~snow & ~gset, "tp": roi & snow & gset,
           "fn": roi & gset & ~snow, "fp": roi & snow & ~gset}
    render(pts, cls, None, a)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
