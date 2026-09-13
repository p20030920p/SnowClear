#!/usr/bin/env python3
"""Render the before/after pair that opens the README.

Two images, each full scene plus a zoom of the same crop:

    <stem>_before.png    raw scan            | zoom, with the points SnowClear will
                                              remove highlighted in red
    <stem>_after.png     de-snowed scan      | the same zoom, cleaned

The full panels stay neutral grey so they read as "the scene"; the red in the
before-zoom is what makes the effect legible in one glance — on a 200 k-point
bird's-eye view, 3 % of the points removed is otherwise invisible.

    python3 tools/render_hero.py --pcd 042126.pcd --detection /tmp/det/042126.txt \\
        --outdir docs/figures --stem fig0 --lang en

The zoom crop defaults to the densest cluster of removed points, so the panel
always shows something worth looking at; override with --crop x0,x1,y0,y1.
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
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from pcd_common import read_pcd, load_indices   # noqa: E402

CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

RAW = LinearSegmentedColormap.from_list(
    "rawscan", ["#c3cad3", "#9aa2ac", "#5d656e", "#2a2f36"])
# dark card for the README hero: a LiDAR scene reads like a real viewer screenshot
DARKBG = "#ffffff"      # the banner is drawn on white like every other figure
# low intensities must still read as structure on the dark card: the scan rings are
# weak returns, and a near-black ramp makes the "after" panel look empty
DARK = LinearSegmentedColormap.from_list(
    "darkscan", ["#8b96a4", "#b3bdc9", "#d8dfe7", "#f7fafd"])
SNOW = "#ff5a52"
REMOVED = "#d73027"          # the points the method takes out
FRAME = "#b8bec8"

TXT = {
    "en": dict(before_full="(a) Raw scan", after_full="(a) SnowClear output",
               before_zoom="(b) Zoom — red: points SnowClear removes",
               after_zoom="(b) Zoom — the same crop, cleaned",
               removed="removed", kept="kept", roi="ROI {r:.0f} m",
               note="BEV, sensor frame; {n} of {m} points removed ({p:.1f} %)",
               banner_raw="RAW  ·  red = snow returns", banner_after="SNOWCLEAR  ·  de-snowed"),
    "zh": dict(before_full="(a) 原始点云", after_full="(a) SnowClear 输出",
               before_zoom="(b) 局部放大 —— 红色为 SnowClear 剔除的点",
               after_zoom="(b) 局部放大 —— 同一区域，已清理",
               removed="剔除", kept="保留", roi="ROI {r:.0f} m",
               note="BEV，传感器坐标系；{m} 点中剔除 {n} 点（{p:.1f} %）",
               banner_raw="原始  ·  红色为雪点回波", banner_after="SNOWCLEAR  ·  去雪后"),
}


def _style(ax, title, fps, lim=None, box=None):
    if lim is not None:
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
    elif box is not None:
        x0, x1, y0, y1 = box
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color(FRAME)
        s.set_linewidth(0.8)
    ax.set_title(title, fontsize=10.0, pad=5, fontproperties=fps)


def _scale_bar(ax, span, label, fps, frac=0.34):
    x0 = ax.get_xlim()[0] + 0.06 * (ax.get_xlim()[1] - ax.get_xlim()[0])
    y0 = ax.get_ylim()[0] + 0.07 * (ax.get_ylim()[1] - ax.get_ylim()[0])
    ax.plot([x0, x0 + span], [y0, y0], color="#2b3138", linewidth=2.0,
            solid_capstyle="butt", zorder=8)
    ax.text(x0 + span / 2, y0 + 0.02 * (ax.get_ylim()[1] - ax.get_ylim()[0]), label,
            fontsize=7.5, ha="center", va="bottom", color="#2b3138", zorder=8,
            fontproperties=fps)


def _pick_crop(x, y, removed_mask, roi, half=2.6, lo=0.15, hi=0.45, min_total=280):
    """Choose the zoom window by what the reader will actually see in it.

    Scoring individual 2 m cells does not control the composition of an 8 m window,
    so the window itself is scored: slide it over the ROI and keep the one with the
    most removed points whose *overall* removed fraction is moderate. That gives a
    visible surface with a layer of snow on it, which is what makes the before/after
    legible; the densest window is a snow cloud with nothing underneath.
    """
    bin_m = 0.5
    edges = np.arange(-roi, roi + bin_m, bin_m)
    total, _, _ = np.histogram2d(x, y, bins=[edges, edges])
    hit, _, _ = np.histogram2d(x[removed_mask], y[removed_mask], bins=[edges, edges])
    w = max(2, int(round(2 * half / bin_m)))
    n = total.shape[0]

    def window(cum, i, j):
        return (cum[i + w, j + w] - cum[i, j + w] - cum[i + w, j] + cum[i, j])

    z = np.zeros((n + 1, n + 1))
    tc = np.concatenate([z, np.concatenate([z, total], axis=1)], axis=0).cumsum(0).cumsum(1) \
        if False else _prefix(total)
    rc = _prefix(hit)

    best, fallback = None, None
    limit = roi - half * 1.45                      # keep the window inside the circle
    for i in range(n - w):
        for j in range(n - w):
            cx = 0.5 * (edges[i] + edges[i + w])
            cy = 0.5 * (edges[j] + edges[j + w])
            if cx * cx + cy * cy > limit * limit:
                continue
            t, r = window(tc, i, j), window(rc, i, j)
            if t <= 0:
                continue
            if fallback is None or r > fallback[0]:
                fallback = (r, cx, cy)
            if t >= min_total and lo <= r / t <= hi:
                if best is None or r > best[0]:
                    best = (r, cx, cy)
    _, cx, cy = best if best else (fallback or (0, 0.0, 0.0))
    return (cx - half, cx + half, cy - half, cy + half)


def _prefix(grid):
    """2D prefix sums with a zero row/column, so any window is four lookups."""
    n = grid.shape[0]
    out = np.zeros((n + 1, n + 1))
    out[1:, 1:] = grid.cumsum(0).cumsum(1)
    return out


def _scatter(ax, x, y, inten, vmax, size=0.32):
    ax.scatter(x, y, s=size, c=inten, cmap=RAW, vmin=0, vmax=vmax,
               linewidths=0, marker=".", rasterized=True)


def render(stem: str, outdir: pathlib.Path, kind: str, pts, keep, removed, crop,
           lim, roi, lang, dpi, vmax, note) -> pathlib.Path:
    t = TXT[lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if lang == "zh" else None)
    if lang == "zh":
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]

    x, y, inten = pts[:, 0], pts[:, 1], pts[:, 3]
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.4), dpi=dpi,
                             gridspec_kw=dict(wspace=0.05, width_ratios=[0.62, 1.0]))

    # ---- full scene -----------------------------------------------------
    ax = axes[0]
    m = keep if kind == "after" else np.ones(x.size, dtype=bool)
    _scatter(ax, x[m], y[m], inten[m], vmax)
    ax.add_patch(Circle((0, 0), roi, fill=False, edgecolor="#6b7280",
                        linewidth=0.9, linestyle=(0, (5, 3)), zorder=6))
    ax.text(roi * 0.715, roi * 0.715 + 0.02 * lim, t["roi"].format(r=roi), fontsize=7.0,
            color="#4b5563", ha="center", va="bottom", zorder=9, fontproperties=fps)
    _style(ax, t["before_full"] if kind == "before" else t["after_full"], fps, lim=lim)
    _scale_bar(ax, 5.0, "5 m", fps)

    # ---- zoom -----------------------------------------------------------
    ax = axes[1]
    x0, x1, y0, y1 = crop
    inside = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    m = inside & (keep if kind == "after" else np.ones(x.size, dtype=bool))
    _scatter(ax, x[m], y[m], inten[m], vmax, size=1.1)
    if kind == "before":
        rm = inside & removed
        ax.scatter(x[rm], y[rm], s=6.5, c=REMOVED, linewidths=0.25,
                   edgecolors="white", marker=".", rasterized=True, zorder=5)
        ax.legend(handles=[
            Line2D([], [], marker=".", ls="", color=REMOVED, markersize=8,
                   label=f'{t["removed"]} ({int(removed.sum())})'),
            Line2D([], [], marker=".", ls="", color="#8b939d", markersize=8,
                   label=t["kept"]),
        ], loc="lower right", fontsize=7.0, framealpha=0.95, borderpad=0.35,
            handletextpad=0.3, prop=fps)
    _style(ax, t["before_zoom"] if kind == "before" else t["after_zoom"], fps,
           box=crop)
    _scale_bar(ax, 1.0, "1 m", fps)

    # mark the zoom window on the full panel so the two are connected
    axes[0].add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                                edgecolor="#2b3138", linewidth=1.1, zorder=7))
    fig.text(0.5, 0.012, note, ha="center", va="bottom", fontsize=7.2,
             color="#5b616b", fontproperties=fps)
    fig.subplots_adjust(left=0.012, right=0.988, top=0.90, bottom=0.10, wspace=0.045)

    out = outdir / f"{stem}_{kind}.png"
    fig.savefig(out, dpi=dpi, facecolor="white")
    plt.close(fig)
    return out


def render_banner(path: pathlib.Path, pts, kept, removed, crop, lang, dpi, vmax):
    """One wide card: raw zoom with the removed returns in red, then the cleaned zoom."""
    t = TXT[lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if lang == "zh" else None)
    x, y, inten = pts[:, 0], pts[:, 1], pts[:, 3]
    x0, x1, y0, y1 = crop
    inside = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)

    fig, axes = plt.subplots(1, 2, figsize=(6.9, 3.15), dpi=dpi,
                             gridspec_kw=dict(wspace=0.02))
    fig.patch.set_facecolor(DARKBG)
    for ax, kind in zip(axes, ("before", "after")):
        ax.set_facecolor(DARKBG)
        m = inside & (kept if kind == "after" else np.ones(x.size, dtype=bool))
        ax.scatter(x[m], y[m], s=2.2, c=inten[m], cmap=RAW, vmin=0, vmax=vmax,
                   linewidths=0, marker=".", rasterized=True)
        if kind == "before":
            rm = inside & removed
            ax.scatter(x[rm], y[rm], s=9.0, c=SNOW, linewidths=0.3,
                       edgecolors="#0d1117", marker=".", rasterized=True, zorder=5)
            ax.text(0.03, 0.05, t["banner_raw"], transform=ax.transAxes, fontsize=9.5,
                    color="#b3261e", fontproperties=fps, zorder=8)
        else:
            ax.text(0.03, 0.05, t["banner_after"], transform=ax.transAxes, fontsize=9.5,
                    color="#1a7f37", fontproperties=fps, zorder=8)
        ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xticks([]); ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_color("#d0d7de"); sp.set_linewidth(1.0)
    fig.text(0.5, 0.5, "\u2192", fontsize=17, color="#1f2328", ha="center", va="center",
             zorder=10, bbox=dict(boxstyle="circle,pad=0.22", facecolor="#eef3f8",
                                  edgecolor="none"))
    fig.subplots_adjust(left=0.008, right=0.992, top=0.985, bottom=0.015)
    fig.savefig(path, dpi=dpi, facecolor=DARKBG)
    plt.close(fig)


# --------------------------------------------------------------- 3D banner ----
# A perspective camera and a painter's-algorithm scatter, instead of mplot3d: this
# keeps depth cues under our control (size and fog by distance) and stays fast on
# the tens of thousands of points a hero crop contains.
BG3D = "#0a0e14"
KEPT_LO = (0.33, 0.42, 0.53)      # weak returns / ground: cool, dim
KEPT_HI = (0.85, 0.90, 0.96)      # strong returns: near white
SNOW3D = "#ff5a52"


def _camera(azim_deg, elev_deg):
    """World-from-camera rotation: returns (right, up, forward) unit vectors."""
    a, e = math.radians(azim_deg), math.radians(elev_deg)
    fwd = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    fwd = -fwd                                    # camera looks from +offset to target
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, world_up)
    right /= np.linalg.norm(right) + 1e-12
    up = np.cross(right, fwd)
    return right, up, fwd


def _project(P, target, right, up, fwd, dist, focal):
    v = P - target
    z = dist + v @ fwd                            # depth along the view axis
    x = v @ right
    y = v @ up
    return x * focal / z, y * focal / z, z


def render_banner3d(path, pts, kept, removed, crop, lang, dpi, azim, elev, fog):
    t = TXT[lang]
    fps = (matplotlib.font_manager.FontProperties(fname=CJK) if lang == "zh" else None)
    x0, x1, y0, y1 = crop
    inside = (pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1)
    P = np.ascontiguousarray(pts[inside][:, :3])
    I = pts[inside][:, 3]
    R = removed[inside]
    K = ~R
    span = max(x1 - x0, y1 - y0)
    target = np.array([0.5 * (x0 + x1), 0.5 * (y0 + y1), float(np.median(P[:, 2]))])
    dist = 1.9 * span
    focal = 1.35
    right, up, fwd = _camera(azim, elev)

    # one viewport for both panels: they must stay comparable, so the bounds come
    # from the union of the two projections rather than per-panel autoscaling
    allx, ally, allz = _project(P, target, right, up, fwd, dist, focal)
    mx = 0.06 * (allx.max() - allx.min())
    my = 0.06 * (ally.max() - ally.min())
    view = (allx.min() - mx, allx.max() + mx, ally.min() - my, ally.max() + my)

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.9), dpi=dpi,
                             gridspec_kw=dict(wspace=0.025))
    fig.patch.set_facecolor(BG3D)
    for ax, kind in zip(axes, ("before", "after")):
        ax.set_facecolor(BG3D)
        m = K if kind == "after" else np.ones(P.shape[0], dtype=bool)
        sx, sy, z = _project(P[m], target, right, up, fwd, dist, focal)
        zmin, zmax = float(z.min()), float(z.max())
        zn = np.clip((z - zmin) / max(zmax - zmin, 1e-6), 0.0, 1.0)
        base = np.array(KEPT_LO)[None, :] * (1 - zn)[:, None] + \
               np.array(KEPT_HI)[None, :] * zn[:, None]
        # aerial perspective: pull distant points toward the background
        bg = np.array(matplotlib.colors.to_rgb(BG3D))[None, :]
        rgb = base * (1 - fog * zn)[:, None] + bg * (fog * zn)[:, None]
        order = np.argsort(-z)                    # far first
        ax.scatter(sx[order], sy[order], s=(0.9 + 3.6 * (1 - zn[order])),
                   c=rgb[order], linewidths=0, marker=".", rasterized=True)
        if kind == "before":
            sr = R
            rx, ry, rz = _project(P[sr], target, right, up, fwd, dist, focal)
            rzn = np.clip((rz - zmin) / max(zmax - zmin, 1e-6), 0.0, 1.0)
            ro = np.argsort(-rz)
            # a hairline of background colour keeps neighbouring flakes apart, so the
            # cloud reads as particles instead of one red mass
            ax.scatter(rx[ro], ry[ro], s=(2.2 + 7.0 * (1 - rzn[ro])), c=SNOW3D,
                       linewidths=0.18, edgecolors=BG3D, marker=".", rasterized=True,
                       zorder=5)
            ax.text(0.03, 0.05, t["banner_raw"], transform=ax.transAxes, fontsize=9.5,
                    color="#b3261e", fontproperties=fps, zorder=8)
        else:
            ax.text(0.03, 0.05, t["banner_after"], transform=ax.transAxes, fontsize=9.5,
                    color="#1a7f37", fontproperties=fps, zorder=8)
        ax.set_xlim(view[0], view[1]); ax.set_ylim(view[2], view[3])
        ax.set_aspect("equal", adjustable="box")
        ax.set_box_aspect((view[3] - view[2]) / (view[1] - view[0]))
        ax.set_xticks([]); ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_color("#232a33"); sp.set_linewidth(1.0)
    fig.text(0.5, 0.5, "\u2192", fontsize=17, color=BG3D, ha="center", va="center",
             zorder=10, bbox=dict(boxstyle="circle,pad=0.22", facecolor="#eef3f8",
                                  edgecolor="none"))
    fig.subplots_adjust(left=0.006, right=0.994, top=0.988, bottom=0.012)
    fig.savefig(path, dpi=dpi, facecolor=BG3D)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pcd", type=pathlib.Path, required=True)
    ap.add_argument("--detection", type=pathlib.Path, required=True,
                    help="saved snow indices = the points the method removes")
    ap.add_argument("--outdir", type=pathlib.Path, default=pathlib.Path("docs/figures"))
    ap.add_argument("--stem", default="fig0",
                    help='file stem; "--lang zh" appends "_zh" unless --stem is given')
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    ap.add_argument("--roi", type=float, default=17.0)
    ap.add_argument("--crop", default="auto", help='"auto" or "x0,x1,y0,y1"')
    ap.add_argument("--dpi", type=int, default=200)
    ap.add_argument("--banner", action="store_true",
                    help="also write <stem>_banner.png: one dark before/after card")
    ap.add_argument("--banner3d", action="store_true",
                    help="write <stem>_banner3d.png: perspective 3D before/after card")
    ap.add_argument("--half", type=float, default=2.6, help="zoom half-width, metres")
    ap.add_argument("--azim", type=float, default=-60.0, help="3D camera azimuth, degrees")
    ap.add_argument("--elev", type=float, default=22.0, help="3D camera elevation, degrees")
    ap.add_argument("--fog", type=float, default=0.72, help="aerial perspective strength")
    args = ap.parse_args()
    stem = args.stem if (args.stem != "fig0" or args.lang == "en") else "fig0"
    if args.lang == "zh" and not stem.endswith("_zh"):
        stem += "_zh"

    pts = read_pcd(args.pcd)
    finite = np.isfinite(pts).all(axis=1)
    if not finite.all():
        print(f"warning: dropping {(~finite).sum()} non-finite points", file=sys.stderr)
        pts = pts[finite]
    det = load_indices(args.detection)
    det = det[det < pts.shape[0]]
    removed = np.zeros(pts.shape[0], dtype=bool)
    removed[det] = True
    kept = ~removed
    x, y = pts[:, 0], pts[:, 1]

    if args.crop == "auto":
        crop = _pick_crop(x, y, removed & (np.hypot(x, y) <= args.roi), args.roi,
                          half=args.half)
    else:
        x0, x1, y0, y1 = (float(v) for v in args.crop.split(","))
        crop = (x0, x1, y0, y1)

    vmax = max(1.0, float(np.percentile(pts[:, 3], 99)))
    note = TXT[args.lang]["note"].format(
        n=int(removed.sum()), m=int(pts.shape[0]),
        p=100.0 * removed.sum() / max(pts.shape[0], 1))
    args.outdir.mkdir(parents=True, exist_ok=True)
    for kind in ("before", "after"):
        out = render(stem, args.outdir, kind, pts, kept, removed, crop,
                     args.roi, args.roi, args.lang, args.dpi, vmax, note)
        in_crop = ((x >= crop[0]) & (x <= crop[1]) & (y >= crop[2]) & (y <= crop[3]))
        print(f"{out}  zoom x {crop[0]:.1f}..{crop[1]:.1f} m: "
              f"{int((in_crop & removed).sum())} removed / {int(in_crop.sum())} points "
              f"({100.0 * (in_crop & removed).sum() / max(int(in_crop.sum()), 1):.1f} %)")
    if args.banner:
        b = args.outdir / f"{stem}_banner.png"
        render_banner(b, pts, kept, removed, crop, args.lang, args.dpi, vmax)
        print(f"{b}")
    if args.banner3d:
        b = args.outdir / f"{stem}_banner3d.png"
        render_banner3d(b, pts, kept, removed, crop, args.lang, args.dpi,
                        args.azim, args.elev, args.fog)
        print(f"{b}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
