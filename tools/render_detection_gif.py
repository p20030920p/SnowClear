#!/usr/bin/env python3
"""Animate a scene as six panels: what the detector does, and what the answer is.

    Raw scan | Desnow | Result        <- SnowClear
    Raw scan | Desnow | Result        <- Ground truth

Row one is the pipeline: the raw frame, the cloud after removal, and the detection split into
green TP / red FP / blue FN. Row two is the reference: the same raw frame, the cloud with the
annotated snow taken out, and the annotations themselves highlighted. Reading the two rows
against each other is the point - the honest question about a de-snowing method is not "does the
picture look clean" but "how far is the top row from the bottom row".

The labels are English in both READMEs on purpose: a figure that has to be regenerated per
language is a figure that drifts.

    ros2 run snowclear_core snowclear_runner --mode eval_folders \\
        --params src/snowclear_ros/config/snowclear_params.yaml \\
        pcd_root:=<root> result_root:=<mirror>/result save_results:=true output_dir:=<dump>
    python3 tools/render_detection_gif.py --pcd-dir <mirror>/pcd_output/35/velodyne \\
        --gt-dir <mirror>/result/35 --detection-dir <dump>/velodyne \\
        --out docs/figures/detect_scene35.gif --step 5 --limit 22

`--step` thins the frames and `--dpi` sets the panel resolution; both exist to keep the GIF
inside a few megabytes, which is what GitHub will play smoothly.
"""

from __future__ import annotations

import argparse
import io
import pathlib
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from pcd_common import load_indices, read_pcd   # noqa: E402

BG = "#ffffff"
PANEL = "#f7f9fc"
EDGE = "#c9d3e0"
GREY_LO = (0.55, 0.59, 0.65)     # weak returns: mid grey, still readable on white
GREY_HI = (0.11, 0.14, 0.19)     # strong returns: near black
TP = "#1a7f37"
FP = "#cf222e"
FN = "#0968c8"
FG = "#1f2328"
MUTED = "#5b6572"

COLUMNS = ["Raw scan", "Desnow", "Result"]
ROWS = ["SnowClear", "Ground truth"]
LEGEND = ("grey = returns kept  ·  green = snow flagged and annotated  ·  "
          "red = flagged without annotation  ·  blue = annotated but missed")


def masks(pts, roi_radius, gt_idx, det_idx):
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r3 = np.sqrt(x * x + y * y + z * z)
    elev = np.degrees(np.arcsin(np.clip(z / np.maximum(r3, 1e-9), -1.0, 1.0)))
    roi = (z >= -1.0) & (z <= 2.6) & (x * x + y * y <= roi_radius ** 2) & (elev >= -23.0)
    snow = np.zeros(pts.shape[0], dtype=bool)
    snow[det_idx[det_idx < pts.shape[0]]] = True
    gset = np.zeros(pts.shape[0], dtype=bool)
    gset[gt_idx[gt_idx < pts.shape[0]]] = True
    return roi, snow, gset


def draw(ax, pts, base, remove, colour_map, roi_radius):
    """One panel: the kept returns as a grey cloud, then the coloured subset on top.

    `remove` is the mask taken out of the cloud; `colour_map` decides what is drawn on top and
    in which colour (empty for the raw and de-snowed panels).
    """
    keep = ~remove
    ax.scatter(pts[keep, 0], pts[keep, 1], s=0.7, c=base[keep], linewidths=0,
               marker=".", rasterized=True, zorder=2)
    for mask, colour, size in colour_map:
        if mask.any():
            ax.scatter(pts[mask, 0], pts[mask, 1], s=size, c=colour, linewidths=0,
                       marker=".", rasterized=True, zorder=3)
    ax.set_xlim(-roi_radius, roi_radius)
    ax.set_ylim(-roi_radius * 0.40, roi_radius * 0.40)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_facecolor(PANEL)
    for side in ax.spines.values():
        side.set_color(EDGE)
        side.set_linewidth(0.8)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pcd-dir", type=pathlib.Path, required=True)
    ap.add_argument("--gt-dir", type=pathlib.Path, required=True)
    ap.add_argument("--detection-dir", type=pathlib.Path, required=True)
    ap.add_argument("--out", type=pathlib.Path, default=pathlib.Path("docs/figures/detect.gif"))
    ap.add_argument("--step", type=int, default=5, help="keep every Nth frame")
    ap.add_argument("--limit", type=int, default=22, help="maximum frames in the GIF")
    ap.add_argument("--roi", type=float, default=17.0)
    ap.add_argument("--ms", type=int, default=190, help="per-frame duration in the GIF")
    ap.add_argument("--dpi", type=int, default=92)
    ap.add_argument("--width", type=float, default=10.8, help="figure width, inches")
    ap.add_argument("--height", type=float, default=4.9, help="figure height, inches")
    a = ap.parse_args()
    matplotlib.rcParams["font.sans-serif"] = ["DejaVu Sans"]

    frames = sorted(a.pcd_dir.glob("*.pcd"))[::a.step][:a.limit]
    if not frames:
        raise SystemExit(f"no frames under {a.pcd_dir}")
    print(f"{len(frames)} frames from {a.pcd_dir.name} (step {a.step})")

    images = []
    for n, frame in enumerate(frames):
        det_file = a.detection_dir / f"{frame.stem}.txt"
        gt_file = a.gt_dir / f"{frame.stem}.txt"
        if not det_file.exists() or not gt_file.exists():
            print(f"  {frame.stem}: missing indices, skipped")
            continue
        pts = read_pcd(frame)
        if not np.isfinite(pts).all(axis=1).all():
            pts = pts[np.isfinite(pts).all(axis=1)]
        roi, snow, gset = masks(pts, a.roi, load_indices(gt_file), load_indices(det_file))
        inten = np.clip(pts[:, 3] / max(float(np.percentile(pts[:, 3], 99)), 1.0), 0, 1)
        base = np.array(GREY_LO)[None, :] * (1 - inten)[:, None] + \
            np.array(GREY_HI)[None, :] * inten[:, None]

        tp = int((roi & snow & gset).sum())
        fp = int((roi & snow & ~gset).sum())
        fn = int((roi & gset & ~snow).sum())
        prec = 100.0 * tp / max(tp + fp, 1)
        rec = 100.0 * tp / max(tp + fn, 1)
        f1 = 2 * prec * rec / (prec + rec) if prec + rec else 0.0

        nothing = np.zeros(pts.shape[0], dtype=bool)
        panels = [
            (nothing, []),                                    # SnowClear  raw
            (snow, []),                                       # SnowClear  desnow
            (nothing, [(roi & snow & gset, TP, 2.2),          # SnowClear  result
                       (roi & snow & ~gset, FP, 3.4),
                       (roi & gset & ~snow, FN, 5.5)]),
            (nothing, []),                                    # truth      raw
            (gset, []),                                       # truth      desnow
            (nothing, [(roi & gset, TP, 2.2)]),               # truth      result
        ]

        fig, axes = plt.subplots(2, 3, figsize=(a.width, a.height), dpi=a.dpi, facecolor=BG)
        fig.subplots_adjust(left=0.075, right=0.995, top=0.845, bottom=0.105,
                            wspace=0.03, hspace=0.06)
        for ax, (remove, colour_map) in zip(axes.ravel(), panels):
            draw(ax, pts, base, remove, colour_map, a.roi)
        for col, name in enumerate(COLUMNS):
            fig.text(0.075 + (col + 0.5) * (0.92 / 3), 0.875, name, color=FG, fontsize=11.5,
                     fontweight="bold", ha="center", va="bottom")
        for row, name in enumerate(ROWS):
            fig.text(0.070, 0.72 - row * 0.40, name, color=FG, fontsize=11.5,
                     fontweight="bold", ha="right", va="center", rotation=90)
        fig.text(0.5, 0.985, f"frame {frame.stem}    precision {prec:.1f}  ·  "
                             f"recall {rec:.1f}  ·  F1 {f1:.1f}   (in-ROI)",
                 color=FG, fontsize=10.5, ha="center", va="top")
        fig.text(0.5, 0.022, LEGEND, color=MUTED, fontsize=8.6, ha="center")
        buf = io.BytesIO()
        fig.savefig(buf, format="png", facecolor=BG)
        plt.close(fig)
        buf.seek(0)
        images.append(Image.open(buf).convert("RGB"))
        if (n + 1) % 6 == 0:
            print(f"  rendered {n + 1}/{len(frames)}", flush=True)

    if not images:
        raise SystemExit("nothing rendered")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    quantised = [im.quantize(colors=96, method=Image.MEDIANCUT) for im in images]
    quantised[0].save(a.out, save_all=True, append_images=quantised[1:], duration=a.ms,
                      loop=0, optimize=True, disposal=2)
    print(f"{a.out}  {len(images)} frames, {a.out.stat().st_size / 1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
