#!/usr/bin/env python3
"""Animate a scene: raw scan -> per-frame detection -> de-snowed cloud.

One GIF, three panels, a fixed bird's-eye view and one colour grammar, so what the reader sees is
what the detector does over time rather than one flattering frame:

    raw scan      every return, shaded by intensity
    detection     grey = kept, green = flagged and annotated, red = flagged but not annotated
                  (false positive), blue = annotated but not flagged (missed)
    de-snowed     what leaves the pipeline

The per-frame numbers printed in the panel are recomputed from the same index files the
evaluation uses, so the animation cannot disagree with the tables.

    ros2 run snowclear_core snowclear_runner --mode eval_folders \\
        --params src/snowclear_ros/config/snowclear_params.yaml \\
        pcd_root:=<root> result_root:=<mirror>/result save_results:=true output_dir:=<dump>
    python3 tools/render_detection_gif.py --pcd-dir <mirror>/pcd_output/35/velodyne \\
        --gt-dir <mirror>/result/35 --detection-dir <dump>/velodyne \\
        --out docs/figures/detect_scene35.gif --step 3

Keep the GIF small: `--step` thins the frames and `--dpi` sets the panel resolution. GitHub plays
animated GIFs; a static fallback matters, so both READMEs also carry the still figures.
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
PANEL = "#ffffff"
GREY_LO = (0.74, 0.77, 0.81)
GREY_HI = (0.18, 0.21, 0.26)
TP = "#1a7f37"
FP = "#cf222e"
FN = "#0969da"
FG = "#1f2328"
MUTED = "#6e7781"

TXT = {
    "en": dict(panels=["raw scan", "detection", "de-snowed"],
               metrics="frame {f}   P {p:.1f} · R {r:.1f} · F1 {s:.1f}",
               legend=["kept", "TP", "FP", "FN"],
               note="in-ROI, per frame · grey shading = intensity"),
    "zh": dict(panels=["原始点云", "检出结果", "去雪后"],
               metrics="帧 {f}   P {p:.1f} · R {r:.1f} · F1 {s:.1f}",
               legend=["保留", "TP", "FP", "FN"],
               note="ROI 内、逐帧统计 · 灰度表示强度"),
}


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


def draw(ax, pts, roi, snow, gset, panel, shadow):
    """One BEV panel. `shadow` adds the detection colouring on top of the kept points."""
    inten = np.clip(pts[:, 3] / max(float(np.percentile(pts[:, 3], 99)), 1.0), 0, 1)
    base = np.array(GREY_LO)[None, :] * (1 - inten)[:, None] + \
        np.array(GREY_HI)[None, :] * inten[:, None]
    keep = ~snow if shadow else np.ones(pts.shape[0], dtype=bool)
    ax.scatter(pts[keep, 0], pts[keep, 1], s=0.5, c=base[keep], linewidths=0,
               marker=".", rasterized=True, zorder=2)
    if shadow:
        for mask, colour, size in ((roi & snow & gset, TP, 1.6), (roi & snow & ~gset, FP, 2.6),
                                   (roi & gset & ~snow, FN, 4.5)):
            if mask.any():
                ax.scatter(pts[mask, 0], pts[mask, 1], s=size, c=colour, linewidths=0,
                           marker=".", rasterized=True, zorder=3)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pcd-dir", type=pathlib.Path, required=True)
    ap.add_argument("--gt-dir", type=pathlib.Path, required=True)
    ap.add_argument("--detection-dir", type=pathlib.Path, required=True)
    ap.add_argument("--out", type=pathlib.Path, default=pathlib.Path("docs/figures/detect.gif"))
    ap.add_argument("--lang", default="en", choices=["en", "zh"])
    ap.add_argument("--step", type=int, default=3, help="keep every Nth frame")
    ap.add_argument("--limit", type=int, default=40, help="maximum frames in the GIF")
    ap.add_argument("--roi", type=float, default=17.0)
    ap.add_argument("--ms", type=int, default=180, help="per-frame duration in the GIF")
    ap.add_argument("--dpi", type=int, default=95)
    ap.add_argument("--width", type=float, default=11.4, help="figure width, inches")
    ap.add_argument("--height", type=float, default=4.0, help="figure height, inches")
    a = ap.parse_args()
    t = TXT[a.lang]
    matplotlib.rcParams["font.sans-serif"] = (["Noto Sans CJK JP", "DejaVu Sans"]
                                              if a.lang == "zh" else ["DejaVu Sans"])

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
        tp = int((roi & snow & gset).sum())
        fp = int((roi & snow & ~gset).sum())
        fn = int((roi & gset & ~snow).sum())
        prec = 100.0 * tp / max(tp + fp, 1)
        rec = 100.0 * tp / max(tp + fn, 1)
        f1 = 2 * prec * rec / (prec + rec) if prec + rec else 0.0

        fig, axes = plt.subplots(1, 3, figsize=(a.width, a.height), dpi=a.dpi, facecolor=BG)
        fig.subplots_adjust(left=0.006, right=0.994, top=0.845, bottom=0.075, wspace=0.015)
        for ax, kind in zip(axes, ("raw", "detection", "desnowed")):
            ax.set_facecolor(PANEL)
            draw(ax, pts, roi, snow, gset, kind, shadow=(kind == "detection"))
            ax.set_xlim(-a.roi, a.roi)
            ax.set_ylim(-a.roi * 0.37, a.roi * 0.37)
            ax.set_aspect("equal", adjustable="datalim")
            ax.set_xticks([])
            ax.set_yticks([])
            for side in ax.spines.values():
                side.set_color("#d0d7de")
        fig.text(0.02, 0.945, t["panels"][0], color=FG, fontsize=10.5, va="top")
        fig.text(0.35, 0.945, t["panels"][1], color=FG, fontsize=10.5, va="top")
        fig.text(0.68, 0.945, t["panels"][2], color=FG, fontsize=10.5, va="top")
        fig.text(0.5, 0.995, t["metrics"].format(f=frame.stem, p=prec, r=rec, s=f1),
                 color=MUTED, fontsize=9.5, ha="center", va="top")
        fig.text(0.5, 0.022, t["note"], color="#6e7781", fontsize=8.0, ha="center")
        buf = io.BytesIO()
        fig.savefig(buf, format="png", facecolor=BG)
        plt.close(fig)
        buf.seek(0)
        images.append(Image.open(buf).convert("RGB"))
        if (n + 1) % 8 == 0:
            print(f"  rendered {n + 1}/{len(frames)}", flush=True)

    if not images:
        raise SystemExit("nothing rendered")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    # one shared palette keeps the GIF small and the colours stable frame to frame
    quantised = [im.quantize(colors=96, method=Image.MEDIANCUT) for im in images]
    quantised[0].save(a.out, save_all=True, append_images=quantised[1:], duration=a.ms,
                      loop=0, optimize=True, disposal=2)
    print(f"{a.out}  {len(images)} frames, {a.out.stat().st_size / 1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
