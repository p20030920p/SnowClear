#!/usr/bin/env python3
"""Shared bird's-eye rendering for the README figures.

The animation at the top of the README and the comparison board are drawn by this module, so a
panel of DROR looks like a panel of SnowClear looks like the animation: same palette, same panel
chrome, same grey ramp, same class colours, same ROI definition.

Everything here is presentation plus the one piece of semantics the figures share - which points are
annotated, which are detected, and what that makes them. The numbers a panel prints come from
`stats()`, so a figure cannot disagree with the table it sits next to.
"""

from __future__ import annotations

import numpy as np

BG = "#ffffff"
PANEL = "#f7f9fc"
EDGE = "#c9d3e0"
GREY_LO = (0.55, 0.59, 0.65)     # weak returns: mid grey, readable on white
GREY_HI = (0.11, 0.14, 0.19)     # strong returns: near black
TP = "#1a7f37"
FP = "#cf222e"
FN = "#0968c8"
FG = "#1f2328"
MUTED = "#5b6572"

WINDOW_STRIP = "strip"           # a wide, short view: good for a row of panels
WINDOW_DISC = "disc"             # the whole ROI disc: good for a board


def roi_mask(pts: np.ndarray, radius: float = 17.0) -> np.ndarray:
    """The released ROI gate, as the detector applies it."""
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r3 = np.sqrt(x * x + y * y + z * z)
    elev = np.degrees(np.arcsin(np.clip(z / np.maximum(r3, 1e-9), -1.0, 1.0)))
    return (z >= -1.0) & (z <= 2.6) & (x * x + y * y <= radius ** 2) & (elev >= -23.0)


def grey_base(pts: np.ndarray) -> np.ndarray:
    """Per-point grey from intensity: weak returns stay pale, strong ones go near black."""
    inten = np.clip(pts[:, 3] / max(float(np.percentile(pts[:, 3], 99)), 1.0), 0, 1)
    return np.array(GREY_LO)[None, :] * (1 - inten)[:, None] + \
        np.array(GREY_HI)[None, :] * inten[:, None]


def labels(pts: np.ndarray, gt_idx: np.ndarray, det_idx: np.ndarray):
    """Annotated / detected masks and the four classes they define inside the ROI."""
    gset = np.zeros(pts.shape[0], dtype=bool)
    gset[gt_idx[gt_idx < pts.shape[0]]] = True
    snow = np.zeros(pts.shape[0], dtype=bool)
    snow[det_idx[det_idx < pts.shape[0]]] = True
    return gset, snow


def stats(pts: np.ndarray, gset: np.ndarray, snow: np.ndarray, radius: float = 17.0):
    """Per-frame P / R / F1 inside the ROI - the definition every table in the repository uses."""
    roi = roi_mask(pts, radius)
    tp = int((roi & snow & gset).sum())
    fp = int((roi & snow & ~gset).sum())
    fn = int((roi & gset & ~snow).sum())
    p = 100.0 * tp / max(tp + fp, 1)
    r = 100.0 * tp / max(tp + fn, 1)
    return dict(tp=tp, fp=fp, fn=fn, precision=p, recall=r,
                f1=(2 * p * r / (p + r) if p + r else 0.0))


def draw_panel(ax, pts, base, remove, colour_map, radius=17.0, window=WINDOW_STRIP):
    """One panel: kept returns as a grey cloud, then the coloured subset on top.

    `remove` is the mask taken out of the cloud; `colour_map` is a list of
    `(mask, colour, size)` drawn over what is left.
    """
    keep = ~remove
    ax.scatter(pts[keep, 0], pts[keep, 1], s=0.7, c=base[keep], linewidths=0,
               marker=".", rasterized=True, zorder=2)
    for mask, colour, size in colour_map:
        if mask.any():
            ax.scatter(pts[mask, 0], pts[mask, 1], s=size, c=colour, linewidths=0,
                       marker=".", rasterized=True, zorder=3)
    half_y = radius if window == WINDOW_DISC else radius * 0.40
    ax.set_xlim(-radius, radius)
    ax.set_ylim(-half_y, half_y)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_facecolor(PANEL)
    for side in ax.spines.values():
        side.set_color(EDGE)
        side.set_linewidth(0.8)


def add_columns(fig, axes, names, x0=0.075, width=0.92, y=0.875, fontsize=11.5):
    """Column headers, aligned to the axes rather than guessed."""
    for ax, name in zip(axes, names):
        pos = ax.get_position()
        fig.text(pos.x0 + pos.width / 2, y, name, color=FG, fontsize=fontsize,
                 fontweight="bold", ha="center", va="bottom")
