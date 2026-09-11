#!/usr/bin/env python3
"""Shared readers for the audit and figure tooling.

Kept in one place so the audit script and the qualitative renderer cannot drift in
how they read a PCD or an index file — the two must agree bit for bit for a figure's
metrics to match the tables.

Not a library: each tool imports it by path (`sys.path` insert), matching the way
`live_check.py` imports `scan_to_cloud.py`.
"""

from __future__ import annotations

import pathlib

import numpy as np


def read_pcd(path: pathlib.Path) -> np.ndarray:
    """(N, 4) float32 `x y z intensity`, from an ascii or binary PCD v0.7 file."""
    with open(path, "rb") as fh:
        header: dict[str, list[str]] = {}
        while True:
            raw = fh.readline()
            if not raw:
                raise ValueError(f"{path}: truncated header")
            line = raw.decode("ascii", "ignore").strip()
            if not line or line.startswith("#"):
                continue
            key, *rest = line.split()
            header[key] = rest
            if key == "DATA":
                break
        n = int(header["POINTS"][0])
        if header["DATA"][0] == "binary":
            return np.ascontiguousarray(
                np.frombuffer(fh.read(n * 16), dtype=np.float32).reshape(-1, 4))
        return np.ascontiguousarray(np.loadtxt(fh, dtype=np.float32, ndmin=2)[:, :4])


def load_indices(path: pathlib.Path) -> np.ndarray:
    """`index` or `index,class` per line, one per row; negatives dropped.

    Used for both ground truth and saved detection output — the two share a format
    on purpose (see docs/MIGRATION_ROS1.md section 4).
    """
    out = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        head = line.split(",")[0].strip()
        try:
            v = int(head)
        except ValueError:
            continue
        if v >= 0:
            out.append(v)
    return np.unique(np.asarray(out, dtype=np.int64))


# Alias with the ground-truth-specific name, so call sites read clearly.
load_gt = load_indices


def confusions(detected: np.ndarray, gt: np.ndarray):
    """TP/FP/FN index arrays plus precision, recall, F1 (0.0 on a zero denominator)."""
    det = np.unique(detected)
    g = np.unique(gt)
    tp = np.intersect1d(det, g, assume_unique=True)
    fp = np.setdiff1d(det, g, assume_unique=True)
    fn = np.setdiff1d(g, det, assume_unique=True)
    p = tp.size / det.size if det.size else 0.0
    r = tp.size / g.size if g.size else 0.0
    f1 = 2 * p * r / (p + r) if (p + r) > 0 else 0.0
    return tp, fp, fn, p, r, f1
