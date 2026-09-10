#!/usr/bin/env python3
"""Audit the released configuration against the data, without running the pipeline.

Three independent measurements, all recomputed from the shipped equations rather
than from a run — so they describe the *configuration's* structure, not one
execution of it:

  --mode budget        which stage makes each ground-truth point undetectable
                       (ROI gate / intensity ceiling / surface veto), reported as a
                       per-frame macro average, the protocol the metrics use
  --mode intensities   the intensity distribution of ground truth inside the ROI
  --mode separability  can a simple neighbourhood-density feature separate the
                       ground-truth points above the intensity ceiling from the
                       non-ground-truth ones? (median neighbour counts, both classes)

Usage
    python3 tools/audit_error_budget.py --mode budget --scenes 35 11 14 16
    SNOWCLEAR_DATA=/path/to/mirror python3 tools/audit_error_budget.py --mode intensities

The dataset is not redistributable; see docs/DATASET.md for the expected layout.
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import sys

import numpy as np

# ---------------------------------------------------------------- released config
Z_LO, Z_HI = -1.0, 2.6
XY_MAX = 17.0
ELEV_MIN_DEG = -23.0
SMOOTH_SCALE, SMOOTH_RHO = 0.8, 3.0
SMOOTH_K, SMOOTH_THETA = 2.15, 2.38
SUP_RADIUS, SUP_MIN_I, SUP_RANGE_FLOOR = 0.6, 1.0, 7.0
SCORE_THRESHOLD = 0.75          # theta at its middle branch
HEIGHT_MAX = 0.15               # attainable height contribution of the fused score
S_MIN = (SCORE_THRESHOLD - HEIGHT_MAX) / 0.7      # s > 0.75 is necessary in every branch
IT_MAX = 1.0 - S_MIN ** (1.0 / 1.2)
MAX_I = 255.0
GAMMA_K = math.gamma(SMOOTH_K)


# ------------------------------------------------------------------------ reading
def read_pcd(path: pathlib.Path) -> np.ndarray:
    """(N, 4) float32 x y z intensity, ascii or binary PCD."""
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


def load_gt(path: pathlib.Path) -> np.ndarray:
    """`index` or `index,class` per line; negatives and duplicates dropped."""
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


def roi_mask(x, y, z) -> np.ndarray:
    """filter_by_height_and_distance(), expressed in exact arithmetic."""
    r3 = np.sqrt(x * x + y * y + z * z)
    elev = np.degrees(np.arcsin(np.clip(z / np.maximum(r3, 1e-9), -1.0, 1.0)))
    return ((z >= Z_LO) & (z <= Z_HI) & (x * x + y * y <= XY_MAX ** 2)
            & (elev >= ELEV_MIN_DEG))


def q1_of(intensity: np.ndarray) -> float:
    """256-bin histogram Q1 over the post-ROI cloud, as FeatureExtractor computes it."""
    if intensity.size == 0:
        return 0.0
    hist = np.bincount(np.clip(intensity.astype(np.int64), 0, 255), minlength=256)
    return float(min(int(np.searchsorted(np.cumsum(hist), 0.25 * intensity.size)), 255))


def threshold_of(x, y, intensity, tg) -> np.ndarray:
    """T(r, I) of the released configuration, with Tg already clamped."""
    r = np.sqrt(x * x + y * y)
    xr = np.where(r > 0, r / SMOOTH_THETA, 0.0)
    fr = np.where(r > 0, np.power(np.maximum(xr, 1e-12), SMOOTH_K - 1.0) * np.exp(-xr)
                  / (GAMMA_K * SMOOTH_THETA), 0.0)
    alpha = SMOOTH_RHO * fr / (SMOOTH_RHO * fr + 1.0)
    h = 1.0 - np.minimum(1.0, intensity / MAX_I)
    return np.clip(SMOOTH_SCALE * tg * (1.0 - alpha * h), 2.0, 20.0)


def _grid(x, y, z, cell, subset=None):
    """Spatial hash keyed by cell; `subset` restricts which points get indexed."""
    idx = np.arange(x.size) if subset is None else np.asarray(subset)
    keys = np.stack([np.floor(x[idx] / cell), np.floor(y[idx] / cell),
                     np.floor(z[idx] / cell)], axis=1).astype(np.int64)
    grid: dict[tuple[int, int, int], list[int]] = {}
    for i, k in zip(idx, map(tuple, keys)):
        grid.setdefault(k, []).append(int(i))
    return grid, keys


def vetoed(x, y, z, intensity, query: np.ndarray) -> np.ndarray:
    """Zero-intensity surface suppression, evaluated for the points in `query`.

    The support set is `I > SUP_MIN_I` — not the whole cloud.
    """
    support = np.nonzero(intensity > SUP_MIN_I)[0]
    if support.size == 0 or query.size == 0:
        return np.zeros(query.size, dtype=bool)
    grid, _ = _grid(x, y, z, SUP_RADIUS, support)
    out = np.zeros(query.size, dtype=bool)
    r2 = SUP_RADIUS * SUP_RADIUS
    for n, j in enumerate(query):
        if not (intensity[j] <= 0.5 * SUP_MIN_I
                and math.hypot(x[j], y[j]) > SUP_RANGE_FLOOR):
            continue
        ck = (int(math.floor(x[j] / SUP_RADIUS)), int(math.floor(y[j] / SUP_RADIUS)),
              int(math.floor(z[j] / SUP_RADIUS)))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for i in grid.get((ck[0] + dx, ck[1] + dy, ck[2] + dz), ()):
                        if ((x[j] - x[i]) ** 2 + (y[j] - y[i]) ** 2
                                + (z[j] - z[i]) ** 2) <= r2:
                            out[n] = True
                            break
                    if out[n]:
                        break
                if out[n]:
                    break
            if out[n]:
                break
    return out


def counts_within(x, y, z, sel, radius, cap=1200, seed=0):
    """Neighbour count within `radius` (self included), vectorised per query.

    Candidates come from the hash; the distance test is one numpy operation, which
    keeps this usable on clouds carrying hundreds of points per cell.
    """
    grid, keys = _grid(x, y, z, SUP_RADIUS)
    if sel.size > cap:
        sel = np.random.default_rng(seed).choice(sel, cap, replace=False)
    span = int(math.ceil(radius / SUP_RADIUS))
    r2 = radius * radius
    counts = np.zeros(sel.size, dtype=np.int32)
    for n, j in enumerate(sel):
        cx, cy, cz = keys[j]
        cand: list[int] = []
        for dx in range(-span, span + 1):
            for dy in range(-span, span + 1):
                for dz in range(-span, span + 1):
                    cand.extend(grid.get((cx + dx, cy + dy, cz + dz), ()))
        if not cand:
            continue
        c = np.asarray(cand, dtype=np.int64)
        d2 = (x[c] - x[j]) ** 2 + (y[c] - y[j]) ** 2 + (z[c] - z[j]) ** 2
        counts[n] = int((d2 <= r2).sum())
    return counts


# ------------------------------------------------------------------------- modes
def frames_of(data: pathlib.Path, scene: str):
    pcd_dir = data / "pcd_output" / scene / "velodyne"
    gt_dir = data / "result" / scene
    if not pcd_dir.is_dir():
        raise SystemExit(f"no such scene: {pcd_dir}  (see docs/DATASET.md)")
    return sorted(pcd_dir.glob("*.pcd")), gt_dir


def mode_budget(data, scenes) -> int:
    print("Error budget: which stage makes each ground-truth point undetectable")
    print("(per-frame macro average — the protocol the metrics use)\n")
    print(f"{'scene':>6} {'frames':>7} {'GT':>10} {'ROI':>8} {'I-ceiling':>10} "
          f"{'veto':>7} {'reachable':>10} {'Tg=2.5':>7}")
    tot = dict(gt=0, roi=0, ceiling=0, veto=0, reach=0)
    mac = dict(roi=[], ceiling=[], veto=[], reach=[])
    for scene in scenes:
        frames, gt_dir = frames_of(data, scene)
        acc = dict(gt=0, roi=0, ceiling=0, veto=0, reach=0)
        m = dict(roi=[], ceiling=[], veto=[], reach=[])
        clamped = 0
        for f in frames:
            gt = load_gt(gt_dir / f"{f.stem}.txt")
            if gt.size == 0:
                continue
            pts = read_pcd(f)
            gt = gt[gt < pts.shape[0]]
            x, y, z, inten = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
            inside = roi_mask(x, y, z)
            roi = np.nonzero(inside)[0]
            tg = min(max(0.8 * q1_of(inten[roi]), 2.5), 8.0)
            if tg == 2.5:
                clamped += 1
            gt_in = gt[inside[gt]]
            acc["gt"] += gt.size
            acc["roi"] += gt.size - gt_in.size
            fr = dict(roi=gt.size - gt_in.size, ceiling=0, veto=0, reach=0)
            if gt_in.size:
                t = threshold_of(x[gt_in], y[gt_in], inten[gt_in], tg)
                under = inten[gt_in] < IT_MAX * t
                v = vetoed(x, y, z, inten, gt_in[under])
                acc["ceiling"] += int((~under).sum())
                acc["veto"] += int(v.sum())
                acc["reach"] += int((~v).sum())
                fr["ceiling"] = int((~under).sum())
                fr["veto"] = int(v.sum())
                fr["reach"] = int((~v).sum())
            g = max(int(gt.size), 1)
            for k in m:
                m[k].append(100.0 * fr[k] / g)
        for k in tot:
            tot[k] += acc[k]
        for k in mac:
            mac[k].extend(m[k])
        mean = {k: (sum(v) / len(v) if v else 0.0) for k, v in m.items()}
        print(f"{scene:>6} {len(frames):>7} {acc['gt']:>10} {mean['roi']:>7.1f}% "
              f"{mean['ceiling']:>9.1f}% {mean['veto']:>6.1f}% {mean['reach']:>9.1f}% "
              f"{clamped:>7}")
    mean = {k: (sum(v) / len(v) if v else 0.0) for k, v in mac.items()}
    print("-" * 70)
    print(f"{'ALL':>6} {'':>7} {tot['gt']:>10} {mean['roi']:>7.1f}% "
          f"{mean['ceiling']:>9.1f}% {mean['veto']:>6.1f}% {mean['reach']:>9.1f}%")
    print(f"\n  A. removed by the ROI gate        : {mean['roi']:6.2f}% of GT")
    print(f"  B. above the intensity ceiling    : {mean['ceiling']:6.2f}% of GT")
    print(f"  C. vetoed as attached to a surface: {mean['veto']:6.2f}% of GT")
    print(f"  D. reachable by the shipped rule  : {mean['reach']:6.2f}% of GT"
          f"   <- recall cannot exceed this")
    print("\n  Note: each point is charged to the first stage that rejects it, so A is not")
    print("  independently recoverable — see docs/OPTIMIZATION.md section 3.")
    return 0


def mode_intensities(data, scenes) -> int:
    print("Intensity distribution of ground truth inside the ROI\n")
    print(f"{'scene':>6} {'GT in ROI':>11} | {'I=0':>8} {'0<I<1':>8} {'1<=I<2':>8} "
          f"{'I>=2':>8} | {'median':>7} {'max':>6}")
    tot = np.zeros(4, dtype=np.int64)
    for scene in scenes:
        frames, gt_dir = frames_of(data, scene)
        buckets = np.zeros(4, dtype=np.int64)
        vals = []
        for f in frames:
            gt = load_gt(gt_dir / f"{f.stem}.txt")
            if gt.size == 0:
                continue
            pts = read_pcd(f)
            gt = gt[gt < pts.shape[0]]
            x, y, z, inten = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
            g = inten[gt[roi_mask(x, y, z)[gt]]]
            vals.append(g)
            buckets += [(g == 0).sum(), ((g > 0) & (g < 1)).sum(),
                        ((g >= 1) & (g < 2)).sum(), (g >= 2).sum()]
        tot += buckets
        v = np.concatenate(vals) if vals else np.zeros(0)
        n = max(int(buckets.sum()), 1)
        print(f"{scene:>6} {int(buckets.sum()):>11} | "
              + " ".join(f"{100.0*b/n:7.2f}%" for b in buckets)
              + f" | {np.median(v) if v.size else 0:7.1f} {v.max() if v.size else 0:6.0f}")
    n = max(int(tot.sum()), 1)
    print("-" * 74)
    print(f"{'ALL':>6} {int(tot.sum()):>11} | "
          + " ".join(f"{100.0*b/n:7.2f}%" for b in tot))
    return 0


def mode_separability(data, scenes) -> int:
    print("Neighbour counts of points above the intensity ceiling (I >= 2),")
    print("ground truth vs the rest, inside the ROI. A usable geometric feature")
    print("would separate these two columns.\n")
    print(f"{'scene':>6} {'bright':>9} {'GT@I>=2':>9} {'N(0.6m) GT':>12} "
          f"{'non-GT':>9} {'N(0.3m) GT':>12} {'non-GT':>9}")
    for scene in scenes:
        frames, gt_dir = frames_of(data, scene)
        res = {0.6: ([], []), 0.3: ([], [])}
        n_gt = n_non = 0
        for f in frames[:4]:
            gt = load_gt(gt_dir / f"{f.stem}.txt")
            if gt.size == 0:
                continue
            pts = read_pcd(f)
            gt = gt[gt < pts.shape[0]]
            x, y, z, inten = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
            roi = np.nonzero(roi_mask(x, y, z))[0]
            xs, ys, zs, Is = x[roi], y[roi], z[roi], inten[roi]
            bright = np.nonzero(Is >= 2.0)[0]
            if bright.size == 0:
                continue
            is_gt = np.zeros(pts.shape[0], dtype=bool)
            is_gt[gt] = True
            m = is_gt[roi][bright]
            n_gt += int(m.sum())
            n_non += int((~m).sum())
            for rad in res:
                if m.any():
                    res[rad][0].append(counts_within(xs, ys, zs, bright[m], rad))
                if (~m).any():
                    res[rad][1].append(counts_within(xs, ys, zs, bright[~m], rad))
        if not res[0.6][0]:
            print(f"{scene:>6}  (no ground-truth points above the ceiling)")
            continue
        med = {r: (np.median(np.concatenate(v[0])), np.median(np.concatenate(v[1])))
               for r, v in res.items()}
        print(f"{scene:>6} {n_gt + n_non:>9} {n_gt:>9} {med[0.6][0]:>12.0f} "
              f"{med[0.6][1]:>9.0f} {med[0.3][0]:>12.0f} {med[0.3][1]:>9.0f}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", default="budget",
                    choices=["budget", "intensities", "separability"])
    ap.add_argument("--scenes", nargs="+", default=["35", "11", "14", "16"])
    ap.add_argument("--data", type=pathlib.Path,
                    default=pathlib.Path(os.environ.get("SNOWCLEAR_DATA", "./data")),
                    help="WADS mirror root (default $SNOWCLEAR_DATA or ./data)")
    args = ap.parse_args()
    if not args.data.is_dir():
        print(f"no dataset at {args.data}; set SNOWCLEAR_DATA or --data "
              f"(layout: docs/DATASET.md)", file=sys.stderr)
        return 2
    return {"budget": mode_budget, "intensities": mode_intensities,
            "separability": mode_separability}[args.mode](args.data, args.scenes)


if __name__ == "__main__":
    raise SystemExit(main())
