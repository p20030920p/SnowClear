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
from scipy.spatial import cKDTree

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


def support_index(x, y, z, intensity):
    """The `I > SUP_MIN_I` support set as a KD-tree, or None if it is empty.

    Independent of the ROI, so a caller that asks the same frame several questions builds
    it once. The predicate itself is unchanged from the hash-grid version this replaced
    (see `vetoed`); only the neighbour lookup is vectorised, which is what makes a
    seven-variant sweep of the same frames affordable.
    """
    support = np.nonzero(intensity > SUP_MIN_I)[0]
    if support.size == 0:
        return None
    return cKDTree(np.column_stack((x[support], y[support], z[support])))


def vetoed(x, y, z, intensity, query: np.ndarray, grid=None) -> np.ndarray:
    """Zero-intensity surface suppression, evaluated for the points in `query`.

    A query point is vetoed when it is a weak return (`I <= 0.5`), beyond
    `SUP_RANGE_FLOOR`, and has at least one `I > SUP_MIN_I` point within `SUP_RADIUS`:
    the support set is not the whole cloud. Pass the index from `support_index` when
    several queries share one frame.
    """
    if query.size == 0:
        return np.zeros(0, dtype=bool)
    if grid is None:
        grid = support_index(x, y, z, intensity)
    if grid is None:
        return np.zeros(query.size, dtype=bool)
    candidate = (intensity[query] <= 0.5 * SUP_MIN_I) & \
                (np.hypot(x[query], y[query]) > SUP_RANGE_FLOOR)
    out = np.zeros(query.size, dtype=bool)
    sel = np.nonzero(candidate)[0]
    if sel.size:
        pts = np.column_stack((x[query[sel]], y[query[sel]], z[query[sel]]))
        out[sel] = grid.query_ball_point(pts, SUP_RADIUS, return_length=True) > 0
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


def mode_budget(data, scenes, csv_path=None) -> int:
    print("Error budget: which stage makes each ground-truth point undetectable")
    print("(per-frame macro average — the protocol the metrics use)\n")
    print(f"{'scene':>6} {'frames':>7} {'GT':>10} {'ROI':>8} {'I-ceiling':>10} "
          f"{'veto':>7} {'reachable':>10} {'Tg=2.5':>7}")
    rows = []
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
        rows.append((scene, len(frames), acc["gt"], mean["roi"], mean["ceiling"],
                     mean["veto"], mean["reach"], clamped))
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
    if csv_path is not None:
        with open(csv_path, "w", encoding="utf-8") as fh:
            fh.write("scene,frames,gt,roi,ceiling,veto,reachable,tg_clamped\n")
            for row in rows:
                fh.write(",".join([row[0]] + [f"{v:.4f}" if isinstance(v, float) else str(v)
                                              for v in row[1:]]) + "\n")
        print(f"  wrote {csv_path}")
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


# ------------------------------------------------------------------ ROI variants
# Widening the gate is the only lever on the quarter of the ground truth the ROI throws
# away before any decision is made (mode budget, item A). Each variant is scored with the
# shipped rule's own necessary condition — intensity below the ceiling and not vetoed —
# which is the same notion of "reachable" mode budget uses. On this subset that proxy
# reads 68.57 % against a measured recall of 68.60 %, so it tracks the detector; but it
# is an *upper* bound: it ignores the score gate's h_ag term, so the false-positive
# column is the worst case, not a prediction.
VARIANTS = [
    ("released", Z_HI, XY_MAX, ELEV_MIN_DEG),
    ("z <= 4.0 m", 4.0, XY_MAX, ELEV_MIN_DEG),
    ("r <= 25 m", Z_HI, 25.0, ELEV_MIN_DEG),
    ("elev >= -30 deg", Z_HI, XY_MAX, -30.0),
    ("z <= 4.0 and r <= 25", 4.0, 25.0, ELEV_MIN_DEG),
    ("all three widened", 4.0, 25.0, -30.0),
]


def gate_mask(x, y, z, z_hi, r_max, elev_min) -> np.ndarray:
    r3 = np.sqrt(x * x + y * y + z * z)
    elev = np.degrees(np.arcsin(np.clip(z / np.maximum(r3, 1e-9), -1.0, 1.0)))
    return (z >= Z_LO) & (z <= z_hi) & (x * x + y * y <= r_max ** 2) & (elev >= elev_min)


def reachable_idx(x, y, z, intensity, idx, tg, grid=None) -> np.ndarray:
    """The subset of `idx` the shipped rule could still admit (ceiling, then veto)."""
    if idx.size == 0:
        return idx
    t = threshold_of(x[idx], y[idx], intensity[idx], tg)
    sub = idx[intensity[idx] < IT_MAX * t]
    return sub if sub.size == 0 else sub[~vetoed(x, y, z, intensity, sub, grid)]


def mode_roi_variants(data, scenes) -> int:
    print("Widening the ROI gate: the ground truth it makes reachable, and the non-snow")
    print("points that become reachable with it (per-frame macro average)\n")
    agg = {name: dict(gt=[], gtn=[], fp=[]) for name, *_ in VARIANTS[1:]}
    pool = []
    for scene in scenes:
        frames, gt_dir = frames_of(data, scene)
        print(f"  scene {scene}: {len(frames)} frames", flush=True)
        for f in frames:
            gt = load_gt(gt_dir / f"{f.stem}.txt")
            if gt.size == 0:
                continue
            pts = read_pcd(f)
            gt = gt[gt < pts.shape[0]]
            x, y, z, inten = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
            is_gt = np.zeros(pts.shape[0], dtype=bool)
            is_gt[gt] = True
            base = gate_mask(x, y, z, Z_HI, XY_MAX, ELEV_MIN_DEG)
            b_idx = np.nonzero(base)[0]
            tg = min(max(0.8 * q1_of(inten[b_idx]), 2.5), 8.0)
            grid = support_index(x, y, z, inten)
            base_reach = np.zeros(pts.shape[0], dtype=bool)
            base_reach[reachable_idx(x, y, z, inten, b_idx, tg, grid)] = True
            pool.append(100.0 * float((~base & is_gt).sum()) / max(int(gt.size), 1))
            for name, z_hi, r_max, elev_min in VARIANTS[1:]:
                idx = np.nonzero(gate_mask(x, y, z, z_hi, r_max, elev_min))[0]
                if idx.size == 0:
                    continue
                tg_v = min(max(0.8 * q1_of(inten[idx]), 2.5), 8.0)
                new = np.zeros(pts.shape[0], dtype=bool)
                new[reachable_idx(x, y, z, inten, idx, tg_v, grid)] = True
                added_gt = int((new & is_gt & ~base_reach).sum())
                agg[name]["gt"].append(100.0 * added_gt / max(int(gt.size), 1))
                agg[name]["gtn"].append(float(added_gt))
                agg[name]["fp"].append(float((new & ~is_gt & ~base_reach).sum()))
    print(f"{'variant':>20} {'dGT pp':>8} {'dGT/frame':>10} {'dFP/frame':>10} "
          f"{'FP/GT':>7}")
    best = None
    for name, *_ in VARIANTS[1:]:
        g = sum(agg[name]["gt"]) / max(len(agg[name]["gt"]), 1)
        gn = sum(agg[name]["gtn"]) / max(len(agg[name]["gtn"]), 1)
        fp = sum(agg[name]["fp"]) / max(len(agg[name]["fp"]), 1)
        gpf = fp / g if g else float("inf")
        print(f"{name:>20} {g:>7.2f}% {gn:>10.0f} {fp:>10.0f} {gpf:>7.1f}")
        if best is None or gpf < best[1]:
            best = (name, gpf, g, fp)
    print(f"\n  ground truth the released gate removes: {np.mean(pool):.2f}% of GT "
          f"(the pool a wider gate can draw from)")
    if best:
        print(f"  best trade: {best[0]} — +{best[2]:.2f} pp of GT reachable for "
              f"{best[3]:.0f} extra reachable non-snow points per frame "
              f"({best[1]:.1f} FP per recovered GT)")
    print("  Caveat: FP/GT counts non-snow points that clear the ceiling and the veto, not")
    print("  points the full score would accept; treat it as the worst case and see")
    print("  docs/OPTIMIZATION.md section 3 for why A is not independently recoverable.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", default="budget",
                    choices=["budget", "intensities", "separability", "roi_variants"])
    ap.add_argument("--scenes", nargs="+", default=["35", "11", "14", "16"])
    ap.add_argument("--csv", type=pathlib.Path, default=None,
                    help="write the per-scene table as CSV (fig 8 reads this)")
    ap.add_argument("--data", type=pathlib.Path,
                    default=pathlib.Path(os.environ.get("SNOWCLEAR_DATA", "./data")),
                    help="WADS mirror root (default $SNOWCLEAR_DATA or ./data)")
    args = ap.parse_args()
    if not args.data.is_dir():
        print(f"no dataset at {args.data}; set SNOWCLEAR_DATA or --data "
              f"(layout: docs/DATASET.md)", file=sys.stderr)
        return 2
    if args.mode == "budget":
        return mode_budget(args.data, args.scenes, args.csv)
    return {"intensities": mode_intensities,
            "separability": mode_separability,
            "roi_variants": mode_roi_variants}[args.mode](args.data, args.scenes)


if __name__ == "__main__":
    raise SystemExit(main())
