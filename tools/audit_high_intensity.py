#!/usr/bin/env python3
"""Price the high-intensity recall lever the released rule structurally cannot reach.

Why this exists
---------------
`SnowDetector::evaluate_point` only ever admits a *low*-intensity point:

  * `snow_detector.cpp:218` returns immediately for `I >= 1.2 * T` ("早停"), and
  * `snow_detector.cpp:224` sets the intensity score to 0 for `I >= T`, while the
    final test (`snow_detector.cpp:451`) needs `intensity_score > 0.3`.

`T = clamp(0.8*Tg*(1 - alpha(r)*h), 2.0, 20.0)` with `Tg = clamp(0.8*Q1, 2.5, 8.0)`, so
`T <= 6.4` on every frame: any point with `I >= 2` that survives the ROI gate is already
inadmissible, and annotated snow is bimodal (almost all I = 0, a bright tail at I >= 2).
This script measures that bright tail and prices the obvious remedy — an opt-in "admit
bright points that look like snow geometrically" branch — so the decision is made from
numbers instead of from the shape of the equation.

What it measures (everything in-ROI, per frame, then macro-averaged over scenes, the
protocol of `tools/audit_error_budget.py`)
  1. pool     — how much annotated snow sits in the `I >= 2` and `I >= 1.2*T` bands, as a
                share of in-ROI ground truth and of all ground truth;
  2. shape    — range, height above the local (1 m cell, 10th percentile) ground and 0.6 m
                neighbourhood support, bright pool vs the non-snow in the same band;
  3. branch   — the candidate rule `(I >= 2) and (h_ag >= X) and (support <= Y)`, swept
                over X and Y and priced against (a) the current rule and (b) the naive
                branch that admits every in-ROI `I >= 2` point.

Consistency: the released-config constants, the ROI mask, Q1, `T(r, I)` and the surface
veto all come from `audit_error_budget`, so the `A`/`B` budget columns printed here are the
same measurement as `--mode budget`. I/O uses `pcd_common` (the shared reader); the two
readers are compared exactly in section 0, and the vectorised support/veto counters are
compared there against `audit_error_budget`'s reference implementations too.

The "current rule" column is a *simulation*, not a pipeline run: the released switch set
(planarity/density/KNN/propagation/pre-downsampling/dedup all off) makes `evaluate_point` a
pure function of `(x, y, z, I, ground)`, transcribed from the C++ lines cited plus
`feature_extractor.cpp:510-563` (ground grid) and `snow_detector.cpp:231-238` (veto).
Section 0 asserts the simulation never accepts a point outside `audit_error_budget`'s
reachable set, which is what makes the recall/precision deltas comparable with the
published budget.

Usage
    python3 tools/audit_high_intensity.py
    SNOWCLEAR_DATA=/path/to/mirror python3 tools/audit_high_intensity.py --scenes 35 11 14 16

The dataset is not redistributable; see docs/DATASET.md for the expected layout.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import sys

import numpy as np
from scipy.spatial import cKDTree

# House pattern: tools import each other by path (see pcd_common's docstring).
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import audit_error_budget as aeb  # released-config constants, ROI, T(r,I), veto, frames_of
import pcd_common as pcd  # shared PCD / index readers

# --------------------------------------------------------------- measurement knobs
I_POOL = 2.0             # the "bright" band the task asks about (aeb.mode_intensities' last bucket)
EARLY_RETURN_MULT = 1.2  # snow_detector.cpp:218 `I >= 1.2*T` -> unreachable before scoring
S_GATE = 0.3             # snow_detector.cpp:451 `intensity_score > 0.3` (hard necessary condition)
# The score test is `combined > local_threshold and score > 0.3`. With only intensity
# (0.35) and height (0.15) enabled, `combined = 0.7*score + 0.3*height_score` and
# `height_score <= 0.5`, so the test is satisfiable only through the high-score branch,
# where `local_threshold = 0.75*0.9`: `0.7*score > 0.675 - 0.15` -> `score > 0.75`.
# That makes `I < IT_TRUE*T` a true necessary condition, while audit_error_budget's IT_MAX
# (derived with the 0.75 mid-branch threshold) is stricter than necessary. Both are
# reported so the pool numbers can sit next to the published ceiling.
IT_TRUE = 1.0 - aeb.SCORE_THRESHOLD ** (1.0 / 1.2)
INTENSITY_WEIGHT, HEIGHT_WEIGHT = 0.35, 0.15  # system_config.hpp defaults; planarity/density off
GROUND_CELL = 1.0        # feature_extractor.cpp:516
GROUND_MIN_PTS = 5       # feature_extractor.cpp:529 — thinner cells fall back to the cloud min z
HEIGHT_SCALE = 1.5       # feature_extractor.cpp:560 — height_above_ground saturates 1.5 m above ground
BRIGHT_SUPPORT_MIN_I = aeb.SUP_MIN_I  # the detector's own support set: I > 1.0
SUPPORT_R = aeb.SUP_RADIUS            # 0.6 m
# Candidate rule sweep. Heights are metres above the local ground estimate; supports are
# counts of in-ROI points within 0.6 m (self included, as in the detector's radius query).
RULE_HEIGHTS = (0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.50)
RULE_SUPPORTS = (4, 8, 16, 32, 64, 128, 256, np.inf)
SAMPLE = 300  # queries used for the reference-implementation checks in section 0
SEED = 0


# ------------------------------------------------------------------------ geometry
def ground_z(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
    """Per-point ground estimate, replicating `FeatureExtractor::build_ground_grid`.

    Cells are 1 m x 1 m in XY; the estimate is the z at rank `size // 10` of the cell's
    z-sorted points (the detector's 10th percentile, floor index included), and cells with
    fewer than 5 points fall back to the cloud-wide minimum z. Built on the post-ROI cloud,
    which is the cloud `analyze()` sees in the released configuration (`preprocessor.cpp:606`,
    dedup off -> stats_cloud == ROI cloud).
    """
    fx = np.floor(x / GROUND_CELL).astype(np.int64)
    fy = np.floor(y / GROUND_CELL).astype(np.int64)
    # feature_extractor.cpp:35 — (int64(floor(x)) << 32) ^ (int64(floor(y)) & 0xffffffff),
    # reproduced on the bit pattern so negative cells hash identically to the C++.
    key = (fx.view(np.uint64) << np.uint64(32)) ^ (fy.view(np.uint64) & np.uint64(0xFFFFFFFF))
    _uniq, inv = np.unique(key, return_inverse=True)  # inv is grouped by cell, cells sorted

    order = np.lexsort((z, inv))  # by cell, then z ascending == the per-cell std::sort
    z_sorted, inv_sorted = z[order], inv[order]
    starts = np.flatnonzero(np.r_[True, inv_sorted[1:] != inv_sorted[:-1]])
    sizes = np.diff(np.r_[starts, inv_sorted.size])
    # rank = floor(size/10) is < size for any size >= 1, so the gather stays in range even
    # for the skipped thin cells; np.where then substitutes the fallback for those.
    per_cell = np.where(sizes >= GROUND_MIN_PTS, z_sorted[starts + sizes // 10], z.min())
    return per_cell[inv]


def radius_counts(tree: cKDTree, points: np.ndarray, radius: float) -> np.ndarray:
    """Number of `tree` points within `radius` of each row of `points`, self included.

    Same query as `audit_error_budget.counts_within` (which the detector implements as a
    0.6 m cell hash plus an exact distance test); section 0 verifies they agree exactly.
    """
    if points.shape[0] == 0:
        return np.zeros(0, dtype=np.int64)
    return np.asarray(tree.query_ball_point(points, radius, return_length=True), dtype=np.int64)


def released_accept(x, y, z, intensity, t, ground, tree_bright):
    """The released `evaluate_point`, vectorised, on the post-ROI cloud.

    Returns `(accept, reach, reach_true)`: `accept` is the simulated detector output;
    `reach` is `audit_error_budget`'s necessary condition (`I < IT_MAX*T` and not vetoed),
    i.e. the published error-budget ceiling; `reach_true` is the same with this file's
    tighter derivation of the score gate (`I < IT_TRUE*T`, see IT_TRUE above). `accept`
    must always be a subset of both. `tree_bright` is the KD tree over the `I > 1.0`
    support set, built once per frame and shared with the support column of section 2.
    """
    r = np.hypot(x, y)
    score = np.where(intensity < t, np.power(np.clip(1.0 - intensity / t, 0.0, 1.0), 1.2), 0.0)
    height_score = 0.5 * np.clip((z - ground) / HEIGHT_SCALE, 0.0, 1.0)
    weight_sum = INTENSITY_WEIGHT + HEIGHT_WEIGHT  # planarity/density are off in the release
    combined = (INTENSITY_WEIGHT / weight_sum) * score + (HEIGHT_WEIGHT / weight_sum) * height_score

    # snow_detector.cpp:438-448 — local threshold: -10% once the score is high, +20% below 0.4
    local = np.where(score < 0.4, 0.75 * 1.2, np.where(score > 0.7, 0.75 * 0.9, 0.75))

    # snow_detector.cpp:231-238 — I <= 0.5, r > 7 m, a support point (I > 1.0) within 0.6 m
    veto = np.zeros(x.size, dtype=bool)
    cand = np.flatnonzero((intensity <= 0.5 * aeb.SUP_MIN_I) & (r > aeb.SUP_RANGE_FLOOR))
    if cand.size and tree_bright is not None:
        veto[cand] = radius_counts(
            tree_bright, np.column_stack((x[cand], y[cand], z[cand])), SUPPORT_R) > 0

    accept = (combined > local) & (score > S_GATE) & ~veto
    reach = (intensity < aeb.IT_MAX * t) & ~veto        # audit_error_budget's (conservative) ceiling
    reach_true = (intensity < IT_TRUE * t) & ~veto      # this file's necessary condition
    return accept, reach, reach_true


# ---------------------------------------------------------------------- one frame
def frame_stats(pcd_path: pathlib.Path, gt_path: pathlib.Path) -> dict:
    """Every per-frame quantity this audit reports; the kept arrays are pool-sized."""
    pts = pcd.read_pcd(pcd_path)
    n = pts.shape[0]
    gt = pcd.load_indices(gt_path)
    gt = gt[gt < n]
    x, y, z, intensity = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]

    inside = aeb.roi_mask(x, y, z)
    roi = np.flatnonzero(inside)
    xr, yr, zr, ir = x[roi], y[roi], z[roi], intensity[roi]
    is_gt = np.zeros(n, dtype=bool)
    is_gt[gt] = True
    gt_in = is_gt[roi]

    # Q1 and Tg exactly as cloud_operations.cpp:301 derives them from the post-ROI cloud.
    tg = min(max(0.8 * aeb.q1_of(ir), 2.5), 8.0)
    t = aeb.threshold_of(xr, yr, ir, tg)
    gz = ground_z(xr, yr, zr)
    h_ag = zr - gz

    # Two KD trees per frame, built once and shared by the simulation, the veto and the
    # support columns (the bright set is the detector's own support definition).
    tree_all = cKDTree(np.column_stack((xr, yr, zr))) if roi.size else None
    bright = np.flatnonzero(ir > BRIGHT_SUPPORT_MIN_I)
    tree_bright = (cKDTree(np.column_stack((xr[bright], yr[bright], zr[bright])))
                   if bright.size else None)
    accept, reach, reach_true = released_accept(xr, yr, zr, ir, t, gz, tree_bright)
    r = np.hypot(xr, yr)

    pool = ir >= I_POOL
    band = ir >= EARLY_RETURN_MULT * t
    sup = np.zeros(roi.size, dtype=np.int64)
    bright_sup = np.zeros(roi.size, dtype=np.int64)
    if pool.any():
        pq = np.column_stack((xr[pool], yr[pool], zr[pool]))
        sup[pool] = radius_counts(tree_all, pq, SUPPORT_R)
        if tree_bright is not None:
            bright_sup[pool] = radius_counts(tree_bright, pq, SUPPORT_R)

    return dict(
        n_gt=int(gt.size), n_gt_in=int(gt_in.sum()), n_roi=int(roi.size),
        tp_cur=int((accept & gt_in).sum()), fp_cur=int((accept & ~gt_in).sum()),
        accepted=int(accept.sum()), unreachable=int((accept & ~reach).sum()),
        unreachable_true=int((accept & ~reach_true).sum()),
        pool_admitted=int((accept & pool).sum()),
        # audit_error_budget's A/B stages, recomputed here so both tables share a protocol
        roi_out=int(gt.size - gt_in.sum()),
        ceiling=int((~reach & (ir < aeb.IT_MAX * t) & gt_in).sum()),
        ceiling_true=int(((ir >= IT_TRUE * t) & gt_in).sum()),
        pool_gt=int((pool & gt_in).sum()), pool_non=int((pool & ~gt_in).sum()),
        band_gt=int((band & gt_in).sum()), band_non=int((band & ~gt_in).sum()),
        # pool point clouds, kept for the rule sweep and the distribution table
        pool_gt_h=h_ag[pool][gt_in[pool]], pool_gt_s=sup[pool][gt_in[pool]],
        pool_gt_r=r[pool][gt_in[pool]], pool_gt_b=bright_sup[pool][gt_in[pool]],
        pool_no_h=h_ag[pool][~gt_in[pool]], pool_no_s=sup[pool][~gt_in[pool]],
        pool_no_r=r[pool][~gt_in[pool]], pool_no_b=bright_sup[pool][~gt_in[pool]],
    )


# -------------------------------------------------------------------- aggregation
def macro(values: list) -> float:
    return float(np.mean(values)) if values else 0.0


def spread(frames: list, key: str) -> tuple[float, float, float, int]:
    """Macro mean of the per-frame median and of the per-frame quartiles.

    Per-frame quantiles then averaged keeps the per-frame macro protocol; pooling every
    point instead would let one dense frame dominate the shape of the table. `n` counts the
    frames that contribute (a frame with no bright snow adds no median to average).
    """
    vals = [f[key] for f in frames if f[key].size]
    if not vals:
        return 0.0, 0.0, 0.0, 0
    return (macro([np.median(v) for v in vals]), macro([np.percentile(v, 25) for v in vals]),
            macro([np.percentile(v, 75) for v in vals]), len(vals))


def prf(tp: list, fp: list, fn: list) -> tuple[float, float, float]:
    """Macro precision/recall/F1: the mean of the per-frame ratios (evaluator.cpp:211-213)."""
    p = macro([t / (t + f) if (t + f) else 0.0 for t, f in zip(tp, fp)])
    r = macro([t / (t + f) if (t + f) else 0.0 for t, f in zip(tp, fn)])
    f1 = macro([2 * t / (2 * t + f + n) if (2 * t + f + n) else 0.0
                for t, f, n in zip(tp, fp, fn)])
    return p, r, f1


def pct(cur: list, base: list) -> float:
    """Macro ratio of two per-frame count lists, in percent (0 where the base is 0)."""
    return 100.0 * macro([c / b if b else 0.0 for c, b in zip(cur, base)])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenes", nargs="+", default=["35", "11", "14", "16"],
                    help="scene ids (default: the four-scene audit sample, 406 frames)")
    ap.add_argument("--data", type=pathlib.Path,
                    default=pathlib.Path(os.environ.get("SNOWCLEAR_DATA", "./data")),
                    help="WADS mirror root (default $SNOWCLEAR_DATA or ./data)")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="cap frames per scene (0 = all); for smoke tests only")
    args = ap.parse_args()
    if not args.data.is_dir():
        print(f"no dataset at {args.data}; set SNOWCLEAR_DATA or --data "
              f"(layout: docs/DATASET.md)", file=sys.stderr)
        return 2

    per_scene: dict[str, list] = {}
    for scene in args.scenes:
        frames, gt_dir = aeb.frames_of(args.data, scene)
        if args.max_frames:
            frames = frames[:args.max_frames]
        st = [frame_stats(f, gt_dir / f"{f.stem}.txt") for f in frames]
        per_scene[scene] = [s for s in st if s["n_gt"] > 0]
    allf = [s for st in per_scene.values() for s in st]
    if not allf:
        print("no annotated frames in the requested scenes", file=sys.stderr)
        return 2

    # ---------------------------------------------------------------- section 0
    sample_pcd = aeb.frames_of(args.data, args.scenes[0])[0][0]
    same = np.array_equal(pcd.read_pcd(sample_pcd), aeb.read_pcd(sample_pcd))
    print("Section 0 — consistency with tools/audit_error_budget.py and tools/pcd_common.py\n")
    print(f"  readers agree bit for bit on {sample_pcd.name}: {same}")
    print(f"  simulation accepts {sum(s['accepted'] for s in allf)} points, "
          f"{sum(s['unreachable_true'] for s in allf)} of them outside the true necessary "
          f"condition (must be 0);")
    print(f"  {sum(s['unreachable'] for s in allf)} of them outside audit_error_budget's "
          f"IT_MAX = {aeb.IT_MAX:.4f} ceiling, which assumes the 0.75 mid-branch local "
          f"threshold;")
    print(f"  the binding bound is IT_TRUE = {IT_TRUE:.4f} (score > {aeb.SCORE_THRESHOLD}), so "
          f"the published I-ceiling over-counts by the {aeb.IT_MAX:.4f}-{IT_TRUE:.4f} gap "
          f"(see the ceiling_true note in section 1).")
    print(f"  released rule admits in-ROI I >= {I_POOL:g} points: "
          f"{sum(s['pool_admitted'] for s in allf)} (structural claim: must be 0)")

    # Reference-implementation checks on one frame: the vectorised counters must match
    # audit_error_budget's hash-grid versions exactly, or these numbers could not sit next
    # to the budget table.
    pts = pcd.read_pcd(sample_pcd)
    x, y, z, intensity = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
    roi = np.flatnonzero(aeb.roi_mask(x, y, z))
    xr, yr, zr, ir = x[roi], y[roi], z[roi], intensity[roi]
    rng = np.random.default_rng(SEED)
    q = rng.choice(roi.size, size=min(SAMPLE, roi.size), replace=False)
    tree = cKDTree(np.column_stack((xr, yr, zr)))
    d_sup = np.abs(radius_counts(tree, np.column_stack((xr[q], yr[q], zr[q])), SUPPORT_R)
                   - aeb.counts_within(xr, yr, zr, q, SUPPORT_R, cap=SAMPLE))
    vq = q[(ir[q] <= 0.5 * aeb.SUP_MIN_I) & (np.hypot(xr[q], yr[q]) > aeb.SUP_RANGE_FLOOR)][:SAMPLE]
    d_veto = 0
    if vq.size:
        bright = np.flatnonzero(ir > BRIGHT_SUPPORT_MIN_I)
        btree = cKDTree(np.column_stack((xr[bright], yr[bright], zr[bright])))
        v_mine = radius_counts(btree, np.column_stack((xr[vq], yr[vq], zr[vq])), SUPPORT_R) > 0
        d_veto = int(np.abs(v_mine.astype(int) - aeb.vetoed(xr, yr, zr, ir, vq).astype(int)).sum())
    print(f"  support counter vs audit_error_budget.counts_within: max |diff| = "
          f"{int(d_sup.max()) if d_sup.size else 0} over {q.size} queries")
    print(f"  veto counter vs audit_error_budget.vetoed: {d_veto} mismatches over {vq.size} queries")
    print(f"  note: the ROI floor is the released constant z >= {aeb.Z_LO} m; the "
          f"self-calibrated variant (-h_s+1.125 = -1.008 m) is off by default, so this "
          f"audit matches --mode budget.")

    # ---------------------------------------------------------------- section 1
    stages = dict(gt=0, gt_in=0, roi=0, pool=0, non=0, band=0, band_non=0)
    for st in per_scene.values():
        for k, key in (("gt", "n_gt"), ("gt_in", "n_gt_in"), ("roi", "n_roi"), ("pool", "pool_gt"),
                       ("non", "pool_non"), ("band", "band_gt"), ("band_non", "band_non")):
            stages[k] += sum(s[key] for s in st)
    print("\nSection 1 — how large is the pool? (counts summed, shares per-frame macro)\n")
    print(f"{'scene':>6} {'fr':>4} {'GT':>7} {'in-ROI':>7} {'pts in ROI':>10} | {'A:ROI-out':>9} "
          f"{'B:I-ceil':>9} | {'I>=2 GT':>8} {'%in-ROI':>8} {'pooled':>7} {'%all':>7} "
          f"{'I>=2 no':>8} | {'I>=1.2T':>8} {'%in-ROI':>8} {'non-GT':>7}")
    for scene, st in per_scene.items():
        a = pct([s["roi_out"] for s in st], [s["n_gt"] for s in st])
        b = pct([s["ceiling"] for s in st], [s["n_gt"] for s in st])
        # Point-pooled share of in-ROI GT, printed next to the per-frame macro one because
        # the "bimodal intensity" headline in the task background is a pooled figure.
        pooled = 100.0 * sum(s["pool_gt"] for s in st) / max(sum(s["n_gt_in"] for s in st), 1)
        print(f"{scene:>6} {len(st):>4} {sum(s['n_gt'] for s in st):>7} "
              f"{sum(s['n_gt_in'] for s in st):>7} {sum(s['n_roi'] for s in st):>10} | "
              f"{a:>8.2f}% {b:>8.2f}% | "
              f"{sum(s['pool_gt'] for s in st):>8} "
              f"{pct([s['pool_gt'] for s in st], [s['n_gt_in'] for s in st]):>7.2f}% "
              f"{pooled:>6.2f}% "
              f"{pct([s['pool_gt'] for s in st], [s['n_gt'] for s in st]):>6.2f}% "
              f"{sum(s['pool_non'] for s in st):>8} | {sum(s['band_gt'] for s in st):>8} "
              f"{pct([s['band_gt'] for s in st], [s['n_gt_in'] for s in st]):>7.2f}% "
              f"{sum(s['band_non'] for s in st):>7}")
    a = pct([s["roi_out"] for s in allf], [s["n_gt"] for s in allf])
    b = pct([s["ceiling"] for s in allf], [s["n_gt"] for s in allf])
    p_in = pct([s["pool_gt"] for s in allf], [s["n_gt_in"] for s in allf])
    p_in_pooled = 100.0 * stages["pool"] / max(stages["gt_in"], 1)
    p_all = pct([s["pool_gt"] for s in allf], [s["n_gt"] for s in allf])
    b_in = pct([s["band_gt"] for s in allf], [s["n_gt_in"] for s in allf])
    print("-" * 143)
    print(f"{'ALL':>6} {len(allf):>4} {stages['gt']:>7} {stages['gt_in']:>7} {stages['roi']:>10} | "
          f"{a:>8.2f}% {b:>8.2f}% | {stages['pool']:>8} {p_in:>7.2f}% {p_in_pooled:>6.2f}% "
          f"{p_all:>6.2f}% {stages['non']:>8} | {stages['band']:>8} {b_in:>7.2f}% "
          f"{stages['band_non']:>7}")
    print(f"\n  A/B are audit_error_budget's first two stages (ROI gate, intensity ceiling) "
          f"recomputed in this pass;")
    print(f"  the pool columns are the lever this audit prices. '%in-ROI' is the per-frame "
          f"macro (the --mode budget protocol),")
    print(f"  'pooled' is sum(pool GT)/sum(in-ROI GT) over the sample — the two differ when "
          f"bright snow concentrates in few frames.")
    print(f"  frames with no annotated I >= {I_POOL:g} point in ROI: "
          f"{sum(1 for s in allf if s['pool_gt'] == 0)}/{len(allf)}")
    b_true = pct([s["ceiling_true"] for s in allf], [s["n_gt"] for s in allf])
    print(f"  B above uses audit_error_budget's IT_MAX = {aeb.IT_MAX:.4f}*T; with the binding "
          f"score gate (I >= {IT_TRUE:.4f}*T) the unreachable share is {b_true:.2f}% of GT, "
          f"i.e. B over-counts.")
    print(f"  the 1.2*T band is a subset of the pool: T >= 2 always, so the band needs "
          f"I >= {EARLY_RETURN_MULT * 2.0:.1f} (measured {stages['band']} <= {stages['pool']} GT points)")

    # ---------------------------------------------------------------- section 2
    print("\nSection 2 — can geometry tell the pool's snow from the non-snow in the same "
          "band?\n")
    print("  per-frame median [q25, q75] of each feature, macro-averaged over frames; the "
          "last column is the\n  detector's own support set (points with I > 1.0), the "
          "others count all in-ROI points\n")
    print(f"{'class':>15} {'n pooled':>9} {'fr':>5} | {'r [m]':>20} | {'h above ground [m]':>21} "
          f"| {'N(0.6 m)':>20} | {'N(I>1, 0.6 m)':>20}")
    for name, kh, kr, ks, kb, n in (("GT @ I>=2", "pool_gt_h", "pool_gt_r", "pool_gt_s",
                                     "pool_gt_b", stages["pool"]),
                                    ("non-GT @ I>=2", "pool_no_h", "pool_no_r", "pool_no_s",
                                     "pool_no_b", stages["non"])):
        h, rr, ss, bb = spread(allf, kh), spread(allf, kr), spread(allf, ks), spread(allf, kb)
        print(f"{name:>15} {n:>9} {h[3]:>5} | {rr[0]:7.2f} [{rr[1]:5.2f},{rr[2]:6.2f}] | "
              f"{h[0]:7.3f} [{h[1]:5.3f},{h[2]:6.3f}] | {ss[0]:7.1f} [{ss[1]:5.1f},{ss[2]:6.1f}] "
              f"| {bb[0]:7.1f} [{bb[1]:5.1f},{bb[2]:6.1f}]")

    # ---------------------------------------------------------------- section 3
    tp_cur = [s["tp_cur"] for s in allf]
    fp_cur = [s["fp_cur"] for s in allf]
    fn_cur = [s["n_gt"] - t for s, t in zip(allf, tp_cur)]
    gt_in_cur = [s["n_gt_in"] for s in allf]  # in-ROI recall denominator
    p0, r0, f10 = prf(tp_cur, fp_cur, fn_cur)
    rin0 = macro([t / n if n else 0.0 for t, n in zip(tp_cur, gt_in_cur)])
    print("\nSection 3 — price of a bright admission branch (per-frame macro)\n")
    print(f"{'rule':>34} {'TP/fr':>8} {'FP/fr':>8} | {'recall':>7} {'in-ROI R':>9} | {'prec':>7} "
          f"{'F1':>7} | {'dR pp':>7} {'dRin pp':>8} {'dF1 pp':>7}")
    print(f"{'current (released rule, simulated)':>34} {macro(tp_cur):>8.1f} {macro(fp_cur):>8.1f} "
          f"| {100*r0:>6.2f}% {100*rin0:>8.2f}% | {100*p0:>6.2f}% {100*f10:>6.2f}% | "
          f"{'-':>7} {'-':>8} {'-':>7}")

    def sweep_sub(st: list, hx: float, sy: float, ge: bool):
        """Per-frame branch TP/FP over a frame subset for `h_ag >= hx` plus a support cut.

        `ge=False` is the rule form the audit was asked for (`N(0.6 m) <= sy`); `ge=True` is
        its mirror (`N(0.6 m) >= sy`), reported so a negative result cannot be an artefact of
        guessing the support direction wrong.
        """
        cmp_sup = (lambda v, y: v >= y) if ge else (lambda v, y: v <= y)
        tp_b = [int((cmp_sup(s["pool_gt_s"], sy) & (s["pool_gt_h"] >= hx)).sum()) for s in st]
        fp_b = [int((cmp_sup(s["pool_no_s"], sy) & (s["pool_no_h"] >= hx)).sum()) for s in st]
        return tp_b, fp_b

    def sweep(hx: float, sy: float, ge: bool = False):
        return sweep_sub(allf, hx, sy, ge)

    def metrics(tp_b: list, fp_b: list) -> dict:
        tp_all = [a + b for a, b in zip(tp_cur, tp_b)]
        fp_all = [a + b for a, b in zip(fp_cur, fp_b)]
        p, r, f1 = prf(tp_all, fp_all, [s["n_gt"] - t for s, t in zip(allf, tp_all)])
        rin = macro([t / n if n else 0.0 for t, n in zip(tp_all, gt_in_cur)])
        return dict(tp_b=macro(tp_b), fp_b=macro(fp_b), r=r, rin=rin, p=p, f1=f1,
                    branch_p=sum(tp_b) / max(sum(tp_b) + sum(fp_b), 1),
                    recall_pool=macro([100.0 * t / s["pool_gt"] if s["pool_gt"] else 0.0
                                       for t, s in zip(tp_b, allf)]),
                    recall_pool_pooled=100.0 * sum(tp_b) / max(sum(s["pool_gt"] for s in allf), 1))

    def report(label: str, tp_b: list, fp_b: list) -> dict:
        m = metrics(tp_b, fp_b)
        print(f"{label:>34} {macro([a + b for a, b in zip(tp_cur, tp_b)]):>8.1f} "
              f"{macro([a + b for a, b in zip(fp_cur, fp_b)]):>8.1f} | {100*m['r']:>6.2f}% "
              f"{100*m['rin']:>8.2f}% | {100*m['p']:>6.2f}% {100*m['f1']:>6.2f}% | "
              f"{100*(m['r']-r0):>+7.2f} {100*(m['rin']-rin0):>+8.2f} {100*(m['f1']-f10):>+7.2f}")
        return m

    naive = report("naive: admit all I >= 2", *sweep(-np.inf, np.inf))
    grid = {(hx, sy): report(f"h >= {hx:.2f} m, N(0.6) <= {sy:g}", *sweep(hx, sy))
            for hx in RULE_HEIGHTS for sy in RULE_SUPPORTS}
    mirror = {(hx, sy): metrics(*sweep(hx, sy, ge=True))
              for hx in RULE_HEIGHTS for sy in RULE_SUPPORTS}
    bx, by = max(grid.items(), key=lambda kv: kv[1]["f1"])[0]
    best = grid[(bx, by)]
    mx, my = max(mirror.items(), key=lambda kv: kv[1]["f1"])[0]
    mbest = mirror[(mx, my)]

    def grid_table(title: str, table: dict, cell) -> None:
        print(f"\n  {title}\n")
        print(f"{'X \\ Y':>8} " + " ".join(f"{('inf' if np.isinf(s) else f'{s:g}'):>8}"
                                           for s in RULE_SUPPORTS))
        for hx in RULE_HEIGHTS:
            print(f"{hx:>8.2f} " + " ".join(f"{cell(table[(hx, sy)]):>8}"
                                            for sy in RULE_SUPPORTS))

    grid_table("Grid — macro F1 delta [pp] vs the current rule (rows: height >= X m, "
               "cols: support <= Y)",
               grid, lambda g: f"{100*(g['f1']-f10):+.2f}")
    grid_table("Grid — share of the in-ROI GT pool the branch admits [%]",
               grid, lambda g: f"{g['recall_pool']:.1f}")
    grid_table("Grid — false positives per frame added by the branch",
               grid, lambda g: f"{g['fp_b']:.0f}")
    grid_table("Grid — mirror: macro F1 delta [pp] for the opposite support direction "
               "(cols: support >= Y)",
               mirror, lambda g: f"{100*(g['f1']-f10):+.2f}")

    named_best = max(grid.items(), key=lambda kv: kv[1]["f1"])
    # `support >= inf` is the do-nothing cell, so it must not win the mirror comparison.
    mirror_best = max((kv for kv in mirror.items() if not np.isinf(kv[0][1])),
                      key=lambda kv: kv[1]["f1"])
    # The audit is asked about `support <= Y`; the mirror is only used if it does better,
    # so a "geometry does not help" verdict cannot come from picking the wrong direction.
    if mirror_best[1]["f1"] > named_best[1]["f1"]:
        (bx, by), best, form = (mirror_best[0][0], mirror_best[0][1]), mirror_best[1], ">="
    else:
        (bx, by), best, form = (named_best[0][0], named_best[0][1]), named_best[1], "<="

    print(f"\n  naive branch (admit every in-ROI I >= {I_POOL:g}): branch precision "
          f"{100*naive['branch_p']:.2f}% pooled, {naive['tp_b']:.0f} TP + {naive['fp_b']:.0f} FP "
          f"per frame, {naive['recall_pool_pooled']:.1f}% of the pool")
    print(f"  best cell with support <= Y (h >= {named_best[0][0]:.2f} m, Y = "
          f"{named_best[0][1]:g}): macro F1 {100*(named_best[1]['f1']-f10):+.2f} pp, branch "
          f"precision {100*named_best[1]['branch_p']:.2f}% pooled, {named_best[1]['tp_b']:.0f} TP "
          f"+ {named_best[1]['fp_b']:.0f} FP per frame")
    print(f"  best cell overall (h >= {bx:.2f} m, support {form} {by:g}): macro F1 "
          f"{100*(best['f1']-f10):+.2f} pp, branch precision {100*best['branch_p']:.2f}% pooled, "
          f"{best['tp_b']:.0f} TP + {best['fp_b']:.0f} FP per frame, "
          f"{best['recall_pool_pooled']:.1f}% of the pool")
    print(f"  geometry buys {100*(best['f1']-naive['f1']):+.2f} pp of macro F1 over the naive "
          f"branch, i.e. {(best['fp_b']-naive['fp_b'])/max(naive['fp_b'],1e-9)*100:+.1f}% FP "
          f"for {100*(best['tp_b']/max(naive['tp_b'],1e-9)-1):+.1f}% TP")

    # Per-scene stability: the cell is picked on all frames at once, so the within-scene
    # spread is the only honest overfit check available here.
    print("\n  Per-scene check of the naive branch and of the best cell (macro within each "
          "scene)\n")
    print(f"{'scene':>6} {'fr':>4} | {'naive dF1 pp':>13} {'naive br prec':>14} | "
          f"{'best dF1 pp':>12} {'best br prec':>13}")
    for scene, st in per_scene.items():
        tc = [s["tp_cur"] for s in st]
        fc = [s["fp_cur"] for s in st]
        f1c = prf(tc, fc, [s["n_gt"] - t for s, t in zip(st, tc)])[2]
        line = f"{scene:>6} {len(st):>4} |"
        for hx, sy, ge in ((-np.inf, np.inf, False), (bx, by, form == ">=")):
            tb, fb = sweep_sub(st, hx, sy, ge)
            ta = [a + b for a, b in zip(tc, tb)]
            fa = [a + b for a, b in zip(fc, fb)]
            f1 = prf(ta, fa, [s["n_gt"] - t for s, t in zip(st, ta)])[2]
            line += (f" {100*(f1-f1c):>+12.2f} "
                     f"{100*sum(tb)/max(sum(tb)+sum(fb),1):>13.2f}% |")
        print(line)

    # ---------------------------------------------------------------- bottom line
    print("\nBottom line\n")
    print(f"  pool: {stages['pool']} annotated in-ROI points at I >= {I_POOL:g} ({p_in:.2f}% of "
          f"in-ROI GT per-frame macro, {p_in_pooled:.2f}% pooled, {p_all:.2f}% of all GT); the "
          f"released rule admits none of them.")
    print(f"  oracle bound: a perfect bright classifier admits exactly that pool, so the lever "
          f"cannot exceed +{p_all:.2f} pp of recall (all-GT denominator) at zero FP cost.")
    print(f"  naive branch: recall {100*(naive['r']-r0):+.2f} pp (in-ROI {100*(naive['rin']-rin0):+.2f} pp), "
          f"precision {100*(naive['p']-p0):+.2f} pp, macro F1 {100*(naive['f1']-f10):+.2f} pp, at "
          f"{naive['fp_b']:.0f} extra FP/frame (current FP/frame {macro(fp_cur):.0f}).")
    print(f"  best geometric cell (h >= {bx:.2f} m, support {form} {by:g}): recall "
          f"{100*(best['r']-r0):+.2f} pp (in-ROI {100*(best['rin']-rin0):+.2f} pp), precision "
          f"{100*(best['p']-p0):+.2f} pp, macro F1 {100*(best['f1']-f10):+.2f} pp, at "
          f"{best['fp_b']:.0f} extra FP/frame.")
    print(f"  branch precision: naive {100*naive['branch_p']:.2f}% vs geometric "
          f"{100*best['branch_p']:.2f}% pooled.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
