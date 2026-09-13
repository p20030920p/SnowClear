#!/usr/bin/env python3
"""Evaluate CRFOR on our frames, with our ground truth and our metric.

CRFOR (Wang et al., RA-L 2023, https://github.com/dut-mdmu/CRFOR) is the spatio-temporal CRF
method this repository is compared against in the README. It is a separate implementation with its
own preprocessing, so it is not compiled into the core like DROR / DSOR / SOR / ROR; this bridge
runs it as published and scores its output the way every other number here is scored.

Three things are deliberately *not* adjusted:

  * their parameters stay at the values in their Table 1 and their `__main__` block - knn 8,
    thresholds -0.4 / 0.6, intensity ceiling 2, range ceiling 30 m, ground removal at z = -1.8
  * their output is not passed through our ROI gate: the point-wise comparison is against the same
    ground truth on the same cloud, so a method that detects outside our ROI is credited for it
  * our clouds are not de-duplicated, because the ground-truth indices refer to the cloud as
    stored; CRFOR's own loader removes duplicate returns

    git clone --depth 1 https://github.com/dut-mdmu/CRFOR.git ~/.cache/CRFOR
    python3 tools/eval_crfor.py --crfor-dir ~/.cache/CRFOR --scenes 35 --step 5 \\
        --out experiments/crfor

Writes <out>/<scene>_<frame>.txt (snow indices, same format as the reference files) and
<out>/crfor.csv, one row per frame with the point-wise metrics and the runtime. Needs scikit-learn;
the visualisation helpers of the upstream repository need open3d and are not imported.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import os
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from pcd_common import load_indices, read_pcd   # noqa: E402


def load_crfor(crfor_dir: pathlib.Path):
    """Import `CRFOR_desnow` without importing `utils` (which needs open3d)."""
    path = crfor_dir / "CRFOR.py"
    if not path.exists():
        raise SystemExit(f"no CRFOR.py under {crfor_dir}; clone the upstream repository first")
    spec = importlib.util.spec_from_file_location("crfor_upstream", path)
    module = importlib.util.module_from_spec(spec)
    # CRFOR.py does `from utils import ...` at import time; stub it so open3d is never needed
    import types
    stub = types.ModuleType("utils")
    stub.read_from_WADS = lambda *a, **k: (_ for _ in ()).throw(NotImplementedError)
    stub.analyse_and_visualize = lambda *a, **k: None
    sys.modules["utils"] = stub
    spec.loader.exec_module(module)
    return module.CRFOR_desnow


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", type=pathlib.Path,
                    default=pathlib.Path(os.environ.get("SNOWCLEAR_DATA", "./data")))
    ap.add_argument("--crfor-dir", type=pathlib.Path, required=True)
    ap.add_argument("--scenes", nargs="+", default=["35"])
    ap.add_argument("--step", type=int, default=1, help="keep every Nth frame")
    ap.add_argument("--limit", type=int, default=0, help="stop after N frames per scene (0 = all)")
    ap.add_argument("--roi", type=float, default=17.0,
                    help="radius used only for the extra in-ROI column, not for the verdict")
    ap.add_argument("--out", type=pathlib.Path, default=pathlib.Path("experiments/crfor"))
    a = ap.parse_args()
    desnow = load_crfor(a.crfor_dir)
    a.out.mkdir(parents=True, exist_ok=True)

    rows = []
    for scene in a.scenes:
        pcd_dir = a.data / "pcd_output" / scene / "velodyne"
        gt_dir = a.data / "result" / scene
        frames = sorted(pcd_dir.glob("*.pcd"))[::a.step]
        if a.limit:
            frames = frames[:a.limit]
        print(f"scene {scene}: {len(frames)} frames")
        for frame in frames:
            gt_file = gt_dir / f"{frame.stem}.txt"
            if not gt_file.exists():
                continue
            pts = read_pcd(frame)
            if not np.isfinite(pts).all(axis=1).all():
                pts = pts[np.isfinite(pts).all(axis=1)]
            t0 = time.time()
            snow = desnow(pts)                       # {index: 'f'}, as the upstream returns it
            ms = 1000.0 * (time.time() - t0)

            idx = np.fromiter(snow.keys(), dtype=np.int64, count=len(snow))
            (a.out / f"{scene}_{frame.stem}.txt").write_text(
                "\n".join(str(int(i)) for i in np.sort(idx)) + "\n", encoding="utf-8")

            gset = np.zeros(pts.shape[0], dtype=bool)
            g = load_indices(gt_file)
            gset[g[g < pts.shape[0]]] = True
            snow_mask = np.zeros(pts.shape[0], dtype=bool)
            snow_mask[idx[idx < pts.shape[0]]] = True
            x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
            r3 = np.sqrt(x * x + y * y + z * z)
            elev = np.degrees(np.arcsin(np.clip(z / np.maximum(r3, 1e-9), -1.0, 1.0)))
            roi = (z >= -1.0) & (z <= 2.6) & (x * x + y * y <= a.roi ** 2) & (elev >= -23.0)

            def scores(mask):
                tp = int((mask & gset).sum())
                fp = int((mask & ~gset).sum())
                fn = int((~mask & gset).sum())
                p = 100.0 * tp / max(tp + fp, 1)
                r = 100.0 * tp / max(tp + fn, 1)
                return p, r, (2 * p * r / (p + r) if p + r else 0.0)

            p, r, f1 = scores(snow_mask)
            pi, ri, f1i = scores(snow_mask & roi)
            rows.append(dict(scene=scene, frame=frame.stem, precision=p, recall=r, f1=f1,
                             precision_roi=pi, recall_roi=ri, f1_roi=f1i, ms=ms,
                             detected=int(snow_mask.sum()), annotated=int(gset.sum())))
            print(f"  {frame.stem}: P {p:5.2f} R {r:5.2f} F1 {f1:5.2f} | in-ROI "
                  f"P {pi:5.2f} R {ri:5.2f} F1 {f1i:5.2f} | {snow_mask.sum():6d} detections, "
                  f"{ms / 1000:5.1f} s", flush=True)

    if not rows:
        raise SystemExit("no frames evaluated")
    csv_path = a.out / "crfor.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    mean = lambda k: sum(r[k] for r in rows) / len(rows)
    print(f"\n{len(rows)} frames -> {csv_path}")
    print(f"  point-wise   P {mean('precision'):.2f}  R {mean('recall'):.2f}  F1 {mean('f1'):.2f}")
    print(f"  in-ROI       P {mean('precision_roi'):.2f}  R {mean('recall_roi'):.2f}  "
          f"F1 {mean('f1_roi'):.2f}")
    print(f"  runtime      {mean('ms') / 1000:.1f} s per frame")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
