#!/usr/bin/env python3
"""Write a derived mirror with the sensor mounting height changed.

The released configuration holds four constants that are absolute in the sensor frame: the ROI
bounds `z in [-1.0, 2.6]`, `r <= 17 m`, elevation `>= -23 deg`, and the support intensity floor
`I > 1.0` (docs/METHOD.md section 6). Mount the sensor 0.9 m higher on the same vehicle and every
return arrives 0.9 m lower in the sensor frame, so an absolute `z` window no longer covers the
same part of the world. That is the perturbation this writes: the point clouds are re-emitted
with `z += dz`, the ground-truth index files are symlinked because the point order - and therefore
every index - is untouched.

The transform is a rigid translation with no change in orientation and a locally flat ground; it
is a simulation of a higher mount, not a recording from one, and the figure says so.

    python3 tools/make_derived_frames.py --src <mirror> --dst <dir> --dz -0.9 --scenes 35 11 14 16

`--src` defaults to $SNOWCLEAR_DATA. The output keeps the mirror layout
(`pcd_output/<scene>/velodyne/*.pcd` plus `result/<scene>/*.txt`) so every tool in this repository
- `tools/eval_baselines.sh`, `tools/audit_error_budget.py`, the renderers - can be pointed at it
with SNOWCLEAR_DATA and nothing else.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import shutil
import sys

import numpy as np


def read_header(fh) -> tuple[str, dict]:
    """Header text (verbatim, so the derived file is byte-identical apart from the body)."""
    lines = []
    fields = {}
    while True:
        raw = fh.readline()
        if not raw:
            raise ValueError("truncated PCD header")
        line = raw.decode("ascii", "ignore").strip()
        lines.append(raw)
        if line and not line.startswith("#"):
            key, *rest = line.split()
            fields[key] = rest
        if line.startswith("DATA"):
            break
    return b"".join(lines), fields


def shift_frame(src: pathlib.Path, dst: pathlib.Path, dz: float) -> int:
    with open(src, "rb") as fh:
        header, fields = read_header(fh)
        body = fh.read()
    data_kind = fields.get("DATA", [""])[0]
    if data_kind not in ("binary", "ascii"):
        raise SystemExit(f"{src}: unsupported DATA {data_kind!r}")
    if fields.get("FIELDS") != ["x", "y", "z", "intensity"]:
        raise SystemExit(f"{src}: unexpected fields {fields.get('FIELDS')}")
    dst.parent.mkdir(parents=True, exist_ok=True)
    with open(dst, "wb") as out:
        out.write(header)
        if data_kind == "binary":
            points = np.frombuffer(body, dtype=np.float32)
            if points.size % 4:
                raise SystemExit(f"{src}: body is not a multiple of 4 floats")
            points = points.reshape(-1, 4).copy()
            points[:, 2] += np.float32(dz)
            out.write(points.astype(np.float32).tobytes())
            return points.shape[0]
        # ascii: a handful of frames in the mirror are stored this way. %.9g round-trips
        # float32 exactly, so the rewritten cloud carries the same values the reader would
        # have parsed, shifted - the text formatting is the only thing that changes.
        text = body.decode("ascii").split("\n")
        count = 0
        for line in text:
            cols = line.split()
            if len(cols) < 4:
                continue
            cols[2] = "%.9g" % (float(cols[2]) + dz)
            out.write((" ".join("%.9g" % float(c) if i < 4 else c
                                for i, c in enumerate(cols)) + "\n").encode("ascii"))
            count += 1
        return count


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", type=pathlib.Path,
                    default=pathlib.Path(os.environ.get("SNOWCLEAR_DATA", "./data")))
    ap.add_argument("--dst", type=pathlib.Path, required=True)
    ap.add_argument("--dz", type=float, default=-0.9,
                    help="z offset in metres; negative simulates a higher mount (default -0.9)")
    ap.add_argument("--scenes", nargs="+", default=None,
                    help="scenes to derive (default: every scene in the source mirror)")
    a = ap.parse_args()
    if not (a.src / "pcd_output").is_dir():
        raise SystemExit(f"no pcd_output under {a.src} (see docs/DATASET.md)")

    scenes = a.scenes or sorted(p.name for p in (a.src / "pcd_output").iterdir() if p.is_dir())
    print(f"deriving {len(scenes)} scene(s) with dz = {a.dz:+.2f} m -> {a.dst}")
    total = 0
    for scene in scenes:
        src_frames = sorted((a.src / "pcd_output" / scene / "velodyne").glob("*.pcd"))
        gt_dir = a.src / "result" / scene
        dst_gt = a.dst / "result" / scene
        dst_gt.mkdir(parents=True, exist_ok=True)
        for gt in sorted(gt_dir.glob("*.txt")):
            # the point order is untouched, so ground-truth indices carry over verbatim
            target = dst_gt / gt.name
            if target.exists() or target.is_symlink():
                target.unlink()
            target.symlink_to(gt.resolve())
        for frame in src_frames:
            out = a.dst / "pcd_output" / scene / "velodyne" / frame.name
            total += shift_frame(frame, out, a.dz)
        print(f"  scene {scene:>3}: {len(src_frames)} frames, {len(src_frames)} clouds rewritten")
    print(f"done: {total} points rewritten")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
