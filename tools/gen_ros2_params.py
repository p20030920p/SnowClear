#!/usr/bin/env python3
"""Generate the ROS 2 parameter file from the canonical flat config.

`src/snowclear_ros/config/snowclear_params.yaml` is the single source of truth
for the released configuration: the offline CLI, `snowclear_runner --mode
param_check` and every piece of documentation read that file. ROS 2 wants a
nested `node: ros__parameters:` layout instead, so this script derives the ROS 2
view of the same values rather than letting a second hand-maintained copy drift.

    python3 tools/gen_ros2_params.py            # rewrite
    python3 tools/gen_ros2_params.py --check    # exit 1 if out of date
"""

from __future__ import annotations

import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
FLAT = ROOT / "src/snowclear_ros/config/snowclear_params.yaml"
ROS2 = ROOT / "src/snowclear_ros/config/snowclear_node_params.yaml"
NODE_NAME = "snowclear"

BANNER = """# =====================================================================
# GENERATED FILE -- do not edit by hand.
#
# Source of truth: config/snowclear_params.yaml (flat, used by the offline
# CLI and by `snowclear_runner --mode param_check`).
# Regenerate with:  python3 tools/gen_ros2_params.py
#
# These are the released parameters, identical to the node's compiled-in
# defaults, written out explicitly so the configuration in force is visible
# via `ros2 param dump /snowclear` and reviewable in a diff.
# =====================================================================
"""


def parse_flat(path: pathlib.Path) -> list[tuple[str, str]]:
    out: list[tuple[str, str]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        out.append((key.strip(), value.strip()))
    return out


def render(entries: list[tuple[str, str]]) -> str:
    body = "\n".join(f"    {k}: {v}" for k, v in entries)
    return f"{BANNER}\n{NODE_NAME}:\n  ros__parameters:\n{body}\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    new = render(parse_flat(FLAT))
    old = ROS2.read_text(encoding="utf-8") if ROS2.exists() else ""

    if old == new:
        print(f"  up to date  {ROS2.relative_to(ROOT)}")
        return 0
    if args.check:
        print(f"  STALE       {ROS2.relative_to(ROOT)}", file=sys.stderr)
        print("运行 `python3 tools/gen_ros2_params.py` 重新生成。", file=sys.stderr)
        return 1
    ROS2.write_text(new, encoding="utf-8")
    print(f"  regenerated {ROS2.relative_to(ROOT)} ({len(parse_flat(FLAT))} params)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
