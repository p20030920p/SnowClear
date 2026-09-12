#!/usr/bin/env python3
"""Publish the three clouds RViz needs, straight from files.

The live pipeline (node + replay) costs far more than this: the node reprocesses a
208 k-point scan every cycle. RViz only needs the three messages, so this reads the
scan, the ground truth and the saved detection indices and publishes:

    /snowclear/input/points        raw scan
    /snowclear/output/points       de-snowed (raw minus detected)
    /snowclear/output/snow_points  the removed points

Publishing is time-limited (--cycles) so the desktop is left alone afterwards.
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from pcd_common import read_pcd, load_indices   # noqa: E402

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def to_msg(pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    m = PointCloud2()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.height = 1
    m.width = int(pts.shape[0])
    m.fields = FIELDS
    m.is_bigendian = False
    m.point_step = 16
    m.row_step = 16 * m.width
    m.is_dense = bool(np.isfinite(pts).all())
    m.data = np.ascontiguousarray(pts.astype(np.float32)).tobytes()
    return m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pcd", required=True)
    ap.add_argument("--detection", required=True)
    ap.add_argument("--rate", type=float, default=0.5)
    ap.add_argument("--cycles", type=int, default=6)
    ap.add_argument("--frame-id", default="lidar")
    ap.add_argument("--gt", default="", help="ground-truth index file; enables TP/FN/FP")
    a = ap.parse_args()

    pts = read_pcd(pathlib.Path(a.pcd))
    keep = finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    det = load_indices(pathlib.Path(a.detection))
    det = det[det < pts.shape[0]]
    snow = np.zeros(pts.shape[0], dtype=bool)
    snow[det] = True
    # Four classes, so the RViz display names are the legend:
    #   structure  ROI points that are neither annotated snow nor detected
    #   tp         detected and annotated          fn  annotated, missed
    #   fp         detected, not annotated
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r3 = np.sqrt(x * x + y * y + z * z)
    elev = np.degrees(np.arcsin(np.clip(z / np.maximum(r3, 1e-9), -1.0, 1.0)))
    roi = (z >= -1.0) & (z <= 2.6) & (x * x + y * y <= 17.0 ** 2) & (elev >= -23.0)

    gt_file = getattr(a, "gt", None)
    if gt_file:
        gt = load_indices(pathlib.Path(gt_file))
        gt = gt[gt < pts.shape[0]]
        gset = np.zeros(pts.shape[0], dtype=bool); gset[gt] = True
        cls = {"structure": roi & ~snow & ~gset, "tp": roi & snow & gset,
               "fn": roi & gset & ~snow, "fp": roi & snow & ~gset}
    else:                                   # no labels: keep the two-class split
        cls = {"structure": roi & ~snow, "tp": snow, "fn": np.zeros_like(snow),
               "fp": np.zeros_like(snow)}
    raw, desnowed, removed = pts, pts[cls["structure"]], pts[snow]
    print(f"raw {raw.shape[0]}  de-snowed {desnowed.shape[0]}  removed {removed.shape[0]}")

    rclpy.init()
    node = Node("rviz_feed")
    topics = ["/snowclear/output/points", "/snowclear/hero/tp",
              "/snowclear/hero/fn", "/snowclear/hero/fp"]
    clouds = [cls["structure"], cls["tp"], cls["fn"], cls["fp"]]
    pubs = tuple(node.create_publisher(PointCloud2, t, qos_profile_sensor_data) for t in topics)
    for t, c in zip(topics, clouds):
        print(f"    {t:34s} {c.sum():6d} points")
    period = 1.0 / a.rate if a.rate > 0 else 0.0
    for i in range(a.cycles):
        stamp = node.get_clock().now().to_msg()
        for pub, cloud in zip(pubs, [pts[c] for c in clouds]):
            pub.publish(to_msg(cloud, a.frame_id, stamp))
        print(f"  cycle {i + 1}/{a.cycles}", flush=True)
        time.sleep(period)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
