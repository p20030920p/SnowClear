#!/usr/bin/env python3
"""Publish a PCD file as sensor_msgs/PointCloud2, once or on a timer.

Lets the node be exercised without a live LiDAR:

    ros2 run snowclear_ros scan_to_cloud.py --pcd frame.pcd --rate 1.0
    ros2 run snowclear_ros scan_to_cloud.py --pcd-dir data/pcd_output/35/velodyne \\
        --rate 5.0 --topic /snowclear/input/points

The reader handles both `ascii` and `binary` PCD with an `x y z intensity`
layout, which is what the WADS mirror uses. It is deliberately dependency-light
(numpy only) so it works on a bare ROS 2 install.
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

_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def read_pcd(path: pathlib.Path) -> np.ndarray:
    """Return an (N, 4) float32 array of x, y, z, intensity."""
    with open(path, "rb") as handle:
        header: dict[str, list[str]] = {}
        while True:
            raw = handle.readline()
            if not raw:
                raise ValueError(f"{path}: truncated header")
            line = raw.decode("ascii", "ignore").strip()
            if not line or line.startswith("#"):
                continue
            key, *rest = line.split()
            header[key] = rest
            if key == "DATA":
                break

        fmt = header["DATA"][0]
        count = int(header["POINTS"][0])
        fields = header.get("FIELDS", [])

        if fmt == "binary":
            blob = handle.read(count * 16)
            data = np.frombuffer(blob, dtype=np.float32).reshape(-1, 4)
        elif fmt == "ascii":
            data = np.loadtxt(handle, dtype=np.float32, ndmin=2)[:, :4]
        else:
            raise ValueError(f"{path}: unsupported DATA format {fmt!r}")

    if fields[:4] != ["x", "y", "z", "intensity"]:
        print(f"warning: {path} field order is {fields[:4]}, assuming x y z intensity",
              file=sys.stderr)
    return np.ascontiguousarray(data.astype(np.float32))


def to_msg(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = _FIELDS
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * msg.width
    msg.is_dense = bool(np.isfinite(points).all())
    msg.data = points.tobytes()
    return msg


class ScanPublisher(Node):
    def __init__(self, scans: list[pathlib.Path], topic: str, rate: float,
                 frame_id: str, loop: bool) -> None:
        super().__init__("scan_to_cloud")
        self._scans = scans
        self._frame_id = frame_id
        self._loop = loop
        self._index = 0
        self._publisher = self.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
        period = 1.0 / rate if rate > 0 else 0.0
        self._timer = self.create_timer(max(period, 1e-3), self._tick)
        self.get_logger().info(
            f"publishing {len(scans)} scan(s) on {topic} at {rate:g} Hz, frame_id={frame_id}")

    def _tick(self) -> None:
        if self._index >= len(self._scans):
            if not self._loop:
                self.get_logger().info("all scans published, shutting down")
                rclpy.shutdown()
                return
            self._index = 0

        path = self._scans[self._index]
        self._index += 1
        points = read_pcd(path)
        stamp = self.get_clock().now().to_msg()
        self._publisher.publish(to_msg(points, self._frame_id, stamp))
        self.get_logger().info(f"published {path.name}: {points.shape[0]} points")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--pcd", type=pathlib.Path, help="single PCD file")
    src.add_argument("--pcd-dir", type=pathlib.Path, help="directory of PCD files")
    ap.add_argument("--topic", default="/snowclear/input/points")
    ap.add_argument("--rate", type=float, default=1.0, help="Hz (0 = as fast as possible)")
    ap.add_argument("--frame-id", default="lidar")
    ap.add_argument("--loop", action="store_true", help="restart after the last scan")
    ap.add_argument("--limit", type=int, default=0, help="publish at most N scans")
    args = ap.parse_args()

    scans = [args.pcd] if args.pcd else sorted(args.pcd_dir.rglob("*.pcd"))
    if args.limit:
        scans = scans[: args.limit]
    scans = [s for s in scans if s is not None]
    if not scans:
        print("no PCD files found", file=sys.stderr)
        return 2

    rclpy.init()
    node = ScanPublisher(scans, args.topic, args.rate, args.frame_id, args.loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
