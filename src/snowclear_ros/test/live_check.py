#!/usr/bin/env python3
"""End-to-end check: does the live ROS 2 path reproduce the offline result?

Publishes one scan on `~/input/points`, collects `~/output/snow_indices`, and
compares the indices against the released reference output byte for byte. This
is the ROS 2 counterpart of `snowclear_runner --mode regression`: the offline
gate proves the core did not change, this one proves the message plumbing did
not either.

    python3 src/snowclear_ros/test/live_check.py \
        --pcd <frame.pcd> \
        --reference testdata/reference_042126.txt

Exit status 0 on an exact match, 1 otherwise. Run the node first, e.g.

    ros2 run snowclear_ros snowclear_node &
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32MultiArray

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from scan_to_cloud import read_pcd, to_msg  # noqa: E402


def read_reference(path: pathlib.Path) -> list[int]:
    """Ground truth / reference files are 'index,class' per line."""
    out = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        out.append(int(line.split(",")[0]))
    return out


class LiveCheck(Node):
    def __init__(self, reference: list[int], timeout_s: float) -> None:
        super().__init__("snowclear_live_check")
        self._reference = reference
        self._received: list[int] | None = None
        self._done = threading.Event()

        self._publisher = self.create_publisher(
            PointCloud2, "/snowclear/input/points", qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            Int32MultiArray, "/snowclear/output/snow_indices",
            self._on_indices, qos_profile_sensor_data)
        self._deadline = time.monotonic() + timeout_s
        self._last_publish = 0.0

    def _on_indices(self, msg: Int32MultiArray) -> None:
        self._received = list(msg.data)
        self._done.set()

    def run(self, points) -> bool:
        """Republish until the result arrives.

        A single publish races DDS discovery: the node may not have matched the
        publisher yet, and a dropped best-effort sample is simply gone. A real
        sensor streams, so the check streams too, at 2 Hz until it hears back.
        """
        msg = to_msg(points, "lidar", self.get_clock().now().to_msg())
        matched = False
        while time.monotonic() < self._deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._done.is_set():
                return True
            if not matched and self._publisher.get_subscription_count() > 0:
                matched = True
                print("发现订阅者，开始发送")
            if not matched:
                continue
            now = time.monotonic()
            if now - self._last_publish >= 0.5:
                self._last_publish = now
                msg.header.stamp = self.get_clock().now().to_msg()
                self._publisher.publish(msg)
        return False

    @property
    def received(self) -> list[int]:
        return self._received or []


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pcd", type=pathlib.Path, required=True)
    ap.add_argument("--reference", type=pathlib.Path, required=True)
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    reference = read_reference(args.reference)
    points = read_pcd(args.pcd)
    print(f"输入 {args.pcd.name}: {points.shape[0]} 点")
    print(f"参考 {args.reference.name}: {len(reference)} 个雪点索引")

    rclpy.init()
    node = LiveCheck(reference, args.timeout)
    try:
        if not node.run(points):
            print("[FAIL] 超时：没有收到 /snowclear/output/snow_indices", file=sys.stderr)
            print("       节点是否在运行？ros2 run snowclear_ros snowclear_node", file=sys.stderr)
            return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    got = node.received
    if got == reference:
        print(f"[OK] ROS 2 在线路径与参考逐位一致（{len(got)} 个索引）")
        return 0

    print(f"[FAIL] 与参考不一致：收到 {len(got)} 个索引，参考 {len(reference)} 个", file=sys.stderr)
    if len(got) != len(reference):
        print(f"       数量差异 {len(got) - len(reference):+d}", file=sys.stderr)
    else:
        for i, (a, b) in enumerate(zip(got, reference)):
            if a != b:
                print(f"       首个差异在下标 {i}: 收到 {a}, 参考 {b}", file=sys.stderr)
                break
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
