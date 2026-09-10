#!/usr/bin/env bash
# End-to-end verification of SnowClear. Run from the repository root.
set +u
cd "$(dirname "$0")/.."
ROOT=$PWD
source /opt/ros/jazzy/setup.bash

echo "=== 1. clean colcon build ==="
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | grep -E "Finished|Failed|Summary|error:" | head

source install/setup.bash
DATA=${SNOWCLEAR_DATA:-/home/qzl/workspace/Snow_Removal/Snow_point/src/clustering/data}
FRAME=$DATA/pcd_output/35/velodyne/042126.pcd
PARAMS=src/snowclear_ros/config/snowclear_params.yaml

echo
echo "=== 2. generated artefacts up to date ==="
python3 tools/gen_param_map.py --check
python3 tools/gen_ros2_params.py --check

echo
echo "=== 3. unit tests ==="
./build/snowclear_core/test/test_core 2>&1 | tail -2
./build/snowclear_ros/test/test_node 2>&1 | tail -2

echo
echo "=== 4. offline byte-exact gate ==="
OMP_NUM_THREADS=2 OMP_DYNAMIC=false ros2 run snowclear_core snowclear_runner \
  --mode all_checks --params "$PARAMS" pcd_file:="$FRAME" \
  reference_file:="$ROOT/testdata/reference_042126.txt" output_dir:=/tmp/sc_final 2>&1 \
  | grep -E "\[OK\]|\[FAIL\]"

echo
echo "=== 5. live ROS 2 byte-exact gate ==="
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
ros2 run snowclear_ros snowclear_node > /tmp/sc_final_node.log 2>&1 &
NODE_PID=$!
sleep 6
timeout 90 python3 src/snowclear_ros/test/live_check.py --pcd "$FRAME" \
  --reference "$ROOT/testdata/reference_042126.txt" 2>&1 | grep -E "\[OK\]|\[FAIL\]|发现订阅者"
kill "$NODE_PID" 2>/dev/null
echo "node log:"; grep -E "判为雪" /tmp/sc_final_node.log | head -1
