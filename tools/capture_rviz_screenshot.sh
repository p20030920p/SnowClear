#!/usr/bin/env bash
# Screenshot the RViz layout for the README hero.
#
#   RVCFG=<cfg> bash tools/capture_rviz_screenshot.sh
#
# rviz2 first, then a light publisher, then capture. PIDs are tracked explicitly:
# no pkill anywhere, because a pattern can match this script's own command line.
set +u
cd "$(dirname "$0")/.." || exit 1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export DISPLAY=:0 ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
STAGE=${STAGE:-$HOME/.cache/snowclear_shots}; mkdir -p "$STAGE"
DATA=${SNOWCLEAR_DATA:?set SNOWCLEAR_DATA to a WADS mirror (see docs/DATASET.md)}
FRAME=$DATA/pcd_output/35/velodyne/042126.pcd
GT=$DATA/result/35/042126.txt
DET=$PWD/testdata/reference_042126.txt   # the released reference output for this frame
CFG=${RVCFG:-tools/hero_rviz.rviz}

rm -f "$STAGE"/*.xwd "$STAGE"/rviz_shot.png
echo "[1/4] rviz2"
rviz2 -d "$CFG" > "$STAGE/rviz.log" 2>&1 &
RVIZ=$!
sleep 25
echo "[2/4] static TF map->lidar (otherwise RViz warns the fixed frame does not exist)"
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id lidar \
    > "$STAGE/tf.log" 2>&1 &
TF=$!
sleep 2
echo "      publishing three clouds at 0.5 Hz for 6 cycles"
python3 tools/rviz_feed.py --pcd "$FRAME" --detection "$DET" --gt "$GT" \
    --cycles 6 --rate 0.5 \
    > "$STAGE/feed.log" 2>&1
echo "[3/4] settling"
sleep 4
WID=$(xwininfo -root -tree -display :0 2>/dev/null \
      | grep -i rviz | grep '"rviz2" "rviz2"' | grep -oE '0x[0-9a-f]+' | head -1)
echo "     window $WID"
timeout 60 xwd -id "$WID" -display :0 -silent > "$STAGE/rviz_shot.xwd" 2>&1
echo "[4/4] captured $(stat -c%s "$STAGE/rviz_shot.xwd" 2>/dev/null) bytes; closing rviz"
kill "$RVIZ" ${TF:-0} 2>/dev/null
sleep 2
python3 tools/xwd_to_png.py "$STAGE"
echo DONE
