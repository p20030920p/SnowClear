#!/usr/bin/env bash
# Measure the per-stage frame time of one configuration on one scene.
#
#   bash tools/measure_timing.sh released
#   bash tools/measure_timing.sh lut_off use_threshold_lut:=false
#   bash tools/measure_timing.sh elev_off use_fast_elevation_gate:=false
#
# Writes $OUT/<label>.log and $OUT/<label>.csv, which tools/gen_runtime_fig.py turns into
# Fig. 6. The scene defaults to 35 - the one Tables 1 and 5 are measured on - and OMP is
# pinned to 2 threads to match the released timings. `verbose:=true` is what makes the
# runner print the per-frame stage breakdown this reads.
#
# Note on the buckets: the stage the runner prints as "参数优化时间" is the sum of the
# feature analysis and the optimiser (cloud_operations.cpp sets param_optimize_time =
# feature_us + optimize_us), and the optimiser alone returns the defaults in ~0.0001 ms.
# The figure labels the bucket accordingly.
set +u
cd "$(dirname "$0")/.."
LABEL=${1:?usage: measure_timing.sh <label> [key:=value ...]}
shift || true
DATA=${SNOWCLEAR_DATA:?set SNOWCLEAR_DATA to a WADS mirror (see docs/DATASET.md)}
OUT=${OUT:-experiments/timing}
SCENES=${SCENES:-35}

source /opt/ros/jazzy/setup.bash
source install/setup.bash

stage=$OUT/pcd_root
mkdir -p "$stage" "$OUT"
rm -f "$stage"/*                      # the farm only ever holds symlinks this script made
for scene in $SCENES; do ln -sfn "$DATA/pcd_output/$scene" "$stage/$scene"; done

export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2} OMP_DYNAMIC=false
# This machine is shared, so a single pass can be 40 % off the next one. REPEAT runs are
# appended to the same log and averaged per frame by tools/gen_runtime_fig.py; repeat every
# configuration the same number of times, and preferably in round-robin order, so load drift
# does not land on one of them.
REPEAT=${REPEAT:-1}
echo "timing $LABEL on scene(s) $SCENES, $REPEAT run(s) -> $OUT/$LABEL.log"
: > "$OUT/$LABEL.log"
for i in $(seq 1 "$REPEAT"); do
  echo "--- run $i/$REPEAT: $LABEL $* ---" >> "$OUT/$LABEL.log"
  ros2 run snowclear_core snowclear_runner --mode eval_folders \
    --params src/snowclear_ros/config/snowclear_params.yaml \
    pcd_root:="$stage" result_root:="$DATA/result" \
    csv_path:="$OUT/$LABEL.csv" verbose:=true "$@" >> "$OUT/$LABEL.log" 2>&1
done
tail -1 "$OUT/$LABEL.log"
