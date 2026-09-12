#!/usr/bin/env bash
# Evaluate SnowClear and the compiled-in non-learned baselines over the reported set.
#
# The reported set is 16 of the 19 mirrored WADS scenes - all but 14, 16 and 76, the
# three scenes reported separately in README Table 4 - which is 1 620 frames. They are
# staged as a symlink farm because `eval_folders` walks every directory it finds under
# `pcd_root`, so the evaluated set has to be the set on disk.
#
#   SNOWCLEAR_DATA=<mirror> bash tools/eval_baselines.sh [outdir] [method ...]
#
# `method` is any of feature_fusion (the released configuration), dror, dsor, sor, ror;
# with no method given all five run. Writes <outdir>/<method>.csv (one row per scene,
# the columns snowclear_runner emits), <outdir>/<method>.log, and prints the macro
# average over scenes plus the F1 recomputed from that mean P / R.
set +u
cd "$(dirname "$0")/.."
ROOT=$PWD
DATA=${SNOWCLEAR_DATA:?set SNOWCLEAR_DATA to a WADS mirror (see docs/DATASET.md)}
OUT=${1:-$ROOT/experiments/baselines}; shift || true
METHODS=("$@"); [ ${#METHODS[@]} -eq 0 ] && METHODS=(feature_fusion dror dsor sor ror)
EXCLUDE=${EXCLUDE:-14 16 76}          # the scenes outside the reported set
SCENES=${SCENES:-}                    # restrict to a scene list, e.g. SCENES=35 for a
                                      # single-scene latency run on an idle machine
PARAMS=src/snowclear_ros/config/snowclear_params.yaml

if [[ ! -x build/snowclear_core/snowclear_runner && ! -f install/setup.bash ]]; then
  echo "no build: run 'colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release' first" >&2
  exit 1
fi
[[ -f $DATA/pcd_output/35/velodyne/042126.pcd ]] || { echo "no dataset at $DATA" >&2; exit 1; }

source /opt/ros/jazzy/setup.bash
source install/setup.bash

STAGE=$OUT/pcd_root
mkdir -p "$STAGE" "$OUT"
rm -f "$STAGE"/*                      # the farm only ever holds symlinks this script made
for scene in $(ls "$DATA/pcd_output"); do
  if [[ -n $SCENES ]]; then
    keep=0; for s in $SCENES; do [[ $scene == "$s" ]] && keep=1; done
    [[ $keep == 1 ]] || continue
  fi
  skip=0
  for x in $EXCLUDE; do [[ $scene == "$x" ]] && skip=1; done
  [[ $skip == 1 ]] || ln -sfn "$DATA/pcd_output/$scene" "$STAGE/$scene"
done
frames=$(ls "$STAGE"/*/velodyne/*.pcd 2>/dev/null | wc -l)
scenes=$(ls "$STAGE" | wc -l)
echo "reported set: $scenes scenes / $frames frames  (excluding:$EXCLUDE)"

# OMP is pinned to 2 threads in the released timings, and the frame count makes the
# numbers comparable across methods: same frames, same ROI gate, same evaluation path.
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2} OMP_DYNAMIC=false

for method in "${METHODS[@]}"; do
  echo
  echo "=== $method ==="
  ros2 run snowclear_core snowclear_runner --mode eval_folders --params "$PARAMS" \
    pcd_root:="$STAGE" result_root:="$DATA/result" \
    csv_path:="$OUT/$method.csv" detector_type:="$method" > "$OUT/$method.log" 2>&1 \
    || echo "[FAIL] $method (see $OUT/$method.log)"
  tail -1 "$OUT/$method.log" 2>/dev/null | cut -c1-120
done

python3 - "$OUT" "${METHODS[@]}" <<'PY'
import csv, pathlib, sys
out = pathlib.Path(sys.argv[1])
print(f"\n{'method':16s} {'scenes':>6s} {'precision':>10s} {'recall':>8s} {'F1':>8s} "
      f"{'F1(P,R)':>8s} {'ms/frame':>9s}")
for method in sys.argv[2:]:
    p = out / f"{method}.csv"
    if not p.exists():
        print(f"{method:16s}  (no csv)")
        continue
    rows = list(csv.DictReader(p.open()))
    if not rows:
        print(f"{method:16s}  (empty)")
        continue
    n = len(rows)
    mean = lambda k: sum(float(r[k]) for r in rows) / n
    p_, r_, f1 = mean("precision"), mean("recall"), mean("f1")
    ms = sum(float(r["time_ms"]) for r in rows) / n
    print(f"{method:16s} {n:6d} {p_:10.4f} {r_:8.4f} {f1:8.4f} "
          f"{2 * p_ * r_ / (p_ + r_) if p_ + r_ else 0:8.4f} {ms:9.1f}")
PY
