#!/usr/bin/env bash
# The portability experiment behind Fig. 7: what the label-free self-calibration is worth when
# the sensor moves.
#
#   SNOWCLEAR_DATA=<mirror> bash tools/measure_portability.sh [outdir] [scenes...]
#
# Four runs on the same frames: {released, self-calibrated} x {reference mirror, +0.9 m mount}.
# The perturbed mirror is derived on the spot with tools/make_derived_frames.py (z += -0.9 m, the
# ground-truth indices carry over because the point order does not change) and the released
# configuration's four absolute constants - z bounds, radial bound, elevation gate, support
# intensity floor - are what it breaks.
#
# Writes <outdir>/<dataset>_<config>.csv, one row per scene, plus a .log per run.
set +u
cd "$(dirname "$0")/.."
DATA=${SNOWCLEAR_DATA:?set SNOWCLEAR_DATA to a WADS mirror (see docs/DATASET.md)}
OUT=${1:-experiments/portability}; shift || true
SCENES=("$@"); [ ${#SCENES[@]} -eq 0 ] && SCENES=(35 11 14 16)
SHIFTED=${SHIFTED:-$OUT/shifted}
PARAMS=src/snowclear_ros/config/snowclear_params.yaml
# the documented path for a new sensor: the three switches that are off by default
SELFCAL="enable_sensor_height_roi:=true enable_range_selfcal:=true enable_sparse_support_expand:=true"

source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2} OMP_DYNAMIC=false
mkdir -p "$OUT"

if [[ ! -f $SHIFTED/.complete ]]; then
  echo "=== deriving the +0.9 m mirror ==="
  python3 tools/make_derived_frames.py --src "$DATA" --dst "$SHIFTED" --dz -0.9 \
    --scenes "${SCENES[@]}" || exit 1
  touch "$SHIFTED/.complete"
fi

stage_of() {                       # $1 = dataset, $2 = mirror root
  local stage=$OUT/stage_$1
  mkdir -p "$stage"; rm -f "$stage"/*
  for scene in "${SCENES[@]}"; do ln -sfn "$2/pcd_output/$scene" "$stage/$scene"; done
  echo "$stage"
}

echo
echo "scene subset: ${SCENES[*]}   (${#SCENES[@]} scenes)"
printf '%-24s %s\n' "dataset / config" "macro F1"
for dataset in reference shifted; do
  if [[ $dataset == reference ]]; then mirror=$DATA; else mirror=$SHIFTED; fi
  stage=$(stage_of "$dataset" "$mirror")
  for config in released selfcal; do
    extra=""; [[ $config == selfcal ]] && extra=$SELFCAL
    ros2 run snowclear_core snowclear_runner --mode eval_folders --params "$PARAMS" \
      pcd_root:="$stage" result_root:="$mirror/result" \
      csv_path:="$OUT/${dataset}_${config}.csv" detector_type:=feature_fusion $extra \
      > "$OUT/${dataset}_${config}.log" 2>&1 || echo "[FAIL] $dataset/$config"
  done
done

python3 - "$OUT" <<'PY'
import csv, pathlib, statistics, sys
out = pathlib.Path(sys.argv[1])
print(f"\n{'dataset':>10} {'config':>9} {'scenes':>7} {'precision':>10} {'recall':>8} {'F1':>8} {'ms/frame':>9}")
for dataset in ("reference", "shifted"):
    for config in ("released", "selfcal"):
        p = out / f"{dataset}_{config}.csv"
        rows = list(csv.DictReader(p.open())) if p.exists() else []
        if not rows:
            print(f"{dataset:>10} {config:>9}  (no csv)"); continue
        mean = lambda k: statistics.fmean(float(r[k]) for r in rows)
        print(f"{dataset:>10} {config:>9} {len(rows):>7} {mean('precision'):>10.2f} "
              f"{mean('recall'):>8.2f} {mean('f1'):>8.2f} {mean('time_ms'):>9.1f}")
PY
