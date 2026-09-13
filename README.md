<div align="center">

# SnowClear

**Training-free snow removal for spinning LiDAR via RITS**

<sub><b>R</b>ange–<b>I</b>ntensity <b>T</b>hresholding with zero-intensity <b>S</b>urface suppression &nbsp;·&nbsp; real-time &nbsp;·&nbsp; CPU-only &nbsp;·&nbsp; no learned weights</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![PCL](https://img.shields.io/badge/PCL-1.10%2B-0F9D58)](https://pointclouds.org/)
[![Platform](https://img.shields.io/badge/platform-Linux-333333?logo=linux&logoColor=white)](#quick-start)
[![Regression](https://img.shields.io/badge/byte--exact%20regression-passing-success)](#reproducibility)
[![License](https://img.shields.io/badge/license-TODO-lightgrey)](LICENSE)

[Quick start](#quick-start) &nbsp;•&nbsp; [Method](#method) &nbsp;•&nbsp; [Results](#results) &nbsp;•&nbsp; [Docs](#documentation)

*English &nbsp;|&nbsp; [中文](README_CN.md)*

</div>

![One scene frame by frame: raw scan, detections coloured by outcome, and the de-snowed cloud](docs/figures/detect_scene35.gif)

*One scene, 26 consecutive frames, one fixed view: **raw scan → detection → de-snowed**. Green is
flagged and annotated (TP), red is flagged without an annotation (FP), blue is annotated but missed
(FN); the header carries that frame's in-ROI precision / recall / F1. The snow moves, the method
does not.*

**SnowClear removes snowfall noise from a LiDAR scan point by point, at ≈10 ms per frame on CPU,
with no training and no learned weights.** A raw frame goes in; the de-snowed cloud and the snow
indices come out in the index space of the cloud you passed in. The core links only PCL, OpenMP and
TBB — never ROS — so the same library runs offline and as a ROS 2 node, and the two produce
bit-identical output, which two byte-exact gates enforce rather than assume.

| | |
|---|---|
| **Accuracy** — 16 scenes, 1 620 frames | P 96.69 · R 89.98 · **F1 92.82** |
| **Speed** — Release build, CPU only | **≈ 10 ms** per frame, no GPU |
| **Against the best non-learned baseline** | **2.4×** its F1 (SOR 37.93), 9× faster |
| **Learning** | none — no weights, no dataset to download |
| **Output** | snow indices in the input cloud's own index space |

---

## Quick start

| Component | Version |
|---|---|
| OS / ROS | Ubuntu 24.04 (tested) · ROS 2 Jazzy |
| PCL | 1.10+ (`common`, `filters`, `io`, `kdtree`, `search`); `pcl_ros` is **not** needed |
| Toolchain | C++17 (g++ 9+), TBB, OpenMP, Eigen, `yaml-cpp` (CLI only) |

```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/p20030920p/SnowClear.git && cd SnowClear
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release   # -O0 misrepresents runtime ~10x
source install/setup.bash
```

Offline — no ROS graph involved:

```bash
# one frame
ros2 run snowclear_core snowclear_cli pcd_file:=/path/frame.pcd result_folder:=/path/gt

# a folder, writing the snow indices
ros2 run snowclear_core snowclear_cli process_all_frames:=true pcd_folder:=/path/scans \
  result_folder:=/path/gt save_results:=true output_dir:=/tmp/out
```

As a ROS 2 node:

```bash
ros2 launch snowclear_ros snowclear.launch.py rviz:=true
```

| Topic | Type | |
|---|---|---|
| `~/input/points` | `sensor_msgs/PointCloud2` | subscribe |
| `~/output/points` | `sensor_msgs/PointCloud2` | the de-snowed cloud |
| `~/output/snow_points` | `sensor_msgs/PointCloud2` | what was removed |
| `~/output/snow_indices` | `std_msgs/Int32MultiArray` | indices into the input cloud |

Every algorithm parameter is a ROS 2 parameter whose default *is* the released value, so
`ros2 param set /snowclear score_threshold 0.6` takes effect on the next scan and a params file is
only ever an override. Replay a PCD with no sensor: `ros2 run snowclear_ros scan_to_cloud.py
--pcd frame.pcd --rate 1.0`. Details in [`docs/ROS2.md`](docs/ROS2.md).

---

## Method

Each point is tested in this order, and every step only sees what the previous one kept:

1. **ROI gate** — keep `z ∈ [−1.0, 2.6] m`, `r ≤ 17 m`, elevation `≥ −23°`.
2. **Range–intensity score** — a *weak* return scores high: `s = (1 − I/T)^1.2`, against a threshold
   `T(r, I)` that a Gamma-shaped weight pulls down where beam density peaks.
3. **Zero-intensity surface veto** — a zero return within 0.6 m of a bright point, 7 m out, is a
   surface artefact and not snow.
4. **Fused decision** — `C = 0.7·s + 0.15·h_ag`, accepted above `θ`; the height term is capped at
   0.15 so geometry can never outvote intensity.

![Algorithm 1: the per-frame detection loop](docs/figures/algorithm1_en.png)

*Algorithm 1 — the whole detector, 25 lines. Derivation, disabled features and the exact constants:
[`docs/METHOD.md`](docs/METHOD.md).*

---

## Results

Release build, `OMP_NUM_THREADS=2`, I/O and evaluation excluded from the timing; macro average over
scenes on 16 scenes / 1 620 frames of [WADS](https://digitalcommons.mtu.edu/wads/). Every figure
below is regenerated by a tool in this repository — the commands are in
[`docs/figures/README.md`](docs/figures/README.md).

![Precision, recall and F1 of SnowClear against the four non-learned baselines, macro over the reported set](docs/figures/fig3_comparison.png)

*Against non-learned baselines — the four filters are compiled into the core and share the identical
ROI gate, index mapping and evaluation path, so the comparison isolates the decision rule.*

| Method | Precision | Recall | F1 | ms / frame |
|---|---:|---:|---:|---:|
| DROR (Charron et al., CRV 2018) | 85.54 | 3.57 | 6.80 | 1041 |
| DSOR (Kurup & Bos, 2021) | 85.02 | 3.88 | 7.36 | 70 |
| SOR (Rusu et al., 2008) | 89.24 | 25.56 | 37.93 | 110 |
| ROR (Rusu, 2009) | 79.70 | 0.89 | 1.76 | 1013 |
| **SnowClear (RITS)** | **96.69** | **89.97** | **92.82** | **≈ 10** |

The density filters are precise only because they return almost nothing — under 4 % recall. Reproduce
the whole table with `bash tools/eval_baselines.sh <outdir>`.

![The same frame under five detectors: ground truth, SnowClear, DROR, DSOR, SOR and ROR](docs/figures/fig12_baselines.png)

*One frame, five detectors, one camera: the density filters leave the annotated snow untouched
(blue), SOR floods it with false positives (red), SnowClear is the panel that matches the ground
truth.*

![Released constants against the label-free self-calibration, on the reference mount and with the sensor 0.9 m higher](docs/figures/fig7_cross_sensor.png)

*Portability, measured: mounting the sensor 0.9 m higher costs the released constants **24.25 pp** of
macro F1 — recall halves while precision holds — and the label-free self-calibration 0.06 pp.*

![Where the recall goes: the ground-truth budget by scene](docs/figures/fig8_gt_ceiling.png)

*Where the recall goes: 89.90 % of ground truth stays reachable over the reported set, 7.38 % falls
outside the ROI gate and 2.12 % above the intensity ceiling. The measured recall of 89.98 % sits
within 0.1 pp of that reachable band, so the remaining loss is the gate and the ceiling, not the
decision rule.*

---

## Reproducibility

```bash
SNOWCLEAR_DATA=/path/to/wads-mirror bash tools/verify.sh   # clean build + all gates
```

Three gates, all cheap: unit tests (`colcon test`), the offline byte-exact gate
(`snowclear_runner --mode all_checks`, reproduces the released reference output byte for byte) and
the live byte-exact gate (`src/snowclear_ros/test/live_check.py`, proves the ROS 2 path emits the
same indices). Any change that can alter detection must turn the second one red. Data layout and
ground-truth format: [`docs/DATASET.md`](docs/DATASET.md).

---

## Documentation

Method and shipped constants [`docs/METHOD.md`](docs/METHOD.md) &nbsp;·&nbsp; node reference
[`docs/ROS2.md`](docs/ROS2.md) &nbsp;·&nbsp; package split
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) &nbsp;·&nbsp; measurements, ablations and the
prioritised roadmap [`docs/OPTIMIZATION.md`](docs/OPTIMIZATION.md) &nbsp;·&nbsp; figure index
[`docs/figures/README.md`](docs/figures/README.md) &nbsp;·&nbsp; ROS 1 differences
[`docs/MIGRATION_ROS1.md`](docs/MIGRATION_ROS1.md).

## Citation

```bibtex
@article{snowclear,
  title   = {SnowClear: Training-Free Snow-Point Detection and Removal for Spinning LiDAR
             via Range--Intensity Thresholding and Zero-Intensity Surface Suppression},
  author  = {TODO: authors},
  journal = {TODO: venue},
  year    = {TODO: year},
  url     = {https://github.com/p20030920p/SnowClear}
}
```

## License

Not yet chosen — see [`LICENSE`](LICENSE). Third-party material keeps its upstream terms: the
non-learned baselines in `dynamic_outlier_filters.cpp` follow
[DROR](https://github.com/nickcharron/lidar_snow_removal) (Charron et al., CRV 2018) and
[DSOR](https://github.com/assasinXL/dsor_filter) (Kurup & Bos, 2021).

## Contact

Questions, bugs and reproduction failures: please open an issue — include the output of
`--mode all_checks` and your `OMP_NUM_THREADS`.
