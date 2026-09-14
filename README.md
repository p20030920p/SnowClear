<div align="center">

# SnowClear

**Training-free snow removal for spinning LiDAR via RITS**

<sub><b>R</b>ange–<b>I</b>ntensity <b>T</b>hresholding with zero-intensity <b>S</b>urface suppression</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![PCL](https://img.shields.io/badge/PCL-1.10%2B-0F9D58)](https://pointclouds.org/)
[![Platform](https://img.shields.io/badge/platform-Linux-333333?logo=linux&logoColor=white)](#quick-start)
[![Regression](https://img.shields.io/badge/byte--exact%20regression-passing-success)](#reproducibility)
[![License](https://img.shields.io/badge/license-TODO-lightgrey)](LICENSE)

[Quick start](#quick-start) &nbsp;•&nbsp; [Method](#method) &nbsp;•&nbsp; [Results](#results) &nbsp;•&nbsp; [Docs](#documentation)

*English &nbsp;|&nbsp; [中文](README_CN.md)*

</div>

![Six panels per frame: raw scan, what was removed, what was left — SnowClear above, ground truth below](docs/figures/detect_scene35.gif)

*Scene 35, 21 consecutive frames, one fixed view. Top row: SnowClear. Bottom row: ground truth.
Columns: raw scan, removed, de-snowed. Green: removed and annotated. Red: removed, not annotated.
Blue: annotated, kept.*

SnowClear is a snow-removal library for spinning LiDAR point clouds. It runs on CPU at about 10 ms
per frame, with no training and no learned weights.

The core does not depend on ROS. It can be called offline or run as a ROS 2 node.

| | |
|---|---|
| **Accuracy** — 16 scenes, 1 620 frames | P 96.69 · R 89.98 · **F1 92.82** |
| **Speed** — Release build, CPU only | **≈ 10 ms** per frame |
| **Against the best non-learned baseline** | **2.4×** its F1 (SOR 37.93) |
| **Learning** | none |
| **Output** | snow indices in the input cloud's own index space |

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

Offline, no ROS graph involved:

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

All algorithm parameters are ROS 2 parameters; their defaults are the released values.
`ros2 param set /snowclear score_threshold 0.6` applies to the next scan. Replay a PCD without a
sensor: `ros2 run snowclear_ros scan_to_cloud.py --pcd frame.pcd --rate 1.0`. Node reference:
[`docs/ROS2.md`](docs/ROS2.md).

## Method

Four tests per point, in this order; each sees only what the previous kept.

1. **ROI gate** — `z ∈ [−1.0, 2.6] m`, `r ≤ 17 m`, elevation `≥ −23°`.
2. **Range–intensity score** — weak returns score high: `s = (1 − I/T)^1.2`. `T(r, I)` drops where
   beam density peaks.
3. **Surface veto** — a zero return within 0.6 m of a bright point beyond 7 m is rejected.
4. **Decision** — `C = 0.7·s + 0.15·h_ag` above `θ`. The height term is capped at 0.15.

![Algorithm 1: the per-frame detection loop](docs/figures/algorithm1_en.png)

*Algorithm 1. Constants and disabled features: [`docs/METHOD.md`](docs/METHOD.md).*

## Results

Release build, `OMP_NUM_THREADS=2`; timing excludes I/O and evaluation. Macro over scenes.
Reproduction commands: [`docs/figures/README.md`](docs/figures/README.md).

![Seven methods on one frame, bird's-eye view](docs/figures/fig12_baselines.png)

*Frame `042126`, one window and one ground truth for every panel. CRFOR (Wang et al., RA-L 2023)
runs as published, with its own preprocessing and parameters. The four filters and SnowClear share
ours.*

![Precision, recall and F1 per method on scene 35](docs/figures/fig16_scores.png)

*Scene 35, 101 frames, in-ROI. Latency from the idle single-scene run.*

### Scene 35 — every method on the same 101 frames

| Method | Precision | Recall | F1 | ms / frame |
|---|---:|---:|---:|---:|
| **SnowClear** | **96.79** | **97.07** | **96.90** | **≈ 9** |
| CRFOR (Wang et al., RA-L 2023) | 95.95 | 96.92 | 96.41 | 17 132 |
| DROR (Charron et al., CRV 2018) | 82.45 | 6.76 | 12.48 | 1041 |
| DSOR (Kurup & Bos, 2021) | 81.58 | 6.93 | 12.73 | 70 |
| SOR (Rusu et al., 2008) | 82.87 | 44.28 | 56.68 | 110 |
| ROR (Rusu, 2009) | 77.13 | 1.86 | 3.63 | 1013 |

CRFOR's numbers come from its own repository via `tools/eval_crfor.py`; the others from
`SCENES=35 bash tools/eval_baselines.sh`.

### The 16-scene reported set — SnowClear against the four built-in filters

| Method | Precision | Recall | F1 | ms / frame |
|---|---:|---:|---:|---:|
| DROR (Charron et al., CRV 2018) | 85.54 | 3.57 | 6.80 | 1041 |
| DSOR (Kurup & Bos, 2021) | 85.02 | 3.88 | 7.36 | 70 |
| SOR (Rusu et al., 2008) | 89.24 | 25.56 | 37.93 | 110 |
| ROR (Rusu, 2009) | 79.70 | 0.89 | 1.76 | 1013 |
| **SnowClear (RITS)** | **96.69** | **89.97** | **92.82** | **≈ 10** |

A 1 620-frame CRFOR run takes hours at ~17 s per frame, so it is not in this table.

![Where the ground truth goes](docs/figures/fig15_budget.png)

*Ground-truth budget for the released configuration: each annotated point is charged to the first
stage that rejects it. Green is the recall ceiling (89.90 % on the reported set, 89.98 % measured); on
scene 16 the ROI gate takes 40.5 % and the intensity ceiling 14.6 %.*

More figures: [`docs/figures/README.md`](docs/figures/README.md).

## Reproducibility

```bash
SNOWCLEAR_DATA=/path/to/wads-mirror bash tools/verify.sh   # clean build + all gates
```

Three gates: unit tests (`colcon test`), the offline byte-exact gate (`--mode all_checks`) and the
live byte-exact gate (`src/snowclear_ros/test/live_check.py`). A change that can alter detection
turns the second one red. Data layout: [`docs/DATASET.md`](docs/DATASET.md).

## Documentation

- Method and shipped constants: [`docs/METHOD.md`](docs/METHOD.md)
- Node reference: [`docs/ROS2.md`](docs/ROS2.md)
- Package layout: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)
- Measurements, ablations and roadmap: [`docs/OPTIMIZATION.md`](docs/OPTIMIZATION.md)
- Figure index and regeneration commands: [`docs/figures/README.md`](docs/figures/README.md)
- Differences from the ROS 1 build: [`docs/MIGRATION_ROS1.md`](docs/MIGRATION_ROS1.md)

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

Not chosen yet; see [`LICENSE`](LICENSE). The non-learned baselines in
`dynamic_outlier_filters.cpp` follow [DROR](https://github.com/nickcharron/lidar_snow_removal)
(Charron et al., CRV 2018) and [DSOR](https://github.com/assasinXL/dsor_filter) (Kurup & Bos, 2021).

## Contact

Open an issue for bugs or reproduction failures. Include the output of `--mode all_checks` and your
`OMP_NUM_THREADS`.
