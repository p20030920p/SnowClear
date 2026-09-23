<div align="center">

# SVOR

**<b>S</b>urface-<b>V</b>eto <b>O</b>utlier <b>R</b>emoval**

Training-free snow removal for spinning LiDAR

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![PCL](https://img.shields.io/badge/PCL-1.10%2B-0F9D58)](https://pointclouds.org/)
[![Platform](https://img.shields.io/badge/platform-Linux-333333?logo=linux&logoColor=white)](#quick-start)

[Quick start](#quick-start) &nbsp;•&nbsp; [Method](#method) &nbsp;•&nbsp; [Results](#results) &nbsp;•&nbsp; [Status](#status)

*English &nbsp;|&nbsp; [中文](README_CN.md)*

</div>

![Six panels per frame: raw scan, what was removed, what was left — SVOR above, ground truth below](docs/figures/detect_scene35.gif)

*Scene 35, 21 consecutive frames, one fixed view. Top row: SVOR. Bottom row: ground truth.
Columns: raw scan, removed, de-snowed. Green: removed and annotated. Red: removed, not annotated.
Blue: annotated, kept.*

SVOR removes snowfall from spinning LiDAR point clouds with no training and no learned weights. One
frame costs about 10 ms on CPU, and the core does not depend on ROS: call it offline, or run it as a
ROS 2 node. `SnowClear` is the reference implementation, and the name of this repository.

A snowfall return is weak, usually zero-intensity, and sits where real structure is. Left in, those
points reach scan matching and mapping as if they were part of the scene; SVOR removes them first.

| | |
|---|---|
| **Accuracy** — 16 scenes, 1 620 frames (WADS) | P 96.69 · R 89.98 · **F1 92.82** |
| **Speed** — CPU only, no GPU | **≈ 10 ms** per frame |
| **Measured on** | thin-and-light laptop: Huawei MateBook 14 (2022) · Intel Core i5-1240P (12 cores / 16 threads, 28 W) · Ubuntu 24.04 · Release build · `OMP_NUM_THREADS=2`, i.e. two of its sixteen threads |
| **Same 101 frames against CRFOR (Wang et al., RA-L 2023)** | F1 96.90 vs 96.41, at **≈ 1 900×** its speed (9 ms vs 17 132 ms per frame, same machine) |
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
ros2 run snowclear_core snowclear_cli pcd_file:=/path/frame.pcd result_folder:=/path/gt
# a folder: process_all_frames:=true pcd_folder:=/path/scans save_results:=true output_dir:=/tmp/out
```

As a ROS 2 node — `PointCloud2` in, the de-snowed cloud, the removed points and the snow indices
out, every algorithm parameter declared as a ROS 2 parameter:

```bash
ros2 launch snowclear_ros snowclear.launch.py rviz:=true
```

Topics, parameters, launch arguments and diagnostics: [`docs/ROS2.md`](docs/ROS2.md).

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

Macro over scenes; timing excludes I/O and evaluation. All of it was measured on one thin-and-light
laptop (Huawei MateBook 14 2022, Intel Core i5-1240P, Release build, no GPU, `OMP_NUM_THREADS=2`) in
idle runs: the ratio between two methods timed the same way travels, the absolute milliseconds do
not. Reproduction commands: [`docs/figures/README.md`](docs/figures/README.md).

### Scene 35 — every method on the same 101 frames

| Method | Precision | Recall | F1 | ms / frame |
|---|---:|---:|---:|---:|
| **SVOR** | **96.79** | **97.07** | **96.90** | **≈ 9** |
| CRFOR (Wang et al., RA-L 2023) | 95.95 | 96.92 | 96.41 | 17 132 |
| DROR (Charron et al., CRV 2018) | 82.45 | 6.76 | 12.48 | 1041 |
| DSOR (Kurup & Bos, 2021) | 81.58 | 6.93 | 12.73 | 70 |
| SOR (Rusu et al., 2008) | 82.87 | 44.28 | 56.68 | 110 |
| ROR (Rusu, 2009) | 77.13 | 1.86 | 3.63 | 1013 |

*CRFOR runs as published, with its own preprocessing and parameters; the four filters and SVOR
share ours. CRFOR's numbers come from its own repository via `tools/eval_crfor.py`, the others from
`SCENES=35 bash tools/eval_baselines.sh`.*

![Seven methods on one frame, bird's-eye view](docs/figures/fig12_baselines.png)

*Frame `042126`, one window and one ground truth for every panel.*

### The 16-scene reported set — SVOR against the four built-in filters

| Method | Precision | Recall | F1 | ms / frame |
|---|---:|---:|---:|---:|
| DROR (Charron et al., CRV 2018) | 85.54 | 3.57 | 6.80 | 1041 |
| DSOR (Kurup & Bos, 2021) | 85.02 | 3.88 | 7.36 | 70 |
| SOR (Rusu et al., 2008) | 89.24 | 25.56 | 37.93 | 110 |
| ROR (Rusu, 2009) | 79.70 | 0.89 | 1.76 | 1013 |
| **SVOR** | **96.69** | **89.97** | **92.82** | **≈ 10** |

*A 1 620-frame CRFOR run takes hours at ~17 s per frame, so it is not in this table. What these
numbers measure, and where the remaining error sits, is in [`docs/METHOD.md`](docs/METHOD.md) and
[`docs/OPTIMIZATION.md`](docs/OPTIMIZATION.md).*

![Precision, recall and F1 per method on scene 35](docs/figures/fig16_scores.png)

*Scene 35, 101 frames, in-ROI.*

## Reproducibility

```bash
SNOWCLEAR_DATA=/path/to/wads-mirror bash tools/verify.sh   # clean build + all gates
```

Three gates: unit tests (`colcon test`), the offline byte-exact gate (`--mode all_checks`) and the
live byte-exact gate (`src/snowclear_ros/test/live_check.py`). A change that can alter detection
turns the second one red. Data layout: [`docs/DATASET.md`](docs/DATASET.md).

## Status

- **SVOR** — Surface-Veto Outlier Removal. `SnowClear` is the reference implementation and the
  repository name. Paper in preparation; author Zilin Zhu (朱子霖).
- What is inherited: the smooth range–intensity threshold is **IDSOR** (Yan & Bengtsson, KTH, 2026,
  [arXiv:2602.05876](https://arxiv.org/abs/2602.05876)) — the released defaults `idsor_k = 2.15` and
  `idsor_theta = 2.38` are that paper's constants, and the classical filters are cited at the foot of
  this page. What this repository adds on top is the zero-intensity surface veto, the released
  decision rule, the audit behind it and the ROS 2 packaging.
- The pipeline is the ROS 1 `clustering` code moved to ROS 2 — numerics unchanged, held there by the
  byte-exact gates above ([`docs/MIGRATION_ROS1.md`](docs/MIGRATION_ROS1.md)).
- No licence chosen yet; see [`LICENSE`](LICENSE).

## Documentation

- Method and shipped constants: [`docs/METHOD.md`](docs/METHOD.md)
- Node reference: [`docs/ROS2.md`](docs/ROS2.md)
- Package layout: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)
- Measurements, ablations and roadmap: [`docs/OPTIMIZATION.md`](docs/OPTIMIZATION.md)
- Figure index and regeneration commands: [`docs/figures/README.md`](docs/figures/README.md)
- Differences from the ROS 1 build: [`docs/MIGRATION_ROS1.md`](docs/MIGRATION_ROS1.md)

## Citation

```bibtex
@misc{svor,
  title  = {Snow Removal for Spinning LiDAR Point Clouds with Surface-Veto Outlier Removal},
  author = {Zhu, Zilin},
  year   = {2026},
  note   = {Manuscript in preparation},
  url    = {https://github.com/p20030920p/SnowClear}
}
```

## Contact

Open an issue for bugs or reproduction failures. Include the output of `--mode all_checks` and your
`OMP_NUM_THREADS`.

The range–intensity threshold follows **IDSOR** (Yan & Bengtsson, KTH Royal Institute of Technology,
2026, [arXiv:2602.05876](https://arxiv.org/abs/2602.05876)), which itself extends DSOR. The
non-learned baselines in `dynamic_outlier_filters.cpp` follow
[DROR](https://github.com/nickcharron/lidar_snow_removal) (Charron et al., CRV 2018) and
[DSOR](https://github.com/assasinXL/dsor_filter) (Kurup & Bos, 2021), and the comparison uses
[CRFOR](https://github.com/dut-mdmu/CRFOR) (Wang et al., 2022/2023). The evaluation set is the
[Winter Adverse Driving dataSet](https://digitalcommons.mtu.edu/wads/) (WADS), Michigan
Technological University.
