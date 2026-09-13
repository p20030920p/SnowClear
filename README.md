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

[Quick start](#quick-start) &nbsp;•&nbsp; [Method](#method) &nbsp;•&nbsp; [Results](#results) &nbsp;•&nbsp; [Analysis](#analysis) &nbsp;•&nbsp; [Docs](#documentation)

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

|  |  |
|---|---|
| ![The same frame under five detectors: ground truth, SnowClear, DROR, DSOR, SOR and ROR](docs/figures/fig12_baselines.png) | ![The same frame, SnowClear against SOR in bird's-eye view](docs/figures/fig11_comparison.png) |
| *One frame, five detectors, one camera: the density filters leave the annotated snow untouched (blue), SOR floods it with false positives (red), SnowClear matches the ground truth.* | *The same frame against SOR in bird's-eye view — the classic before/after comparison.* |
| ![Released constants against the label-free self-calibration under a mount change](docs/figures/fig7_cross_sensor.png) | ![Ground-truth budget by scene: reachable, above the ceiling, vetoed, outside the ROI](docs/figures/fig8_gt_ceiling.png) |
| *Portability, measured: a +0.9 m mount costs the released constants **24.25 pp** of F1 — recall halves while precision holds — and the label-free self-calibration 0.06 pp.* | *Where the recall goes: 89.90 % of ground truth stays reachable, 7.38 % falls outside the ROI gate and 2.12 % above the intensity ceiling.* |

---

## Analysis

The qualitative panels and the measurements behind the tables. Every one is reproducible from the
repository; captions state the protocol.

|  |  |
|---|---|
| ![Reference frame in four panels: raw scan, annotated snow, detection outcome, de-snowed cloud](docs/figures/fig2_qualitative.png) | ![A recall-limited frame from scene 16, most missed snow lies outside the ROI](docs/figures/fig10_qualitative_hard.png) |
| *Reference frame `042126` (scene 35): raw scan, annotated snow, the detection split into TP / FN / FP with per-frame scores, and the de-snowed cloud.* | *A recall-limited frame (scene 16): most missed snow lies **outside** the ROI circle, so it is unreachable by construction.* |
| ![Before and after at 3D perspective](docs/figures/fig0_before.png) | ![The same crop after removal](docs/figures/fig0_after.png) |
| *Before: the full scene plus a 5.2 m zoom, with the points SnowClear removes in red.* | *After: the same crop cleaned. 6 957 of 208 504 points removed on this frame.* |
| ![The same frame in 3D, coloured by detection outcome](docs/figures/fig0_hero.png) | ![The before/after pair as one print-ready card](docs/figures/fig0_banner.png) |
| *Frame `042126` in 3D — grey structure shaded by intensity, green TP, red FP, blue FN, with leader lines at the miss and false-alarm clusters.* | *The same before/after pair as one card, for slides or print.* |

![Per-scene precision, recall and F1 across the 19 mirrored scenes, the 16-scene reported set shaded](docs/figures/fig1_per_scene.png)

*Per-scene result, released configuration. The shaded band is the 16-scene reported set; the dashed
lines are its macro average. Scenes 14 and 16 are recall-limited and reported separately, scene 76
holds 5 frames.*

![Single-switch ablation: macro-F1 delta per module on the 4-scene subset](docs/figures/fig4_ablation.png)

*Ablation: one switch flipped away from the released configuration per run. Both modules that ship
disabled cost F1 when enabled; the grid-search optimiser buys +0.0012 pp while roughly tripling the
frame time.*

![Frame time by stage on scene 35, plus the two equivalence-preserving fast paths](docs/figures/fig6_runtime.png)

*Frame time by stage. I/O and evaluation are drawn separately because the published timing excludes
them; the `α(r)` LUT is worth a paired +1.53 ms, and all configurations print the same F1.*

|  |  |
|---|---|
| ![Acceptance region of the released decision function and the intensity ceiling it implies](docs/figures/fig5_acceptance.png) | ![The threshold family T(r, I), the alpha(r) weight, and the per-frame base threshold](docs/figures/fig9_threshold_curve.png) |
| *The score gate in closed form: the acceptance region in the `(I/T, h_ag)` plane, and the intensity ceiling it puts on a detection.* | *The threshold family `T(r, I)`, the `α(r)` weight that shapes it, and the per-frame base threshold `Tg`.* |

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
