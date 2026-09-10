<div align="center">

# SnowClear

**Real-time, training-free snow-point detection and removal for LiDAR point clouds — with a ROS-agnostic core**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![PCL](https://img.shields.io/badge/PCL-1.10%2B-0F9D58)](https://pointclouds.org/)
[![Platform](https://img.shields.io/badge/platform-Linux-333333?logo=linux&logoColor=white)](#requirements)
[![Regression](https://img.shields.io/badge/byte--exact%20regression-passing-success)](#verification)
[![License](https://img.shields.io/badge/license-TODO-lightgrey)](LICENSE)

[Architecture](#architecture) &nbsp;•&nbsp; [Quick start](#quick-start) &nbsp;•&nbsp; [ROS 2 usage](#ros-2-usage) &nbsp;•&nbsp; [Verification](#verification) &nbsp;•&nbsp; [Citation](#citation)

*English &nbsp;|&nbsp; [中文](README_CN.md)*

</div>

---

SnowClear removes snowfall-induced noise from mechanical spinning LiDAR scans **per point**,
at frame rate, on CPU, with **no training, no GPU and no learned weights**. Given a point
cloud it returns the de-snowed cloud and the indices of the points classified as snow, in the
coordinate frame and index space of the *original* input.

The design constraint that shapes this repository: **the algorithm must not depend on a
middleware**. The detection code links only PCL, OpenMP and TBB, so the same library is
driven by a ROS 2 node, by an offline CLI, and by unit tests — and all three are proven to
produce identical output.

> **Reproducibility first.** The repository ships the released parameter set and a reference
> detection output. Two independent gates verify a rebuild: an offline byte-exact regression
> and a live publish/subscribe check that asserts the ROS 2 path emits the *same* 6 957
> indices. Neither is optional in CI.

---

## Highlights

- **Middleware-free core.** `snowclear_core` never links `rclcpp`. The ROS 2 layer is a thin
  adapter, so an embedded or offline deployment does not pay for ROS.
- **Live and offline agree bit-for-bit.** The node calls the same
  `CloudOperations::process_cloud()` the CLI does, and `live_check.py` proves it on real data.
- **No learning, no GPU.** ~10 ms per frame on the reference machine, CPU only, Release build.
- **Byte-exact regression.** One command fails the build if a refactor moves a single index.
- **Typed ROS 2 parameters.** Every algorithm parameter is declared from a generated
  registry, so `ros2 param list` shows the complete tunable surface — and
  `tools/gen_param_map.py --check` fails if a parameter is added without registering it.
- **Honest evaluation protocol.** Zero-detection frames, empty ground truth, whole-frame true
  negatives and 0/0 denominators are handled explicitly instead of inflating scores.
- **Non-learned baselines included.** DROR and DSOR share the exact preprocessing and
  evaluation path as the proposed method.

---

## Architecture

Two `ament` packages, split by dependency rather than by convenience:

```text
SnowClear/
├── src/
│   ├── snowclear_core/        # the algorithm. PCL + OpenMP + TBB. NO ROS.
│   │   ├── include/snowclear/ # public headers, namespace snowclear
│   │   ├── src/               # preprocessor, features, detector, evaluator, calibration
│   │   ├── apps/              # snowclear_cli, snowclear_runner (offline, ROS-free)
│   │   └── test/              # gtest: config, parameter plumbing, decision-function invariants
│   └── snowclear_ros/         # the ROS 2 layer. No algorithm.
│       ├── src/               # rclcpp node + composable-node registration
│       ├── launch/            # snowclear.launch.py
│       ├── config/            # released parameters (flat + ROS 2 views)
│       ├── rviz/              # three-cloud layout
│       ├── scripts/           # scan_to_cloud.py — replay a PCD as PointCloud2
│       └── test/              # node wiring tests + live_check.py
├── tools/                     # generators that keep the config views in sync
└── docs/
```

The seam between them is two small interfaces:

| Interface | Purpose | Implementations |
|---|---|---|
| `snowclear::ParamSource` | where configuration comes from | `RosParamSource` (rclcpp), `MapParamSource` (YAML + CLI) |
| `snowclear::set_log_sink()` | where diagnostics go | stdout/stderr by default, `RCLCPP_*` in the node |

Everything numeric lives below that seam. That is why the offline and live paths cannot
drift: they are the same code, differing only in where a string comes from and where a line
of log output goes.

### Pipeline

```text
                 sensor_msgs/PointCloud2                  (ROS 2)
                          │
                   pcl::fromROSMsg
                          │
  raw point cloud ────────┴────────────────────────────────────────
        │
   ┌────▼─────────────┐  ROI gate: z, horizontal range, lowest-ring
   │  preprocessor    │  elevation → removes ~73 % of points
   └────┬─────────────┘
   ┌────▼─────────────┐  intensity histogram → Q1; 1 m local ground grid
   │ feature_extractor│
   └────┬─────────────┘
   ┌────▼─────────────┐  T(r, I) = s·Tg·(1 − α(r)·h(I)), α from a Gamma
   │ range–intensity  │  fit, LUT-cached per frame
   │ threshold        │
   └────┬─────────────┘
   ┌────▼─────────────┐  intensity score + relative height → local
   │  snow_detector   │  threshold; I≈0 attached to a bright surface is
   └────┬─────────────┘  vetoed (the dominant false-positive source)
   ┌────▼─────────────┐  processed index → original index
   │ index mapping    │
   └────┬─────────────┘
        │
  de-snowed cloud + snow indices ────► PointCloud2 trips            (ROS 2)
```

The decision function **as shipped**, including which features are disabled in the released
configuration, is written down in [`docs/METHOD.md`](docs/METHOD.md). Read it before changing
a parameter: the fused score collapses to `0.7·S + 0.15·hag`, and one consequence is that
**no point with intensity ≥ 2 can ever be classified as snow** under the released settings.

---

## Requirements

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 (tested) |
| ROS | ROS 2 Jazzy |
| PCL | 1.10 or newer (`common`, `filters`, `io`, `kdtree`, `search`) |
| Compiler | C++17 (g++ 9+) |
| Also | TBB, OpenMP, yaml-cpp (CLI only), Eigen |

`pcl_ros` is **not** required — `pcl_conversions` covers the `PointCloud2` ↔ `pcl::PointCloud`
conversion, and everything else uses PCL directly.

---

## Quick start

```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/p20030920p/SnowClear.git
cd SnowClear

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release    # -O0 misrepresents runtime ~10x
source install/setup.bash
```

Two build-system notes that cost time if you hit them cold:

- Both packages call `find_package(MPI REQUIRED COMPONENTS C)` **before** `find_package(PCL)`.
  PCL's config pulls in VTK, whose link interface needs the `MPI::MPI_C` target to already
  exist; otherwise configuration fails inside `VTK-targets.cmake`.
- Both projects declare `LANGUAGES C CXX`, because `FindMPI` refuses to resolve the `C`
  component in a C++-only project.

### Offline, no ROS graph needed

```bash
# one frame, no evaluation
ros2 run snowclear_core snowclear_cli \
  pcd_file:=/path/frame.pcd result_folder:=/path/gt save_results:=false

# a whole folder, writing snow indices
ros2 run snowclear_core snowclear_cli \
  process_all_frames:=true pcd_folder:=/path/scans result_folder:=/path/gt \
  save_results:=true output_dir:=/tmp/out

# the quality gate — must print [OK] twice
ros2 run snowclear_core snowclear_runner --mode all_checks \
  --params install/snowclear_ros/share/snowclear_ros/config/snowclear_params.yaml \
  pcd_file:=/path/frame.pcd reference_file:=testdata/reference_042126.txt \
  output_dir:=/tmp/regression
```

Parameter precedence is `compiled-in defaults < --params file.yaml < key:=value`, and the
`key:=value` spelling is deliberately the same as the ROS 1 build's, so configurations carry
over unchanged.

---

## ROS 2 usage

```bash
ros2 launch snowclear_ros snowclear.launch.py rviz:=true
```

| Topic | Type | Direction |
|---|---|---|
| `~/input/points` | `sensor_msgs/msg/PointCloud2` | subscribe |
| `~/output/points` | `sensor_msgs/msg/PointCloud2` | publish (de-snowed) |
| `~/output/snow_points` | `sensor_msgs/msg/PointCloud2` | publish (removed points) |
| `~/output/snow_indices` | `std_msgs/msg/Int32MultiArray` | publish (indices into the input) |

`~/output/snow_points` exists so the removed points can be seen directly in RViz; the indices
are the machine-readable form, and index `i` refers to the cloud the detector saw (for a
sparse message, after NaN removal).

**Every algorithm parameter is a ROS 2 parameter**, declared with the released value as its
default:

```bash
ros2 param list /snowclear                    # 109 algorithm params + 2 node options
ros2 param get  /snowclear score_threshold    # 0.75
ros2 param set  /snowclear score_threshold 0.6   # applies to the next scan
ros2 param dump /snowclear > my_config.yaml
```

Starting the node with no parameters therefore reproduces the published configuration — a
params file is only ever an override.

**Replay a PCD without a sensor:**

```bash
# terminal 1
ros2 launch snowclear_ros snowclear.launch.py
# terminal 2
ros2 run snowclear_ros scan_to_cloud.py --pcd /path/frame.pcd --rate 1.0
```

### Notes for constrained environments

If DDS discovery is slow or multicast is blocked (containers, some VMs), the `ros2` CLI can
appear to hang. Restrict discovery to the loopback interface:

```bash
export ROS_LOCALHOST_ONLY=1     # or ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

---

## Verification

Three gates, in increasing scope. All three are cheap; run all three before quoting a number.

| Gate | Command | What it proves |
|---|---|---|
| 1. Unit tests | `colcon test --packages-select snowclear_core snowclear_ros` | config defaults, parameter plumbing, and the documented decision-function invariants (e.g. `I ≥ 2` is never snow, OpenMP ≡ serial) |
| 2. Offline byte-exact | `snowclear_runner --mode all_checks …` | the rebuild reproduces the released reference output **byte for byte** |
| 3. Live byte-exact | `python3 src/snowclear_ros/test/live_check.py …` | the ROS 2 message path emits the **same indices** as the offline build |

Gate 2 output:

```text
[OK] 配置一致 (104 个键, 校验 104 个, 生效参数 109 个)
[OK] 检测输出与参考逐字节一致 ("reference_042126.txt", 6957 行)
```

Gate 3 output (node running, one replay):

```text
输入 042126.pcd: 208504 点
参考 reference_042126.txt: 6957 个雪点索引
发现订阅者，开始发送
[OK] ROS 2 在线路径与参考逐位一致（6957 个索引）
```

### Results

Ground truth: per-frame snow-point index lists. Reference conditions: Release build,
`OMP_NUM_THREADS=2`, `OMP_DYNAMIC=false`, excluding I/O and evaluation.

| Protocol | Precision | Recall | F1 |
|---|---:|---:|---:|
| **Macro average — 16 scenes, 1 620 frames** | **96.6934** | **89.9765** | **92.8229** |
| F1 recomputed from the mean P / R | — | — | 93.2141 |
| Pooled over 1 620 frames | 96.8927 | 88.7047 | **92.6181** |
| Reference machine, per frame | — | — | ≈ 10 ms |

> [!NOTE]
> The evaluation set contains three further scenes that are reported separately rather than
> dropped — including two that are recall-limited. See [`docs/DATASET.md`](docs/DATASET.md).

---

## Configuration is generated, not hand-maintained

The same configuration is needed in three shapes: the corpus that the algorithm reads, a flat
YAML for the CLI and the quality gate, and a nested `ros__parameters` file for ROS 2. Rather
than keeping three copies in sync by hand:

```bash
python3 tools/gen_param_map.py     # regenerate to_map() + the typed parameter registry
python3 tools/gen_ros2_params.py   # regenerate the ROS 2 params file from the flat one
```

Both support `--check`, which exits non-zero when the generated artefact is stale. That is
what turns "somebody added a parameter and forgot to register it" into a build failure
instead of a mystery at runtime.

---

## Documentation

| Document | Contents |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | package layout, the two interfaces, why the split is where it is |
| [`docs/METHOD.md`](docs/METHOD.md) | the method exactly as shipped, including disabled features |
| [`docs/ROS2.md`](docs/ROS2.md) | node reference: topics, QoS, parameters, launch, diagnostics |
| [`docs/DATASET.md`](docs/DATASET.md) | data layout, ground-truth format, evaluation split |
| [`docs/MIGRATION_ROS1.md`](docs/MIGRATION_ROS1.md) | what changed from the ROS 1 / catkin version |

---

## Citation

<!-- TODO(authors): replace this block with the real BibTeX entry once the paper is public. -->

```bibtex
@article{snowclear,
  title   = {TODO: paper title},
  author  = {TODO: authors},
  journal = {TODO: venue},
  year    = {TODO: year},
  doi     = {TODO: DOI or arXiv identifier},
  url     = {https://github.com/p20030920p/SnowClear}
}
```

## License

<!-- TODO(license): choose a license, then update LICENSE, CITATION.cff, both package.xml
     files and the badge above. -->

Not yet chosen — see [`LICENSE`](LICENSE). Third-party material keeps its upstream terms: the
non-learned baselines re-implemented in `dynamic_outlier_filters.cpp` follow
[DROR](https://github.com/nickcharron/lidar_snow_removal) (Charron et al., CRV 2018) and
[DSOR](https://github.com/assasinXL/dsor_filter) (Kurup & Bos, 2021).

## Contact

Questions, bugs and reproduction failures: please open an issue — include the output of
`--mode all_checks` and your `OMP_NUM_THREADS`.
