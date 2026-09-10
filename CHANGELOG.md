# Changelog

All notable changes to this project are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Fixed

Documentation, comments and tooling only — no behaviour change. Both byte-exact gates and all
15 unit tests still pass, and the generated parameter views are unchanged
(`gen_param_map.py --check` and `gen_ros2_params.py --check` both clean).

- `tools/verify.sh` hard-coded an author-local dataset path. It now defaults to the documented
  `./data` layout (still overridable via `SNOWCLEAR_DATA`) and exits with the layout reference
  when the frame is missing, instead of reporting a confusing reference mismatch.
- Six comments and one documentation section pointed at `docs/AUDIT_REPORT.md`, which is not
  distributed with this repository. They now point at `docs/MIGRATION_ROS1.md` §5.
- Comments in ten files still used the pre-port header names (`system_config.h`,
  `ablation_switches.h`, `evaluator.h`, `sensor_calibration.h`). The headers are `.hpp` under
  `include/snowclear/`.
- `src/preprocessor.cpp`'s parameter table quoted stale defaults (`height_threshold` 2.3,
  `xy_threshold` 13.0) instead of the released 2.6 / 17.0.
- The two divergent point-count gates behind `enable_pre_downsampling` (>100 k points in the
  detector stage, >400 k in `Preprocessor::run`) are now stated at both sites and in the switch
  declaration, explicitly marked as deliberately unchanged: unifying them would alter behaviour
  when the switch is enabled.

## [1.0.0] — 2026-09-10

First release of SnowClear: a ROS 2 port of the ROS 1 `clustering` catkin package,
restructured so that the algorithm carries no middleware dependency.

### Added

- **`snowclear_core`** — the detection algorithm as a ROS-free library
  (PCL + OpenMP + TBB only), with a generated typed parameter registry.
- **`snowclear_ros`** — an rclcpp node subscribing to `sensor_msgs/PointCloud2` and
  publishing the de-snowed cloud, the removed points and their indices; a Python launch file;
  an RViz layout; and `scan_to_cloud.py` for replaying PCD files without a sensor.
- **`CloudOperations::process_cloud()`** — a single, file-I/O-free entry point for one frame,
  now shared by the offline CLI, the experiment runner and the ROS 2 node. The ROS 1 build had
  two near-duplicate per-frame paths.
- **Two byte-exact gates**: an offline regression against the released reference output
  (`--mode all_checks`) and a live publish/subscribe check (`test/live_check.py`) that asserts
  the ROS 2 path emits the same indices as the offline build.
- **Generated configuration views** — `tools/gen_param_map.py` derives `to_map()` and the
  parameter registry from the loaders; `tools/gen_ros2_params.py` derives the ROS 2 params file
  from the flat YAML. Both support `--check`.
- Documentation: `docs/ARCHITECTURE.md`, `docs/METHOD.md`, `docs/ROS2.md`,
  `docs/DATASET.md`, `docs/MIGRATION_ROS1.md`.
- Bilingual `README.md` / `README_CN.md`, `CONTRIBUTING.md`, `LICENSE` and `CITATION.cff`
  (the last two as explicit placeholders).

### Changed

- **Packages split by dependency.** The ROS 1 single catkin package became two ament packages:
  an algorithm core with no ROS dependency, and a thin ROS 2 adapter containing no algorithm.
- **Configuration flows through `snowclear::ParamSource`.** `SystemConfig::load_from_ros` and
  `AblationSwitches::load_from_ros` became `load(const ParamSource&)`, with `RosParamSource`
  for the node and `MapParamSource` (YAML + `key:=value`) for the offline tools.
- **Logging flows through `snowclear::LogSink`.** `ROS_*` became `SC_*` with identical
  printf formatting and identical message strings; the node forwards to `RCLCPP_*`.
- **Timing attribution corrected.** The bucket labelled "parameter optimisation" contained
  ground-truth I/O and feature analysis as well as the optimiser. It is now split, and
  ground-truth I/O is charged to `io_time`. Under the released configuration the optimiser
  itself contributes 0.0001 ms per frame.
- Headers moved to `include/snowclear/*.hpp` and everything placed in `namespace snowclear`.
- The single-frame path now accounts for disk I/O separately, matching the multi-frame path.
- The launch files no longer hard-code an absolute dataset path; a missing input fails loudly.

### Fixed

Carried over from the ROS 1 sources, all behaviour-preserving for the released configuration
(the byte-exact regression passes):

- Stage durations printed as `ms` while computed in microseconds — a 1000× mislabel.
- `ror_radius` drove two different members, so the preprocessor's outlier radius and the ROR
  baseline's radius could not be set independently. Split into `ror_radius` and
  `dynamic_ror_radius`; both default to 0.8.
- The grid-search score function computed an F-beta and discarded it while its comment claimed
  otherwise. Dead computation removed and the comment corrected.
- `point_idx >= 0` compared a `size_t`, so it was always true.
- The shipped configuration contained a `use_fast_search` key that no code read; `param_check`
  now reports any unread key.
- The parameter-consistency guard skipped values it could not parse instead of failing, and
  compared floats with a tolerance tighter than `float` storage.

### Verified

- Offline: `[OK] 配置一致 (104 个键, 校验 104 个, 生效参数 109 个)` and
  `[OK] 检测输出与参考逐字节一致 ("reference_042126.txt", 6957 行)`.
- Live: `[OK] ROS 2 在线路径与参考逐位一致（6957 个索引）`.
- Unit tests: 11 in `snowclear_core` (config, parameter plumbing, decision-function
  invariants) and 4 in `snowclear_ros` (parameter declaration, defaults, round-trip).
- Released metrics unchanged: macro P / R / F1 = 96.6934 / 89.9765 / 92.8229 over the
  16-scene, 1 620-frame evaluation set; pooled 96.8927 / 88.7047 / 92.6181.
