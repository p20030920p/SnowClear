# Migrating from the ROS 1 / catkin version

SnowClear is a port of the ROS 1 `clustering` catkin package to ROS 2, restructured so the
algorithm no longer depends on any middleware. This document lists what changed, what did
**not** change, and what to check when moving a workspace over.

---

## 1. What did not change

**The numerics.** The detection pipeline is the same code. Same defaults, same decision
function, same output. This is not a claim, it is enforced: the released reference output
(`testdata/reference_042126.txt`, 6 957 indices for frame `042126`) is reproduced byte for
byte by the ported build, and by the ROS 2 node.

**The parameter names and defaults.** Every key keeps its ROS 1 spelling, and the defaults are
the released configuration. `_score_threshold:=0.7` still works on the command line, and the
`key:=value` convention is preserved in the offline CLI.

| | ROS 1 | SnowClear |
|---|---|---|
| Package(s) | `clustering` (catkin) | `snowclear_core` + `snowclear_ros` (ament) |
| Build | `catkin_make` | `colcon build` |
| ROS API | `ros::NodeHandle`, `ROS_*`, `ros::package::getPath` | `rclcpp` parameters, `RCLCPP_*`, `ament_index` |
| Entry points | `cloud_operations`, `experiment_runner` | `snowclear_cli`, `snowclear_runner`, `snowclear_node` |
| Launch | XML (`roslaunch`) | Python (`ros2 launch`) |
| Live interface | none — files only | `PointCloud2` in, three topics out |
| Headers | `clustering/include/*.h` | `snowclear_core/include/snowclear/*.hpp` |
| Namespace | global | `snowclear` |

---

## 2. Structural changes

### The algorithm was decoupled from ROS

The ROS 1 sources read their configuration directly off the parameter server
(`nh.param("name", member, default)`) and logged through `ROS_*` macros, which meant no part of
the library could be constructed or tested without a live ROS node. Both couplings are now
behind interfaces:

| Was | Now | Implementations |
|---|---|---|
| `SystemConfig::load_from_ros(ros::NodeHandle&)` | `SystemConfig::load(const ParamSource&)` | `RosParamSource`, `MapParamSource` |
| `AblationSwitches::load_from_ros(ros::NodeHandle&)` | `AblationSwitches::load(const ParamSource&)` | same |
| `ROS_INFO / ROS_WARN / ROS_ERROR` | `SC_INFO / SC_WARN / SC_ERROR` | stdout/stderr by default; `RCLCPP_*` in the node |
| `ros::package::getPath("clustering")` | explicit `output_dir` parameter | — |

Message strings are preserved verbatim, including the ones external tooling parsed.

### One pipeline implementation instead of two

The ROS 1 build had two near-duplicate per-frame paths: `process_pcd_file()` and
`process_multiple_frames()`, which had already drifted apart in their timing attribution.
Both now call a single `CloudOperations::process_cloud()`:

```cpp
FrameResult process_cloud(const CloudPtr& input_cloud,
                          const std::vector<int>& ground_truth_original = {});
```

`process_pcd_file()` adds file I/O, ground-truth loading, index saving and evaluation;
`snowclear_node` adds message conversion. Neither re-implements the detection sequence — which
is what lets the byte-exact gates cover the ROS 2 path as well as the offline one.

### Timing attribution was corrected

The bucket labelled "parameter optimisation" used to contain ground-truth file I/O, scene
feature analysis and the optimiser together. Measured on the released configuration, the
optimiser contributes **0.0001 ms per frame** — it returns the defaults immediately, because
both optimisation switches are off. The bucket is now split into `feature_us` and
`optimize_us`, and ground-truth I/O is charged to `io_time`.

The single-frame path also used to leave `io_time` at zero and fold the disk read into
`preprocess_time`, while the multi-frame path accounted for it separately. Both paths are now
consistent.

### Defects carried over from the ROS 1 tree were fixed

All of these are behaviour-preserving for the released configuration — the byte-exact
regression passes — and each was found while reading the original sources, not guessed:

| Defect | Fix |
|---|---|
| Stage duration printed as `%ld ms` but computed in microseconds — a 1000× mislabel | converted before printing |
| `ror_radius` drove **two different members** (preprocessor outlier removal and the ROR baseline) so they could not be set independently | split into `ror_radius` and `dynamic_ror_radius`, both defaulting to 0.8 |
| `evaluate_score()` computed an F-beta and then discarded it, while the comment claimed it drove the search | dead computation removed; the comment now states what the returned score actually is |
| `point_idx >= 0` on a `size_t` — always true | comparison dropped |
| A `use_fast_search` key existed in the shipped config that no code read | removed; `param_check` now reports any unread key |
| The parameter-consistency guard silently skipped values it could not parse | values are compared numerically with a tolerance appropriate to `float` storage, and unread keys are reported |

### Configuration is generated

Three views of the same configuration are needed (CLI YAML, the core's `to_map()`, ROS 2
params). Rather than three hand-maintained copies, `tools/gen_param_map.py` derives the maps
and the typed registry from the `load()` bodies, and `tools/gen_ros2_params.py` derives the
ROS 2 file from the flat one. Both take `--check`, so staleness is a build failure.

---

## 3. Moving a workspace over

| ROS 1 usage | SnowClear equivalent |
|---|---|
| `roslaunch clustering cluster_operation.launch process_all_frames:=true pcd_folder:=… result_folder:=…` | `ros2 run snowclear_core snowclear_cli process_all_frames:=true pcd_folder:=… result_folder:=…` |
| `roslaunch clustering reproduce_full.launch` | `ros2 run snowclear_core snowclear_runner --mode reproduce_full pcd_root:=… result_root:=…` |
| `roslaunch clustering eval_folders.launch` | `--mode eval_folders` |
| `roslaunch clustering all_checks.launch` | `--mode all_checks --params <yaml> pcd_file:=… reference_file:=…` |
| `roslaunch clustering param_check.launch` | `--mode param_check --params <yaml>` |
| `rosrun clustering cloud_operations pcd_file:=…` | `ros2 run snowclear_core snowclear_cli pcd_file:=…` |
| (none) | `ros2 launch snowclear_ros snowclear.launch.py` — live operation |

The launch-file paths it used to read
(`experiments/results/reference_042126_final2.txt`, `data/pcd_output/<scene>/velodyne/`) are
unchanged in meaning: the reference file ships as `testdata/reference_042126.txt`, and the
expected data layout is in [`DATASET.md`](DATASET.md).

Note that the ROS 1 `cluster_operation.launch` hard-coded the author's absolute data path as
an argument default. That is gone: the CLI requires an explicit input, so a missing dataset
fails loudly instead of silently resolving to a path that exists on one machine.

---

## 4. Output compatibility

The snow-index file format is unchanged: one `index` per line, matching the ground-truth
`index,class` convention so the two can be diffed directly. That is what makes the
byte-exact regression portable across the two builds.

The live node publishes the same information in three forms: the de-snowed cloud, the removed
points, and the indices.

---

## 5. What still needs doing

- **The audit tooling was not ported.** The ROS 1 repository carried a set of independent
  Python audit scripts (ROI-ceiling analysis, separability bounds, holdout self-calibration,
  cross-sensor checks). They are data-analysis tools, not runtime code, and they still live in
  the original repository. Several of them were already broken there — in particular the
  ROI-ceiling script used a non-recursive glob and silently reported zeros. Port them
  deliberately, fixing that class of bug, rather than copying them across.
- **The non-learned baselines (DROR / DSOR / SOR / ROR) are compiled into the core** and
  selectable with `detector_type:=dror|dsor|sor|ror`, but the official upstream harnesses used
  for the paper's comparison table are third-party and are not redistributed here.
- **The ablated configurations are not yet regenerated** under the ROS 2 build. The
  configuration surface is identical, so the existing sweeps remain valid, but no automated
  check enforces that.
