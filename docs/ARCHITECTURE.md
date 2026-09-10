# Architecture

Why the repository is split the way it is, and what holds the halves together.

---

## 1. The split

```text
snowclear_core                        snowclear_ros
──────────────────────────────        ──────────────────────────────
detection algorithm                   rclcpp node
offline CLI + experiment runner       launch files, RViz, param files
unit tests                            node wiring tests, live check
links: PCL, OpenMP, TBB, yaml-cpp     links: rclcpp, sensor_msgs, pcl_conversions
                                      and snowclear_core
```

The boundary is drawn on **dependencies**, not on convenience. `snowclear_core` has no ROS
dependency at all — it does not include a ROS header, does not link a ROS library, and cannot
fail to build because a middleware is missing. The ROS package contains no algorithm; it
converts messages and forwards parameters.

Two consequences that justify the extra package:

- **The algorithm is testable without a middleware.** The decision-function invariants in
  `snowclear_core/test/test_core.cpp` run in 27 ms with no ROS graph, no launch file and no
  simulator. That is the fast feedback loop.
- **Live and offline cannot drift.** They call the same function. A divergence would be a
  bug in the adapter, and `live_check.py` is built specifically to catch that class of bug.

A single-package layout would have made both of these harder, at the cost of one extra
`package.xml`.

---

## 2. The two interfaces

Only two things cross the boundary. Both are ordinary C++ abstractions with no ROS types in
their signatures.

### `snowclear::ParamSource` — where configuration comes from

```cpp
class ParamSource {
public:
    virtual bool has(const std::string& key) const = 0;
    virtual std::string raw(const std::string& key) const = 0;

    bool        get_bool  (const std::string&, bool) const;
    int         get_int   (const std::string&, int) const;
    float       get_float (const std::string&, float) const;
    double      get_double(const std::string&, double) const;
    std::string get_string(const std::string&, const std::string&) const;
};
```

| Implementation | Package | Backing store |
|---|---|---|
| `RosParamSource` | `snowclear_ros` | `rclcpp::Node` parameters |
| `MapParamSource` | `snowclear_core` | YAML file + `key:=value` command line |

**The defaults live in the core, not in the front ends.** `SystemConfig` and
`AblationSwitches` carry the released configuration as member initialisers, and `load()`
overwrites them from whatever source it is given. That is what makes an unset parameter behave
identically whether it came from ROS 2, a YAML file or nothing at all — and it is why the
node's compiled-in defaults reproduce the paper without a params file.

Key spelling is normalised (`~name`, `/node/name`, `_name` → `name`) so the ROS 1 convention
`_score_threshold:=0.7` still works on the command line.

### `snowclear::LogSink` — where diagnostics go

```cpp
enum class LogLevel { kDebug, kInfo, kWarn, kError };
using LogSink = std::function<void(LogLevel, const std::string&)>;
void set_log_sink(LogSink);          // empty restores stdout/stderr
```

The sources call `SC_INFO` / `SC_WARN` / `SC_ERROR` / `SC_DEBUG`, which are printf-style —
the same formatting the code used under ROS 1, so every message string, including ones
external tooling greps for, is preserved. The node installs a sink that forwards to
`RCLCPP_*`; the CLI keeps the default; tests install a discard sink so their output stays
readable.

---

## 3. Data flow

```text
  ROS 2 topic ──► snowclear_node ──► pcl::fromROSMsg ──┐
                                                       ├──► CloudOperations::process_cloud()
  PCD file   ──► snowclear_cli   ──► loadPCDFile     ──┘             │
                                                                    ▼
                                        calibration → preprocess → features
                                        → threshold → detect → index mapping
                                                                    │
                              ┌─────────────────────────────────────┴───────────┐
                              ▼                                                 ▼
                    FrameResult{snow_indices,                    file path: GT load →
                    desnowed_cloud, processed_cloud,            save indices + evaluate
                    per-stage timings}                          → metrics for the CSV
```

`process_cloud()` is the single implementation of the pipeline. `process_pcd_file()` calls it
and adds file I/O, ground-truth loading, index saving and evaluation on top;
`snowclear_node` calls it and adds message conversion. Neither duplicates the detection
sequence, which is what the byte-exact gates then rely on.

### Per-frame result

```cpp
struct FrameResult {
    bool ok;
    size_t original_cloud_size;
    CloudFeatures features;
    pcl::PointIndices::Ptr snow_indices;   // indices into the input cloud
    CloudPtr desnowed_cloud;               // input minus snow
    CloudPtr processed_cloud;              // post-ROI, for debugging
    long preprocess_us, feature_us, optimize_us, filtering_us, total_us;
};
```

The timing split is deliberate. Historically one bucket labelled "parameter optimisation"
contained ground-truth I/O, feature analysis and the optimiser together. Under the released
configuration the optimiser is a no-op — it returns the defaults immediately because both
optimisation switches are off — so that bucket was almost entirely mislabelled. It is now
split, and ground-truth I/O is charged to `io_time` where it belongs.

---

## 4. Generated artefacts

Three views of the same configuration are needed, and hand-maintaining them rots:

| View | File | Consumer |
|---|---|---|
| flat YAML | `src/snowclear_ros/config/snowclear_params.yaml` | CLI, `snowclear_runner --mode param_check`, documentation |
| `to_map()` + typed registry | generated into `system_config.cpp` / `ablation_switches.hpp` | `param_check`, the ROS 2 node's `declare_parameter` loop |
| ROS 2 params | `src/snowclear_ros/config/snowclear_node_params.yaml` | `ros2 launch` |

```bash
python3 tools/gen_param_map.py     # --check to fail instead of write
python3 tools/gen_ros2_params.py
```

`gen_param_map.py` derives both `to_map()` and `param_registry()` from the `load()` bodies
that actually read the parameters, so the reported key set cannot diverge from the consumed
key set. `param_check` then verifies two invariants at run time:

1. every key the shipped YAML pins resolves to the value it states;
2. every key in the YAML is read by somebody — an unread key is a parameter that silently
   does nothing.

Invariant 2 is not hypothetical: the ROS 1 configuration shipped a `use_fast_search` key that
no code ever read, and it is exactly what this check flagged when the configuration was
carried over.

---

## 5. Threading and determinism

- OpenMP parallelises per-point loops. Serial and parallel results are identical by
  construction — results are gathered per thread and then sorted — and a unit test asserts it.
- The pipeline keeps per-frame state (`index_mapping_`, `original_cloud_size_`,
  `current_cloud_features_`), so **one `CloudOperations` instance is not thread-safe**: use one
  instance per thread. The node processes scans sequentially on the executor thread.
- `FeatureExtractor` caches the ground grid for the cloud it was built from. `analyze()` must
  be called on the same cloud `detect()` will see, which `process_cloud()` guarantees. A test
  that reuses one extractor across different clouds would silently reuse a stale grid.
