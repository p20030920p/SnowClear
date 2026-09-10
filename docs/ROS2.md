# ROS 2 node reference

`ros2 run snowclear_ros snowclear_node`, or through the launch file.

---

## 1. Topics

Names are relative to the node, which is called `snowclear`, so `~/input/points` resolves to
`/snowclear/input/points` unless the node is renamed or remapped.

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `~/input/points` | `sensor_msgs/msg/PointCloud2` | subscribe | `x y z intensity` fields required |
| `~/output/points` | `sensor_msgs/msg/PointCloud2` | publish | the de-snowed cloud |
| `~/output/snow_points` | `sensor_msgs/msg/PointCloud2` | publish | only the removed points; disable with `publish_snow_points:=false` |
| `~/output/snow_indices` | `std_msgs/msg/Int32MultiArray` | publish | ascending indices; disable with `publish_indices:=false` |

All four use `rclcpp::SensorDataQoS()` — best-effort, keep-last 5, volatile — which is the
convention for sensor streams and matches what RViz expects.

**Index semantics.** An index refers to the cloud the detector actually saw. If the incoming
message has `is_dense == false`, NaN points are removed first (as the offline path does), so
indices refer to the point cloud after that removal. Dense messages are passed through
untouched, so for a well-formed driver message the indices index the input message directly.

**Header handling.** `header.frame_id` and `header.stamp` are copied from the input message to
all three outputs. The node performs no TF lookup; it works entirely in the sensor frame.

**Field requirements.** The message must carry `x`, `y`, `z` and `intensity` as `FLOAT32`.
`pcl::fromROSMsg` maps by field name, so extra fields are ignored and a different order is
fine. A missing `intensity` field will produce zero intensities, which the detector will read
as snow — check your driver if the output looks like everything was removed.

---

## 2. Parameters

The node declares **every algorithm parameter the core reads**, with the released
configuration as the default value, plus two node-level options. The list is generated from
the core's loaders, so it is complete by construction.

```bash
ros2 param list /snowclear                    # 109 algorithm params + 2 node options
ros2 param describe /snowclear score_threshold
ros2 param get /snowclear use_idsor_intensity_threshold
ros2 param set /snowclear score_threshold 0.6        # next scan
ros2 param dump /snowclear > snapshot.yaml           # reproducible record of a run
```

| Group | Examples | Effect |
|---|---|---|
| ROI | `height_threshold`, `xy_threshold`, `lowest_ring_elevation_deg` | what the detector is allowed to see |
| decision | `score_threshold`, `enable_local_threshold_adjustment` | the pass/fail boundary |
| threshold shape | `idsor_scale`, `idsor_rho`, `idsor_k`, `idsor_theta` | range–intensity curve |
| surface suppression | `zero_intensity_support_radius`, `..._min_intensity`, `..._range_floor` | `I = 0` veto |
| feature switches | `enable_planarity_calculation`, `enable_density_calculation`, ... | **off** in the released configuration |
| optimisation switches | `enable_grid_search_optimization`, `enable_feature_based_recommendation` | **off**: the optimiser is a no-op |
| portability | `enable_sensor_height_roi`, `enable_range_selfcal`, `enable_sparse_support_expand` | **off**: they change the numbers |
| node | `publish_snow_points`, `publish_indices` | output volume |

Because the defaults *are* the released configuration, starting the node with no params file
reproduces the paper. A params file is only ever an override:

```bash
ros2 run snowclear_ros snowclear_node --ros-args --params-file my_overrides.yaml
```

`config/snowclear_node_params.yaml` writes all released values out explicitly. It is generated
from `config/snowclear_params.yaml` by `tools/gen_ros2_params.py`; regenerate it rather than
editing it.

> [!WARNING]
> Read [`METHOD.md`](METHOD.md) before tuning anything. Several features the paper describes
> are disabled in the released configuration, the fused score has collapsed to two terms, and
> under these parameters no point with intensity ≥ 2 can be classified as snow at all. Turning
> a switch on changes behaviour; that is what the byte-exact regression is there to catch.

---

## 3. Launch

```bash
ros2 launch snowclear_ros snowclear.launch.py
ros2 launch snowclear_ros snowclear.launch.py rviz:=true
ros2 launch snowclear_ros snowclear.launch.py pcd:=/path/frame.pcd pcd_rate:=2.0
ros2 launch snowclear_ros snowclear.launch.py params_file:=/path/overrides.yaml
ros2 launch snowclear_ros snowclear.launch.py input_topic:=/velodyne_points
```

| Argument | Default | Purpose |
|---|---|---|
| `params_file` | `config/snowclear_node_params.yaml` | released configuration |
| `rviz` | `false` | start RViz with the three-cloud layout |
| `pcd` | `""` | replay a PCD through the node (starts `scan_to_cloud.py`) |
| `pcd_rate` | `1.0` | replay rate in Hz |
| `input_topic` | `/snowclear/input/points` | remap the subscription |

The RViz layout shows the raw input dimmed, the de-snowed cloud on the intensity ramp, and the
removed points as flat red — the quickest way to sanity-check that the detector is removing
snow rather than structure.

---

## 4. Driving it without a sensor

```bash
ros2 run snowclear_ros scan_to_cloud.py --pcd frame.pcd --rate 1.0
ros2 run snowclear_ros scan_to_cloud.py --pcd-dir data/35/velodyne --rate 5 --loop
```

Reads `ascii` or `binary` PCD with `x y z intensity` layout (numpy only, no PCL binding
needed) and publishes on `/snowclear/input/points` by default.

---

## 5. Diagnostics

```bash
ros2 topic hz /snowclear/output/points
ros2 topic echo /snowclear/output/snow_indices --once
ros2 param dump /snowclear | head -20
```

The node logs one line per scan:

```text
[INFO] [snowclear]: 本帧 208504 点，判为雪 6957 点，用时 6.80 ms
```

The per-scan count is the primary signal. If it is zero on a snowy frame, check the ROI
parameters first (a mis-set `xy_threshold` or `height_threshold` removes everything before the
detector sees it); if it is ~everything, check the incoming `intensity` field.

Core diagnostics — calibration results, threshold values, stage timings — go to the ROS logger
at `INFO` through the installed log sink, exactly as they do on the CLI's stdout.

---

## 6. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ros2` CLI hangs, no topics listed | DDS discovery blocked by the environment | `export ROS_LOCALHOST_ONLY=1` |
| Topics listed, but no output | publisher matched after the scan was sent | stream the scans (`--rate 1.0`) rather than publishing once |
| `ParameterNotDeclaredException` at start-up | params file contains a key the node does not declare | check the spelling against `ros2 param list`; the node declares the full set |
| All points classified as snow | missing or all-zero `intensity` field | verify the input message fields |
| Very slow first scan | calibration observes the first `calib_frames` frames | expected; steady-state timing follows |
| Node restarts lose calibration | calibration is per-process state, frozen after `calib_frames` | expected; the window is a parameter |
