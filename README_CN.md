<div align="center">

# SnowClear

**实时、免训练的 LiDAR 雪点检测与去除 —— 算法核不依赖任何中间件**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![PCL](https://img.shields.io/badge/PCL-1.10%2B-0F9D58)](https://pointclouds.org/)
[![平台](https://img.shields.io/badge/platform-Linux-333333?logo=linux&logoColor=white)](#环境要求)
[![回归](https://img.shields.io/badge/%E9%80%90%E5%AD%97%E8%8A%82%E5%9B%9E%E5%BD%92-%E9%80%9A%E8%BF%87-success)](#验证)
[![许可证](https://img.shields.io/badge/license-TODO-lightgrey)](LICENSE)

[架构](#架构) &nbsp;•&nbsp; [快速开始](#快速开始) &nbsp;•&nbsp; [ROS 2 使用](#ros-2-使用) &nbsp;•&nbsp; [验证](#验证) &nbsp;•&nbsp; [引用](#引用)

*[English](README.md) &nbsp;|&nbsp; 中文*

</div>

---

SnowClear 面向机械旋转式 LiDAR，**逐点**检测并去除降雪噪声：帧率级速度、纯 CPU、
**无需训练、无需 GPU、无任何学习权重**。输入一帧点云，输出 (i) 去雪后的点云和
(ii) 被判为雪点的索引——索引位于**原始输入点云**的坐标系与索引空间中。

塑造本仓库结构的那条约束是：**算法不得依赖中间件**。检测代码只链接 PCL、OpenMP 与
TBB，因此同一份库既能被 ROS 2 节点驱动，也能被离线 CLI 与单元测试驱动——并且三者
**产出的结果被证明完全一致**。

> **可复现优先。** 仓库随附发布参数集与一份参考检测输出，重建后由两道独立的关卡验证：
> 离线逐字节回归，以及一个断言「ROS 2 在线路径发出**同样那 6 957 个索引**」的实时
> 发布/订阅检查。两者在 CI 中都不是可选项。

---

## 亮点

- **算法核与中间件解耦。** `snowclear_core` 从不链接 `rclcpp`。ROS 2 层只是薄适配层，
  嵌入式或离线部署不必为 ROS 付出代价。
- **在线与离线逐位一致。** 节点调用的是 CLI 调用的同一个
  `CloudOperations::process_cloud()`，并由 `live_check.py` 在真实数据上验证。
- **无学习、无 GPU。** 参考机器上约 10 ms/帧，纯 CPU，Release 构建。
- **逐字节回归。** 一条命令即可让"重构移动了任何一个索引"变成构建失败。
- **带类型的 ROS 2 参数。** 全部算法参数由生成的登记表声明，`ros2 param list` 能看到
  完整的可调面；而 `tools/gen_param_map.py --check` 会在"加了参数却忘了登记"时失败。
- **诚实的评估口径。** 零检测帧、空真值、全帧真负例、0/0 分母都被显式处理，而不是用来
  抬高指标。
- **随附非学习式基线。** DROR 与 DSOR 与本方法共享完全相同的预处理与评估路径。

---

## 架构

两个 `ament` 包，按**依赖**而不是按方便程度切分：

```text
SnowClear/
├── src/
│   ├── snowclear_core/        # 算法本体。PCL + OpenMP + TBB，无 ROS。
│   │   ├── include/snowclear/ # 公开头文件，命名空间 snowclear
│   │   ├── src/               # 预处理 / 特征 / 检测 / 评估 / 自标定
│   │   ├── apps/              # snowclear_cli、snowclear_runner（离线，无 ROS）
│   │   └── test/              # gtest：配置、参数通路、判定函数不变量
│   └── snowclear_ros/         # ROS 2 层。不含任何算法。
│       ├── src/               # rclcpp 节点 + 可组合节点注册
│       ├── launch/            # snowclear.launch.py
│       ├── config/            # 发布参数（扁平视图 + ROS 2 视图）
│       ├── rviz/              # 三层点云布局
│       ├── scripts/           # scan_to_cloud.py —— 把 PCD 当 PointCloud2 回放
│       └── test/              # 节点接线测试 + live_check.py
├── tools/                     # 保持各参数视图同步的生成脚本
└── docs/
```

两者之间的接缝只有两个小接口：

| 接口 | 作用 | 实现 |
|---|---|---|
| `snowclear::ParamSource` | 配置从哪来 | `RosParamSource`（rclcpp）、`MapParamSource`（YAML + 命令行） |
| `snowclear::set_log_sink()` | 日志往哪去 | 默认 stdout/stderr，节点里走 `RCLCPP_*` |

所有数值逻辑都在接缝之下。这就是在线与离线路径不可能漂移的原因：它们是同一份代码，
差别只在于"一个字符串从哪来"和"一行日志往哪去"。

### 流水线

```text
                 sensor_msgs/PointCloud2                  (ROS 2)
                          │
                   pcl::fromROSMsg
                          │
  raw point cloud ────────┴────────────────────────────────────────
        │
   ┌────▼─────────────┐  ROI 门控：z、水平距离、最低环仰角
   │  preprocessor    │  → 剔除约 73 % 的点
   └────┬─────────────┘
   ┌────▼─────────────┐  强度直方图 → Q1；1 m 局部地面网格
   │ feature_extractor│
   └────┬─────────────┘
   ┌────▼─────────────┐  T(r, I) = s·Tg·(1 − α(r)·h(I))，α 来自 Gamma
   │ 距离-强度阈值    │  拟合，逐帧建 LUT
   └────┬─────────────┘
   ┌────▼─────────────┐  强度得分 + 相对离地高度 → 局部阈值；
   │  snow_detector   │  紧贴亮表面的 I≈0 点被否决（误检主来源）
   └────┬─────────────┘
   ┌────▼─────────────┐  处理后索引 → 原始索引
   │  索引映射        │
   └────┬─────────────┘
        │
   去雪点云 + 雪点索引 ──────────────► 三个 PointCloud2 话题      (ROS 2)
```

**出厂配置下**确切的判定函数——包括发布配置里哪些特征被关闭——写在
[`docs/METHOD.md`](docs/METHOD.md)。改参数前请先读它：融合得分已退化为
`0.7·S + 0.15·hag`，其推论之一是**强度 ≥ 2 的点在当前参数下结构上不可能被判为雪点**。

---

## 环境要求

| 组件 | 版本 |
|---|---|
| 操作系统 | Ubuntu 24.04（已实测） |
| ROS | ROS 2 Jazzy |
| PCL | 1.10 或更新（`common`、`filters`、`io`、`kdtree`、`search`） |
| 编译器 | C++17（g++ 9+） |
| 其他 | TBB、OpenMP、yaml-cpp（仅 CLI）、Eigen |

**不需要 `pcl_ros`**：`PointCloud2` ↔ `pcl::PointCloud` 的转换由 `pcl_conversions` 提供，
其余部分直接使用 PCL。

---

## 快速开始

```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/p20030920p/SnowClear.git
cd SnowClear

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release    # -O0 会让耗时虚高约 10 倍
source install/setup.bash
```

两个构建上的坑，先踩过就不用再花时间：

- 两个包都在 `find_package(PCL)` **之前**调用
  `find_package(MPI REQUIRED COMPONENTS C)`。PCL 的 config 会拉入 VTK，其链接接口要求
  `MPI::MPI_C` 目标已经存在；否则配置会在 `VTK-targets.cmake` 里失败。
- 两个 project 都声明 `LANGUAGES C CXX`，因为 `FindMPI` 在纯 C++ 工程里拒绝解析 `C` 组件。

### 离线模式（不需要 ROS 图）

```bash
# 单帧，不评估
ros2 run snowclear_core snowclear_cli \
  pcd_file:=/path/frame.pcd result_folder:=/path/gt save_results:=false

# 整目录，写出雪点索引
ros2 run snowclear_core snowclear_cli \
  process_all_frames:=true pcd_folder:=/path/scans result_folder:=/path/gt \
  save_results:=true output_dir:=/tmp/out

# 质量门禁 —— 必须打印两次 [OK]
ros2 run snowclear_core snowclear_runner --mode all_checks \
  --params install/snowclear_ros/share/snowclear_ros/config/snowclear_params.yaml \
  pcd_file:=/path/frame.pcd reference_file:=testdata/reference_042126.txt \
  output_dir:=/tmp/regression
```

参数优先级为 `编译期默认值 < --params file.yaml < key:=value`，且 `key:=value` 的写法
刻意与 ROS 1 版保持一致，因此原有配置可以原样搬过来。

---

## ROS 2 使用

```bash
ros2 launch snowclear_ros snowclear.launch.py rviz:=true
```

| 话题 | 类型 | 方向 |
|---|---|---|
| `~/input/points` | `sensor_msgs/msg/PointCloud2` | 订阅 |
| `~/output/points` | `sensor_msgs/msg/PointCloud2` | 发布（去雪后） |
| `~/output/snow_points` | `sensor_msgs/msg/PointCloud2` | 发布（被剔除的点） |
| `~/output/snow_indices` | `std_msgs/msg/Int32MultiArray` | 发布（相对输入的索引） |

`~/output/snow_points` 的存在是为了让被剔除的点能直接在 RViz 里看到；索引是机器可读形式，
索引 `i` 指向检测器真正看到的那份点云（稀疏消息会先去 NaN）。

**每一个算法参数都是 ROS 2 参数**，默认值即发布配置：

```bash
ros2 param list /snowclear                       # 109 个算法参数 + 2 个节点选项
ros2 param get  /snowclear score_threshold       # 0.75
ros2 param set  /snowclear score_threshold 0.6   # 下一帧生效
ros2 param dump /snowclear > my_config.yaml
```

因此**不带任何参数文件启动节点就是发布配置**，params 文件只是覆盖手段。

**没有传感器时回放 PCD：**

```bash
# 终端 1
ros2 launch snowclear_ros snowclear.launch.py
# 终端 2
ros2 run snowclear_ros scan_to_cloud.py --pcd /path/frame.pcd --rate 1.0
```

### 受限环境提示

若 DDS 发现缓慢或多播被禁（容器、部分虚拟机），`ros2` CLI 可能看起来卡住。把发现范围
限制到回环即可：

```bash
export ROS_LOCALHOST_ONLY=1     # 或 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

---

## 验证

三道关卡，范围递增，都很便宜；引用任何数字之前请全跑一遍。

| 关卡 | 命令 | 证明了什么 |
|---|---|---|
| 1. 单元测试 | `colcon test --packages-select snowclear_core snowclear_ros` | 配置默认值、参数通路，以及已文档化的判定函数不变量（如 `I ≥ 2` 永不为雪、OpenMP ≡ 串行） |
| 2. 离线逐字节 | `snowclear_runner --mode all_checks …` | 重建后的输出与参考**逐字节一致** |
| 3. 在线逐字节 | `python3 src/snowclear_ros/test/live_check.py …` | ROS 2 消息通路发出与离线**完全相同的索引** |

关卡 2 输出：

```text
[OK] 配置一致 (104 个键, 校验 104 个, 生效参数 109 个)
[OK] 检测输出与参考逐字节一致 ("reference_042126.txt", 6957 行)
```

关卡 3 输出（节点运行中，回放一帧）：

```text
输入 042126.pcd: 208504 点
参考 reference_042126.txt: 6957 个雪点索引
发现订阅者，开始发送
[OK] ROS 2 在线路径与参考逐位一致（6957 个索引）
```

### 结果

真值形式：每帧一个雪点索引列表。参考条件：Release 构建、
`OMP_NUM_THREADS=2`、`OMP_DYNAMIC=false`，不含 I/O 与评估。

| 口径 | Precision | Recall | F1 |
|---|---:|---:|---:|
| **宏平均 —— 16 场景 / 1 620 帧** | **96.6934** | **89.9765** | **92.8229** |
| 由平均 P / R 反算的 F1 | — | — | 93.2141 |
| 1 620 帧 Pooled | 96.8927 | 88.7047 | **92.6181** |
| 参考机器，每帧 | — | — | ≈ 10 ms |

> [!NOTE]
> 评测集还包含三个额外场景。它们被**单独报告**而不是被丢弃——其中两个受召回限制。
> 详见 [`docs/DATASET.md`](docs/DATASET.md)。

---

## 配置是生成出来的，不是手工维护的

同一份配置需要三种形态：算法读取的实体、给 CLI 与质量门禁用的扁平 YAML、给 ROS 2 用的
嵌套 `ros__parameters`。与其手工维护三份副本：

```bash
python3 tools/gen_param_map.py     # 重新生成 to_map() 与带类型的参数登记表
python3 tools/gen_ros2_params.py   # 从扁平配置重新生成 ROS 2 params 文件
```

两者都支持 `--check`：生成物过期时返回非零。这正是把"加了参数却忘了登记"从运行时的
谜题变成构建失败的关键。

---

## 文档

| 文档 | 内容 |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | 包布局、两个接口、为什么在处切开 |
| [`docs/METHOD.md`](docs/METHOD.md) | 出厂配置下方法的确切行为，含被关闭的特征 |
| [`docs/ROS2.md`](docs/ROS2.md) | 节点参考：话题、QoS、参数、launch、诊断 |
| [`docs/DATASET.md`](docs/DATASET.md) | 数据布局、真值格式、评测划分 |
| [`docs/MIGRATION_ROS1.md`](docs/MIGRATION_ROS1.md) | 相对 ROS 1 / catkin 版的改动清单 |

---

## 引用

<!-- TODO(authors): 论文公开后把本段替换为真实的 BibTeX 条目。 -->

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

## 许可证

<!-- TODO(license): 选定许可证后，同步更新 LICENSE、CITATION.cff、两个 package.xml 与上方徽章。 -->

尚未选定——见 [`LICENSE`](LICENSE)。第三方内容遵循各自上游条款：
`dynamic_outlier_filters.cpp` 中重新实现的非学习式基线分别遵循
[DROR](https://github.com/nickcharron/lidar_snow_removal)（Charron et al., CRV 2018）与
[DSOR](https://github.com/assasinXL/dsor_filter)（Kurup & Bos, 2021）。

## 联系

问题、Bug 与复现失败请开 issue，并附上 `--mode all_checks` 的输出与你的 `OMP_NUM_THREADS`。
