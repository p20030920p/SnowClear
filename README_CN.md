<div align="center">

# SnowClear

**基于 RITS 的旋转式 LiDAR 免训练去雪**

<sub><b>R</b>ange–<b>I</b>ntensity <b>T</b>hresholding with zero-intensity <b>S</b>urface suppression &nbsp;·&nbsp; 实时 &nbsp;·&nbsp; 纯 CPU &nbsp;·&nbsp; 无学习权重</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![PCL](https://img.shields.io/badge/PCL-1.10%2B-0F9D58)](https://pointclouds.org/)
[![Platform](https://img.shields.io/badge/platform-Linux-333333?logo=linux&logoColor=white)](#快速开始)
[![Regression](https://img.shields.io/badge/byte--exact%20regression-passing-success)](#可复现性)
[![License](https://img.shields.io/badge/license-TODO-lightgrey)](LICENSE)

[快速开始](#快速开始) &nbsp;•&nbsp; [方法](#方法) &nbsp;•&nbsp; [实验结果](#实验结果) &nbsp;•&nbsp; [分析](#分析) &nbsp;•&nbsp; [文档](#文档)

*[English](README.md) &nbsp;|&nbsp; 中文*

</div>

![每帧六面板：原始点云、剔除了什么、留下了什么；上排 SnowClear、下排真值](docs/figures/detect_scene35.gif)

*场景 35，21 帧。上排 SnowClear，下排真值。三列：原始点云、剔除、去雪后。绿=剔除且被标注，
红=剔除但无标注，蓝=被标注却保留。*

**面向旋转式 LiDAR 的逐点去雪：纯 CPU 约 10 ms/帧，无需训练、无学习权重。**
输入一帧原始点云；输出去雪后的点云与雪点索引，索引仍在输入点云自身的索引空间中。核心只链接 PCL、
OpenMP 与 TBB，不链接 ROS，因此离线与 ROS 2 两条路径输出完全一致，并由两道逐字节门禁校验。

| | |
|---|---|
| **精度** —— 16 场景 / 1 620 帧 | P 96.69 · R 89.98 · **F1 92.82** |
| **速度** —— Release 构建、纯 CPU | **≈ 10 ms**/帧，无需 GPU |
| **对比最强非学习基线** | F1 是它的 **2.4 倍**（SOR 37.93），且快 9 倍 |
| **是否需要训练** | 不需要 —— 无权重、无需下载数据集 |
| **输出** | 雪点索引，位于输入点云自身的索引空间 |

---

## 快速开始

| 组件 | 版本 |
|---|---|
| 系统 / ROS | Ubuntu 24.04（已测）· ROS 2 Jazzy |
| PCL | 1.10+（`common`、`filters`、`io`、`kdtree`、`search`）；**不需要** `pcl_ros` |
| 工具链 | C++17（g++ 9+）、TBB、OpenMP、Eigen、`yaml-cpp`（仅 CLI） |

```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/p20030920p/SnowClear.git && cd SnowClear
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release   # -O0 会让耗时虚高约 10 倍
source install/setup.bash
```

离线运行 —— 不需要 ROS 图：

```bash
# 单帧
ros2 run snowclear_core snowclear_cli pcd_file:=/path/frame.pcd result_folder:=/path/gt

# 整个文件夹，并写出雪点索引
ros2 run snowclear_core snowclear_cli process_all_frames:=true pcd_folder:=/path/scans \
  result_folder:=/path/gt save_results:=true output_dir:=/tmp/out
```

作为 ROS 2 节点运行：

```bash
ros2 launch snowclear_ros snowclear.launch.py rviz:=true
```

| 话题 | 类型 | 说明 |
|---|---|---|
| `~/input/points` | `sensor_msgs/PointCloud2` | 订阅 |
| `~/output/points` | `sensor_msgs/PointCloud2` | 去雪后的点云 |
| `~/output/snow_points` | `sensor_msgs/PointCloud2` | 被剔除的点 |
| `~/output/snow_indices` | `std_msgs/Int32MultiArray` | 指向输入点云的索引 |

所有算法参数都是 ROS 2 参数，默认值即发布值；`ros2 param set /snowclear score_threshold 0.6`
会在下一帧生效。无传感器时回放 PCD：
`ros2 run snowclear_ros scan_to_cloud.py --pcd frame.pcd --rate 1.0`。节点参考：
[`docs/ROS2.md`](docs/ROS2.md)。

---

## 方法

每个点依次经过四步，后一步只看前一步留下的点：

1. **ROI 门控** —— `z ∈ [−1.0, 2.6] m`、`r ≤ 17 m`、仰角 `≥ −23°`。
2. **距离–强度打分** —— 弱回波得分高：`s = (1 − I/T)^1.2`，`T(r, I)` 在光束最密处被 Gamma 形权重压低。
3. **表面否决** —— 7 m 之外、与高亮点相距 0.6 m 内的零回波被拒绝。
4. **判定** —— `C = 0.7·s + 0.15·h_ag` 超过 `θ` 即接受；高度项上限 0.15。

![算法 1：逐帧检测循环](docs/figures/algorithm1_zh.png)

*算法 1 —— 整个检测器 25 行。推导、默认关闭的特性与全部常量见 [`docs/METHOD.md`](docs/METHOD.md)。*

---

## 实验结果

Release 构建、`OMP_NUM_THREADS=2`；耗时不含 I/O 与评测。复现命令见
[`docs/figures/README.md`](docs/figures/README.md)。

![同一帧、七种方法、同一相机](docs/figures/fig12_baselines_zh.png)

*帧 `042126`。CRFOR（Wang et al., RA-L 2023）按其发布设置与自带预处理运行；其余四个滤波器与
SnowClear 共用我们的流水线。*

### 场景 35 —— 所有方法跑同一批 101 帧

| Method | Precision | Recall | F1 | ms / frame |
|---|---:|---:|---:|---:|
| **SnowClear** | **96.79** | **97.07** | **96.90** | **≈ 9** |
| CRFOR（Wang et al., RA-L 2023） | 95.95 | 96.92 | 96.41 | 17 132 |
| DROR（Charron et al., CRV 2018） | 82.45 | 6.76 | 12.48 | 1041 |
| DSOR（Kurup & Bos, 2021） | 81.58 | 6.93 | 12.73 | 70 |
| SOR（Rusu et al., 2008） | 82.87 | 44.28 | 56.68 | 110 |
| ROR（Rusu, 2009） | 77.13 | 1.86 | 3.63 | 1013 |

CRFOR 的数字由其官方仓库经 `tools/eval_crfor.py` 得到；其余来自 `SCENES=35 bash tools/eval_baselines.sh`。
耗时列取空闲单场景运行，以免混入精度运行时的机器负载。

### 16 场景报告集 —— SnowClear 与四个内置滤波器

| Method | Precision | Recall | F1 | ms / frame |
|---|---:|---:|---:|---:|
| DROR（Charron et al., CRV 2018） | 85.54 | 3.57 | 6.80 | 1041 |
| DSOR（Kurup & Bos, 2021） | 85.02 | 3.88 | 7.36 | 70 |
| SOR（Rusu et al., 2008） | 89.24 | 25.56 | 37.93 | 110 |
| ROR（Rusu, 2009） | 79.70 | 0.89 | 1.76 | 1013 |
| **SnowClear（RITS）** | **96.69** | **89.97** | **92.82** | **≈ 10** |

CRFOR 未列入本表：按每帧约 17 s 计算，跑满 1 620 帧需要数小时；上表场景 35 的结果
是它目前诚实的样本。

![真值去了哪里](docs/figures/fig15_budget_zh.png)

*绿色段即召回上限。报告集上为 89.90 %，实测召回 89.98 %；场景 16 上 ROI 门控占 40.5 %、强度上限占
14.6 %。*

其余图（逐场景、消融、单帧耗时、判定门闭式、逐帧轨迹、强度分布）见
[`docs/figures/README.md`](docs/figures/README.md)。

---

## 可复现性

```bash
SNOWCLEAR_DATA=/path/to/wads-mirror bash tools/verify.sh   # 干净构建 + 全部门禁
```

三道门禁：单元测试（`colcon test`）、离线逐字节门禁（`--mode all_checks`，逐字节复现发布参考
输出）、在线逐字节门禁（`src/snowclear_ros/test/live_check.py`）。任何可能改变检测结果的改动都会让
第二道变红。数据布局见 [`docs/DATASET.md`](docs/DATASET.md)。

---

## 文档

方法与发布常量 [`docs/METHOD.md`](docs/METHOD.md) &nbsp;·&nbsp; 节点参考
[`docs/ROS2.md`](docs/ROS2.md) &nbsp;·&nbsp; 包划分
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) &nbsp;·&nbsp; 实测、消融与优先级路线
[`docs/OPTIMIZATION.md`](docs/OPTIMIZATION.md) &nbsp;·&nbsp; 图片索引
[`docs/figures/README.md`](docs/figures/README.md) &nbsp;·&nbsp; ROS 1 差异
[`docs/MIGRATION_ROS1.md`](docs/MIGRATION_ROS1.md)。

## 引用

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

## 许可证

尚未选定 —— 见 [`LICENSE`](LICENSE)。第三方材料沿用其上游条款：`dynamic_outlier_filters.cpp` 中的
非学习基线实现遵循 [DROR](https://github.com/nickcharron/lidar_snow_removal)（Charron et al.,
CRV 2018）与 [DSOR](https://github.com/assasinXL/dsor_filter)（Kurup & Bos, 2021）。

## 联系

问题、缺陷与复现失败请开 issue —— 请附上 `--mode all_checks` 的输出与你的 `OMP_NUM_THREADS`。
