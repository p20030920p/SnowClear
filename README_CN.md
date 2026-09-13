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

Release 构建、`OMP_NUM_THREADS=2`；耗时不含 I/O 与评测。指标为 [WADS](https://digitalcommons.mtu.edu/wads/)
16 场景 / 1 620 帧的按场景宏平均。复现命令见 [`docs/figures/README.md`](docs/figures/README.md)。

![SnowClear 与四个非学习基线的精确率 / 召回率 / F1（报告集宏平均）](docs/figures/fig3_comparison_zh.png)

*基线使用同一批帧、同一条流水线。密度类滤波器只返回不到 4 % 的标注点。*

| 方法 | 精确率 | 召回率 | F1 | ms / 帧 |
|---|---:|---:|---:|---:|
| DROR（Charron et al., CRV 2018） | 85.54 | 3.57 | 6.80 | 1041 |
| DSOR（Kurup & Bos, 2021） | 85.02 | 3.88 | 7.36 | 70 |
| SOR（Rusu et al., 2008） | 89.24 | 25.56 | 37.93 | 110 |
| ROR（Rusu, 2009） | 79.70 | 0.89 | 1.76 | 1013 |
| **SnowClear（RITS）** | **96.69** | **89.97** | **92.82** | **≈ 10** |

`bash tools/eval_baselines.sh <输出目录>` 可复现该表。

### 同一帧，五种检测器

![同一帧在五种检测器下的结果：真值、SnowClear、DROR、DSOR、SOR、ROR](docs/figures/fig12_baselines_zh.png)

*DROR / DSOR / ROR 几乎什么都没剔除（蓝色）。SOR 把路面一起削掉（红色）。*

![同一帧上 SnowClear 与 SOR 的鸟瞰对比](docs/figures/fig11_comparison_zh.png)

*同一帧，SnowClear 与 SOR，鸟瞰视角。*

### 可移植性

![发布常量与无标注自标定的对比：参考安装高度与抬高 0.9 m](docs/figures/fig7_cross_sensor_zh.png)

*安装高度 +0.9 m：发布常量掉 24.25 pp F1，自标定掉 0.06 pp。召回腰斩，精确率不变。*

### 召回去了哪里

![召回去了哪里：逐场景真值预算](docs/figures/fig8_gt_ceiling_zh.png)

*89.90 % 的真值可达；实测召回 89.98 %。*

---

## 分析

### 逐帧

![单场景逐帧的精确率 / 召回率 / F1，以及逐帧被标注与被剔除的点数](docs/figures/fig13_per_frame_zh.png)

*101 帧的 F1 为 98.0。剔除点数与标注点数基本一致。*

### 强度上限

![弱回波判据的实测，以及逐场景高于上限的标注占比](docs/figures/fig14_intensity_zh.png)

*场景 35 上 99.6 % 的标注回波 `I = 0`，其余点只有 2 %。高于上限的占比：场景 35 为 0.19 %，
场景 16 为 27.20 %。*

### 残余误差

![参考帧四联图：原始点云、真值标注、检测结果、去雪后](docs/figures/fig2_qualitative_zh.png)

*参考帧，四联图。*

![场景 16 的召回受限帧：大部分漏检雪点在 ROI 之外](docs/figures/fig10_qualitative_hard_zh.png)

*场景 16，召回 44 %。漏检大多在 ROI 之外。*

### 去雪前后

![去雪前：整场加 5.2 m 局部放大，红色为将被剔除的点](docs/figures/fig0_zh_before.png)

![同一区域去雪后](docs/figures/fig0_zh_after.png)

*该帧 208 504 点中剔除 6 957 点。*

### 三维视图

![同一帧的三维透视图，按检测结果着色](docs/figures/fig0_hero_zh.png)

*雪浮在路面之上。*

![去雪前后合成的一张卡片](docs/figures/fig0_zh_banner.png)

*合成一张卡片。*

### 逐场景、消融、耗时

![19 个镜像场景的逐场景精确率 / 召回率 / F1，阴影带为 16 场景报告集](docs/figures/fig1_per_scene_zh.png)

*场景 14 与 16 落在报告带之外。*

![4 场景子集上各消融开关的宏 F1 变化](docs/figures/fig4_ablation_zh.png)

*每个模块单独翻转一次。两个默认关闭的模块一开启就掉 F1。*

![场景 35 的逐阶段单帧耗时，以及两个数学等价快速路径的实测影响](docs/figures/fig6_runtime_zh.png)

*场景 35 的逐阶段耗时。`α(r)` 查表值 +1.53 ms。*

![发布判定函数在强度比 / 高度平面上的接受域与强度上限](docs/figures/fig5_acceptance_zh.png)

*判定门的闭式结果。上限落在两个强度尖峰之间的空档里。*

![阈值族 T(r, I)、α(r) 权重与逐帧基阈值](docs/figures/fig9_threshold_curve_zh.png)

*`T(r, I)`、`α(r)` 权重，以及 69.7 % 的帧里被钳在下限的 `Tg`。*

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
