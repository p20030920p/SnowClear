<div align="center">

# SnowClear

**面向旋转式 LiDAR 的免训练雪点去除 —— RITS**

<sub><b>R</b>ange–<b>I</b>ntensity <b>T</b>hresholding with zero-intensity <b>S</b>urface suppression（距离–强度阈值 + 零强度表面抑制） &nbsp;·&nbsp; 实时 &nbsp;·&nbsp; 纯 CPU &nbsp;·&nbsp; 无学习权重</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![PCL](https://img.shields.io/badge/PCL-1.10%2B-0F9D58)](https://pointclouds.org/)
[![平台](https://img.shields.io/badge/platform-Linux-333333?logo=linux&logoColor=white)](#环境要求)
[![回归](https://img.shields.io/badge/%E9%80%90%E5%AD%97%E8%8A%82%E5%9B%9E%E5%BD%92-%E9%80%9A%E8%BF%87-success)](#可复现性)
[![许可证](https://img.shields.io/badge/license-TODO-lightgrey)](LICENSE)

[快速开始](#快速开始) &nbsp;•&nbsp; [实验结果](#实验结果) &nbsp;•&nbsp; [方法](#方法) &nbsp;•&nbsp; [ROS 2](#ros-2-使用) &nbsp;•&nbsp; [文档](#文档)

*[English](README.md) &nbsp;|&nbsp; 中文*

</div>

![参考帧检测结果的三维透视图：灰色为结构，绿色为正确检出，蓝色为漏检，红色为误检](docs/figures/fig0_hero_zh.png)

*参考帧 `042126`（208 504 点，ROI <= 17 m）。**绿色 —— 检出且被标注的雪点 6 719 个**；
**蓝色 —— 被标注但漏检的 22 个**；**红色 —— 检出却没有标注的 238 个**；灰色是 ROI 内其余结构，
按强度着色。该帧 ROI 内精确率 96.6 %、召回率 99.7 %；16 个场景的宏观结果为 96.69 / 89.98 / 92.82。
图片由 [`tools/render_hero3d.py`](tools/render_hero3d.py) 从 PCD 与已发布索引渲染；实时 RViz
路径见 [`tools/capture_rviz_screenshot.sh`](tools/capture_rviz_screenshot.sh)。*

SnowClear 逐点去除 LiDAR 扫描中的降雪噪声：纯 CPU 约 10 ms/帧，**无需训练、无学习权重**。
算法核只链接 PCL、OpenMP 与 TBB，**不依赖 ROS**——同一份库既能离线运行，也能作为 ROS 2
节点运行，两者输出**逐位一致**，且这一点由两道逐字节关卡强制保证而非口头声明。

| | |
|---|---|
| **精度** —— 16 场景 / 1 620 帧 | P 96.69 · R 89.98 · **F1 92.82** |
| **速度** —— 参考机器，Release | **≈ 10 ms**/帧，纯 CPU |
| **学习** | 无——无权重、无 GPU、无需下载数据集 |
| **内置基线** | DROR · DSOR · SOR · ROR，共用同一流水线 |
| **算法核依赖** | 仅 PCL + OpenMP + TBB（`snowclear_core` 从不链接 `rclcpp`） |

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
---

## 方法

### 算法 1 —— 单帧检测

下面这段伪代码**就是**发布配置本身，而不是理想化的版本：哪些开关是关的、融合得分因此
退化成什么，见[§ 出厂配置究竟在算什么](#出厂配置究竟在算什么)。

![算法 1 —— 出厂配置下的 SnowClear 单帧雪点检测](docs/figures/algorithm1_zh.png)

*图 0 —— 出厂状态的算法 1。请用 `python3 tools/gen_algorithm_fig.py` 重新生成
（PNG + SVG，中英双版），不要直接改图。*

<details>
<summary>算法 1 的纯文本源码（便于复制到论文或幻灯片）</summary>

```text
算法 1  SnowClear：单帧雪点检测（出厂配置）
────────────────────────────────────────────────────────────────────────────────────────
输入    原始扫描  P = { p_i = (x_i, y_i, z_i, I_i) },  i = 1..N
        发布参数  Θ（score_threshold 0.75、idsor_scale 0.8、ρ 3.0、
                   k 2.15、θ 2.38、支撑 0.6 m / I>1.0 / r>7 m，…）
输出    雪点索引集合 S ⊆ {1..N}（输入索引空间）；去雪后点云 P \ S

 1  P' ← ∅ ;  map ← ∅                              ▷ ROI 门控 + 处理后→原始索引映射
 2  for each p_i ∈ P do                            ▷ 数据并行，与顺序无关
 3      if  z_i ∈ [−1.0, 2.6]  ∧  x_i² + y_i² ≤ 17²
 4          ∧  asin(z_i / ‖p_i‖) ≥ −23°  then
 5          append p_i to P' ;  map(|P'|) ← i
 6
 7  H  ← P' 的 256 级强度直方图                     ▷ 每线程局部直方图，按整数求和
 8  Q1 ← CDF(H) 首次 ≥ 0.25·|P'| 的分箱
 9  Tg ← clamp(0.8·Q1, 2.5, 8.0)
10  在 r ∈ [0, 40] m 上以 1 cm 步长建 α 查表:        ▷ Γ(2.15) 是帧常量，提到循环外
11      α(r) ← ρ·f(r) / (ρ·f(r) + 1),   f(r) = Gamma_pdf(r; k = 2.15, θ = 2.38)
12  ground ← 1 m × 1 m 网格内 z 的第 10 百分位（格内 ≥ 5 点；否则取全局最小 z）
13  G  ← 支撑点 { p ∈ P' : I > 1.0 } 的空间哈希，cell = 0.6 m
14
15  S ← ∅
16  for each p_j = (x, y, z, I) ∈ P' do            ▷ 数据并行；各线程结果合并后排序
17      r ← √(x² + y²) ;   h ← 1 − min(1, I/255)
18      T ← clamp( 0.8 · Tg · (1 − α(r)·h), 2.0, 20.0 )      ▷ 平滑距离–强度阈值
19      if  I ≥ 1.2·T  then continue                          ▷ 早停
20      s ← ( I < T ) ? (1 − I/T)^1.2 : 0                     ▷ 强度得分
21      if  I ≤ 0.5·I_min  ∧  r > 7  ∧  G 中存在距 p_j 0.6 m 内的支撑点
22          then continue                                     ▷ 零强度表面抑制
23      if  s < 0.25  then continue                           ▷ 预筛
24      hag ← clamp( (z − ground(cell(x, y))) / 1.5, 0, 1 )   ▷ 归一化离地高度
25      C ← 0.7·s + 0.15·hag                                  ▷ 融合得分（发布配置）
26      θ ← 0.75 ;  if s < 0.4 then θ ← 0.90 else if s > 0.7 then θ ← 0.675
27      if  C > θ  ∧  s > 0.3  then  S ← S ∪ { map(j) }
28
29  return  S ,  P \ S
```

</details>

伪代码没有体现的两个实现要点：

- **并行下的确定性。** 只有逐点循环（第 16–27 行）被并行化，每线程各自累积索引、最后
  拼接并排序去重；直方图累加（第 7 行）用整数，使场景统计同样与线程数无关。单元测试断言
  串行与 OpenMP 路径返回**逐位相同**的集合。
- **索引空间。** 检测器只看到 `P'`，在第 27 行映射回输入索引，因此调用方可以直接索引
  自己的点云；该映射同时承载（默认关闭的）去重路径所需的精确重复点分组展开。

### 出厂配置究竟在算什么

> [!IMPORTANT]
> 论文里描述的若干特征在**发布配置中是关闭的**，融合得分因此退化为两项：
>
> - `weight_sum` 为 `0.35 + 0.15 = 0.50`，故第 25 行实际计算 `0.7·s + 0.15·hag`。高度项
>   最多只能贡献 **0.15**，而阈值是 0.675–0.90。
> - 由此可推出：这些参数下**强度 `I ≥ 2` 的点结构上不可能被判为雪**——判定要求
>   `s > 0.75`，即 `I/T < 0.2132`，而任何一帧都有 `T ≤ 6.4`。报告集之外的三个评测场景里
>   有两个正是被这条边界限制住召回的。
> - **参数优化是空操作**：两个优化开关都关，`ParameterOptimizer::optimize()` 直接返回默认值
>   （实测耗时 0.0001 ms/帧）。网格搜索、特征推荐、平面度、密度、熵、高度一致性、
>   孤立点处理与预降采样全部关闭。
>
> [`docs/METHOD.md`](docs/METHOD.md) 逐条写下了出厂状态下的判定函数，以及每个关闭开关背后
> 的实测依据。改任何参数之前请先读它，引用任何模块之前也请先确认它在发布配置里是否真的启用。

### 与中间件解耦的接缝

两个 `ament` 包，按**依赖**而不是按方便程度切分：

```text
src/snowclear_core/     算法本体。PCL + OpenMP + TBB，无 ROS。
                        + 离线 CLI 与实验运行器，+ 单元测试
src/snowclear_ros/      ROS 2 层。不含任何算法。
                        + launch、配置、RViz、PCD 回放、节点接线测试
```

跨越边界的只有两样东西，且签名里都没有 ROS 类型：

| 接口 | 作用 | 实现 |
|---|---|---|
| `snowclear::ParamSource` | 配置从哪来 | `RosParamSource`（rclcpp）、`MapParamSource`（YAML + 命令行） |
| `snowclear::set_log_sink()` | 日志往哪去 | 默认 stdout/stderr，节点里走 `RCLCPP_*` |

所有数值逻辑都在接缝之下。这就是在线与离线路径不可能漂移的原因：它们是同一份代码，
差别只在于"一个字符串从哪来"和"一行日志往哪去"。包布局、线程模型与生成式配置的设计
见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。

---

## 实验结果

**条件。** Release 构建、`OMP_NUM_THREADS=2`、`OMP_DYNAMIC=false`，单帧耗时不含 I/O 与评估。
真值为逐帧的雪点索引列表。评测集为
[Winter Adverse Driving dataSet (WADS)][wads] 的 16 个场景 / 1 620 帧；点云不可再分发，
因此不随仓库发布——见 [`docs/DATASET.md`](docs/DATASET.md)。

[wads]: https://digitalcommons.mtu.edu/wads/

### 表 1 —— 报告集上的检测精度与耗时

| 口径 | Precision | Recall | F1 | ms/帧 |
|---|---:|---:|---:|---:|
| **宏平均 —— 16 场景 / 1 620 帧** | **96.6934** | **89.9765** | **92.8229** | **≈ 10** |
| 由平均 P / R 反算的 F1 | — | — | 93.2141 | — |
| 1 620 帧 Pooled | 96.8927 | 88.7047 | 92.6181 | — |

*图 1 —— 19 个镜像场景的逐场景 Precision / Recall / F1，并标出 16 场景报告集。
图片位：`docs/figures/fig1_per_scene.png`。*

![参考帧 042126 的定性对比：原始点云、真值标注、带逐帧 P/R/F1 的检出结果、去雪后点云](docs/figures/fig2_qualitative_zh.png)

*图 2 —— 参考帧 `042126`（场景 35）四联图：(a) 原始点云，(b) 真值雪点标注，(c) 检出结果按
TP / FN / FP 拆分，并在图内引出本帧 P / R / F1 与最密集的误差聚集区，(d) 去雪后点云（剔除
6 957 点）。虚线 ROI 圈之外的真值在构造上不可达。用 `python3 tools/render_qualitative.py`
重新生成，见 [`figures/README.md`](docs/figures/README.md)。*

### 表 2 —— 与非学习式基线的对比

基线已编译进算法核，且与本方法共用完全相同的 ROI 门控、索引映射与评估路径，因此对比
隔离出的是判定规则本身，而不是工程管线。

| 方法 | Precision | Recall | F1 | ms/帧 |
|---|---:|---:|---:|---:|
| DROR（Charron et al., CRV 2018） | TODO | TODO | TODO | TODO |
| DSOR（Kurup & Bos, 2021） | TODO | TODO | TODO | TODO |
| SOR（Rusu et al., 2008） | TODO | TODO | TODO | TODO |
| ROR（Rusu, 2009） | TODO | TODO | TODO | TODO |
| **SnowClear（发布配置）** | **96.6934** | **89.9765** | **92.8229** | **≈ 10** |

*图 3 —— SnowClear 与表 2 中各非学习式基线的 Precision / Recall / F1 对比。
图片位：`docs/figures/fig3_comparison.png`。*

> [!NOTE]
> 表中的 `TODO` 是刻意保留的：论文里基线的数字来自第三方上游实现，本仓库不重新分发它们，
> 因此**不发布无法自行复现的数字**。用 `detector_type:=dror|dsor|sor|ror` 配合
> `snowclear_runner --mode eval_folders` 跑一遍即可填上。本仓库自实现的 4 场景子集结果见
> [`OPTIMIZATION.md`](docs/OPTIMIZATION.md) §7：SnowClear 宏平均 F1 77.56，DROR 7.10、
> DSOR 6.93、SOR 36.36、ROR 1.92。

![同一帧上 SnowClear 与 SOR 的对比：精确率 96.58 对 70.21，召回率 96.04 对 56.05](docs/figures/fig11_comparison_zh.png)

*图 11 —— 同一帧上 SnowClear（左）与 SOR（右），两者共用完全相同的流水线。SOR 面板被误检
（紫色）与漏检（蓝色）占满；F1 96.31 对 62.33。这是表 2 的定性对照，用 `--detection2`
重新生成。*

### 表 3 —— 消融实验

| 配置 | 宏平均 F1 | Δ F1 | 依据 |
|---|---:|---:|---|
| **发布配置（完整）** | **92.8229** | — | 表 1 |
| 去掉零强度表面抑制 | TODO | **≈ −2.2 pp** | 实测*增量*，[`METHOD.md`](docs/METHOD.md) §4 |
| 仅 ROI 门控 + `I = 0`（无阈值、无得分、无高度项） | 90.12 | −2.70 pp（推算） | 实测，[`METHOD.md`](docs/METHOD.md) §2 |
| 加入平面度项（`enable_planarity_calculation`） | TODO | ≈ 0 | 100 帧实测贡献 ≈ 0，[`METHOD.md`](docs/METHOD.md) §5 |
| 加入密度项（`enable_density_calculation`） | TODO | 负贡献 | [`METHOD.md`](docs/METHOD.md) §5 |
| 加入特征熵（`enable_feature_entropy`） | TODO | 负贡献 | [`METHOD.md`](docs/METHOD.md) §5 |
| 加入网格搜索优化（前 3 帧） | 92.8229 | 0 | 无额外收益；优化器是空操作 |
| 加入安装高度导出 ROI（`enable_sensor_height_roi`） | TODO | 会改变数值 | 设计上默认关闭，[`METHOD.md`](docs/METHOD.md) §6 |

4 场景子集上的单开关实测消融（含上表两个默认关闭项）见
[`OPTIMIZATION.md`](docs/OPTIMIZATION.md) §6：平面度 −12.4 pp、密度 −21.2 pp、熵 −0.2 pp、
表面抑制 −2.6 pp。

*图 4 —— 模块级消融：各开关的宏平均 F1 增量，并以"ROI + `I=0`"这一平凡基线作为参考线。
图片位：`docs/figures/fig4_ablation.png`。*

*图 5 —— 发布判定函数在 `(I/T, h_ag)` 平面上的可接受域，展示 `s > 0.75` 的要求及其导致的
`I < 1.36` 上界。图片位：`docs/figures/fig5_acceptance.png`。*

<details>
<summary><b>更多结果 —— 逐场景细节、单帧耗时、跨传感器鲁棒性、误差预算</b></summary>

### 表 4 —— 报告集之外的三个评测场景

它们被**单独报告**而不是被丢弃，因为它们界定了方法的弱点（见
[`docs/DATASET.md`](docs/DATASET.md) §3）。

| 场景 | 帧数 | Precision | Recall | F1 |
|---|---:|---:|---:|---:|
| 14 | 101 | 93.96 | 53.47 | 67.34 |
| 16 | 102 | 95.82 | 44.13 | 59.74 |
| 76 | 5 | 97.85 | 98.84 | 98.34 |
| 全部 19 场景，宏平均 | 1 828 | 96.5654 | 85.4256 | 90.0295 |

场景 14 与 16 受发布判定函数的**强度上界**限制而召回偏低，并非过检：精确率维持在
94–96 %，而召回率减半。

### 表 5 —— 单帧耗时去向

在场景 35（101 帧、`OMP_NUM_THREADS=2`）上实测，每帧：

| 阶段 | ms | 说明 |
|---|---:|---|
| 真值读盘 + 解析 | 0.57 | 计入 I/O，不计入算法时间 |
| 场景统计 + 特征分析（第 7–13 行） | 4.18 | 直方图、地面网格、支撑点哈希 |
| 参数优化（第 22 行） | **0.0001** | 直接返回默认值 |
| `α(r)` 阈值查表，取代逐点 `tgamma`/`pow` | 1.140 → 0.179 | 1 cm 表、4 001 项，逐帧构建一次 |
| 仰角门控快路径，取代逐点 `asin` | 1.816 → 1.166 | 在 ROI 前的全量点云上省 36%（≈ 20万点/帧） |
| ROI 门控合计 / 逐点判定合计 | TODO | 用 `verbose:=true` 补齐 |
| **端到端，每帧** | **≈ 10** | 表 1 |

*图 6 —— 逐阶段单帧耗时分解，并给出两项"数学等价"优化（仰角门控、`α(r)` 查表）的前后对比。
图片位：`docs/figures/fig6_runtime.png`。*

### 表 6 —— 换传感器时的鲁棒性

发布配置里有四个平台相关的绝对常数。不重新标定就换平台是**已被实测的**失效模式，
而不是假想问题。

| 扰动 | 发布配置 | 打开自标定开关 |
|---|---:|---:|
| 参考传感器（WADS，64 线） | 92.92 F1 | 92.92 F1（`δ_I = 1.0`，完全等价） |
| 安装高度 + 0.9 m | 68.56 F1（−24.36 pp） | TODO |
| CADC（VLP-32C，`intensity` 归一化到 0…1） | 90.16 % 的 ROI 点被判为雪 | TODO |

*图 7 —— 跨传感器鲁棒性：发布配置的绝对常数 vs. [`METHOD.md`](docs/METHOD.md) §6 的无标签
自标定替代量。图片位：`docs/figures/fig7_cross_sensor.png`。*

*图 8 —— ROI 门控造成的召回上界：在检测器看到之前就被剔除的真值雪点，按场景给出
（16 场景 8.23 %，1 828 帧 12.25 %）。图片位：`docs/figures/fig8_gt_ceiling.png`。*

### 表 7 —— 召回损失在哪里

`tools/audit_error_budget.py --mode budget` 用精确算术复算发布规则，并把每个真值点记到
**第一个**使它无法被检出的阶段（场景 35、11、14、16，共 406 帧）：

| 阶段 | 占真值的比例（逐帧宏平均） |
|---|---:|
| 被 ROI 门控剔除 | 24.35 % |
| 高于强度上界（`s > 0.75` ⟹ `I < 0.2132·T`） | 6.67 % |
| 因紧贴亮表面被否决 | 0.41 % |
| **发布规则可达** | **68.57 %** |

同一批帧上的实测宏平均召回为 **68.60 %**：凡是规则**能**接受的真值点，几乎都已被检出——
且逐场景一致（场景 35 实测 97.07 / 可达 97.1；11：79.74 / 79.7；14：53.47 / 53.5；
16：44.13 / 44.1）。剩余差距是**结构性的而非算法性的**：真值强度呈双峰分布
（`I = 0` 占 85.59 %，`1 ≤ I < 2` 仅 0.76 %，`I ≥ 2` 占 13.65 %），因此当前参数化下任何阈值
都触不到 `I ≥ 2` 的那部分；把 `score_threshold` 从 0.75 放宽到 0.55，召回只动 0.02 pp。
后果、消融与优先级排序见 [`docs/OPTIMIZATION.md`](docs/OPTIMIZATION.md)。

![场景 16 帧 040036：召回率 44.07%，因为大部分标注雪点在 ROI 之外](docs/figures/fig10_qualitative_hard_zh.png)

*图 10 —— 一个受召回限制的帧（场景 16）：16 779 个漏检是 ROI 圈之外的标注雪点，判定规则
根本看不到它们。这就是表 7 中 A 阶段的直观来源，也是"仅扩大 ROI 无效"的原因，见
[`OPTIMIZATION.md`](docs/OPTIMIZATION.md) §5。*

**待补的图。** 上表中标注"图片位"的行是预留图位，其数据本仓库已经能生成。
[`docs/figures/README.md`](docs/figures/README.md) 列出了每个图位的准确文件名、图注与复现
命令；放入文件并取消对应行的注释即可显示。

---

</details>

---

## 可复现性

三道关卡，范围递增，都很便宜；引用任何数字之前请全跑一遍。

| 关卡 | 命令 | 证明了什么 |
|---|---|---|
| 1. 单元测试 | `colcon test --packages-select snowclear_core snowclear_ros` | 配置默认值、参数通路，以及已文档化的判定函数不变量（如 `I ≥ 2` 永不为雪、OpenMP ≡ 串行） |
| 2. 离线逐字节 | `snowclear_runner --mode all_checks …` | 重建后的输出与参考**逐字节一致** |
| 3. 在线逐字节 | `python3 src/snowclear_ros/test/live_check.py …` | ROS 2 消息通路发出与离线**完全相同的索引** |

`tools/verify.sh` 一次跑完全部五步（干净重建、两个生成器、两套测试、两道逐字节关卡）；
先把 `SNOWCLEAR_DATA` 指向 WADS 镜像。

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

任何可能改变检测结果的理由，都必须让关卡 2 从通过变为失败；规则见
[`CONTRIBUTING.md`](CONTRIBUTING.md)。

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
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | 包布局、两个接口、为什么在此处切开 |
| [`docs/METHOD.md`](docs/METHOD.md) | 出厂配置下方法的确切行为，含被关闭的特征 |
| [`docs/OPTIMIZATION.md`](docs/OPTIMIZATION.md) | 剩余误差在哪、实测消融与基线、优先级排序的后续优化建议 |
| [`docs/figures/README.md`](docs/figures/README.md) | 上文全部图片位的索引、图注与复现命令 |
| [`docs/ROS2.md`](docs/ROS2.md) | 节点参考：话题、QoS、参数、launch、诊断 |
| [`docs/DATASET.md`](docs/DATASET.md) | 数据布局、真值格式、评测划分 |
| [`docs/MIGRATION_ROS1.md`](docs/MIGRATION_ROS1.md) | 相对 ROS 1 / catkin 版的改动清单 |

---

## 引用

<!-- TODO(authors): 论文公开后把本段替换为真实的 BibTeX 条目。 -->

```bibtex
@article{snowclear,
  title   = {SnowClear: Training-Free Snow-Point Detection and Removal for Spinning LiDAR
             via Range--Intensity Thresholding and Zero-Intensity Surface Suppression},
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
