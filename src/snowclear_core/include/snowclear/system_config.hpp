#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "snowclear/cloud_operations_types.hpp"
#include "snowclear/dynamic_outlier_filters.hpp"

#include "snowclear/logging.hpp"
#include "snowclear/param_source.hpp"
#include <iostream>
#include <map>
#include <ostream>
#include <string>

namespace snowclear {

// =====================================================================
// 系统参数集中管理（解耦：参数读取/打印不再散落在 CloudOperations 构造函数）
// ---------------------------------------------------------------------
// 全部可调参数（除消融开关，见 snowclear/ablation_switches.hpp）在此处声明默认值，
// 由 load() 从 ParamSource 读取（ROS 2 参数 / YAML / 命令行 key:=value
// 均可覆盖），并在程序启动时统一打印。CloudOperations 只消费本结构。
//
// ============== 可调参数总表（= config/snowclear_params.yaml 发布配置） ==============
// 【几何与 ROI】                         默认值        作用
//   height_threshold                     2.6        z 上界（米），预处理高度过滤
//   xy_threshold                         17.0       水平距离上界（米），表面抑制后重扫描最优
//   epsilon                              0.7        聚类半径（保留字段，旧流程）
//   min_points                           4          聚类最小点数（保留字段，旧流程）
//   ratio_threshold                      0.75       聚类删除比率（保留字段，旧流程）
//   ror_radius                           0.8        离群点过滤半径（enable_outlier_removal 时）
//   ror_neighbors                        2          离群点邻居数阈值（同上）
//   lowest_ring_elevation_deg            -23.0      最低扫描环排除仰角阈值（度）
// 【强度阈值与雪点判定】
//   score_threshold                      0.75       融合得分判定阈值（全量1620帧验证最优）
//   detector_knn                         5          检测 KNN 近邻数（几何特征开启时）
//   detector_min_intensity_score         0.25       强度得分预筛门槛（低于则跳过几何计算）
//   planarity_filter_threshold           0.85       平面度过滤门控（表面性 >此值跳过）
//   direct_planarity_gate                1.0        超低强度直判的平面度门控（1.0=关闭门控）
//   rgor_intensity_threshold             6.0        固定强度阈值（自适应阈值关闭时使用）
// 【零强度表面抑制】（I=0 点单独几何检验）
//   zero_intensity_support_radius        0.6        支撑点邻域半径（米）
//   zero_intensity_support_min_intensity 1.0        支撑点强度下界（>I 视为表面点）
//   zero_intensity_support_range_floor   7.0        近场豁免水平距离（米）
// 【IDSOR 平滑距离-强度阈值】
//   idsor_scale                          0.8        阈值缩放 s（终验最优）
//   idsor_rho                            3.0        天气强度 ρ（终验最优）
//   idsor_slope                          0.0        距离递增阈值斜率（远雪召回，实测无益）
//   idsor_r0                             5.0        距离递增起始半径（米）
//   idsor_k                              2.15       Gamma 形状（论文原值）
//   idsor_theta                          2.38       Gamma 尺度（论文原值）
// 【特征权重】（仅开启的特征参与归一化；平面度/密度默认关闭）
//   intensity_weight                     0.35
//   planarity_weight                     0.30
//   density_weight                       0.20
//   height_weight                        0.15
// 【DCOR/RGOR 保留字段】（当前检测不消费，仅参数推荐/网格搜索使用）
//   dcor_distance_factor                 0.025
//   dcor_min_pts                         4
//   rgor_alpha                           0.015
//   rgor_min_variance_ratio              0.10
// 【性能与优化】
//   max_optimization_iterations          15         网格搜索最大迭代（默认关闭）
//   max_grid_search_points               5000       网格搜索降采样点数上限
//   block_size                           1024       逐点判定分块大小（动态块开启时按点数缩放）
//   filter_downsampling_leaf_size        0.06       预降采样叶子尺寸（预降采样开启时）
// 【DROR/DSOR 对比基准】
//   dror_r_min / dror_alpha / dror_min_neighbors / dsor_knn / dsor_std_mult / dsor_bin_size
// 【路径与 I/O】
//   result_folder                        data/result        GT 目录（默认由 launch 指定）
//   gt_folder                            =result_folder     单帧模式 GT 目录
//   output_dir                           experiments/output 检测索引保存目录
//   save_results                         false              是否保存检测索引
//   detector_type                         feature_fusion | dror | dsor
// =====================================================================
struct SystemConfig {
    // ---- 几何与 ROI ----
    double height_threshold = 2.6;
    double xy_threshold = 17.0;   // 零强度表面抑制后重扫描最优（全量1620帧 F1 +0.45）
    double epsilon = 0.7;
    int min_points = 4;
    double ratio_threshold = 0.75;
    double ror_radius = 0.8;
    int ror_neighbors = 2;
    float lowest_ring_elevation_deg = -23.0f;

    // ---- 雪点判定 ----
    float score_threshold = 0.75f;  // 融合得分阈值（全量1620帧验证最优：F1 +0.016）
    int detector_knn = 5;
    float detector_min_intensity_score = 0.25f;
    float planarity_filter_threshold = 0.85f;
    float direct_planarity_gate = 1.0f;  // 1.0 = 关闭直判平面度门控（实测伤F1）
    double rgor_intensity_threshold = 6.0;

    // ---- 零强度点表面抑制（2026-08-15 进化，全量1620帧 F1 +1.50） ----
    // 全部 I=0 点中，与强度>min_intensity 的表面点距离 <radius 且不在近场豁免
    // 范围内的点，视为贴附暗表面（FP 主来源），不再判雪。
    float zero_intensity_support_radius = 0.6f;      // 表面支持邻域半径（米）
    float zero_intensity_support_min_intensity = 1.0f;  // “表面点”强度下界
    float zero_intensity_support_range_floor = 7.0f;    // 近场豁免半径（米）

    // ---- IDSOR ----
    float idsor_scale = 0.8f;
    float idsor_rho = 3.0f;
    float idsor_slope = 0.0f;
    float idsor_r0 = 5.0f;
    float idsor_k = 2.15f;
    float idsor_theta = 2.38f;

    // ---- 特征权重 ----
    double intensity_weight = 0.35;
    double planarity_weight = 0.30;
    double density_weight = 0.20;
    double height_weight = 0.15;

    // ---- DCOR/RGOR 保留字段 ----
    double dcor_distance_factor = 0.025;
    int dcor_min_pts = 4;
    double rgor_alpha = 0.015;
    double rgor_min_variance_ratio = 0.10;

    // ---- 性能与优化 ----
    int max_optimization_iterations = 15;
    int max_grid_search_points = 5000;
    int block_size = 1024;
    float filter_downsampling_leaf_size = 0.06f;

    // ---- 可移植性（跨传感器自标定，见 snowclear/sensor_calibration.hpp） ----
    // 这三个是**有物理含义、跨平台可直接给定**的量，取代 z/仰角三个绝对常数。
    // WADS 反推值：h_s=2.13 m 时 -h_s+1.125=-1.008(≈-1.0)、-h_s+4.725=2.592(≈2.6)、
    // -atan(h_s/5.0)=-23.11deg(≈-23.0)，三者与硬编码常数自洽（误差<1%）。
    // 注意：以下声明**不要加行尾注释**。
    // experiment_runner 的 mode:=param_check 用 `^...name = value;\s*$` 匹配，
    // 行尾注释会让该参数被静默跳过、失去守护。
    // 离地净空（米）
    float roi_clearance = 1.125f;
    // 关注净空高度（米）
    float roi_top_height = 4.725f;
    // 最低光束落地距离（米）
    float roi_lowest_beam_ground_dist = 5.0f;
    // 自标定窗口帧数（之后冻结）
    int calib_frames = 5;
    // 支撑判定所需最少邻点（不足则扩半径）
    int sparse_support_min_neighbors = 30;
    // 扩半径上限（米）
    float sparse_support_max_radius = 1.5f;

    // ---- 路径与 I/O ----
    std::string result_folder;
    std::string gt_folder;
    std::string output_dir;  // 默认 "snowclear_output"（相对 CWD）
    bool save_results = false;
    bool verbose = false;    // 逐帧详细日志开关（默认关，减少全量运行日志量）
    std::string detector_type = "feature_fusion";

    // ---- 动态滤波器对比基准 ----
    DynamicFilterParams dynamic_params;

    // 从参数源读取全部参数（ROS 2 参数 / YAML 文件 / 命令行 key:=value 均可）
    void load(const ParamSource& params);

    // 参数合法性校验/钳制：防止负数/零值/非法检测器类型导致死循环或异常行为
    void validate();

    // 转成检测流水线使用的 FilterParameters
    FilterParameters to_filter_parameters() const;

    // 参数名 -> 当前值（由 load() 自动生成，键集与 load() 完全一致）。
    // param_check 用它把发布配置与编译期默认值逐项比对。
    std::map<std::string, std::string> to_map() const;

    // 打印参数（启动诊断）。默认写 stdout；ROS 节点可传入自己的流。
    void print(std::ostream& os = std::cout) const;
};

}  // namespace snowclear

#endif  // SYSTEM_CONFIG_H
