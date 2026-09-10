#ifndef ABLATION_SWITCHES_H
#define ABLATION_SWITCHES_H

#include "snowclear/logging.hpp"
#include "snowclear/param_source.hpp"
#include <iostream>
#include <ostream>
#include <map>

namespace snowclear {

// =============== 消融实验开关结构 =============== //
struct AblationSwitches {
    // 注意：本结构与 load() 的默认值必须与 config/snowclear_params.yaml
    // 的“最终配置”保持一致，否则直接运行二进制（不经 launch）会静默启用已废弃特征。
    // 1. 预处理模块开关
    // 最终配置关闭。注意：这一个开关背后有两道**不同**的点数门限——
    // 检测阶段 >10万点（cloud_operations.cpp），Preprocessor::run >40万点。
    // 两者故意不统一：统一会改变开关打开时的降采样路径。
    bool enable_pre_downsampling = false;
    bool use_adaptive_leaf_size = false;      // 仅预降采样开启时有意义，默认关闭
    bool enable_height_distance_filter = true;
    bool use_conservative_filtering = false;  // 最终配置关闭（保守放宽默认不启用）
    bool enable_outlier_removal = false;      // 全量验证无效果，默认关闭（保留消融开关）
    bool enable_dedup_exact_points = false;   // 精确重复点合并（指标逐位一致，但实测哈希开销大于省时，默认关闭）
    bool enable_lowest_ring_exclusion = true; // 排除最低地面扫描环（FP 主要来源）

    // 2. 参数优化模块开关（基线默认关闭：前三帧网格搜索无额外收益）
    bool enable_grid_search_optimization = false;
    bool enable_feature_based_recommendation = false;
    bool enable_adaptive_parameter_adjustment = false;

    // 3. 特征计算模块开关
    bool use_intensity_histogram_stats = true;
    bool use_adaptive_intensity_threshold = true;
    bool enable_planarity_calculation = false;   // 最终配置移除（100帧消融贡献≈0）
    bool enable_planarity_cache = false;
    bool enable_density_calculation = false;     // 最终配置移除（负贡献）
    bool enable_relative_height = true;
    bool enable_height_consistency = false;      // 最终配置移除（负贡献）
    bool enable_feature_entropy = false;         // 最终配置移除（负贡献）
    bool enable_scene_complexity = false;
    bool enable_adaptive_weight_adjustment = false;

    // 4. 雪点检测核心算法开关
    bool enable_ultra_low_intensity_direct = false;  // 参数扫描最优（IDSOR 后不再必需）
    bool enable_isolated_point_handling = false;  // 全量验证无效果，默认关闭（保留消融开关）
    bool enable_local_threshold_adjustment = true;
    bool enable_zero_intensity_surface_suppression = true;  // I=0贴附暗表面抑制（全量+1.50 F1）
    bool enable_neighbor_intensity_analysis = false;
    bool enable_intensity_comparison = false;
    bool enable_planarity_filtering = false;
    bool enable_height_consistency_check = false;

    // 5. 后处理模块开关（仅保留已实现且有效的开关；预降采样关闭时无作用）
    bool enable_downsampled_mapping_propagation = false;

    // 6. 性能优化开关
    bool enable_openmp_parallel = true;
    bool enable_block_processing = true;
    bool use_dynamic_block_size = true;
    bool enable_feature_cache = true;
    bool enable_kdtree_cache = true;
    bool enable_sampling_optimization = true;

    // 7. 场景自适应开关（final-2 全部关闭：Q1 基阈值 + IDSOR 已足够）
    bool enable_complexity_adaptation = false;
    bool enable_density_adaptation = false;
    bool enable_intensity_distribution_adaptation = false;  // final-2 关闭：Q1 基阈值已足够，全量 +0.004

    // 8. 进化方向开关（默认与最终配置一致：仅 IDSOR 开启）
    bool use_range_binned_intensity_threshold = false;  // 距离分段强度阈值
    bool use_idsor_intensity_threshold = true;          // IDSOR 式平滑距离-强度阈值（1062帧验证+0.17pp F1）
    bool enable_vertical_zscore = false;                // FlakeOut 式垂直 z-score 特征
    bool use_scattering_term = false;                   // 平面度项改用散射度 λ3/λ1（v3 候选）
    bool use_soft_consistency = false;                  // 高度一致性改为软项(1-consistency)，不再硬门控
    bool use_otsu_intensity_threshold = false;          // Otsu 强度阈值（直方图双峰切分）
    // 注：多帧时序一致性已用"仅亮点占用"重做。旧版用"任意点"占用，抑制纯度仅 15.71%、
    // 误杀 99968 个真雪点 —— 这才是旧结论 F1<0.65 的真正根因（密雪体积性互相占位）。
    // 修正后抑制纯度 98.17%（高于现行度量球 95.34%），但其检出的背景点 99.7% 是现行
    // 测试的子集，端到端仅 +0.03 pp，不值得引入多帧缓存/ICP 依赖，故仍不集成。
    // 详见 docs/MIGRATION_ROS1.md §5（原审计报告未随本仓库发布）。

    // 9. 可移植性（跨传感器）开关
    // ---------------------------------------------------------------------
    // 原实现把平台相关量写成绝对常数，换传感器即失效（实测）：
    //   CADC (VLP-32C, intensity 归一化 0..1)：无任何点满足 I>1.0 -> 支撑点集为空
    //     -> 零强度表面抑制整体失效 -> 90.16% 的 ROI 点被判成雪
    //   安装高度 +0.9 m（虚拟平台，保留标签）：F1 92.92 -> 68.56 (-24.36 pp)
    // 下列开关把这些常数换成无标签自标定量。
    //
    // 默认策略：**凡会改变 WADS 数值行为的一律默认关闭**，保证论文结果逐字节可复现。
    // 三个默认开启项均已实测为等价变换：
    //   enable_intensity_scale_autocal: 全量 1620 帧自标定 delta_I 恒为 1.0 = 硬编码值
    //   fix_quantile_sentinel:          全量 1620 帧最终阈值 0 帧改变（被 clamp 掩盖）
    //   use_fast_elevation_gate / use_threshold_lut: 数学等价改写，仅提速
    bool enable_intensity_scale_autocal = true;   // δ_I 自标定替代硬编码 1.0（WADS 上等价）
    bool enable_sensor_height_roi = false;        // 用 h_s 导出 z/仰角门限（改变数值，默认关）
    bool enable_range_selfcal = false;            // 用孤立率曲线自标定 r_min/r_max（默认关）
    bool enable_sparse_support_expand = false;    // 稀疏传感器采样不足时扩支撑半径（默认关）
    bool fix_quantile_sentinel = true;            // 修 Q1/中位数哨兵缺陷（WADS 上等价）
    bool use_fast_elevation_gate = true;          // 仰角判定去 asin/除法（等价，省 0.65 ms/帧）
    bool use_threshold_lut = true;                // IDSOR 阈值改查表（等价，省 0.96 ms/帧）

    // 10. 评估口径
    // 严格口径（默认开启）修正五处会让指标偏乐观/失真的缺陷，详见 snowclear/evaluator.hpp 顶部注释：
    //   零检测帧不再被排除在宏平均之外 / 空GT视为零雪帧照常评估 /
    //   accuracy 按评估子集口径 / 零分母报 0 而非 100% / GT 去重+过滤负索引。
    // 关闭它可复现旧口径数字以作对照。
    bool strict_evaluation = true;

    // 从 ROS 参数服务器读取所有开关
    // 注意：以下所有默认值必须与上面的结构体成员默认值一致（= launch 最终配置），
    // 防止两处默认值分叉导致“直接跑二进制”与“launch 跑”行为不一致。
    void load(const ParamSource& params) {
        // 1. 预处理模块
        enable_pre_downsampling = params.get_bool("enable_pre_downsampling", false);
        use_adaptive_leaf_size = params.get_bool("use_adaptive_leaf_size", false);
        enable_height_distance_filter = params.get_bool("enable_height_distance_filter", true);
        use_conservative_filtering = params.get_bool("use_conservative_filtering", false);
        enable_outlier_removal = params.get_bool("enable_outlier_removal", false);
        enable_dedup_exact_points = params.get_bool("enable_dedup_exact_points", false);
        enable_lowest_ring_exclusion = params.get_bool("enable_lowest_ring_exclusion", true);

        // 2. 参数优化模块
        enable_grid_search_optimization = params.get_bool("enable_grid_search_optimization", false);
        enable_feature_based_recommendation = params.get_bool("enable_feature_based_recommendation", false);
        enable_adaptive_parameter_adjustment = params.get_bool("enable_adaptive_parameter_adjustment", false);

        // 3. 特征计算模块
        use_intensity_histogram_stats = params.get_bool("use_intensity_histogram_stats", true);
        use_adaptive_intensity_threshold = params.get_bool("use_adaptive_intensity_threshold", true);
        enable_planarity_calculation = params.get_bool("enable_planarity_calculation", false);
        enable_planarity_cache = params.get_bool("enable_planarity_cache", false);
        enable_density_calculation = params.get_bool("enable_density_calculation", false);
        enable_relative_height = params.get_bool("enable_relative_height", true);
        enable_height_consistency = params.get_bool("enable_height_consistency", false);
        enable_feature_entropy = params.get_bool("enable_feature_entropy", false);
        enable_scene_complexity = params.get_bool("enable_scene_complexity", false);
        enable_adaptive_weight_adjustment = params.get_bool("enable_adaptive_weight_adjustment", false);

        // 4. 雪点检测核心算法
        enable_ultra_low_intensity_direct = params.get_bool("enable_ultra_low_intensity_direct", false);
        enable_isolated_point_handling = params.get_bool("enable_isolated_point_handling", false);
        enable_local_threshold_adjustment = params.get_bool("enable_local_threshold_adjustment", true);
        enable_zero_intensity_surface_suppression = params.get_bool("enable_zero_intensity_surface_suppression", true);
        enable_neighbor_intensity_analysis = params.get_bool("enable_neighbor_intensity_analysis", false);
        enable_intensity_comparison = params.get_bool("enable_intensity_comparison", false);
        enable_planarity_filtering = params.get_bool("enable_planarity_filtering", false);
        enable_height_consistency_check = params.get_bool("enable_height_consistency_check", false);

        // 5. 后处理模块
        enable_downsampled_mapping_propagation = params.get_bool("enable_downsampled_mapping_propagation", false);

        // 6. 性能优化
        enable_openmp_parallel = params.get_bool("enable_openmp_parallel", true);
        enable_block_processing = params.get_bool("enable_block_processing", true);
        use_dynamic_block_size = params.get_bool("use_dynamic_block_size", true);
        enable_feature_cache = params.get_bool("enable_feature_cache", true);
        enable_kdtree_cache = params.get_bool("enable_kdtree_cache", true);
        enable_sampling_optimization = params.get_bool("enable_sampling_optimization", true);

        // 7. 场景自适应
        enable_complexity_adaptation = params.get_bool("enable_complexity_adaptation", false);
        enable_density_adaptation = params.get_bool("enable_density_adaptation", false);
        enable_intensity_distribution_adaptation = params.get_bool("enable_intensity_distribution_adaptation", false);

        // 8. 进化方向
        use_range_binned_intensity_threshold = params.get_bool("use_range_binned_intensity_threshold", false);
        use_idsor_intensity_threshold = params.get_bool("use_idsor_intensity_threshold", true);
        enable_vertical_zscore = params.get_bool("enable_vertical_zscore", false);
        use_scattering_term = params.get_bool("use_scattering_term", false);
        use_soft_consistency = params.get_bool("use_soft_consistency", false);
        use_otsu_intensity_threshold = params.get_bool("use_otsu_intensity_threshold", false);

        // 9. 可移植性
        enable_intensity_scale_autocal = params.get_bool("enable_intensity_scale_autocal", true);
        enable_sensor_height_roi = params.get_bool("enable_sensor_height_roi", false);
        enable_range_selfcal = params.get_bool("enable_range_selfcal", false);
        enable_sparse_support_expand = params.get_bool("enable_sparse_support_expand", false);
        fix_quantile_sentinel = params.get_bool("fix_quantile_sentinel", true);
        use_fast_elevation_gate = params.get_bool("use_fast_elevation_gate", true);
        use_threshold_lut = params.get_bool("use_threshold_lut", true);

        // 10. 评估口径
        strict_evaluation = params.get_bool("strict_evaluation", true);
    }


    // 参数名 -> 当前值。键集由 load() 生成（tools/gen_param_map.py），
    // 保证"代码真正读取的键"与"对外报告的键"永不脱节；
    // param_check 依赖它比对发布配置。
    std::map<std::string, std::string> to_map() const {
        return {
            {"enable_pre_downsampling", param_value_string(enable_pre_downsampling)},
            {"use_adaptive_leaf_size", param_value_string(use_adaptive_leaf_size)},
            {"enable_height_distance_filter", param_value_string(enable_height_distance_filter)},
            {"use_conservative_filtering", param_value_string(use_conservative_filtering)},
            {"enable_outlier_removal", param_value_string(enable_outlier_removal)},
            {"enable_dedup_exact_points", param_value_string(enable_dedup_exact_points)},
            {"enable_lowest_ring_exclusion", param_value_string(enable_lowest_ring_exclusion)},
            {"enable_grid_search_optimization", param_value_string(enable_grid_search_optimization)},
            {"enable_feature_based_recommendation", param_value_string(enable_feature_based_recommendation)},
            {"enable_adaptive_parameter_adjustment", param_value_string(enable_adaptive_parameter_adjustment)},
            {"use_intensity_histogram_stats", param_value_string(use_intensity_histogram_stats)},
            {"use_adaptive_intensity_threshold", param_value_string(use_adaptive_intensity_threshold)},
            {"enable_planarity_calculation", param_value_string(enable_planarity_calculation)},
            {"enable_planarity_cache", param_value_string(enable_planarity_cache)},
            {"enable_density_calculation", param_value_string(enable_density_calculation)},
            {"enable_relative_height", param_value_string(enable_relative_height)},
            {"enable_height_consistency", param_value_string(enable_height_consistency)},
            {"enable_feature_entropy", param_value_string(enable_feature_entropy)},
            {"enable_scene_complexity", param_value_string(enable_scene_complexity)},
            {"enable_adaptive_weight_adjustment", param_value_string(enable_adaptive_weight_adjustment)},
            {"enable_ultra_low_intensity_direct", param_value_string(enable_ultra_low_intensity_direct)},
            {"enable_isolated_point_handling", param_value_string(enable_isolated_point_handling)},
            {"enable_local_threshold_adjustment", param_value_string(enable_local_threshold_adjustment)},
            {"enable_zero_intensity_surface_suppression", param_value_string(enable_zero_intensity_surface_suppression)},
            {"enable_neighbor_intensity_analysis", param_value_string(enable_neighbor_intensity_analysis)},
            {"enable_intensity_comparison", param_value_string(enable_intensity_comparison)},
            {"enable_planarity_filtering", param_value_string(enable_planarity_filtering)},
            {"enable_height_consistency_check", param_value_string(enable_height_consistency_check)},
            {"enable_downsampled_mapping_propagation", param_value_string(enable_downsampled_mapping_propagation)},
            {"enable_openmp_parallel", param_value_string(enable_openmp_parallel)},
            {"enable_block_processing", param_value_string(enable_block_processing)},
            {"use_dynamic_block_size", param_value_string(use_dynamic_block_size)},
            {"enable_feature_cache", param_value_string(enable_feature_cache)},
            {"enable_kdtree_cache", param_value_string(enable_kdtree_cache)},
            {"enable_sampling_optimization", param_value_string(enable_sampling_optimization)},
            {"enable_complexity_adaptation", param_value_string(enable_complexity_adaptation)},
            {"enable_density_adaptation", param_value_string(enable_density_adaptation)},
            {"enable_intensity_distribution_adaptation", param_value_string(enable_intensity_distribution_adaptation)},
            {"use_range_binned_intensity_threshold", param_value_string(use_range_binned_intensity_threshold)},
            {"use_idsor_intensity_threshold", param_value_string(use_idsor_intensity_threshold)},
            {"enable_vertical_zscore", param_value_string(enable_vertical_zscore)},
            {"use_scattering_term", param_value_string(use_scattering_term)},
            {"use_soft_consistency", param_value_string(use_soft_consistency)},
            {"use_otsu_intensity_threshold", param_value_string(use_otsu_intensity_threshold)},
            {"enable_intensity_scale_autocal", param_value_string(enable_intensity_scale_autocal)},
            {"enable_sensor_height_roi", param_value_string(enable_sensor_height_roi)},
            {"enable_range_selfcal", param_value_string(enable_range_selfcal)},
            {"enable_sparse_support_expand", param_value_string(enable_sparse_support_expand)},
            {"fix_quantile_sentinel", param_value_string(fix_quantile_sentinel)},
            {"use_fast_elevation_gate", param_value_string(use_fast_elevation_gate)},
            {"use_threshold_lut", param_value_string(use_threshold_lut)},
            {"strict_evaluation", param_value_string(strict_evaluation)},
        };
    }
    // 打印开关状态
    void print_switches(std::ostream& os = std::cout) const {
        os << "\n============ 消融实验开关状态 ============" << std::endl;
        os << "=== 1. 预处理模块 ===" << std::endl;
        os << "预降采样: " << (enable_pre_downsampling ? "开启" : "关闭") << std::endl;
        os << "自适应叶子大小: " << (use_adaptive_leaf_size ? "开启" : "关闭") << std::endl;
        os << "高度距离过滤: " << (enable_height_distance_filter ? "开启" : "关闭") << std::endl;
        os << "保守过滤策略: " << (use_conservative_filtering ? "开启" : "关闭") << std::endl;
        os << "离群点过滤: " << (enable_outlier_removal ? "开启" : "关闭") << std::endl;
        os << "精确重复点合并: " << (enable_dedup_exact_points ? "开启" : "关闭") << std::endl;
        os << "最低扫描环排除: " << (enable_lowest_ring_exclusion ? "开启" : "关闭") << std::endl;

        os << "=== 2. 参数优化模块 ===" << std::endl;
        os << "网格搜索优化: " << (enable_grid_search_optimization ? "开启" : "关闭") << std::endl;
        os << "特征推荐参数: " << (enable_feature_based_recommendation ? "开启" : "关闭") << std::endl;
        os << "自适应参数调整: " << (enable_adaptive_parameter_adjustment ? "开启" : "关闭") << std::endl;

        os << "=== 3. 特征计算模块 ===" << std::endl;
        os << "强度直方图统计: " << (use_intensity_histogram_stats ? "开启" : "关闭") << std::endl;
        os << "自适应强度阈值: " << (use_adaptive_intensity_threshold ? "开启" : "关闭") << std::endl;
        os << "平面度计算: " << (enable_planarity_calculation ? "开启" : "关闭") << std::endl;
        os << "平面度缓存: " << (enable_planarity_cache ? "开启" : "关闭") << std::endl;
        os << "密度计算: " << (enable_density_calculation ? "开启" : "关闭") << std::endl;
        os << "相对高度: " << (enable_relative_height ? "开启" : "关闭") << std::endl;
        os << "高度一致性: " << (enable_height_consistency ? "开启" : "关闭") << std::endl;
        os << "特征熵计算: " << (enable_feature_entropy ? "开启" : "关闭") << std::endl;
        os << "场景复杂度: " << (enable_scene_complexity ? "开启" : "关闭") << std::endl;
        os << "权重自适应调整: " << (enable_adaptive_weight_adjustment ? "开启" : "关闭") << std::endl;

        os << "=== 4. 雪点检测核心算法 ===" << std::endl;
        os << "极低强度直接判定: " << (enable_ultra_low_intensity_direct ? "开启" : "关闭") << std::endl;
        os << "孤立点特殊处理: " << (enable_isolated_point_handling ? "开启" : "关闭") << std::endl;
        os << "局部阈值调整: " << (enable_local_threshold_adjustment ? "开启" : "关闭") << std::endl;
        os << "零强度表面抑制: " << (enable_zero_intensity_surface_suppression ? "开启" : "关闭") << std::endl;
        os << "邻域强度分析: " << (enable_neighbor_intensity_analysis ? "开启" : "关闭") << std::endl;
        os << "强度对比检查: " << (enable_intensity_comparison ? "开启" : "关闭") << std::endl;
        os << "平面度过滤: " << (enable_planarity_filtering ? "开启" : "关闭") << std::endl;
        os << "高度一致性检查: " << (enable_height_consistency_check ? "开启" : "关闭") << std::endl;

        os << "=== 5. 后处理模块 ===" << std::endl;
        os << "降采样映射传播: " << (enable_downsampled_mapping_propagation ? "开启" : "关闭") << std::endl;

        os << "=== 6. 性能优化 ===" << std::endl;
        os << "OpenMP并行: " << (enable_openmp_parallel ? "开启" : "关闭") << std::endl;
        os << "块处理: " << (enable_block_processing ? "开启" : "关闭") << std::endl;
        os << "动态块大小: " << (use_dynamic_block_size ? "开启" : "关闭") << std::endl;
        os << "特征缓存: " << (enable_feature_cache ? "开启" : "关闭") << std::endl;
        os << "KD树缓存: " << (enable_kdtree_cache ? "开启" : "关闭") << std::endl;
        os << "采样优化: " << (enable_sampling_optimization ? "开启" : "关闭") << std::endl;

        os << "=== 7. 场景自适应 ===" << std::endl;
        os << "复杂度自适应: " << (enable_complexity_adaptation ? "开启" : "关闭") << std::endl;
        os << "密度自适应: " << (enable_density_adaptation ? "开启" : "关闭") << std::endl;
        os << "强度分布自适应: " << (enable_intensity_distribution_adaptation ? "开启" : "关闭") << std::endl;

        os << "=== 8. 进化方向 ===" << std::endl;
        os << "距离分段强度阈值: " << (use_range_binned_intensity_threshold ? "开启" : "关闭") << std::endl;
        os << "IDSOR平滑阈值: " << (use_idsor_intensity_threshold ? "开启" : "关闭") << std::endl;
        os << "垂直z-score特征: " << (enable_vertical_zscore ? "开启" : "关闭") << std::endl;
        os << "散射度替代平面度: " << (use_scattering_term ? "开启" : "关闭") << std::endl;
        os << "一致性软项: " << (use_soft_consistency ? "开启" : "关闭") << std::endl;
        os << "Otsu强度阈值: " << (use_otsu_intensity_threshold ? "开启" : "关闭") << std::endl;

        os << "=== 9. 可移植性（跨传感器） ===" << std::endl;
        os << "强度量纲自标定(δ_I): " << (enable_intensity_scale_autocal ? "开启" : "关闭") << std::endl;
        os << "安装高度导出ROI: " << (enable_sensor_height_roi ? "开启" : "关闭") << std::endl;
        os << "距离自标定(孤立率): " << (enable_range_selfcal ? "开启" : "关闭") << std::endl;
        os << "稀疏扩支撑半径: " << (enable_sparse_support_expand ? "开启" : "关闭") << std::endl;
        os << "分位数哨兵修复: " << (fix_quantile_sentinel ? "开启" : "关闭") << std::endl;
        os << "快速仰角判定: " << (use_fast_elevation_gate ? "开启" : "关闭") << std::endl;
        os << "阈值查表: " << (use_threshold_lut ? "开启" : "关闭") << std::endl;

        os << "=== 10. 评估口径 ===" << std::endl;
        os << "严格评估口径: " << (strict_evaluation ? "开启" : "关闭") << std::endl;
        os << "======================================" << std::endl;
    }
};

}  // namespace snowclear

#endif  // ABLATION_SWITCHES_H
