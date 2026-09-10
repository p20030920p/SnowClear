#ifndef PARAMETER_OPTIMIZER_H
#define PARAMETER_OPTIMIZER_H

#include "snowclear/cloud_operations_types.hpp"
#include "snowclear/ablation_switches.hpp"

#include <vector>

namespace snowclear {

class SnowDetector;

// =============== 参数优化模块 =============== //
// 职责: 特征推荐参数、网格搜索优化、自适应强度阈值/块大小/特征权重。
// 依赖 SnowDetector 完成候选评估，依赖 CloudFeatures 完成场景自适应。
class ParameterOptimizer {
public:
    ParameterOptimizer(const AblationSwitches& switches,
                       SnowDetector& detector,
                       const FilterParameters& defaults);

    // 自适应强度阈值（Q1 驱动 + 复杂度/分布修正，钳制 [2.5, 8.0]）
    float adaptive_intensity_threshold(const CloudFeatures& features) const;

    // 距离分段强度阈值：按 0-5/5-10/10-15/15-18m 分段，段内样本过少回退全局
    RangeIntensityThreshold compute_range_thresholds(const CloudPtr& cloud,
                                                     const CloudFeatures& features) const;

    // Otsu 强度阈值：按强度直方图最大化类间方差（经典非学习双峰切分）
    float compute_otsu_threshold(const CloudPtr& cloud) const;

    // 动态块大小（按点数缩放）
    int dynamic_block_size(size_t point_count, int default_block) const;

    // 场景自适应权重调整（归一化）
    void adjust_feature_weights_for_scene(FilterParameters& params,
                                          const CloudFeatures& features) const;

    // 基于场景特征的参数推荐（无 GT 时使用）
    FilterParameters recommend(const CloudFeatures& features) const;

    // 参数优化入口：网格搜索 > 特征推荐 > 默认参数
    FilterParameters optimize(const CloudPtr& cloud,
                              const std::vector<int>& ground_truth_indices,
                              const CloudFeatures& features,
                              int max_iterations,
                              int max_grid_search_points) const;

    // 参数评估（Fβ 加权 + 精度/召回惩罚）
    double evaluate(const CloudPtr& cloud,
                    const std::vector<int>& ground_truth_indices,
                    const CloudFeatures& features,
                    const FilterParameters& params) const;

    // 体素降采样 + 最近点映射
    CloudPtr downsample(const CloudPtr& cloud,
                        std::vector<int>& downsampled_to_original,
                        float leaf_size) const;

    // 网格搜索专用降采样（0.08~0.12m）
    CloudPtr downsample_for_optimization(const CloudPtr& cloud,
                                         std::vector<int>& downsampled_to_original) const;

private:
    // 网格搜索：迭代搜索 DCOR/RGOR/epsilon/权重组合
    FilterParameters grid_search_optimize(const CloudPtr& cloud,
                                          const std::vector<int>& ground_truth_indices,
                                          const CloudFeatures& features,
                                          int max_iterations,
                                          int max_grid_search_points) const;

    const AblationSwitches& switches_;
    SnowDetector& detector_;
    FilterParameters defaults_;
};

}  // namespace snowclear

#endif  // PARAMETER_OPTIMIZER_H
