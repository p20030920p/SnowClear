#ifndef SNOW_DETECTOR_H
#define SNOW_DETECTOR_H

#include "snowclear/cloud_operations_types.hpp"
#include "snowclear/ablation_switches.hpp"
#include "snowclear/feature_extractor.hpp"

#include <cstdint>
#include <unordered_map>

namespace snowclear {

// =============== 雪点检测模块 =============== //
// 职责: 基于强度门控 + 多特征融合得分的逐点判定，以及降采样映射传播。
// 依赖 FeatureExtractor 提供 KD 树与几何特征。
// 线程安全：逐点特征缓存按线程持有（thread_caches_），串并行结果一致。
//
// 逐点判定核心逻辑只实现一次（evaluate_point），detect_parallel/detect_serial
// 仅负责调度，保证串/并行结果逐位一致。
class SnowDetector {
public:
    SnowDetector(const AblationSwitches& switches,
                 FeatureExtractor& extractor,
                 int knn_k,
                 float min_intensity_score,
                 float planarity_filter_threshold,
                 float direct_planarity_gate,
                 float zero_intensity_support_radius,
                 float zero_intensity_support_min_intensity,
                 float zero_intensity_support_range_floor);

    // 基础检测：强度门控 + 多特征融合得分，输出候选雪点索引（当前坐标系）
    // thresholds: 全局或距离分段强度阈值；feature_weights 内含 score_threshold
    void detect(const CloudPtr& cloud,
                const RangeIntensityThreshold& thresholds,
                const FilterParameters& feature_weights,
                pcl::PointIndices::Ptr& snow_indices,
                int block_size);

    // 降采样映射回原始点云 + 保守传播（低强度/高度接近/距离近）
    void map_and_propagate(const CloudPtr& input_cloud,
                           pcl::PointIndices::Ptr& snow_indices,
                           const std::vector<int>& downsampled_to_original,
                           const CloudFeatures& features,
                           const RangeIntensityThreshold& thresholds);

    // 新帧预处理后清空逐点特征缓存
    void clear_caches() { thread_caches_.clear(); }

    // 可移植性：用自标定的强度量化步长 δ_I 覆盖硬编码的"支撑点强度下界"。
    // 原硬编码 1.0 只在 0..255 量纲下成立；CADC(VLP-32C) 的 intensity 归一化到
    // 0..1，无任何点满足 I>1.0 -> 支撑点集为空 -> 表面抑制整体失效
    // -> 90.16% 的 ROI 点被判成雪（实测）。
    void set_support_min_intensity(float v) { zero_intensity_support_min_intensity_ = v; }
    float support_min_intensity() const { return zero_intensity_support_min_intensity_; }

    // 稀疏传感器：邻点不足时扩大支撑半径（避免"采样不足"被误判成"孤立"）
    void set_sparse_expand(bool on, int min_neighbors, float max_radius) {
        sparse_expand_ = on;
        sparse_min_neighbors_ = min_neighbors;
        sparse_max_radius_ = max_radius;
    }

private:
    // 当前配置下是否需要 KD 树/KNN（最终配置全关时可直接跳过）
    bool needs_knn() const;

    // 单点判定核心（串/并行共用同一实现）
    // 判定为雪点候选时把点索引 i 加入 out
    void evaluate_point(size_t i,
                        const CloudPtr& cloud,
                        const RangeIntensityThreshold& thresholds,
                        const FilterParameters& feature_weights,
                        std::vector<PointFeature>& cache,
                        std::vector<int>& out);

    // OpenMP 并行检测：每线程独立结果向量与特征缓存，块级动态调度
    void detect_parallel(const CloudPtr& cloud,
                         const RangeIntensityThreshold& thresholds,
                         const FilterParameters& feature_weights,
                         int block_size,
                         std::vector<std::vector<int>>& thread_local_results);

    // 串行检测：与并行版逐点逻辑一致（使用 thread_caches_[0]）
    void detect_serial(const CloudPtr& cloud,
                       const RangeIntensityThreshold& thresholds,
                       const FilterParameters& feature_weights,
                       int block_size,
                       std::vector<int>& results);

    // 零强度点表面抑制：构建“强度 > min_I”的支撑点空间哈希网格；
    // 对每个 I=0 点查找半径 radius 内是否存在支撑点。网格只读，可被 OpenMP 并行查询。
    void build_zero_intensity_support_grid(const CloudPtr& cloud);
    bool has_support_point_within(const CloudPoint& point, const CloudPtr& cloud) const;

    const AblationSwitches& switches_;
    FeatureExtractor& extractor_;
    int knn_k_;                  // KNN 近邻数（最终配置默认 5）
    float min_intensity_score_;  // 强度预筛门槛（低于此分数直接跳过 KNN）
    float planarity_filter_threshold_;  // 平面度过滤门控
    float direct_planarity_gate_;       // 超低强度直判的平面度门控
    float zero_intensity_support_radius_;       // I=0 表面支撑判定半径（米）
    float zero_intensity_support_min_intensity_;  // 支撑点强度下界
    float zero_intensity_support_range_floor_;     // 近场豁免水平距离（米）
    std::unordered_map<int64_t, std::vector<int>> support_grid_;  // I>min_I 点空间哈希
    std::unordered_map<int64_t, std::vector<int>> all_point_grid_;  // 全点哈希（仅稀疏扩半径时构建）
    bool sparse_expand_ = false;
    int sparse_min_neighbors_ = 30;
    float sparse_max_radius_ = 1.5f;
    std::vector<std::vector<PointFeature>> thread_caches_;  // 线程本地特征缓存
};

}  // namespace snowclear

#endif  // SNOW_DETECTOR_H
