#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include "snowclear/cloud_operations_types.hpp"
#include "snowclear/ablation_switches.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <unordered_map>
#include <vector>

namespace snowclear {

// =============== 特征提取模块 =============== //
// 职责: 场景级特征统计、逐点几何特征（平面度/熵）、KD树管理。
// 注意：逐点特征缓存由 SnowDetector 以线程本地方式持有，本类不再管理缓存。
class FeatureExtractor {
public:
    explicit FeatureExtractor(const AblationSwitches& switches);

    // 场景级特征分析：强度统计/密度/噪声比/复杂度（按需构建/复用 KD 树）
    CloudFeatures analyze(const CloudPtr& cloud);

    // KD 树管理：点云变化时调用；缓存开关关闭时不建树
    void ensure_kdtree(const CloudPtr& cloud);
    CloudPtr current_cloud() const { return last_cloud_; }

    // K 近邻查询（返回命中数，未建树返回 0）
    int nearest_k(const CloudPoint& point, int k,
                  std::vector<int>& indices, std::vector<float>& dists) const;
    // 半径邻域查询（返回命中数，未建树返回 0）
    int radius_search(const CloudPoint& point, float radius,
                      std::vector<int>& indices, std::vector<float>& dists) const;

    // 局部平面度（PCA：(λ2-λ3)/λ1，真正的地面/墙面表面性度量；
    // 雪点团簇平面度低、scattering 高），含调用方提供的缓存
    double planarity(size_t point_idx,
                     const std::vector<int>& neighbor_indices,
                     const CloudPtr& cloud,
                     std::vector<PointFeature>& cache,
                     double* scattering_out = nullptr);

    // 离地高度：先按 1m 网格估计局部地面高度，再返回点相对地面的归一化高度
    void build_ground_grid(const CloudPtr& cloud);
    float height_above_ground(const CloudPoint& point) const;
    bool ground_grid_empty() const { return ground_grid_.empty(); }

    // FlakeOut 式垂直 z-score：(z - 局部均值)/局部标准差，>0 表示高于局部表面
    float vertical_zscore(const CloudPoint& point) const;
    // 局部特征熵（强度/高度/空间方差加权）
    double feature_entropy(const CloudPtr& cloud,
                           const std::vector<int>& neighbor_indices) const;

private:
    void compute_intensity_histogram_stats(const CloudPtr& cloud,
                                           float& mean, float& stddev,
                                           float& q1, float& median, float& q3) const;

    const AblationSwitches& switches_;
    pcl::KdTreeFLANN<CloudPoint>::Ptr kdtree_;
    bool kdtree_initialized_ = false;
    CloudPtr last_cloud_;
    std::unordered_map<int64_t, float> ground_grid_;  // 1m 网格 -> 局部地面 z
    float ground_grid_fallback_ = 0.0f;               // 无网格时的兜底地面高度
    std::unordered_map<int64_t, std::pair<float, float>> cell_zstats_;  // 网格 -> (均值, 标准差)
};

}  // namespace snowclear

#endif  // FEATURE_EXTRACTOR_H
