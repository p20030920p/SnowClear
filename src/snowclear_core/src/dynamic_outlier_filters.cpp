#include "snowclear/dynamic_outlier_filters.hpp"

// =====================================================================
// 本文件可调参数（对比基准 DROR/DSOR/SOR/ROR，经 launch 或 _param:=value 覆盖）
// ---------------------------------------------------------------------
//   dror_r_min            0.5     DROR 最小搜索半径（米）
//   dror_alpha            0.035   DROR 半径 = max(r_min, alpha×range)
//   dror_min_neighbors    2       DROR 半径内最少邻居数（不含自身）
//   dsor_knn              8       DSOR KNN 近邻数
//   dsor_std_mult         2.5     DSOR 阈值 = 均值 + std_mult×标准差
//   dsor_bin_size         5.0     DSOR 距离分箱大小（米）
//   sor_mean_k            8       SOR（PCL 官方）统计近邻数
//   sor_std_mult          1.0     SOR（PCL 官方）标准差倍数
//   ror_radius            0.8     ROR（PCL 官方）搜索半径（米）
//   ror_min_neighbors     2       ROR（PCL 官方）半径内最少邻居数
// =====================================================================

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <omp.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace snowclear {

// =====================================================================
// 实现说明与引用
// ---------------------------------------------------------------------
// 本文件实现的是公开文献中的经典非学习式滤波器，作为论文对比基准使用：
//   - DROR: N. Charron, S. Phillips, S. L. Waslander,
//           "De-noising of Lidar Point Clouds Corrupted by Snowfall",
//           CRV 2018.
//   - DSOR: A. Kurup, J. Bos, "DSOR: A Scalable Statistical Filter for
//           Removing Falling Snow from LiDAR Point Clouds in Severe
//           Winter Weather", 2021.
// 实现为自行编写，未复制任何第三方代码；论文中必须引用上述文献。
// =====================================================================

namespace {

double point_range(const CloudPoint& pt) {
    return std::sqrt(pt.x * pt.x + pt.y * pt.y);
}

}  // namespace

// =====================================================================
// DROR：动态半径离群点移除
// ---------------------------------------------------------------------
// 对每个点：搜索半径 = max(r_min, alpha × 水平距离)。
// 半径内邻居数（不含自身）< dror_min_neighbors 判为雪点/离群点。
// 动机：LiDAR 点云随距离变稀疏，固定半径会误删远处真实目标；
//       动态半径让远近使用一致的“邻居数”标准。
// 返回：输入点云坐标系下的离群点索引。
// =====================================================================
void DynamicOutlierFilters::dror(const CloudPtr& cloud,
                                 const pcl::KdTreeFLANN<CloudPoint>::Ptr& tree,
                                 const DynamicFilterParams& params,
                                 std::vector<int>& outlier_indices) {
    outlier_indices.clear();
    if (!cloud || cloud->empty() || !tree) return;

    std::vector<bool> is_outlier(cloud->size(), false);

    #pragma omp parallel for schedule(dynamic, 256)
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& point = cloud->points[i];
        double range = point_range(point);
        double radius = std::max(params.dror_r_min, params.dror_alpha * range);

        std::vector<int> nn_indices;
        std::vector<float> nn_dists;
        int found = tree->radiusSearch(point, static_cast<float>(radius), nn_indices, nn_dists);

        // PCL 返回包含自身，因此有效邻居数 = found - 1
        int neighbors = std::max(0, found - 1);
        if (neighbors < params.dror_min_neighbors) {
            is_outlier[i] = true;
        }
    }

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (is_outlier[i]) {
            outlier_indices.push_back(static_cast<int>(i));
        }
    }
}

// =====================================================================
// DSOR：动态统计离群点移除
// ---------------------------------------------------------------------
// 1. 每个点计算到 K 个近邻的平均距离
// 2. 按水平距离分箱（bin_size），统计每箱平均距离的均值 μ 与标准差 σ
// 3. 平均距离 > μ + dsor_std_mult × σ 的点判为雪点/离群点
// 动机：近处点云密集、远处稀疏，按距离分箱使判定标准随距离变化。
// =====================================================================
void DynamicOutlierFilters::dsor(const CloudPtr& cloud,
                                 const pcl::KdTreeFLANN<CloudPoint>::Ptr& tree,
                                 const DynamicFilterParams& params,
                                 std::vector<int>& outlier_indices) {
    outlier_indices.clear();
    if (!cloud || cloud->empty() || !tree) return;

    const int k = std::max(2, params.dsor_knn);
    const double bin_size = std::max(0.5, params.dsor_bin_size);

    std::vector<float> mean_dists(cloud->size(), 0.0f);

    #pragma omp parallel for schedule(dynamic, 256)
    for (size_t i = 0; i < cloud->size(); ++i) {
        std::vector<int> nn_indices;
        std::vector<float> nn_dists;
        int found = tree->nearestKSearch(cloud->points[i], k + 1, nn_indices, nn_dists);

        if (found <= 1) {
            mean_dists[i] = 1000.0f;  // 孤立点
            continue;
        }

        float sum = 0.0f;
        int count = 0;
        for (int j = 1; j < found; ++j) {  // 跳过自身
            sum += nn_dists[j];
            count++;
        }
        mean_dists[i] = count > 0 ? sum / count : 1000.0f;
    }

    // 按距离分箱统计
    struct BinStats {
        double sum = 0.0;
        double sum_sq = 0.0;
        size_t count = 0;
    };
    std::unordered_map<int, BinStats> bins;

    for (size_t i = 0; i < cloud->size(); ++i) {
        int bin = static_cast<int>(point_range(cloud->points[i]) / bin_size);
        BinStats& stats = bins[bin];
        stats.sum += mean_dists[i];
        stats.sum_sq += static_cast<double>(mean_dists[i]) * mean_dists[i];
        stats.count++;
    }

    std::unordered_map<int, std::pair<double, double>> thresholds;  // bin -> (mean, threshold)
    for (const auto& entry : bins) {
        double mean = entry.second.sum / entry.second.count;
        double variance = entry.second.sum_sq / entry.second.count - mean * mean;
        double stddev = std::sqrt(std::max(0.0, variance));
        thresholds[entry.first] = {mean, mean + params.dsor_std_mult * stddev};
    }

    for (size_t i = 0; i < cloud->size(); ++i) {
        int bin = static_cast<int>(point_range(cloud->points[i]) / bin_size);
        auto it = thresholds.find(bin);
        if (it != thresholds.end() && mean_dists[i] > it->second.second) {
            outlier_indices.push_back(static_cast<int>(i));
        }
    }
}

// =====================================================================
// SOR：PCL 官方 StatisticalOutlierRemoval（Rusu et al., 2008）
// ---------------------------------------------------------------------
// 统计每个点到 K 近邻的平均距离，全局均值为 μ、标准差为 σ，
// 平均距离 > μ + sor_std_mult×σ 的点判为离群（雪）点。
// 注意: PCL 1.10 的 getRemovedIndices() 不随 setNegative 填充（实测为空），
//       因此用 (x,y,z,intensity) 位模式多重集把负滤波输出匹配回输入下标。
// =====================================================================
namespace {

struct PtKey64 {
    uint64_t lo, hi;
    bool operator==(const PtKey64& o) const { return lo == o.lo && hi == o.hi; }
};
struct PtKey64Hash {
    size_t operator()(const PtKey64& k) const {
        return static_cast<size_t>(k.lo ^ (k.hi * 0x9e3779b97f4a7c15ULL));
    }
};

inline PtKey64 key_of(const CloudPoint& p) {
    uint32_t bx, by, bz, bi;
    std::memcpy(&bx, &p.x, 4);
    std::memcpy(&by, &p.y, 4);
    std::memcpy(&bz, &p.z, 4);
    std::memcpy(&bi, &p.intensity, 4);
    return PtKey64{(static_cast<uint64_t>(bx) << 32) | by,
                   (static_cast<uint64_t>(bz) << 32) | bi};
}

// 把“负滤波输出（离群点云）”多重集匹配回输入点云下标
void match_back_indices(const CloudPtr& input,
                        const pcl::PointCloud<CloudPoint>& outliers,
                        std::vector<int>& outlier_indices) {
    std::unordered_map<PtKey64, int, PtKey64Hash> counts;
    counts.reserve(outliers.size() * 2);
    for (const auto& p : outliers.points) {
        counts[key_of(p)]++;
    }
    for (size_t i = 0; i < input->size(); ++i) {
        auto it = counts.find(key_of(input->points[i]));
        if (it != counts.end() && it->second > 0) {
            it->second--;
            outlier_indices.push_back(static_cast<int>(i));
        }
    }
}

}  // namespace

void DynamicOutlierFilters::sor(const CloudPtr& cloud,
                                const DynamicFilterParams& params,
                                std::vector<int>& outlier_indices) {
    outlier_indices.clear();
    if (!cloud || cloud->empty()) return;

    pcl::StatisticalOutlierRemoval<CloudPoint> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(std::max(2, params.sor_mean_k));
    filter.setStddevMulThresh(params.sor_std_mult);
    filter.setNegative(true);  // 输出离群点

    pcl::PointCloud<CloudPoint> filtered;
    filter.filter(filtered);

    match_back_indices(cloud, filtered, outlier_indices);
}

// =====================================================================
// ROR：PCL 官方 RadiusOutlierRemoval（Rusu, 2009）
// ---------------------------------------------------------------------
// 半径 ror_radius 内邻居数（不含自身）< ror_min_neighbors 的点判为离群（雪）点。
// =====================================================================
void DynamicOutlierFilters::ror(const CloudPtr& cloud,
                                const DynamicFilterParams& params,
                                std::vector<int>& outlier_indices) {
    outlier_indices.clear();
    if (!cloud || cloud->empty()) return;

    pcl::RadiusOutlierRemoval<CloudPoint> filter;
    filter.setInputCloud(cloud);
    filter.setRadiusSearch(params.ror_radius);
    filter.setMinNeighborsInRadius(params.ror_min_neighbors);
    filter.setNegative(true);

    pcl::PointCloud<CloudPoint> filtered;
    filter.filter(filtered);

    match_back_indices(cloud, filtered, outlier_indices);
}

}  // namespace snowclear