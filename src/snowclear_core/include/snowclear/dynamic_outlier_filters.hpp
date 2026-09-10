#ifndef DYNAMIC_OUTLIER_FILTERS_H
#define DYNAMIC_OUTLIER_FILTERS_H

#include "snowclear/cloud_operations_types.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

namespace snowclear {

// =============== 主流非学习式雪点滤波器 =============== //
// DROR: Dynamic Radius Outlier Removal (Charron et al., 2018)
//   - 搜索半径随距离增大而增大，补偿 LiDAR 远距离稀疏性
// DSOR: Dynamic Statistical Outlier Removal (Kurup & Bos, 2021)
//   - 按距离分箱统计 KNN 平均距离，超过 均值+倍数*标准差 判为雪点
// SOR: PCL 官方 StatisticalOutlierRemoval (Rusu et al., 2008)
//   - 全局统计 KNN 平均距离，超过 均值+倍数*标准差 判为离群/雪点
// ROR: PCL 官方 RadiusOutlierRemoval (Rusu, 2009)
//   - 半径内邻居数不足阈值判为离群/雪点
struct DynamicFilterParams {
    // DROR
    double dror_r_min = 0.5;       // 最小搜索半径 (m)
    double dror_alpha = 0.035;     // 半径 = max(r_min, alpha * range)
    int dror_min_neighbors = 2;    // 半径内至少需要的邻居数（不含自身）

    // DSOR
    int dsor_knn = 8;              // KNN 近邻数
    double dsor_std_mult = 2.5;    // 标准差倍数
    double dsor_bin_size = 5.0;    // 距离分箱大小 (m)

    // SOR（PCL StatisticalOutlierRemoval）
    int sor_mean_k = 8;            // 统计近邻数（PCL 默认 8）
    double sor_std_mult = 1.0;     // 标准差倍数（PCL 默认 1.0）

    // ROR（PCL RadiusOutlierRemoval）
    double ror_radius = 0.8;       // 搜索半径 (m)
    int ror_min_neighbors = 2;     // 半径内最少邻居数（不含自身）
};

class DynamicOutlierFilters {
public:
    // 返回在输入点云坐标系下的离群点（雪点）索引
    static void dror(const CloudPtr& cloud,
                     const pcl::KdTreeFLANN<CloudPoint>::Ptr& tree,
                     const DynamicFilterParams& params,
                     std::vector<int>& outlier_indices);

    static void dsor(const CloudPtr& cloud,
                     const pcl::KdTreeFLANN<CloudPoint>::Ptr& tree,
                     const DynamicFilterParams& params,
                     std::vector<int>& outlier_indices);

    // PCL 官方 SOR（统计离群点移除）
    static void sor(const CloudPtr& cloud,
                    const DynamicFilterParams& params,
                    std::vector<int>& outlier_indices);

    // PCL 官方 ROR（半径离群点移除）
    static void ror(const CloudPtr& cloud,
                    const DynamicFilterParams& params,
                    std::vector<int>& outlier_indices);
};

}  // namespace snowclear

#endif  // DYNAMIC_OUTLIER_FILTERS_H
