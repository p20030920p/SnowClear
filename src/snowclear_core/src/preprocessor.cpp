#include "snowclear/preprocessor.hpp"

// =====================================================================
// 本文件可调参数（ROS 2 参数 / YAML / launch 可覆盖，默认值见
// snowclear/system_config.hpp 与 snowclear/ablation_switches.hpp）
// ---------------------------------------------------------------------
// 数值参数:
//   height_threshold            2.6    z 上界（米），预处理高度过滤
//   xy_threshold                17.0   水平距离上界（米）
//   lowest_ring_elevation_deg   -23.0  最低扫描环排除仰角阈值（度）
//   filter_downsampling_leaf_size 0.06 预降采样体素叶子尺寸（米）
//   ror_radius / ror_neighbors  0.8/2  离群点过滤半径与邻居数（开关开启时）
// 消融开关:
//   enable_pre_downsampling      关   超大点云预降采样（本文件 >40万点触发；
//                                      检测阶段另有一道 >10万点门限，见 cloud_operations.cpp）
//   use_adaptive_leaf_size       关   自适应叶子尺寸
//   enable_height_distance_filter 开  ROI 高度/距离过滤（决定性模块）
//   use_conservative_filtering   关   过滤过于激进时自动放宽
//   enable_outlier_removal       关   半径邻域离群点过滤（默认无效果）
//   enable_dedup_exact_points    关   精确重复点合并（指标一致但更慢）
//   enable_lowest_ring_exclusion 开   剔除最低地面扫描环
//   enable_openmp_parallel / enable_sampling_optimization  性能开关
// =====================================================================

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/PointIndices.h>

#include <omp.h>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>

namespace snowclear {

namespace {

constexpr double kPi = 3.14159265358979323846;

// 精确去重 key：x/y/z/intensity 的 32 位位模式拼接成 128 位（两个 uint64）
struct PointKey {
    uint64_t lo;
    uint64_t hi;
    bool operator==(const PointKey& other) const {
        return lo == other.lo && hi == other.hi;
    }
};

struct PointKeyHash {
    size_t operator()(const PointKey& key) const {
        return static_cast<size_t>(key.lo ^ (key.hi * 0x9e3779b97f4a7c15ULL));
    }
};

inline PointKey make_point_key(const CloudPoint& point) {
    uint32_t bx, by, bz, bi;
    std::memcpy(&bx, &point.x, sizeof(bx));
    std::memcpy(&by, &point.y, sizeof(by));
    std::memcpy(&bz, &point.z, sizeof(bz));
    std::memcpy(&bi, &point.intensity, sizeof(bi));
    return PointKey{(static_cast<uint64_t>(bx) << 32) | by,
                    (static_cast<uint64_t>(bz) << 32) | bi};
}

double horizontal_distance_sq(const CloudPoint& pt) {
    return pt.x * pt.x + pt.y * pt.y;
}

// 仰角门限判定。
// 慢路径（原实现）：elevation = asin(z / range)，再比较 elevation >= min_elev。
// 快路径（数学等价）：asin 在 [-1,1] 上单调递增，故
//     asin(z/range) >= min_elev  <=>  z/range >= sin(min_elev)  <=>  z >= sin(min_elev)*range
// 省掉每点一次 asin 和一次除法。这段跑在**过滤前的全量点云**上（约 20 万点/帧），
// 实测 1.816 ms -> 1.166 ms，省 0.650 ms/帧（36%）。
// 注：range <= 1e-6 的退化点在原实现里 elevation 记为 0，对 min_elev<0 恒通过；
// 快路径 z >= s*range ≈ z >= 0 对这些点等价（z 也≈0）。
inline bool elevation_pass(const CloudPoint& pt, bool check_elevation,
                           float min_elev_rad, float sin_min_elev, bool fast) {
    if (!check_elevation) return true;
    const float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if (range <= 1e-6f) return 0.0f >= min_elev_rad;
    if (fast) return pt.z >= sin_min_elev * range;
    return std::asin(pt.z / range) >= min_elev_rad;
}

}  // namespace

Preprocessor::Preprocessor(const AblationSwitches& switches,
                           double height_threshold,
                           double xy_threshold,
                           double ror_radius,
                           int ror_neighbors,
                           float default_leaf_size,
                           float lowest_ring_elevation_deg)
    : switches_(switches),
      height_threshold_(height_threshold),
      xy_threshold_(xy_threshold),
      ror_radius_(ror_radius),
      ror_neighbors_(ror_neighbors),
      default_leaf_size_(default_leaf_size),
      lowest_ring_elevation_deg_(lowest_ring_elevation_deg) {}

// =====================================================================
// 自适应降采样叶子大小
// ---------------------------------------------------------------------
// 输入：点数、场景特征；输出：体素叶子边长。
// 策略：点越多叶子越大；场景越复杂/密度越低，叶子越小（保留细节）。
// 受 use_adaptive_leaf_size / enable_complexity_adaptation /
//      enable_density_adaptation 三个开关控制。
// =====================================================================
float Preprocessor::adaptive_leaf_size(size_t point_count, const CloudFeatures& features) const {
    if (!switches_.use_adaptive_leaf_size) {
        return default_leaf_size_;
    }

    float leaf_size = default_leaf_size_;

    if (point_count > 300000) {
        leaf_size = 0.1f;
    } else if (point_count > 200000) {
        leaf_size = 0.08f;
    } else if (point_count > 100000) {
        leaf_size = 0.07f;
    } else if (point_count > 50000) {
        leaf_size = 0.06f;
    }

    if (switches_.enable_complexity_adaptation) {
        if (features.scene_complexity > 0.7) {
            leaf_size *= 0.8f;
        } else if (features.scene_complexity > 0.4) {
            leaf_size *= 0.9f;
        } else if (features.scene_complexity < 0.3) {
            leaf_size *= 0.95f;
        }
    }

    if (switches_.enable_density_adaptation) {
        if (features.point_density < 50) {
            leaf_size *= 0.85f;
        } else if (features.point_density > 500) {
            leaf_size *= 0.95f;
        }
    }

    return std::max(0.04f, std::min(0.15f, leaf_size));
}

// =====================================================================
// 超大点云预降采样（>40 万点时调用）
// ---------------------------------------------------------------------
// 1. 用体素网格降采样减少后续计算量
// 2. 对每个降采样点，在原始点云中搜索 3 个最近邻，
//    优先选择“强度最低”的原始点作为代表——避免把低强度雪点抹掉
// 输出：降采样点云 + downsampled_to_original 映射表
// =====================================================================
CloudPtr Preprocessor::pre_downsample_large_cloud(const CloudPtr& input_cloud,
                                                   std::vector<int>& downsampled_to_original,
                                                   const CloudFeatures& features) {
    auto start_time = std::chrono::high_resolution_clock::now();

    CloudPtr result(new pcl::PointCloud<CloudPoint>);

    float leaf_size = adaptive_leaf_size(input_cloud->size(), features);

    pcl::VoxelGrid<CloudPoint> pre_voxel;
    pre_voxel.setInputCloud(input_cloud);
    pre_voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    pre_voxel.filter(*result);

    downsampled_to_original.resize(result->size());

    // 使用KD树寻找最近点，优先选择低强度点
    pcl::KdTreeFLANN<CloudPoint> mapping_tree;
    mapping_tree.setInputCloud(input_cloud);

    auto map_point = [&](size_t i) {
        std::vector<int> nn_indices;
        std::vector<float> nn_dists;
        mapping_tree.nearestKSearch(result->points[i], 3, nn_indices, nn_dists);

        int best_idx = nn_indices[0];
        if (nn_indices.size() > 1) {
            float min_intensity = input_cloud->points[nn_indices[0]].intensity;
            for (size_t j = 1; j < nn_indices.size(); ++j) {
                if (input_cloud->points[nn_indices[j]].intensity < min_intensity) {
                    min_intensity = input_cloud->points[nn_indices[j]].intensity;
                    best_idx = nn_indices[j];
                }
            }
        }
        downsampled_to_original[i] = best_idx;
    };

    if (switches_.enable_openmp_parallel) {
        #pragma omp parallel for
        for (size_t i = 0; i < result->size(); ++i) {
            map_point(i);
        }
    } else {
        for (size_t i = 0; i < result->size(); ++i) {
            map_point(i);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    SC_INFO("大型点云预采样: %lu -> %lu 点 (耗时: %ld ms, 叶子大小: %.3f)",
             input_cloud->size(), result->size(), duration, leaf_size);

    return result;
}

// =====================================================================
// 高度与水平距离过滤
// ---------------------------------------------------------------------
// 条件：z ∈ [-1.0, height_threshold_]，xy 水平距离 ≤ xy_threshold_
// - 目的：剔除高空噪声和传感器范围外的点，保留 75% 以上视为“不过度过滤”
// - use_conservative_filtering=true 时阈值放宽，并在保留率过低时二次放宽
// - 并行版使用线程局部向量再合并，保证结果与串行一致
// =====================================================================
void Preprocessor::filter_by_height_and_distance(const CloudPtr& input_cloud,
                                                  std::vector<int>& filtered_indices) {
    filtered_indices.clear();

    if (!input_cloud || input_cloud->empty()) return;

    if (!switches_.enable_height_distance_filter) {
        filtered_indices.resize(input_cloud->size());
        for (size_t i = 0; i < input_cloud->size(); ++i) {
            filtered_indices[i] = i;
        }
        return;
    }

    float xy_threshold_factor = switches_.use_conservative_filtering ? 1.05f : 1.0f;
    float height_factor = switches_.use_conservative_filtering ? 0.3f : 0.0f;

    const float xy_threshold_sq = (xy_threshold_ * xy_threshold_factor) * (xy_threshold_ * xy_threshold_factor);
    const float xy_min_sq = xy_min_override_ * xy_min_override_;
    // min_height 原为硬编码 -1.0f（既非 ROS 参数、也从未进入任何消融组，
    // 而实测它单独造成 6.402 pp 的召回硬损失）。启用自标定 ROI 时由 h_s 导出。
    const float min_height = use_height_derived_roi_ ? min_height_override_ : -1.0f;
    const float max_height = height_threshold_ + height_factor;
    // 注：关闭最低环排除时 check_elevation=false，仰角判定被整体跳过。
    // （旧注释称 -1.0f 是"极小值保证全部通过"，实为 -1.0 弧度 = -57.3 度，并非极小值；
    //   该值现已只在 check_elevation=true 时使用，不再有误导性行为。）
    const float min_elevation = lowest_ring_elevation_deg_ * static_cast<float>(kPi) / 180.0f;
    const float sin_min_elevation = std::sin(min_elevation);
    const bool check_elevation = switches_.enable_lowest_ring_exclusion;
    const bool fast_elev = switches_.use_fast_elevation_gate;

    if (switches_.enable_openmp_parallel) {
        std::vector<std::vector<int>> thread_local_indices(omp_get_max_threads());

        #pragma omp parallel
        {
            int thread_id = omp_get_thread_num();
            thread_local_indices[thread_id].reserve(input_cloud->size() / omp_get_max_threads());

            #pragma omp for nowait
            for (size_t i = 0; i < input_cloud->size(); ++i) {
                const auto& pt = input_cloud->points[i];
                // 廉价判据前置短路：z / xy 先过，避免为注定被剔除的点算 sqrt
                const double hd2 = horizontal_distance_sq(pt);
                if (pt.z >= min_height && pt.z <= max_height &&
                    hd2 <= xy_threshold_sq && hd2 >= xy_min_sq &&
                    elevation_pass(pt, check_elevation, min_elevation,
                                   sin_min_elevation, fast_elev)) {
                    thread_local_indices[thread_id].push_back(i);
                }
            }
        }

        for (const auto& indices : thread_local_indices) {
            filtered_indices.insert(filtered_indices.end(), indices.begin(), indices.end());
        }
    } else {
        for (size_t i = 0; i < input_cloud->size(); ++i) {
            const auto& pt = input_cloud->points[i];
            const double hd2s = horizontal_distance_sq(pt);
            if (pt.z >= min_height && pt.z <= max_height &&
                hd2s <= xy_threshold_sq && hd2s >= xy_min_sq &&
                elevation_pass(pt, check_elevation, min_elevation,
                               sin_min_elevation, fast_elev)) {
                filtered_indices.push_back(i);
            }
        }
    }

    // 过滤过于激进的检查
    if (switches_.use_conservative_filtering &&
        filtered_indices.size() < input_cloud->size() * 0.75) {
        SC_WARN("高度和距离过滤过于激进 (%lu -> %lu, %.1f%%)，放宽过滤条件",
                 input_cloud->size(), filtered_indices.size(),
                 100.0 * filtered_indices.size() / input_cloud->size());

        filtered_indices.clear();
        const float relaxed_xy_threshold_sq = (xy_threshold_ * 1.2f) * (xy_threshold_ * 1.2f);
        const float relaxed_min_height = -1.3f;
        const float relaxed_max_height = height_threshold_ + 0.6f;

        for (size_t i = 0; i < input_cloud->size(); ++i) {
            const auto& pt = input_cloud->points[i];
            if (pt.z >= relaxed_min_height && pt.z <= relaxed_max_height &&
                horizontal_distance_sq(pt) <= relaxed_xy_threshold_sq &&
                elevation_pass(pt, check_elevation, min_elevation,
                               sin_min_elevation, fast_elev)) {
                filtered_indices.push_back(i);
            }
        }
    }

    SC_INFO("高度和距离过滤: %lu -> %lu 点 (保留率: %.1f%%)",
             input_cloud->size(), filtered_indices.size(),
             100.0 * filtered_indices.size() / input_cloud->size());
}

// =====================================================================
// 优化统计离群点过滤（半径邻域计数法）
// ---------------------------------------------------------------------
// 规则：半径 ror_radius_ 内邻居数 < min_neighbors 判为离群；
//       低强度点（<5）只需 1 个邻居；强度 <4 的“疑似雪点”一律保留。
// 加速：enable_sampling_optimization 时只采样约 400 个点，再按最近采样点
//       向整块传播标记（空间顺序近似，保守设计）。
// 兜底：若移除比例 >10%，放弃过滤（防止误删雪点）。
// =====================================================================
void Preprocessor::apply_optimized_outlier_removal(const CloudPtr& input_cloud,
                                                   std::vector<int>& indices) {
    if (!switches_.enable_outlier_removal || indices.size() < 1000) {
        return;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    CloudPtr temp_cloud(new pcl::PointCloud<CloudPoint>);
    temp_cloud->resize(indices.size());

    #pragma omp parallel for if (switches_.enable_openmp_parallel)
    for (size_t i = 0; i < indices.size(); ++i) {
        temp_cloud->points[i] = input_cloud->points[indices[i]];
    }

    temp_cloud->width = temp_cloud->size();
    temp_cloud->height = 1;

    pcl::KdTreeFLANN<CloudPoint>::Ptr temp_tree(new pcl::KdTreeFLANN<CloudPoint>);
    temp_tree->setInputCloud(temp_cloud);

    std::vector<bool> is_outlier(indices.size(), false);
    const int min_neighbors = 2;
    const float search_radius = 0.7f;

    int step = switches_.enable_sampling_optimization ?
               std::max(1, static_cast<int>(indices.size() / 400)) : 1;

    #pragma omp parallel for if (switches_.enable_openmp_parallel)
    for (size_t i = 0; i < indices.size(); i += step) {
        const auto& point = temp_cloud->points[i];

        int required_neighbors = min_neighbors;
        if (point.intensity < 5.0f) {
            required_neighbors = 1;
        }

        std::vector<int> nn_indices;
        std::vector<float> nn_dists;
        int found = temp_tree->radiusSearch(point, search_radius, nn_indices, nn_dists);

        if (found < required_neighbors) {
            is_outlier[i] = true;
        }
    }

    // 传播结果
    if (step > 1 && switches_.enable_sampling_optimization) {
        #pragma omp parallel for if (switches_.enable_openmp_parallel)
        for (size_t i = 0; i < indices.size(); ++i) {
            if (i % step != 0) {
                size_t prev = (i / step) * step;
                size_t next = prev + step;
                // 尾部修正：next 必须也是被采样过的下标，否则会继承未计算的 false
                if (next >= indices.size()) {
                    next = (indices.size() - 1) / step * step;
                }

                float dist_prev = (i - prev);
                float dist_next = (next - i);

                if (dist_prev <= dist_next) {
                    is_outlier[i] = is_outlier[prev];
                } else {
                    is_outlier[i] = is_outlier[next];
                }
            }
        }
    }

    std::vector<int> clean_indices;
    clean_indices.reserve(indices.size());

    for (size_t i = 0; i < indices.size(); ++i) {
        if (!is_outlier[i] || temp_cloud->points[i].intensity < 4.0f) {
            clean_indices.push_back(indices[i]);
        }
    }

    if (clean_indices.size() < indices.size() * 0.9) {
        SC_WARN("离群点过滤移除了太多点 (%lu -> %lu, %.1f%%)，使用原始过滤结果",
                 indices.size(), clean_indices.size(),
                 100.0 * clean_indices.size() / indices.size());
        return;
    }

    indices.swap(clean_indices);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    SC_INFO("优化离群点滤波: %lu -> %lu 点 (耗时: %ld ms, 保留率: %.1f%%)",
             temp_cloud->size(), indices.size(), duration,
             100.0 * indices.size() / temp_cloud->size());
}

// =====================================================================
// 由过滤索引构建输出点云 + 双向索引映射
// ---------------------------------------------------------------------
// - processed_to_original: 新点云每个点对应原始点云中的哪个点
// - original_to_processed: 原始点云每个点在处理后点云中的位置（-1 表示被滤除）
// 映射表是后续“检测结果映射回原始索引”和“GT 映射到处理后索引”的基础。
// =====================================================================
void Preprocessor::build_output_cloud(const CloudPtr& input_cloud,
                                      std::vector<int>& filtered_indices,
                                      ProcessedCloud& out) {
    if (filtered_indices.empty() || !input_cloud || input_cloud->empty()) {
        SC_WARN("过滤后索引为空或输入点云为空");
        if (!out.cloud) {
            out.cloud.reset(new pcl::PointCloud<CloudPoint>);
        }
        out.cloud->clear();
        out.mapping.processed_to_original.clear();
        out.mapping.original_to_processed.clear();
        return;
    }

    if (!out.cloud) {
        out.cloud.reset(new pcl::PointCloud<CloudPoint>);
    }

    out.cloud->clear();
    out.cloud->resize(filtered_indices.size());
    out.mapping.processed_to_original.resize(filtered_indices.size());

    #pragma omp parallel for if (switches_.enable_openmp_parallel)
    for (size_t new_idx = 0; new_idx < filtered_indices.size(); ++new_idx) {
        int orig_idx = filtered_indices[new_idx];
        out.cloud->points[new_idx] = input_cloud->points[orig_idx];
        out.mapping.processed_to_original[new_idx] = orig_idx;
    }

    out.cloud->width = out.cloud->size();
    out.cloud->height = 1;
    out.cloud->is_dense = false;

    out.mapping.original_to_processed.assign(input_cloud->size(), -1);

    #pragma omp parallel for if (switches_.enable_openmp_parallel)
    for (size_t i = 0; i < filtered_indices.size(); ++i) {
        int orig_idx = filtered_indices[i];
        out.mapping.original_to_processed[orig_idx] = i;
    }
}

// =====================================================================
// 精确重复点分组（x/y/z/intensity 全等）
// ---------------------------------------------------------------------
// - original_to_canonical: 每个原始索引 -> 该组首个原始索引
// - canonical_group_id: canonical 原始索引 -> 密集分组号
// - group_members_flat / group_start: 分组内全部原始索引（含 canonical 自身）
// 关闭去重时退化为每个点自成一组，保证映射逻辑统一。
// =====================================================================
void Preprocessor::build_dedup_groups(const CloudPtr& cloud,
                                      std::vector<int>& original_to_canonical,
                                      std::vector<int>& canonical_group_id,
                                      std::vector<int>& group_members_flat,
                                      std::vector<int>& group_start) const {
    const size_t n = cloud->size();

    if (!switches_.enable_dedup_exact_points) {
        original_to_canonical.resize(n);
        canonical_group_id.resize(n);
        group_members_flat.resize(n);
        group_start.resize(n + 1);
        for (size_t i = 0; i < n; ++i) {
            original_to_canonical[i] = static_cast<int>(i);
            canonical_group_id[i] = static_cast<int>(i);
            group_members_flat[i] = static_cast<int>(i);
            group_start[i] = static_cast<int>(i);
        }
        group_start[n] = static_cast<int>(n);
        return;
    }

    original_to_canonical.assign(n, -1);
    canonical_group_id.assign(n, -1);

    std::unordered_map<PointKey, int, PointKeyHash> key_to_group;
    key_to_group.reserve(n * 2);
    std::vector<int> group_canonical;
    group_canonical.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        const PointKey key = make_point_key(cloud->points[i]);
        auto it = key_to_group.find(key);
        if (it == key_to_group.end()) {
            const int group = static_cast<int>(group_canonical.size());
            group_canonical.push_back(static_cast<int>(i));
            key_to_group.emplace(key, group);
            canonical_group_id[i] = group;
            original_to_canonical[i] = static_cast<int>(i);
        } else {
            const int group = it->second;
            canonical_group_id[i] = group;
            original_to_canonical[i] = group_canonical[group];
        }
    }

    const int group_count = static_cast<int>(group_canonical.size());
    std::vector<int> group_size(group_count, 0);
    for (size_t i = 0; i < n; ++i) {
        group_size[canonical_group_id[i]]++;
    }
    group_start.resize(group_count + 1);
    group_start[0] = 0;
    for (int g = 0; g < group_count; ++g) {
        group_start[g + 1] = group_start[g] + group_size[g];
    }
    group_members_flat.resize(n);
    std::vector<int> cursor = group_start;
    for (size_t i = 0; i < n; ++i) {
        const int group = canonical_group_id[i];
        group_members_flat[cursor[group]++] = static_cast<int>(i);
    }
}

// =====================================================================
// 统一收尾：把 processed->working 索引转为 canonical 原始索引
// ---------------------------------------------------------------------
// 无论是否去重/降采样，最终 processed_to_original 一律指向 canonical
// 原始索引；original_to_processed 只保留 canonical->processed 映射；
// 重复点展开信息由 build_dedup_groups 填充。
// =====================================================================
void Preprocessor::finalize_index_mapping(ProcessedCloud& out,
                                          const CloudPtr& original_cloud,
                                          const std::vector<int>& working_to_original) const {
    auto& mapping = out.mapping;

    for (size_t i = 0; i < mapping.processed_to_original.size(); ++i) {
        const int working_idx = mapping.processed_to_original[i];
        if (working_idx >= 0 &&
            working_idx < static_cast<int>(working_to_original.size())) {
            mapping.processed_to_original[i] = working_to_original[working_idx];
        }
    }

    mapping.original_to_processed.assign(original_cloud->size(), -1);
    for (size_t i = 0; i < mapping.processed_to_original.size(); ++i) {
        const int canonical = mapping.processed_to_original[i];
        if (canonical >= 0 &&
            canonical < static_cast<int>(original_cloud->size())) {
            mapping.original_to_processed[canonical] = static_cast<int>(i);
        }
    }

    if (switches_.enable_dedup_exact_points) {
        build_dedup_groups(original_cloud,
                           mapping.original_to_canonical,
                           mapping.canonical_group_id,
                           mapping.group_members_flat,
                           mapping.group_start);
    } else {
        // 默认关闭去重：不填充分组表，映射函数直接退化为 canonical 单点
        mapping.original_to_canonical.clear();
        mapping.canonical_group_id.clear();
        mapping.group_members_flat.clear();
        mapping.group_start.clear();
    }
}

// =====================================================================
// 构建统计用点云
// ---------------------------------------------------------------------
// 去重后 detection 点云只剩唯一副本；若直接对它做强度直方图/地面网格
// 百分位统计，重复点数量会影响 count 阈值，导致边界点判定轻微变化。
// 这里把每个 processed 点按分组展开回全部原始副本，得到与不去重等价的
// 统计点云，保证指标逐位一致。
// =====================================================================
void Preprocessor::build_stats_cloud(ProcessedCloud& out,
                                     const CloudPtr& original_cloud) const {
    if (!switches_.enable_dedup_exact_points || !out.cloud || out.cloud->empty()) {
        out.stats_cloud = out.cloud;
        return;
    }

    CloudPtr stats(new pcl::PointCloud<CloudPoint>);
    stats->reserve(out.cloud->size() * 2);
    const auto& mapping = out.mapping;

    for (const int canonical : mapping.processed_to_original) {
        if (canonical < 0 ||
            canonical >= static_cast<int>(mapping.canonical_group_id.size())) {
            continue;
        }
        const int group = mapping.canonical_group_id[canonical];
        if (group < 0 ||
            group + 1 >= static_cast<int>(mapping.group_start.size())) {
            continue;
        }
        const int begin = mapping.group_start[group];
        const int end = mapping.group_start[group + 1];
        for (int k = begin; k < end; ++k) {
            const int idx = mapping.group_members_flat[k];
            if (idx >= 0 && idx < static_cast<int>(original_cloud->size())) {
                stats->push_back(original_cloud->points[idx]);
            }
        }
    }

    if (stats->empty()) {
        out.stats_cloud = out.cloud;
        return;
    }
    stats->width = stats->size();
    stats->height = 1;
    stats->is_dense = false;
    out.stats_cloud = stats;
}

// =====================================================================
// 预处理异常兜底：用最宽松的几何条件重新过滤
// ---------------------------------------------------------------------
// 当主流程抛异常时，保证程序不中断、仍能输出一个可用点云。
// =====================================================================
void Preprocessor::fallback_simple_filter(const CloudPtr& input, ProcessedCloud& out) {
    std::vector<int> simple_indices;
    simple_indices.reserve(input->size() * 0.8);

    const float min_elevation = lowest_ring_elevation_deg_ * static_cast<float>(kPi) / 180.0f;
    const float sin_min_elevation = std::sin(min_elevation);
    const bool check_elevation = switches_.enable_lowest_ring_exclusion;
    const bool fast_elev = switches_.use_fast_elevation_gate;

    for (size_t i = 0; i < input->size(); ++i) {
        const auto& pt = input->points[i];
        const float xy_dist_sq = horizontal_distance_sq(pt);
        if (pt.z >= -1.5 && pt.z <= height_threshold_ + 1.0 &&
            xy_dist_sq <= (xy_threshold_ * 1.3) * (xy_threshold_ * 1.3) &&
            elevation_pass(pt, check_elevation, min_elevation,
                           sin_min_elevation, fast_elev)) {
            simple_indices.push_back(i);
        }
    }

    build_output_cloud(input, simple_indices, out);
}

// =====================================================================
// 预处理主流程（模块入口）
// ---------------------------------------------------------------------
// 顺序：预降采样(可选) -> 高度/距离过滤 -> 离群点过滤 -> 建输出+映射
// 若预降采样生效，最后会把映射表“串联”回原始点云索引。
// =====================================================================
ProcessedCloud Preprocessor::run(const CloudPtr& input_cloud, const CloudFeatures& features) {
    ProcessedCloud out;

    if (!input_cloud || input_cloud->empty()) {
        SC_ERROR("预处理点云时输入点云为空");
        out.cloud.reset(new pcl::PointCloud<CloudPoint>);
        return out;
    }

    CloudPtr working_cloud = input_cloud;
    std::vector<int> working_to_original(input_cloud->size());
    for (size_t i = 0; i < input_cloud->size(); ++i) {
        working_to_original[i] = static_cast<int>(i);
    }

    // 1. 精确重复点合并（指标与不去重逐位一致；实测哈希开销大于检测省时，
    //    默认关闭，仅保留开关供消融复现）
    if (switches_.enable_dedup_exact_points) {
        CloudPtr deduped(new pcl::PointCloud<CloudPoint>);
        deduped->reserve(input_cloud->size());
        std::vector<int> dedup_to_original;
        dedup_to_original.reserve(input_cloud->size());

        std::unordered_map<PointKey, int, PointKeyHash> key_to_canonical;
        key_to_canonical.reserve(input_cloud->size() * 2);
        for (size_t i = 0; i < input_cloud->size(); ++i) {
            const PointKey key = make_point_key(input_cloud->points[i]);
            if (key_to_canonical.find(key) != key_to_canonical.end()) {
                continue;
            }
            key_to_canonical.emplace(key, static_cast<int>(i));
            deduped->push_back(input_cloud->points[i]);
            dedup_to_original.push_back(static_cast<int>(i));
        }
        deduped->width = deduped->size();
        deduped->height = 1;
        deduped->is_dense = false;

        SC_INFO("精确重复点合并: %lu -> %lu 点 (%.1f%%)",
                 input_cloud->size(), deduped->size(),
                 100.0f * deduped->size() / input_cloud->size());
        working_cloud = deduped;
        working_to_original.swap(dedup_to_original);
    }

    // 2. 超大点云预降采样（与去重映射串联）
    // 【已知问题，故意未改】检测阶段还有一道 >10万点的同类门限
    // （cloud_operations.cpp），与这里的 >40万点不一致；统一会改变开关打开时的
    // 行为，故仅在两处互相注明。
    if (switches_.enable_pre_downsampling && working_cloud->size() > 400000) {
        std::vector<int> sampling_to_original;
        CloudPtr downsampled =
            pre_downsample_large_cloud(working_cloud, sampling_to_original, features);
        std::vector<int> composed(downsampled->size());
        for (size_t i = 0; i < downsampled->size(); ++i) {
            composed[i] = working_to_original[sampling_to_original[i]];
        }
        working_cloud = downsampled;
        working_to_original.swap(composed);
    }

    try {
        std::vector<int> filtered_indices;
        filter_by_height_and_distance(working_cloud, filtered_indices);

        if (switches_.enable_outlier_removal && filtered_indices.size() > 80000) {
            apply_optimized_outlier_removal(working_cloud, filtered_indices);
        }

        build_output_cloud(working_cloud, filtered_indices, out);
        finalize_index_mapping(out, input_cloud, working_to_original);
        build_stats_cloud(out, input_cloud);

        SC_INFO("预处理完成: 输入点云 %lu 点，预处理后点云 %lu 点 (保留率: %.1f%%)",
                 input_cloud->size(), out.cloud->size(),
                 100.0f * out.cloud->size() / input_cloud->size());
    } catch (const std::exception& e) {
        SC_ERROR("预处理点云异常: %s", e.what());
        // 修复索引双映射：fallback_simple_filter 直接在**原始** input_cloud 上建索引，
        // 因此 processed_to_original 已经是原始索引；若再经 working_to_original
        // （去重/预降采样映射）过一遍会二次映射导致索引错乱。此处传恒等映射。
        fallback_simple_filter(input_cloud, out);
        std::vector<int> identity(input_cloud->size());
        for (size_t i = 0; i < input_cloud->size(); ++i) identity[i] = static_cast<int>(i);
        finalize_index_mapping(out, input_cloud, identity);
        build_stats_cloud(out, input_cloud);
    }

    return out;
}

}  // namespace snowclear