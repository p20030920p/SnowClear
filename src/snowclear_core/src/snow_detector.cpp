#include "snowclear/snow_detector.hpp"

// =====================================================================
// 本文件可调参数（默认值见 snowclear/system_config.hpp 与 snowclear/ablation_switches.hpp）
// ---------------------------------------------------------------------
// 数值参数（构造注入，来自 SystemConfig）:
//   detector_knn                    5     检测 KNN 近邻数（几何特征开启时）
//   detector_min_intensity_score    0.25  强度得分预筛门槛（低于跳过几何计算）
//   planarity_filter_threshold      0.85  平面度过滤门控
//   direct_planarity_gate           1.0   超低强度直判平面度门控（1.0=关闭）
//   score_threshold                 0.75  融合得分判定阈值（1620帧验证最优）
//   idsor_scale/rho/slope/r0/k/theta        IDSOR 平滑阈值参数
//   intensity/planarity/density/height_weight 特征权重（按开启项归一化）
//   zero_intensity_support_radius         0.6    I=0 表面支撑判定半径（米）
//   zero_intensity_support_min_intensity  1.0    支撑点强度下界
//   zero_intensity_support_range_floor    7.0    近场豁免水平距离（米）
// 消融开关:
//   enable_ultra_low_intensity_direct 关   极低强度直判（IDSOR后不再必需）
//   enable_local_threshold_adjustment 开   局部阈值微调
//   enable_zero_intensity_surface_suppression 开 I=0贴附暗表面抑制（全量+1.50 F1）
//   enable_isolated_point_handling    关   孤立点特殊处理
//   enable_planarity_filtering        关   平面度硬过滤
//   enable_neighbor_intensity_analysis/intensity_comparison 关
//   enable_height_consistency_check   关
//   enable_downsampled_mapping_propagation 关  降采样映射传播
//   use_scattering_term / use_soft_consistency / enable_vertical_zscore 关
// =====================================================================

#include <pcl/common/common.h>

#include <omp.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace snowclear {

namespace {

// I=0 表面抑制用 3D 空间哈希。cell = support_radius；
// 相邻偏移 -1..1 足以覆盖“与查询点距离 <= radius”的全部支撑点。
// 3D 空间哈希键。
// 原实现为 (cx+64)<<16 | (cy+64)<<8 | (cz+64)：每轴仅 8 位，安全条件是
// radius >= xy_threshold/64（xy=17 m 时即 radius >= 0.266 m）。
// radius <= 0.25 m 或 xy_threshold > 38.4 m 会让索引越界/变负，
// 位段互相污染 -> 键碰撞 -> 误判"存在支撑点" -> 误删真雪。
// 改为每轴 21 位有符号打包，量程 ±(2^20)·cell，任何车载雷达参数都不会越界。
// 当前默认参数（cell=0.6, xy=17）下索引范围 35..92，两种打包均无碰撞，故行为不变。
inline int64_t cell_key_21(int64_t cx, int64_t cy, int64_t cz) {
    return ((cx & 0x1FFFFF) << 42) | ((cy & 0x1FFFFF) << 21) | (cz & 0x1FFFFF);
}

inline int64_t support_cell_key(float x, float y, float z, float cell_size) {
    return cell_key_21(static_cast<int64_t>(std::floor(x / cell_size)),
                       static_cast<int64_t>(std::floor(y / cell_size)),
                       static_cast<int64_t>(std::floor(z / cell_size)));
}

}  // namespace

// =====================================================================
// 构造：注入消融开关、特征提取器、KNN 数与强度预筛门槛
// =====================================================================
SnowDetector::SnowDetector(const AblationSwitches& switches,
                           FeatureExtractor& extractor,
                           int knn_k,
                           float min_intensity_score,
                           float planarity_filter_threshold,
                           float direct_planarity_gate,
                           float zero_intensity_support_radius,
                           float zero_intensity_support_min_intensity,
                           float zero_intensity_support_range_floor)
    : switches_(switches),
      extractor_(extractor),
      knn_k_(std::max(3, knn_k)),
      min_intensity_score_(min_intensity_score),
      planarity_filter_threshold_(planarity_filter_threshold),
      direct_planarity_gate_(direct_planarity_gate),
      zero_intensity_support_radius_(zero_intensity_support_radius),
      zero_intensity_support_min_intensity_(zero_intensity_support_min_intensity),
      zero_intensity_support_range_floor_(zero_intensity_support_range_floor) {}

// =====================================================================
// 是否需要 KD 树/KNN
// ---------------------------------------------------------------------
// 最终配置（孤立点处理/平面度/密度/熵/一致性/传播全关）下逐点判定
// 只用“阈值 + 强度得分 + 离地高度”，可完全跳过 KD 树构建与近邻查询。
// =====================================================================
bool SnowDetector::needs_knn() const {
    return switches_.enable_isolated_point_handling ||
           switches_.enable_planarity_calculation ||
           switches_.enable_planarity_filtering ||
           switches_.enable_density_calculation ||
           switches_.enable_neighbor_intensity_analysis ||
           switches_.enable_feature_entropy ||
           switches_.enable_height_consistency ||
           switches_.enable_height_consistency_check ||
           switches_.enable_downsampled_mapping_propagation;
}

// =====================================================================
// 基础检测入口
// ---------------------------------------------------------------------
// 1. 按开关决定块大小（enable_block_processing=false 时整片处理）
// 2. 预分配线程本地特征缓存（enable_feature_cache）
// 3. 确保 KD 树与点云一致
// 4. 并行/串行逐点打分（共用 evaluate_point），合并、排序、去重后输出
// =====================================================================
void SnowDetector::detect(const CloudPtr& cloud,
                          const RangeIntensityThreshold& thresholds,
                          const FilterParameters& feature_weights,
                          pcl::PointIndices::Ptr& snow_indices,
                          int block_size) {
    if (!cloud || cloud->empty() || !snow_indices) {
        return;
    }

    if (!switches_.enable_block_processing) {
        block_size = cloud->size();
    }

    const int num_threads = switches_.enable_openmp_parallel ? omp_get_max_threads() : 1;

    // 线程本地特征缓存：每线程一份，避免并行竞争；串行时只用 [0]。
    // 每次 detect 先清空：避免同一个 SnowDetector 在不同点云上复用旧邻居/特征缓存。
    if (switches_.enable_feature_cache) {
        thread_caches_.clear();
    }
    const bool use_knn = needs_knn();
    if (switches_.enable_feature_cache && use_knn) {
        if (thread_caches_.size() < static_cast<size_t>(num_threads)) {
            thread_caches_.resize(num_threads);
        }
        for (auto& cache : thread_caches_) {
            if (cache.size() < cloud->size()) {
                cache.resize(cloud->size());
            }
        }
    } else {
        thread_caches_.assign(num_threads, {});
    }

    if (use_knn) {
        extractor_.ensure_kdtree(cloud);
    }
    if ((switches_.enable_relative_height || switches_.enable_vertical_zscore) &&
        extractor_.ground_grid_empty()) {
        extractor_.build_ground_grid(cloud);
    }

    if (switches_.enable_zero_intensity_surface_suppression) {
        build_zero_intensity_support_grid(cloud);
    } else {
        support_grid_.clear();
    }

    std::vector<std::vector<int>> thread_local_results(num_threads);

    if (switches_.enable_openmp_parallel) {
        detect_parallel(cloud, thresholds, feature_weights, block_size, thread_local_results);
    } else {
        detect_serial(cloud, thresholds, feature_weights, block_size, thread_local_results[0]);
    }

    snow_indices->indices.clear();
    for (const auto& local_indices : thread_local_results) {
        snow_indices->indices.insert(snow_indices->indices.end(),
                                     local_indices.begin(), local_indices.end());
    }

    std::sort(snow_indices->indices.begin(), snow_indices->indices.end());
    snow_indices->indices.erase(
        std::unique(snow_indices->indices.begin(), snow_indices->indices.end()),
        snow_indices->indices.end());

    SC_INFO("特征融合基础检测: 检测到 %lu 个雪点候选", snow_indices->indices.size());
}

// =====================================================================
// 单点判定核心（串/并行共用同一实现，保证逐位一致）
// ---------------------------------------------------------------------
// 流程：极低强度直判 -> 早停 -> 强度得分与预筛 -> KNN/几何特征
//       -> 融合得分（仅累加已开启特征，权重按开启项归一化）
//       -> 局部阈值调整 -> 最终判定
// =====================================================================
void SnowDetector::evaluate_point(size_t i,
                                  const CloudPtr& cloud,
                                  const RangeIntensityThreshold& thresholds,
                                  const FilterParameters& feature_weights,
                                  std::vector<PointFeature>& cache,
                                  std::vector<int>& out) {
    const auto& point = cloud->points[i];
    const float point_threshold =
        thresholds.threshold_at(point.x, point.y, point.intensity);

    // 极低强度点直接判定为雪点
    if (switches_.enable_ultra_low_intensity_direct &&
        point.intensity < point_threshold * 0.3f) {
        // 直判旁路加平面度门控：贴地高平面度点（如路面）多为误检
        if (switches_.enable_planarity_filtering && direct_planarity_gate_ < 1.0f) {
            std::vector<int> nb;
            std::vector<float> nd;
            extractor_.nearest_k(point, std::min(8, knn_k_), nb, nd);
            if (nb.size() >= 3 &&
                extractor_.planarity(i, nb, cloud, cache) > direct_planarity_gate_) {
                return;
            }
        }
        out.push_back(i);
        return;
    }

    // 早停：强度高于阈值 1.2 倍的点必不可能是雪点，跳过后续计算
    if (point.intensity >= point_threshold * 1.2f) {
        return;
    }

    // 计算强度得分
    double intensity_score = 0.0;
    if (point.intensity < point_threshold) {
        intensity_score = std::pow(1.0 - (point.intensity / point_threshold), 1.2);
    }

    // 零强度点表面抑制：I=0 时强度得分饱和为 1，传统融合判据失去决策权。
    // 若该点紧贴“强度 > 下界”的表面支撑点，则判为暗表面而非雪点；
    // 近场（r <= range_floor）保留原判定，避免把密雪核心区真实雪点误删。
    if (switches_.enable_zero_intensity_surface_suppression &&
        point.intensity <= 0.5f * zero_intensity_support_min_intensity_) {
        const float horizontal_range = std::sqrt(point.x * point.x + point.y * point.y);
        if (horizontal_range > zero_intensity_support_range_floor_ &&
            has_support_point_within(point, cloud)) {
            return;
        }
    }

    // 强度预筛：低于门槛的点不进入 KNN/几何特征计算
    if (intensity_score < min_intensity_score_) {
        return;
    }

    std::vector<int> neighbor_indices;

    if (needs_knn()) {
        // 查找邻居（优先命中缓存）
        if (switches_.enable_feature_cache &&
            i < cache.size() && cache[i].is_computed) {
            neighbor_indices = cache[i].neighbors;
        } else {
            std::vector<float> nn_dists;
            if (switches_.enable_kdtree_cache) {
                extractor_.nearest_k(point, knn_k_, neighbor_indices, nn_dists);
            } else {
                neighbor_indices.reserve(knn_k_);
                for (size_t j = 0; j < cloud->size(); ++j) {
                    if (j != i) {
                        const auto& other_pt = cloud->points[j];
                        float dist_sq = (point.x - other_pt.x) * (point.x - other_pt.x) +
                                        (point.y - other_pt.y) * (point.y - other_pt.y) +
                                        (point.z - other_pt.z) * (point.z - other_pt.z);
                        if (dist_sq < 1.0f &&
                            neighbor_indices.size() < static_cast<size_t>(knn_k_)) {
                            neighbor_indices.push_back(j);
                        }
                    }
                }
            }

            // 【已删】原实现在此写 cache[i].neighbors，但读取条件是 cache[i].is_computed，
            // 而 is_computed 只在 enable_planarity_cache 分支置位 -> 该缓存永不命中；
            // 且同一帧内每个点只被 evaluate_point 访问一次，按构造就无复用价值。
            // 保留它等于为每个被评估点白做一次 vector<int> 分配+拷贝。
        }
    }

    // 孤立点特殊处理
    if (switches_.enable_isolated_point_handling && neighbor_indices.size() < 5) {
        if (intensity_score > 0.7) {
            out.push_back(i);
        }
        return;
    }

    // 几何特征计算
    double planarity = 0.0;
    double scattering = 0.0;
    double entropy = 1.0;

    if (switches_.enable_planarity_calculation) {
        if (switches_.enable_planarity_cache &&
            i < cache.size() && cache[i].is_computed) {
            planarity = cache[i].planarity;
            scattering = cache[i].scattering;
        } else {
            planarity = extractor_.planarity(i, neighbor_indices, cloud, cache,
                                             &scattering);

            if (switches_.enable_planarity_cache && i < cache.size()) {
                cache[i].planarity = planarity;
                // 修复：原实现只写 planarity 与 is_computed，不写 scattering，
                // 于是缓存命中时 scattering 静默读到 0，污染 use_scattering_term 消融。
                cache[i].scattering = scattering;
                cache[i].is_computed = true;
            }
        }
        // 修复：原实现把 entropy 计算嵌在上面的"缓存未命中"分支内，且整段嵌在
        // enable_planarity_calculation 之内。于是单独开 enable_feature_entropy 时
        // entropy 恒为默认值 1.0，combined_score 恒加常数 0.12 偏置，
        // 使 G1 的"特征熵"消融结论不可信。这里改为独立计算。
        if (switches_.enable_feature_entropy) {
            entropy = extractor_.feature_entropy(cloud, neighbor_indices);
        }
    }

    // 平面度过滤 - 雪点不应该在平面上
    if (switches_.enable_planarity_filtering &&
        planarity > planarity_filter_threshold_) {
        return;
    }

    // 相对高度得分
    double height_score = 0.0;
    if (switches_.enable_relative_height) {
        height_score = extractor_.height_above_ground(point) * 0.5;
    }

    // 密度得分
    double density_score = 0.0;
    if (switches_.enable_density_calculation && !neighbor_indices.empty()) {
        std::vector<float> dists;
        for (size_t j = 0; j < std::min(size_t(5), neighbor_indices.size()); ++j) {
            const auto& neighbor = cloud->points[neighbor_indices[j]];
            float dist = std::sqrt(
                (point.x - neighbor.x) * (point.x - neighbor.x) +
                (point.y - neighbor.y) * (point.y - neighbor.y) +
                (point.z - neighbor.z) * (point.z - neighbor.z)
            );
            dists.push_back(dist);
        }

        float avg_distance = std::accumulate(dists.begin(), dists.end(), 0.0f) / dists.size();
        density_score = std::min(1.0, 0.075 / (avg_distance + 0.001));

        // 邻居强度分析
        if (switches_.enable_neighbor_intensity_analysis) {
            float neighbor_intensity_sum = 0.0f;
            float neighbor_min_intensity = std::numeric_limits<float>::max();
            int lower_intensity_neighbors = 0;

            for (size_t j = 0; j < std::min(size_t(8), neighbor_indices.size()); ++j) {
                float intensity = cloud->points[neighbor_indices[j]].intensity;
                neighbor_intensity_sum += intensity;
                neighbor_min_intensity = std::min(neighbor_min_intensity, intensity);

                if (intensity < point.intensity) {
                    lower_intensity_neighbors++;
                }
            }

            float avg_neighbor_intensity = neighbor_intensity_sum /
                std::min(size_t(8), neighbor_indices.size());

            if (switches_.enable_intensity_comparison &&
                point.intensity < avg_neighbor_intensity * 0.7) {
                density_score = std::min(1.0, density_score + 0.2);
            }

            if (switches_.enable_intensity_comparison &&
                point.intensity <= neighbor_min_intensity * 1.05f) {
                density_score = std::min(1.0, density_score + 0.15);
            }

            if (switches_.enable_intensity_comparison && lower_intensity_neighbors <= 2) {
                density_score = std::min(1.0, density_score + 0.1);
            }
        }
    }

    // 高度一致性
    double height_consistency = 0.0;
    if (switches_.enable_height_consistency && neighbor_indices.size() >= 5) {
        int consistent_neighbors = 0;
        for (size_t j = 0; j < std::min(size_t(8), neighbor_indices.size()); ++j) {
            const auto& neighbor = cloud->points[neighbor_indices[j]];
            if (std::abs(neighbor.z - point.z) < 0.1f) {
                consistent_neighbors++;
            }
        }
        height_consistency = static_cast<double>(consistent_neighbors) /
            std::min(size_t(8), neighbor_indices.size());
    }

    // 熵因子
    double entropy_factor = switches_.enable_feature_entropy ? entropy * 0.12 : 0.0;

    // 特征融合得分：只累加已开启特征，权重按开启项归一化
    double weight_sum = feature_weights.intensity_weight;
    if (switches_.enable_planarity_calculation) weight_sum += feature_weights.planarity_weight;
    if (switches_.enable_density_calculation) weight_sum += feature_weights.density_weight;
    if (switches_.enable_relative_height) weight_sum += feature_weights.height_weight;
    if (weight_sum <= 0) weight_sum = 1.0;

    double combined_score =
        (feature_weights.intensity_weight / weight_sum) * intensity_score;
    if (switches_.enable_planarity_calculation) {
        const double geometry_term =
            switches_.use_scattering_term ? scattering : (1.0 - planarity);
        combined_score += (feature_weights.planarity_weight / weight_sum) * geometry_term;
    }
    if (switches_.enable_density_calculation) {
        combined_score += (feature_weights.density_weight / weight_sum) * density_score;
    }
    if (switches_.enable_relative_height) {
        combined_score += (feature_weights.height_weight / weight_sum) * height_score;
    }
    if (switches_.enable_feature_entropy) {
        combined_score += entropy_factor;
    }
    if (switches_.enable_height_consistency) {
        combined_score += 0.08 * height_consistency;
    }
    if (switches_.use_soft_consistency) {
        // 软项：高度无序(1-consistency)≈雪团簇，加分；不再使用硬门控
        combined_score += 0.06 * (1.0 - height_consistency);
    }
    // FlakeOut 式垂直 z-score：悬浮雪点加分（0~3 归一化）
    if (switches_.enable_vertical_zscore) {
        combined_score += 0.08 *
            std::max(0.0f, std::min(3.0f, extractor_.vertical_zscore(point))) / 3.0;
    }

    // 局部阈值调整
    double local_threshold = feature_weights.score_threshold;

    if (switches_.enable_local_threshold_adjustment) {
        if (intensity_score < 0.4) {
            local_threshold *= 1.2f;
        } else if (intensity_score > 0.7) {
            local_threshold *= 0.9f;
        }

        if (planarity > 0.45) {
            local_threshold *= 1.15f;
        }
    }

    // 最终判定
    bool passes_basic_test = combined_score > local_threshold && intensity_score > 0.3;
    bool passes_consistency_test =
        switches_.use_soft_consistency ||
        !switches_.enable_height_consistency_check ||
        height_consistency > 0.4;

    if (passes_basic_test && passes_consistency_test) {
        out.push_back(i);
    }
}

// =====================================================================
// 零强度点表面抑制：支撑点空间哈希
// ---------------------------------------------------------------------
// 支撑点 = intensity > zero_intensity_support_min_intensity_。
// 以 support_radius 为 cell size 建立 3D 哈希；检测阶段哈希只读，
// 因此可安全地在 OpenMP 并行块内查询。相邻 cell 偏移 -1..1 覆盖全部
// 距离 <= radius 的支撑点。
// =====================================================================
void SnowDetector::build_zero_intensity_support_grid(const CloudPtr& cloud) {
    support_grid_.clear();
    if (!cloud || cloud->empty() || zero_intensity_support_radius_ <= 0.0f) {
        return;
    }

    support_grid_.reserve(cloud->size() / 8);
    all_point_grid_.clear();
    // 稀疏传感器扩半径需要"邻域总点数"，故额外建一份全点网格。
    // cell 用扩半径上限，保证一次邻格遍历即可覆盖放大后的搜索球。
    const float all_cell = sparse_expand_ ? sparse_max_radius_
                                          : zero_intensity_support_radius_;
    if (sparse_expand_) {
        all_point_grid_.reserve(cloud->size() / 4);
    }
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& pt = cloud->points[i];
        if (pt.intensity > zero_intensity_support_min_intensity_) {
            const int64_t key = support_cell_key(pt.x, pt.y, pt.z,
                                                 zero_intensity_support_radius_);
            support_grid_[key].push_back(static_cast<int>(i));
        }
        if (sparse_expand_) {
            all_point_grid_[support_cell_key(pt.x, pt.y, pt.z, all_cell)]
                .push_back(static_cast<int>(i));
        }
    }

    SC_INFO("零强度表面抑制: 半径=%.2fm, 强度下界=%.1f, 近场豁免=%.1fm, %lu 个空间单元",
             zero_intensity_support_radius_, zero_intensity_support_min_intensity_,
             zero_intensity_support_range_floor_, support_grid_.size());
}

namespace {

// 在给定哈希网格上做半径查询：返回落在 radius 内的点数（可提前截断）
inline int count_within(const std::unordered_map<int64_t, std::vector<int>>& grid,
                        float cell, const CloudPoint& point, const CloudPtr& cloud,
                        float radius, int stop_at) {
    const float r2 = radius * radius;
    const int64_t cx = static_cast<int64_t>(std::floor(point.x / cell));
    const int64_t cy = static_cast<int64_t>(std::floor(point.y / cell));
    const int64_t cz = static_cast<int64_t>(std::floor(point.z / cell));
    // 邻格跨度必须覆盖搜索半径：span = ceil(radius / cell)。
    // 只用 ±1 在 radius > cell 时会漏点（例如在 cell=0.6 的网格上查 1.5 m）。
    const int span = std::max(1, static_cast<int>(std::ceil(radius / cell)));
    int found = 0;
    for (int dx = -span; dx <= span; ++dx) {
        for (int dy = -span; dy <= span; ++dy) {
            for (int dz = -span; dz <= span; ++dz) {
                const auto it = grid.find(cell_key_21(cx + dx, cy + dy, cz + dz));
                if (it == grid.end()) continue;
                for (int idx : it->second) {
                    if (idx < 0 || idx >= static_cast<int>(cloud->size())) continue;
                    const auto& o = cloud->points[idx];
                    const float ax = point.x - o.x;
                    const float ay = point.y - o.y;
                    const float az = point.z - o.z;
                    if (ax * ax + ay * ay + az * az <= r2) {
                        if (++found >= stop_at) return found;
                    }
                }
            }
        }
    }
    return found;
}

}  // namespace

bool SnowDetector::has_support_point_within(const CloudPoint& point,
                                            const CloudPtr& cloud) const {
    if (support_grid_.empty() || !cloud) {
        return false;
    }

    const float radius = zero_intensity_support_radius_;
    if (count_within(support_grid_, radius, point, cloud, radius, 1) > 0) {
        return true;
    }

    // 稀疏传感器补救：没找到亮邻点，可能是"真的孤立"（雪），也可能只是
    // "采样不足"（该体积里几乎没点，结论不可靠）。实测 CADC(VLP-32C) 6-10m 处
    // 0.6m 球内仅 68 点（WADS 436、Boreas 713），只有 42.79% 的 I=0 点邻点数>=30。
    // 因此当邻域总点数不足时，放大半径重查以凑够样本再下结论。
    // 注意：在密集传感器上这不是免费的（WADS 实测触发 29.9%、macro F1 -0.14 pp），
    // 故默认关闭，仅稀疏平台启用。
    if (sparse_expand_ && !all_point_grid_.empty()) {
        const int total = count_within(all_point_grid_, sparse_max_radius_, point, cloud,
                                       radius, sparse_min_neighbors_);
        if (total < sparse_min_neighbors_) {
            return count_within(support_grid_, radius, point, cloud,
                                sparse_max_radius_, 1) > 0;
        }
    }
    return false;
}

// =====================================================================
// 并行逐点检测（OpenMP）
// ---------------------------------------------------------------------
// 每线程独立结果向量与特征缓存，块级动态调度；
// 每个点 i 只由单个线程处理，逐点判定逻辑见 evaluate_point。
// =====================================================================
void SnowDetector::detect_parallel(const CloudPtr& cloud,
                                   const RangeIntensityThreshold& thresholds,
                                   const FilterParameters& feature_weights,
                                   int block_size,
                                   std::vector<std::vector<int>>& thread_local_results) {
    thread_local_results.resize(omp_get_max_threads());

    #pragma omp parallel
    {
        int thread_id = omp_get_thread_num();
        std::vector<int>& local_results = thread_local_results[thread_id];
        local_results.reserve(cloud->size() / omp_get_num_threads() / 5);
        auto& cache = thread_caches_[thread_id];

        #pragma omp for schedule(dynamic)
        for (size_t block_start = 0; block_start < cloud->size(); block_start += block_size) {
            size_t block_end = std::min(block_start + block_size, cloud->size());

            for (size_t i = block_start; i < block_end; ++i) {
                evaluate_point(i, cloud, thresholds, feature_weights, cache, local_results);
            }
        }
    }
}

// =====================================================================
// 串行逐点检测
// ---------------------------------------------------------------------
// 与并行版逐点逻辑一致（共用 evaluate_point，使用 thread_caches_[0]）。
// =====================================================================
void SnowDetector::detect_serial(const CloudPtr& cloud,
                                 const RangeIntensityThreshold& thresholds,
                                 const FilterParameters& feature_weights,
                                 int block_size,
                                 std::vector<int>& local_results) {
    local_results.clear();
    local_results.reserve(cloud->size() / 5);
    auto& cache = thread_caches_[0];

    for (size_t block_start = 0; block_start < cloud->size(); block_start += block_size) {
        size_t block_end = std::min(block_start + block_size, cloud->size());

        for (size_t i = block_start; i < block_end; ++i) {
            evaluate_point(i, cloud, thresholds, feature_weights, cache, local_results);
        }
    }
}

// =====================================================================
// 降采样结果映射回原始点云 + 局部传播
// ---------------------------------------------------------------------
// 1. 通过 downsampled_to_original 表把检测索引映射到原始点云
// 2. 以映射出的雪点为种子，在原始点云中做小半径邻域搜索：
//    邻居满足“低强度 + 高度接近 + 距离更近”才并入（保守传播）
// 3. 防止传播爆炸：额外点不得超过种子数的 30%，按强度低优先保留
// 阈值支持距离分段（每个种子点取自身所在段的阈值）。
// =====================================================================
void SnowDetector::map_and_propagate(const CloudPtr& input_cloud,
                                     pcl::PointIndices::Ptr& snow_indices,
                                     const std::vector<int>& downsampled_to_original,
                                     const CloudFeatures& features,
                                     const RangeIntensityThreshold& thresholds) {
    if (!switches_.enable_downsampled_mapping_propagation) {
        pcl::PointIndices::Ptr mapped_indices(new pcl::PointIndices);
        mapped_indices->indices.reserve(snow_indices->indices.size());

        for (int idx : snow_indices->indices) {
            if (idx >= 0 && idx < static_cast<int>(downsampled_to_original.size())) {
                int original_idx = downsampled_to_original[idx];
                if (original_idx >= 0 && original_idx < static_cast<int>(input_cloud->size())) {
                    mapped_indices->indices.push_back(original_idx);
                }
            }
        }

        snow_indices = mapped_indices;
        return;
    }

    pcl::PointIndices::Ptr mapped_indices(new pcl::PointIndices);
    mapped_indices->indices.reserve(snow_indices->indices.size() * 2);

    std::unordered_set<int> mapped_set;
    mapped_set.reserve(snow_indices->indices.size() * 2);

    for (int idx : snow_indices->indices) {
        if (idx >= 0 && idx < static_cast<int>(downsampled_to_original.size())) {
            int original_idx = downsampled_to_original[idx];
            if (original_idx >= 0 && original_idx < static_cast<int>(input_cloud->size())) {
                mapped_indices->indices.push_back(original_idx);
                mapped_set.insert(original_idx);
            }
        }
    }

    extractor_.ensure_kdtree(input_cloud);

    float search_radius = 0.06f;
    if (features.point_density < 100) {
        search_radius = 0.08f;
    } else if (features.point_density > 500) {
        search_radius = 0.05f;
    }

    std::vector<int> additional_indices;
    additional_indices.reserve(mapped_indices->indices.size());

    if (switches_.enable_openmp_parallel) {
        #pragma omp parallel
        {
            std::vector<int> local_additional;
            local_additional.reserve(mapped_indices->indices.size() / omp_get_num_threads());

            #pragma omp for schedule(dynamic, 20)
            for (size_t i = 0; i < mapped_indices->indices.size(); ++i) {
                int snow_idx = mapped_indices->indices[i];
                const auto& snow_pt = input_cloud->points[snow_idx];
                const float intensity_threshold =
                    thresholds.threshold_at(snow_pt.x, snow_pt.y, snow_pt.intensity) * 1.1f;

                if (snow_pt.intensity > intensity_threshold * 0.7f) continue;

                std::vector<int> nn_indices;
                std::vector<float> nn_dists;
                extractor_.radius_search(snow_pt, search_radius, nn_indices, nn_dists);

                for (size_t j = 0; j < nn_indices.size(); ++j) {
                    int nn_idx = nn_indices[j];

                    const auto& nn_pt = input_cloud->points[nn_idx];

                    if (nn_pt.intensity < intensity_threshold * 0.9f &&
                        std::abs(nn_pt.z - snow_pt.z) < 0.1f &&
                        nn_dists[j] < search_radius * 0.7f) {
                        // find+insert 必须在同一临界区内完成：
                        // 并发读 unordered_set 同时有线程插入是数据竞争（未定义行为），
                        // 且会造成同一邻居被多个种子重复加入。
                        bool claimed = false;
                        #pragma omp critical
                        {
                            if (mapped_set.find(nn_idx) == mapped_set.end()) {
                                mapped_set.insert(nn_idx);
                                claimed = true;
                            }
                        }
                        if (claimed) {
                            local_additional.push_back(nn_idx);
                        }
                    }
                }
            }

            #pragma omp critical
            {
                additional_indices.insert(additional_indices.end(),
                                          local_additional.begin(),
                                          local_additional.end());
            }
        }
    } else {
        for (size_t i = 0; i < mapped_indices->indices.size(); ++i) {
            int snow_idx = mapped_indices->indices[i];
            const auto& snow_pt = input_cloud->points[snow_idx];
            const float intensity_threshold =
                thresholds.threshold_at(snow_pt.x, snow_pt.y, snow_pt.intensity) * 1.1f;

            if (snow_pt.intensity > intensity_threshold * 0.7f) continue;

            std::vector<int> nn_indices;
            std::vector<float> nn_dists;
            extractor_.radius_search(snow_pt, search_radius, nn_indices, nn_dists);

            for (size_t j = 0; j < nn_indices.size(); ++j) {
                int nn_idx = nn_indices[j];

                if (mapped_set.find(nn_idx) != mapped_set.end()) continue;

                const auto& nn_pt = input_cloud->points[nn_idx];

                if (nn_pt.intensity < intensity_threshold * 0.9f &&
                    std::abs(nn_pt.z - snow_pt.z) < 0.1f &&
                    nn_dists[j] < search_radius * 0.7f) {
                    additional_indices.push_back(nn_idx);
                    mapped_set.insert(nn_idx);
                }
            }
        }
    }

    if (additional_indices.size() > mapped_indices->indices.size() * 0.3f) {
        SC_WARN("映射传播增长过快，限制增长");

        std::sort(additional_indices.begin(), additional_indices.end(),
                  [&input_cloud](int a, int b) {
                      return input_cloud->points[a].intensity < input_cloud->points[b].intensity;
                  });

        additional_indices.resize(mapped_indices->indices.size() * 0.3f);
    }

    mapped_indices->indices.insert(mapped_indices->indices.end(),
                                   additional_indices.begin(),
                                   additional_indices.end());

    std::sort(mapped_indices->indices.begin(), mapped_indices->indices.end());
    mapped_indices->indices.erase(
        std::unique(mapped_indices->indices.begin(), mapped_indices->indices.end()),
        mapped_indices->indices.end());

    size_t original_size = snow_indices->indices.size();
    snow_indices = mapped_indices;

    SC_INFO("映射并传播: %lu -> %lu 个雪点 (映射增幅: %.1f%%)",
             original_size, snow_indices->indices.size(),
             original_size > 0 ?
                 (snow_indices->indices.size() - original_size) * 100.0 / original_size : 0.0);
}

}  // namespace snowclear