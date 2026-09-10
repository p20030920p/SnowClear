#include "snowclear/parameter_optimizer.hpp"
#include "snowclear/snow_detector.hpp"

// =====================================================================
// 本文件可调参数（默认值见 snowclear/system_config.hpp 与 snowclear/ablation_switches.hpp）
// ---------------------------------------------------------------------
//   use_adaptive_intensity_threshold   开   自适应强度阈值（Q1×0.8±修正，钳制[2.5,8.0]）
//   rgor_intensity_threshold           6.0  固定强度阈值（自适应关闭时使用）
//   use_range_binned_intensity_threshold 关 距离分段强度阈值（实测降精度）
//   use_idsor_intensity_threshold      开   IDSOR 平滑距离-强度阈值
//   use_otsu_intensity_threshold       关   Otsu 直方图双峰阈值（实测劣化）
//   enable_grid_search_optimization    关   前3帧网格搜索（无额外收益）
//   enable_feature_based_recommendation 关  场景特征参数推荐
//   enable_adaptive_parameter_adjustment 关 场景自适应权重调整
//   enable_intensity_distribution_adaptation 关 final-2 关闭：Q1 基阈值 + IDSOR 已足够
//   max_optimization_iterations / max_grid_search_points  网格搜索预算
// =====================================================================

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <omp.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace snowclear {

ParameterOptimizer::ParameterOptimizer(const AblationSwitches& switches,
                                       SnowDetector& detector,
                                       const FilterParameters& defaults)
    : switches_(switches),
      detector_(detector),
      defaults_(defaults) {}

// =====================================================================
// 自适应强度阈值
// ---------------------------------------------------------------------
// 基础 = Q1 × 0.8，再按场景复杂度/强度范围/标准差做 ±调整，最终钳制在
// [2.5, 8.0]。阈值越低，判定越严格（只留极低强度点）。
// 受 use_adaptive_intensity_threshold 控制；关闭时返回固定阈值。
// =====================================================================
float ParameterOptimizer::adaptive_intensity_threshold(const CloudFeatures& features) const {
    if (!switches_.use_adaptive_intensity_threshold) {
        return defaults_.rgor_intensity_threshold;
    }

    float base_threshold = features.intensity_q1 * 0.8f;

    if (switches_.enable_complexity_adaptation) {
        if (features.scene_complexity > 0.7) {
            base_threshold *= 0.85f;
        } else if (features.scene_complexity > 0.4) {
            base_threshold *= 0.9f;
        } else if (features.scene_complexity < 0.3) {
            base_threshold *= 1.0f;
        }
    }

    float range_adjustment = 0.0f;
    if (switches_.enable_intensity_distribution_adaptation) {
        if (features.intensity_range > 80) {
            range_adjustment = 0.5f;
        } else if (features.intensity_range < 20) {
            range_adjustment = -0.2f;
        }
    }

    float stddev_adjustment = 0.0f;
    if (switches_.enable_intensity_distribution_adaptation) {
        if (features.intensity_stddev > 12.0) {
            stddev_adjustment = 0.5f;
        } else if (features.intensity_stddev < 2.5) {
            stddev_adjustment = -0.3f;
        }
    }

    float threshold = base_threshold + range_adjustment + stddev_adjustment;

    return std::max(std::min(threshold, 8.0f), 2.5f);
}

// =====================================================================
// 距离分段强度阈值
// ---------------------------------------------------------------------
// 按水平距离分段（0-5/5-10/10-15/15-18m）分别统计强度分布，
// 用与全局自适应公式相同的规则计算每段阈值；段内点数过少时回退全局阈值。
// =====================================================================
RangeIntensityThreshold ParameterOptimizer::compute_range_thresholds(
    const CloudPtr& cloud,
    const CloudFeatures& features) const {
    RangeIntensityThreshold result;
    const size_t nbins = result.bin_edges.size() - 1;
    result.thresholds.resize(nbins, result.global_threshold);

    if (!cloud || cloud->empty()) {
        return result;
    }

    std::vector<int> counts(nbins, 0);
    std::vector<double> sums(nbins, 0.0), sumsq(nbins, 0.0);
    std::vector<float> mins(nbins, 255.0f), maxs(nbins, 0.0f);
    std::vector<std::array<int, 256>> hists(nbins);
    for (auto& h : hists) h.fill(0);

    for (const auto& pt : cloud->points) {
        const float range = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        int bin = -1;
        for (size_t i = 0; i + 1 < result.bin_edges.size(); ++i) {
            if (range >= result.bin_edges[i] && range < result.bin_edges[i + 1]) {
                bin = static_cast<int>(i);
                break;
            }
        }
        if (bin < 0) continue;

        counts[bin]++;
        sums[bin] += pt.intensity;
        sumsq[bin] += static_cast<double>(pt.intensity) * pt.intensity;
        mins[bin] = std::min(mins[bin], pt.intensity);
        maxs[bin] = std::max(maxs[bin], pt.intensity);
        hists[bin][std::max(0, std::min(255, static_cast<int>(pt.intensity)))]++;
    }

    const float global = adaptive_intensity_threshold(features);
    result.global_threshold = global;

    const int min_points_per_bin = 500;
    for (size_t b = 0; b < nbins; ++b) {
        if (counts[b] < min_points_per_bin) {
            result.thresholds[b] = global;
            continue;
        }

        // Q1：利用段内 256 级直方图 CDF
        int cumulative = 0;
        float q1 = 0.0f;
        const int q1_target = counts[b] / 4;
        for (int i = 0; i < 256; ++i) {
            cumulative += hists[b][i];
            if (q1 == 0.0f && cumulative >= q1_target) {
                q1 = static_cast<float>(i);
                break;
            }
        }

        const float mean = static_cast<float>(sums[b] / counts[b]);
        double variance = sumsq[b] / counts[b] - static_cast<double>(mean) * mean;
        if (variance < 0.0) variance = 0.0;
        const float stddev = static_cast<float>(std::sqrt(variance));
        const float range = maxs[b] - mins[b];

        float base_threshold = q1 * 0.8f;
        if (switches_.enable_complexity_adaptation) {
            if (features.scene_complexity > 0.7) {
                base_threshold *= 0.85f;
            } else if (features.scene_complexity > 0.4) {
                base_threshold *= 0.9f;
            } else if (features.scene_complexity < 0.3) {
                base_threshold *= 1.0f;
            }
        }

        float range_adjustment = 0.0f;
        float stddev_adjustment = 0.0f;
        if (switches_.enable_intensity_distribution_adaptation) {
            if (range > 80) {
                range_adjustment = 0.5f;
            } else if (range < 20) {
                range_adjustment = -0.2f;
            }
            if (stddev > 12.0) {
                stddev_adjustment = 0.5f;
            } else if (stddev < 2.5) {
                stddev_adjustment = -0.3f;
            }
        }

        float threshold = base_threshold + range_adjustment + stddev_adjustment;
        result.thresholds[b] = std::max(std::min(threshold, 8.0f), 2.5f);
    }

    SC_INFO("距离分段强度阈值: [%.2f, %.2f, %.2f, %.2f], 全局回退: %.2f",
             result.thresholds[0], result.thresholds[1],
             result.thresholds[2], result.thresholds[3], global);

    return result;
}

// =====================================================================
// Otsu 强度阈值
// ---------------------------------------------------------------------
// 对 256 级强度直方图最大化类间方差，自动寻找“雪(低强度峰)/表面(高强度峰)”
// 的分割点。返回 0~255 的阈值（强度小于阈值视为雪候选）。
// =====================================================================
float ParameterOptimizer::compute_otsu_threshold(const CloudPtr& cloud) const {
    if (!cloud || cloud->empty()) {
        return 2.5f;
    }
    std::array<int, 256> hist{};
    for (const auto& pt : cloud->points) {
        hist[std::max(0, std::min(255, static_cast<int>(pt.intensity)))]++;
    }
    const size_t total = cloud->size();
    double sum = 0.0;
    for (int b = 0; b < 256; ++b) {
        sum += static_cast<double>(b) * hist[b];
    }
    double sum_b = 0.0;
    int w_b = 0;
    double best = -1.0;
    float threshold = 2.5f;
    for (int b = 0; b < 256; ++b) {
        w_b += hist[b];
        if (w_b == 0) continue;
        const int w_f = static_cast<int>(total) - w_b;
        if (w_f == 0) break;
        sum_b += static_cast<double>(b) * hist[b];
        const double m_b = sum_b / w_b;
        const double m_f = (sum - sum_b) / w_f;
        const double between = static_cast<double>(w_b) * w_f *
                               (m_b - m_f) * (m_b - m_f);
        if (between > best) {
            best = between;
            threshold = static_cast<float>(b);
        }
    }
    return std::max(2.0f, std::min(20.0f, threshold));
}

// =====================================================================
// 动态块大小
// ---------------------------------------------------------------------
// 点越多块越大（但不超过默认块大小），点很少时用小块，兼顾缓存与负载均衡。
// =====================================================================
int ParameterOptimizer::dynamic_block_size(size_t point_count, int default_block) const {
    if (!switches_.use_dynamic_block_size) {
        return default_block;
    }

    int dynamic_block_size = default_block;

    if (point_count > 100000) {
        dynamic_block_size = std::min(default_block, static_cast<int>(point_count / 24));
        dynamic_block_size = std::max(768, dynamic_block_size);
    } else if (point_count > 50000) {
        dynamic_block_size = std::min(default_block, static_cast<int>(point_count / 20));
        dynamic_block_size = std::max(512, dynamic_block_size);
    } else if (point_count < 10000) {
        dynamic_block_size = std::max(256, static_cast<int>(point_count / 8));
    }

    return dynamic_block_size;
}

// =====================================================================
// 场景自适应权重调整
// ---------------------------------------------------------------------
// 依据强度标准差、地面比例、密度、场景复杂度缩放四个特征权重并重新归一化。
// 受 enable_adaptive_weight_adjustment 控制。
// =====================================================================
void ParameterOptimizer::adjust_feature_weights_for_scene(FilterParameters& params,
                                                          const CloudFeatures& features) const {
    if (!switches_.enable_adaptive_weight_adjustment) {
        return;
    }

    if (features.intensity_stddev > 10.0) {
        params.intensity_weight *= 0.85;
        params.planarity_weight *= 1.15;
    } else if (features.intensity_stddev > 6.0) {
        params.intensity_weight *= 0.9;
        params.planarity_weight *= 1.1;
    }

    if (features.ground_ratio > 0.3) {
        params.height_weight *= 1.25;
    } else if (features.ground_ratio > 0.2) {
        params.height_weight *= 1.15;
    }

    if (switches_.enable_density_adaptation) {
        if (features.point_density < 40) {
            params.density_weight *= 1.2;
        } else if (features.point_density < 80) {
            params.density_weight *= 1.1;
        }
    }

    if (switches_.enable_complexity_adaptation) {
        if (features.scene_complexity > 0.7) {
            params.planarity_weight *= 1.15;
            params.density_weight *= 1.1;
            params.intensity_weight *= 0.85;
        } else if (features.scene_complexity < 0.3) {
            params.intensity_weight *= 1.1;
            params.planarity_weight *= 0.9;
        }
    }

    double sum = params.intensity_weight + params.planarity_weight +
                 params.density_weight + params.height_weight;

    if (sum > 0) {
        params.intensity_weight /= sum;
        params.planarity_weight /= sum;
        params.density_weight /= sum;
        params.height_weight /= sum;
    }
}

// =====================================================================
// 基于场景特征的参数推荐（无 GT 时的参数来源）
// ---------------------------------------------------------------------
// - DCOR 距离因子/最小点数：按点密度选择
// - RGOR 强度阈值：按 Q1 比例 + 标准差自适应，钳制 [3.5, 10]
// - 聚类 epsilon/min_points：按复杂度与噪声比
// - 特征权重：按复杂度三档预设，高方差场景加重平面度
// =====================================================================
FilterParameters ParameterOptimizer::recommend(const CloudFeatures& features) const {
    // 开关全关（当前最终配置）时直接返回默认参数，避免无用的场景计算与日志
    if (!switches_.enable_feature_based_recommendation) {
        return defaults_;
    }

    FilterParameters params;
    params.score_threshold = defaults_.score_threshold;
    params.idsor_scale = defaults_.idsor_scale;
    params.idsor_rho = defaults_.idsor_rho;
    params.idsor_slope = defaults_.idsor_slope;
    params.idsor_r0 = defaults_.idsor_r0;
    params.idsor_k = defaults_.idsor_k;
    params.idsor_theta = defaults_.idsor_theta;

    if (switches_.enable_density_adaptation) {
        if (features.point_density < 100) {
            params.dcor_distance_factor = 0.022;
            params.dcor_min_pts = 5;
        } else if (features.point_density < 500) {
            params.dcor_distance_factor = 0.02;
            params.dcor_min_pts = 6;
        } else {
            params.dcor_distance_factor = 0.018;
            params.dcor_min_pts = 7;
        }
    } else {
        params.dcor_distance_factor = defaults_.dcor_distance_factor;
        params.dcor_min_pts = defaults_.dcor_min_pts;
    }

    if (switches_.enable_intensity_distribution_adaptation) {
        if (features.intensity_q1 < 1.5) {
            params.rgor_intensity_threshold = std::max(features.intensity_q1 * 2.0, 3.5);
        } else if (features.intensity_q1 < 3.0) {
            params.rgor_intensity_threshold = features.intensity_q1 * 1.5;
        } else if (features.intensity_q1 < 6.0) {
            params.rgor_intensity_threshold = features.intensity_q1 * 1.3;
        } else {
            params.rgor_intensity_threshold = features.intensity_q1 * 1.2;
        }

        params.rgor_intensity_threshold = std::min(std::max(params.rgor_intensity_threshold, 3.5), 10.0);
    } else {
        params.rgor_intensity_threshold = defaults_.rgor_intensity_threshold;
    }

    params.rgor_alpha = switches_.enable_intensity_distribution_adaptation ?
        0.01 * (features.intensity_stddev / 5.0 + 0.5) : defaults_.rgor_alpha;
    params.rgor_alpha = std::min(std::max(params.rgor_alpha, 0.005), 0.02);

    params.rgor_min_variance_ratio = 0.12;

    double avg_nn_distance = 0.1;
    if (features.point_density > 0) {
        avg_nn_distance = 0.5 / std::cbrt(features.point_density);
    }

    if (switches_.enable_complexity_adaptation && features.scene_complexity > 0.7) {
        params.epsilon = avg_nn_distance * 1.6;
        params.min_points = 5;
    } else if (switches_.enable_complexity_adaptation && features.scene_complexity > 0.4) {
        params.epsilon = avg_nn_distance * 1.8;
        params.min_points = 5;
    } else if (switches_.enable_complexity_adaptation) {
        params.epsilon = avg_nn_distance * 2.0;
        params.min_points = 4;
    } else {
        params.epsilon = defaults_.epsilon;
        params.min_points = defaults_.min_points;
    }

    if (switches_.enable_complexity_adaptation) {
        params.epsilon = std::min(std::max(params.epsilon, 0.3), 1.1);
    }

    if (features.noise_ratio > 0.2) {
        params.min_points = 7;
    } else if (features.noise_ratio > 0.1) {
        params.min_points = 6;
    } else if (!switches_.enable_complexity_adaptation) {
        params.min_points = defaults_.min_points;
    }

    if (features.ground_ratio > 0.3) {
        params.ratio_threshold = 0.75;
    } else {
        params.ratio_threshold = 0.65;
    }

    params.ror_radius = params.epsilon * 0.85;
    params.ror_neighbors = params.min_points + 1;

    if (switches_.enable_adaptive_weight_adjustment) {
        if (features.scene_complexity > 0.7) {
            params.intensity_weight = 0.3;
            params.planarity_weight = 0.35;
            params.density_weight = 0.2;
            params.height_weight = 0.15;
        } else if (features.scene_complexity > 0.4) {
            params.intensity_weight = 0.35;
            params.planarity_weight = 0.3;
            params.density_weight = 0.2;
            params.height_weight = 0.15;
        } else {
            params.intensity_weight = 0.35;
            params.planarity_weight = 0.3;
            params.density_weight = 0.2;
            params.height_weight = 0.15;
        }

        if (features.intensity_stddev > 10.0) {
            params.intensity_weight *= 0.85;
            params.planarity_weight *= 1.15;

            double sum = params.intensity_weight + params.planarity_weight +
                         params.density_weight + params.height_weight;
            params.intensity_weight /= sum;
            params.planarity_weight /= sum;
            params.density_weight /= sum;
            params.height_weight /= sum;
        }
    } else {
        params.intensity_weight = defaults_.intensity_weight;
        params.planarity_weight = defaults_.planarity_weight;
        params.density_weight = defaults_.density_weight;
        params.height_weight = defaults_.height_weight;
    }

    SC_INFO("基于特征推荐参数 (平衡策略):");
    SC_INFO("DCOR: distance_factor=%.3f, min_pts=%d",
             params.dcor_distance_factor, params.dcor_min_pts);
    SC_INFO("RGOR: intensity_threshold=%.2f, alpha=%.4f",
             params.rgor_intensity_threshold, params.rgor_alpha);
    SC_INFO("聚类: epsilon=%.2f, min_points=%d, ratio_threshold=%.2f",
             params.epsilon, params.min_points, params.ratio_threshold);
    SC_INFO("特征权重: 强度=%.2f, 平面度=%.2f, 密度=%.2f, 高度=%.2f",
             params.intensity_weight, params.planarity_weight,
             params.density_weight, params.height_weight);

    return params;
}

// =====================================================================
// 参数优化入口
// ---------------------------------------------------------------------
// 优先级：有 GT 且开网格搜索 -> 网格搜索；否则开特征推荐 -> 推荐参数；
// 两者都关 -> 返回默认参数（launch 基线配置即此路径）。
// =====================================================================
FilterParameters ParameterOptimizer::optimize(const CloudPtr& cloud,
                                              const std::vector<int>& ground_truth_indices,
                                              const CloudFeatures& features,
                                              int max_iterations,
                                              int max_grid_search_points) const {
    SC_INFO("开始优化滤波参数...");

    if (switches_.enable_grid_search_optimization && !ground_truth_indices.empty()) {
        return grid_search_optimize(cloud, ground_truth_indices, features,
                                    max_iterations, max_grid_search_points);
    } else if (switches_.enable_feature_based_recommendation) {
        return recommend(features);
    }

    return defaults_;
}

// =====================================================================
// 体素降采样 + 最近点映射
// ---------------------------------------------------------------------
// ≤5000 点直接复制（映射为恒等）；否则体素化后用 KD 树为每个降采样点
// 找原始点云最近点，建立 downsampled_to_original。
// =====================================================================
CloudPtr ParameterOptimizer::downsample(const CloudPtr& cloud,
                                        std::vector<int>& downsampled_to_original,
                                        float leaf_size) const {
    CloudPtr result(new pcl::PointCloud<CloudPoint>);
    downsampled_to_original.clear();

    if (!cloud || cloud->empty()) {
        return result;
    }

    if (cloud->size() <= 5000) {
        *result = *cloud;
        downsampled_to_original.resize(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i) {
            downsampled_to_original[i] = i;
        }
        return result;
    }

    pcl::VoxelGrid<CloudPoint> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*result);

    downsampled_to_original.resize(result->size());

    pcl::KdTreeFLANN<CloudPoint> mapping_tree;
    mapping_tree.setInputCloud(cloud);

    #pragma omp parallel for if (switches_.enable_openmp_parallel)
    for (size_t i = 0; i < result->size(); ++i) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        mapping_tree.nearestKSearch(result->points[i], 1, indices, distances);
        downsampled_to_original[i] = indices[0];
    }

    SC_INFO("点云下采样: %lu -> %lu 点 (体素大小 %.2f)",
             cloud->size(), result->size(), leaf_size);

    return result;
}

// 网格搜索专用降采样：按点数选择叶子大小（0.08~0.12m）
CloudPtr ParameterOptimizer::downsample_for_optimization(const CloudPtr& cloud,
                                                         std::vector<int>& downsampled_to_original) const {
    float leaf_size = 0.08f;
    if (cloud->size() > 100000) {
        leaf_size = 0.12f;
    } else if (cloud->size() > 50000) {
        leaf_size = 0.1f;
    }
    return downsample(cloud, downsampled_to_original, leaf_size);
}

// =====================================================================
// 网格搜索参数优化（有 GT 时使用）
// ---------------------------------------------------------------------
// 在降采样点云上，迭代搜索 DCOR 距离因子/最小点数、强度阈值、epsilon、
// 权重组合，用 evaluate() 的 Fβ 加权得分挑选最优参数。
// 注意：DCOR/RGOR 参数目前不参与检测判定，搜索它们对结果无影响，
//       论文前建议把搜索维度改为 score_threshold 与强度阈值。
// =====================================================================
FilterParameters ParameterOptimizer::grid_search_optimize(const CloudPtr& cloud,
                                                          const std::vector<int>& ground_truth_indices,
                                                          const CloudFeatures& features,
                                                          int max_iterations,
                                                          int max_grid_search_points) const {
    SC_INFO("执行网格搜索参数优化，最大迭代次数: %d", max_iterations);

    FilterParameters best_params = defaults_;

    std::vector<int> downsampled_to_original;
    CloudPtr search_cloud;
    std::vector<int> search_ground_truth;

    if (cloud->size() > static_cast<size_t>(max_grid_search_points)) {
        search_cloud = downsample_for_optimization(cloud, downsampled_to_original);

        std::unordered_map<int, int> original_to_downsampled;
        for (size_t i = 0; i < downsampled_to_original.size(); ++i) {
            original_to_downsampled[downsampled_to_original[i]] = i;
        }

        for (int idx : ground_truth_indices) {
            auto it = original_to_downsampled.find(idx);
            if (it != original_to_downsampled.end()) {
                search_ground_truth.push_back(it->second);
            }
        }

        SC_INFO("优化用点云下采样: %lu -> %lu 点 (ground truth: %lu -> %lu)",
                 cloud->size(), search_cloud->size(),
                 ground_truth_indices.size(), search_ground_truth.size());
    } else {
        search_cloud = cloud;
        search_ground_truth = ground_truth_indices;
    }

    double best_score = evaluate(search_cloud, search_ground_truth, features, best_params);

    SC_INFO("初始参数评分: %.4f", best_score);

    // 实际参与判定的参数维度：融合得分阈值 + 固定强度阈值（仅固定阈值模式）+ 权重
    std::vector<float> score_thresholds = {0.45f, 0.50f, 0.55f, 0.60f, 0.65f,
                                           0.70f, 0.75f, 0.80f, 0.85f};
    const bool tune_intensity_threshold = !switches_.use_adaptive_intensity_threshold;
    std::vector<double> intensity_thresholds = {4.0, 5.0, 6.0, 7.0, 8.0};

    std::vector<std::vector<double>> weight_combinations = {
        {0.35, 0.3, 0.2, 0.15},
        {0.4, 0.25, 0.2, 0.15},
        {0.3, 0.35, 0.2, 0.15},
        {0.3, 0.3, 0.25, 0.15},
        {0.3, 0.3, 0.15, 0.25}
    };

    int iteration = 0;
    bool improved = true;

    while (improved && iteration < max_iterations) {
        improved = false;
        iteration++;

        // 融合得分阈值
        for (float st : score_thresholds) {
            if (st == best_params.score_threshold) continue;

            FilterParameters test_params = best_params;
            test_params.score_threshold = st;

            double score = evaluate(search_cloud, search_ground_truth, features, test_params);
            if (score > best_score) {
                best_score = score;
                best_params = test_params;
                improved = true;
                SC_INFO("迭代 %d: 改进 score_threshold = %.2f, 新评分: %.4f",
                         iteration, st, score);
            }
        }

        // 固定强度阈值（仅 use_adaptive_intensity_threshold=false 时有意义）
        if (tune_intensity_threshold) {
            for (double threshold : intensity_thresholds) {
                if (threshold == best_params.rgor_intensity_threshold) continue;

                FilterParameters test_params = best_params;
                test_params.rgor_intensity_threshold = threshold;

                double score = evaluate(search_cloud, search_ground_truth, features, test_params);
                if (score > best_score) {
                    best_score = score;
                    best_params = test_params;
                    improved = true;
                    SC_INFO("迭代 %d: 改进 rgor_intensity_threshold = %.1f, 新评分: %.4f",
                             iteration, threshold, score);
                }
            }
        }

        for (const auto& weights : weight_combinations) {
            if (weights[0] == best_params.intensity_weight &&
                weights[1] == best_params.planarity_weight &&
                weights[2] == best_params.density_weight &&
                weights[3] == best_params.height_weight) continue;

            FilterParameters test_params = best_params;
            test_params.intensity_weight = weights[0];
            test_params.planarity_weight = weights[1];
            test_params.density_weight = weights[2];
            test_params.height_weight = weights[3];

            double score = evaluate(search_cloud, search_ground_truth, features, test_params);
            if (score > best_score) {
                best_score = score;
                best_params = test_params;
                improved = true;
                SC_INFO("迭代 %d: 改进权重 [%.2f, %.2f, %.2f, %.2f], 新评分: %.4f",
                         iteration, weights[0], weights[1], weights[2], weights[3], score);
            }
        }
    }

    SC_INFO("参数搜索优化完成，最终评分: %.4f", best_score);
    SC_INFO("最优参数: score_threshold=%.2f, intensity_thresh=%.1f, 权重=[%.2f, %.2f, %.2f, %.2f]",
             best_params.score_threshold, best_params.rgor_intensity_threshold,
             best_params.intensity_weight, best_params.planarity_weight,
             best_params.density_weight, best_params.height_weight);

    return best_params;
}

// =====================================================================
// 参数评估（供网格搜索使用）
// ---------------------------------------------------------------------
// 用给定参数跑一遍检测，与 GT 对比计算 Precision/Recall/Fβ 加权得分，
// 并对“低于目标精度 90%/召回 98%”施加二次惩罚，得分越高越好。
// =====================================================================
double ParameterOptimizer::evaluate(const CloudPtr& cloud,
                                    const std::vector<int>& ground_truth_indices,
                                    const CloudFeatures& features,
                                    const FilterParameters& params) const {
    pcl::PointIndices::Ptr snow_indices(new pcl::PointIndices);

    try {
        RangeIntensityThreshold thresholds;
        if (switches_.use_range_binned_intensity_threshold) {
            thresholds = compute_range_thresholds(cloud, features);
        } else {
            thresholds.global_threshold = switches_.use_adaptive_intensity_threshold
                ? adaptive_intensity_threshold(features)
                : static_cast<float>(params.rgor_intensity_threshold);
        }
        if (switches_.use_otsu_intensity_threshold) {
            thresholds.global_threshold = compute_otsu_threshold(cloud);
        }
        if (switches_.use_idsor_intensity_threshold) {
            thresholds.use_smooth_range = true;
            thresholds.smooth_scale = params.idsor_scale;
            thresholds.smooth_rho = params.idsor_rho;
            thresholds.smooth_slope = params.idsor_slope;
            thresholds.smooth_r0 = params.idsor_r0;
            thresholds.smooth_k = params.idsor_k;
            thresholds.smooth_theta = params.idsor_theta;
        }
        int block_size = dynamic_block_size(cloud->size(), 1024);
        detector_.detect(cloud, thresholds, params, snow_indices, block_size);
    } catch (const std::exception& e) {
        SC_ERROR("参数评估异常: %s", e.what());
        return 0.0;
    }

    std::unordered_set<int> detected_set(snow_indices->indices.begin(),
                                         snow_indices->indices.end());
    std::unordered_set<int> gt_set(ground_truth_indices.begin(),
                                   ground_truth_indices.end());

    size_t tp = 0;
    size_t fp = 0;
    size_t fn = 0;

    for (int idx : snow_indices->indices) {
        if (gt_set.find(idx) != gt_set.end()) {
            tp++;
        } else {
            fp++;
        }
    }

    for (int idx : ground_truth_indices) {
        if (detected_set.find(idx) == detected_set.end()) {
            fn++;
        }
    }

    double precision = (tp + fp > 0) ? static_cast<double>(tp) / (tp + fp) : 0.0;
    double recall = (tp + fn > 0) ? static_cast<double>(tp) / (tp + fn) : 0.0;

    // NOTE: this is deliberately *not* an F-beta. Historically the function
    // computed an F-beta and then discarded it; the score that actually drove
    // the grid search is the weighted arithmetic mean below, penalised when
    // either metric falls short of its target. Kept as-is so that any result
    // produced by the grid search is unchanged.
    double precision_weight = 1.2;
    double recall_weight = 1.0;

    double weighted_score = 0.0;
    if (precision + recall > 0) {
        weighted_score = (precision_weight * precision + recall_weight * recall) /
                         (precision_weight + recall_weight);
    }

    double precision_target = 0.9;
    double recall_target = 0.98;

    double precision_penalty = (precision < precision_target) ?
        std::pow((precision / precision_target), 2) : 1.0;
    double recall_penalty = (recall < recall_target) ?
        std::pow((recall / recall_target), 2) : 1.0;

    double final_score = weighted_score * precision_penalty * recall_penalty;

    return final_score;
}

}  // namespace snowclear