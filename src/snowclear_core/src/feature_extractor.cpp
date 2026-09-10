#include "snowclear/feature_extractor.hpp"

// =====================================================================
// 本文件可调参数（消融开关，默认值见 ablation_switches.h）
// ---------------------------------------------------------------------
//   use_intensity_histogram_stats   开   256级直方图统计（确定性并行）
//   use_adaptive_intensity_threshold 开  自适应强度阈值（Q1驱动，见 parameter_optimizer）
//   enable_planarity_calculation    关   局部PCA平面度（最终配置移除，贡献≈0）
//   enable_planarity_cache          关   平面度缓存
//   enable_density_calculation      关   密度得分（负贡献）
//   enable_relative_height          开   离地高度（唯一保留的几何特征）
//   enable_height_consistency       关   高度一致性（负贡献）
//   enable_feature_entropy          关   特征熵（负贡献）
//   enable_scene_complexity         关   场景复杂度（自适应路径用）
//   enable_vertical_zscore          关   FlakeOut 垂直 z-score（无增益）
//   enable_kdtree_cache / enable_openmp_parallel  性能开关
// =====================================================================

#include <pcl/common/common.h>
#include <Eigen/Dense>

#include <omp.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <cstdint>

namespace snowclear {

namespace {

// 1m 地面网格 key：用无符号位运算代替“负数左移”这一未定义行为。
// 结果与旧实现（x86 上实际产生的位模式）保持一致，避免改变已发表 WADS 结果。
inline int64_t ground_cell_key(float x, float y, float cell) {
    const uint64_t x_bits = static_cast<uint64_t>(static_cast<int64_t>(std::floor(x / cell)));
    const uint64_t y_bits = static_cast<uint64_t>(static_cast<int64_t>(std::floor(y / cell)));
    return static_cast<int64_t>((x_bits << 32) ^ (y_bits & 0xffffffffULL));
}

}  // namespace

FeatureExtractor::FeatureExtractor(const AblationSwitches& switches)
    : switches_(switches) {
    kdtree_.reset(new pcl::KdTreeFLANN<CloudPoint>);
}

// =====================================================================
// KD 树管理：确保当前查询树与给定点云一致
// ---------------------------------------------------------------------
// 受 enable_kdtree_cache 控制；缓存开启时，同一片点云只建树一次。
// =====================================================================
void FeatureExtractor::ensure_kdtree(const CloudPtr& cloud) {
    if (!switches_.enable_kdtree_cache) return;
    if (kdtree_initialized_ && last_cloud_ == cloud) return;

    kdtree_->setInputCloud(cloud);
    last_cloud_ = cloud;
    kdtree_initialized_ = true;
}

// K 近邻查询（内部 KD 树）
int FeatureExtractor::nearest_k(const CloudPoint& point, int k,
                                std::vector<int>& indices,
                                std::vector<float>& dists) const {
    if (!kdtree_initialized_) return 0;
    return kdtree_->nearestKSearch(point, k, indices, dists);
}

// 半径邻域查询（内部 KD 树）
int FeatureExtractor::radius_search(const CloudPoint& point, float radius,
                                    std::vector<int>& indices,
                                    std::vector<float>& dists) const {
    if (!kdtree_initialized_) return 0;
    return kdtree_->radiusSearch(point, radius, indices, dists);
}

// =====================================================================
// 强度直方图统计（确定性并行）
// ---------------------------------------------------------------------
// 用 256 级直方图 + CDF 计算均值/标准差/Q1/中位数/Q3。
// 并行策略：每个线程维护局部整数直方图，最后按线程序整数合并——
// 整数加法与线程数无关，因此串行/并行结果逐位一致。
// 均值/标准差也由直方图分箱值计算，进一步保证确定性。
// =====================================================================
void FeatureExtractor::compute_intensity_histogram_stats(const CloudPtr& cloud,
                                                         float& mean, float& stddev,
                                                         float& q1, float& median,
                                                         float& q3) const {
    if (!cloud || cloud->empty()) {
        mean = 0.0f;
        stddev = 0.0f;
        q1 = 0.0f;
        median = 0.0f;
        q3 = 0.0f;
        return;
    }

    std::vector<int> histogram(256, 0);
    const size_t n = cloud->size();

    if (switches_.enable_openmp_parallel) {
        const int num_threads = omp_get_max_threads();
        std::vector<std::vector<int>> thread_hists(
            num_threads, std::vector<int>(256, 0));

        #pragma omp parallel
        {
            const int tid = omp_get_thread_num();
            auto& local = thread_hists[tid];
            #pragma omp for nowait
            for (size_t i = 0; i < n; ++i) {
                // 强度必须钳制到 [0,255]：负强度/异常值会索引到数组边界之外（内存越界写）
                const int bin = std::max(0, std::min(255,
                    static_cast<int>(cloud->points[i].intensity)));
                local[bin]++;
            }
        }

        for (const auto& local : thread_hists) {
            for (int b = 0; b < 256; ++b) {
                histogram[b] += local[b];
            }
        }
    } else {
        for (const auto& point : cloud->points) {
            const int bin = std::max(0, std::min(255,
                static_cast<int>(point.intensity)));
            histogram[bin]++;
        }
    }

    // 由直方图分箱值计算均值与标准差（整数合并，串并行一致）
    double sum = 0.0;
    for (int b = 0; b < 256; ++b) {
        sum += static_cast<double>(b) * histogram[b];
    }
    mean = static_cast<float>(sum / n);

    double sum_squared_diff = 0.0;
    for (int b = 0; b < 256; ++b) {
        double diff = static_cast<double>(b) - mean;
        sum_squared_diff += histogram[b] * diff * diff;
    }
    stddev = static_cast<float>(std::sqrt(sum_squared_diff / n));

    std::vector<int> cdf(256, 0);
    cdf[0] = histogram[0];
    for (int i = 1; i < 256; ++i) {
        cdf[i] = cdf[i - 1] + histogram[i];
    }

    int total_points = static_cast<int>(n);
    float q1_threshold = total_points * 0.25f;
    float median_threshold = total_points * 0.5f;
    float q3_threshold = total_points * 0.75f;

    q1 = 0.0f;
    median = 0.0f;
    q3 = 0.0f;

    if (switches_.fix_quantile_sentinel) {
        // 修复：原实现用 `q1 == 0.0f` 当"尚未赋值"哨兵。当 bin 0 的累计已过 25%
        // （雪场景极常见：ROI 内 I=0 平均占 43.26%），q1 在 i=0 被赋 0.0f 后哨兵
        // 仍为真，于是后续每个 bin 继续覆写 q1，最终 q1 == q3 所在 bin。
        // 全量 1620 帧实测：879 帧(54.26%) 的 Q1 算错。
        // 当前被 clamp(Q1*0.8, 2.5, 8.0) 完全掩盖（0 帧最终阈值改变），
        // 但一旦改 clamp/系数或开启强度分布自适应就会立即生效。
        bool q1_set = false, med_set = false;
        for (int i = 0; i < 256; ++i) {
            if (!q1_set && cdf[i] >= q1_threshold) { q1 = static_cast<float>(i); q1_set = true; }
            if (!med_set && cdf[i] >= median_threshold) { median = static_cast<float>(i); med_set = true; }
            if (cdf[i] >= q3_threshold) { q3 = static_cast<float>(i); break; }
        }
    } else {
        for (int i = 0; i < 256; ++i) {
            if (cdf[i] >= q1_threshold && q1 == 0.0f) {
                q1 = static_cast<float>(i);
            }
            if (cdf[i] >= median_threshold && median == 0.0f) {
                median = static_cast<float>(i);
            }
            if (cdf[i] >= q3_threshold && q3 == 0.0f) {
                q3 = static_cast<float>(i);
                break;
            }
        }
    }
}

// =====================================================================
// 场景级特征分析（模块主入口）
// ---------------------------------------------------------------------
// 输出 CloudFeatures：
//   - 空间范围、平均距离、体素密度
//   - 强度均值/标准差/范围/分位数（受 use_intensity_histogram_stats 控制）
//   - 地面点比例、噪声点比例（采样 1/1000 点估计）
//   - 场景复杂度（受 enable_scene_complexity 控制）
// 内部会按需构建 KD 树供噪声比例统计复用。
// =====================================================================
CloudFeatures FeatureExtractor::analyze(const CloudPtr& cloud) {
    CloudFeatures features;

    if (!cloud || cloud->empty()) {
        SC_ERROR("分析点云时输入点云为空");
        return features;
    }

    try {
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);

        features.x_range = max_pt.x - min_pt.x;
        features.y_range = max_pt.y - min_pt.y;
        features.z_range = max_pt.z - min_pt.z;

        double total_distance = 0.0;
        float min_intensity = 255.0f;
        float max_intensity = 0.0f;
        for (const auto& point : cloud->points) {
            double dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            total_distance += dist;
            min_intensity = std::min(min_intensity, point.intensity);
            max_intensity = std::max(max_intensity, point.intensity);
        }
        features.avg_distance_to_sensor = total_distance / cloud->size();
        features.intensity_range = max_intensity - min_intensity;

        double volume = features.x_range * features.y_range * features.z_range;
        features.point_density = (volume > 0) ? cloud->size() / volume : 0;

        if (switches_.use_intensity_histogram_stats) {
            float mean, stddev, q1, median, q3;
            compute_intensity_histogram_stats(cloud, mean, stddev, q1, median, q3);
            features.avg_intensity = mean;
            features.intensity_stddev = stddev;
            features.intensity_q1 = q1;
            features.intensity_median = median;
            features.intensity_q3 = q3;
        } else {
            float sum = 0.0f;
            for (const auto& point : cloud->points) {
                sum += point.intensity;
            }
            features.avg_intensity = sum / cloud->size();
            features.intensity_stddev = 5.0f;
            features.intensity_q1 = features.avg_intensity * 0.5f;
            features.intensity_median = features.avg_intensity;
            features.intensity_q3 = features.avg_intensity * 1.5f;
        }

        // 地面/噪声比例采样：仅在参数推荐/自适应路径确实会消费这些特征时计算，
        // 否则跳过 ~1000 次半径查询（基线路径下这些数值不影响检测判定）。
        const bool need_noise_stats =
            switches_.enable_feature_based_recommendation ||
            switches_.enable_grid_search_optimization ||
            switches_.enable_adaptive_parameter_adjustment ||
            switches_.enable_density_adaptation;
        if (need_noise_stats) {
            int ground_points = 0;
            int noise_points = 0;

            ensure_kdtree(cloud);

            const int sample_step = std::max(1, static_cast<int>(cloud->size() / 1000));

            auto sample_point = [&](size_t i) {
                const auto& point = cloud->points[i];
                if (point.z < (min_pt.z + 0.3)) {
                    ground_points++;
                }

                if (switches_.enable_kdtree_cache) {
                    std::vector<int> indices;
                    std::vector<float> distances;
                    if (radius_search(point, 0.5f, indices, distances) < 5) {
                        noise_points++;
                    }
                } else {
                    int neighbors = 0;
                    for (size_t j = 0; j < cloud->size() && neighbors < 5; j += sample_step) {
                        if (j != i) {
                            const auto& other_pt = cloud->points[j];
                            float dist_sq = (point.x - other_pt.x) * (point.x - other_pt.x) +
                                            (point.y - other_pt.y) * (point.y - other_pt.y) +
                                            (point.z - other_pt.z) * (point.z - other_pt.z);
                            if (dist_sq < 0.25f) {
                                neighbors++;
                            }
                        }
                    }
                    if (neighbors < 5) {
                        noise_points++;
                    }
                }
            };

            if (switches_.enable_openmp_parallel) {
                #pragma omp parallel for reduction(+:ground_points,noise_points)
                for (size_t i = 0; i < cloud->size(); i += sample_step) {
                    sample_point(i);
                }
            } else {
                for (size_t i = 0; i < cloud->size(); i += sample_step) {
                    sample_point(i);
                }
            }

            ground_points = static_cast<int>(ground_points * sample_step);
            noise_points = static_cast<int>(noise_points * sample_step);

            features.ground_ratio = static_cast<double>(ground_points) / cloud->size();
            features.noise_ratio = static_cast<double>(noise_points) / cloud->size();
        }

        if (switches_.enable_scene_complexity) {
            features.calculate_complexity();
        } else {
            features.scene_complexity = 0.5;
        }

        // 地面网格：离地高度与垂直 z-score 都依赖它
        if (switches_.enable_relative_height || switches_.enable_vertical_zscore) {
            build_ground_grid(cloud);
        }

        SC_INFO("点云特征分析完成:");
        SC_INFO("空间范围: X:%.2f Y:%.2f Z:%.2f", features.x_range, features.y_range, features.z_range);
        SC_INFO("点密度: %.2f 点/立方米", features.point_density);
        SC_INFO("平均距离: %.2f m", features.avg_distance_to_sensor);
        SC_INFO("强度特征: 均值=%.2f 标准差=%.2f 范围=%.2f",
                 features.avg_intensity, features.intensity_stddev, features.intensity_range);
        SC_INFO("强度分位数: Q1=%.2f 中位数=%.2f Q3=%.2f",
                 features.intensity_q1, features.intensity_median, features.intensity_q3);
        SC_INFO("地面点比例: %.2f%%", features.ground_ratio * 100);
        SC_INFO("噪声点比例: %.2f%%", features.noise_ratio * 100);
        SC_INFO("场景复杂度: %.2f", features.scene_complexity);

        return features;
    } catch (const std::exception& e) {
        SC_ERROR("点云特征分析异常: %s", e.what());
        return features;
    }
}

// =====================================================================
// 局部特征熵
// ---------------------------------------------------------------------
// 综合邻域强度方差、高度方差、空间方差，归一化到 [0,1]。
// 值越大说明邻域越“混乱”。雪点邻域通常稀疏且混合，熵偏高。
// 受 enable_feature_entropy 控制；关闭时返回固定值 0.5。
// =====================================================================
double FeatureExtractor::feature_entropy(const CloudPtr& cloud,
                                         const std::vector<int>& neighbor_indices) const {
    if (!switches_.enable_feature_entropy) {
        return 0.5;
    }

    if (neighbor_indices.size() < 5) return 1.0;

    std::vector<float> intensities;
    intensities.reserve(neighbor_indices.size());
    for (int idx : neighbor_indices) {
        intensities.push_back(cloud->points[idx].intensity);
    }

    float mean_intensity = std::accumulate(intensities.begin(), intensities.end(), 0.0f) / intensities.size();

    float intensity_variance = 0.0f;
    for (float intensity : intensities) {
        float diff = intensity - mean_intensity;
        intensity_variance += diff * diff;
    }
    intensity_variance /= intensities.size();

    std::vector<float> heights;
    heights.reserve(neighbor_indices.size());
    for (int idx : neighbor_indices) {
        heights.push_back(cloud->points[idx].z);
    }

    float mean_height = std::accumulate(heights.begin(), heights.end(), 0.0f) / heights.size();

    float height_variance = 0.0f;
    for (float height : heights) {
        float diff = height - mean_height;
        height_variance += diff * diff;
    }
    height_variance /= heights.size();

    Eigen::Vector3f centroid(0, 0, 0);
    for (int idx : neighbor_indices) {
        centroid[0] += cloud->points[idx].x;
        centroid[1] += cloud->points[idx].y;
        centroid[2] += cloud->points[idx].z;
    }
    centroid /= neighbor_indices.size();

    float spatial_variance = 0.0f;
    for (int idx : neighbor_indices) {
        const auto& pt = cloud->points[idx];
        float dist_sq = (pt.x - centroid[0]) * (pt.x - centroid[0]) +
                        (pt.y - centroid[1]) * (pt.y - centroid[1]) +
                        (pt.z - centroid[2]) * (pt.z - centroid[2]);
        spatial_variance += dist_sq;
    }
    spatial_variance /= neighbor_indices.size();

    double entropy = std::min(1.0, (intensity_variance * 0.5 +
                                    height_variance * 10.0 +
                                    spatial_variance * 5.0) / 15.0);
    return entropy;
}

// =====================================================================
// 局部几何特征（PCA）
// ---------------------------------------------------------------------
// 对邻域点做协方差特征值分解（λ1>=λ2>=λ3）：
//   - planarity = (λ2-λ3)/λ1 ：真实“表面性”，地面/墙面接近 1，雪点团簇接近 0
//   - scattering = λ3/λ1     ：各向同性/团簇程度，雪点团簇高、平面低
// 旧版用 1-λ3/Σλ 会把近各向同性团簇也判成高“平面度”，导致门控语义反转。
// 受 enable_planarity_calculation 控制；缓存由调用方（检测器）按线程持有。
// =====================================================================
double FeatureExtractor::planarity(size_t point_idx,
                                   const std::vector<int>& neighbor_indices,
                                   const CloudPtr& cloud,
                                   std::vector<PointFeature>& cache,
                                   double* scattering_out) {
    if (!switches_.enable_planarity_calculation || neighbor_indices.size() < 3) {
        return 0.0;
    }

    if (switches_.enable_planarity_cache &&
        point_idx < cache.size() && cache[point_idx].is_computed) {
        return cache[point_idx].planarity;
    }

    Eigen::Vector3f centroid(0, 0, 0);
    for (size_t i = 0; i < neighbor_indices.size(); ++i) {
        centroid[0] += cloud->points[neighbor_indices[i]].x;
        centroid[1] += cloud->points[neighbor_indices[i]].y;
        centroid[2] += cloud->points[neighbor_indices[i]].z;
    }
    centroid /= neighbor_indices.size();

    Eigen::Matrix3f covariance_matrix;
    covariance_matrix.setZero();

    for (size_t i = 0; i < neighbor_indices.size(); ++i) {
        Eigen::Vector3f pt_centered(
            cloud->points[neighbor_indices[i]].x - centroid[0],
            cloud->points[neighbor_indices[i]].y - centroid[1],
            cloud->points[neighbor_indices[i]].z - centroid[2]
        );

        covariance_matrix(0, 0) += pt_centered[0] * pt_centered[0];
        covariance_matrix(0, 1) += pt_centered[0] * pt_centered[1];
        covariance_matrix(0, 2) += pt_centered[0] * pt_centered[2];
        covariance_matrix(1, 1) += pt_centered[1] * pt_centered[1];
        covariance_matrix(1, 2) += pt_centered[1] * pt_centered[2];
        covariance_matrix(2, 2) += pt_centered[2] * pt_centered[2];
    }

    covariance_matrix(1, 0) = covariance_matrix(0, 1);
    covariance_matrix(2, 0) = covariance_matrix(0, 2);
    covariance_matrix(2, 1) = covariance_matrix(1, 2);

    covariance_matrix /= neighbor_indices.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
    Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();

    std::sort(eigenvalues.data(), eigenvalues.data() + 3, std::greater<float>());

    const float lambda1 = eigenvalues[0];
    const float lambda2 = eigenvalues[1];
    const float lambda3 = eigenvalues[2];
    double planarity = 0.0;
    double scattering = 0.0;

    if (lambda1 > 1e-8) {
        planarity = static_cast<double>((lambda2 - lambda3) / lambda1);
        scattering = static_cast<double>(lambda3 / lambda1);
    }

    if (switches_.enable_planarity_cache &&
        point_idx < cloud->size()) {
        if (cache.size() <= static_cast<size_t>(point_idx)) {
            cache.resize(cloud->size());
        }
        cache[point_idx].planarity = planarity;
        cache[point_idx].scattering = scattering;
        cache[point_idx].neighbors = neighbor_indices;
        cache[point_idx].is_computed = true;
    }

    if (scattering_out) {
        *scattering_out = scattering;
    }
    return planarity;
}

// =====================================================================
// 局部地面网格构建
// ---------------------------------------------------------------------
// 按 1m 水平网格统计各格内点的 z，取第 10 百分位作为该格地面高度；
// 格内点数 <5 视为无有效地面。全局最低 z 作为兜底。
// 只在 enable_relative_height 时调用，O(n) 单趟。
// =====================================================================
void FeatureExtractor::build_ground_grid(const CloudPtr& cloud) {
    ground_grid_.clear();
    ground_grid_fallback_ = 0.0f;
    cell_zstats_.clear();
    if (!cloud || cloud->empty()) return;

    constexpr float cell = 1.0f;
    std::unordered_map<int64_t, std::vector<float>> cell_zs;
    float global_min_z = std::numeric_limits<float>::max();

    for (const auto& pt : cloud->points) {
        const int64_t key = ground_cell_key(pt.x, pt.y, cell);
        cell_zs[key].push_back(pt.z);
        global_min_z = std::min(global_min_z, pt.z);
    }
    ground_grid_fallback_ = global_min_z;

    for (auto& entry : cell_zs) {
        auto& zs = entry.second;
        if (zs.size() < 5) continue;
        std::sort(zs.begin(), zs.end());
        // 第10百分位，抗离群点。
        // 【已知问题，故意未改】zs.size()/10 在格内点数 5~9 时下标为 0，
        // 退化成"取最小 z"，最易受离群点影响。修它会改变 ground_grid ->
        // height_above_ground -> 检测结果，破坏与论文的逐字节一致性，
        // 因此仅在此记录；若要修，应新增开关并重跑全量验证。
        ground_grid_[entry.first] = zs[zs.size() / 10];

        // 垂直 z-score 用到的局部均值/标准差。仅在 enable_vertical_zscore 时才需要：
        // 原实现无条件计算，白跑每格两遍循环 + 一次哈希插入。
        if (switches_.enable_vertical_zscore) {
            double sum = 0.0;
            for (float z : zs) sum += z;
            const float mean = static_cast<float>(sum / zs.size());
            double sq = 0.0;
            for (float z : zs) {
                const double d = z - mean;
                sq += d * d;
            }
            const float stddev = static_cast<float>(std::sqrt(sq / zs.size()));
            cell_zstats_[entry.first] = {mean, std::max(0.05f, stddev)};
        }
    }
}

// =====================================================================
// 归一化离地高度：0=贴地，1=高于地面 1.5m 以上
// =====================================================================
float FeatureExtractor::height_above_ground(const CloudPoint& point) const {
    constexpr float cell = 1.0f;
    constexpr float scale = 1.5f;
    const int64_t key = ground_cell_key(point.x, point.y, cell);
    auto it = ground_grid_.find(key);
    const float ground_z = (it != ground_grid_.end()) ? it->second : ground_grid_fallback_;
    return std::max(0.0f, std::min(1.0f, (point.z - ground_z) / scale));
}

// =====================================================================
// 垂直 z-score（FlakeOut, Clemens-Sewall et al. 2022）
// ---------------------------------------------------------------------
// 用所在 1m 网格内点的 z 均值/标准差计算：(z-μ)/σ。
// 雪点若悬浮在表面上空则 z-score 高；贴地雪点接近 0。
// =====================================================================
float FeatureExtractor::vertical_zscore(const CloudPoint& point) const {
    constexpr float cell = 1.0f;
    const int64_t key = ground_cell_key(point.x, point.y, cell);
    auto it = cell_zstats_.find(key);
    if (it == cell_zstats_.end()) return 0.0f;
    return (point.z - it->second.first) / it->second.second;
}

}  // namespace snowclear