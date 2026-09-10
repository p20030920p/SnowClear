#ifndef CLOUD_OPERATIONS_TYPES_H
#define CLOUD_OPERATIONS_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace snowclear {

// =============== 点云别名 =============== //
using CloudPoint = pcl::PointXYZI;
using CloudPtr = pcl::PointCloud<CloudPoint>::Ptr;
using CloudConstPtr = pcl::PointCloud<CloudPoint>::ConstPtr;

// =============== 时间统计结构 =============== //
// 全部字段单位为**微秒**。
// 原实现用 duration_cast<milliseconds> 逐帧向下取整后再整数累加、整数相除：
// 每个阶段每帧最多丢 1 ms，而"不含评估"是三个截断值之和，在 9~10 ms 量级上
// 是系统性低估。改用微秒累计、打印时再换算成 ms（带小数）。
// io_time 单独一列：原实现把 loadPCDFile(约 3.2 MB/帧) 计在 preprocess 里，
// 导致报告的"预处理/总时间"实际包含磁盘性能，冷热缓存能差 2 倍。
struct TimingStats {
    long io_time = 0;
    long preprocess_time = 0;
    long param_optimize_time = 0;
    long filtering_time = 0;
    long evaluation_time = 0;
    long total_time = 0;

    void reset() {
        io_time = preprocess_time = param_optimize_time = filtering_time = 0;
        evaluation_time = total_time = 0;
    }

    static double to_ms(long us) { return static_cast<double>(us) / 1000.0; }

    void print(const std::string& prefix = "") const {
        std::cout << "\n============ " << prefix << " 时间统计 ============" << std::endl;
        std::cout << "读盘+去NaN时间(不计入算法): " << to_ms(io_time) << " ms" << std::endl;
        std::cout << "预处理时间: " << to_ms(preprocess_time) << " ms" << std::endl;
        std::cout << "参数优化时间: " << to_ms(param_optimize_time) << " ms" << std::endl;
        std::cout << "雪点滤波时间: " << to_ms(filtering_time) << " ms" << std::endl;
        std::cout << "评估时间: " << to_ms(evaluation_time) << " ms" << std::endl;
        std::cout << "算法总时间(不含IO/评估): "
                  << to_ms(preprocess_time + param_optimize_time + filtering_time)
                  << " ms" << std::endl;
        std::cout << "======================================" << std::endl;
    }
};

// =============== 评估指标结构 =============== //
struct EvaluationMetrics {
    int total_frames = 0;
    double total_precision = 0.0;
    double total_recall = 0.0;
    double total_f1_score = 0.0;
    double total_f2_score = 0.0;
    double total_accuracy = 0.0;
    size_t total_tp = 0;
    size_t total_fp = 0;
    size_t total_fn = 0;
    size_t total_tn = 0;

    // 数据卫生统计（严格评估口径）：让"被排除的帧"和"被清洗的 GT"显式可见，
    // 而不是像原实现那样静默跳过。
    size_t frames_zero_detection = 0;   // 零检测帧（旧口径会整帧丢弃 -> 宏平均偏乐观）
    size_t frames_zero_gt = 0;          // 零 GT 帧（旧口径当"无标注"跳过 -> FP 不被惩罚）
    size_t gt_duplicates_removed = 0;
    size_t gt_negatives_removed = 0;
    size_t gt_parse_failures = 0;

    void add(double precision, double recall, double f1_score, double f2_score, double accuracy,
             size_t tp, size_t fp, size_t fn, size_t tn);
    void print_average() const;
    void reset();
};

// =============== 点云特征结构 =============== //
struct CloudFeatures {
    // 强度特征
    float avg_intensity = 0.0f;
    float intensity_stddev = 0.0f;
    float intensity_range = 0.0f;
    float intensity_q1 = 0.0f;
    float intensity_median = 0.0f;
    float intensity_q3 = 0.0f;

    // 空间特征
    double x_range = 0.0;
    double y_range = 0.0;
    double z_range = 0.0;
    double avg_distance_to_sensor = 0.0;
    double point_density = 0.0;
    double ground_ratio = 0.0;
    double noise_ratio = 0.0;
    double scene_complexity = 0.0;

    void calculate_complexity() {
        double intensity_complexity = std::min(1.0, static_cast<double>(intensity_stddev) / 20.0);
        double spatial_complexity = std::min(1.0, (x_range + y_range + z_range) / 100.0);
        double density_complexity = std::min(1.0, point_density / 1000.0);
        scene_complexity = (intensity_complexity * 0.4 + spatial_complexity * 0.3 + density_complexity * 0.3);
    }
};

// =============== 点特征缓存 =============== //
struct PointFeature {
    bool is_computed = false;
    double planarity = 0.0;
    double scattering = 0.0;  // λ3/λ1：近各向同性团簇高，平面低
    std::vector<int> neighbors;
};

// =============== 索引映射结构 =============== //
struct IndexMapping {
    std::vector<int> processed_to_original;
    std::vector<int> original_to_processed;

    // 精确重复点分组：canonical original index -> 全部原始索引（含自身）
    // 用于去重后把检测结果展开回所有重复副本；去重关闭时每个分组大小为 1。
    std::vector<int> original_to_canonical;   // 原始索引 -> canonical 原始索引
    std::vector<int> canonical_group_id;      // canonical 原始索引 -> 密集分组号
    std::vector<int> group_members_flat;      // 各分组原始索引拼接
    std::vector<int> group_start;             // group_start[g]..group_start[g+1]-1
};

// =============== 滤波参数结构 =============== //
struct FilterParameters {
    // 融合得分阈值（网格搜索可优化的核心参数之一）
    float score_threshold = 0.65f;
    // IDSOR 平滑阈值参数（默认关闭时无效）
    float idsor_scale = 1.0f;
    float idsor_rho = 1.0f;
    float idsor_slope = 0.0f;   // 距离递增阈值斜率（远距离雪召回）
    float idsor_r0 = 5.0f;      // 距离递增起始半径
    float idsor_k = 2.15f;      // Gamma 形状（论文原值，1062 帧验证版本）
    float idsor_theta = 2.38f;  // Gamma 尺度（论文原值，1062 帧验证版本）

    // DCOR参数（保留字段，供参数推荐/网格搜索使用）
    double dcor_distance_factor = 0.025;
    int dcor_min_pts = 5;

    // RGOR参数
    double rgor_intensity_threshold = 8.0;
    double rgor_alpha = 0.01;
    double rgor_min_variance_ratio = 0.1;

    // 聚类参数
    double epsilon = 0.8;
    int min_points = 4;
    double ratio_threshold = 0.75;
    double ror_radius = 0.8;
    int ror_neighbors = 2;

    // 特征权重
    double intensity_weight = 0.35;
    double planarity_weight = 0.25;
    double density_weight = 0.25;
    double height_weight = 0.15;

};

// =============== 距离分段强度阈值 =============== //
// 按水平距离分段维护强度阈值：近处雪密、远处雪稀，单一阈值有偏。
// thresholds.size() == 0 时退化为全局阈值（threshold_at 返回 global_threshold）。
struct RangeIntensityThreshold {
    std::vector<float> bin_edges = {0.0f, 5.0f, 10.0f, 15.0f, 18.0f};
    std::vector<float> thresholds;  // 每段一个阈值，长度应为 bin_edges.size()-1
    float global_threshold = 8.0f;

    // IDSOR 式平滑距离-强度阈值（Yan & Bengtsson, 2025/2026）
    // T_i = s*Tg*(1 - α_i*h_i)；α 由天气粒子距离分布(Gamma)决定，
    // 近处强度主导(阈值压低利于低强度雪点)，远处退回几何基线。
    bool use_smooth_range = false;
    float smooth_scale = 1.0f;
    float smooth_rho = 1.0f;
    float smooth_k = 2.15f;
    float smooth_theta = 2.38f;
    float smooth_slope = 0.0f;  // 距离递增斜率：T += slope*max(0, r-r0)
    float smooth_r0 = 5.0f;     // 距离递增起始半径
    float max_intensity = 255.0f;

    // ---- α(r) 查表加速（数学等价，见 docs/AUDIT_REPORT.md §3.1 H2） ----
    // 原实现对**每个点**调用 pow + exp + tgamma + sqrt。其中 tgamma(smooth_k) 是
    // 帧常量却逐点重算；且 α 只依赖 r，与 intensity 无关。
    // 实测（单线程 -O3, ROI 5.4 万点/帧）：1.140 ms -> 0.179 ms，省 84%。
    // 表在 build_lut() 里每帧建一次；未建表时 threshold_at 自动回退到原式。
    std::vector<float> alpha_lut;      // α 采样，步长 kLutStep
    static constexpr float kLutStep = 0.01f;   // 1 cm
    static constexpr int kLutSize = 4001;      // 0 .. 40 m

    void build_lut() {
        alpha_lut.clear();
        if (!use_smooth_range || smooth_theta <= 0.0f) return;
        alpha_lut.resize(kLutSize);
        const double gk = std::tgamma(smooth_k);          // 常量，移出逐点循环
        for (int i = 0; i < kLutSize; ++i) {
            const double r = i * static_cast<double>(kLutStep);
            double fr = 0.0;
            if (r > 0.0) {
                const double xr = r / smooth_theta;
                fr = std::pow(xr, smooth_k - 1.0) * std::exp(-xr) / (gk * smooth_theta);
            }
            alpha_lut[i] = static_cast<float>(smooth_rho * fr / (smooth_rho * fr + 1.0));
        }
    }

    float threshold_at(float x, float y, float intensity) const {
        if (use_smooth_range) {
            const float r = std::sqrt(x * x + y * y);
            double alpha;
            if (!alpha_lut.empty()) {
                int idx = static_cast<int>(r / kLutStep);
                if (idx < 0) idx = 0;
                if (idx >= kLutSize) idx = kLutSize - 1;
                alpha = alpha_lut[idx];
            } else {
                double fr = 0.0;
                if (r > 0.0f && smooth_theta > 0.0f) {
                    const double xr = r / smooth_theta;
                    fr = std::pow(xr, smooth_k - 1.0) * std::exp(-xr) /
                         (std::tgamma(smooth_k) * smooth_theta);
                }
                alpha = smooth_rho * fr / (smooth_rho * fr + 1.0);
            }
            const double h = 1.0 - std::min(1.0f, intensity / max_intensity);
            const float r_extra = smooth_slope * std::max(0.0f, r - smooth_r0);
            const float t = smooth_scale * global_threshold *
                            static_cast<float>(1.0 - alpha * h) + r_extra;
            return std::max(2.0f, std::min(20.0f, t));
        }
        if (thresholds.empty()) {
            return global_threshold;
        }
        const float range = std::sqrt(x * x + y * y);
        for (size_t i = 0; i + 1 < bin_edges.size() && i < thresholds.size(); ++i) {
            if (range >= bin_edges[i] && range < bin_edges[i + 1]) {
                return thresholds[i];
            }
        }
        return thresholds.back();
    }
};

// =============== 预处理输出结构 =============== //
struct ProcessedCloud {
    CloudPtr cloud;
    CloudPtr stats_cloud;  // 保留重复点的统计用点云（去重时与 cloud 不同；关闭时相同）
    IndexMapping mapping;  // processed -> original / original -> processed
};

}  // namespace snowclear

#endif  // CLOUD_OPERATIONS_TYPES_H
