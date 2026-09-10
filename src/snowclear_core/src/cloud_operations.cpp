#include "snowclear/cloud_operations.hpp"

// =====================================================================
// 本文件可调参数：全部集中在 include/system_config.h（数值参数）与
// include/ablation_switches.h（消融开关），可经 launch 或 _param:=value 覆盖。
// 本文件直接相关的运行参数:
//   detector_type   feature_fusion | dror | dsor   检测器选择
//   result_folder / gt_folder / output_dir          GT 与输出目录
//   save_results    false                           是否保存检测索引
// =====================================================================

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <unordered_set>

namespace snowclear {

// =====================================================================
// 构造函数：读取 ROS 参数、装配各模块实例
// ---------------------------------------------------------------------
// 职责：
//   1. SystemConfig::load 读取全部数值参数（几何/阈值/权重/路径）
//   2. AblationSwitches::load 读取消融开关
//   3. 按依赖关系创建 FeatureExtractor / SnowDetector / ParameterOptimizer /
//      Preprocessor / Evaluator，并注入默认参数
// 说明：score_threshold 默认 0.75（全量1620帧验证最优），修复了旧版未初始化问题
//      （未初始化阈值曾导致同一配置多次运行结果随机）。
// =====================================================================
CloudOperations::CloudOperations(const ParamSource& params) {
    // =============== 参数集中读取 =============== //
    config_.load(params);
    switches_.load(params);
    config_.validate();

    // =============== 模块装配 =============== //
    const FilterParameters defaults = config_.to_filter_parameters();

    feature_extractor_.reset(new FeatureExtractor(switches_));
    detector_.reset(new SnowDetector(switches_, *feature_extractor_,
                                     config_.detector_knn,
                                     config_.detector_min_intensity_score,
                                     config_.planarity_filter_threshold,
                                     config_.direct_planarity_gate,
                                     config_.zero_intensity_support_radius,
                                     config_.zero_intensity_support_min_intensity,
                                     config_.zero_intensity_support_range_floor));
    optimizer_.reset(new ParameterOptimizer(switches_, *detector_, defaults));
    preprocessor_.reset(new Preprocessor(switches_,
                                         config_.height_threshold,
                                         config_.xy_threshold,
                                         config_.ror_radius,
                                         config_.ror_neighbors,
                                         config_.filter_downsampling_leaf_size,
                                         config_.lowest_ring_elevation_deg));
    evaluator_.reset(new Evaluator);
    evaluator_->set_strict(switches_.strict_evaluation);
    calibration_.reset(new SensorCalibration(switches_));
    calibration_->set_calib_frames(config_.calib_frames);
    detector_->set_sparse_expand(switches_.enable_sparse_support_expand,
                                 config_.sparse_support_min_neighbors,
                                 config_.sparse_support_max_radius);

    total_timing_.reset();
    processed_frames_ = 0;

    SC_INFO("CloudOperations初始化完成，保存结果:%s，检测器:%s",
             config_.save_results ? "启用" : "禁用", config_.detector_type.c_str());
    SC_INFO("特征权重: 强度=%.2f, 平面度=%.2f, 密度=%.2f, 高度=%.2f",
             config_.intensity_weight, config_.planarity_weight,
             config_.density_weight, config_.height_weight);

    config_.print();
    switches_.print_switches();
}

// 平均耗时打印（微秒累计 -> ms 带小数）。
// 明确区分"算法时间"与"读盘时间"，避免把磁盘性能报成算法性能。
void CloudOperations::print_average_timing(const std::string& prefix) const {
    if (processed_frames_ <= 0) return;
    const double n = static_cast<double>(processed_frames_);
    auto ms = [n](long us) { return static_cast<double>(us) / 1000.0 / n; };
    std::cout << "\n============ " << prefix << "平均处理时间 (" << processed_frames_
              << " 帧) ============" << std::endl;
    std::cout << "平均读盘+去NaN(不计入算法): " << ms(total_timing_.io_time) << " ms" << std::endl;
    std::cout << "平均预处理时间: " << ms(total_timing_.preprocess_time) << " ms" << std::endl;
    std::cout << "平均参数优化时间: " << ms(total_timing_.param_optimize_time) << " ms" << std::endl;
    std::cout << "平均特征融合滤波时间: " << ms(total_timing_.filtering_time) << " ms" << std::endl;
    std::cout << "平均评估时间: " << ms(total_timing_.evaluation_time) << " ms" << std::endl;
    std::cout << "平均算法总时间(不含IO/评估): "
              << ms(total_timing_.preprocess_time + total_timing_.param_optimize_time +
                    total_timing_.filtering_time) << " ms" << std::endl;
    std::cout << "平均端到端(含IO+评估): " << ms(total_timing_.total_time) << " ms" << std::endl;
    std::cout << "======================================" << std::endl;
}

// =============== 传感器自标定 =============== //
// 每帧调用：在标定窗口内累积观测；窗口满后把标定结果一次性应用到各模块。
// 目的：把原实现里四个平台相关的绝对常数换成无标签自标定量。
// 实测依据（docs/AUDIT_REPORT.md 附录 A）：
//   CADC(VLP-32C, intensity 归一化 0..1)：无点满足 I>1.0 -> 支撑点集空
//     -> 表面抑制整体失效 -> 90.16% 的 ROI 点被判成雪
//   安装高度 +0.9 m（虚拟平台，保留标签）：F1 92.92 -> 68.56 (-24.36 pp)
void CloudOperations::update_calibration(const CloudPtr& raw_cloud) {
    if (!calibration_ || !raw_cloud || raw_cloud->empty()) return;

    if (!calibration_->ready()) {
        calibration_->observe(raw_cloud);
    }
    if (!calibration_->ready() || calibration_applied_) return;

    // 1) 强度量纲：δ_I 取代硬编码的支撑点强度下界。
    //    WADS 全量 1620 帧实测 δ_I 恒为 1.0，与硬编码值相同 -> 输出不变。
    if (switches_.enable_intensity_scale_autocal) {
        const float step = calibration_->intensity_step(
            config_.zero_intensity_support_min_intensity);
        detector_->set_support_min_intensity(step);
    }

    // 2) 安装高度导出 z / 仰角门限（会改变数值，默认关闭）
    if (switches_.enable_sensor_height_roi) {
        const float h_s = calibration_->sensor_height(2.13f);
        preprocessor_->set_height_derived_roi(h_s, config_.roi_clearance,
                                              config_.roi_top_height,
                                              config_.roi_lowest_beam_ground_dist);
    }

    // 3) 孤立率曲线自标定水平距离上下界（会改变数值，默认关闭）
    if (switches_.enable_range_selfcal) {
        float rmin = 0.0f, rmax = 0.0f;
        if (calibration_->range_bounds(rmin, rmax)) {
            preprocessor_->set_range_bounds(rmin, rmax);
        } else {
            SC_WARN("距离自标定失败（该场景 I=0 孤立率曲线无有效峰），沿用原 ROI");
        }
    }

    calibration_->print(config_.zero_intensity_support_min_intensity, 2.13f);
    SC_INFO("自标定已应用: 支撑点强度下界 = %.6f",
             detector_->support_min_intensity());
    calibration_applied_ = true;
}

// =============== 索引映射 =============== //
// 将“处理后点云索引”映射回“原始点云索引”
// - 依据：Preprocessor 在预处理阶段维护的 processed_to_original 表
// - 越界索引会告警并跳过，返回的 PointIndices 与输入一一对应
pcl::PointIndices::Ptr CloudOperations::map_to_original_indices(
    const pcl::PointIndices::Ptr& processed_indices) const {
    pcl::PointIndices::Ptr original_indices(new pcl::PointIndices);
    if (!processed_indices) {
        SC_ERROR("映射到原始索引: 处理后索引为空");
        return original_indices;
    }

    original_indices->indices.reserve(processed_indices->indices.size());

    for (int processed_idx : processed_indices->indices) {
        if (processed_idx >= 0 &&
            processed_idx < static_cast<int>(index_mapping_.processed_to_original.size())) {
            const int canonical = index_mapping_.processed_to_original[processed_idx];
            // 去重模式下展开该 canonical 点对应的全部原始副本；关闭时分组大小为 1
            if (canonical >= 0 &&
                canonical < static_cast<int>(index_mapping_.canonical_group_id.size()) &&
                index_mapping_.canonical_group_id[canonical] >= 0) {
                const int group = index_mapping_.canonical_group_id[canonical];
                if (group >= 0 &&
                    group + 1 < static_cast<int>(index_mapping_.group_start.size())) {
                    const int begin = index_mapping_.group_start[group];
                    const int end = index_mapping_.group_start[group + 1];
                    for (int k = begin; k < end; ++k) {
                        if (k >= 0 &&
                            k < static_cast<int>(index_mapping_.group_members_flat.size())) {
                            original_indices->indices.push_back(
                                index_mapping_.group_members_flat[k]);
                        }
                    }
                    continue;
                }
            }
            // 兜底：映射数据未填充时退化为 canonical 单点
            original_indices->indices.push_back(canonical);
        } else {
            SC_WARN("映射到原始索引: 无效的处理后索引 %d (范围 [0,%lu))",
                     processed_idx, index_mapping_.processed_to_original.size());
        }
    }

    std::sort(original_indices->indices.begin(), original_indices->indices.end());
    original_indices->indices.erase(
        std::unique(original_indices->indices.begin(), original_indices->indices.end()),
        original_indices->indices.end());

    return original_indices;
}

// 将“原始点云索引”映射到“处理后点云索引”
// - 依据：original_to_processed 反向表
// - 被预处理过滤掉（映射为 -1）的原始点不会进入结果
std::vector<int> CloudOperations::map_to_processed_indices(
    const std::vector<int>& original_indices) const {
    std::vector<int> processed_indices;
    processed_indices.reserve(original_indices.size());

    for (int original_idx : original_indices) {
        if (original_idx >= 0 &&
            original_idx < static_cast<int>(index_mapping_.original_to_processed.size())) {
            int canonical = original_idx;
            if (original_idx < static_cast<int>(index_mapping_.original_to_canonical.size())) {
                canonical = index_mapping_.original_to_canonical[original_idx];
            }
            int mapped = (canonical >= 0 &&
                          canonical < static_cast<int>(index_mapping_.original_to_processed.size()))
                             ? index_mapping_.original_to_processed[canonical]
                             : -1;
            if (mapped >= 0) {
                processed_indices.push_back(mapped);
            }
        }
    }

    return processed_indices;
}

// =============== 特征融合检测流水线 =============== //
// 主检测流水线（论文核心方法）：
//   1. 场景特征分析（强度统计/复杂度/密度/噪声比）
//   2. 自适应参数选择（特征推荐 + 可选权重调整）
//   3. 大点云体素降采样（>10 万点），并记录降采样->原始映射
//   4. 计算自适应强度阈值
//   5. SnowDetector 逐点检测（强度门控 + 几何特征融合得分）
//   6. 降采样结果映射回原始点云（可选传播）
//   7. 排序去重、构建“去雪后”输出点云
// 说明：本函数直接消费输入点云（预处理后的点云），结果索引仍处于
//       “预处理坐标系”，由调用方再映射回原始坐标系。
void CloudOperations::snow_filter_feature_fusion(const CloudPtr& input_cloud,
                                                 CloudPtr& output_cloud,
                                                 pcl::PointIndices::Ptr& snow_indices,
                                                 const CloudFeatures& features,
                                                 const FilterParameters& params) {
    if (!input_cloud || input_cloud->empty()) {
        SC_ERROR("特征融合滤波输入点云为空");
        if (!output_cloud) output_cloud.reset(new pcl::PointCloud<CloudPoint>);
        if (!snow_indices) snow_indices.reset(new pcl::PointIndices);
        return;
    }

    if (!output_cloud) output_cloud.reset(new pcl::PointCloud<CloudPoint>);
    if (!snow_indices) snow_indices.reset(new pcl::PointIndices);

    try {
        auto start_time = std::chrono::high_resolution_clock::now();

        // 1. 使用调用方传入的场景特征（已由 process_* 计算，避免重复分析）
        current_cloud_features_ = features;

        // 2. 使用调用方传入的参数（已由 optimizer_->optimize 决定）
        FilterParameters fused_params = params;
        if (switches_.enable_adaptive_parameter_adjustment) {
            optimizer_->adjust_feature_weights_for_scene(fused_params, current_cloud_features_);
        }

        // 3. 大型点云降采样
        CloudPtr processing_cloud;
        std::vector<int> downsampled_to_original;
        bool using_downsampled = false;

        if (switches_.enable_pre_downsampling && input_cloud->size() > 100000) {
            float adaptive_leaf_size = preprocessor_->adaptive_leaf_size(input_cloud->size(),
                                                                          current_cloud_features_);
            processing_cloud = optimizer_->downsample(input_cloud, downsampled_to_original,
                                                      adaptive_leaf_size);
            using_downsampled = true;
        } else {
            processing_cloud = input_cloud;
        }

        // 注意：不在这里无条件建 KD 树。最终配置没有任何 KNN 消费者
        // （needs_knn() 全关），建树纯属浪费；SnowDetector::detect 与
        // map_and_propagate 会在真正需要时按需 ensure_kdtree。

        // 4. 构建强度阈值（全局自适应 或 距离分段）
        RangeIntensityThreshold thresholds;
        if (switches_.use_range_binned_intensity_threshold) {
            thresholds = optimizer_->compute_range_thresholds(processing_cloud, current_cloud_features_);
        } else {
            thresholds.global_threshold = switches_.use_adaptive_intensity_threshold
                ? optimizer_->adaptive_intensity_threshold(current_cloud_features_)
                : static_cast<float>(fused_params.rgor_intensity_threshold);
        }
        if (switches_.use_otsu_intensity_threshold) {
            thresholds.global_threshold =
                optimizer_->compute_otsu_threshold(processing_cloud);
        }
        if (switches_.use_idsor_intensity_threshold) {
            thresholds.use_smooth_range = true;
            thresholds.smooth_scale = fused_params.idsor_scale;
            thresholds.smooth_rho = fused_params.idsor_rho;
            thresholds.smooth_slope = fused_params.idsor_slope;
            thresholds.smooth_r0 = fused_params.idsor_r0;
            thresholds.smooth_k = fused_params.idsor_k;
            thresholds.smooth_theta = fused_params.idsor_theta;
        }
        // α(r) 查表：tgamma(k) 是帧常量却被逐点重算，α 也只依赖 r。
        // 实测 1.140 ms -> 0.179 ms（ROI 5.4 万点/帧，单线程 -O3）。
        if (switches_.use_threshold_lut) {
            thresholds.build_lut();
        }
        SC_INFO("特征融合滤波 - 强度阈值: %.2f (Q1: %.2f, 分段阈值:%s)",
                 thresholds.global_threshold, current_cloud_features_.intensity_q1,
                 switches_.use_range_binned_intensity_threshold ? "开" : "关");

        // 5. 基础检测
        snow_indices->indices.clear();
        int dynamic_block_size = optimizer_->dynamic_block_size(processing_cloud->size(), config_.block_size);
        detector_->detect(processing_cloud, thresholds, fused_params,
                          snow_indices, dynamic_block_size);

        size_t initial_size = snow_indices->indices.size();
        SC_INFO("基础检测完成: %lu 个雪点", initial_size);

        // 6. 降采样映射与传播
        if (using_downsampled && switches_.enable_downsampled_mapping_propagation) {
            size_t before_mapping = snow_indices->indices.size();
            detector_->map_and_propagate(input_cloud, snow_indices, downsampled_to_original,
                                         current_cloud_features_, thresholds);
            SC_INFO("映射到原始点云完成: %lu -> %lu 个雪点",
                     before_mapping, snow_indices->indices.size());
        } else if (using_downsampled) {
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
            SC_INFO("简单映射到原始点云完成: %lu 个雪点", snow_indices->indices.size());
        }

        // 7. 排序去重
        std::sort(snow_indices->indices.begin(), snow_indices->indices.end());
        snow_indices->indices.erase(
            std::unique(snow_indices->indices.begin(), snow_indices->indices.end()),
            snow_indices->indices.end());

        // 8. 构建输出点云
        std::vector<bool> is_snow(input_cloud->size(), false);
        for (int idx : snow_indices->indices) {
            if (idx >= 0 && idx < static_cast<int>(input_cloud->size())) {
                is_snow[idx] = true;
            }
        }

        output_cloud->clear();
        output_cloud->reserve(input_cloud->size() - snow_indices->indices.size());
        for (size_t i = 0; i < input_cloud->size(); ++i) {
            if (!is_snow[i]) {
                output_cloud->push_back(input_cloud->points[i]);
            }
        }
        output_cloud->width = output_cloud->size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;

        auto end_time = std::chrono::high_resolution_clock::now();
        auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time).count();

        SC_INFO("特征融合滤波完成: 识别雪点 %lu 个，过滤后点云大小: %lu, 总耗时: %.2f ms",
                 snow_indices->indices.size(), output_cloud->size(),
                 static_cast<double>(total_duration) / 1000.0);

        float snow_percentage = 100.0f * snow_indices->indices.size() / input_cloud->size();
        SC_INFO("雪点占比: %.2f%%", snow_percentage);
    } catch (const std::exception& e) {
        SC_ERROR("特征融合滤波异常: %s", e.what());
        *output_cloud = *input_cloud;
        snow_indices->indices.clear();
    }
}

// =============== 动态离群点滤波器流水线（DROR / DSOR / SOR / ROR） =============== //
// 论文对比基准：非学习式主流方法
//   - detector_type == "dror": 动态半径离群点移除（Charron et al., 2018，本仓自实现）
//   - detector_type == "dsor": 动态统计离群点移除（Kurup & Bos, 2021，本仓自实现）
//   - detector_type == "sor":  PCL 官方统计离群点移除（Rusu et al., 2008）
//   - detector_type == "ror":  PCL 官方半径离群点移除（Rusu, 2009）
// 与特征融合共用同一套预处理和评估流程，保证对比公平。
// 注意：这些方法只使用几何稀疏性，不利用强度，实测在密雪场景召回率很低。
void CloudOperations::run_dynamic_outlier_filter(const CloudPtr& input_cloud,
                                                 CloudPtr& output_cloud,
                                                 pcl::PointIndices::Ptr& snow_indices) {
    if (!input_cloud || input_cloud->empty()) {
        if (!output_cloud) output_cloud.reset(new pcl::PointCloud<CloudPoint>);
        if (!snow_indices) snow_indices.reset(new pcl::PointIndices);
        *output_cloud = *input_cloud;
        return;
    }

    if (!output_cloud) output_cloud.reset(new pcl::PointCloud<CloudPoint>);
    if (!snow_indices) snow_indices.reset(new pcl::PointIndices);

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<int> outliers;
    if (config_.detector_type == "dror" || config_.detector_type == "dsor") {
        // DROR/DSOR 需要 KD 树
        pcl::KdTreeFLANN<CloudPoint>::Ptr tree(new pcl::KdTreeFLANN<CloudPoint>);
        tree->setInputCloud(input_cloud);
        if (config_.detector_type == "dror") {
            DynamicOutlierFilters::dror(input_cloud, tree, config_.dynamic_params, outliers);
        } else {
            DynamicOutlierFilters::dsor(input_cloud, tree, config_.dynamic_params, outliers);
        }
    } else if (config_.detector_type == "sor") {
        DynamicOutlierFilters::sor(input_cloud, config_.dynamic_params, outliers);
    } else if (config_.detector_type == "ror") {
        DynamicOutlierFilters::ror(input_cloud, config_.dynamic_params, outliers);
    } else {
        SC_ERROR("未知动态滤波器类型: %s", config_.detector_type.c_str());
        *output_cloud = *input_cloud;
        return;
    }

    snow_indices->indices = outliers;

    std::vector<bool> is_snow(input_cloud->size(), false);
    for (int idx : snow_indices->indices) {
        if (idx >= 0 && idx < static_cast<int>(input_cloud->size())) {
            is_snow[idx] = true;
        }
    }

    output_cloud->clear();
    output_cloud->reserve(input_cloud->size() - snow_indices->indices.size());
    for (size_t i = 0; i < input_cloud->size(); ++i) {
        if (!is_snow[i]) {
            output_cloud->push_back(input_cloud->points[i]);
        }
    }
    output_cloud->width = output_cloud->size();
    output_cloud->height = 1;
    output_cloud->is_dense = false;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time).count();

    SC_INFO("%s滤波完成: 识别雪点 %lu 个，过滤后点云大小: %lu, 总耗时: %.2f ms",
             config_.detector_type.c_str(), snow_indices->indices.size(),
             output_cloud->size(), static_cast<double>(total_duration) / 1000.0);
}

// =====================================================================
// 单帧、无文件 I/O 的处理入口
// ---------------------------------------------------------------------
// 顺序：自标定 -> 预处理(映射维护) -> 特征分析 -> 参数优化 -> 检测
//       -> 映射回原始索引
// 与 process_pcd_file / process_multiple_frames 共用同一批模块实例，因此
// 在线（ROS 节点）与离线（CLI / 实验运行器）的检测结果逐位一致。
// =====================================================================
CloudOperations::FrameResult CloudOperations::process_cloud(
        const CloudPtr& input_cloud, const std::vector<int>& ground_truth_original) {
    using clock = std::chrono::high_resolution_clock;
    const auto us = [](clock::time_point a, clock::time_point b) {
        return std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
    };

    FrameResult result;
    if (!input_cloud || input_cloud->empty()) {
        SC_ERROR("处理点云为空");
        return result;
    }

    original_cloud_size_ = input_cloud->size();
    result.original_cloud_size = original_cloud_size_;

    const auto t0 = clock::now();

    // 传感器自标定（标定窗口内）
    update_calibration(input_cloud);

    // 预处理
    ProcessedCloud processed = preprocessor_->run(input_cloud, current_cloud_features_);
    index_mapping_ = std::move(processed.mapping);  // 避免每帧深拷贝 ~1.6MB 索引表
    if (switches_.enable_feature_cache) {
        detector_->clear_caches();
    }
    const auto t1 = clock::now();
    result.preprocess_us = us(t0, t1);

    if (processed.cloud->empty()) {
        SC_ERROR("预处理后点云为空，无法继续处理");
        return result;
    }

    // 真值只在网格搜索路径下被消费；在线调用传空即可。
    std::vector<int> processed_gt_indices;
    if (!ground_truth_original.empty()) {
        processed_gt_indices = map_to_processed_indices(ground_truth_original);
    }

    // 分析点云特征
    current_cloud_features_ = feature_extractor_->analyze(
        processed.stats_cloud ? processed.stats_cloud : processed.cloud);
    const auto t2 = clock::now();
    result.feature_us = us(t1, t2);

    // 优化滤波参数
    FilterParameters params = optimizer_->optimize(
        processed.cloud, processed_gt_indices, current_cloud_features_,
        config_.max_optimization_iterations, config_.max_grid_search_points);
    const auto t3 = clock::now();
    result.optimize_us = us(t2, t3);

    // 检测
    CloudPtr desnowed(new pcl::PointCloud<CloudPoint>);
    pcl::PointIndices::Ptr detected_processed(new pcl::PointIndices);
    if (config_.detector_type == "feature_fusion") {
        snow_filter_feature_fusion(processed.cloud, desnowed, detected_processed,
                                   current_cloud_features_, params);
    } else {
        run_dynamic_outlier_filter(processed.cloud, desnowed, detected_processed);
    }
    const auto t4 = clock::now();
    result.filtering_us = us(t3, t4);

    // 映射回原始索引
    result.snow_indices = map_to_original_indices(detected_processed);
    SC_INFO("索引映射完成: 处理后点云检测到 %lu 个雪点，映射回原始点云得到 %lu 个雪点",
            detected_processed->indices.size(), result.snow_indices->indices.size());

    result.desnowed_cloud = desnowed;
    result.processed_cloud = processed.cloud;
    result.features = current_cloud_features_;
    result.total_us = us(t0, clock::now());
    result.ok = true;
    return result;
}

// =============== 单帧处理 =============== //
// 单帧完整处理流程：
//   加载 PCD -> 去 NaN -> 预处理(映射维护) -> 加载 GT -> 特征分析
//   -> 参数优化 -> 检测(特征融合或DROR/DSOR) -> 映射回原始索引
//   -> 保存/评估 -> 计时与统计输出
// 与多帧模式共享同一批模块，GT 路径由 gt_folder 参数指定。
void CloudOperations::process_pcd_file(const std::string& pcd_file_path) {
    using clock = std::chrono::high_resolution_clock;
    const auto us = [](clock::time_point a, clock::time_point b) {
        return std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
    };

    TimingStats timing;

    const auto io_begin = clock::now();
    CloudPtr input_cloud(new pcl::PointCloud<CloudPoint>);

    if (pcl::io::loadPCDFile<CloudPoint>(pcd_file_path, *input_cloud) == -1) {
        PCL_ERROR("无法读取 PCD 文件\n");
        return;
    }
    SC_INFO("已加载 PCD 文件: %s, 点数: %lu",
            pcd_file_path.c_str(), input_cloud->points.size());

    if (input_cloud->empty()) {
        SC_ERROR("PCD文件包含空点云");
        return;
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);
    SC_INFO("去除NaN后点云大小: %lu", input_cloud->points.size());

    // 读盘与去 NaN 属于 I/O，和单帧/多帧口径保持一致，不计入算法时间。
    timing.io_time = us(io_begin, clock::now());

    original_cloud_size_ = input_cloud->size();

    // 加载地面真实标注（严格口径：空 GT 视为零雪点帧，缺文件才跳过）
    std::string pcd_filename = pcd_file_path.substr(pcd_file_path.find_last_of("/\\") + 1);
    std::string base_filename = pcd_filename.substr(0, pcd_filename.find_last_of('.'));
    std::string gt_file_path = config_.gt_folder.empty() ? ""
                            : config_.gt_folder + "/" + base_filename + ".txt";

    std::vector<int> ground_truth_snow_indices;
    bool has_ground_truth = false;
    const auto gt_begin = clock::now();
    if (!gt_file_path.empty()) {
        size_t gt_fail = 0, gt_neg = 0, gt_dup = 0;
        const Evaluator::GtStatus gt_status = evaluator_->load_ground_truth_ex(
            gt_file_path, ground_truth_snow_indices, &gt_fail, &gt_neg, &gt_dup);
        has_ground_truth = (gt_status != Evaluator::GtStatus::kMissing);
        if (!has_ground_truth) {
            SC_WARN("缺少地面真实标注文件，仅处理不评估: %s", gt_file_path.c_str());
        } else if (gt_status == Evaluator::GtStatus::kEmpty) {
            SC_WARN("GT 文件为空 = 该帧零雪点；严格口径下仍参与评估");
        } else {
            SC_INFO("成功加载地面真值: %lu 个雪点索引", ground_truth_snow_indices.size());
            int max_gt = *std::max_element(ground_truth_snow_indices.begin(),
                                           ground_truth_snow_indices.end());
            if (max_gt >= static_cast<int>(original_cloud_size_)) {
                SC_WARN("警告: 地面真值最大索引 (%d) 超出原始点云大小 (%lu)!",
                        max_gt, original_cloud_size_);
                ground_truth_snow_indices.erase(
                    std::remove_if(ground_truth_snow_indices.begin(),
                                   ground_truth_snow_indices.end(),
                                   [&](int idx) { return idx >= static_cast<int>(original_cloud_size_); }),
                    ground_truth_snow_indices.end());
                SC_INFO("过滤后地面真值: %lu 个雪点索引", ground_truth_snow_indices.size());
            }
        }
    }
    timing.io_time += us(gt_begin, clock::now());

    // 流水线本体：与在线路径完全共用
    const FrameResult frame = process_cloud(input_cloud, ground_truth_snow_indices);
    if (!frame.ok) return;

    timing.preprocess_time = frame.preprocess_us;
    // 历史上这一桶同时装着 GT 读盘、特征分析与参数优化。现在 GT 读盘归入
    // io_time，桶内只剩真正的特征分析与（发布配置下为 no-op 的）参数优化。
    timing.param_optimize_time = frame.feature_us + frame.optimize_us;
    timing.filtering_time = frame.filtering_us;

    // 落盘与评估同属"检测之后的收尾"，计时口径与原实现一致。
    const auto eval_begin = clock::now();

    if (config_.save_results) {
        evaluator_->save_indices_direct(frame.snow_indices, pcd_file_path, config_.output_dir);
    }

    if (has_ground_truth) {
        std::cout << "=== 雪点滤波评估结果 ===" << std::endl;
        evaluator_->evaluate_original(frame.snow_indices,
                                      ground_truth_snow_indices,
                                      original_cloud_size_,
                                      frame.processed_cloud->size());
    }

    const auto eval_end = clock::now();
    timing.evaluation_time = us(eval_begin, eval_end);
    timing.total_time = timing.io_time + frame.total_us + timing.evaluation_time;

    total_timing_.io_time += timing.io_time;
    total_timing_.preprocess_time += timing.preprocess_time;
    total_timing_.param_optimize_time += timing.param_optimize_time;
    total_timing_.filtering_time += timing.filtering_time;
    total_timing_.evaluation_time += timing.evaluation_time;
    total_timing_.total_time += timing.total_time;
    processed_frames_++;

    if (config_.verbose) {
        timing.print();
        std::cout << "\n============ 处理结果统计 ============" << std::endl;
        std::cout << "初始点云: " << input_cloud->size() << " 点" << std::endl;
        std::cout << "预处理后点云: " << frame.processed_cloud->size() << " 点" << std::endl;
        std::cout << "预处理后检测到雪点: " << frame.snow_indices->indices.size() << " 点" << std::endl;
        std::cout << "映射回原始点云雪点: " << frame.snow_indices->indices.size() << " 点" << std::endl;
        std::cout << "滤波后点云: " << frame.desnowed_cloud->size() << " 点" << std::endl;
        if (has_ground_truth) {
            std::cout << "地面真实雪点: " << ground_truth_snow_indices.size() << " 点" << std::endl;
        }
        std::cout << "======================================" << std::endl;
    }
}

// =============== 多帧处理 =============== //
// 批量处理整个文件夹：
//   - 自动按文件名排序，逐帧执行与单帧相同的流程
//   - 前 3 帧若存在 GT 则做参数优化（网格搜索/特征推荐），后续帧用特征推荐
//   - 每 5 帧与结束时输出累计平均指标和平均耗时
// 性能提示：本函数为论文实验主入口，1062 帧约需数分钟（受 per-frame 耗时影响）。
void CloudOperations::process_multiple_frames(const std::string& pcd_folder,
                                              const std::string& result_folder) {
    feature_fusion_metrics_.reset();
    total_timing_.reset();
    processed_frames_ = 0;

    // GT 根目录：优先使用独立 _gt_folder，未设置时沿用 result_folder。
    const std::string& gt_root =
        config_.gt_folder.empty() ? result_folder : config_.gt_folder;

    if (!std::filesystem::exists(pcd_folder)) {
        SC_ERROR("PCD文件夹不存在: %s", pcd_folder.c_str());
        return;
    }

    if (gt_root.empty() || !std::filesystem::exists(gt_root)) {
        SC_ERROR("GT/结果文件夹不存在: %s", gt_root.c_str());
        return;
    }

    std::vector<std::string> pcd_files;
    try {
        // 递归扫描：支持用户按场景分文件夹（如 data/pcd_output/11、17/velodyne/...）
        for (const auto& entry : std::filesystem::recursive_directory_iterator(pcd_folder)) {
            if (entry.path().extension() == ".pcd") {
                pcd_files.push_back(entry.path().string());
            }
        }
    } catch (const std::exception& e) {
        SC_ERROR("读取PCD文件夹异常: %s", e.what());
        return;
    }

    if (pcd_files.empty()) {
        SC_ERROR("PCD文件夹为空: %s", pcd_folder.c_str());
        return;
    }

    std::sort(pcd_files.begin(), pcd_files.end());
    int total_frames = pcd_files.size();
    SC_INFO("找到 %d 个PCD文件，开始处理...", total_frames);

    for (int frame_idx = 0; frame_idx < total_frames; frame_idx++) {
        try {
            const auto& pcd_file = pcd_files[frame_idx];

            std::string pcd_filename = pcd_file.substr(pcd_file.find_last_of("/\\") + 1);
            std::string base_filename = pcd_filename.substr(0, pcd_filename.find_last_of('.'));

            // GT 路径自适应：依次尝试
            //  1) 与 PCD 相对结构一致（result/<id>/velodyne/x.txt）
            //  2) 去掉 velodyne 中间层（result/<id>/x.txt，本数据集实际布局）
            //  3) result/<当前PCD文件夹名>/x.txt
            //  4) result/x.txt（平铺回退）
            std::string gt_file_path;
            const std::filesystem::path pcd_path(pcd_file);
            // 词法级相对路径：不解析符号链接（subset 中的 PCD 是软链）
            const std::filesystem::path rel =
                pcd_path.lexically_relative(std::filesystem::path(pcd_folder));
            std::string save_rel_dir;
            if (!rel.empty() && rel.has_parent_path()) {
                save_rel_dir = rel.parent_path().string();
            }
            if (!rel.empty() && rel.has_parent_path()) {
                gt_file_path = (std::filesystem::path(gt_root) /
                                rel.parent_path() / (base_filename + ".txt")).string();
                if (!std::filesystem::exists(gt_file_path)) {
                    // 去掉 velodyne 等中间层：result/<id>/x.txt
                    std::filesystem::path clean_parent;
                    for (const auto& comp : rel.parent_path()) {
                        const std::string s = comp.string();
                        if (s != "velodyne") {
                            clean_parent /= s;
                        }
                    }
                    gt_file_path = (std::filesystem::path(gt_root) /
                                    clean_parent / (base_filename + ".txt")).string();
                }
            }
            if (gt_file_path.empty() || !std::filesystem::exists(gt_file_path)) {
                const std::string folder_name =
                    std::filesystem::path(pcd_folder).filename().string();
                if (!folder_name.empty()) {
                    gt_file_path = (std::filesystem::path(gt_root) /
                                    folder_name / (base_filename + ".txt")).string();
                }
            }
            if (gt_file_path.empty() || !std::filesystem::exists(gt_file_path)) {
                gt_file_path = (std::filesystem::path(gt_root) /
                                (base_filename + ".txt")).string();
            }

            SC_INFO("\n========== 处理帧 %d/%d: %s ==========",
                     frame_idx + 1, total_frames, pcd_file.c_str());

            TimingStats frame_timing;
            auto io_begin = std::chrono::high_resolution_clock::now();

            CloudPtr input_cloud(new pcl::PointCloud<CloudPoint>);
            if (pcl::io::loadPCDFile<CloudPoint>(pcd_file, *input_cloud) == -1) {
                SC_ERROR("无法读取 PCD 文件: %s", pcd_file.c_str());
                continue;
            }

            if (input_cloud->empty()) {
                SC_ERROR("PCD文件包含空点云: %s", pcd_file.c_str());
                continue;
            }

            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);
            original_cloud_size_ = input_cloud->size();

            // 算法计时从此处开始：读盘与去 NaN 不属于算法开销
            // （原实现把它们计入 preprocess_time，使报告值随缓存状态波动 2 倍）
            auto start_time = std::chrono::high_resolution_clock::now();
            frame_timing.io_time = std::chrono::duration_cast<std::chrono::microseconds>(
                start_time - io_begin).count();

            // 传感器自标定（标定窗口内）
            update_calibration(input_cloud);

            // 预处理
            ProcessedCloud processed = preprocessor_->run(input_cloud, current_cloud_features_);
            index_mapping_ = std::move(processed.mapping);  // 避免每帧深拷贝
            if (switches_.enable_feature_cache) {
                detector_->clear_caches();
            }

            auto preprocess_time = std::chrono::high_resolution_clock::now();
            frame_timing.preprocess_time = std::chrono::duration_cast<std::chrono::microseconds>(
                preprocess_time - start_time).count();

            if (processed.cloud->empty()) {
                SC_ERROR("预处理后点云为空，跳过此帧: %s", pcd_file.c_str());
                continue;
            }

            // 地面真值。严格口径下"文件存在但为空"= 该帧零雪点，应照常评估
            // （否则该帧的 FP 完全不被惩罚）；只有文件缺失才跳过。
            std::vector<int> ground_truth_indices;
            size_t gt_fail = 0, gt_neg = 0, gt_dup = 0;
            const Evaluator::GtStatus gt_status = evaluator_->load_ground_truth_ex(
                gt_file_path, ground_truth_indices, &gt_fail, &gt_neg, &gt_dup);
            feature_fusion_metrics_.gt_parse_failures += gt_fail;
            feature_fusion_metrics_.gt_negatives_removed += gt_neg;
            feature_fusion_metrics_.gt_duplicates_removed += gt_dup;

            bool has_ground_truth = (gt_status != Evaluator::GtStatus::kMissing);
            if (!has_ground_truth) {
                SC_WARN("帧 %d/%d 缺少地面真实标注文件，仅处理不评估",
                         frame_idx + 1, total_frames);
            } else if (gt_status == Evaluator::GtStatus::kEmpty) {
                SC_WARN("帧 %d/%d GT 文件为空 = 该帧零雪点；严格口径下仍参与评估",
                         frame_idx + 1, total_frames);
            } else {
                int max_gt = *std::max_element(ground_truth_indices.begin(),
                                               ground_truth_indices.end());
                if (max_gt >= static_cast<int>(original_cloud_size_)) {
                    SC_WARN("帧 %d/%d 地面真值最大索引 (%d) 超出原始点云大小 (%lu)!",
                             frame_idx + 1, total_frames, max_gt, original_cloud_size_);
                    ground_truth_indices.erase(
                        std::remove_if(ground_truth_indices.begin(),
                                       ground_truth_indices.end(),
                                       [&](int idx) { return idx >= static_cast<int>(original_cloud_size_); }),
                        ground_truth_indices.end());
                }

                if (ground_truth_indices.empty()) {
                    SC_WARN("帧 %d/%d 过滤后地面真值为空（视为零雪点帧）",
                             frame_idx + 1, total_frames);
                }
            }

            // 参数优化用GT
            std::vector<int> processed_gt_indices_for_optimization;
            if (has_ground_truth && !ground_truth_indices.empty()) {
                processed_gt_indices_for_optimization = map_to_processed_indices(ground_truth_indices);

                if (processed_gt_indices_for_optimization.empty()) {
                    SC_WARN("帧 %d/%d: 所有地面真值索引都被预处理过滤掉了，无法用于参数优化!",
                             frame_idx + 1, total_frames);
                } else {
                    SC_INFO("参数优化用: 原始ground truth: %lu 点，映射到处理后点云: %lu 点",
                             ground_truth_indices.size(),
                             processed_gt_indices_for_optimization.size());
                }
            }

            // 分析特征
            current_cloud_features_ = feature_extractor_->analyze(
                processed.stats_cloud ? processed.stats_cloud : processed.cloud);

            // 优化参数
            FilterParameters params;
            if (frame_idx < 3) {
                params = optimizer_->optimize(processed.cloud,
                                              processed_gt_indices_for_optimization,
                                              current_cloud_features_,
                                              config_.max_optimization_iterations,
                                              config_.max_grid_search_points);
            } else {
                params = optimizer_->recommend(current_cloud_features_);
            }

            auto param_optimize_time = std::chrono::high_resolution_clock::now();
            frame_timing.param_optimize_time = std::chrono::duration_cast<std::chrono::microseconds>(
                param_optimize_time - preprocess_time).count();

            // 检测
            CloudPtr fusion_output(new pcl::PointCloud<CloudPoint>);
            pcl::PointIndices::Ptr fusion_indices_processed(new pcl::PointIndices);

            if (config_.detector_type == "feature_fusion") {
                snow_filter_feature_fusion(processed.cloud, fusion_output,
                                           fusion_indices_processed,
                                           current_cloud_features_, params);
            } else {
                run_dynamic_outlier_filter(processed.cloud, fusion_output,
                                           fusion_indices_processed);
            }

            auto snow_filter_time = std::chrono::high_resolution_clock::now();
            frame_timing.filtering_time = std::chrono::duration_cast<std::chrono::microseconds>(
                snow_filter_time - param_optimize_time).count();

            // 映射回原始索引
            pcl::PointIndices::Ptr fusion_indices_original =
                map_to_original_indices(fusion_indices_processed);

            SC_INFO("索引映射完成: 处理后点云检测到 %lu 个雪点，映射回原始点云得到 %lu 个雪点",
                     fusion_indices_processed->indices.size(),
                     fusion_indices_original->indices.size());

            // 评估。传入 evaluated_size = 实际参与判定的点数（ROI 后），
            // 使 accuracy 能按"评估子集"口径给出（全帧口径含 73% 从未被检测的点，虚高）。
            if (has_ground_truth) {
                std::cout << "\n=== 帧 " << (frame_idx + 1) << " 特征融合滤波评估结果 ===" << std::endl;
                evaluator_->evaluate_original(fusion_indices_original,
                                              ground_truth_indices,
                                              original_cloud_size_,
                                              processed.cloud->size());
                evaluator_->accumulate_original(fusion_indices_original,
                                                ground_truth_indices,
                                                original_cloud_size_,
                                                feature_fusion_metrics_,
                                                processed.cloud->size());
            }

            auto evaluation_time = std::chrono::high_resolution_clock::now();
            frame_timing.evaluation_time = std::chrono::duration_cast<std::chrono::microseconds>(
                evaluation_time - snow_filter_time).count();

            if (config_.save_results) {
                evaluator_->save_indices_direct(fusion_indices_original,
                                                pcd_file, config_.output_dir,
                                                save_rel_dir);
            }

            frame_timing.total_time = std::chrono::duration_cast<std::chrono::microseconds>(
                evaluation_time - start_time).count();

            total_timing_.io_time += frame_timing.io_time;
            total_timing_.preprocess_time += frame_timing.preprocess_time;
            total_timing_.param_optimize_time += frame_timing.param_optimize_time;
            total_timing_.filtering_time += frame_timing.filtering_time;
            total_timing_.evaluation_time += frame_timing.evaluation_time;
            total_timing_.total_time += frame_timing.total_time;
            processed_frames_++;

            if (config_.verbose) {
                const std::string prefix = "帧 " + std::to_string(frame_idx + 1);
                frame_timing.print(prefix);

                std::cout << "\n============ 帧 " << (frame_idx + 1) << " 处理结果统计 ============" << std::endl;
                std::cout << "初始点云: " << input_cloud->size() << " 点" << std::endl;
                std::cout << "预处理后点云: " << processed.cloud->size() << " 点" << std::endl;
                std::cout << "预处理后检测到雪点: " << fusion_indices_processed->indices.size() << " 点" << std::endl;
                std::cout << "映射回原始点云雪点: " << fusion_indices_original->indices.size() << " 点" << std::endl;
                std::cout << "滤波后点云: " << fusion_output->size() << " 点" << std::endl;
                if (has_ground_truth) {
                    std::cout << "地面真实雪点: " << ground_truth_indices.size() << " 点" << std::endl;
                }
                std::cout << "======================================" << std::endl;
            }

            if ((frame_idx + 1) % 5 == 0 || frame_idx == total_frames - 1) {
                std::cout << "\n处理进度: " << (frame_idx + 1) << "/" << total_frames << " 帧" << std::endl;

                if (feature_fusion_metrics_.total_frames > 0) {
                    std::cout << "=== 特征融合滤波平均指标 (累计"
                              << feature_fusion_metrics_.total_frames << "帧) ===" << std::endl;
                    feature_fusion_metrics_.print_average();
                }

                if (processed_frames_ > 0) {
                    print_average_timing();
                }
            }
        } catch (const std::exception& e) {
            SC_ERROR("处理帧 %s 异常: %s", pcd_files[frame_idx].c_str(), e.what());
            continue;
        }
    }

    SC_INFO("\n\n========== 所有帧处理完毕，总帧数: %d ==========", processed_frames_);

    if (processed_frames_ > 0) {
        print_average_timing("最终");

        if (feature_fusion_metrics_.total_frames > 0) {
            std::cout << "\n====== 最终评估结果 ======" << std::endl;
            feature_fusion_metrics_.print_average();
        }
    }
}

}  // namespace snowclear