#ifndef CLOUD_OPERATIONS_H
#define CLOUD_OPERATIONS_H

#include "snowclear/cloud_operations_types.hpp"
#include "snowclear/ablation_switches.hpp"
#include "snowclear/system_config.hpp"
#include "snowclear/preprocessor.hpp"
#include "snowclear/feature_extractor.hpp"
#include "snowclear/snow_detector.hpp"
#include "snowclear/parameter_optimizer.hpp"
#include "snowclear/evaluator.hpp"
#include "snowclear/dynamic_outlier_filters.hpp"
#include "snowclear/sensor_calibration.hpp"

#include "snowclear/logging.hpp"
#include "snowclear/param_source.hpp"
#include <pcl/PointIndices.h>
#include <memory>
#include <string>
#include <vector>

namespace snowclear {

// =============== 主控制器（门面/编排） =============== //
// 职责: 参数装配（SystemConfig + AblationSwitches）、模块装配、
//       单帧/多帧处理流程、计时与汇总。算法全部下沉到各模块。
//
// 可调参数分两处：
//   1. include/snowclear/system_config.hpp —— 数值参数（阈值/权重/ROI/路径等）
//   2. include/snowclear/ablation_switches.hpp —— 消融开关（模块开/关）
// 两者均由 ParamSource 注入：ROS 2 节点传 RosParamSource，离线 CLI 传
// MapParamSource（YAML + 命令行）。默认值即发布配置，与 ROS 1 版逐位一致。
class CloudOperations {
public:
    explicit CloudOperations(const ParamSource& params);
    ~CloudOperations() = default;

    // =============== 单帧结果（无文件 I/O） =============== //
    // 在线调用方（ROS 节点）只需要数据，不需要落盘、不需要真值、不需要
    // 逐帧日志。FrameResult 把这些都摊开，同时给出分阶段微秒耗时，便于
    // 上游决定什么时候打点。
    //
    // 关于计时口径：feature_us 是特征分析，optimize_us 是参数优化。在发布
    // 配置下 optimize_us ≈ 0（两个优化开关都关，optimize() 直接返回默认值），
    // 二者之和才是历史上被笼统称为"参数优化时间"的那个桶。
    struct FrameResult {
        bool ok = false;
        size_t original_cloud_size = 0;
        CloudFeatures features;

        pcl::PointIndices::Ptr snow_indices;   // 原始点云索引空间
        CloudPtr desnowed_cloud;               // 原始点云去掉雪点
        CloudPtr processed_cloud;              // ROI 之后的点云（调试用）

        long preprocess_us = 0;
        long feature_us = 0;
        long optimize_us = 0;
        long filtering_us = 0;
        long total_us = 0;
    };

    // 处理一帧已经读入内存的点云。ground_truth_original 仅在开启网格搜索
    // 时被消费；在线调用留空即可。
    FrameResult process_cloud(const CloudPtr& input_cloud,
                              const std::vector<int>& ground_truth_original = {});

    // 文件入口。内部复用 process_cloud，因此两条路径永远共用同一份流水线。
    void process_pcd_file(const std::string& pcd_file_path);
    void process_multiple_frames(const std::string& pcd_folder, const std::string& result_folder);

    void set_save_results(bool value) { config_.save_results = value; }

    // 实验运行器需要读取汇总结果/耗时（只读访问）
    const EvaluationMetrics& evaluation_metrics() const { return feature_fusion_metrics_; }
    int processed_frames() const { return processed_frames_; }
    double average_algorithm_time_ms() const {
        if (processed_frames_ <= 0) return 0.0;
        return static_cast<double>(total_timing_.preprocess_time +
                                   total_timing_.param_optimize_time +
                                   total_timing_.filtering_time) / 1000.0 / processed_frames_;
    }
    double average_total_time_ms() const {
        if (processed_frames_ <= 0) return 0.0;
        return static_cast<double>(total_timing_.total_time) / 1000.0 / processed_frames_;
    }

private:
    // =============== 检测流水线 =============== //
    void snow_filter_feature_fusion(const CloudPtr& input_cloud,
                                    CloudPtr& output_cloud,
                                    pcl::PointIndices::Ptr& snow_indices,
                                    const CloudFeatures& features,
                                    const FilterParameters& params);

    void run_dynamic_outlier_filter(const CloudPtr& input_cloud,
                                    CloudPtr& output_cloud,
                                    pcl::PointIndices::Ptr& snow_indices);

    // =============== 索引映射 =============== //
    pcl::PointIndices::Ptr map_to_original_indices(const pcl::PointIndices::Ptr& processed_indices) const;
    std::vector<int> map_to_processed_indices(const std::vector<int>& original_indices) const;

    // =============== 配置 =============== //
    SystemConfig config_;            // 全部数值参数（集中管理）
    AblationSwitches switches_;      // 全部消融开关（集中管理）

    // =============== 运行状态 =============== //
    size_t original_cloud_size_ = 0;
    CloudFeatures current_cloud_features_;
    IndexMapping index_mapping_;
    int processed_frames_ = 0;
    EvaluationMetrics feature_fusion_metrics_;
    TimingStats total_timing_;

    // =============== 模块实例 =============== //
    std::unique_ptr<FeatureExtractor> feature_extractor_;
    std::unique_ptr<SnowDetector> detector_;
    std::unique_ptr<ParameterOptimizer> optimizer_;
    std::unique_ptr<Preprocessor> preprocessor_;
    std::unique_ptr<Evaluator> evaluator_;
    std::unique_ptr<SensorCalibration> calibration_;

    // 每帧开头：累积自标定观测，并把标定结果应用到各模块（幂等）
    void update_calibration(const CloudPtr& raw_cloud);
    void print_average_timing(const std::string& prefix = "累计") const;
    bool calibration_applied_ = false;
};

}  // namespace snowclear

#endif  // CLOUD_OPERATIONS_H
