#ifndef EVALUATOR_H
#define EVALUATOR_H

#include "snowclear/cloud_operations_types.hpp"

#include <pcl/PointIndices.h>
#include <string>
#include <vector>

namespace snowclear {

// =============== 评估与 I/O 模块 =============== //
// 职责: 地面真值加载、指标计算（原始索引）、雪点索引保存。
//
// 【严格评估口径 strict_evaluation（默认开启）】
// 原实现有五处会让指标偏乐观或失真的缺陷，逐一修正：
//   1. accumulate_original 在"检测结果为空"时直接 return，不计入 metrics。
//      调用方已保证 GT 非空，故唯一可达分支就是零检测帧——即 recall=0 的最差帧
//      被系统性排除在宏平均之外。（同一帧 evaluate_original 却会打印 recall=0，
//      两处数字互相矛盾。）
//   2. load_ground_truth 对空 GT 文件返回 false -> 被当成"无标注"跳过。
//      语义上空文件应是"该帧零雪点"，此时检测器的 FP 完全不被惩罚。
//   3. TN = original_cloud_size - (tp+fp+fn) 用**全帧**点数，而检测只在 ROI
//      内进行（实测 ROI 仅保留 26.97% 的点）-> 73% 从未被检测的点被算作 TN
//      -> accuracy 无意义地虚高。现同时报"评估子集"口径。
//   4. 零分母返回 1.0 -> 零检测帧打印 "Precision 100%"。现返回 0。
//   5. GT 未去重、未过滤负索引 -> FN 虚高；且 tn 为 size_t，
//      tp+fp+fn > N 时会下溢成天文数字、accuracy > 100%。现做清洗与钳制。
// 关闭 strict_evaluation 可复现旧口径数字，便于对照。
class Evaluator {
public:
    Evaluator() = default;

    void set_strict(bool v) { strict_ = v; }
    bool strict() const { return strict_; }

    // GT 文件加载状态
    enum class GtStatus {
        kMissing,   // 打不开（文件不存在/无权限）
        kEmpty,     // 文件可读但没有有效索引 = 该帧零雪点（严格口径下仍应评估）
        kOk         // 有有效索引
    };

    // 加载 GT：支持 `index` 与 `index,110` 两种格式。
    // 严格口径下会：拒绝尾随垃圾、丢弃负索引、去重、统计解析失败数。
    GtStatus load_ground_truth_ex(const std::string& gt_file_path,
                                  std::vector<int>& ground_truth_indices,
                                  size_t* parse_failures = nullptr,
                                  size_t* negatives_removed = nullptr,
                                  size_t* duplicates_removed = nullptr) const;

    // 兼容旧接口：返回 status == kOk
    bool load_ground_truth(const std::string& gt_file_path,
                           std::vector<int>& ground_truth_indices) const;

    // 单帧评估（原始索引坐标系）。evaluated_size = 实际参与检测的点数（ROI 后），
    // 传 0 表示未知，此时只报全帧口径。
    void evaluate_original(const pcl::PointIndices::Ptr& detected_snow_indices_original,
                           const std::vector<int>& ground_truth_snow_indices,
                           size_t original_cloud_size,
                           size_t evaluated_size = 0) const;

    // 累积评估（不打印，累加进 metrics）
    void accumulate_original(const pcl::PointIndices::Ptr& detected_snow_indices_original,
                             const std::vector<int>& ground_truth_snow_indices,
                             size_t original_cloud_size,
                             EvaluationMetrics& metrics,
                             size_t evaluated_size = 0) const;

    // 保存原始索引（`index,110` 格式，output_dir 由调用方指定）。
    // relative_dir 非空时保存到 output_dir/relative_dir/ 下，避免多场景同名文件覆盖。
    void save_indices_direct(const pcl::PointIndices::Ptr& snow_indices,
                             const std::string& pcd_file_path,
                             const std::string& output_dir,
                             const std::string& relative_dir = "") const;

private:
    // 计算一帧的混淆矩阵与指标；返回是否成功
    bool compute(const pcl::PointIndices::Ptr& detected,
                 const std::vector<int>& gt,
                 size_t original_cloud_size,
                 size_t evaluated_size,
                 size_t& tp, size_t& fp, size_t& fn, size_t& tn,
                 double& precision, double& recall, double& f1, double& f2,
                 double& accuracy_full, double& accuracy_eval) const;

    bool strict_ = true;
};

}  // namespace snowclear

#endif  // EVALUATOR_H
