#ifndef PREPROCESSOR_H
#define PREPROCESSOR_H

#include "snowclear/cloud_operations_types.hpp"

#include <cmath>
#include "snowclear/ablation_switches.hpp"

#include <vector>

namespace snowclear {

// =============== 预处理模块 =============== //
// 职责: 预降采样 -> 高度/距离过滤 -> 离群点过滤 -> 输出点云与索引映射。
// 不持有任何点云缓存状态，所有结果通过 ProcessedCloud 返回。
class Preprocessor {
public:
    Preprocessor(const AblationSwitches& switches,
                 double height_threshold,
                 double xy_threshold,
                 double ror_radius,
                 int ror_neighbors,
                 float default_leaf_size,
                 float lowest_ring_elevation_deg = -23.0f);

    // 主入口：输入原始点云，输出预处理后点云 + 双向索引映射
    ProcessedCloud run(const CloudPtr& input, const CloudFeatures& features);

    // 自适应叶子大小：点数/复杂度/密度驱动，供降采样与检测流水线复用
    float adaptive_leaf_size(size_t point_count, const CloudFeatures& features) const;

    // 可移植性：用自标定的安装高度 h_s 覆盖 ROI 的 z / 仰角门限。
    // 原实现把 z∈[-1.0, 2.6]、elev>=-23° 写成绝对常数，依赖具体安装高度与
    // 光束布局；实测安装高度 +0.9 m 即让 F1 从 92.92 掉到 68.56（-24.36 pp）。
    // clearance/top/lowest_beam_dist 是有物理含义、跨平台可直接给定的量。
    void set_height_derived_roi(float sensor_height, float clearance,
                                float top_height, float lowest_beam_ground_dist) {
        use_height_derived_roi_ = true;
        min_height_override_ = -sensor_height + clearance;
        height_threshold_ = -sensor_height + top_height;
        lowest_ring_elevation_deg_ = static_cast<float>(
            -std::atan(sensor_height / std::max(0.1f, lowest_beam_ground_dist)) *
            180.0 / 3.14159265358979323846);
    }
    // 可移植性：自标定的水平距离上下界（原实现只有硬编码上界 17.0、无下界；
    // 实测 0-2 m 的 I=0 点纯度为 0.00%，加下界是纯收益）
    void set_range_bounds(float r_min, float r_max) {
        xy_min_override_ = r_min;
        xy_threshold_ = r_max;
    }

private:
    // 超大点云预降采样：体素化后按“最低强度最近邻”映射回原始点云
    CloudPtr pre_downsample_large_cloud(const CloudPtr& input,
                                        std::vector<int>& downsampled_to_original,
                                        const CloudFeatures& features);

    // 高度/水平距离过滤（z 与 xy 阈值）
    void filter_by_height_and_distance(const CloudPtr& cloud, std::vector<int>& filtered_indices);

    // 保守离群点过滤：半径邻居计数，低强度点放宽
    void apply_optimized_outlier_removal(const CloudPtr& cloud, std::vector<int>& indices);

    // 由过滤索引构建输出点云 + 双向映射表
    void build_output_cloud(const CloudPtr& input,
                            std::vector<int>& filtered_indices,
                            ProcessedCloud& out);

    // 精确重复点分组：按 x/y/z/intensity 全等分组，维护 canonical 映射
    void build_dedup_groups(const CloudPtr& cloud,
                            std::vector<int>& original_to_canonical,
                            std::vector<int>& canonical_group_id,
                            std::vector<int>& group_members_flat,
                            std::vector<int>& group_start) const;

    // 统一收尾：把 processed->working 索引转为 canonical 原始索引，
    // 重建 original_to_processed，并填充重复点分组映射
    void finalize_index_mapping(ProcessedCloud& out,
                                const CloudPtr& original_cloud,
                                const std::vector<int>& working_to_original) const;

    // 构建统计用点云：去重时按分组展开回全部重复副本，保证直方图/地面网格
    // 与不去重完全一致；去重关闭时直接复用 out.cloud
    void build_stats_cloud(ProcessedCloud& out,
                           const CloudPtr& original_cloud) const;

    // 异常兜底：最宽松几何过滤
    void fallback_simple_filter(const CloudPtr& input, ProcessedCloud& out);

    const AblationSwitches& switches_;
    bool use_height_derived_roi_ = false;
    float min_height_override_ = -1.0f;
    float xy_min_override_ = 0.0f;
    double height_threshold_;
    double xy_threshold_;
    double ror_radius_;
    int ror_neighbors_;
    float default_leaf_size_;
    float lowest_ring_elevation_deg_;
};

}  // namespace snowclear

#endif  // PREPROCESSOR_H
