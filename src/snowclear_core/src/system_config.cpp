#include "snowclear/system_config.hpp"

#include "snowclear/param_registry.hpp"

#include <algorithm>
#include <ostream>
#include <iostream>

namespace snowclear {

// =====================================================================
// 参数读取：全部可调参数的唯一读取入口（默认值与 snowclear/system_config.hpp 一致）
// ---------------------------------------------------------------------
// 注意：默认值必须与 snowclear/system_config.hpp 的成员默认值、launch 文件保持一致，
// 可用 experiment_runner 的 mode:=param_check 自动校验。
// =====================================================================
void SystemConfig::load(const ParamSource& params) {
    // ---- 几何与 ROI ----
    height_threshold = params.get_double("height_threshold", 2.6);
    xy_threshold = params.get_double("xy_threshold", 17.0);
    epsilon = params.get_double("epsilon", 0.7);
    min_points = params.get_int("min_points", 4);
    ratio_threshold = params.get_double("ratio_threshold", 0.75);
    ror_radius = params.get_double("ror_radius", 0.8);
    ror_neighbors = params.get_int("ror_neighbors", 2);
    lowest_ring_elevation_deg = params.get_float("lowest_ring_elevation_deg", -23.0f);

    // ---- 雪点判定 ----
    score_threshold = params.get_float("score_threshold", 0.75f);
    detector_knn = params.get_int("detector_knn", 5);
    detector_min_intensity_score = params.get_float("detector_min_intensity_score", 0.25f);
    planarity_filter_threshold = params.get_float("planarity_filter_threshold", 0.85f);
    direct_planarity_gate = params.get_float("direct_planarity_gate", 1.0f);
    rgor_intensity_threshold = params.get_double("rgor_intensity_threshold", 6.0);
    zero_intensity_support_radius = params.get_float("zero_intensity_support_radius", 0.6f);
    zero_intensity_support_min_intensity = params.get_float("zero_intensity_support_min_intensity", 1.0f);
    zero_intensity_support_range_floor = params.get_float("zero_intensity_support_range_floor", 7.0f);

    // ---- IDSOR ----
    idsor_scale = params.get_float("idsor_scale", 0.8f);
    idsor_rho = params.get_float("idsor_rho", 3.0f);
    idsor_slope = params.get_float("idsor_slope", 0.0f);
    idsor_r0 = params.get_float("idsor_r0", 5.0f);
    idsor_k = params.get_float("idsor_k", 2.15f);
    idsor_theta = params.get_float("idsor_theta", 2.38f);

    // ---- 特征权重 ----
    intensity_weight = params.get_double("intensity_weight", 0.35);
    planarity_weight = params.get_double("planarity_weight", 0.30);
    density_weight = params.get_double("density_weight", 0.20);
    height_weight = params.get_double("height_weight", 0.15);

    // ---- DCOR/RGOR 保留字段 ----
    dcor_distance_factor = params.get_double("dcor_distance_factor", 0.025);
    dcor_min_pts = params.get_int("dcor_min_pts", 4);
    rgor_alpha = params.get_double("rgor_alpha", 0.015);
    rgor_min_variance_ratio = params.get_double("rgor_min_variance_ratio", 0.10);

    // ---- 性能与优化 ----
    max_optimization_iterations = params.get_int("max_optimization_iterations", 15);
    max_grid_search_points = params.get_int("max_grid_search_points", 5000);
    block_size = params.get_int("block_size", 1024);
    filter_downsampling_leaf_size = params.get_float("filter_downsampling_leaf_size", 0.06f);

    // ---- 可移植性（跨传感器自标定，见 snowclear/sensor_calibration.hpp） ----
    roi_clearance = params.get_float("roi_clearance", 1.125f);
    roi_top_height = params.get_float("roi_top_height", 4.725f);
    roi_lowest_beam_ground_dist = params.get_float("roi_lowest_beam_ground_dist", 5.0f);
    calib_frames = params.get_int("calib_frames", 5);
    sparse_support_min_neighbors = params.get_int("sparse_support_min_neighbors", 30);
    sparse_support_max_radius = params.get_float("sparse_support_max_radius", 1.5f);

    // ---- 路径与 I/O ----
    result_folder = params.get_string("result_folder", "");
    gt_folder = params.get_string("gt_folder", result_folder);
    // 默认落盘目录：相对当前工作目录，不再依赖 ROS 包路径解析。
    output_dir = params.get_string("output_dir", "snowclear_output");
    save_results = params.get_bool("save_results", false);
    verbose = params.get_bool("verbose", false);
    detector_type = params.get_string("detector_type", "feature_fusion");

    // ---- 动态滤波器 ----
    dynamic_params.dror_r_min = params.get_double("dror_r_min", 0.5);
    dynamic_params.dror_alpha = params.get_double("dror_alpha", 0.035);
    dynamic_params.dror_min_neighbors = params.get_int("dror_min_neighbors", 2);
    dynamic_params.dsor_knn = params.get_int("dsor_knn", 8);
    dynamic_params.dsor_std_mult = params.get_double("dsor_std_mult", 2.5);
    dynamic_params.dsor_bin_size = params.get_double("dsor_bin_size", 5.0);
    dynamic_params.sor_mean_k = params.get_int("sor_mean_k", 8);
    dynamic_params.sor_std_mult = params.get_double("sor_std_mult", 1.0);
    // 独立的键：原实现让 preprocessing 的 ror_radius 与 ROR baseline 的
    // ror_radius 共用同一个参数名，两者无法分别设置（默认值恰好相同，
    // 所以此前没有暴露）。这里拆开，默认值不变。
    dynamic_params.ror_radius = params.get_double("dynamic_ror_radius", 0.8);
    dynamic_params.ror_min_neighbors = params.get_int("ror_min_neighbors", 2);
}

// =====================================================================
// 参数合法性校验/钳制
// ---------------------------------------------------------------------
// ROS 参数可能来自 launch 或命令行，数值被误传为 0/负数会引发死循环、
// 越界或异常行为。这里统一做一次保守钳制；非法检测器类型回退到 feature_fusion。
// =====================================================================
// =====================================================================
// 参数名 -> 当前值
// ---------------------------------------------------------------------
// 键集由 load() 生成（tools/gen_param_map.py），因此"代码读取的键"与
// "这里报告的键"不可能不一致。param_check 用它把发布用的 YAML 配置与
// 编译期默认值逐项比对。
// =====================================================================
std::map<std::string, std::string> SystemConfig::to_map() const {
    return {
        {"height_threshold", param_value_string(height_threshold)},
        {"xy_threshold", param_value_string(xy_threshold)},
        {"epsilon", param_value_string(epsilon)},
        {"min_points", param_value_string(min_points)},
        {"ratio_threshold", param_value_string(ratio_threshold)},
        {"ror_radius", param_value_string(ror_radius)},
        {"ror_neighbors", param_value_string(ror_neighbors)},
        {"lowest_ring_elevation_deg", param_value_string(lowest_ring_elevation_deg)},
        {"score_threshold", param_value_string(score_threshold)},
        {"detector_knn", param_value_string(detector_knn)},
        {"detector_min_intensity_score", param_value_string(detector_min_intensity_score)},
        {"planarity_filter_threshold", param_value_string(planarity_filter_threshold)},
        {"direct_planarity_gate", param_value_string(direct_planarity_gate)},
        {"rgor_intensity_threshold", param_value_string(rgor_intensity_threshold)},
        {"zero_intensity_support_radius", param_value_string(zero_intensity_support_radius)},
        {"zero_intensity_support_min_intensity", param_value_string(zero_intensity_support_min_intensity)},
        {"zero_intensity_support_range_floor", param_value_string(zero_intensity_support_range_floor)},
        {"idsor_scale", param_value_string(idsor_scale)},
        {"idsor_rho", param_value_string(idsor_rho)},
        {"idsor_slope", param_value_string(idsor_slope)},
        {"idsor_r0", param_value_string(idsor_r0)},
        {"idsor_k", param_value_string(idsor_k)},
        {"idsor_theta", param_value_string(idsor_theta)},
        {"intensity_weight", param_value_string(intensity_weight)},
        {"planarity_weight", param_value_string(planarity_weight)},
        {"density_weight", param_value_string(density_weight)},
        {"height_weight", param_value_string(height_weight)},
        {"dcor_distance_factor", param_value_string(dcor_distance_factor)},
        {"dcor_min_pts", param_value_string(dcor_min_pts)},
        {"rgor_alpha", param_value_string(rgor_alpha)},
        {"rgor_min_variance_ratio", param_value_string(rgor_min_variance_ratio)},
        {"max_optimization_iterations", param_value_string(max_optimization_iterations)},
        {"max_grid_search_points", param_value_string(max_grid_search_points)},
        {"block_size", param_value_string(block_size)},
        {"filter_downsampling_leaf_size", param_value_string(filter_downsampling_leaf_size)},
        {"roi_clearance", param_value_string(roi_clearance)},
        {"roi_top_height", param_value_string(roi_top_height)},
        {"roi_lowest_beam_ground_dist", param_value_string(roi_lowest_beam_ground_dist)},
        {"calib_frames", param_value_string(calib_frames)},
        {"sparse_support_min_neighbors", param_value_string(sparse_support_min_neighbors)},
        {"sparse_support_max_radius", param_value_string(sparse_support_max_radius)},
        {"result_folder", param_value_string(result_folder)},
        {"gt_folder", param_value_string(gt_folder)},
        {"output_dir", param_value_string(output_dir)},
        {"save_results", param_value_string(save_results)},
        {"verbose", param_value_string(verbose)},
        {"detector_type", param_value_string(detector_type)},
        {"dror_r_min", param_value_string(dynamic_params.dror_r_min)},
        {"dror_alpha", param_value_string(dynamic_params.dror_alpha)},
        {"dror_min_neighbors", param_value_string(dynamic_params.dror_min_neighbors)},
        {"dsor_knn", param_value_string(dynamic_params.dsor_knn)},
        {"dsor_std_mult", param_value_string(dynamic_params.dsor_std_mult)},
        {"dsor_bin_size", param_value_string(dynamic_params.dsor_bin_size)},
        {"sor_mean_k", param_value_string(dynamic_params.sor_mean_k)},
        {"sor_std_mult", param_value_string(dynamic_params.sor_std_mult)},
        {"dynamic_ror_radius", param_value_string(dynamic_params.ror_radius)},
        {"ror_min_neighbors", param_value_string(dynamic_params.ror_min_neighbors)},
    };
}

// =====================================================================
// 参数登记表
// ---------------------------------------------------------------------
// 核心读取的每一个参数及其声明类型。由 load() 生成
// （tools/gen_param_map.py），ROS 2 节点据此 declare_parameter，
// 因此 ros2 param 看到的就是算法真正读取的全部键。
// =====================================================================
const std::vector<ParamSpec>& param_registry() {
    static const std::vector<ParamSpec> kRegistry = {
        {"enable_pre_downsampling", ParamType::kBool},
        {"use_adaptive_leaf_size", ParamType::kBool},
        {"enable_height_distance_filter", ParamType::kBool},
        {"use_conservative_filtering", ParamType::kBool},
        {"enable_outlier_removal", ParamType::kBool},
        {"enable_dedup_exact_points", ParamType::kBool},
        {"enable_lowest_ring_exclusion", ParamType::kBool},
        {"enable_grid_search_optimization", ParamType::kBool},
        {"enable_feature_based_recommendation", ParamType::kBool},
        {"enable_adaptive_parameter_adjustment", ParamType::kBool},
        {"use_intensity_histogram_stats", ParamType::kBool},
        {"use_adaptive_intensity_threshold", ParamType::kBool},
        {"enable_planarity_calculation", ParamType::kBool},
        {"enable_planarity_cache", ParamType::kBool},
        {"enable_density_calculation", ParamType::kBool},
        {"enable_relative_height", ParamType::kBool},
        {"enable_height_consistency", ParamType::kBool},
        {"enable_feature_entropy", ParamType::kBool},
        {"enable_scene_complexity", ParamType::kBool},
        {"enable_adaptive_weight_adjustment", ParamType::kBool},
        {"enable_ultra_low_intensity_direct", ParamType::kBool},
        {"enable_isolated_point_handling", ParamType::kBool},
        {"enable_local_threshold_adjustment", ParamType::kBool},
        {"enable_zero_intensity_surface_suppression", ParamType::kBool},
        {"enable_neighbor_intensity_analysis", ParamType::kBool},
        {"enable_intensity_comparison", ParamType::kBool},
        {"enable_planarity_filtering", ParamType::kBool},
        {"enable_height_consistency_check", ParamType::kBool},
        {"enable_downsampled_mapping_propagation", ParamType::kBool},
        {"enable_openmp_parallel", ParamType::kBool},
        {"enable_block_processing", ParamType::kBool},
        {"use_dynamic_block_size", ParamType::kBool},
        {"enable_feature_cache", ParamType::kBool},
        {"enable_kdtree_cache", ParamType::kBool},
        {"enable_sampling_optimization", ParamType::kBool},
        {"enable_complexity_adaptation", ParamType::kBool},
        {"enable_density_adaptation", ParamType::kBool},
        {"enable_intensity_distribution_adaptation", ParamType::kBool},
        {"use_range_binned_intensity_threshold", ParamType::kBool},
        {"use_idsor_intensity_threshold", ParamType::kBool},
        {"enable_vertical_zscore", ParamType::kBool},
        {"use_scattering_term", ParamType::kBool},
        {"use_soft_consistency", ParamType::kBool},
        {"use_otsu_intensity_threshold", ParamType::kBool},
        {"enable_intensity_scale_autocal", ParamType::kBool},
        {"enable_sensor_height_roi", ParamType::kBool},
        {"enable_range_selfcal", ParamType::kBool},
        {"enable_sparse_support_expand", ParamType::kBool},
        {"fix_quantile_sentinel", ParamType::kBool},
        {"use_fast_elevation_gate", ParamType::kBool},
        {"use_threshold_lut", ParamType::kBool},
        {"strict_evaluation", ParamType::kBool},
        {"height_threshold", ParamType::kDouble},
        {"xy_threshold", ParamType::kDouble},
        {"epsilon", ParamType::kDouble},
        {"min_points", ParamType::kInt},
        {"ratio_threshold", ParamType::kDouble},
        {"ror_radius", ParamType::kDouble},
        {"ror_neighbors", ParamType::kInt},
        {"lowest_ring_elevation_deg", ParamType::kDouble},
        {"score_threshold", ParamType::kDouble},
        {"detector_knn", ParamType::kInt},
        {"detector_min_intensity_score", ParamType::kDouble},
        {"planarity_filter_threshold", ParamType::kDouble},
        {"direct_planarity_gate", ParamType::kDouble},
        {"rgor_intensity_threshold", ParamType::kDouble},
        {"zero_intensity_support_radius", ParamType::kDouble},
        {"zero_intensity_support_min_intensity", ParamType::kDouble},
        {"zero_intensity_support_range_floor", ParamType::kDouble},
        {"idsor_scale", ParamType::kDouble},
        {"idsor_rho", ParamType::kDouble},
        {"idsor_slope", ParamType::kDouble},
        {"idsor_r0", ParamType::kDouble},
        {"idsor_k", ParamType::kDouble},
        {"idsor_theta", ParamType::kDouble},
        {"intensity_weight", ParamType::kDouble},
        {"planarity_weight", ParamType::kDouble},
        {"density_weight", ParamType::kDouble},
        {"height_weight", ParamType::kDouble},
        {"dcor_distance_factor", ParamType::kDouble},
        {"dcor_min_pts", ParamType::kInt},
        {"rgor_alpha", ParamType::kDouble},
        {"rgor_min_variance_ratio", ParamType::kDouble},
        {"max_optimization_iterations", ParamType::kInt},
        {"max_grid_search_points", ParamType::kInt},
        {"block_size", ParamType::kInt},
        {"filter_downsampling_leaf_size", ParamType::kDouble},
        {"roi_clearance", ParamType::kDouble},
        {"roi_top_height", ParamType::kDouble},
        {"roi_lowest_beam_ground_dist", ParamType::kDouble},
        {"calib_frames", ParamType::kInt},
        {"sparse_support_min_neighbors", ParamType::kInt},
        {"sparse_support_max_radius", ParamType::kDouble},
        {"result_folder", ParamType::kString},
        {"gt_folder", ParamType::kString},
        {"output_dir", ParamType::kString},
        {"save_results", ParamType::kBool},
        {"verbose", ParamType::kBool},
        {"detector_type", ParamType::kString},
        {"dror_r_min", ParamType::kDouble},
        {"dror_alpha", ParamType::kDouble},
        {"dror_min_neighbors", ParamType::kInt},
        {"dsor_knn", ParamType::kInt},
        {"dsor_std_mult", ParamType::kDouble},
        {"dsor_bin_size", ParamType::kDouble},
        {"sor_mean_k", ParamType::kInt},
        {"sor_std_mult", ParamType::kDouble},
        {"dynamic_ror_radius", ParamType::kDouble},
        {"ror_min_neighbors", ParamType::kInt},    };
    return kRegistry;
}

void SystemConfig::validate() {
    if (detector_type != "feature_fusion" &&
        detector_type != "dror" && detector_type != "dsor" &&
        detector_type != "sor" && detector_type != "ror") {
        SC_WARN("未知 detector_type '%s'，已回退为 feature_fusion",
                 detector_type.c_str());
        detector_type = "feature_fusion";
    }

    block_size = std::max(1, block_size);
    max_optimization_iterations = std::max(0, max_optimization_iterations);
    max_grid_search_points = std::max(1, max_grid_search_points);
    detector_knn = std::max(3, std::min(64, detector_knn));
    detector_min_intensity_score = std::max(0.0f, std::min(1.0f, detector_min_intensity_score));
    score_threshold = std::max(0.0f, std::min(1.0f, score_threshold));
    planarity_filter_threshold = std::max(0.0f, std::min(1.0f, planarity_filter_threshold));
    direct_planarity_gate = std::max(0.0f, std::min(1.0f, direct_planarity_gate));
    height_threshold = std::max(0.1, height_threshold);
    xy_threshold = std::max(0.1, xy_threshold);
    lowest_ring_elevation_deg = std::max(-90.0f, std::min(0.0f, lowest_ring_elevation_deg));

    zero_intensity_support_radius = std::max(0.01f, std::min(10.0f, zero_intensity_support_radius));
    zero_intensity_support_min_intensity = std::max(0.0f, zero_intensity_support_min_intensity);
    zero_intensity_support_range_floor = std::max(0.0f, zero_intensity_support_range_floor);

    ror_radius = std::max(0.01, ror_radius);
    ror_neighbors = std::max(1, ror_neighbors);
    filter_downsampling_leaf_size = std::max(0.01f, filter_downsampling_leaf_size);
    sparse_support_min_neighbors = std::max(1, sparse_support_min_neighbors);
    sparse_support_max_radius = std::max(0.01f, sparse_support_max_radius);

    idsor_scale = std::max(0.01f, idsor_scale);
    idsor_rho = std::max(0.01f, idsor_rho);
    idsor_k = std::max(0.01f, idsor_k);
    idsor_theta = std::max(0.01f, idsor_theta);

    intensity_weight = std::max(0.0, intensity_weight);
    planarity_weight = std::max(0.0, planarity_weight);
    density_weight = std::max(0.0, density_weight);
    height_weight = std::max(0.0, height_weight);
    const double wsum = intensity_weight + planarity_weight + density_weight + height_weight;
    if (wsum <= 0.0) {
        intensity_weight = 0.35;
        planarity_weight = 0.30;
        density_weight = 0.20;
        height_weight = 0.15;
    }

    dynamic_params.dror_r_min = std::max(0.01, dynamic_params.dror_r_min);
    dynamic_params.dror_alpha = std::max(0.0, dynamic_params.dror_alpha);
    dynamic_params.dror_min_neighbors = std::max(1, dynamic_params.dror_min_neighbors);
    dynamic_params.dsor_knn = std::max(2, dynamic_params.dsor_knn);
    dynamic_params.dsor_std_mult = std::max(0.0, dynamic_params.dsor_std_mult);
    dynamic_params.dsor_bin_size = std::max(0.5, dynamic_params.dsor_bin_size);
    dynamic_params.sor_mean_k = std::max(2, dynamic_params.sor_mean_k);
    dynamic_params.sor_std_mult = std::max(0.0, dynamic_params.sor_std_mult);
    dynamic_params.ror_radius = std::max(0.01, dynamic_params.ror_radius);
    dynamic_params.ror_min_neighbors = std::max(1, dynamic_params.ror_min_neighbors);
}

FilterParameters SystemConfig::to_filter_parameters() const {
    FilterParameters p;
    p.dcor_distance_factor = dcor_distance_factor;
    p.dcor_min_pts = dcor_min_pts;
    p.rgor_intensity_threshold = rgor_intensity_threshold;
    p.rgor_alpha = rgor_alpha;
    p.rgor_min_variance_ratio = rgor_min_variance_ratio;
    p.epsilon = epsilon;
    p.min_points = min_points;
    p.ratio_threshold = ratio_threshold;
    p.ror_radius = ror_radius;
    p.ror_neighbors = ror_neighbors;
    p.intensity_weight = intensity_weight;
    p.planarity_weight = planarity_weight;
    p.density_weight = density_weight;
    p.height_weight = height_weight;
    p.score_threshold = score_threshold;
    p.idsor_scale = idsor_scale;
    p.idsor_rho = idsor_rho;
    p.idsor_slope = idsor_slope;
    p.idsor_r0 = idsor_r0;
    p.idsor_k = idsor_k;
    p.idsor_theta = idsor_theta;
    return p;
}

void SystemConfig::print(std::ostream& os) const {
    os << "\n============ 系统参数 ============" << std::endl;
    os << "检测器类型: " << detector_type << std::endl;
    os << "几何: height_threshold=" << height_threshold
              << " xy_threshold=" << xy_threshold
              << " lowest_ring_elevation_deg=" << lowest_ring_elevation_deg << std::endl;
    os << "判定: score_threshold=" << score_threshold
              << " knn=" << detector_knn
              << " min_intensity_score=" << detector_min_intensity_score
              << " planarity_gate=" << planarity_filter_threshold
              << " direct_gate=" << direct_planarity_gate << std::endl;
    os << "零强度表面抑制: radius=" << zero_intensity_support_radius
              << " min_I=" << zero_intensity_support_min_intensity
              << " range_floor=" << zero_intensity_support_range_floor << std::endl;
    os << "IDSOR: scale=" << idsor_scale << " rho=" << idsor_rho
              << " slope=" << idsor_slope << " r0=" << idsor_r0
              << " k=" << idsor_k << " theta=" << idsor_theta << std::endl;
    os << "权重: intensity=" << intensity_weight
              << " planarity=" << planarity_weight
              << " density=" << density_weight
              << " height=" << height_weight << std::endl;
    os << "路径: result=" << result_folder << " gt=" << gt_folder
              << " output=" << output_dir << " save=" << (save_results ? "true" : "false")
              << std::endl;
    os << "================================" << std::endl;
}

}  // namespace snowclear