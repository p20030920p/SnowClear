#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include "snowclear/cloud_operations_types.hpp"
#include "snowclear/ablation_switches.hpp"

#include <cstdint>
#include <vector>

namespace snowclear {

// =====================================================================
// 传感器自标定模块（可移植性核心）
// ---------------------------------------------------------------------
// 动机：原实现把四个平台相关量写成绝对常数，换传感器即失效（实测）：
//   CADC (VLP-32C, intensity 归一化到 0..1)：没有任何点满足 I > 1.0
//     -> 支撑点集为空 -> 零强度表面抑制整体失效 -> 90.16% 的 ROI 点被判成雪
//   安装高度 +0.9 m（虚拟平台，保留标签）：F1 92.92 -> 68.56 (-24.36 pp)
//
// 本模块用**无标签**数据标定出等价的平台无关量：
//   intensity_step (δ_I)  = 最小正强度 = 一个量化步长
//                           语义"有任何可探测回波"；WADS=1.0, CADC=0.003922
//   sensor_height  (h_s)  = 最低几条扫描光束的 z 中位数取负
//                           （不能用 z 直方图众数：城区场景被建筑/车辆带偏，
//                             实测 Boreas 偏 2.5 m）
//   beam_angles           = 仰角直方图峰，用于定位最低光束（无需 datasheet）
//   range bounds          = 由 I=0 点"孤立率"曲线自标定
//                           （孤立率与真纯度 Pearson r = 0.888, p = 6e-11）
//
// 全部标定量在前 N 帧累积后**冻结**，避免逐帧抖动（h_s 逐帧 std 0.17 m）。
// =====================================================================
class SensorCalibration {
public:
    explicit SensorCalibration(const AblationSwitches& switches);

    // 累积一帧标定观测；返回是否仍在标定窗口内
    void observe(const CloudPtr& cloud);

    // 标定窗口是否已满（满后各 getter 返回冻结值）
    bool ready() const { return frames_observed_ >= calib_frames_; }

    void set_calib_frames(int n) { calib_frames_ = (n > 0 ? n : 1); }

    // ---- 标定结果 ----
    // 强度量化步长；未标定成功时返回 fallback
    float intensity_step(float fallback) const;
    // 安装高度（米）；未标定成功时返回 fallback
    float sensor_height(float fallback) const;
    // 恢复出的光束仰角表（弧度，升序）
    const std::vector<float>& beam_elevations() const { return beam_elev_; }

    // 孤立率曲线自标定的水平距离上下界；返回 false 表示无法标定
    bool range_bounds(float& r_min, float& r_max) const;

    // 诊断打印
    void print(float i_step_fallback, float h_fallback) const;

    // 供预处理/检测复用：静态工具
    // 由仰角直方图峰恢复光束表（弧度，升序）
    static void recover_beam_elevations(const CloudPtr& cloud,
                                        std::vector<float>& beams_rad);

private:
    // 用最低 k 条光束的 z 中位数估地面
    static bool estimate_height_from_lowest_beams(const CloudPtr& cloud,
                                                  const std::vector<float>& beams_rad,
                                                  int k_rings, float& h_s);
    // 累积 I=0 点的孤立率直方图（分距离段）
    void accumulate_isolation(const CloudPtr& cloud, float i_step, float h_s);

    const AblationSwitches& switches_;
    int calib_frames_ = 5;
    int frames_observed_ = 0;

    // 强度量化步长：取标定窗口内的最小正强度
    float min_positive_intensity_ = -1.0f;

    // 安装高度：标定窗口内各帧估计值（最后取中位数）
    std::vector<float> height_samples_;

    std::vector<float> beam_elev_;  // 弧度，升序
    size_t beam_source_size_ = 0;   // 用于选择点数最多的帧恢复光束表

    // 孤立率曲线：1 m 分箱
    static constexpr int kRangeBins = 40;      // 0..40 m
    static constexpr float kBinWidth = 1.0f;
    std::vector<double> iso_num_;  // I=0 且无明亮支撑点
    std::vector<double> iso_den_;  // I=0 总数
};

}  // namespace snowclear

#endif  // SENSOR_CALIBRATION_H
