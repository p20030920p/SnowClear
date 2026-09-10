#include "snowclear/sensor_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>

#include "snowclear/logging.hpp"

namespace snowclear {

namespace {

constexpr double kPi = 3.14159265358979323846;

// 仰角直方图分辨率（度）与相邻峰合并阈值（度）
constexpr float kElevBinDeg = 0.05f;
constexpr float kElevMergeDeg = 0.12f;
constexpr float kElevMinDeg = -40.0f;
constexpr float kElevMaxDeg = 30.0f;

inline float elevation_deg(const CloudPoint& p) {
    const float r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (r <= 1e-6f) return 0.0f;
    float s = p.z / r;
    s = std::max(-1.0f, std::min(1.0f, s));
    return static_cast<float>(std::asin(s) * 180.0 / kPi);
}

}  // namespace

SensorCalibration::SensorCalibration(const AblationSwitches& switches)
    : switches_(switches),
      iso_num_(kRangeBins, 0.0),
      iso_den_(kRangeBins, 0.0) {}

// =====================================================================
// 由仰角直方图峰恢复光束表
// ---------------------------------------------------------------------
// PointXYZI 没有 ring 字段，但旋转式激光雷达的仰角是离散的：
// WADS/HDL-64E 实测 63~64 个峰，分环残差中位 0.013 度，可无损恢复。
// =====================================================================
void SensorCalibration::recover_beam_elevations(const CloudPtr& cloud,
                                                std::vector<float>& beams_rad) {
    beams_rad.clear();
    if (!cloud || cloud->size() < 1000) return;

    const int nbins = static_cast<int>((kElevMaxDeg - kElevMinDeg) / kElevBinDeg) + 1;
    std::vector<int> hist(nbins, 0);
    for (const auto& p : cloud->points) {
        const float e = elevation_deg(p);
        if (e < kElevMinDeg || e > kElevMaxDeg) continue;
        const int b = static_cast<int>((e - kElevMinDeg) / kElevBinDeg);
        if (b >= 0 && b < nbins) hist[b]++;
    }

    // 峰阈值：与点数成比例，避免稀疏噪声被当成光束
    const int thr = std::max(5, static_cast<int>(0.0004 * cloud->size()));
    std::vector<float> cand;
    for (int i = 1; i + 1 < nbins; ++i) {
        if (hist[i] > hist[i - 1] && hist[i] >= hist[i + 1] && hist[i] > thr) {
            cand.push_back(kElevMinDeg + (i + 0.5f) * kElevBinDeg);
        }
    }
    if (cand.empty()) return;

    // 合并过近的峰（同一条光束可能跨两个 bin）
    std::vector<float> merged;
    merged.push_back(cand[0]);
    for (size_t i = 1; i < cand.size(); ++i) {
        if (cand[i] - merged.back() > kElevMergeDeg) merged.push_back(cand[i]);
    }

    beams_rad.reserve(merged.size());
    for (float d : merged) beams_rad.push_back(static_cast<float>(d * kPi / 180.0));
}

// =====================================================================
// 用最低 k 条光束的 z 中位数估安装高度
// ---------------------------------------------------------------------
// 为什么不用 "z 直方图众数"：城区场景 4~12 m 环带的 z 众数由建筑/车辆等
// 垂直结构主导。实测 Boreas(多伦多城区) 众数在 +0.22 m，真实地面 -2.0 m，
// 偏 2.5 m。最低光束在几米内必然打到地面，是鲁棒且平台无关的判据。
// =====================================================================
bool SensorCalibration::estimate_height_from_lowest_beams(
    const CloudPtr& cloud, const std::vector<float>& beams_rad,
    int k_rings, float& h_s) {
    if (!cloud || cloud->empty() || beams_rad.empty()) return false;

    const float tol = static_cast<float>(kElevMergeDeg * kPi / 180.0);
    std::vector<float> ring_ground;
    const int kmax = std::min<int>(k_rings, static_cast<int>(beams_rad.size()));

    for (int k = 0; k < kmax; ++k) {
        const float target = beams_rad[k];
        std::vector<float> zs;
        zs.reserve(2048);
        for (const auto& p : cloud->points) {
            const float rh = std::sqrt(p.x * p.x + p.y * p.y);
            if (rh <= 3.0f || rh >= 15.0f) continue;
            const float e = static_cast<float>(elevation_deg(p) * kPi / 180.0);
            if (std::abs(e - target) < tol) zs.push_back(p.z);
        }
        if (zs.size() > 50) {
            std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
            ring_ground.push_back(zs[zs.size() / 2]);
        }
    }
    if (ring_ground.empty()) return false;

    std::nth_element(ring_ground.begin(),
                     ring_ground.begin() + ring_ground.size() / 2,
                     ring_ground.end());
    h_s = -ring_ground[ring_ground.size() / 2];
    return true;
}

// =====================================================================
// 累积 I=0 点的"孤立率"曲线（无标签）
// ---------------------------------------------------------------------
// 孤立率 = I=0 点中"半径内不存在明亮点(I > δ_I)"的比例。
//   近处：I=0 是悬浮雪，找不到支撑 -> 孤立率高
//   远处：I=0 是真实表面上的丢点   -> 孤立率低
// 实测与真纯度 Pearson r = 0.888 (p = 6e-11)，故可替代标签定距离上下界。
// 用空间哈希做半径查询，避免建 KD 树。
// =====================================================================
void SensorCalibration::accumulate_isolation(const CloudPtr& cloud,
                                             float i_step, float h_s) {
    if (!cloud || cloud->empty()) return;

    const float radius = 0.6f;      // 雪团物理尺度（非密度导出量，见报告 A.6）
    const float cell = radius;
    const float zlo = -h_s + 1.125f;
    const float zhi = -h_s + 4.725f;

    // 明亮点空间哈希
    std::unordered_map<int64_t, std::vector<int>> grid;
    grid.reserve(cloud->size() / 4 + 1);
    auto key_of = [cell](float x, float y, float z) {
        const int64_t cx = static_cast<int64_t>(std::floor(x / cell));
        const int64_t cy = static_cast<int64_t>(std::floor(y / cell));
        const int64_t cz = static_cast<int64_t>(std::floor(z / cell));
        // 21 位/轴 有符号打包，量程 ±(2^20)*cell，远超任何车载雷达
        return ((cx & 0x1FFFFF) << 42) | ((cy & 0x1FFFFF) << 21) | (cz & 0x1FFFFF);
    };

    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& p = cloud->points[i];
        if (p.z < zlo || p.z > zhi) continue;
        if (p.intensity > i_step) {
            grid[key_of(p.x, p.y, p.z)].push_back(static_cast<int>(i));
        }
    }
    if (grid.empty()) return;

    const float r2 = radius * radius;
    const float zero_bin_threshold = 0.5f * i_step;
    for (const auto& p : cloud->points) {
        if (p.z < zlo || p.z > zhi) continue;
        if (p.intensity > zero_bin_threshold) continue;
        const float rh = std::sqrt(p.x * p.x + p.y * p.y);
        const int b = static_cast<int>(rh / kBinWidth);
        if (b < 0 || b >= kRangeBins) continue;

        bool supported = false;
        const int64_t cx = static_cast<int64_t>(std::floor(p.x / cell));
        const int64_t cy = static_cast<int64_t>(std::floor(p.y / cell));
        const int64_t cz = static_cast<int64_t>(std::floor(p.z / cell));
        for (int dx = -1; dx <= 1 && !supported; ++dx) {
            for (int dy = -1; dy <= 1 && !supported; ++dy) {
                for (int dz = -1; dz <= 1 && !supported; ++dz) {
                    const int64_t k = (((cx + dx) & 0x1FFFFF) << 42) |
                                      (((cy + dy) & 0x1FFFFF) << 21) |
                                      ((cz + dz) & 0x1FFFFF);
                    auto it = grid.find(k);
                    if (it == grid.end()) continue;
                    for (int idx : it->second) {
                        const auto& o = cloud->points[idx];
                        const float ddx = p.x - o.x, ddy = p.y - o.y, ddz = p.z - o.z;
                        if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                            supported = true;
                            break;
                        }
                    }
                }
            }
        }
        iso_den_[b] += 1.0;
        if (!supported) iso_num_[b] += 1.0;
    }
}

// =====================================================================
// 标定观测入口
// =====================================================================
void SensorCalibration::observe(const CloudPtr& cloud) {
    if (ready() || !cloud || cloud->empty()) return;

    // 1) 强度量化步长 = 最小正强度（跨帧取最小，冻结）
    float mn = std::numeric_limits<float>::max();
    for (const auto& p : cloud->points) {
        if (p.intensity > 0.0f && p.intensity < mn) mn = p.intensity;
    }
    if (mn < std::numeric_limits<float>::max()) {
        if (min_positive_intensity_ < 0.0f || mn < min_positive_intensity_) {
            min_positive_intensity_ = mn;
        }
    }

    // 2) 光束表：用“窗口内点数最多的一帧”恢复，避免首帧退化导致标定被锁死。
    if (beam_elev_.empty() || cloud->size() > beam_source_size_) {
        std::vector<float> candidate;
        recover_beam_elevations(cloud, candidate);
        if (!candidate.empty()) {
            beam_elev_.swap(candidate);
            beam_source_size_ = cloud->size();
        }
    }

    // 3) 安装高度
    float h = 0.0f;
    if (!beam_elev_.empty() &&
        estimate_height_from_lowest_beams(cloud, beam_elev_, 3, h)) {
        if (h > 0.3f && h < 5.0f) height_samples_.push_back(h);
    }

    // 4) 孤立率曲线（需要 δ_I 与 h_s，用当前最佳估计）
    if (switches_.enable_range_selfcal && min_positive_intensity_ > 0.0f &&
        !height_samples_.empty()) {
        std::vector<float> hs = height_samples_;
        std::nth_element(hs.begin(), hs.begin() + hs.size() / 2, hs.end());
        accumulate_isolation(cloud, min_positive_intensity_, hs[hs.size() / 2]);
    }

    frames_observed_++;
}

float SensorCalibration::intensity_step(float fallback) const {
    return (min_positive_intensity_ > 0.0f) ? min_positive_intensity_ : fallback;
}

float SensorCalibration::sensor_height(float fallback) const {
    if (height_samples_.empty()) return fallback;
    std::vector<float> hs = height_samples_;
    std::nth_element(hs.begin(), hs.begin() + hs.size() / 2, hs.end());
    return hs[hs.size() / 2];
}

// =====================================================================
// 由孤立率曲线定距离上下界
// ---------------------------------------------------------------------
// 从 **I=0 质量峰** 向两侧走，孤立率跌破阈值处即为界。
// 锚点必须用点数峰而非孤立率峰：传感器近场盲区是空的，
// 那里任何 I=0 点都找不到支撑邻点，孤立率退化性接近 1，会把锚点吸到 r≈0。
// =====================================================================
bool SensorCalibration::range_bounds(float& r_min, float& r_max) const {
    const double min_pts = 200.0;
    const double iso_th = 0.25;

    int peak = -1;
    double peak_den = 0.0;
    for (int i = 0; i < kRangeBins; ++i) {
        if (iso_den_[i] >= min_pts && iso_den_[i] > peak_den) {
            peak_den = iso_den_[i];
            peak = i;
        }
    }
    if (peak < 0) return false;

    r_min = 0.0f;
    for (int i = peak; i >= 0; --i) {
        const bool weak = iso_den_[i] < min_pts ||
                          (iso_num_[i] / iso_den_[i]) < iso_th;
        if (weak) {
            r_min = (i + 1) * kBinWidth;
            break;
        }
    }
    r_max = kRangeBins * kBinWidth;
    for (int i = peak; i < kRangeBins; ++i) {
        const bool weak = iso_den_[i] < min_pts ||
                          (iso_num_[i] / iso_den_[i]) < iso_th;
        if (weak) {
            r_max = i * kBinWidth;
            break;
        }
    }
    return r_max > r_min;
}

void SensorCalibration::print(float i_step_fallback, float h_fallback) const {
    std::cout << "\n============ 传感器自标定 ============" << std::endl;
    std::cout << "标定帧数: " << frames_observed_ << "/" << calib_frames_ << std::endl;
    std::cout << "恢复光束数: " << beam_elev_.size();
    if (!beam_elev_.empty()) {
        std::cout << " (仰角 " << beam_elev_.front() * 180.0 / kPi << " .. "
                  << beam_elev_.back() * 180.0 / kPi << " 度)";
    }
    std::cout << std::endl;
    std::cout << "强度量化步长 delta_I: " << intensity_step(i_step_fallback)
              << (min_positive_intensity_ > 0.0f ? " (自标定)" : " (回退)") << std::endl;
    std::cout << "安装高度 h_s: " << sensor_height(h_fallback) << " m"
              << (height_samples_.empty() ? " (回退)" : " (自标定)") << std::endl;
    float a = 0.0f, b = 0.0f;
    if (range_bounds(a, b)) {
        std::cout << "距离自标定: r_min=" << a << " m, r_max=" << b << " m" << std::endl;
    } else {
        std::cout << "距离自标定: 未启用或数据不足" << std::endl;
    }
    std::cout << "======================================" << std::endl;
}

}  // namespace snowclear