#include "snowclear/evaluator.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "snowclear/logging.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unordered_set>

namespace snowclear {

// =============== EvaluationMetrics 实现 =============== //
void EvaluationMetrics::add(double precision, double recall, double f1_score, double f2_score,
                           double accuracy, size_t tp, size_t fp, size_t fn, size_t tn) {
    total_frames++;
    total_precision += precision;
    total_recall += recall;
    total_f1_score += f1_score;
    total_f2_score += f2_score;
    total_accuracy += accuracy;
    total_tp += tp;
    total_fp += fp;
    total_fn += fn;
    total_tn += tn;
}

void EvaluationMetrics::print_average() const {
    if (total_frames == 0) return;

    const double mean_p = total_precision / total_frames;
    const double mean_r = total_recall / total_frames;
    const double f1_from_means = (mean_p + mean_r > 0)
        ? 2 * mean_p * mean_r / (mean_p + mean_r) : 0.0;
    // 严格口径：pooled 分母为 0 时报 0，不再报 1.0（会显示成 "100%"）
    const double pooled_p = (total_tp + total_fp > 0)
        ? static_cast<double>(total_tp) / (total_tp + total_fp) : 0.0;
    const double pooled_r = (total_tp + total_fn > 0)
        ? static_cast<double>(total_tp) / (total_tp + total_fn) : 0.0;
    const double pooled_f1 = (pooled_p + pooled_r > 0)
        ? 2 * pooled_p * pooled_r / (pooled_p + pooled_r) : 0.0;

    std::cout << "\n============ 平均评估指标 (" << total_frames << " 帧) ============" << std::endl;
    std::cout << "平均精确度 (Precision): " << (total_precision / total_frames) * 100 << "%" << std::endl;
    std::cout << "平均召回率 (Recall): " << (total_recall / total_frames) * 100 << "%" << std::endl;
    std::cout << "平均F1分数 (F1 Score): " << (total_f1_score / total_frames) * 100 << "%" << std::endl;
    std::cout << "平均F2分数 (F2 Score): " << (total_f2_score / total_frames) * 100 << "%" << std::endl;
    std::cout << "平均准确率 (Accuracy): " << (total_accuracy / total_frames) * 100 << "%" << std::endl;
    std::cout << "总真正例 (TP): " << total_tp << std::endl;
    std::cout << "总假正例 (FP): " << total_fp << std::endl;
    std::cout << "总假负例 (FN): " << total_fn << std::endl;
    std::cout << "总真负例 (TN): " << total_tn << std::endl;
    std::cout << "F1(由平均P/R计算): " << f1_from_means * 100 << "%" << std::endl;
    std::cout << "总体Pooled: P=" << pooled_p * 100 << "%, R=" << pooled_r * 100
              << "%, F1=" << pooled_f1 * 100 << "%" << std::endl;
    if (frames_zero_detection > 0 || frames_zero_gt > 0 || gt_duplicates_removed > 0 ||
        gt_negatives_removed > 0 || gt_parse_failures > 0) {
        std::cout << "--- 数据卫生（严格口径统计） ---" << std::endl;
        std::cout << "零检测帧(已计入平均): " << frames_zero_detection << std::endl;
        std::cout << "零GT帧(已计入平均): " << frames_zero_gt << std::endl;
        std::cout << "GT去重移除: " << gt_duplicates_removed
                  << ", GT负索引/非法行移除: " << gt_negatives_removed
                  << ", GT解析失败行: " << gt_parse_failures << std::endl;
    }
    std::cout << "==================================================" << std::endl;
}

void EvaluationMetrics::reset() {
    total_frames = 0;
    total_precision = 0.0;
    total_recall = 0.0;
    total_f1_score = 0.0;
    total_f2_score = 0.0;
    total_accuracy = 0.0;
    total_tp = 0;
    total_fp = 0;
    total_fn = 0;
    total_tn = 0;
    frames_zero_detection = 0;
    frames_zero_gt = 0;
    gt_duplicates_removed = 0;
    gt_negatives_removed = 0;
    gt_parse_failures = 0;
}

// =============== Evaluator 实现 =============== //

// =====================================================================
// 加载地面真值（GT）
// ---------------------------------------------------------------------
// 严格口径修正：
//   - 用 std::from_chars 要求整字段被完全消费，拒绝 "12a" 这类尾随垃圾
//     （std::stoi 会静默解析成 12，等于悄悄改写 GT）
//   - 丢弃负索引（原实现只过滤 >= size；负值会进入 gt_set 变成虚假 FN，
//     并可能让 tp+fp+fn > N 导致 size_t 下溢、accuracy > 100%）
//   - 去重（原实现检测侧去重、GT 侧不去重 -> 重复项使 FN 重复计数）
//   - 空文件返回 kEmpty 而非失败，使"该帧零雪点"能被正常评估（FP 才会被惩罚）
// =====================================================================
Evaluator::GtStatus Evaluator::load_ground_truth_ex(const std::string& gt_file_path,
                                                    std::vector<int>& ground_truth_indices,
                                                    size_t* parse_failures,
                                                    size_t* negatives_removed,
                                                    size_t* duplicates_removed) const {
    ground_truth_indices.clear();
    size_t failures = 0, negatives = 0, dups = 0;

    std::ifstream file(gt_file_path);
    if (!file.is_open()) {
        SC_WARN("无法打开地面真实标注文件: %s", gt_file_path.c_str());
        if (parse_failures) *parse_failures = 0;
        if (negatives_removed) *negatives_removed = 0;
        if (duplicates_removed) *duplicates_removed = 0;
        return GtStatus::kMissing;
    }

    std::string line;
    while (std::getline(file, line)) {
        while (!line.empty() &&
               (line.back() == '\r' ||
                std::isspace(static_cast<unsigned char>(line.back())))) {
            line.pop_back();
        }
        size_t b = 0;
        while (b < line.size() && std::isspace(static_cast<unsigned char>(line[b]))) ++b;
        if (b >= line.size()) continue;

        size_t e = line.find(',', b);
        if (e == std::string::npos) e = line.size();
        while (e > b && std::isspace(static_cast<unsigned char>(line[e - 1]))) --e;
        if (e <= b) continue;

        int value = 0;
        const char* first = line.data() + b;
        const char* last = line.data() + e;
        auto res = std::from_chars(first, last, value);
        const bool ok = strict_ ? (res.ec == std::errc() && res.ptr == last)
                                : (res.ec == std::errc());
        if (!ok) {
            ++failures;
            SC_WARN("GT 行解析失败(已跳过): '%s'", std::string(first, last).c_str());
            continue;
        }
        if (strict_ && value < 0) {
            ++negatives;
            continue;
        }
        ground_truth_indices.push_back(value);
    }
    file.close();

    if (strict_) {
        const size_t before = ground_truth_indices.size();
        std::sort(ground_truth_indices.begin(), ground_truth_indices.end());
        ground_truth_indices.erase(
            std::unique(ground_truth_indices.begin(), ground_truth_indices.end()),
            ground_truth_indices.end());
        dups = before - ground_truth_indices.size();
    }

    if (parse_failures) *parse_failures = failures;
    if (negatives_removed) *negatives_removed = negatives;
    if (duplicates_removed) *duplicates_removed = dups;

    SC_INFO("已加载地面真实标注文件: %s, 雪点数量: %zu",
             gt_file_path.c_str(), ground_truth_indices.size());
    return ground_truth_indices.empty() ? GtStatus::kEmpty : GtStatus::kOk;
}

bool Evaluator::load_ground_truth(const std::string& gt_file_path,
                                  std::vector<int>& ground_truth_indices) const {
    return load_ground_truth_ex(gt_file_path, ground_truth_indices) == GtStatus::kOk;
}

// =====================================================================
// 混淆矩阵与指标
// ---------------------------------------------------------------------
// - FN 遍历**全帧 GT**：被 ROI 删掉的 GT 雪点全部计入 FN（召回不虚高）
// - accuracy 给两种口径：全帧（含 73% 从未被检测的点，会虚高）
//   与评估子集（只算真正参与判定的点）。严格口径下累积用后者。
// - 零分母返回 0，不返回 1.0
// - tn 计算钳制，避免 size_t 下溢
// =====================================================================
bool Evaluator::compute(const pcl::PointIndices::Ptr& detected,
                        const std::vector<int>& gt,
                        size_t original_cloud_size,
                        size_t evaluated_size,
                        size_t& tp, size_t& fp, size_t& fn, size_t& tn,
                        double& precision, double& recall, double& f1, double& f2,
                        double& accuracy_full, double& accuracy_eval) const {
    if (!detected) return false;

    std::unordered_set<int> detected_set(detected->indices.begin(), detected->indices.end());
    std::unordered_set<int> gt_set(gt.begin(), gt.end());

    tp = fp = fn = tn = 0;
    for (int idx : detected_set) {
        if (gt_set.count(idx)) ++tp; else ++fp;
    }
    for (int idx : gt_set) {
        if (!detected_set.count(idx)) ++fn;
    }

    const size_t positives = tp + fp + fn;
    tn = (original_cloud_size > positives) ? (original_cloud_size - positives) : 0;

    precision = (tp + fp > 0) ? static_cast<double>(tp) / (tp + fp) : 0.0;
    recall = (tp + fn > 0) ? static_cast<double>(tp) / (tp + fn) : 0.0;
    f1 = (precision + recall > 0) ? 2 * precision * recall / (precision + recall) : 0.0;
    f2 = (4 * precision + recall > 0)
             ? 5 * precision * recall / (4 * precision + recall) : 0.0;

    accuracy_full = (original_cloud_size > 0)
        ? static_cast<double>(tp + tn) / original_cloud_size : 0.0;
    if (evaluated_size > 0) {
        // 评估子集准确率：只看检测器真正参与判定的点，分母为 evaluated_size。
        // ROI 外被直接删除的 GT 不计入该口径（它们已经反映在召回率里）。
        const size_t tn_eval = (evaluated_size > (tp + fp)) ? (evaluated_size - tp - fp) : 0;
        accuracy_eval = evaluated_size > 0
            ? static_cast<double>(tp + tn_eval) / evaluated_size : 0.0;
    } else {
        accuracy_eval = accuracy_full;
    }
    return true;
}

void Evaluator::evaluate_original(const pcl::PointIndices::Ptr& detected_snow_indices_original,
                                  const std::vector<int>& ground_truth_snow_indices,
                                  size_t original_cloud_size,
                                  size_t evaluated_size) const {
    if (!detected_snow_indices_original) {
        SC_ERROR("评估时检测结果为空");
        return;
    }
    if (!strict_ && (ground_truth_snow_indices.empty() ||
                     detected_snow_indices_original->indices.empty())) {
        SC_WARN("地面真实标注(%zu)或检测结果(%zu)为空，评估可能不准确",
                 ground_truth_snow_indices.size(),
                 detected_snow_indices_original->indices.size());
    }

    size_t tp, fp, fn, tn;
    double p, r, f1, f2, acc_full, acc_eval;
    if (!compute(detected_snow_indices_original, ground_truth_snow_indices,
                 original_cloud_size, evaluated_size,
                 tp, fp, fn, tn, p, r, f1, f2, acc_full, acc_eval)) {
        return;
    }

    std::cout << "===== 雪点滤波评估结果 (原始索引) =====" << std::endl;
    std::cout << "原始点云大小: " << original_cloud_size << std::endl;
    if (evaluated_size > 0) {
        std::cout << "实际参与判定(ROI后): " << evaluated_size << std::endl;
    }
    std::cout << "检测到的雪点数: " << detected_snow_indices_original->indices.size() << std::endl;
    std::cout << "地面真实雪点数: " << ground_truth_snow_indices.size() << std::endl;
    std::cout << "真正例 (TP): " << tp << std::endl;
    std::cout << "假正例 (FP): " << fp << std::endl;
    std::cout << "假负例 (FN): " << fn << std::endl;
    std::cout << "真负例 (TN, 全帧口径): " << tn << std::endl;
    std::cout << "精确度 (Precision): " << p * 100 << "%" << std::endl;
    std::cout << "召回率 (Recall): " << r * 100 << "%" << std::endl;
    std::cout << "F1分数 (F1 Score): " << f1 * 100 << "%" << std::endl;
    std::cout << "F2分数 (F2 Score): " << f2 * 100 << "%" << std::endl;
    if (evaluated_size > 0) {
        std::cout << "准确率 (评估子集口径): " << acc_eval * 100 << "%" << std::endl;
        std::cout << "准确率 (全帧口径, 含未参与判定的点, 会虚高): "
                  << acc_full * 100 << "%" << std::endl;
    } else {
        std::cout << "准确率 (Accuracy): " << acc_full * 100 << "%" << std::endl;
    }
    std::cout << "========================================" << std::endl;
}

void Evaluator::accumulate_original(const pcl::PointIndices::Ptr& detected_snow_indices_original,
                                    const std::vector<int>& ground_truth_snow_indices,
                                    size_t original_cloud_size,
                                    EvaluationMetrics& metrics,
                                    size_t evaluated_size) const {
    if (!detected_snow_indices_original) {
        SC_ERROR("累积评估时检测结果为空");
        return;
    }

    // 旧口径：检测为空或 GT 为空整帧跳过 -> recall=0 的最差帧被排除在宏平均之外。
    // 严格口径：照常计入（这是真实结果，不该被隐藏）。
    if (!strict_ && (ground_truth_snow_indices.empty() ||
                     detected_snow_indices_original->indices.empty())) {
        SC_WARN("地面真实标注(%zu)或检测结果(%zu)为空，跳过评估",
                 ground_truth_snow_indices.size(),
                 detected_snow_indices_original->indices.size());
        return;
    }

    size_t tp, fp, fn, tn;
    double p, r, f1, f2, acc_full, acc_eval;
    if (!compute(detected_snow_indices_original, ground_truth_snow_indices,
                 original_cloud_size, evaluated_size,
                 tp, fp, fn, tn, p, r, f1, f2, acc_full, acc_eval)) {
        return;
    }

    if (detected_snow_indices_original->indices.empty()) metrics.frames_zero_detection++;
    if (ground_truth_snow_indices.empty()) metrics.frames_zero_gt++;

    metrics.add(p, r, f1, f2, strict_ ? acc_eval : acc_full, tp, fp, fn, tn);
}

// =====================================================================
// 保存雪点索引（`index,110`，与 GT 文件格式一致）
// =====================================================================
void Evaluator::save_indices_direct(const pcl::PointIndices::Ptr& snow_indices,
                                    const std::string& pcd_file_path,
                                    const std::string& output_dir,
                                    const std::string& relative_dir) const {
    if (!snow_indices) {
        SC_ERROR("保存雪点索引时参数为空");
        return;
    }
    if (output_dir.empty()) {
        SC_ERROR("保存雪点索引时 output_dir 为空，拒绝写入");
        return;
    }

    std::string pcd_filename = pcd_file_path.substr(pcd_file_path.find_last_of("/\\") + 1);
    std::string base_filename = pcd_filename.substr(0, pcd_filename.find_last_of('.'));
    if (base_filename.empty()) {
        SC_WARN("无法从 PCD 路径提取文件名，跳过保存: %s", pcd_file_path.c_str());
        return;
    }

    try {
        std::filesystem::path dir_path(output_dir);
        if (!relative_dir.empty()) {
            dir_path /= relative_dir;
        }
        std::filesystem::create_directories(dir_path);

        const std::filesystem::path output_file_path = dir_path / (base_filename + ".txt");
        std::ofstream out_file(output_file_path);
        if (!out_file.is_open()) {
            SC_ERROR("无法打开文件进行写入: %s", output_file_path.c_str());
            return;
        }
        for (const int& index : snow_indices->indices) {
            out_file << index << ",110" << std::endl;
        }
        out_file.close();
        SC_INFO("雪点索引已保存到文件: %s, 包含 %zu 个索引",
                 output_file_path.c_str(), snow_indices->indices.size());
    } catch (const std::exception& e) {
        SC_ERROR("保存雪点索引异常: %s", e.what());
    }
}

}  // namespace snowclear