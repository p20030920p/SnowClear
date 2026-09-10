// =====================================================================
// snowclear_runner — experiment driver and quality gate
// ---------------------------------------------------------------------
// Modes
//   param_check     the shipped YAML config vs the compiled-in defaults;
//                   also reports YAML keys nothing reads (dead parameters)
//   regression      process one frame, save the snow indices, compare
//                   byte-for-byte against a reference file
//   reproduce_full  every scan under a root, one CSV row
//   eval_folders    one CSV row per scene directory
//   all_checks      param_check then regression — the gate to run before
//                   quoting any number
//
// Everything is driven from the same key:=value parameter convention as
// snowclear_cli, so a configuration is written down exactly once.
// =====================================================================

#include "snowclear/ablation_switches.hpp"
#include "snowclear/cloud_operations.hpp"
#include "snowclear/logging.hpp"
#include "snowclear/param_source.hpp"
#include "snowclear/system_config.hpp"
#include "yaml_params.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using snowclear::AblationSwitches;
using snowclear::CloudOperations;
using snowclear::EvaluationMetrics;
using snowclear::MapParamSource;
using snowclear::SystemConfig;

namespace {

struct ParsedArgs {
    std::string mode = "all_checks";
    std::string params_file;
    MapParamSource overrides;
};

ParsedArgs parse_args(int argc, char** argv) {
    ParsedArgs out;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--mode" && i + 1 < argc) {
            out.mode = argv[++i];
            continue;
        }
        if (arg.rfind("--mode=", 0) == 0) {
            out.mode = arg.substr(7);
            continue;
        }
        if (arg == "--params" && i + 1 < argc) {
            out.params_file = argv[++i];
            continue;
        }
        const auto sep = arg.find(":=");
        if (sep != std::string::npos) out.overrides.set(arg.substr(0, sep), arg.substr(sep + 2));
    }
    return out;
}

std::string read_text(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return "";
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool same_number(const std::string& a, const std::string& b) {
    try {
        size_t used_a = 0, used_b = 0;
        const double da = std::stod(a, &used_a);
        const double db = std::stod(b, &used_b);
        if (used_a == a.size() && used_b == b.size()) {
            // Most parameters are stored as float. Demanding better than a
            // relative 1e-6 would flag float rounding, not a real mismatch.
            const double scale = std::max({1.0, std::abs(da), std::abs(db)});
            return std::abs(da - db) <= 1e-6 * scale;
        }
    } catch (...) {
    }
    return a == b;
}

// Keys that a YAML config is not expected to pin: they describe a particular
// run (where to read frames from, where to write indices), not the algorithm.
bool is_runtime_key(const std::string& key) {
    static const std::vector<std::string> kRuntime = {
        "pcd_file", "pcd_folder", "pcd_root", "result_folder", "result_root",
        "gt_folder", "output_dir", "save_results", "verbose", "csv_path",
        "reference_file", "process_all_frames", "mode", "params_file",
    };
    return std::find(kRuntime.begin(), kRuntime.end(), key) != kRuntime.end();
}

// ---------------------------------------------------------------------
// param_check
// ---------------------------------------------------------------------
// Two independent invariants:
//   1. every key the shipped YAML pins must resolve to the value the YAML
//      states, given the compiled-in defaults underneath it;
//   2. every key in the YAML must be read by somebody — an unread key is a
//      parameter that silently does nothing.
// ---------------------------------------------------------------------
int run_param_check(const std::string& yaml_path) {
    MapParamSource file;
    if (!snowclear::app::load_yaml_into(yaml_path, file)) {
        std::cout << "[FAIL] 无法读取 " << yaml_path << std::endl;
        return 1;
    }

    const SystemConfig default_config;
    const AblationSwitches default_switches;
    SystemConfig config;
    AblationSwitches switches;
    config.load(file);
    switches.load(file);

    const auto defaults = [&] {
        std::map<std::string, std::string> m = default_config.to_map();
        for (const auto& kv : default_switches.to_map()) m[kv.first] = kv.second;
        return m;
    }();
    const auto effective = [&] {
        std::map<std::string, std::string> m = config.to_map();
        for (const auto& kv : switches.to_map()) m[kv.first] = kv.second;
        return m;
    }();

    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    size_t checked = 0;

    for (const auto& kv : file.entries()) {
        const auto it = effective.find(kv.first);
        if (it == effective.end()) {
            warnings.push_back("YAML 键无人读取（死参数？）: " + kv.first);
            continue;
        }
        ++checked;
        if (!same_number(kv.second, it->second)) {
            errors.push_back("参数 " + kv.first + ": YAML 声明 " + kv.second +
                             " != 生效值 " + it->second);
        }
    }

    // A parameter absent from the YAML still has a default, but the shipped
    // configuration should pin every one of them explicitly.
    for (const auto& kv : effective) {
        if (!file.has(kv.first) && !is_runtime_key(kv.first)) {
            warnings.push_back("YAML 未固定该参数: " + kv.first);
        }
    }

    for (const auto& w : warnings) std::cout << "[WARN] " << w << std::endl;
    if (!errors.empty()) {
        std::cout << "[FAIL] 发现 " << errors.size() << " 处配置不一致:" << std::endl;
        for (const auto& e : errors) std::cout << "  - " << e << std::endl;
        return 1;
    }
    std::cout << "[OK] 配置一致 (" << file.size() << " 个键, 校验 " << checked
              << " 个, 生效参数 " << effective.size() << " 个)" << std::endl;
    return 0;
}

// ---------------------------------------------------------------------
// regression
// ---------------------------------------------------------------------
int run_regression(CloudOperations& ops, const std::string& pcd_file,
                   const std::string& reference_file, const std::string& output_dir) {
    if (pcd_file.empty() || reference_file.empty() || output_dir.empty()) {
        std::cout << "[FAIL] regression 需要 pcd_file / reference_file / output_dir" << std::endl;
        return 1;
    }

    ops.set_save_results(true);
    ops.process_pcd_file(pcd_file);

    const std::string produced = (fs::path(output_dir) /
                                  (fs::path(pcd_file).stem().string() + ".txt")).string();
    const std::string reference = read_text(reference_file);
    const std::string actual = read_text(produced);

    if (reference.empty()) {
        std::cout << "[FAIL] 参考文件为空或不存在: " << reference_file << std::endl;
        return 1;
    }
    if (actual == reference) {
        const auto lines = std::count(actual.begin(), actual.end(), '\n');
        std::cout << "[OK] 检测输出与参考逐字节一致 (\"" << fs::path(reference_file).filename().string()
                  << "\", " << lines << " 行)" << std::endl;
        return 0;
    }

    std::cout << "[FAIL] 检测输出与参考不一致 —— 检测行为已被改动！" << std::endl;
    std::cout << "  参考: " << reference_file << " (" << reference.size() << " 字节)" << std::endl;
    std::cout << "  实测: " << produced << " (" << actual.size() << " 字节)" << std::endl;
    return 1;
}

// ---------------------------------------------------------------------
// CSV output
// ---------------------------------------------------------------------
void write_metrics_header(const std::string& csv_path) {
    std::ofstream out(csv_path, std::ios::trunc);
    if (!out.is_open()) return;
    out << "folder,precision,recall,f1,f1_means,pooled_p,pooled_r,pooled_f1,time_ms\n";
}

void write_metrics_row(const std::string& csv_path, const std::string& folder,
                       const EvaluationMetrics& m, double algo_ms) {
    std::ofstream out(csv_path, std::ios::app);
    if (!out.is_open()) return;
    const auto mean = [&](double sum) { return m.total_frames > 0 ? sum / m.total_frames : 0.0; };
    const double p = mean(m.total_precision) * 100.0;
    const double r = mean(m.total_recall) * 100.0;
    const double f1 = mean(m.total_f1_score) * 100.0;
    const double f1_means = (p + r > 0) ? 2.0 * p * r / (p + r) : 0.0;
    const double pp = (m.total_tp + m.total_fp > 0)
        ? 100.0 * static_cast<double>(m.total_tp) / (m.total_tp + m.total_fp) : 0.0;
    const double pr = (m.total_tp + m.total_fn > 0)
        ? 100.0 * static_cast<double>(m.total_tp) / (m.total_tp + m.total_fn) : 0.0;
    const double pf1 = (pp + pr > 0) ? 2.0 * pp * pr / (pp + pr) : 0.0;
    out << folder << ',' << p << ',' << r << ',' << f1 << ',' << f1_means << ','
        << pp << ',' << pr << ',' << pf1 << ',' << algo_ms << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    const ParsedArgs args = parse_args(argc, argv);

    MapParamSource params;
    if (!args.params_file.empty()) snowclear::app::load_yaml_into(args.params_file, params);
    for (const auto& kv : args.overrides.entries()) params.set(kv.first, kv.second);

    if (args.mode == "param_check") {
        return run_param_check(args.params_file);
    }

    const std::string pcd_root = params.get_string("pcd_root", "");
    const std::string result_root = params.get_string("result_root", "");
    const std::string pcd_file = params.get_string("pcd_file", "");
    const std::string reference_file = params.get_string("reference_file", "");
    const std::string output_dir = params.get_string("output_dir", "");
    const std::string csv_path = params.get_string("csv_path", "");

    if (args.mode == "all_checks") {
        const int rc = run_param_check(args.params_file);
        if (rc != 0) return rc;
        CloudOperations ops(params);
        return run_regression(ops, pcd_file, reference_file, output_dir);
    }

    CloudOperations operations(params);

    if (args.mode == "regression") {
        return run_regression(operations, pcd_file, reference_file, output_dir);
    }

    if (args.mode == "reproduce_full") {
        if (pcd_root.empty() || result_root.empty()) {
            std::cout << "[FAIL] reproduce_full 需要 pcd_root 与 result_root" << std::endl;
            return 1;
        }
        operations.process_multiple_frames(pcd_root, result_root);
        if (!csv_path.empty()) {
            write_metrics_header(csv_path);
            write_metrics_row(csv_path, "all", operations.evaluation_metrics(),
                              operations.average_algorithm_time_ms());
        }
        return operations.processed_frames() > 0 ? 0 : 1;
    }

    if (args.mode == "eval_folders") {
        if (pcd_root.empty() || result_root.empty()) {
            std::cout << "[FAIL] eval_folders 需要 pcd_root 与 result_root" << std::endl;
            return 1;
        }
        if (!csv_path.empty()) write_metrics_header(csv_path);

        std::vector<std::string> folders;
        for (const auto& entry : fs::directory_iterator(pcd_root)) {
            if (entry.is_directory()) folders.push_back(entry.path().string());
        }
        std::sort(folders.begin(), folders.end());
        if (folders.empty()) {
            std::cout << "[FAIL] " << pcd_root << " 下没有场景目录" << std::endl;
            return 1;
        }
        for (const auto& folder : folders) {
            SC_INFO("=== 评估文件夹: %s ===", folder.c_str());
            operations.process_multiple_frames(folder, result_root);
            if (operations.processed_frames() <= 0) {
                std::cout << "[FAIL] 场景 " << fs::path(folder).filename().string()
                          << " 没有处理任何帧 —— 拒绝写入 0 指标行" << std::endl;
                return 1;
            }
            if (!csv_path.empty()) {
                write_metrics_row(csv_path, fs::path(folder).filename().string(),
                                  operations.evaluation_metrics(),
                                  operations.average_algorithm_time_ms());
            }
        }
        return 0;
    }

    std::cout << "[FAIL] 未知 mode: " << args.mode << std::endl
              << "       可用: param_check | regression | reproduce_full | eval_folders | all_checks"
              << std::endl;
    return 1;
}
