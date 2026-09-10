// =====================================================================
// snowclear_cli — offline, ROS-free entry point
// ---------------------------------------------------------------------
// Single frame or whole folder, straight from PCD files. This is the
// executable used to reproduce the paper's numbers and to run the byte-exact
// regression; the ROS 2 node (snowclear_ros) is a thin live wrapper around
// the same CloudOperations pipeline.
//
// Usage
//   snowclear_cli --help
//   snowclear_cli --dump-params
//   snowclear_cli --params <file.yaml> pcd_file:=<frame.pcd> [key:=value ...]
//   snowclear_cli process_all_frames:=true pcd_folder:=<dir> result_folder:=<dir>
//
// Parameter precedence, lowest first:
//   compiled-in defaults  <  --params <yaml>  <  command line key:=value
//
// Every algorithm parameter accepted by the ROS 1 build is accepted here with
// the same name and the same default, which is what makes the two builds
// numerically interchangeable.
// =====================================================================

#include "snowclear/cloud_operations.hpp"
#include "snowclear/logging.hpp"
#include "snowclear/param_source.hpp"
#include "yaml_params.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using snowclear::CloudOperations;
using snowclear::MapParamSource;
using snowclear::ParamSource;

namespace {

void print_usage(const char* argv0) {
    std::cout <<
        "snowclear_cli — training-free LiDAR snow-point detection (offline)\n"
        "\n"
        "Usage:\n"
        "  " << argv0 << " [options] [key:=value ...]\n"
        "\n"
        "Options:\n"
        "  --params <file.yaml>   load parameters from a YAML file\n"
        "  --dump-params          print the effective configuration and exit\n"
        "  -h, --help             this message\n"
        "\n"
        "Common keys (names identical to the ROS 1 build):\n"
        "  pcd_file:=<path>            single-frame mode\n"
        "  process_all_frames:=true    folder mode\n"
        "  pcd_folder:=<dir>           input folder (recursive)\n"
        "  result_folder:=<dir>        ground-truth folder\n"
        "  gt_folder:=<dir>            ground truth, when different from above\n"
        "  save_results:=true          write snow indices to output_dir\n"
        "  output_dir:=<dir>           where those indices go\n"
        "  verbose:=true               per-frame timing and counts\n"
        "\n"
        "Examples:\n"
        "  " << argv0 << " pcd_file:=data/frame.pcd save_results:=false\n"
        "  " << argv0 << " process_all_frames:=true pcd_folder:=data/scans \\\n"
        "      result_folder:=data/gt save_results:=true output_dir:=/tmp/out\n";
}

// Split argv into (yaml file, key:=value overrides). ROS 1 style remappings
// are kept so that every command in the documentation still works verbatim.
struct ParsedArgs {
    std::string params_file;
    bool dump_params = false;
    bool show_help = false;
    MapParamSource overrides;
    std::vector<std::string> unknown;
};

bool parse_args(int argc, char** argv, ParsedArgs& out) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            out.show_help = true;
            return true;
        }
        if (arg == "--dump-params") {
            out.dump_params = true;
            continue;
        }
        if (arg == "--params") {
            if (i + 1 >= argc) {
                SC_ERROR("--params 需要一个文件路径");
                return false;
            }
            out.params_file = argv[++i];
            continue;
        }
        if (arg.rfind("--params=", 0) == 0) {
            out.params_file = arg.substr(9);
            continue;
        }

        const auto sep = arg.find(":=");
        if (sep != std::string::npos) {
            out.overrides.set(arg.substr(0, sep), arg.substr(sep + 2));
            continue;
        }
        out.unknown.push_back(arg);
    }
    return true;
}

void dump_params(const MapParamSource& source) {
    std::cout << "============ snowclear 生效参数 ============\n";
    for (const auto& kv : source.entries()) {
        std::cout << kv.first << " = " << kv.second << '\n';
    }
    std::cout << "===========================================\n";
}

}  // namespace

int main(int argc, char** argv) {
    ParsedArgs args;
    if (!parse_args(argc, argv, args)) return 2;

    if (args.show_help) {
        print_usage(argv[0]);
        return 0;
    }

    // Layered parameter source: YAML first, command line wins.
    MapParamSource params;
    if (!args.params_file.empty()) {
        if (!snowclear::app::load_yaml_into(args.params_file, params)) {
            SC_ERROR("无法加载参数文件: %s", args.params_file.c_str());
            return 2;
        }
    }
    for (const auto& kv : args.overrides.entries()) params.set(kv.first, kv.second);

    if (!args.unknown.empty()) {
        std::cout << "忽略无法识别的参数:";
        for (const auto& u : args.unknown) std::cout << ' ' << u;
        std::cout << "\n(使用 --help 查看用法)\n";
    }

    if (args.dump_params) {
        CloudOperations ops(params);
        dump_params(params);
        return 0;
    }

    const bool process_all = params.get_bool("process_all_frames", false);
    const std::string pcd_folder = params.get_string("pcd_folder", "");
    const std::string result_folder = params.get_string("result_folder", "");
    const std::string pcd_file = params.get_string("pcd_file", "");

    CloudOperations operations(params);

    try {
        if (process_all) {
            if (pcd_folder.empty() || !fs::exists(pcd_folder)) {
                SC_ERROR("PCD 文件夹不存在: %s", pcd_folder.c_str());
                return 1;
            }
            operations.process_multiple_frames(pcd_folder, result_folder);
            return 0;
        }

        if (!pcd_file.empty()) {
            if (!fs::exists(pcd_file)) {
                SC_ERROR("PCD 文件不存在: %s", pcd_file.c_str());
                return 1;
            }
            operations.process_pcd_file(pcd_file);
            return 0;
        }

        // No explicit input: fall back to the first scan in pcd_folder, which
        // is the smoke test the ROS 1 build used.
        if (pcd_folder.empty() || !fs::exists(pcd_folder)) {
            SC_ERROR("未指定输入。用 pcd_file:=<路径> 或 pcd_folder:=<目录>");
            print_usage(argv[0]);
            return 1;
        }

        std::vector<std::string> scans;
        for (const auto& entry : fs::recursive_directory_iterator(pcd_folder)) {
            if (entry.path().extension() == ".pcd") scans.push_back(entry.path().string());
        }
        if (scans.empty()) {
            SC_ERROR("目录中没有 PCD 文件: %s", pcd_folder.c_str());
            return 1;
        }
        std::sort(scans.begin(), scans.end());
        SC_INFO("默认模式处理第一帧: %s", scans.front().c_str());
        operations.process_pcd_file(scans.front());
        return 0;
    } catch (const std::exception& e) {
        SC_ERROR("主函数异常: %s", e.what());
        return 1;
    } catch (...) {
        SC_ERROR("主函数发生未知异常");
        return 1;
    }
}
