#include "yaml_params.hpp"

#include "snowclear/logging.hpp"

#include <fstream>

#include <yaml-cpp/yaml.h>

namespace snowclear {
namespace app {
namespace {

void flatten(const YAML::Node& node, const std::string& prefix, MapParamSource& out) {
    if (node.IsMap()) {
        for (auto it = node.begin(); it != node.end(); ++it) {
            const std::string key = it->first.as<std::string>();
            flatten(it->second, prefix.empty() ? key : prefix + "/" + key, out);
        }
    } else if (node.IsScalar()) {
        out.set(prefix, node.as<std::string>());
    }
}

}  // namespace

bool load_yaml_into(const std::string& path, MapParamSource& out) {
    if (path.empty()) return false;

    std::ifstream in(path);
    if (!in) {
        SC_WARN("无法打开参数文件: %s", path.c_str());
        return false;
    }

    try {
        flatten(YAML::Load(in), "", out);
        return true;
    } catch (const std::exception& e) {
        SC_WARN("解析参数文件失败 %s: %s", path.c_str(), e.what());
        return false;
    }
}

}  // namespace app
}  // namespace snowclear
