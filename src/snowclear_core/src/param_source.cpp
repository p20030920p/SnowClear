#include "snowclear/param_source.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <sstream>

namespace snowclear {
namespace {

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

// std::istringstream with the classic locale: identical behaviour to the
// previous ros1_compat convert<T>(), including "12abc" parsing as 12.
template <typename T>
T convert(const std::string& value, const T& fallback) {
    std::istringstream iss(value);
    T out{};
    iss >> out;
    if (iss.fail()) return fallback;
    return out;
}

}  // namespace

std::string normalize_param_key(const std::string& key) {
    std::string k = key;
    while (!k.empty() && (k.front() == '~' || k.front() == '/')) k.erase(k.begin());
    while (!k.empty() && k.front() == '_') k.erase(k.begin());
    const auto slash = k.find_last_of('/');
    if (slash != std::string::npos) k = k.substr(slash + 1);
    return k;
}

bool ParamSource::get_bool(const std::string& key, bool default_value) const {
    if (!has(key)) return default_value;
    const std::string v = lower(raw(key));
    if (v == "true" || v == "1" || v == "yes" || v == "on") return true;
    if (v == "false" || v == "0" || v == "no" || v == "off") return false;
    return default_value;
}

int ParamSource::get_int(const std::string& key, int default_value) const {
    if (!has(key)) return default_value;
    return convert<int>(raw(key), default_value);
}

float ParamSource::get_float(const std::string& key, float default_value) const {
    if (!has(key)) return default_value;
    return convert<float>(raw(key), default_value);
}

double ParamSource::get_double(const std::string& key, double default_value) const {
    if (!has(key)) return default_value;
    return convert<double>(raw(key), default_value);
}

std::string ParamSource::get_string(const std::string& key,
                                    const std::string& default_value) const {
    return has(key) ? raw(key) : default_value;
}

void MapParamSource::set(const std::string& key, const std::string& value) {
    values_[normalize_param_key(key)] = value;
}

void MapParamSource::set_if_absent(const std::string& key, const std::string& value) {
    values_.emplace(normalize_param_key(key), value);
}

bool MapParamSource::has(const std::string& key) const {
    return values_.find(normalize_param_key(key)) != values_.end();
}

std::string MapParamSource::raw(const std::string& key) const {
    const auto it = values_.find(normalize_param_key(key));
    return it == values_.end() ? std::string() : it->second;
}

}  // namespace snowclear
