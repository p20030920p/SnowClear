#ifndef SNOWCLEAR_PARAM_SOURCE_HPP
#define SNOWCLEAR_PARAM_SOURCE_HPP

// =====================================================================
// Parameter access — the core's only configuration dependency
// ---------------------------------------------------------------------
// The algorithm used to read its configuration straight off a ROS 1
// parameter server (`nh.param("name", member, default)`), which meant the
// library could not be constructed at all without a live ROS node.
//
// Configuration is now pulled through this interface:
//
//   ROS 2 node    RosParamSource   wraps rclcpp::Node parameters
//   CLI / tests   MapParamSource   YAML file + `key:=value` overrides
//
// The **defaults stay in the core** (`SystemConfig`, `AblationSwitches`), so
// whichever front end is used, an unset parameter resolves to exactly the
// same value it did before — which is what keeps the published numbers
// bit-reproducible.
//
// Type coercion mirrors the previous behaviour: an unparsable value falls
// back to the default rather than throwing, and booleans accept
// true/false/1/0/yes/no/on/off case-insensitively.
// =====================================================================

#include <cstdio>
#include <map>
#include <string>

namespace snowclear {

class ParamSource {
public:
    virtual ~ParamSource() = default;

    // True when the key is present at all (even with an empty value).
    virtual bool has(const std::string& key) const = 0;

    // Raw string value, or "" when the key is absent.
    virtual std::string raw(const std::string& key) const = 0;

    // Typed reads with fallback. Both "name" and "_name" spellings resolve to
    // the same key, matching the ROS 1 private-parameter convention the
    // command line used to follow.
    bool get_bool(const std::string& key, bool default_value) const;
    int get_int(const std::string& key, int default_value) const;
    float get_float(const std::string& key, float default_value) const;
    double get_double(const std::string& key, double default_value) const;
    std::string get_string(const std::string& key, const std::string& default_value) const;
};

// Key/value store. Used by the offline CLI and by unit tests; also the base
// class for front ends that need to layer sources (file < command line).
class MapParamSource : public ParamSource {
public:
    MapParamSource() = default;

    void set(const std::string& key, const std::string& value);
    void set_if_absent(const std::string& key, const std::string& value);

    bool has(const std::string& key) const override;
    std::string raw(const std::string& key) const override;

    const std::map<std::string, std::string>& entries() const { return values_; }
    size_t size() const { return values_.size(); }

private:
    std::map<std::string, std::string> values_;
};

// Normalise "~name", "/node/name", "_name" to the bare "name" spelling.
std::string normalize_param_key(const std::string& key);

// ---------------------------------------------------------------------
// Canonical text form of a parameter value
// ---------------------------------------------------------------------
// Used by SystemConfig::to_map() / AblationSwitches::to_map() so that a
// configuration can be printed and compared without relying on overload
// resolution of a generic formatter. Floating point uses %.10g, which
// round-trips every default in the shipped configuration exactly.
// ---------------------------------------------------------------------
inline std::string param_value_string(bool value) { return value ? "true" : "false"; }
inline std::string param_value_string(int value) { return std::to_string(value); }
inline std::string param_value_string(long value) { return std::to_string(value); }
inline std::string param_value_string(const std::string& value) { return value; }
inline std::string param_value_string(const char* value) { return std::string(value); }

inline std::string param_value_string(double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.10g", value);
    return std::string(buffer);
}

inline std::string param_value_string(float value) {
    // 7 significant digits is exactly what a float round-trips, so a YAML
    // literal such as 0.06 is reported as "0.06" rather than 0.05999999866.
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.7g", static_cast<double>(value));
    return std::string(buffer);
}


}  // namespace snowclear

#endif  // SNOWCLEAR_PARAM_SOURCE_HPP
