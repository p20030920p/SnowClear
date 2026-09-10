#ifndef SNOWCLEAR_ROS_ROS_PARAM_SOURCE_HPP
#define SNOWCLEAR_ROS_ROS_PARAM_SOURCE_HPP

// =====================================================================
// RosParamSource — rclcpp parameters as a snowclear::ParamSource
// ---------------------------------------------------------------------
// This is the entire bridge between the ROS 2 parameter server and the
// algorithm. Everything below it (thresholds, switches, the decision
// function) is ROS-agnostic, which is why the same core also runs from the
// CLI and from unit tests.
// =====================================================================

#include "snowclear/param_source.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <string>

namespace snowclear_ros {

class RosParamSource : public snowclear::ParamSource {
public:
    explicit RosParamSource(rclcpp::Node& node) : node_(node) {}

    bool has(const std::string& key) const override {
        return node_.has_parameter(snowclear::normalize_param_key(key));
    }

    std::string raw(const std::string& key) const override {
        const std::string name = snowclear::normalize_param_key(key);
        if (!node_.has_parameter(name)) return {};

        const rclcpp::Parameter& p = node_.get_parameter(name);
        switch (p.get_type()) {
            case rclcpp::ParameterType::PARAMETER_BOOL:
                return p.as_bool() ? "true" : "false";
            case rclcpp::ParameterType::PARAMETER_INTEGER:
                return std::to_string(p.as_int());
            case rclcpp::ParameterType::PARAMETER_DOUBLE: {
                // Same %.10g formatting the core uses, so a value round-trips
                // through ros2 param set without picking up representation noise.
                char buffer[32];
                std::snprintf(buffer, sizeof(buffer), "%.10g", p.as_double());
                return std::string(buffer);
            }
            case rclcpp::ParameterType::PARAMETER_STRING:
                return p.as_string();
            default:
                return {};
        }
    }

private:
    rclcpp::Node& node_;
};

}  // namespace snowclear_ros

#endif  // SNOWCLEAR_ROS_ROS_PARAM_SOURCE_HPP
