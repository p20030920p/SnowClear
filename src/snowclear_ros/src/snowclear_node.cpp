// =====================================================================
// snowclear_node — live ROS 2 wrapper around the snowclear core
// ---------------------------------------------------------------------
// Subscribes to a point cloud, removes snowfall-induced points, and publishes
//   ~/input/points         (subscription)
//   ~/output/points        the de-snowed cloud
//   ~/output/snow_points   only the points classified as snow (for RViz)
//   ~/output/snow_indices  their indices into the cloud the detector saw
//
// The node owns no algorithm. It converts the message, calls
// CloudOperations::process_cloud(), and converts back — the same entry point
// the offline CLI uses, which is what makes the live and offline results
// identical rather than merely similar.
//
// Parameters
//   All algorithm parameters are declared from snowclear::param_registry()
//   with the released configuration as their defaults, so
//     ros2 param list /snowclear
//   shows the complete tunable surface and
//     ros2 param set /snowclear score_threshold 0.7
//   takes effect on the next scan. A YAML file passed with
//   `--ros-args --params-file` overrides them at start-up.
// =====================================================================

#include "snowclear/cloud_operations.hpp"
#include "snowclear/logging.hpp"
#include "snowclear/param_registry.hpp"
#include "snowclear_ros/ros_param_source.hpp"
#include "snowclear_ros/snowclear_node.hpp"

#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace snowclear_ros {

using snowclear::CloudOperations;
using snowclear::CloudPoint;
using snowclear::CloudPtr;
using snowclear::LogLevel;
using snowclear::ParamType;

SnowClearNode::SnowClearNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("snowclear", options) {
    declare_algorithm_parameters();

        // Route the core's printf-style logging into the ROS logger. Installed
        // before the pipeline is constructed, because the constructor dumps the
        // effective configuration.
        snowclear::set_log_sink([this](LogLevel level, const std::string& message) {
            switch (level) {
                case LogLevel::kDebug: RCLCPP_DEBUG(get_logger(), "%s", message.c_str()); break;
                case LogLevel::kInfo:  RCLCPP_INFO(get_logger(), "%s", message.c_str()); break;
                case LogLevel::kWarn:  RCLCPP_WARN(get_logger(), "%s", message.c_str()); break;
                case LogLevel::kError: RCLCPP_ERROR(get_logger(), "%s", message.c_str()); break;
            }
        });

        pipeline_ = std::make_unique<CloudOperations>(RosParamSource(*this));

        RCLCPP_INFO(get_logger(),
                    "配置已加载：detector=%s score_threshold=%.3f xy_threshold=%.1f "
                    "height_threshold=%.1f",
                    get_parameter("detector_type").as_string().c_str(),
                    get_parameter("score_threshold").as_double(),
                    get_parameter("xy_threshold").as_double(),
                    get_parameter("height_threshold").as_double());
        RCLCPP_INFO(get_logger(),
                    "参数总数 %zu —— ros2 param list 查看，ros2 param set 可在线修改",
                    snowclear::param_registry().size());

        const auto qos = rclcpp::SensorDataQoS();
        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "~/input/points", qos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_cloud(msg); });
        publisher_points_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/output/points", qos);
        publisher_snow_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/output/snow_points", qos);
        publisher_indices_ =
            create_publisher<std_msgs::msg::Int32MultiArray>("~/output/snow_indices", qos);

    RCLCPP_INFO(get_logger(), "SnowClear 就绪：%s -> %s / %s / %s",
                subscription_->get_topic_name(),
                publisher_points_->get_topic_name(),
                publisher_snow_->get_topic_name(),
                publisher_indices_->get_topic_name());
}

// ---------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------
SnowClearNode::~SnowClearNode() = default;

void SnowClearNode::declare_algorithm_parameters() {
    // Defaults come from the core, so an unset parameter behaves exactly as
    // it does in the offline build.
    const snowclear::SystemConfig default_config;
    const snowclear::AblationSwitches default_switches;
    std::map<std::string, std::string> defaults = default_config.to_map();
    for (const auto& kv : default_switches.to_map()) defaults[kv.first] = kv.second;

    for (const auto& spec : snowclear::param_registry()) {
        const auto it = defaults.find(spec.name);
        const std::string text = (it == defaults.end()) ? std::string() : it->second;
        switch (spec.type) {
            case ParamType::kBool:
                declare_parameter(spec.name, text == "true");
                break;
            case ParamType::kInt:
                declare_parameter(spec.name, static_cast<int64_t>(std::stoll(text)));
                break;
            case ParamType::kDouble:
                declare_parameter(spec.name, std::stod(text));
                break;
            case ParamType::kString:
                declare_parameter(spec.name, text);
                break;
        }
    }

    // Node-level behaviour, not algorithm parameters.
    declare_parameter("publish_snow_points", true);
    declare_parameter("publish_indices", true);
}

// ---------------------------------------------------------------------
// Per-scan processing
// ---------------------------------------------------------------------
void SnowClearNode::on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    CloudPtr input(new pcl::PointCloud<CloudPoint>);
    pcl::fromROSMsg(*msg, *input);

    // The offline path strips NaN before anything else. A dense message is
    // left untouched — no copy, no index shift — and a sparse one is cleaned
    // identically. Published indices always refer to the cloud the detector
    // actually saw.
    if (!msg->is_dense) {
        const size_t before = input->size();
        std::vector<int> kept;
        pcl::removeNaNFromPointCloud(*input, *input, kept);
        RCLCPP_DEBUG(get_logger(), "去除 NaN: %zu -> %zu", before, input->size());
    }

    if (input->empty()) {
        RCLCPP_WARN(get_logger(), "收到空点云，跳过本帧");
        return;
    }

    const auto begin = std::chrono::steady_clock::now();
    const CloudOperations::FrameResult result = pipeline_->process_cloud(input);
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::steady_clock::now() - begin).count();
    if (!result.ok) {
        RCLCPP_ERROR(get_logger(), "本帧处理失败，未发布任何结果");
        return;
    }
    ++processed_frames_;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*result.desnowed_cloud, output);
    output.header = msg->header;
    publisher_points_->publish(output);

    if (get_parameter("publish_snow_points").as_bool()) {
        pcl::PointCloud<CloudPoint> snow_points;
        snow_points.reserve(result.snow_indices->indices.size());
        for (const int index : result.snow_indices->indices) {
            snow_points.push_back(input->points[static_cast<size_t>(index)]);
        }
        sensor_msgs::msg::PointCloud2 snow_msg;
        pcl::toROSMsg(snow_points, snow_msg);
        snow_msg.header = msg->header;
        publisher_snow_->publish(snow_msg);
    }

    if (get_parameter("publish_indices").as_bool()) {
        std_msgs::msg::Int32MultiArray indices;
        indices.layout.data_offset = 0;
        indices.data.assign(result.snow_indices->indices.begin(),
                            result.snow_indices->indices.end());
        publisher_indices_->publish(indices);
    }

    RCLCPP_INFO(get_logger(), "本帧 %zu 点，判为雪 %zu 点，用时 %.2f ms",
                input->size(), result.snow_indices->indices.size(),
                static_cast<double>(elapsed) / 1000.0);
}

}  // namespace snowclear_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(snowclear_ros::SnowClearNode)
