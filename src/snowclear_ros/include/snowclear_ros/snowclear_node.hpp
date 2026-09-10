#ifndef SNOWCLEAR_ROS_SNOWCLEAR_NODE_HPP
#define SNOWCLEAR_ROS_SNOWCLEAR_NODE_HPP

// =====================================================================
// SnowClearNode — live ROS 2 wrapper around the snowclear core
// ---------------------------------------------------------------------
// See src/snowclear_node.cpp for the implementation notes. Declared in a
// header so the standalone executable and the composable-node registration
// can share one definition instead of including a .cpp.
// =====================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <memory>
#include <vector>

namespace snowclear {

class CloudOperations;

}  // namespace snowclear

namespace snowclear_ros {

class SnowClearNode : public rclcpp::Node {
public:
    explicit SnowClearNode(const rclcpp::NodeOptions& options);
    // Declared out of line: the pipeline is held by unique_ptr and
    // CloudOperations is only forward-declared here.
    ~SnowClearNode() override;

    // Frames processed so far — used by the integration test.
    size_t processed_frames() const { return processed_frames_; }

private:
    void declare_algorithm_parameters();
    void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    std::unique_ptr<snowclear::CloudOperations> pipeline_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_snow_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_indices_;
    size_t processed_frames_ = 0;
};

}  // namespace snowclear_ros

#endif  // SNOWCLEAR_ROS_SNOWCLEAR_NODE_HPP
