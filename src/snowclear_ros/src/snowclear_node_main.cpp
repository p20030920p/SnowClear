// Standalone entry point so the node can be started with
//   ros2 run snowclear_ros snowclear_node
// without a component container. The same class is also registered as a
// composable node, so it can be loaded into an existing container.
#include "snowclear_ros/snowclear_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<snowclear_ros::SnowClearNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
