// =====================================================================
// Node wiring tests
// ---------------------------------------------------------------------
// These check the ROS 2 bridge itself, not the algorithm: that the parameter
// server exposes the whole registry, that values round-trip through
// RosParamSource into the core config, and that the node comes up with the
// released defaults. Algorithm behaviour is covered in snowclear_core.
// =====================================================================

#include "snowclear/ablation_switches.hpp"
#include "snowclear/param_registry.hpp"
#include "snowclear/system_config.hpp"
#include "snowclear_ros/ros_param_source.hpp"
#include "snowclear_ros/snowclear_node.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace {

using snowclear::AblationSwitches;
using snowclear::ParamType;
using snowclear::SystemConfig;
using snowclear_ros::RosParamSource;
using snowclear_ros::SnowClearNode;

class NodeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        if (rclcpp::ok()) rclcpp::shutdown();
    }
};

// The node must expose every parameter the algorithm reads, plus its own two.
TEST_F(NodeTest, DeclaresTheWholeRegistry) {
    auto node = std::make_shared<SnowClearNode>(rclcpp::NodeOptions());
    const auto names = node->get_node_parameters_interface()->get_parameter_overrides();
    (void)names;

    size_t found = 0;
    for (const auto& spec : snowclear::param_registry()) {
        EXPECT_TRUE(node->has_parameter(spec.name)) << "undeclared parameter: " << spec.name;
        ++found;
    }
    EXPECT_EQ(found, snowclear::param_registry().size());
    EXPECT_TRUE(node->has_parameter("publish_snow_points"));
    EXPECT_TRUE(node->has_parameter("publish_indices"));
}

// Unset parameters must resolve to the released configuration, so starting the
// node without any params file reproduces the paper's numbers.
TEST_F(NodeTest, DefaultsReproduceTheReleasedConfiguration) {
    auto node = std::make_shared<SnowClearNode>(rclcpp::NodeOptions());
    RosParamSource source(*node);

    SystemConfig config;
    config.load(source);
    AblationSwitches switches;
    switches.load(source);

    EXPECT_FLOAT_EQ(config.score_threshold, 0.75f);
    EXPECT_DOUBLE_EQ(config.xy_threshold, 17.0);
    EXPECT_DOUBLE_EQ(config.height_threshold, 2.6);
    EXPECT_FLOAT_EQ(config.idsor_scale, 0.8f);
    EXPECT_TRUE(switches.use_idsor_intensity_threshold);
    EXPECT_TRUE(switches.enable_zero_intensity_surface_suppression);
    EXPECT_FALSE(switches.enable_planarity_calculation);
}

// `ros2 param set` must reach the algorithm on the next scan.
TEST_F(NodeTest, RuntimeParameterChangeIsVisibleToTheCore) {
    auto node = std::make_shared<SnowClearNode>(rclcpp::NodeOptions());
    node->set_parameter(rclcpp::Parameter("score_threshold", 0.42));
    node->set_parameter(rclcpp::Parameter("use_idsor_intensity_threshold", false));

    RosParamSource source(*node);
    SystemConfig config;
    config.load(source);
    AblationSwitches switches;
    switches.load(source);

    EXPECT_FLOAT_EQ(config.score_threshold, 0.42f);
    EXPECT_FALSE(switches.use_idsor_intensity_threshold);
}

// Every registry entry must survive a round trip through the parameter server
// and back out through RosParamSource without type or formatting loss.
TEST_F(NodeTest, EveryParameterRoundTripsThroughTheParameterServer) {
    auto node = std::make_shared<SnowClearNode>(rclcpp::NodeOptions());
    RosParamSource source(*node);

    for (const auto& spec : snowclear::param_registry()) {
        const std::string before = source.raw(spec.name);
        // String parameters may legitimately default to "", e.g. result_folder
        // and gt_folder are unset unless a run provides them. Numeric and
        // boolean ones always have a value.
        if (spec.type != ParamType::kString) {
            ASSERT_FALSE(before.empty()) << spec.name;
        }
        switch (spec.type) {
            case ParamType::kBool:
                node->set_parameter(rclcpp::Parameter(spec.name, before != "true"));
                break;
            case ParamType::kInt:
                node->set_parameter(rclcpp::Parameter(
                    spec.name, static_cast<int64_t>(std::stoll(before)) + 1));
                break;
            case ParamType::kDouble:
                node->set_parameter(rclcpp::Parameter(spec.name, std::stod(before) + 1.0));
                break;
            case ParamType::kString:
                node->set_parameter(rclcpp::Parameter(spec.name, before + "_x"));
                break;
        }
        EXPECT_NE(source.raw(spec.name), before) << spec.name;
    }
}

}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
