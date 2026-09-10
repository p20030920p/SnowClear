"""Launch SnowClear: the node, optionally with RViz and a PCD replay.

    ros2 launch snowclear_ros snowclear.launch.py
    ros2 launch snowclear_ros snowclear.launch.py rviz:=true
    ros2 launch snowclear_ros snowclear.launch.py \\
        pcd:=/path/frame.pcd params_file:=/path/to/overrides.yaml

The node needs no parameter file to reproduce the published configuration:
every parameter defaults to the released value, declared from the core's own
registry. `params_file` exists for overrides.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("snowclear_ros")

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([share, "config", "snowclear_node_params.yaml"]),
            description="ROS 2 parameter file with the released configuration",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="false", description="start RViz with the SnowClear layout"
        ),
        DeclareLaunchArgument(
            "pcd", default_value="", description="replay this PCD file through the node"
        ),
        DeclareLaunchArgument(
            "pcd_rate", default_value="1.0", description="replay rate in Hz"
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/snowclear/input/points",
            description="topic the node subscribes to",
        ),
    ]

    node = Node(
        package="snowclear_ros",
        executable="snowclear_node",
        name="snowclear",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[("~/input/points", LaunchConfiguration("input_topic"))],
    )

    replay = ExecuteProcess(
        cmd=[
            "ros2", "run", "snowclear_ros", "scan_to_cloud.py",
            "--pcd", LaunchConfiguration("pcd"),
            "--rate", LaunchConfiguration("pcd_rate"),
            "--topic", LaunchConfiguration("input_topic"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("pcd")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([share, "rviz", "snowclear.rviz"])],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(args + [node, replay, rviz])
