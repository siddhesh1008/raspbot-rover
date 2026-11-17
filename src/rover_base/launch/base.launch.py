"""Start the rover base node.

On the rover:   ros2 launch rover_base base.launch.py
Without robot:  ros2 launch rover_base base.launch.py fake:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("rover_base"), "config", "rover_base.yaml"])
    fake = LaunchConfiguration("fake")
    return LaunchDescription([
        DeclareLaunchArgument("fake", default_value="false", description="run without hardware"),
        Node(
            package="rover_base",
            executable="base_node",
            name="rover_base",
            output="screen",
            parameters=[config, {"use_fake_bus": ParameterValue(fake, value_type=bool)}],
        ),
    ])
