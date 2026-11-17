"""Everything that runs on the rover: base node + browser HMI.

On the rover:      ros2 launch rover_bringup rover.launch.py
Without hardware:  ros2 launch rover_bringup rover.launch.py fake:=true
Then open http://rover.local:8080 (or http://localhost:8080 when running in WSL).

With fake:=true all nodes run in the /fake namespace (/fake/cmd_vel, /fake/ultrasonic).
The PC and the rover share one ROS network, so without this a test in WSL would also
drive the real rover if its base node is running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fake = LaunchConfiguration("fake")
    return LaunchDescription([
        DeclareLaunchArgument("fake", default_value="false", description="run without hardware"),
        GroupAction([
            PushRosNamespace("fake", condition=IfCondition(fake)),
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare("rover_base"), "launch", "base.launch.py"]),
                launch_arguments={"fake": fake}.items(),
            ),
            Node(package="rover_hmi", executable="hmi", name="rover_hmi", output="screen"),
        ]),
    ])
