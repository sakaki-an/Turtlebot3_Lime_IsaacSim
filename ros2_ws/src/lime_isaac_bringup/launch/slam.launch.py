"""Tutorial 14: build a map of the lime_room with slam_toolbox while you drive.

    ros2 launch lime_isaac_bringup slam.launch.py            # with RViz
    ros2 launch lime_isaac_bringup slam.launch.py rviz:=false

Starts robot_state_publisher (base_footprint -> base_scan comes from the URDF), scan_fix (/scan -> /scan_fixed:
Isaac Sim's "no return" -1 becomes +inf), slam_toolbox's own online_async_launch.py with this package's
config/slam_params.yaml, and RViz. slam_toolbox publishes /map and the TF map -> odom.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("lime_isaac_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("slam_params_file", default_value=PathJoinSubstitution([pkg, "config", "slam_params.yaml"])),
            IncludeLaunchDescription(PathJoinSubstitution([pkg, "launch", "robot_state_publisher.launch.py"])),
            Node(package="lime_isaac_bringup", executable="scan_fix", output="screen"),  # /scan -> /scan_fixed
            # slam_toolbox runs as a lifecycle node: its own launch file configures and activates it.
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]),
                launch_arguments={
                    "slam_params_file": LaunchConfiguration("slam_params_file"),
                    "use_sim_time": "true",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg, "rviz", "slam.rviz"])],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
