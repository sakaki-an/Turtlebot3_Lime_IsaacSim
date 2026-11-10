"""Tutorial 11: Nav2 for TurtleBot3 Lime in Isaac Sim's lime_room.

    ros2 launch lime_isaac_bringup navigation.launch.py            # with RViz
    ros2 launch lime_isaac_bringup navigation.launch.py rviz:=false

Starts robot_state_publisher, the standard Nav2 bringup (map_server, AMCL, planner, controller, behaviours,
velocity smoother, collision monitor, ...) with this package's map and parameters, and RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("lime_isaac_bringup")
    nav2 = FindPackageShare("nav2_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("map", default_value=PathJoinSubstitution([pkg, "maps", "lime_room.yaml"])),
            DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])),
            IncludeLaunchDescription(PathJoinSubstitution([pkg, "launch", "robot_state_publisher.launch.py"])),
            IncludeLaunchDescription(
                PathJoinSubstitution([nav2, "launch", "bringup_launch.py"]),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": "true",
                    "autostart": "true",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg, "rviz", "navigation.rviz"])],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
