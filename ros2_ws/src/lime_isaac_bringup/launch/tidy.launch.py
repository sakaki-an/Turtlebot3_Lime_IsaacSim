"""Tutorial 16: tutorial 13's complete stack (MoveIt 2, Nav2, YOLO, RViz), with Nav2 also seeing the depth camera.

    ros2 launch lime_isaac_bringup tidy.launch.py
    ros2 run lime_isaac_bringup tidy             # the mission: tidy the room (a behavior tree)

The only difference to lime_full.launch.py is Nav2's parameter file: config/nav2_params_tidy.yaml adds a
costmap layer for the depth camera's point cloud, so Nav2 drives round the bananas, the mug and the bowl.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("lime_isaac_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            IncludeLaunchDescription(
                PathJoinSubstitution([pkg, "launch", "lime_full.launch.py"]),
                launch_arguments={
                    "rviz": LaunchConfiguration("rviz"),
                    "params_file": PathJoinSubstitution([pkg, "config", "nav2_params_tidy.yaml"]),
                }.items(),
            ),
        ]
    )
