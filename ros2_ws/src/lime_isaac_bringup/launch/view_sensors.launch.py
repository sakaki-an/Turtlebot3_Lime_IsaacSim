"""Tutorials 08/09: robot_state_publisher + RViz with the robot, TF, /scan and the camera image.

    ros2 launch lime_isaac_bringup view_sensors.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("lime_isaac_bringup")
    return LaunchDescription(
        [
            IncludeLaunchDescription(PathJoinSubstitution([pkg, "launch", "robot_state_publisher.launch.py"])),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg, "rviz", "view_sensors.rviz"])],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
