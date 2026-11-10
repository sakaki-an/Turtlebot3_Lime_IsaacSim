"""Tutorial 07: robot_state_publisher + RViz showing the robot model, TF and odometry from Isaac Sim.

    ros2 launch lime_isaac_bringup view_robot.launch.py            # with RViz
    ros2 launch lime_isaac_bringup view_robot.launch.py rviz:=false
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
            IncludeLaunchDescription(PathJoinSubstitution([pkg, "launch", "robot_state_publisher.launch.py"])),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg, "rviz", "view_robot.rviz"])],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
