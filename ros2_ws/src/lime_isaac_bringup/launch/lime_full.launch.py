"""Tutorial 13: everything at once - MoveIt 2 + ros2_control, Nav2, YOLO and RViz.

    ros2 launch lime_isaac_bringup lime_full.launch.py
    ros2 run lime_isaac_bringup mission          # the pick-and-place mission (YOLO + MoveIt + Nav2)

Only ONE robot_state_publisher runs (the one in lime_isaac_moveit/moveit.launch.py, whose URDF also
contains the ros2_control block). /joint_states comes from joint_state_broadcaster.
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
            # MoveIt 2 + ros2_control (+ robot_state_publisher)
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare("lime_isaac_moveit"), "launch", "moveit.launch.py"]),
                launch_arguments={"rviz": "false"}.items(),
            ),
            # Nav2 (map_server, AMCL, planner, controller, ...)
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]),
                launch_arguments={
                    "map": PathJoinSubstitution([pkg, "maps", "lime_room.yaml"]),
                    "params_file": PathJoinSubstitution([pkg, "config", "nav2_params.yaml"]),
                    "use_sim_time": "true",
                    "autostart": "true",
                }.items(),
            ),
            # YOLO - the medium model: from the low camera the nano model (tutorial 10) calls the banana a "frisbee"
            DeclareLaunchArgument("yolo_model", default_value="yolo11m.pt"),
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare("lime_yolo"), "launch", "yolo.launch.py"]),
                launch_arguments={"model": LaunchConfiguration("yolo_model")}.items(),
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
