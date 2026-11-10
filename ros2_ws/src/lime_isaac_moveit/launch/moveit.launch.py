"""Tutorial 12: MoveIt 2 + ros2_control for TurtleBot3 Lime in Isaac Sim (ROS 2 Jazzy).

    ros2 launch lime_isaac_moveit moveit.launch.py            # with RViz (MotionPlanning panel)
    ros2 launch lime_isaac_moveit moveit.launch.py rviz:=false

Starts:
  robot_state_publisher  URDF (+ ros2_control block) -> /robot_description, TF of all links
  ros2_control_node      JointStateTopicSystem: reads /isaac_joint_states, writes /isaac_joint_commands
  spawner                joint_state_broadcaster (-> /joint_states), arm_controller, gripper_controller
  move_group             MoveIt 2 planning + execution (OMPL)
  rviz2                  MotionPlanning plugin (optional)

The planning groups, poses and kinematics come from the vendored turtlebot3_lime_moveit_config
package (SRDF, kinematics.yaml). joint_limits.yaml is this package's own copy with acceleration limits,
which MoveIt 2 Jazzy's time parameterization requires.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    lime_moveit = get_package_share_directory("turtlebot3_lime_moveit_config")  # vendored (ROBOTIS)
    this_pkg = get_package_share_directory("lime_isaac_moveit")

    moveit_config = (
        MoveItConfigsBuilder("turtlebot3_lime", package_name="lime_isaac_moveit")
        .robot_description(file_path="config/turtlebot3_lime_isaac.urdf.xacro")
        .robot_description_semantic(file_path=os.path.join(lime_moveit, "config", "turtlebot3_lime.srdf"))
        .robot_description_kinematics(file_path=os.path.join(lime_moveit, "config", "kinematics.yaml"))
        .joint_limits(file_path="config/joint_limits.yaml")  # own copy: Jazzy needs acceleration limits
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    sim_time = {"use_sim_time": True}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, sim_time],
        output="screen",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(this_pkg, "config", "ros2_controllers.yaml"), sim_time],
        remappings=[("~/robot_description", "/robot_description")],  # Jazzy: read the URDF from the topic
        output="screen",
    )
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller", "--controller-manager-timeout", "60"],
        parameters=[sim_time],
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict(), sim_time],
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(this_pkg, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            sim_time,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            robot_state_publisher,
            ros2_control_node,
            spawner,
            move_group,
            rviz,
        ]
    )
