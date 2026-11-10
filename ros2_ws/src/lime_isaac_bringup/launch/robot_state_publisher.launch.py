"""robot_state_publisher for TurtleBot3 Lime simulated in Isaac Sim.

Isaac Sim publishes /joint_states (wheels, arm, gripper) and /clock. robot_state_publisher turns the
URDF + joint states into the TF tree base_footprint -> base_link -> ... -> end_effector_link.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_lime_description"), "urdf", "turtlebot3_lime.urdf.xacro"]
    )
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="use Isaac Sim's /clock"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
