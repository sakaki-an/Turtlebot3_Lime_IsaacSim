"""Tutorial 10: YOLO object detection on the Lime's RealSense images.

    ros2 launch lime_yolo yolo.launch.py                       # yolo11n, 5 Hz
    ros2 launch lime_yolo yolo.launch.py model:=yolo11s.pt confidence:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {
        "model": "yolo11n.pt",
        "confidence": "0.35",
        "max_rate": "5.0",
        "image_topic": "/camera/camera/color/image_raw",
    }
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
        + [
            Node(
                package="lime_yolo",
                executable="yolo_detector",
                parameters=[
                    {
                        "model": LaunchConfiguration("model"),
                        "confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float),
                        "max_rate": ParameterValue(LaunchConfiguration("max_rate"), value_type=float),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            )
        ]
    )
