# Credits & license

This project is released under the **Apache License 2.0** (see `LICENSE` and `NOTICE` in the repository).

## Built on

| Project | Used for | License |
|---|---|---|
| [ROBOTIS-JAPAN-GIT/turtlebot3_lime](https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_lime) (`humble-devel`, commit `3363b27`) | robot description (xacro, meshes), MoveIt configuration, vendored in `ros2_ws/src/turtlebot3_lime/` and ported to Jazzy | Apache-2.0 |
| [Turtlebot3_Lime_IsaacSim_Humble](https://github.com/momoiorg-repository/Turtlebot3_Lime_IsaacSim_Humble) | the Isaac Sim 4.5 / Humble predecessor of this project: topic and controller names | MIT |
| [Turtlebot3_Lime_IsaacSim_Humble](https://github.com/momoiorg-repository/Turtlebot3_Lime_IsaacSim_Humble) `docs/imgs/lime1.jpg` | the photo of the real TurtleBot3 Lime (`docs/assets/img/real_lime.jpg`, `real_vs_sim.jpg`, resized), and the colours the simulated robot is painted with | MIT |
| [realsense-ros](https://github.com/IntelRealSense/realsense-ros) `realsense2_description` | D435 camera mesh (`assets/urdf/meshes/d435.dae`) | Apache-2.0 |
| [NVIDIA Isaac Sim 6.2](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/) | the simulator, installed by you via pip | NVIDIA license (not redistributed here) |
| [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/), Nav2, MoveIt 2, ros2_control | the ROS side, in the Docker container | Apache-2.0 / BSD |
| [nav2_bringup](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bringup) | `nav2_params.yaml` is generated from its Jazzy defaults; `navigation.rviz` is a modified copy of its RViz config | Apache-2.0 |
| [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | SLAM in tutorial 14; `slam_params.yaml` is generated from its Jazzy `mapper_params_online_async.yaml` | LGPL-2.1 |
| [Ultralytics YOLO](https://github.com/ultralytics/ultralytics) | object detection in `lime_yolo` (installed in the container, not redistributed) | AGPL-3.0 |
| [MkDocs](https://www.mkdocs.org/) + [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) | this website | BSD-2 / MIT |

## Changes to the TurtleBot3 Lime packages

- `turtlebot3_lime.urdf.xacro`: removed the Gazebo-classic plugins (not available on Jazzy) and the
  real-hardware `ros2_control` block. The robot geometry is unchanged.
- `package.xml`: declared the missing dependency on `realsense2_description`.
- `turtlebot3_lime.srdf`: the `home` pose used `joint3 = 1.87`, outside the URDF joint limit (1.8567).
  It now uses 1.80, so MoveIt accepts it as a start state, with a margin for the arm sagging under gravity.

Every modified file says so in its header.
