# Credits & license

This project is released under the **Apache License 2.0** (see `LICENSE` and `NOTICE` in the repository).

## Built on

| Project | Used for | License |
|---|---|---|
| [ROBOTIS-JAPAN-GIT/turtlebot3_lime](https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_lime) (`humble-devel`, commit `3363b27`) | robot description (xacro, meshes), MoveIt configuration, vendored in `ros2_ws/src/turtlebot3_lime/` and ported to Jazzy | Apache-2.0 |
| [Turtlebot3_Lime_IsaacSim_Humble](https://github.com/momoiorg-repository/Turtlebot3_Lime_IsaacSim_Humble) | the Isaac Sim 4.5 / Humble predecessor of this project: topic and controller names | MIT |
| [realsense-ros](https://github.com/IntelRealSense/realsense-ros) `realsense2_description` | D435 camera mesh (`assets/urdf/meshes/d435.dae`) | Apache-2.0 |
| [NVIDIA Isaac Sim 6.2](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/) | the simulator, installed by you via pip | NVIDIA license (not redistributed here) |
| [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/), Nav2, MoveIt 2, ros2_control | the ROS side, in the Docker container | Apache-2.0 / BSD |
| [MkDocs](https://www.mkdocs.org/) + [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) | this website | BSD-2 / MIT |

## Changes to the TurtleBot3 Lime packages

- `turtlebot3_lime.urdf.xacro`: removed the Gazebo-classic plugins (not available on Jazzy) and the
  real-hardware `ros2_control` block. The robot geometry is unchanged.
- `package.xml`: declared the missing dependency on `realsense2_description`.

Every modified file says so in its header.
