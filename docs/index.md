# TurtleBot3 Lime × Isaac Sim 6.2

Learn **NVIDIA Isaac Sim 6.2** step by step with one robot: the **TurtleBot3 Lime**, a mobile
manipulator with a differential-drive base, a 6-DoF arm with gripper, an Intel RealSense D435 camera
and a 2D LiDAR.

The tutorials follow the structure of NVIDIA's own Isaac Sim tutorials. Every step uses the Lime,
and every step is written for people who are **opening Isaac Sim for the first time**.

<figure class="result" markdown>
<video controls autoplay loop muted playsinline>
  <source src="assets/video/tutorial06_ros2_drive.mp4" type="video/mp4">
</video>
<figcaption>TurtleBot3 Lime driven from ROS 2 (<code>/cmd_vel</code>) inside Isaac Sim 6.2.</figcaption>
</figure>

## How each tutorial works

Each tutorial has two ways to do the same thing:

=== "GUI"

    Click through Isaac Sim's interface. This is best for learning *where things are* and *what they mean*.

=== "Python (standalone)"

    Run one script from `isaac/`. Every script also accepts:

    | Flag | What it does |
    |---|---|
    | `--headless` | no window (servers, CI) |
    | `--test` | runs a short automatic check and prints `PASS` / `FAIL` |
    | `--record NAME` | saves the video and picture shown on the tutorial page |

!!! tip "Choosing between the tabs"
    The two tabs are linked. Pick **GUI** or **Python** once and every page switches to it.

## The tutorials

| # | Tutorial | You will learn |
|---|---|---|
| 00 | [Setup](tutorials/00-setup.md) | Install Isaac Sim 6.2 (pip), start the ROS 2 Jazzy container |
| 02 | [Import TurtleBot3 Lime](tutorials/02-import-urdf.md) | URDF → USD with the 6.2 URDF importer, the asset structure, physics variants |
| 03 | [Tune the robot](tutorials/03-tune-robot.md) | Joint drives, gains, friction and friction-combine modes, solver iterations |
| 06 | [Drive with ROS 2](tutorials/06-ros2-drive.md) | ROS 2 bridge, Action Graph, `/clock`, `/cmd_vel`, differential controller |
| 07 | [Odometry, TF & RViz](tutorials/07-odom-tf.md) | `/odom`, TF tree, `/joint_states`, `robot_state_publisher`, RViz |
| 08 | [Camera (RealSense D435)](tutorials/08-camera.md) | RGB, depth, camera_info; optical frames; the lime_room scene |
| 09 | [LiDAR & IMU](tutorials/09-lidar-imu.md) | RTX LiDAR configured as the LDS-01, `/scan`, IMU, `/imu`, sensors in RViz |
| 12 | [Arm control with MoveIt 2](tutorials/12-moveit2.md) | ros2_control over topics, MoveIt 2 on Jazzy, planning from RViz and Python |

More tutorials, all verified on Isaac Sim 6.2, are added as they are finished: the Isaac Sim basics,
Python control, your first Action Graph, YOLO and Nav2.

## What you need

- Ubuntu 22.04 or 24.04 with an NVIDIA RTX GPU (tested on Ubuntu 24.04, RTX 5080, driver 580)
- About 40 GB of free disk space
- Docker (for the ROS 2 Jazzy side)

!!! info "Why ROS 2 Jazzy and not Humble?"
    Isaac Sim 6.2 supports **Humble only on Ubuntu 22.04**. On Ubuntu 24.04 it supports **Jazzy**.
    The TurtleBot3 Lime packages are published for Humble, so this project ports them to Jazzy.
    The ROS side runs in a Docker container, so you do not need ROS installed on your PC.

## Source code

Everything is on GitHub:
[sakaki-an/Turtlebot3_Lime_IsaacSim](https://github.com/sakaki-an/Turtlebot3_Lime_IsaacSim).
The TurtleBot3 Lime packages come from
[ROBOTIS-JAPAN-GIT/turtlebot3_lime](https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_lime) (Apache-2.0).
See [Credits & license](reference/credits.md).
