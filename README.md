# TurtleBot3 Lime × Isaac Sim 6.2 — beginner tutorials

**📖 Tutorials: https://sakaki-an.github.io/Turtlebot3_Lime_IsaacSim/**

Learn NVIDIA **Isaac Sim 6.2** step by step with one robot: the **TurtleBot3 Lime**, a mobile
manipulator with a differential-drive base, a 6-DoF arm with gripper, a RealSense D435 and a 2D LiDAR.
The tutorials follow the structure of NVIDIA's Isaac Sim tutorials. Each one has a **GUI** walkthrough,
a **standalone Python script**, and a video of the result. **ROS 2 Jazzy** (Nav2, MoveIt 2, YOLO) runs in
a Docker container next to Isaac Sim.

![TurtleBot3 Lime driven from ROS 2 in Isaac Sim](docs/assets/img/tutorial06_ros2_drive.png)

## Quick start

```bash
git clone https://github.com/sakaki-an/Turtlebot3_Lime_IsaacSim.git
cd Turtlebot3_Lime_IsaacSim

./scripts/setup_isaacsim.sh          # Isaac Sim 6.2 (pip) into .venv/  (~24 GB)
./docker/run_ros.sh true             # build + start the ROS 2 Jazzy container (first time ~15 min)

# terminal 1
source scripts/isaac_env.sh          # every terminal that runs Isaac Sim
python isaac/00_check_setup.py --headless       # publishes /clock for 60 s

# terminal 2
./docker/run_ros.sh ros2 topic echo /clock --once
```

Then follow the [tutorials](https://sakaki-an.github.io/Turtlebot3_Lime_IsaacSim/).

## Tutorials

| # | Tutorial | Script |
|---|---|---|
| 00 | Setup | `isaac/00_check_setup.py` |
| 02 | Import TurtleBot3 Lime (URDF → USD) | `isaac/02_import_urdf.py` |
| 03 | Tune the robot (drives, friction, solver) | `isaac/03_tune_robot.py` |
| 06 | Drive with ROS 2 `/cmd_vel` (Action Graph) | `isaac/06_ros2_drive.py` |

Every script accepts `--headless`, `--test` (automatic check, prints `PASS`/`FAIL`) and `--record NAME`
(saves the media used on the website). More tutorials are on the way: Isaac Sim basics, Python control,
Action Graph keyboard teleop, odometry/TF/RViz, camera, LiDAR + IMU, YOLO, Nav2 and MoveIt 2.

## Repository layout

```text
isaac/                 standalone Isaac Sim scripts, one per tutorial (+ common/ helpers)
assets/urdf/           TurtleBot3 Lime URDF for Isaac Sim (generated from the xacro)
assets/usd/            imported robot USD + tuned robot (turtlebot3_lime_tuned.usda)
ros2_ws/src/           ROS 2 Jazzy workspace (vendored TurtleBot3 Lime packages, ported from Humble)
docker/                ROS 2 Jazzy container (Nav2, MoveIt 2, ros2_control, YOLO)
scripts/               setup and environment scripts
docs/                  the tutorial website (MkDocs Material, deployed to GitHub Pages)
```

## Requirements

Ubuntu 22.04/24.04, an NVIDIA RTX GPU (tested: RTX 5080, driver 580), about 40 GB of disk, and Docker.

**Why Jazzy and not Humble?** Isaac Sim 6.2 supports ROS 2 Humble only on Ubuntu 22.04, and Jazzy on
Ubuntu 24.04. The TurtleBot3 Lime packages (`humble-devel`) are ported to Jazzy here.

## Credits & license

Apache-2.0. The TurtleBot3 Lime description and MoveIt configuration come from
[ROBOTIS-JAPAN-GIT/turtlebot3_lime](https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_lime) (Apache-2.0).
This project is the Isaac Sim 6.2 successor of
[Turtlebot3_Lime_IsaacSim_Humble](https://github.com/momoiorg-repository/Turtlebot3_Lime_IsaacSim_Humble).
See [NOTICE](NOTICE).
