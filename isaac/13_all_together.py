"""Tutorial 13 - Putting it all together: the complete simulated TurtleBot3 Lime.

    source scripts/isaac_env.sh
    python isaac/13_all_together.py

    # ROS 2 container: MoveIt 2 + ros2_control, Nav2, YOLO, RViz - then the mission
    ./docker/run_ros.sh ros2 launch lime_isaac_bringup lime_full.launch.py
    ./docker/run_ros.sh ros2 run lime_isaac_bringup mission

Everything from tutorials 06-12 in one scene:
    /clock  /cmd_vel  /odom  /tf  /isaac_joint_states  /isaac_joint_commands
    /camera/camera/...  /scan  /imu
"""

import sys

from common.cli import parse_args

args = parse_args(__doc__.splitlines()[0])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from common import graphs, lime, sensors
from common.scene import build_lime_room
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager

graphs.enable_ros2_realtime(simulation_app)

stage = stage_utils.create_new_stage(template="empty")
build_lime_room(stage)
while stage_utils.is_stage_loading():
    simulation_app.update()
robot_path = "/World/turtlebot3_lime"
articulation_root = lime.add_lime(robot_path)

graphs.create_clock_graph()  # 06
graphs.create_drive_graph(articulation_root)  # 06  /cmd_vel   <- Nav2
graphs.create_odom_tf_joint_states_graph(articulation_root, joint_states_topic="/isaac_joint_states")  # 07 + 12
graphs.create_joint_command_graph(articulation_root)  # 12  /isaac_joint_commands <- ros2_control <- MoveIt
graphs.create_camera_graph(sensors.add_d435_camera(robot_path))  # 08  camera  -> YOLO
graphs.create_lidar_graph(sensors.add_lds_lidar(robot_path))  # 09  /scan    -> Nav2
graphs.create_imu_graph(sensors.add_imu(robot_path))  # 09  /imu

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(0.9, -4.4, 3.6), target=(1.0, -0.4, 0.0), every_n=6, fps=20)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
robot = Articulation(articulation_root)
arm = robot.get_dof_indices(lime.ARM_JOINTS)
print("[13] ready: ros2 launch lime_isaac_bringup lime_full.launch.py, then ros2 run lime_isaac_bringup mission", flush=True)

# --test: follow the mission from the simulation's point of view (ground truth)
CORNER, START = np.array([2.8, -1.8]), np.array([0.0, 0.0])
events, steps = {}, 0
while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if recorder:
        recorder.capture()
    if args.test or args.record:
        xy = robot.get_world_poses()[0].numpy()[0, :2]
        q = robot.get_dof_positions(dof_indices=arm).numpy()[0]
        joint1 = float(q[0])
        if "corner" not in events and np.linalg.norm(xy - CORNER) < 0.3:
            events["corner"] = steps / 60
            print(f"[13] reached the far corner at t = {events['corner']:.0f} s", flush=True)
        if "corner" in events and "back" not in events and np.linalg.norm(xy - START) < 0.3:
            events["back"] = steps / 60
            print(f"[13] back at the start at t = {events['back']:.0f} s", flush=True)
        if "back" in events and "pointed" not in events and abs(joint1) > 0.15:
            events["pointed"] = steps / 60
            print(f"[13] arm turned to point (joint1 = {np.degrees(joint1):.0f} deg) at t = {events['pointed']:.0f} s", flush=True)
        if "pointed" in events and "folded" not in events and np.max(np.abs(q - np.array(lime.ARM_HOME))) < 0.05:
            events["folded"] = steps / 60
            print(f"[13] arm folded again at t = {events['folded']:.0f} s", flush=True)
        if ("folded" in events and steps / 60 > events["folded"] + 5) or steps > 400 * 60:
            break  # (5 s more, so the mission can finish cleanly)

ok = True
if args.test:
    for name in ("corner", "back", "pointed", "folded"):
        print(f"[13][check] {'ok  ' if name in events else 'FAIL'} {name}")
    ok = all(n in events for n in ("corner", "back", "pointed", "folded"))
    print(f"[13][test] {'PASS' if ok else 'FAIL'}")
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
