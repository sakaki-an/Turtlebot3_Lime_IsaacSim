"""Tutorial 16 - All together: tidy the room. Three bananas lie around the lime_room; the Lime finds them all.

    source scripts/isaac_env.sh
    python isaac/16_tidy_room.py

    # ROS 2 container: MoveIt 2 + ros2_control, Nav2 (+ the depth camera in the costmaps), YOLO, RViz
    ./docker/run_ros.sh ros2 launch lime_isaac_bringup tidy.launch.py
    ./docker/run_ros.sh ros2 run lime_isaac_bringup tidy        # the mission: a behavior tree

The robot of tutorial 13 (every sensor and graph from tutorials 06-12) in tutorial 13's lime_room, plus:
  * the mug and the bottle moved out of the way, towards the corners: things to find and leave alone
  * two more bananas, where the robot cannot see them from its start (common/scene.py: setup_tidy_room)
  * a small point cloud from the depth camera, so Nav2 sees what is lower than the LiDAR (graphs.create_depth_cloud_graph)
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
from common.scene import OBJECTS, build_lime_room, setup_pick_and_place, setup_tidy_room
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.simulation_manager import SimulationManager

graphs.enable_ros2_realtime(simulation_app)

stage = stage_utils.create_new_stage(template="empty")
build_lime_room(stage, object_physics=True)  # tutorial 13's room: real objects
while stage_utils.is_stage_loading():
    simulation_app.update()
robot_path = "/World/turtlebot3_lime"
articulation_root = lime.add_lime(robot_path)
setup_pick_and_place(stage, robot_path)  # a bigger bowl, grippy fingers
bananas = setup_tidy_room(stage)  # the mug and the bottle out of the way; two more bananas, hidden from the start
while stage_utils.is_stage_loading():
    simulation_app.update()

graphs.create_clock_graph()  # 06
graphs.create_drive_graph(articulation_root)  # 06  /cmd_vel   <- Nav2, mission
graphs.create_odom_tf_joint_states_graph(articulation_root, joint_states_topic="/isaac_joint_states")  # 07 + 12
graphs.create_joint_command_graph(articulation_root)  # 12  /isaac_joint_commands <- ros2_control <- MoveIt
camera = sensors.add_d435_camera(robot_path)
graphs.create_camera_graph(camera)  # 08  camera  -> YOLO, mission
graphs.create_depth_cloud_graph(camera)  # 16  depth point cloud -> Nav2's costmaps
graphs.create_lidar_graph(sensors.add_lds_lidar(robot_path))  # 09  /scan    -> Nav2
graphs.create_imu_graph(sensors.add_imu(robot_path))  # 09  /imu

recorder = None
if args.record:
    from common.recorder import Recorder

    CHASE = np.array([-0.9, -1.7, 1.5])  # the camera follows the robot from behind and above (south-west)
    recorder = Recorder(args.record, eye=CHASE, target=(0.0, 0.0, 0.1), every_n=20, fps=24)
    follow = np.zeros(2)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
robot = Articulation(articulation_root)
arm = robot.get_dof_indices(lime.ARM_JOINTS)
print("[16] ready: ros2 launch lime_isaac_bringup tidy.launch.py, then ros2 run lime_isaac_bringup tidy", flush=True)

# --test: follow the mission from the simulation's point of view (ground truth)
prims = {path.rsplit("/", 1)[1]: XformPrim(path) for path in bananas}
others = {name: XformPrim(f"/World/LimeRoom/Objects/{name}") for name in ("Mug", "Bottle")}  # must not be touched
start = {}  # where they rest after settling (recorded 1 s into the simulation)
BOWL = np.array(next(o for o in OBJECTS if o[0] == "Bowl")[2:4])
HOME = np.array([0.0, 0.0])
events, steps, still_since = {}, 0, {}
if args.test or args.record:
    for name, prim in prims.items():
        (x, y, _), (w, qx, qy, qz) = prim.get_world_poses()[0].numpy()[0], prim.get_world_poses()[1].numpy()[0]
        axis = np.degrees(np.arctan2(2 * (qx * qy + w * qz), 1 - 2 * (qy * qy + qz * qz))) % 180  # its long axis (local x)
        print(f"[16] ground truth: {name.lower()} at ({x:.2f}, {y:.2f}), pointing {axis:.0f} deg", flush=True)


def event(name, text):
    events[name] = steps / 60
    print(f"[16] {text} at t = {events[name]:.0f} s", flush=True)


def in_bowl(p):  # its centre inside the bowl (rim radius 12 cm, 1.5 x the YCB bowl), resting, not held above it
    return np.linalg.norm(p[:2] - BOWL) < 0.10 and p[2] < 0.15


while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if recorder:
        follow += 0.01 * (robot.get_world_poses()[0].numpy()[0, :2] - follow)  # smooth, so the view does not jerk
        target = np.array([follow[0], follow[1], 0.1])
        recorder.look_at(target + CHASE, target)
        recorder.capture()
    if args.test or args.record:
        if steps == 60:
            start = {name: prim.get_world_poses()[0].numpy()[0].copy() for name, prim in others.items()}
        xy = robot.get_world_poses()[0].numpy()[0, :2]
        for name, prim in others.items():  # report the moment the robot bumps into the mug or the bottle
            if start and f"bumped {name}" not in events and np.linalg.norm(prim.get_world_poses()[0].numpy()[0] - start[name]) > 0.005:
                event(f"bumped {name}", f"{name.lower()} bumped (robot at {np.round(xy, 2)})")
        for name, prim in prims.items():
            b = prim.get_world_poses()[0].numpy()[0]
            if f"{name} lifted" not in events and b[2] > 0.10:
                event(f"{name} lifted", f"{name.lower()} lifted")
            still_since[name] = (still_since.get(name) or steps) if in_bowl(b) else None
            if f"{name} lifted" in events and f"{name} in bowl" not in events and still_since[name] and steps - still_since[name] > 60:
                event(f"{name} in bowl", f"{name.lower()} in the bowl, {np.linalg.norm(b[:2] - BOWL) * 100:.1f} cm from its centre")
        tidied = all(f"{name} in bowl" in events for name in prims)
        q = robot.get_dof_positions(dof_indices=arm).numpy()[0]
        if tidied and "home" not in events and np.linalg.norm(xy - HOME) < 0.3 and np.max(np.abs(q - np.array(lime.ARM_HOME))) < 0.05:
            event("home", "robot back home, arm folded")
        if ("home" in events and steps / 60 > events["home"] + 5) or steps > 1800 * 60:
            break  # (5 s more, so the mission can finish cleanly)

ok = True
if args.test:
    checks = {}
    for name, prim in prims.items():
        b = prim.get_world_poses()[0].numpy()[0]
        checks[f"{name.lower()} in the bowl at the end ({np.linalg.norm(b[:2] - BOWL) * 100:.1f} cm from its centre)"] = in_bowl(b)
    checks["robot back home with the arm folded (within 0.3 m)"] = "home" in events
    for name, prim in others.items():
        moved = np.linalg.norm(prim.get_world_poses()[0].numpy()[0] - start[name])
        checks[f"{name.lower()} not bumped into (moved {moved * 100:.1f} cm)"] = moved < 0.01
    for text, passed in checks.items():
        print(f"[16][check] {'ok  ' if passed else 'FAIL'} {text}")
    ok = all(checks.values())
    print(f"[16][test] {'PASS' if ok else 'FAIL'}")
if recorder:  # the picture for the docs: a close-up of the full bowl
    recorder.look_at((BOWL[0] - 0.35, BOWL[1] - 0.45, 0.45), (BOWL[0], BOWL[1], 0.04))
    for _ in range(30):
        simulation_app.update()
    for _ in range(72):  # 3 s at the end of the video
        simulation_app.update()
        recorder.capture(force=True)
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
