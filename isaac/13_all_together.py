"""Tutorial 13 - Putting it all together: the complete simulated TurtleBot3 Lime.

    source scripts/isaac_env.sh
    python isaac/13_all_together.py

    # ROS 2 container: MoveIt 2 + ros2_control, Nav2, YOLO, RViz - then the mission
    ./docker/run_ros.sh ros2 launch lime_isaac_bringup lime_full.launch.py
    ./docker/run_ros.sh ros2 run lime_isaac_bringup mission

Everything from tutorials 06-12 in one scene:
    /clock  /cmd_vel  /odom  /tf  /isaac_joint_states  /isaac_joint_commands
    /camera/camera/...  /scan  /imu
New here: the objects are real (rigid bodies, colliders, masses): the gripper can pick up the banana, the bowl
holds it, and the mug and the bottle would be knocked over (common/scene.py: setup_pick_and_place).
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
from common.scene import OBJECTS, build_lime_room, setup_pick_and_place
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.simulation_manager import SimulationManager

graphs.enable_ros2_realtime(simulation_app)

stage = stage_utils.create_new_stage(template="empty")
build_lime_room(stage, object_physics=True)  # real objects: colliders and mass, so they can be picked up
while stage_utils.is_stage_loading():
    simulation_app.update()
robot_path = "/World/turtlebot3_lime"
articulation_root = lime.add_lime(robot_path)
setup_pick_and_place(stage, robot_path)  # a bigger bowl, grippy fingers

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

    CLOSE_UP = ((1.75, -1.05, 0.85), (0.85, 0.12, 0.05))  # pick and place
    OVERVIEW = ((0.9, -4.4, 3.6), (1.0, -0.4, 0.0))  # parking
    recorder = Recorder(args.record, eye=CLOSE_UP[0], target=CLOSE_UP[1], every_n=8, fps=24)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
robot = Articulation(articulation_root)
arm = robot.get_dof_indices(lime.ARM_JOINTS)
print("[13] ready: ros2 launch lime_isaac_bringup lime_full.launch.py, then ros2 run lime_isaac_bringup mission", flush=True)

# --test: follow the mission from the simulation's point of view (ground truth)
banana = XformPrim("/World/LimeRoom/Objects/Banana")
others = {name: XformPrim(f"/World/LimeRoom/Objects/{name}") for name in ("Mug", "Bottle")}  # must not be touched
start = {}  # where they rest after settling (recorded 1 s into the simulation)
BOWL = np.array(next(o for o in OBJECTS if o[0] == "Bowl")[2:4])
CORNER = np.array([2.8, -1.8])
events, steps, still_since, view_blend, still_frame = {}, 0, None, 0.0, -1
if args.test or args.record:
    print(f"[13] ground truth: banana at {np.round(banana.get_world_poses()[0].numpy()[0, :2], 3)}, bowl at {BOWL}", flush=True)


def event(name, text):
    events[name] = steps / 60
    print(f"[13] {text} at t = {events[name]:.0f} s", flush=True)


while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if recorder:
        if "folded" in events and view_blend < 1.0:  # glide from the close-up to the overview for the parking
            view_blend = min(1.0, view_blend + 1 / 180)
            a = 0.5 - 0.5 * np.cos(np.pi * view_blend)
            recorder.look_at(*[(1 - a) * np.array(c) + a * np.array(o) for c, o in zip(CLOSE_UP, OVERVIEW)])
        recorder.capture()
    if args.test or args.record:
        if steps == 60:
            start = {name: prim.get_world_poses()[0].numpy()[0].copy() for name, prim in others.items()}
        xy = robot.get_world_poses()[0].numpy()[0, :2]
        q = robot.get_dof_positions(dof_indices=arm).numpy()[0]
        for name, prim in others.items():  # report the moment the robot bumps into the mug or the bottle
            if start and f"bumped {name}" not in events and np.linalg.norm(prim.get_world_poses()[0].numpy()[0] - start[name]) > 0.005:
                event(f"bumped {name}", f"{name.lower()} bumped (robot at {np.round(xy, 2)})")
        b = banana.get_world_poses()[0].numpy()[0]
        if "lifted" not in events and b[2] > 0.10:
            event("lifted", f"banana lifted ({b[2] * 100:.0f} cm above the floor)")
        in_bowl = np.linalg.norm(b[:2] - BOWL) < 0.07 and b[2] < 0.10
        still_since = (still_since or steps) if in_bowl else None
        if "lifted" in events and "in_bowl" not in events and still_since and steps - still_since > 60:
            event("in_bowl", f"banana in the bowl, {np.linalg.norm(b[:2] - BOWL) * 100:.1f} cm from its centre")
        if "in_bowl" in events and "folded" not in events and np.max(np.abs(q - np.array(lime.ARM_HOME))) < 0.05:
            event("folded", "arm folded")
            still_frame = len(recorder.frames) - 1 if recorder else -1  # the picture for the docs: banana in the bowl
        if "folded" in events and "parked" not in events and np.linalg.norm(xy - CORNER) < 0.3:
            event("parked", "robot parked in the far corner")
        if ("parked" in events and steps / 60 > events["parked"] + 5) or steps > 500 * 60:
            break  # (5 s more, so the mission can finish cleanly)

ok = True
if args.test:
    b = banana.get_world_poses()[0].numpy()[0]
    checks = {
        "banana picked up (more than 10 cm above the floor)": "lifted" in events,
        "banana put into the bowl (within 7 cm of its centre)": "in_bowl" in events,
        "arm folded afterwards": "folded" in events,
        "robot parked in the far corner (within 0.3 m)": "parked" in events,
        f"banana still in the bowl at the end ({np.linalg.norm(b[:2] - BOWL) * 100:.1f} cm from its centre)": np.linalg.norm(b[:2] - BOWL) < 0.07,
    }
    for name, prim in others.items():
        moved = np.linalg.norm(prim.get_world_poses()[0].numpy()[0] - start[name])
        checks[f"{name.lower()} not bumped into (moved {moved * 100:.1f} cm)"] = moved < 0.01
    for text, passed in checks.items():
        print(f"[13][check] {'ok  ' if passed else 'FAIL'} {text}")
    ok = all(checks.values())
    print(f"[13][test] {'PASS' if ok else 'FAIL'}")
if recorder:
    recorder.save(still_index=still_frame)
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
