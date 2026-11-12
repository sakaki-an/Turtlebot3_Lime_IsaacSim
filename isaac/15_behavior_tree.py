"""Tutorial 15 - Behavior trees: a patrol with Nav2, interrupted by a (pretend) low battery.

    source scripts/isaac_env.sh
    python isaac/15_behavior_tree.py

    # ROS 2 container: Nav2 + RViz (tutorial 11), then the patrol - a behavior tree
    ./docker/run_ros.sh ros2 launch lime_isaac_bringup navigation.launch.py
    ./docker/run_ros.sh ros2 run lime_isaac_bringup patrol

The world of tutorial 11: the lime_room as it was mapped, plus the traffic cone that is not in the map.
Nothing new on the Isaac Sim side: the behavior tree runs in ROS 2 and only sends Nav2 goals.
--test follows the patrol with the simulation's ground truth: the four lookouts in order, one stop at the
charger, back home, and the cone not touched.
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
from common.scene import add_traffic_cone, build_lime_room
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.simulation_manager import SimulationManager

graphs.enable_ros2_realtime(simulation_app)

stage = stage_utils.create_new_stage(template="empty")
build_lime_room(stage, with_objects=False)  # tutorial 11: the room as it was mapped
cone = add_traffic_cone(stage)  # ... and the traffic cone that is not in the map
while stage_utils.is_stage_loading():
    simulation_app.update()
robot_path = "/World/turtlebot3_lime"
articulation_root = lime.add_lime(robot_path)

graphs.create_clock_graph()  # 06
graphs.create_drive_graph(articulation_root)  # 06  /cmd_vel
graphs.create_odom_tf_joint_states_graph(articulation_root)  # 07  /odom /tf /joint_states
graphs.create_lidar_graph(sensors.add_lds_lidar(robot_path))  # 09  /scan

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(1.0, -4.6, 4.6), target=(0.9, 0.0, 0.0), every_n=12, fps=20)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
robot = Articulation(articulation_root)
print("[15] ready: start Nav2 (ros2 launch lime_isaac_bringup navigation.launch.py), then ros2 run lime_isaac_bringup patrol", flush=True)

# the places of the patrol (patrol.py): where the robot must stop, in this order (the charger somewhere in between)
PLACES = {"lookout 1": (3.2, -1.8), "lookout 2": (3.3, 1.9), "lookout 3": (0.5, 1.9), "lookout 4": (-1.3, -1.8), "charger": (-1.5, 0.0)}
HOME = np.array([0.0, 0.0])
visits, steps, here, still = [], 0, None, 0
cone_prim = XformPrim(cone)
rest = None
while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if recorder:
        recorder.capture()
    if args.test or args.record:
        if steps == 60:  # the cone has settled
            rest = cone_prim.get_world_poses()[0].numpy()[0].copy()
        xy = robot.get_world_poses()[0].numpy()[0, :2]
        near = next((name for name, p in PLACES.items() if np.linalg.norm(xy - p) < 0.3), None)
        if near and near != here and visits[-1:] != [near]:  # came to a place
            visits.append(near)
            print(f"[15] at {near}, t = {steps / 60:.0f} s", flush=True)
        here = near
        speed = np.linalg.norm(robot.get_velocities()[0].numpy()[0, :2])
        home = len([v for v in visits if v.startswith("lookout")]) == 4 and np.linalg.norm(xy - HOME) < 0.3
        still = still + 1 if home and speed < 0.02 else 0
        if still == 60:
            print(f"[15] back home, t = {steps / 60:.0f} s", flush=True)
        if still > 180 or steps > 600 * 60:
            break

ok = True
if args.test:
    lookouts = [v for v in visits if v.startswith("lookout")]
    checks = {
        f"the four lookouts in order ({', '.join(visits)})": lookouts == ["lookout 1", "lookout 2", "lookout 3", "lookout 4"],
        "one stop at the charger, between two lookouts": visits.count("charger") == 1 and 0 < visits.index("charger") < len(visits) - 1,
        "back home at the end": still > 180,
    }
    moved = np.linalg.norm(cone_prim.get_world_poses()[0].numpy()[0] - rest)
    checks[f"cone not touched (moved {moved * 100:.1f} cm)"] = moved < 0.01
    for text, passed in checks.items():
        print(f"[15][check] {'ok  ' if passed else 'FAIL'} {text}")
    ok = all(checks.values())
    print(f"[15][test] {'PASS' if ok else 'FAIL'}")
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
