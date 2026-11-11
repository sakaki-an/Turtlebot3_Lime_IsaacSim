"""Tutorial 11 (part 2) - Navigate the lime_room with Nav2.

    source scripts/isaac_env.sh
    python isaac/11_make_map.py --headless      # once: the map (part 1)
    python isaac/11_nav2.py                     # with a traffic cone on the way - it is not in the map
    python isaac/11_nav2.py --no-cone           # the same room without the cone: compare the paths

    # ROS 2 container: Nav2 + RViz. Click "Nav2 Goal" in RViz, or:
    ./docker/run_ros.sh ros2 launch lime_isaac_bringup navigation.launch.py
    ./docker/run_ros.sh ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
        "{pose: {header: {frame_id: map}, pose: {position: {x: 3.2, y: -1.8}, orientation: {w: 1.0}}}}"

Isaac Sim provides everything Nav2 needs - nothing new to build, only the pieces of tutorials 06-09:
  /clock, /cmd_vel (subscribed), /odom, /tf (odom -> base_footprint), /joint_states, /scan
The robot starts at (0, 0) facing +x, the pose AMCL is told to start from (config/nav2_params.yaml).
The traffic cone is not in the map: the LiDAR sees it, the costmaps mark it, and the planner goes round it.
"""

import math
import sys

from common.cli import parse_args

args = parse_args(
    __doc__.splitlines()[0],
    extra=lambda p: (
        p.add_argument("--goal", type=float, nargs=2, default=[3.2, -1.8], help="goal x y checked by --test"),
        p.add_argument("--no-cone", action="store_true", help="leave the traffic cone out"),
    ),
)

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
build_lime_room(stage, with_objects=False)  # the room as it was mapped (part 1)
cone = None if args.no_cone else add_traffic_cone(stage)  # something new on the floor: not in the map
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

    recorder = Recorder(args.record, eye=(1.0, -4.6, 4.2), target=(1.0, -0.2, 0.0), every_n=6, fps=20)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
robot = Articulation(articulation_root)
print("[11] ready: start Nav2 (ros2 launch lime_isaac_bringup navigation.launch.py) and send a goal", flush=True)

goal = np.array(args.goal)
steps, reached_at, path = 0, None, []
cone_prim = XformPrim(cone) if cone else None
rest, closest = None, 9.9  # where the cone stands once it has settled; closest approach of the robot


def footprint_distance(robot_pose, point):
    """Distance from a point to the robot's footprint (the Nav2 rectangle), 0 = inside."""
    (x, y, _), (w, qx, qy, qz) = robot_pose
    yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    dx, dy = point[0] - x, point[1] - y
    fx, fy = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
    return math.hypot(max(-0.205 - fx, 0.0, fx - 0.075), max(-0.165 - fy, 0.0, fy - 0.165))
while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if recorder:
        recorder.capture()
    if args.test or args.record:
        pos, quat = robot.get_world_poses()
        xy = pos.numpy()[0, :2]
        if cone_prim and steps == 60:  # the cone has settled
            rest = cone_prim.get_world_poses()[0].numpy()[0].copy()
        if rest is not None and steps % 6 == 0:
            d = footprint_distance((pos.numpy()[0], quat.numpy()[0]), cone_prim.get_world_poses()[0].numpy()[0])
            closest = min(closest, d)
        if steps % 30 == 0:
            path.append(xy.copy())
        if reached_at is None and np.linalg.norm(xy - goal) < 0.25:
            reached_at = steps
            print(f"[11] goal reached after {steps / 60:.1f} s (sim time), {np.linalg.norm(xy - goal):.2f} m from the goal", flush=True)
        if (reached_at is not None and steps - reached_at > 3 * 60) or steps > 240 * 60:
            break

ok = True
if args.test:
    ok = reached_at is not None
    length = sum(np.linalg.norm(b - a) for a, b in zip(path, path[1:]))
    print(f"[11][check] {'ok  ' if ok else 'FAIL'} robot reached ({goal[0]}, {goal[1]}) within 0.25 m (ground truth); driven {length:.2f} m")
    if cone_prim:  # the cone is a rigid body: if the robot touched it, it moved
        moved = np.linalg.norm(cone_prim.get_world_poses()[0].numpy()[0] - rest)
        untouched = moved < 0.01
        ok = ok and untouched
        print(f"[11][check] {'ok  ' if untouched else 'FAIL'} cone not touched (moved {moved * 100:.1f} cm); "
              f"closest approach of the robot's footprint to its centre: {closest * 100:.0f} cm")
    print(f"[11][test] {'PASS' if ok else 'FAIL'}")
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
