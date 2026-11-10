"""Tutorial 11 (part 2) - Navigate the lime_room with Nav2.

    source scripts/isaac_env.sh
    python isaac/11_make_map.py --headless      # once: the map (part 1)
    python isaac/11_nav2.py

    # ROS 2 container: Nav2 + RViz. Click "Nav2 Goal" in RViz, or:
    ./docker/run_ros.sh ros2 launch lime_isaac_bringup navigation.launch.py
    ./docker/run_ros.sh ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
        "{pose: {header: {frame_id: map}, pose: {position: {x: 3.2, y: -1.8}, orientation: {w: 1.0}}}}"

Isaac Sim provides everything Nav2 needs - nothing new to build, only the pieces of tutorials 06-09:
  /clock, /cmd_vel (subscribed), /odom, /tf (odom -> base_footprint), /joint_states, /scan
The robot starts at (0, 0) facing +x, the pose AMCL is told to start from (config/nav2_params.yaml).
"""

import math
import sys

from common.cli import parse_args

args = parse_args(
    __doc__.splitlines()[0],
    extra=lambda p: p.add_argument("--goal", type=float, nargs=2, default=[3.2, -1.8], help="goal x y checked by --test"),
)

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
while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if recorder:
        recorder.capture()
    if args.test or args.record:
        xy = robot.get_world_poses()[0].numpy()[0, :2]
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
    print(f"[11][test] {'PASS' if ok else 'FAIL'}")
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
