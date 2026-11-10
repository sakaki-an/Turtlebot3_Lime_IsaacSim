"""Tutorial 06 - Drive TurtleBot3 Lime from ROS 2 (/cmd_vel) with an Action Graph.

    source scripts/isaac_env.sh
    python isaac/06_ros2_drive.py

    # in another terminal (ROS 2 Jazzy container):
    ./docker/run_ros.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard

The Action Graph built here:

    On Playback Tick --+--> ROS2 Publish Clock  <-- Isaac Read Simulation Time      (/clock)
                       |
                       +--> ROS2 Subscribe Twist (/cmd_vel)
                       |        |-- linear  --> Break Vector3 --x--+
                       |        |-- angular --> Break Vector3 --z--+--> Differential Controller
                       |                                                  | wheel velocities
                       +--> Articulation Controller  <--------------------+
                                (wheel_left_joint, wheel_right_joint)
"""

import sys

from common.cli import parse_args

args = parse_args(__doc__.splitlines()[0])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from common import lime
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdLux

# --- 1. ROS 2 bridge + real-time pacing ------------------------------------------------------------
# The ROS 2 bridge is an extension. It uses Isaac Sim's bundled ROS 2 Jazzy libraries
# (scripts/isaac_env.sh puts them on LD_LIBRARY_PATH before Isaac Sim starts).
app_utils.enable_extension("isaacsim.ros2.bridge")
# Standalone scripts run "as fast as possible". ROS tools expect roughly real time, so cap the
# main loop at 60 Hz (= one 1/60 s physics step per frame).
settings = carb.settings.get_settings()
settings.set("/app/runLoops/main/rateLimitEnabled", True)
settings.set("/app/runLoops/main/rateLimitFrequency", 60)
simulation_app.update()

import omni.graph.core as og
import usdrt.Sdf

# --- 2. Scene -------------------------------------------------------------------------------------------
stage = stage_utils.create_new_stage(template="sunlight")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(500.0)
GroundPlane("/World/GroundPlane")
for i, (x, y) in enumerate([(1.5, 0.0), (0.0, 1.5), (-1.5, 0.0), (0.0, -1.5)]):  # landmarks
    Cube(f"/World/Landmark_{i}", sizes=0.2, positions=[x, y, 0.1], colors=[0.6, 0.85, 0.2])
robot_path = "/World/turtlebot3_lime"
articulation_root = lime.add_lime(robot_path)

# --- 3. The Action Graph ------------------------------------------------------------------------------
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/ROS2_Drive", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("BreakLinear", "omni.graph.nodes.BreakVector3"),
            ("BreakAngular", "omni.graph.nodes.BreakVector3"),
            ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "/clock"),
            ("SubscribeTwist.inputs:topicName", "/cmd_vel"),
            ("DiffController.inputs:wheelRadius", lime.WHEEL_RADIUS),
            ("DiffController.inputs:wheelDistance", lime.WHEEL_BASE),
            ("DiffController.inputs:maxWheelSpeed", lime.MAX_WHEEL_SPEED),
            ("ArticulationController.inputs:targetPrim", [usdrt.Sdf.Path(articulation_root)]),
            ("ArticulationController.inputs:jointNames", lime.WHEEL_JOINTS),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
            ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
            ("BreakLinear.outputs:x", "DiffController.inputs:linearVelocity"),
            ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
            ("BreakAngular.outputs:z", "DiffController.inputs:angularVelocity"),
            ("DiffController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
        ],
    },
)

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(2.2, -2.2, 1.6), target=(0.0, 0.0, 0.1), every_n=3, fps=20)

# --- 4. Run -----------------------------------------------------------------------------------------------
SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
robot = Articulation(articulation_root)
start = robot.get_world_poses()[0].numpy()[0, :2].copy()
print("[06] ready: publish geometry_msgs/Twist on /cmd_vel to drive the robot", flush=True)

steps, moved, first_move_step = 0, 0.0, None
while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if not (args.test or args.record):
        continue  # interactive: run until you close the window
    # --test / --record: wait (max 60 s) for /cmd_vel commands, then watch the robot for 10 s
    moved = float(np.linalg.norm(robot.get_world_poses()[0].numpy()[0, :2] - start))
    if first_move_step is None and moved > 0.05:
        first_move_step = steps
    if first_move_step is None and steps > 60 * 60:
        break  # nobody published /cmd_vel
    if first_move_step is not None:
        if recorder:
            recorder.capture()
        if steps - first_move_step > 10 * 60:
            break

ok = True
if args.test:
    ok = moved > 0.3
    print(f"[06][test] robot moved {moved:.2f} m after /cmd_vel commands -> {'PASS' if ok else 'FAIL'}")
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
