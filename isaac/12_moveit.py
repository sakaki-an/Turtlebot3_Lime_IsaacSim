"""Tutorial 12 - Control the TurtleBot3 Lime arm with MoveIt 2 (ros2_control over topics).

    source scripts/isaac_env.sh
    python isaac/12_moveit.py

    # ROS 2 container: ros2_control + MoveIt 2 + RViz (MotionPlanning panel)
    ./docker/run_ros.sh ros2 launch lime_isaac_moveit moveit.launch.py
    # or plan + execute from Python
    ./docker/run_ros.sh ros2 run lime_isaac_moveit arm_demo

How Isaac Sim and ros2_control are connected:

    Isaac Sim                                         ROS 2 (container)
    Isaac Read Joint State -> ROS2 Publish Joint State ---- /isaac_joint_states ---->  JointStateTopicSystem
    Articulation Controller <- ROS2 Subscribe Joint State <- /isaac_joint_commands ---  (ros2_control hardware)
                                                                                         |-- arm_controller (JointTrajectoryController) <- MoveIt
                                                                                         |-- gripper_controller (GripperActionController) <- MoveIt
                                                                                         '-- joint_state_broadcaster -> /joint_states
"""

import sys

from common.cli import parse_args

args = parse_args(__doc__.splitlines()[0])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from common import graphs, lime
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdLux

graphs.enable_ros2_realtime(simulation_app)

stage = stage_utils.create_new_stage(template="sunlight")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(500.0)
GroundPlane("/World/GroundPlane")
articulation_root = lime.add_lime("/World/turtlebot3_lime")

graphs.create_clock_graph()
graphs.create_drive_graph(articulation_root)
# Joint states now go to /isaac_joint_states: ros2_control reads them there, and its
# joint_state_broadcaster becomes the ONE publisher of /joint_states.
graphs.create_odom_tf_joint_states_graph(articulation_root, joint_states_topic="/isaac_joint_states")
graphs.create_joint_command_graph(articulation_root)  # NEW: /isaac_joint_commands -> arm + gripper

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(0.75, -0.75, 0.6), target=(0.0, 0.0, 0.3), every_n=2, fps=30)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
robot = Articulation(articulation_root)
arm_idx = robot.get_dof_indices(lime.ARM_JOINTS)
grip_idx = robot.get_dof_indices([lime.GRIPPER_JOINT])
print("[12] ready: /isaac_joint_states published, /isaac_joint_commands subscribed", flush=True)

# The pose that lime_isaac_moveit's arm_demo plans to (see ros2_ws/src/lime_isaac_moveit/scripts/arm_demo.py)
DEMO_TARGET = np.array([0.0, -0.6, 0.3, 0.0, 1.2, 0.0])
steps, reached_at = 0, None
while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if recorder and steps % 2 == 0:
        recorder.capture()
    if args.test or args.record:
        q = robot.get_dof_positions(dof_indices=arm_idx).numpy()[0]
        g = float(robot.get_dof_positions(dof_indices=grip_idx).numpy()[0, 0])
        if reached_at is None and np.max(np.abs(q - DEMO_TARGET)) < 0.05 and g > 0.015:
            reached_at = steps
            print(f"[12] arm reached the demo pose (max error {np.max(np.abs(q - DEMO_TARGET)):.3f} rad), gripper open {g:.4f} m", flush=True)
        if reached_at is not None and steps - reached_at > 90:
            break
        if steps > 240 * 60:  # give up after 4 minutes
            break

ok = True
if args.test:
    ok = reached_at is not None
    print(f"[12][test] MoveIt plan executed in Isaac Sim: {'PASS' if ok else 'FAIL'}")
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
