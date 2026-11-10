"""Tutorial 04 - Move TurtleBot3 Lime from Python ("Hello Robot").

    source scripts/isaac_env.sh
    python isaac/04_hello_robot.py
    python isaac/04_hello_robot.py --headless --test

No ROS, no graphs: the script talks to the robot directly through the Articulation API.
  * wheels  : a DifferentialController turns (forward speed, turn rate) into wheel speeds
              -> set_dof_velocity_targets()
  * arm     : joint angles -> set_dof_position_targets()
  * gripper : finger position -> set_dof_position_targets() (the second finger follows by itself)

The program: drive 0.6 m forward, turn 90 deg left, raise the arm, open and close the gripper, fold the arm.
"""

import math
import sys

from common.cli import parse_args

args = parse_args(__doc__.splitlines()[0])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from common import lime
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdLux

app_utils.enable_extension("isaacsim.robot.experimental.wheeled_robots")
from isaacsim.robot.experimental.wheeled_robots import DifferentialController

stage = stage_utils.create_new_stage(template="sunlight")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(500.0)
GroundPlane("/World/GroundPlane")
articulation_root = lime.add_lime("/World/turtlebot3_lime")

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(0.2, -1.6, 1.0), target=(0.5, 0.2, 0.2), every_n=3, fps=20)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()

# --- The robot and its joints --------------------------------------------------------------------------------
robot = Articulation(articulation_root)
print(f"[04] {robot.num_dofs} DoF: {robot.dof_names}", flush=True)
wheels = robot.get_dof_indices(lime.WHEEL_JOINTS)
arm = robot.get_dof_indices(lime.ARM_JOINTS)
gripper = robot.get_dof_indices([lime.GRIPPER_JOINT])

controller = DifferentialController(wheel_radius=lime.WHEEL_RADIUS, wheel_base=lime.WHEEL_BASE)

REACH = [0.0, -0.6, 0.3, 0.0, 1.2, 0.0]  # rad: reach forward


def drive(v, w):
    """v: forward speed [m/s], w: turn rate [rad/s] (positive = left)."""
    robot.set_dof_velocity_targets([controller.forward(np.array([v, w]))], dof_indices=wheels)


def yaw():
    w_, x_, y_, z_ = robot.get_world_poses()[1].numpy()[0]
    return math.atan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))


# A tiny "program": (start time [s], action)
program = [
    (0.5, lambda: drive(0.2, 0.0)),  # forward 0.2 m/s for 3 s  -> 0.6 m
    (3.5, lambda: drive(0.0, math.pi / 2 / 2.0)),  # turn left: 90 deg in 2 s
    (5.5, lambda: drive(0.0, 0.0)),
    (6.0, lambda: robot.set_dof_position_targets([REACH], dof_indices=arm)),
    (8.0, lambda: robot.set_dof_position_targets([[lime.GRIPPER_OPEN]], dof_indices=gripper)),
    (9.0, lambda: robot.set_dof_position_targets([[lime.GRIPPER_CLOSED]], dof_indices=gripper)),
    (10.0, lambda: robot.set_dof_position_targets([lime.ARM_HOME], dof_indices=arm)),
]
log = {}
steps = 0
while simulation_app.is_running():
    t = steps / 60.0
    while program and t >= program[0][0]:
        program.pop(0)[1]()
    simulation_app.update()
    steps += 1
    if recorder:
        recorder.capture()
    if steps == int(3.5 * 60):
        log["after_forward"] = robot.get_world_poses()[0].numpy()[0, :2].copy()
    if steps == int(5.9 * 60):
        log["yaw_after_turn"] = math.degrees(yaw())
    if steps == int(7.9 * 60):
        log["arm_reach"] = robot.get_dof_positions(dof_indices=arm).numpy()[0].copy()
    if steps == int(8.9 * 60):
        log["gripper_open"] = float(robot.get_dof_positions(dof_indices=gripper).numpy()[0, 0])
    if (args.test or args.record) and t > 12.0:
        break

ok = True
if args.test:
    arm_err = float(np.max(np.abs(log["arm_reach"] - np.array(REACH))))
    checks = {
        f"drove {log['after_forward'][0]:.2f} m forward (commanded 0.6 m)": abs(log["after_forward"][0] - 0.6) < 0.06,
        f"turned left to {log['yaw_after_turn']:.1f} deg (commanded 90)": abs(log["yaw_after_turn"] - 90.0) < 8.0,
        f"arm reached the reach pose (max error {arm_err:.3f} rad)": arm_err < 0.05,
        f"gripper opened to {log['gripper_open']:.4f} m": log["gripper_open"] > 0.015,
    }
    for text, passed in checks.items():
        print(f"[04][check] {'ok  ' if passed else 'FAIL'} {text}")
    ok = all(checks.values())
    print(f"[04][test] {'PASS' if ok else 'FAIL'}")
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
