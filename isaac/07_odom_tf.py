"""Tutorial 07 - Odometry, TF and joint states: see TurtleBot3 Lime in RViz.

    source scripts/isaac_env.sh
    python isaac/07_odom_tf.py

    # ROS 2 container: robot_state_publisher + RViz
    ./docker/run_ros.sh ros2 launch lime_isaac_bringup view_robot.launch.py
    # and drive it
    ./docker/run_ros.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard

New in this tutorial (graph "ROS2_OdomTF"):

    On Playback Tick --> Isaac Compute Odometry (base_link) --+--> ROS2 Publish Odometry      /odom
                                                              +--> ROS2 Publish Raw Transform  /tf  odom -> base_footprint
                     --> Isaac Read Joint State (robot) ------> ROS2 Publish Joint State     /joint_states

robot_state_publisher (ROS side) turns /joint_states + the URDF into the rest of the TF tree:
base_footprint -> base_link -> wheels, arm links, base_scan, camera frames, ...
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
from common import graphs, lime
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdLux

graphs.enable_ros2_realtime(simulation_app)  # ROS 2 bridge + 60 Hz main loop (tutorial 06)

# --- Scene -----------------------------------------------------------------------------------------------
stage = stage_utils.create_new_stage(template="sunlight")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(500.0)
GroundPlane("/World/GroundPlane")
for i, (x, y) in enumerate([(1.5, 0.0), (0.0, 1.5), (-1.5, 0.0), (0.0, -1.5)]):
    Cube(f"/World/Landmark_{i}", sizes=0.2, positions=[x, y, 0.1], colors=[0.6, 0.85, 0.2])
articulation_root = lime.add_lime("/World/turtlebot3_lime")

# --- Graphs ------------------------------------------------------------------------------------------------
graphs.create_clock_graph()  # /clock            (tutorial 06)
graphs.create_drive_graph(articulation_root)  # /cmd_vel -> wheels (tutorial 06)
graphs.create_odom_tf_joint_states_graph(articulation_root)  # NEW: /odom, /tf, /joint_states (see common/graphs.py)

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(2.0, -2.0, 1.5), target=(0.3, 0.3, 0.1), every_n=3, fps=20)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
robot = Articulation(articulation_root)
print("[07] ready: /odom, /tf (odom->base_footprint), /joint_states and /clock are published", flush=True)


def yaw_of(q_wxyz):
    w, x, y, z = q_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


# --- Self-check (--test): drive a known twist and compare /odom with the commanded motion and the truth ---
if args.test or args.record:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node("lime_tutorial07_check")
    seen = {"odom": None, "joints": None, "tf": False}
    node.create_subscription(Odometry, "/odom", lambda m: seen.__setitem__("odom", m), 10)
    node.create_subscription(JointState, "/joint_states", lambda m: seen.__setitem__("joints", m), 10)

    def on_tf(msg):
        if any(t.header.frame_id == "odom" and t.child_frame_id == "base_footprint" for t in msg.transforms):
            seen["tf"] = True

    node.create_subscription(TFMessage, "/tf", on_tf, 10)
    cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    V, W = 0.2, 0.5  # commanded: 0.2 m/s forward, 0.5 rad/s counter-clockwise (left turn)
    start_pose = robot.get_world_poses()
    start_xy, start_yaw = start_pose[0].numpy()[0, :2].copy(), yaw_of(start_pose[1].numpy()[0])

steps = 0
while simulation_app.is_running():
    if args.test or args.record:
        if 60 <= steps < 60 + 4 * 60:  # drive for 4 s
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = V, W
            cmd_pub.publish(cmd)
        elif steps == 60 + 4 * 60:
            cmd_pub.publish(Twist())  # stop
            odom_while_driving = seen["odom"]
            truth = robot.get_world_poses()
        for _ in range(20):  # handle every message that arrived (3 topics at 60 Hz), not just one
            rclpy.spin_once(node, timeout_sec=0.0)
    simulation_app.update()
    if recorder:
        recorder.capture()
    steps += 1
    if (args.test or args.record) and steps > 60 + 4 * 60 + 30:
        break

ok = True
if args.test:
    o = odom_while_driving
    v_meas, w_meas = o.twist.twist.linear.x, o.twist.twist.angular.z
    q = o.pose.pose.orientation
    odom_yaw = yaw_of((q.w, q.x, q.y, q.z))
    odom_xy = np.array([o.pose.pose.position.x, o.pose.pose.position.y])
    true_xy = truth[0].numpy()[0, :2] - start_xy
    true_yaw = yaw_of(truth[1].numpy()[0]) - start_yaw
    names = set(seen["joints"].name) if seen["joints"] else set()
    expected_joints = set(lime.WHEEL_JOINTS + lime.ARM_JOINTS + [lime.GRIPPER_JOINT, lime.GRIPPER_MIMIC_JOINT])
    checks = {
        f"odom twist v={v_meas:.3f} m/s (cmd {V})": abs(v_meas - V) < 0.15 * V,
        f"odom twist w={w_meas:.3f} rad/s (cmd {W})": abs(w_meas - W) < 0.15 * W,
        f"left turn -> yaw increased: odom yaw {math.degrees(odom_yaw):.1f} deg": odom_yaw > 0.5,
        f"odom pose vs truth: |dxy|={np.linalg.norm(odom_xy - true_xy):.3f} m, dyaw={math.degrees(odom_yaw - true_yaw):.2f} deg": np.linalg.norm(odom_xy - true_xy) < 0.02 and abs(odom_yaw - true_yaw) < 0.02,
        f"/joint_states has all {len(expected_joints)} joints": expected_joints <= names,
        "TF odom -> base_footprint received": seen["tf"],
    }
    for text, passed in checks.items():
        print(f"[07][check] {'ok  ' if passed else 'FAIL'} {text}")
    ok = all(checks.values())
    print(f"[07][test] {'PASS' if ok else 'FAIL'}")
if args.test or args.record:
    node.destroy_node()
    rclpy.shutdown()
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
