"""Tutorial 09 - 2D LiDAR (LDS-01) and IMU: /scan and /imu on ROS 2.

    source scripts/isaac_env.sh
    python isaac/09_lidar_imu.py

    # ROS 2 container: robot model + laser scan + camera in RViz
    ./docker/run_ros.sh ros2 launch lime_isaac_bringup view_sensors.launch.py

New in this tutorial:
  * an RTX LiDAR at base_scan, configured like the Lime's LDS-01 (360 points / turn, 5 Hz, 0.12 - 3.5 m)
      graph "ROS2_Lidar": On Playback Tick -> Isaac Create Render Product -> ROS2 RTX Lidar Helper (laser_scan)
      topic /scan  sensor_msgs/LaserScan  frame base_scan
  * a physics IMU on base_link (at imu_link)
      graph "ROS2_Imu": On Playback Tick -> Isaac Read IMU Node -> ROS2 Publish Imu
      topic /imu   sensor_msgs/Imu        frame imu_link
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
from common import graphs, lime, sensors
from common.scene import build_lime_room
from isaacsim.core.simulation_manager import SimulationManager

graphs.enable_ros2_realtime(simulation_app)

stage = stage_utils.create_new_stage(template="empty")
build_lime_room(stage)
while stage_utils.is_stage_loading():
    simulation_app.update()
robot_path = "/World/turtlebot3_lime"
articulation_root = lime.add_lime(robot_path)

graphs.create_clock_graph()
graphs.create_drive_graph(articulation_root)
graphs.create_odom_tf_joint_states_graph(articulation_root)
graphs.create_camera_graph(sensors.add_d435_camera(robot_path))  # tutorial 08

# --- NEW: LiDAR + IMU and their ROS 2 graphs --------------------------------------------------------------
lidar_path = sensors.add_lds_lidar(robot_path)
# show_debug_view: with a window, the LiDAR helper also draws the hits as dots in the viewport
graphs.create_lidar_graph(lidar_path, show_debug_view=not args.headless)
imu_path = sensors.add_imu(robot_path)
graphs.create_imu_graph(imu_path)

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(-1.6, -1.8, 2.2), target=(0.4, 0.0, 0.0), every_n=3, fps=20)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
print("[09] ready: /scan and /imu published", flush=True)

if args.test or args.record:
    import rclpy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu, LaserScan

    rclpy.init()
    node = rclpy.create_node("lime_tutorial09_check")
    got = {"scans": [], "imu": None, "imu_turn": None}
    node.create_subscription(LaserScan, "/scan", lambda m: got["scans"].append(m), 10)
    node.create_subscription(Imu, "/imu", lambda m: got.__setitem__("imu", m), 10)
    cmd = node.create_publisher(Twist, "/cmd_vel", 10)

steps = 0
imu_rest = None
while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if recorder:
        recorder.capture()
    if args.test or args.record:
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.0)
        if steps == 3 * 60:  # t = 3 s, at rest: remember IMU + scan, then turn on the spot
            imu_rest, scan_rest = got["imu"], got["scans"][-1]
        if 3 * 60 < steps < 6 * 60:
            t = Twist()
            t.angular.z = 0.5
            cmd.publish(t)
        if steps == 5 * 60 + 30:
            got["imu_turn"] = got["imu"]
        if steps == 6 * 60:
            cmd.publish(Twist())
        if steps > 6 * 60 + 30:
            break

ok = True
if args.test or args.record:
    s = scan_rest
    ranges = np.array(s.ranges)
    angles = s.angle_min + np.arange(len(ranges)) * s.angle_increment

    def range_at(deg, window_deg=3.0):
        """Median of the valid beams within +-window_deg: the LiDAR model is noisy (~3 cm), like a real one."""
        diff = np.abs(np.angle(np.exp(1j * (angles - math.radians(deg)))))
        sel = ranges[(diff <= math.radians(window_deg)) & (ranges >= s.range_min) & (ranges <= s.range_max)]
        return float(np.median(sel)) if len(sel) else -1.0

    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in got["scans"]]
    rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 2 else 0.0
    a, w = imu_rest.linear_acceleration, got["imu_turn"].angular_velocity
    checks = {
        f"scan: {len(ranges)} beams, {math.degrees(s.angle_increment):.2f} deg apart": 340 <= len(ranges) <= 380,
        f"scan range limits {s.range_min:.2f} .. {s.range_max:.2f} m": abs(s.range_min - 0.12) < 0.01 and abs(s.range_max - 3.5) < 0.01,
        f"scan rate {rate:.1f} Hz (sim time)": 4.0 < rate < 6.0,
        f"left wall  (+90 deg) {range_at(90):.2f} m (expected 2.50)": abs(range_at(90) - 2.50) < 0.06,
        f"right wall (-90 deg) {range_at(-90):.2f} m (expected 2.50)": abs(range_at(-90) - 2.50) < 0.06,
        f"back wall  (180 deg) {range_at(180):.2f} m (expected 2.03)": abs(range_at(180) - 2.03) < 0.06,
        f"front (0 deg): east wall at 3.97 m is beyond 3.5 m -> {range_at(0)}": not (0.12 <= range_at(0) <= 3.5),
        f"imu at rest: acc z = {a.z:.2f} m/s^2 (gravity 9.81)": abs(a.z - 9.81) < 0.3,
        f"imu turning: gyro z = {w.z:.2f} rad/s (commanded 0.50)": abs(w.z - 0.5) < 0.08,
        f"frames {s.header.frame_id} / {imu_rest.header.frame_id}": (s.header.frame_id, imu_rest.header.frame_id) == ("base_scan", "imu_link"),
    }
    for text, passed in checks.items():
        print(f"[09][check] {'ok  ' if passed else 'FAIL'} {text}")
    ok = all(checks.values())
    if args.test:
        print(f"[09][test] {'PASS' if ok else 'FAIL'}")
    node.destroy_node()
    rclpy.shutdown()
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
