"""Tutorial 08 - RealSense D435 camera: RGB, depth and camera_info on ROS 2.

    source scripts/isaac_env.sh
    python isaac/08_camera.py

    # ROS 2 container: look at the images
    ./docker/run_ros.sh ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw

The robot stands in the "lime_room" (isaac/common/scene.py) in front of a cup, a banana, a bowl and a
bottle - the objects tutorial 10 will detect with YOLO.

New in this tutorial:
  * a USD camera at the URDF frame camera_color_optical_frame (640 x 480, 69.4 deg FOV, like the D435)
  * graph "ROS2_Camera":  On Playback Tick -> Isaac Create Render Product -> ROS2 Camera Helper (rgb)
                                                                         -> ROS2 Camera Helper (depth)
                                                                         -> ROS2 Camera Info Helper
    topics (same names as realsense2_camera on the real robot):
      /camera/camera/color/image_raw                   sensor_msgs/Image (rgb8)
      /camera/camera/aligned_depth_to_color/image_raw  sensor_msgs/Image (32FC1, metres)
      /camera/camera/color/camera_info                 sensor_msgs/CameraInfo
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
while stage_utils.is_stage_loading():  # the objects are downloaded from NVIDIA's asset server
    simulation_app.update()
robot_path = "/World/turtlebot3_lime"
articulation_root = lime.add_lime(robot_path)

graphs.create_clock_graph()
graphs.create_drive_graph(articulation_root)
graphs.create_odom_tf_joint_states_graph(articulation_root)

# --- NEW: camera + its ROS 2 graph ---------------------------------------------------------------------
camera_path = sensors.add_d435_camera(robot_path)
graphs.create_camera_graph(camera_path, sensors.CAMERA_WIDTH, sensors.CAMERA_HEIGHT)

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(-0.9, -1.0, 0.8), target=(0.7, 0.0, 0.05), every_n=3, fps=20)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
print("[08] ready: camera topics on /camera/camera/...", flush=True)

if args.test or args.record:
    import rclpy
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = rclpy.create_node("lime_tutorial08_check")
    got = {"rgb": [], "depth": None, "info": None}
    node.create_subscription(Image, "/camera/camera/color/image_raw", lambda m: got["rgb"].append(m), 10)
    node.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", lambda m: got.__setitem__("depth", m), 10)
    node.create_subscription(CameraInfo, "/camera/camera/color/camera_info", lambda m: got.__setitem__("info", m), 10)

steps = 0
while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if recorder:
        recorder.capture()
    if args.test or args.record:
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.0)
        if steps > 5 * 60:  # 5 s
            break

ok = True
if args.test or args.record:
    rgb, depth, info = got["rgb"][-1], got["depth"], got["info"]
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in got["rgb"]]
    rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 2 and stamps[-1] > stamps[0] else 0.0
    image = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, 3)
    d = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    center = d[depth.height // 2 - 20 : depth.height // 2 + 20, depth.width // 2 - 20 : depth.width // 2 + 20]
    center_m = float(np.nanmedian(center[np.isfinite(center)]))
    fx_expected = sensors.CAMERA_WIDTH / (2 * math.tan(sensors.CAMERA_HFOV / 2))
    checks = {
        f"rgb {rgb.width}x{rgb.height} {rgb.encoding}": (rgb.width, rgb.height, rgb.encoding) == (640, 480, "rgb8"),
        f"rgb rate {rate:.1f} Hz (sim time)": rate > 10,
        f"depth {depth.encoding}, centre distance {center_m:.2f} m": depth.encoding == "32FC1" and 0.2 < center_m < 6.0,
        f"camera_info fx={info.k[0]:.1f} (expected {fx_expected:.1f} from the 69.4 deg FOV)": abs(info.k[0] - fx_expected) < 2.0,
        f"frame_id {rgb.header.frame_id}": rgb.header.frame_id == "camera_color_optical_frame",
    }
    for text, passed in checks.items():
        print(f"[08][check] {'ok  ' if passed else 'FAIL'} {text}")
    ok = all(checks.values())
    if args.test:
        print(f"[08][test] {'PASS' if ok else 'FAIL'}")
    if args.record:  # what the robot sees -> docs
        import imageio.v2 as imageio
        from common.paths import IMG_DIR

        imageio.imwrite(IMG_DIR / f"{args.record}_rgb.png", image)
        import matplotlib

        valid = np.isfinite(d) & (d > 0)  # no surface hit (the "sky") -> black
        vis = np.clip(np.where(valid, d, 0.0) / 4.0, 0.0, 1.0)  # 0 .. 4 m
        colored = (matplotlib.colormaps["turbo"](vis)[..., :3] * 255 * valid[..., None]).astype(np.uint8)
        imageio.imwrite(IMG_DIR / f"{args.record}_depth.png", colored)
    node.destroy_node()
    rclpy.shutdown()
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
