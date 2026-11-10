"""Tutorial 10 - Object detection with YOLO on the RealSense images.

    source scripts/isaac_env.sh
    python isaac/10_yolo.py

    # ROS 2 container: the detector, then look at the result
    ./docker/run_ros.sh ros2 launch lime_yolo yolo.launch.py
    ./docker/run_ros.sh ros2 run rqt_image_view rqt_image_view /detections/image
    ./docker/run_ros.sh ros2 topic echo /detections

Isaac Sim does the same as in tutorial 08 (lime_room + robot + camera topics). The new part runs in ROS 2:
the lime_yolo node (ros2_ws/src/lime_yolo) reads /camera/camera/color/image_raw, runs YOLO, and publishes
/detections (vision_msgs/Detection2DArray) and /detections/image (boxes drawn on the image).
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
from common.scene import OBJECTS, build_lime_room
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
graphs.create_camera_graph(sensors.add_d435_camera(robot_path))

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
print("[10] ready: start the detector:  ros2 launch lime_yolo yolo.launch.py", flush=True)

EXPECTED = {coco for *_, coco in OBJECTS}  # cup, banana, bowl, bottle (see common/scene.py)
if args.test or args.record:
    import rclpy
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray

    rclpy.init()
    node = rclpy.create_node("lime_tutorial10_check")
    seen, best, last = {}, {"img": None, "n": -1}, {"classes": set()}

    def on_detections(msg):
        last["classes"] = set()
        for d in msg.detections:
            h = d.results[0].hypothesis
            seen[h.class_id] = max(seen.get(h.class_id, 0.0), h.score)
            last["classes"].add(h.class_id)

    def on_image(msg):  # keep the annotated image showing the most expected objects at once
        n = len(EXPECTED & last["classes"])
        if n >= best["n"]:
            best.update(img=msg, n=n)

    node.create_subscription(Detection2DArray, "/detections", on_detections, 10)
    node.create_subscription(Image, "/detections/image", on_image, 1)

steps, done_at = 0, None
while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if args.test or args.record:
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.0)
        if done_at is None and len(EXPECTED & set(seen)) == len(EXPECTED):
            done_at = steps
        if (done_at is not None and steps - done_at > 120) or steps > 180 * 60:  # all found, or 3 min
            break

ok = True
if args.test or args.record:
    found = EXPECTED & set(seen)
    for cls in sorted(EXPECTED):
        print(f"[10][check] {'ok  ' if cls in seen else 'FAIL'} detected '{cls}'" + (f" (best score {seen[cls]:.2f})" if cls in seen else ""))
    others = sorted(set(seen) - EXPECTED)
    if others:
        print(f"[10][check] other classes reported: {others}")
    ok = len(found) >= 3
    if args.test:
        print(f"[10][test] {len(found)}/{len(EXPECTED)} objects detected: {'PASS' if ok else 'FAIL'}")
    if args.record and best["img"] is not None:
        import imageio.v2 as imageio
        from common.paths import IMG_DIR

        m = best["img"]
        imageio.imwrite(IMG_DIR / f"{args.record}_detections.png", np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3))
        print(f"[10] saved {IMG_DIR / (args.record + '_detections.png')}")
    node.destroy_node()
    rclpy.shutdown()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
