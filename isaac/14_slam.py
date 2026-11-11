"""Tutorial 14 - Make your own map with SLAM (slam_toolbox).

    source scripts/isaac_env.sh
    python isaac/14_slam.py                  # you drive (teleop_twist_keyboard in the container)
    python isaac/14_slam.py --auto-drive     # the script drives a tour of the room for you

    # ROS 2 container: slam_toolbox + RViz, drive, then save the map
    ./docker/run_ros.sh ros2 launch lime_isaac_bringup slam.launch.py
    ./docker/run_ros.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ./docker/run_ros.sh bash -c 'mkdir -p /lime/my_maps && ros2 run nav2_map_server map_saver_cli -f /lime/my_maps/lime_room_slam'

Isaac Sim publishes exactly what it published for Nav2 in tutorial 11:
  /clock, /cmd_vel (subscribed), /odom, /tf (odom -> base_footprint), /joint_states, /scan
The difference is on the ROS side: nobody gives slam_toolbox a map - it builds one from /scan and /odom
and publishes /map and the TF map -> odom. slam.launch.py also starts scan_fix: Isaac Sim marks "no return"
with -1, which slam_toolbox would turn into phantom points (see the tutorial page).
The four objects are cleared off the floor first: a map is for what stays put, and SLAM can't tell a mustard
bottle from a pillar.
"""

import math
import sys

from common.cli import parse_args

args = parse_args(
    __doc__.splitlines()[0],
    extra=lambda p: p.add_argument("--auto-drive", action="store_true", help="drive a tour of the room (on with --test)"),
)
auto_drive = args.auto_drive or args.test or bool(args.record)

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from common import graphs, lime, sensors
from common.scene import OBSTACLES, ROOM_X, ROOM_Y, build_lime_room
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager

graphs.enable_ros2_realtime(simulation_app)

# --- The same robot and graphs as tutorial 11: SLAM needs nothing new from Isaac Sim ---
stage = stage_utils.create_new_stage(template="empty")
build_lime_room(stage, with_objects=False)  # clear the floor first: SLAM would map the bottle like a pillar
while stage_utils.is_stage_loading():
    simulation_app.update()
robot_path = "/World/turtlebot3_lime"
articulation_root = lime.add_lime(robot_path)

graphs.create_clock_graph()  # 06  /clock
graphs.create_drive_graph(articulation_root)  # 06  /cmd_vel
graphs.create_odom_tf_joint_states_graph(articulation_root)  # 07  /odom /tf /joint_states
graphs.create_lidar_graph(sensors.add_lds_lidar(robot_path))  # 09  /scan

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(1.0, -2.6, 7.4), target=(1.0, 0.05, 0.0), resolution=(720, 720), every_n=12, fps=30)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
robot = Articulation(articulation_root)


def pose2d():
    """Ground truth (x, y, yaw) of the robot in the world, straight from the simulation."""
    pos, quat = robot.get_world_poses()
    (x, y, _), (w, qx, qy, qz) = pos.numpy()[0], quat.numpy()[0]
    return x, y, math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# --8<-- [start:route]
# A tour of the lime_room (world = map coordinates, metres). It passes every obstacle on both sides.
ROUTE = [(0.3, -0.9), (1.3, -1.9), (3.2, -1.8), (3.3, 0.2), (3.4, 1.9), (1.8, 1.95), (1.7, 0.7), (-0.8, 1.2), (-0.9, -1.2), (0.0, -0.4)]
V_MAX, W_MAX = 0.2, 0.2  # drive and turn slowly: the LiDAR needs 0.2 s for one turn (5 Hz)


def drive_command(x, y, yaw, goal):
    """Very simple waypoint follower: turn towards the goal, then drive (it uses ground truth - this is
    only the "driver" of the robot; slam_toolbox never sees it)."""
    dx, dy = goal[0] - x, goal[1] - y
    error = wrap(math.atan2(dy, dx) - yaw)
    w = max(-W_MAX, min(W_MAX, 1.5 * error))
    v = 0.0 if abs(error) > 0.35 else min(V_MAX, 0.6 * math.hypot(dx, dy) + 0.05)
    return v, w
# --8<-- [end:route]


# --8<-- [start:map_check]
def occupancy_from_msg(msg):
    """nav_msgs/OccupancyGrid -> (occupied bool array, origin x, origin y, resolution). Row 0 = lowest y."""
    info = msg.info
    data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
    return data >= 65, info.origin.position.x, info.origin.position.y, info.resolution


def occupancy_from_pgm(yaml_path):
    """map_server files (.yaml + .pgm) -> the same tuple. PGM rows go top-down, so flip them."""
    import yaml
    from PIL import Image

    meta = yaml.safe_load(yaml_path.read_text())
    pixels = np.flipud(np.asarray(Image.open(yaml_path.parent / meta["image"])))
    free = pixels > 250
    return pixels < 50, meta["origin"][0], meta["origin"][1], meta["resolution"], free


def dilate(mask, cells):
    out = mask.copy()
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            out |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    return out


def lookup(mask, ox, oy, res, xy):
    """mask value at world points xy (N x 2); False outside the map."""
    col = np.floor((xy[:, 0] - ox) / res).astype(int)
    row = np.floor((xy[:, 1] - oy) / res).astype(int)
    inside = (col >= 0) & (row >= 0) & (col < mask.shape[1]) & (row < mask.shape[0])
    out = np.zeros(len(xy), dtype=bool)
    out[inside] = mask[row[inside], col[inside]]
    return out


def cell_centres(mask, ox, oy, res):
    rows, cols = np.nonzero(mask)
    return np.stack([ox + (cols + 0.5) * res, oy + (rows + 0.5) * res], axis=1)


def cell_grid(mask, ox, oy, res):
    """world x and y of every cell of the map (two arrays shaped like the map)"""
    rows, cols = np.indices(mask.shape)
    return ox + (cols + 0.5) * res, oy + (rows + 0.5) * res


def compare_maps(slam, truth, tolerance=0.10):
    """How well does the SLAM map match Isaac Sim's map (tutorial 11)?
    precision: SLAM wall cells that are on a real wall (within tolerance)
    recall:    real wall faces (occupied cells next to the room's free space) that SLAM found (within tolerance)"""
    s_occ, s_ox, s_oy, res = slam
    t_occ, t_ox, t_oy, _, t_free = truth
    n = int(round(tolerance / res))
    slam_xy = cell_centres(s_occ, s_ox, s_oy, res)
    precision = lookup(dilate(t_occ, n), t_ox, t_oy, res, slam_xy).mean()
    x, y = cell_grid(t_occ, t_ox, t_oy, res)
    room = (x > ROOM_X[0]) & (x < ROOM_X[1]) & (y > ROOM_Y[0]) & (y < ROOM_Y[1])
    faces = t_occ & dilate(t_free & room, 1)  # the side of each wall that faces the room: what the LiDAR can see
    face_xy = cell_centres(faces, t_ox, t_oy, res)
    recall = lookup(dilate(s_occ, n), s_ox, s_oy, res, face_xy).mean()
    return precision, recall, slam_xy
# --8<-- [end:map_check]


def render_map(occ_msg, size=720, window=((-2.3, 4.3), (-3.3, 3.3))):
    """Draw an OccupancyGrid like RViz does (grey unknown, white free, black occupied) on a fixed window."""
    (x0, x1), (y0, y1) = window
    img = np.full((size, size), 205, np.uint8)
    if occ_msg is not None:
        info = occ_msg.info
        data = np.asarray(occ_msg.data, dtype=np.int16).reshape(info.height, info.width)
        px = np.arange(size)
        wx = x0 + (px + 0.5) * (x1 - x0) / size
        wy = y1 - (px + 0.5) * (y1 - y0) / size
        col = np.floor((wx - info.origin.position.x) / info.resolution).astype(int)
        row = np.floor((wy - info.origin.position.y) / info.resolution).astype(int)
        ok_c, ok_r = (col >= 0) & (col < info.width), (row >= 0) & (row < info.height)
        sub = data[np.clip(row, 0, info.height - 1)][:, np.clip(col, 0, info.width - 1)]
        cell = np.full(sub.shape, 205, np.uint8)
        cell[sub == 0] = 254
        cell[sub >= 65] = 0
        mask = ok_r[:, None] & ok_c[None, :]
        img[mask] = cell[mask]
    return img


def to_px(x, y, size=720, window=((-2.3, 4.3), (-3.3, 3.3))):
    (x0, x1), (y0, y1) = window
    return (x - x0) / (x1 - x0) * size, (y1 - y) / (y1 - y0) * size


print("[14] ready: start slam_toolbox (ros2 launch lime_isaac_bringup slam.launch.py), then drive", flush=True)

node = None
if auto_drive or args.test:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node("lime_tutorial14_driver")
    cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    latest = {"map": None, "maps": 0, "map_to_odom": None, "odom": None}

    def on_map(msg):
        latest["map"] = msg
        latest["maps"] += 1

    def on_tf(msg):
        for t in msg.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                latest["map_to_odom"] = t.transform

    map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, "/map", on_map, map_qos)
    node.create_subscription(TFMessage, "/tf", on_tf, 50)
    node.create_subscription(Odometry, "/odom", lambda m: latest.__setitem__("odom", m), 10)

    def slam_pose_error():
        """Where does SLAM think the robot is? map -> odom (slam_toolbox) + odom -> base_footprint (Isaac Sim),
        compared with the ground truth (map = world: SLAM started at the robot's spawn pose)."""
        t, o = latest["map_to_odom"], latest["odom"]
        if t is None or o is None:
            return None

        def yaw_q(q):
            return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        a, (ox, oy) = yaw_q(t.rotation), (o.pose.pose.position.x, o.pose.pose.position.y)
        sx = t.translation.x + math.cos(a) * ox - math.sin(a) * oy
        sy = t.translation.y + math.sin(a) * ox + math.cos(a) * oy
        syaw = a + yaw_q(o.pose.pose.orientation)
        x, y, yaw = pose2d()
        return math.hypot(sx - x, sy - y), math.degrees(abs(wrap(syaw - yaw)))

    def send(v, w):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        cmd_pub.publish(cmd)

    print("[14] auto-drive: waiting for slam_toolbox's first /map ...", flush=True)

steps, leg, started_at, done_at, path, errors = 0, 0, None, None, [], []
while simulation_app.is_running():
    simulation_app.update()
    steps += 1
    if node is not None:
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.0)
        if started_at is None and (latest["maps"] > 0 or steps > 90 * 60):
            started_at = steps
            print(f"[14] auto-drive: starting the tour at t = {steps / 60:.0f} s ({len(ROUTE)} waypoints)", flush=True)
        if started_at is not None and done_at is None and steps % 3 == 0:  # 20 Hz commands
            x, y, yaw = pose2d()
            if steps % 30 == 0:
                path.append((x, y))
            if math.hypot(ROUTE[leg][0] - x, ROUTE[leg][1] - y) < 0.12:
                err = slam_pose_error()
                if err is not None:
                    errors.append(err)
                    print(f"[14] waypoint {leg + 1}/{len(ROUTE)} {ROUTE[leg]}: SLAM pose error {err[0] * 100:.1f} cm, {err[1]:.1f} deg", flush=True)
                leg += 1
                if leg == len(ROUTE):
                    done_at = steps
                    send(0.0, 0.0)
                    print(f"[14] route done at t = {(steps - started_at) / 60:.0f} s: save the map now", flush=True)
            if done_at is None:
                send(*drive_command(x, y, yaw, ROUTE[leg]))
        if done_at is not None and steps - done_at < 30:
            send(0.0, 0.0)
    if recorder:
        before = len(recorder.frames)
        recorder.capture()
        if len(recorder.frames) > before:  # put the SLAM map next to the simulation
            from PIL import Image, ImageDraw

            panel = Image.fromarray(render_map(latest["map"] if node else None)).convert("RGB")
            draw = ImageDraw.Draw(panel)
            x, y, yaw = pose2d()
            px, py = to_px(x, y)
            draw.ellipse((px - 11, py - 11, px + 11, py + 11), fill=(140, 200, 40), outline=(40, 60, 10), width=2)
            draw.line((px, py, px + 18 * math.cos(yaw), py - 18 * math.sin(yaw)), fill=(40, 60, 10), width=3)
            draw.rectangle((0, 0, 330, 34), fill=(48, 48, 48))
            draw.text((10, 6), "/map from slam_toolbox", fill=(255, 255, 255), font_size=22)
            recorder.frames[-1] = np.hstack([recorder.frames[-1], np.asarray(panel)])
    if (args.test or args.record) and ((done_at is not None and steps - done_at > 10 * 60) or steps > 400 * 60):
        break

ok = True
if args.test:
    from common.paths import IMG_DIR, REPO_ROOT

    truth = occupancy_from_pgm(REPO_ROOT / "ros2_ws" / "src" / "lime_isaac_bringup" / "maps" / "lime_room.yaml")
    m = latest["map"]
    checks = {"slam_toolbox published /map": m is not None, "the tour was driven to the end": done_at is not None}
    if m is not None:
        slam = occupancy_from_msg(m)
        precision, recall, slam_xy = compare_maps(slam, truth)
        print(f"[14] /map: {m.info.width} x {m.info.height} cells, {len(slam_xy)} occupied; {latest['maps']} map updates")
        checks[f"SLAM walls lie on real walls (+-10 cm): {precision:.1%}"] = precision > 0.95
        checks[f"real walls found by SLAM (+-10 cm): {recall:.1%}"] = recall > 0.95
        for name, cx, cy, sx, sy, _ in OBSTACLES:  # in the right place - not mirrored, not shifted
            inside = (abs(slam_xy[:, 0] - cx) < sx / 2 + 0.1) & (abs(slam_xy[:, 1] - cy) < sy / 2 + 0.1)
            checks[f"{name} at ({cx}, {cy}) is in the SLAM map ({inside.sum()} cells)"] = inside.sum() >= 5
        err = slam_pose_error()
        if err is not None:
            # a map cell is 5 cm: allow two cells at the end, three on the way (a drifting map is off by 50 cm or more)
            checks[f"SLAM's robot pose at the end vs ground truth: {err[0] * 100:.1f} cm, {err[1]:.1f} deg"] = err[0] < 0.10 and err[1] < 2.0
            worst = np.max(errors, axis=0) if errors else (9.9, 99.0)
            checks[f"largest SLAM pose error at the waypoints: {worst[0] * 100:.1f} cm, {worst[1]:.1f} deg"] = worst[0] < 0.15 and worst[1] < 3.0
        else:
            checks["TF map -> odom from slam_toolbox"] = False
        if args.record:  # picture for the docs: real walls (grey) and what SLAM found (red)
            from PIL import Image

            t_occ, t_ox, t_oy, res, _ = truth
            rgb = np.where(np.flipud(t_occ)[:, :, None], np.array([175, 175, 175], np.uint8), np.array([255, 255, 255], np.uint8))
            col = np.floor((slam_xy[:, 0] - t_ox) / res).astype(int)
            row = t_occ.shape[0] - 1 - np.floor((slam_xy[:, 1] - t_oy) / res).astype(int)
            keep = (col >= 0) & (row >= 0) & (col < rgb.shape[1]) & (row < rgb.shape[0])
            rgb[row[keep], col[keep]] = (215, 40, 40)
            for x_, y_ in path:
                c, r = int((x_ - t_ox) / res), t_occ.shape[0] - 1 - int((y_ - t_oy) / res)
                rgb[r, c] = (60, 140, 230)
            Image.fromarray(rgb).resize((rgb.shape[1] * 5, rgb.shape[0] * 5), Image.NEAREST).save(IMG_DIR / "tutorial14_compare.png")
    for text, passed in checks.items():
        print(f"[14][check] {'ok  ' if passed else 'FAIL'} {text}")
    ok = all(checks.values())
    print(f"[14][test] {'PASS' if ok else 'FAIL'}")
if node is not None:
    node.destroy_node()
    rclpy.shutdown()
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
