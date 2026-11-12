#!/usr/bin/env python3
"""Tutorial 13 mission: tidy up the lime_room - put the banana into the bowl, then park.

    ros2 run lime_isaac_bringup mission

  1. YOLO + depth   find the banana: a 2D box + the depth image + TF -> a 3D position and direction
  2. drive          drive up to it, centimetre-accurate, with odometry
  3. MoveIt         pick it up: top-down grasp, fingers across the banana
  4. YOLO + depth   find the bowl, drive up to it
  5. MoveIt         put the banana into the bowl, fold the arm
  6. Nav2           park in the far corner of the room
"""

import math
import sys
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped, Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, MoveItErrorCodes, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from vision_msgs.msg import Detection2DArray

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
HOME = [0.0, -1.10, 1.80, 0.0, 0.98, 0.0]  # folded (the SRDF "home" pose)
CARRY = [0.0, -0.3, 1.2, 0.0, 1.0, 0.0]  # banana held high above the robot, out of the camera's view
GRIPPER_OPEN, GRIPPER_CLOSED = 0.019, -0.010
CAMERA = "/camera/camera"
PARK = (2.8, -1.8, 0.0)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# --8<-- [start:ik]
# The arm in the base_footprint frame (from the URDF): joint1 turns about a vertical axis at x = -0.024;
# joint2, joint3 and joint5 tilt the arm forward; joint4 and joint6 roll. Looking straight down, the arm is
# a planar 2-link arm (upper arm L1, forearm L2) plus L3 from the wrist to end_effector_link (the fingertips
# reach 2 cm further down).
SHOULDER_X, SHOULDER_Z = -0.024, 0.2065
L1, L2, L3 = 0.124, 0.1045, 0.1625


def top_down_ik(x, y, z, object_yaw):
    """Joint angles that put end_effector_link at (x, y, z) [base_footprint frame], pointing straight down,
    with the fingers closing across an object whose long axis points along object_yaw. None = out of reach."""
    j1 = math.atan2(y, x - SHOULDER_X)
    r = math.hypot(x - SHOULDER_X, y)  # horizontal distance from joint1's axis
    dz = z + L3 - SHOULDER_Z  # the wrist is L3 above end_effector_link
    c3 = (r * r + dz * dz - L1 * L1 - L2 * L2) / (2 * L1 * L2)  # law of cosines
    if abs(c3) > 1.0:
        return None
    j3 = math.acos(c3)  # elbow up
    j2 = math.atan2(r, dz) - math.atan2(L2 * math.sin(j3), L1 + L2 * math.cos(j3))
    j5 = math.pi - j2 - j3  # the three tilts add up to 180 deg: pointing down
    j6 = (j1 - object_yaw + math.pi / 2) % math.pi - math.pi / 2  # fingers across the object (either way round)
    joints = [j1, j2, j3, 0.0, j5, j6]
    limits = [(-2.82, 2.82), (-1.69, 1.76), (-1.75, 1.85), (-1.57, 1.57), (-1.66, 1.78), (-2.82, 2.82)]
    return joints if all(lo <= j <= hi for j, (lo, hi) in zip(joints, limits)) else None
# --8<-- [end:ik]


class Lime:
    """Everything the mission needs: perception, the base, the arm. Nav2 is the BasicNavigator node."""

    def __init__(self):
        self.nav = BasicNavigator("lime_mission")
        self.nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.log = self.nav.get_logger()
        # TF on its own node + thread, so lookups work while the main node waits for actions
        self.tf_node = rclpy.create_node("lime_mission_tf", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self.tf_node)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.tf_node)
        self.tf_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.tf_thread.start()

        n = self.nav
        self.detections, self.depth, self.K = [], {}, None
        n.create_subscription(Detection2DArray, "/detections", self.on_detections, 10)
        n.create_subscription(Image, f"{CAMERA}/aligned_depth_to_color/image_raw", self.on_depth, 5)
        n.create_subscription(CameraInfo, f"{CAMERA}/color/camera_info", lambda m: setattr(self, "K", np.array(m.k).reshape(3, 3)), 1)
        self.cmd = n.create_publisher(Twist, "/cmd_vel", 10)
        self.arm = ActionClient(n, MoveGroup, "/move_action")
        self.grip = ActionClient(n, GripperCommand, "/gripper_controller/gripper_cmd")
        self.scene = n.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def close(self):
        """Stop the TF thread and the nodes before exiting (otherwise Python aborts at exit)."""
        self.send(0.0, 0.0)
        self.executor.shutdown()
        self.tf_thread.join(timeout=2.0)
        self.tf_node.destroy_node()
        self.nav.destroy_node()
        rclpy.try_shutdown()

    def on_depth(self, msg):
        self.depth[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
        while len(self.depth) > 60:
            self.depth.pop(next(iter(self.depth)))

    def on_detections(self, msg):
        """Keep each detection together with the depth image taken at the same moment (same time stamp)."""
        depth = self.depth.get((msg.header.stamp.sec, msg.header.stamp.nanosec))
        if depth is not None:
            self.detections.append((msg, depth))
            del self.detections[:-60]  # the last 60 only (each holds a depth image)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.nav, timeout_sec=0.05)

    def wait(self, future):
        rclpy.spin_until_future_complete(self.nav, future)
        return future.result()

    # --8<-- [start:locate]
    def locate(self, name, seconds=4.0):
        """Find object `name` with YOLO and measure where it is: a 3D point cloud of the object, in the odom frame.

        2D box (YOLO)  +  depth image  +  camera model  ->  3D points in the camera frame
                       +  TF odom <- camera at the image's time stamp  ->  3D points in the odom frame
        """
        self.detections.clear()
        self.spin_for(seconds)  # collect a few seconds of detections, keep the most confident one
        best = None
        for msg, depth_msg in self.detections:
            for d in msg.detections:
                h = d.results[0].hypothesis
                if h.class_id == name and (best is None or h.score > best[0]):
                    best = (h.score, d, msg.header, depth_msg)
        if best is None or self.K is None:
            return None
        score, det, header, depth_msg = best
        pts, camera = self.points(det, header, depth_msg)
        return None if pts is None else {"score": score, "points": pts, "camera": camera}

    def points(self, det, header, depth_msg):
        """The 3D points (odom frame) of one detection's box, and where the camera was. (None, None) if too few."""
        depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)
        cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
        w, h = det.bbox.size_x, det.bbox.size_y
        u0, u1 = int(max(cx - w / 2, 0)), int(min(cx + w / 2, depth_msg.width - 1))
        v0, v1 = int(max(cy - h / 2, 0)), int(min(cy + h / 2, depth_msg.height - 1))
        v, u = np.mgrid[v0:v1 + 1, u0:u1 + 1]
        z = depth[v0:v1 + 1, u0:u1 + 1]
        ok = np.isfinite(z) & (z > 0.05) & (z < 3.0)
        fx, fy, px, py = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        cam = np.stack([(u[ok] - px) * z[ok] / fx, (v[ok] - py) * z[ok] / fy, z[ok]], axis=1)  # optical frame
        tf = self.tf.lookup_transform("odom", header.frame_id, header.stamp, timeout=rclpy.duration.Duration(seconds=1.0))
        t, q = tf.transform.translation, tf.transform.rotation
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        pts = cam @ R.T + np.array([t.x, t.y, t.z])
        pts = pts[(pts[:, 2] > 0.006) & (pts[:, 2] < 0.40)]  # drop the floor (z = 0) around the object
        if len(pts) < 20:
            return None, None
        middle = np.median(pts[:, :2], axis=0)
        pts = pts[np.linalg.norm(pts[:, :2] - middle, axis=1) < 0.25]  # drop the background inside the box
        return pts, np.array([t.x, t.y])
    # --8<-- [end:locate]

    # --8<-- [start:base]
    def pose(self):
        """Robot pose (x, y, yaw) in the odom frame - Isaac Sim's odometry, through TF."""
        tf = self.tf.lookup_transform("odom", "base_footprint", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def send(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd.publish(msg)

    def turn_to(self, yaw):
        w, end = 0.0, time.time() + 30.0
        while time.time() < end:
            error = wrap(yaw - self.pose()[2])
            if abs(error) < 0.01:
                break
            target = math.copysign(min(0.5, max(0.08, 1.5 * abs(error))), error)
            w += max(-0.03, min(0.03, target - w))  # gentle acceleration: the robot carries things
            self.send(0.0, w)
            time.sleep(0.05)
        self.send(0.0, 0.0)
        time.sleep(0.5)

    def drive_to(self, x, y, backwards=False):
        """Turn towards (x, y) [odom frame], drive there in a straight line, stop within a centimetre."""
        px, py, _ = self.pose()
        heading = math.atan2(y - py, x - px) + (math.pi if backwards else 0.0)
        self.turn_to(heading)
        v, sign, end = 0.0, -1.0 if backwards else 1.0, time.time() + 60.0
        while time.time() < end:
            px, py, yaw = self.pose()
            ahead = (x - px) * math.cos(yaw) + (y - py) * math.sin(yaw)  # distance left along the heading
            if sign * ahead < 0.008:
                break
            target = min(0.12, max(0.02, 0.8 * abs(ahead)))
            v += max(-0.01, min(0.01, target - v))
            side = -(x - px) * math.sin(yaw) + (y - py) * math.cos(yaw)
            self.send(sign * v, max(-0.3, min(0.3, 2.0 * side * sign)) if abs(ahead) > 0.05 else 0.0)
            time.sleep(0.05)
        self.send(0.0, 0.0)
        time.sleep(0.5)

    def to_base(self, x, y):
        """odom-frame point -> base_footprint frame (where the arm works)."""
        px, py, yaw = self.pose()
        dx, dy = x - px, y - py
        return dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
    # --8<-- [end:base]

    # --- the arm (MoveIt 2) and the gripper (ros2_control) ---
    def move_arm(self, joints, what):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(joint_name=n, position=p, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
                    for n, p in zip(ARM_JOINTS, joints)
                ]
            )
        ]
        handle = self.wait(self.arm.send_goal_async(goal))
        ok = handle.accepted and self.wait(handle.get_result_async()).result.error_code.val == MoveItErrorCodes.SUCCESS
        self.log.info(f"MoveIt: {what}: {'done' if ok else 'FAILED'}")
        return ok

    def gripper(self, position, seconds=3.0):
        """Open / close the gripper. Closing on the banana, the fingers stop at the banana. The controller would
        call that a "stall" once the finger velocity is ~0, but in simulation the squeezing finger reports a
        velocity (about 0.3 m/s) although it does not move, so the goal never finishes. Wait at most `seconds`:
        the goal stays active, and the controller keeps commanding the closed position, so the fingers keep
        squeezing."""
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = position, 10.0
        handle = self.wait(self.grip.send_goal_async(goal))
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.nav, result, timeout_sec=seconds)
        return handle.accepted

    def add_floor(self):
        """Tell MoveIt about the floor, so no planned arm motion goes through it."""
        floor = CollisionObject(id="floor", operation=CollisionObject.ADD)
        floor.header.frame_id = "base_footprint"
        floor.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[3.0, 3.0, 0.02])]
        floor.primitive_poses = [Pose()]
        floor.primitive_poses[0].position.z = -0.02  # top at -1 cm: below the wheels, above nothing else
        floor.primitive_poses[0].orientation.w = 1.0
        request = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        request.scene.world.collision_objects = [floor]
        self.scene.wait_for_service(timeout_sec=30.0)
        return self.wait(self.scene.call_async(request)).success


def wait_until_nav2_active(nav):
    """BasicNavigator.waitUntilNav2Active(), but with a timeout on every service call, and retries.

    Right after a service appears, DDS may not have finished matching both sides, and the first request can get
    lost. The original then waits for its answer forever (it happened twice in about 25 runs)."""
    from lifecycle_msgs.srv import GetState

    for node in ("amcl", "bt_navigator"):
        client = nav.create_client(GetState, f"{node}/get_state")
        while True:
            if not client.wait_for_service(timeout_sec=1.0):
                nav.info(f"{node}/get_state service not available, waiting...")
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(nav, future, timeout_sec=2.0)
            if future.done() and future.result() is not None and future.result().current_state.label == "active":
                break
            time.sleep(1.0)  # not active yet, or the answer got lost: ask again
        nav.destroy_client(client)
        if node == "amcl":
            nav._waitForInitialPose()  # BasicNavigator's own: (re)publish the initial pose until AMCL has it
    nav.info("Nav2 is ready for use!")


def quat_to_matrix(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


# --8<-- [start:shapes]
def long_object(found):
    """A lying object (the banana): its direction = the main axis of its points (PCA); the grasp point = the
    middle of its length. Not simply the average point: the end nearer the camera gets more depth pixels."""
    xy = found["points"][:, :2]
    mean = xy.mean(axis=0)
    eigenvalues, eigenvectors = np.linalg.eigh(np.cov((xy - mean).T))
    axis = eigenvectors[:, np.argmax(eigenvalues)]
    normal = np.array([-axis[1], axis[0]])
    along, across = (xy - mean) @ axis, (xy - mean) @ normal
    middle = (np.percentile(along, 2) + np.percentile(along, 98)) / 2  # halfway between the two ends
    at_middle = np.abs(along - middle) < 0.02  # the slice of the object there (a banana is curved)
    centre = mean + middle * axis + np.median(across[at_middle]) * normal
    return centre, math.atan2(axis[1], axis[0])


def round_object(found):
    """A round object (the bowl): the camera sees only its front, so centre = front surface + one radius
    further along the line of sight; the radius is half the width the camera sees."""
    xy = found["points"][:, :2]
    ray = np.median(xy, axis=0) - found["camera"]
    ray /= np.linalg.norm(ray)
    side = np.array([-ray[1], ray[0]])
    along, across = (xy - found["camera"]) @ ray, (xy - found["camera"]) @ side
    radius = (np.percentile(across, 98) - np.percentile(across, 2)) / 2
    front = np.percentile(along, 5)
    middle = (np.percentile(across, 98) + np.percentile(across, 2)) / 2
    return found["camera"] + (front + radius) * ray + middle * side, radius
# --8<-- [end:shapes]


def main():
    rclpy.init()
    lime = Lime()
    log = lime.log

    def fail(text):
        log.error(text)
        lime.close()
        sys.exit(1)

    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = lime.nav.get_clock().now().to_msg()
    p.pose.orientation.w = 1.0
    lime.nav.setInitialPose(p)  # the robot starts at the map origin
    wait_until_nav2_active(lime.nav)
    if not (lime.arm.wait_for_server(timeout_sec=60.0) and lime.grip.wait_for_server(timeout_sec=60.0)):
        fail("MoveIt / gripper action servers not available")
    lime.add_floor()
    lime.spin_for(1.0)

    # --8<-- [start:mission]
    # 1. Find the banana and the bowl from where we are (the banana is small and far: collect a few seconds).
    found, found_bowl = lime.locate("banana", seconds=6.0), lime.locate("bowl", seconds=2.0)
    if found is None or found_bowl is None:
        fail("YOLO: I need to see both the banana and the bowl")
    banana, _ = long_object(found)
    bowl, _ = round_object(found_bowl)
    log.info(f"YOLO: banana ({found['score']:.2f}) at ({banana[0]:.2f}, {banana[1]:.2f}), bowl ({found_bowl['score']:.2f}) at ({bowl[0]:.2f}, {bowl[1]:.2f})")

    # 2. From here we see the banana almost end-on: its far half hides behind its near half. Look again from
    #    45 cm away and 55 deg further round (the side away from the mug): now the whole banana is visible.
    x, y, _ = lime.pose()
    look = math.atan2(banana[1] - y, banana[0] - x) + math.radians(55)
    toward = np.array([math.cos(look), math.sin(look)])
    lime.drive_to(*(banana - 0.45 * toward))
    lime.turn_to(look)
    found = lime.locate("banana")
    if found is None:
        fail("YOLO: lost the banana")
    banana, banana_yaw = long_object(found)
    log.info(f"YOLO: banana ({found['score']:.2f}) at ({banana[0]:.3f}, {banana[1]:.3f}), pointing {math.degrees(banana_yaw) % 180:.0f} deg")

    # 3. Stop with the banana 16 cm in front of the robot: inside the arm's reach for a top-down grasp.
    lime.drive_to(*(banana - 0.16 * toward))
    lime.turn_to(look)

    # 4. Pick it up.
    lime.gripper(GRIPPER_OPEN)
    bx, by = lime.to_base(*banana)
    yaw = wrap(banana_yaw - lime.pose()[2])  # the banana's direction as seen from the robot
    above, grasp, lifted = (top_down_ik(bx, by, z, yaw) for z in (0.10, 0.03, 0.15))
    if None in (above, grasp, lifted):
        fail(f"the banana at ({bx:.3f}, {by:.3f}) in the robot frame is out of reach")
    if not (lime.move_arm(above, "above the banana") and lime.move_arm(grasp, "down to the banana")):
        fail("MoveIt: grasp failed")
    lime.gripper(GRIPPER_CLOSED)  # stops when the fingers touch the banana, and keeps squeezing
    if not (lime.move_arm(lifted, "lift") and lime.move_arm(CARRY, "carry")):
        fail("MoveIt: lift failed")

    # 5. Back to the bowl. The mug stands between the banana and the bowl, so go round: approach the bowl
    #    from its open side (from -y, 42 cm away), look again, and drive up until the bowl is 20 cm in front.
    lime.drive_to(bowl[0], bowl[1] - 0.42)
    lime.turn_to(math.pi / 2)
    found = lime.locate("bowl")
    if found is None:
        fail("YOLO: lost the bowl")
    bowl, radius = round_object(found)
    log.info(f"YOLO: bowl ({found['score']:.2f}) at ({bowl[0]:.3f}, {bowl[1]:.3f}), radius {radius * 100:.0f} cm")
    lime.drive_to(bowl[0], bowl[1] - 0.20)
    lime.turn_to(math.pi / 2)

    # 6. Put the banana in: above the bowl's centre, or as close to it as the arm reaches; banana across the robot.
    bx, by = lime.to_base(*bowl)
    release = next((q for x in np.arange(bx, 0.10, -0.005) if (q := top_down_ik(x, by, 0.13, math.pi / 2))), None)
    if release is None or not lime.move_arm(release, "above the bowl"):
        fail("MoveIt: could not reach the bowl")
    lime.gripper(GRIPPER_OPEN)
    lime.spin_for(1.0)
    lime.move_arm(HOME, "arm folded")

    # 7. Back away from the bowl and drive into the open middle of the room, then let Nav2 park the robot.
    #    (Started right next to the bottle, Nav2 would steer the robot straight through it.)
    x, y, yaw = lime.pose()
    lime.drive_to(x - 0.12 * math.cos(yaw), y - 0.12 * math.sin(yaw), backwards=True)
    lime.drive_to(bowl[0] + 0.6, bowl[1] - 0.45)
    log.info(f"Nav2: parking at ({PARK[0]}, {PARK[1]})")
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x, goal.pose.position.y = PARK[0], PARK[1]
    goal.pose.orientation.w = 1.0
    lime.nav.goToPose(goal)
    while not lime.nav.isTaskComplete():
        time.sleep(0.2)
    if lime.nav.getResult() != TaskResult.SUCCEEDED:
        fail(f"Nav2: could not park: {lime.nav.getResult()}")
    # --8<-- [end:mission]
    log.info("mission complete: the banana is in the bowl")
    lime.close()


if __name__ == "__main__":
    main()
