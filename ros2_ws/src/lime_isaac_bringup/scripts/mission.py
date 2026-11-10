#!/usr/bin/env python3
"""Tutorial 13 mission: Nav2 + YOLO + MoveIt 2 working together.

    ros2 run lime_isaac_bringup mission

  1. Nav2     drive to the far corner of the lime_room, then back to the start
  2. YOLO     look at the objects in front of the robot and list them
  3. MoveIt   turn the arm towards one detected object ("point at it"), wave with the gripper
  4. MoveIt   fold the arm again
"""

import math
import sys
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from vision_msgs.msg import Detection2DArray

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
HOME = [0.0, -1.10, 1.85, 0.0, 0.98, 0.0]
FX, CX = 462.3, 320.0  # camera intrinsics (tutorial 08): focal length and image centre in pixels
WAYPOINTS = [("far corner", 2.8, -1.8, 0.0), ("start", 0.0, 0.0, 0.0)]


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def wait(node, future):
    rclpy.spin_until_future_complete(node, future)
    return future.result()


def move_arm(node, client, joints):
    goal = MoveGroup.Goal()
    goal.request.group_name = "arm"
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.5
    goal.request.max_acceleration_scaling_factor = 0.5
    goal.request.goal_constraints = [
        Constraints(
            joint_constraints=[
                JointConstraint(joint_name=n, position=p, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
                for n, p in zip(ARM_JOINTS, joints)
            ]
        )
    ]
    handle = wait(node, client.send_goal_async(goal))
    return handle.accepted and wait(node, handle.get_result_async()).result.error_code.val == MoveItErrorCodes.SUCCESS


def gripper(node, client, position):
    goal = GripperCommand.Goal()
    goal.command.position, goal.command.max_effort = position, 5.0
    handle = wait(node, client.send_goal_async(goal))
    return handle.accepted and wait(node, handle.get_result_async()).result is not None


def main():
    rclpy.init()
    nav = BasicNavigator("lime_mission")
    nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    log = nav.get_logger()

    # --- 1. Nav2 -------------------------------------------------------------------------------------------------
    nav.setInitialPose(pose(nav, 0.0, 0.0, 0.0))  # the robot starts at the map origin
    nav.waitUntilNav2Active()
    for name, x, y, yaw in WAYPOINTS:
        log.info(f"Nav2: going to the {name} ({x}, {y})")
        nav.goToPose(pose(nav, x, y, yaw))
        while not nav.isTaskComplete():
            time.sleep(0.2)
        if nav.getResult() != TaskResult.SUCCEEDED:
            log.error(f"Nav2: could not reach the {name}: {nav.getResult()}")
            sys.exit(1)
        log.info(f"Nav2: reached the {name}")

    # --- 2. YOLO ---------------------------------------------------------------------------------------------------
    seen = {}

    def on_detections(msg):
        for d in msg.detections:
            h = d.results[0].hypothesis
            if h.score > seen.get(h.class_id, (0.0, 0.0))[0]:
                seen[h.class_id] = (h.score, d.bbox.center.position.x)

    nav.create_subscription(Detection2DArray, "/detections", on_detections, 10)
    end = time.time() + 6.0
    while time.time() < end:
        rclpy.spin_once(nav, timeout_sec=0.1)
    if not seen:
        log.error("YOLO: nothing detected - is lime_yolo running?")
        sys.exit(1)
    log.info("YOLO: I see " + ", ".join(f"a {c} ({s:.2f})" for c, (s, _) in sorted(seen.items())))
    target = "bottle" if "bottle" in seen else max(seen, key=lambda c: seen[c][0])
    # Image column -> bearing: right of the image centre = to the robot's right = negative angle.
    bearing = -math.atan((seen[target][1] - CX) / FX)
    log.info(f"YOLO: pointing at the {target}, {math.degrees(bearing):.0f} deg from straight ahead")

    # --- 3. + 4. MoveIt --------------------------------------------------------------------------------------------
    arm = ActionClient(nav, MoveGroup, "/move_action")
    grip = ActionClient(nav, GripperCommand, "/gripper_controller/gripper_cmd")
    if not (arm.wait_for_server(timeout_sec=30.0) and grip.wait_for_server(timeout_sec=30.0)):
        log.error("MoveIt / gripper action servers not available")
        sys.exit(1)
    ok = move_arm(nav, arm, [bearing, -0.6, 0.3, 0.0, 1.2, 0.0])
    log.info(f"MoveIt: point at the {target}: {'done' if ok else 'FAILED'}")
    for position in (0.019, -0.010, 0.019, -0.010):  # wave
        gripper(nav, grip, position)
    ok = move_arm(nav, arm, HOME) and ok
    log.info(f"MoveIt: arm folded: {'done' if ok else 'FAILED'}")
    log.info("mission complete" if ok else "mission FAILED")
    nav.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
