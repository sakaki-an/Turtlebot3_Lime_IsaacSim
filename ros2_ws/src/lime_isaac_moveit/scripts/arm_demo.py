#!/usr/bin/env python3
"""Plan + execute arm motions with MoveIt 2 and open/close the gripper (tutorial 12).

    ros2 run lime_isaac_moveit arm_demo                  # demo pose, open gripper
    ros2 run lime_isaac_moveit arm_demo --pose home --gripper close

It talks to two ROS 2 actions:
  /move_action                       moveit_msgs/action/MoveGroup    (move_group: plan + execute)
  /gripper_controller/gripper_cmd    control_msgs/action/GripperCommand
"""

import argparse
import sys

import rclpy
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
POSES = {  # radians
    "home": [0.0, -1.10, 1.80, 0.0, 0.98, 0.0],  # folded (SRDF "home")
    "demo": [0.0, -0.6, 0.3, 0.0, 1.2, 0.0],  # reach forward, gripper pointing down/forward
    "left": [1.2, -0.6, 0.3, 0.0, 1.2, 0.0],
}
GRIPPER = {"open": 0.019, "close": -0.010}


def wait_result(node, future):
    rclpy.spin_until_future_complete(node, future)
    return future.result()


def move_arm(node, client, joints):
    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = "arm"
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.5
    req.max_acceleration_scaling_factor = 0.5
    req.goal_constraints = [
        Constraints(
            joint_constraints=[
                JointConstraint(joint_name=n, position=p, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
                for n, p in zip(ARM_JOINTS, joints)
            ]
        )
    ]
    goal.planning_options.plan_only = False  # plan AND execute
    handle = wait_result(node, client.send_goal_async(goal))
    if not handle.accepted:
        node.get_logger().error("MoveGroup goal rejected")
        return False
    result = wait_result(node, handle.get_result_async()).result
    ok = result.error_code.val == MoveItErrorCodes.SUCCESS
    node.get_logger().info(f"arm -> {joints}: {'SUCCESS' if ok else f'error code {result.error_code.val}'}")
    return ok


def move_gripper(node, client, position):
    goal = GripperCommand.Goal()
    goal.command.position = position
    goal.command.max_effort = 5.0
    handle = wait_result(node, client.send_goal_async(goal))
    if not handle.accepted:
        node.get_logger().error("gripper goal rejected")
        return False
    result = wait_result(node, handle.get_result_async()).result
    node.get_logger().info(f"gripper -> {position:.3f} m: reached={result.reached_goal} stalled={result.stalled} at {result.position:.4f}")
    return result.reached_goal or result.stalled


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pose", choices=POSES, default="demo")
    parser.add_argument("--gripper", choices=GRIPPER, default="open")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node("lime_arm_demo", parameter_overrides=[])
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    arm = ActionClient(node, MoveGroup, "/move_action")
    gripper = ActionClient(node, GripperCommand, "/gripper_controller/gripper_cmd")
    for name, client in [("move_group", arm), ("gripper_controller", gripper)]:
        if not client.wait_for_server(timeout_sec=60.0):
            node.get_logger().error(f"{name} action server not available - is moveit.launch.py running?")
            sys.exit(1)

    ok = move_arm(node, arm, POSES[args.pose]) and move_gripper(node, gripper, GRIPPER[args.gripper])
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
