#!/usr/bin/env python3
"""Tutorial 15 mission: a patrol, written as a behavior tree (py_trees).

    ros2 run lime_isaac_bringup patrol
    ros2 run lime_isaac_bringup patrol --render DIR     # only draw the tree (DIR/patrol.png), no robot needed

The Lime visits four lookouts with Nav2, looks around at each, then goes home. It has a (pretend) battery that runs down as it
drives. When the battery is low, the tree interrupts the patrol, sends the robot to the charger, and then
continues the patrol where it was interrupted.
"""

import argparse
import math
import operator
import os
import sys

import py_trees
import rclpy
from geometry_msgs.msg import PoseStamped
from py_trees.common import Access, ComparisonExpression, Status

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))  # tutorial 13's mission.py, lime_bt.py
from lime_bt import NavigateTo, TurnAround, run_tree  # noqa: E402

LOOKOUTS = [(3.2, -1.8, 0.0), (3.3, 1.9, math.pi / 2), (0.5, 1.9, math.pi), (-1.3, -1.8, -math.pi / 2)]  # x, y, yaw
CHARGER = (-1.5, 0.0, math.pi)
HOME = (0.0, 0.0, 0.0)
DRAIN = 5.0  # % per metre driven (a pretend battery)
LOW = 30.0  # %


# --8<-- [start:battery]
class Battery(py_trees.behaviour.Behaviour):
    """Writes the battery level to the blackboard on every tick. The Lime's battery is pretend: it runs down by
    DRAIN % per metre driven (measured with odometry). Always SUCCESS: it is a sensor, not a decision."""

    def __init__(self, lime):
        super().__init__("Battery")
        self.lime, self.last = lime, None
        self.blackboard = self.attach_blackboard_client(name="Battery")
        self.blackboard.register_key(key="battery", access=Access.WRITE)
        self.blackboard.battery = 100.0

    def update(self):
        x, y, _ = self.lime.pose()
        if self.last is not None:
            self.blackboard.battery = max(0.0, self.blackboard.battery - DRAIN * math.hypot(x - self.last[0], y - self.last[1]))
        self.last = (x, y)
        self.feedback_message = f"{self.blackboard.battery:.0f} %"
        return Status.SUCCESS


class Charge(py_trees.behaviour.Behaviour):
    """Charge until full: +20 % per second. RUNNING while charging."""

    def __init__(self, log):
        super().__init__("Charge")
        self.log = log
        self.blackboard = self.attach_blackboard_client(name="Charge")
        self.blackboard.register_key(key="battery", access=Access.WRITE)

    def update(self):
        self.blackboard.battery = min(100.0, self.blackboard.battery + 2.0)  # one tick = 0.1 s
        if self.blackboard.battery < 100.0:
            return Status.RUNNING
        self.log.info("battery full")
        return Status.SUCCESS
# --8<-- [end:battery]


# --8<-- [start:lookouts]
class BeenThere(py_trees.behaviour.Behaviour):
    """Condition: is the current lookout checked off already? (After a recharge the patrol starts again
    from the first lookout, and skips the ones it has visited.)"""

    def __init__(self):
        super().__init__("Been there already?")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="lookout", access=Access.READ)
        self.blackboard.register_key(key="visited", access=Access.READ)

    def update(self):
        return Status.SUCCESS if self.blackboard.lookout in self.blackboard.visited else Status.FAILURE


class CheckOff(py_trees.behaviour.Behaviour):
    def __init__(self, log):
        super().__init__("Check it off")
        self.log = log
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="lookout", access=Access.READ)
        self.blackboard.register_key(key="visited", access=Access.WRITE)
        self.blackboard.register_key(key="battery", access=Access.READ)

    def update(self):
        self.blackboard.visited = self.blackboard.visited + [self.blackboard.lookout]
        x, y, _ = self.blackboard.lookout
        self.log.info(f"lookout ({x}, {y}) checked, battery {self.blackboard.battery:.0f} %")
        return Status.SUCCESS
# --8<-- [end:lookouts]


# --8<-- [start:tree]
def build_tree(lime, log):
    blackboard = py_trees.blackboard.Client(name="patrol")
    for key in ("lookouts", "visited"):
        blackboard.register_key(key=key, access=Access.WRITE)
    blackboard.lookouts, blackboard.visited = LOOKOUTS, []

    battery_low = py_trees.behaviours.CheckBlackboardVariableValue(
        name=f"Battery below {LOW:.0f} %?", check=ComparisonExpression(variable="battery", value=LOW, operator=operator.lt)
    )
    recharge = py_trees.composites.Sequence("Recharge", memory=True, children=[
        battery_low,
        NavigateTo("Go to the charger", lime, pose=CHARGER),
        Charge(log),
    ])
    visit = py_trees.composites.Selector("Visit it", memory=False, children=[
        BeenThere(),
        py_trees.composites.Sequence("Go there", memory=True, children=[
            NavigateTo("Go to the lookout", lime, key="lookout"),
            TurnAround("Look around", lime),
            CheckOff(log),
        ]),
    ])
    patrol = py_trees.composites.Sequence("Patrol", memory=True, children=[
        py_trees.decorators.ForEach("Every lookout", visit, source_key="lookouts", target_key="lookout"),
        NavigateTo("Go home", lime, pose=HOME),
    ])
    what_now = py_trees.composites.Selector("What now?", memory=False, children=[
        py_trees.decorators.SuccessIsRunning("Then go on", recharge),  # after charging: carry on patrolling
        patrol,
    ])
    return py_trees.composites.Sequence("Lime patrol", memory=False, children=[Battery(lime), what_now])
# --8<-- [end:tree]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--render", metavar="DIR", help="only draw the tree into DIR (patrol.png / .svg / .dot)")
    args, _ros = parser.parse_known_args()
    if args.render:
        py_trees.display.render_dot_tree(build_tree(None, None), target_directory=args.render, name="patrol")
        return

    from mission import Lime, wait_until_nav2_active  # tutorial 13: Nav2 (BasicNavigator), TF, odometry pose

    rclpy.init()
    lime = Lime()
    start = PoseStamped()
    start.header.frame_id = "map"
    start.header.stamp = lime.nav.get_clock().now().to_msg()
    start.pose.orientation.w = 1.0
    lime.nav.setInitialPose(start)  # the robot starts at the map origin
    wait_until_nav2_active(lime.nav)
    status = run_tree(build_tree(lime, lime.log), lime.log)
    lime.log.info("patrol complete" if status == Status.SUCCESS else "patrol FAILED")
    lime.close()
    sys.exit(0 if status == Status.SUCCESS else 1)


if __name__ == "__main__":
    main()
