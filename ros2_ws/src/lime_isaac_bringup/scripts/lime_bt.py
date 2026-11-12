"""Behavior-tree building blocks for the Lime (tutorials 15 and 16), with py_trees.

A behaviour tree is *ticked*, about 10 times a second. Every tick, each behaviour that is ticked answers
RUNNING (not done yet), SUCCESS or FAILURE. Composites decide which child to tick next:

    Sequence   ticks its children in order; fails as soon as one fails       ("do this, then that")
    Selector   ticks its children in order; succeeds as soon as one succeeds ("try this, else that")

A tick must be quick, but driving to a lookout takes a minute. So long actions (`Action` below) run in a
background thread, and every tick only asks "done yet?". If a higher-priority branch takes over, the tree
*interrupts* the running action: terminate() is called, and the action stops (Nav2 cancels its goal).
"""

import math
import threading
import time

import py_trees
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from py_trees.common import Status


# --8<-- [start:action]
class Action(py_trees.behaviour.Behaviour):
    """A long action, in a background thread. Subclasses write run(): return True for success, False for
    failure, and look at self.stop_requested now and then to stop early when the tree interrupts them."""

    robot = threading.Lock()  # one robot: one action at a time

    def __init__(self, name):
        super().__init__(name)
        self.thread, self.result, self.stop_requested = None, None, False

    def initialise(self):  # first tick: start the action
        self.result, self.stop_requested = None, False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            with Action.robot:  # an interrupted action may still be stopping (cancelling its Nav2 goal)
                self.result = bool(self.run())
        except Exception as e:  # an exception is a failure, not a crash of the whole tree
            self.feedback_message = f"{type(e).__name__}: {e}"
            self.result = False

    def update(self):  # every tick: done yet?
        if self.thread.is_alive():
            return Status.RUNNING
        return Status.SUCCESS if self.result else Status.FAILURE

    def terminate(self, new_status):  # finished, or interrupted by the tree (new_status = INVALID)
        if new_status == Status.INVALID and self.thread is not None and self.thread.is_alive():
            self.stop_requested = True
            self.thread.join()

    def run(self) -> bool:
        raise NotImplementedError
# --8<-- [end:action]


# --8<-- [start:navigate]
class TurnAround(Action):
    """Turn round once on the spot, in four quarter turns (with tutorial 13's turn_to(), odometry)."""

    def __init__(self, name, lime):
        super().__init__(name)
        self.lime = lime

    def run(self):
        _, _, yaw = self.lime.pose()
        for k in range(1, 5):
            if self.stop_requested:
                return False
            self.lime.turn_to(yaw + k * math.pi / 2)
        return True


class NavigateTo(Action):
    """Drive to a pose (x, y, yaw) in the map frame with Nav2. The pose comes from the blackboard (key) or is
    fixed (pose). Interrupted, it cancels the Nav2 goal."""

    def __init__(self, name, lime, key=None, pose=None):
        super().__init__(name)
        self.lime, self.pose = lime, pose
        if key:
            self.key = key
            self.blackboard = self.attach_blackboard_client(name=name)
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def run(self):
        x, y, yaw = self.pose if self.pose else self.blackboard.get(self.key)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.lime.nav.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = float(x), float(y)
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not self.lime.nav.goToPose(goal):
            return False
        while not self.lime.nav.isTaskComplete():  # spins the node for up to 0.1 s
            if self.stop_requested:
                self.lime.nav.cancelTask()
                return False
        return self.lime.nav.getResult() == TaskResult.SUCCEEDED
# --8<-- [end:navigate]


# --8<-- [start:run_tree]
def run_tree(root, log, period=0.1):
    """Tick the tree until it succeeds or fails. Print every behaviour that starts (...), succeeds (ok), fails
    (FAIL) or is interrupted by the tree (stop)."""
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15.0)
    log.info("the tree:\n" + py_trees.display.unicode_tree(root))
    before, started = {}, {}
    while True:
        tree.tick()
        for b in root.iterate():
            if isinstance(b, (py_trees.composites.Composite, py_trees.decorators.Decorator)) or b.status == before.get(b.id):
                continue
            if b.status == Status.RUNNING:
                started[b.id] = time.time()
                log.info(f"[bt] ... {b.name}")
            elif b.status in (Status.SUCCESS, Status.FAILURE):
                begin = started.pop(b.id, None)
                took = f" ({time.time() - begin:.0f} s)" if begin and time.time() - begin > 1 else ""
                # say why an action failed, or what a decision of ours chose (not py_trees' own condition messages)
                ours = not type(b).__module__.startswith("py_trees")
                why = f": {b.feedback_message}" if ours and b.feedback_message and (b.status == Status.FAILURE) == isinstance(b, Action) else ""
                log.info(f"[bt] {'ok  ' if b.status == Status.SUCCESS else 'FAIL'} {b.name}{took}{why}")
            elif b.status == Status.INVALID and before.get(b.id) == Status.RUNNING:
                started.pop(b.id, None)
                log.info(f"[bt] stop {b.name} (interrupted)")
            before[b.id] = b.status
        if root.status in (Status.SUCCESS, Status.FAILURE):
            log.info("the tree at the end:\n" + py_trees.display.unicode_tree(root, show_status=True))
            return root.status
        time.sleep(period)
# --8<-- [end:run_tree]
