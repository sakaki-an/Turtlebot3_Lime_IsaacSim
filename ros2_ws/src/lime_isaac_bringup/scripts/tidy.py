#!/usr/bin/env python3
"""Tutorial 16 mission: tidy the room - a behavior tree (py_trees) on top of Nav2, the depth camera, YOLO and MoveIt.

    ros2 run lime_isaac_bringup tidy
    ros2 run lime_isaac_bringup tidy --render DIR      # only draw the tree (DIR/tidy.png), no robot needed

Bananas lie around the lime_room; the robot does not know where. Put every one into the fruit bowl, then go home.
  1. Find the fruit bowl: YOLO, from the start (tutorial 13).
  2. At each lookout (Nav2): turn round once. The depth camera finds every thing on the floor that is not in
     the map (tutorial 11: the map holds only what stays put).
  3. Check each thing: Nav2 drives up to it, and YOLO says what it is. (YOLO knows a lying banana only from
     up close: from the low camera, a banana 1 m away is a thin yellow line.)
  4. A banana: pick it up (tutorial 13), Nav2 carries it to the bowl, put it in. Anything else: leave it.
  5. Go home.
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import py_trees
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from py_trees.common import Access, Status
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))  # tutorial 13's mission.py, lime_bt.py
from lime_bt import Action, NavigateTo, run_tree  # noqa: E402

LOOKOUTS = [(0.0, 0.0, 0.0), (2.9, -0.5, 0.0), (1.9, 0.7, math.pi / 2)]  # where the robot looks round (map frame)
HOME_POSE = (0.0, 0.0, 0.0)
VIEW = 0.55  # m: look at a thing from this far (YOLO's banana range from the low camera)
SIDE = 0.45  # m: look at a banana from its side, from this far, before grasping it (tutorial 13)
BACK_OUT = 0.60  # m: after the grasp, back out to this far from where the banana was; Nav2 takes over there
GRASP = 0.16  # m: a banana this far in front of base_footprint is in the arm's reach (tutorial 13)
BOWL_APPROACH, BOWL_RELEASE = 0.42, 0.20  # m from the bowl's centre (tutorial 13)
FOOTPRINT = (-0.205, 0.075, 0.165)  # back, front, half width (m): the Lime's base in Nav2 (tutorial 11)
# How big to draw each kind of thing for Nav2 (m): its radius, plus a few cm because the camera saw only its near
# side (its centre may be that much off), plus 3 cm to keep Nav2 off it
RADIUS = {"cup": 0.12, "bottle": 0.11, "banana": 0.14, "bowl": 0.16}
NAV2_ROOM = 0.10  # m round the footprint where Nav2 starts or stops: room to turn, and one 5 cm costmap cell


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def merge(looks):
    """The points of several looks at one object, as if they were one look (for long_object())."""
    looks = [f for f in looks if f is not None]
    return {"points": np.vstack([f["points"] for f in looks]), "camera": looks[-1]["camera"]}


# --8<-- [start:things]
class Thing:
    """Something on the floor that is not in the map. Positions are in the odom frame, like in tutorial 13."""

    def __init__(self, xy, points):
        self.xy, self.points, self.what, self.state = np.array(xy), points, None, "new"  # new / tidied / left

    def __repr__(self):
        return f"{self.what or 'thing'} at ({self.xy[0]:.2f}, {self.xy[1]:.2f})"

    def describe(self):
        return f"{self!r} ({self.points} points)"


class World:
    """What the mission knows: the map (walls etc.), the things on the floor, and the bowl."""

    def __init__(self, lime):
        self.lime, self.things, self.bowl, self.map = lime, [], None, None
        self.gripper = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        lime.nav.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        lime.nav.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.known = lime.nav.create_publisher(OccupancyGrid, "/known_things", qos)  # for Nav2: see tell_nav2()

    def on_map(self, msg):
        occupied = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width) >= 65
        self.map = {"occupied": occupied, "near": grow(occupied, 3), "res": msg.info.resolution,
                    "origin": np.array([msg.info.origin.position.x, msg.info.origin.position.y]), "info": msg.info}

    def tell_nav2(self):
        """Draw every thing the robot knows about into a second map, /known_things, the size of /map. Nav2 reads
        it as one more costmap layer. The camera marks only the sides of a thing it has seen; this map has the
        whole thing, and forgets a banana as soon as it is picked up."""
        info = self.map["info"]
        grid = np.zeros((info.height, info.width), dtype=np.int8)  # 0 = free: this layer never frees anything
        y, x = np.mgrid[0:info.height, 0:info.width]
        cx, cy = info.origin.position.x + (x + 0.5) * info.resolution, info.origin.position.y + (y + 0.5) * info.resolution
        known = [(t.xy, t.what) for t in self.things if t.state in ("new", "left")]  # YOLO "unknown": still something
        for xy, what in known + ([(self.bowl, "bowl")] if self.bowl is not None else []):
            mx, my, _ = self.to_map(*xy, 0.0)
            grid[np.hypot(cx - mx, cy - my) < RADIUS.get(what, 0.12)] = 100
        msg = OccupancyGrid(info=info)
        msg.header.frame_id = "map"
        msg.header.stamp = self.lime.nav.get_clock().now().to_msg()
        msg.data = grid.ravel().tolist()
        self.known.publish(msg)

    def on_joints(self, msg):
        if "gripper_left_joint" in msg.name:
            self.gripper = msg.position[msg.name.index("gripper_left_joint")]

    def map_from_odom(self):
        """(R, t) of the map <- odom transform, in 2D (AMCL keeps it up to date; in simulation it is almost zero)."""
        tf = self.lime.tf.lookup_transform("map", "odom", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        q, t = tf.transform.rotation, tf.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]), np.array([t.x, t.y]), yaw

    def to_map(self, x, y, yaw):
        """A pose in the odom frame -> the map frame (for Nav2 goals)."""
        R, t, dyaw = self.map_from_odom()
        mx, my = R @ np.array([x, y]) + t
        return float(mx), float(my), wrap(yaw + dyaw)

    def in_map(self, xy, layer="occupied"):
        """For points in the odom frame: does the map have something there? (outside the map counts as yes)"""
        R, t, _ = self.map_from_odom()
        cells = np.floor(((np.atleast_2d(xy) @ R.T + t) - self.map["origin"]) / self.map["res"]).astype(int)
        h, w = self.map[layer].shape
        inside = (cells[:, 0] >= 0) & (cells[:, 0] < w) & (cells[:, 1] >= 0) & (cells[:, 1] < h)
        hit = np.ones(len(cells), dtype=bool)
        hit[inside] = self.map[layer][cells[inside, 1], cells[inside, 0]]
        return hit

    def note(self, xy, points):
        """A thing seen on the floor: a new one, or one we know already (within 25 cm)."""
        if self.bowl is not None and np.linalg.norm(xy - self.bowl) < 0.25:
            return  # the bowl itself, and the bananas in it
        for thing in self.things:
            if np.linalg.norm(thing.xy - xy) < 0.25:
                if thing.state == "new" and points > thing.points:  # keep the best (closest) look
                    thing.xy, thing.points = np.array(xy), points
                return
        self.things.append(Thing(xy, points))

    def free(self, x, y, yaw, ignore=None, margin=0.05):
        """Is the robot's footprint at (x, y, yaw) [odom] clear of the map and of every known thing?"""
        back, front, half = FOOTPRINT
        c, s = math.cos(yaw), math.sin(yaw)
        for thing in self.things:
            if thing is ignore:
                continue
            dx, dy = thing.xy[0] - x, thing.xy[1] - y
            fx, fy = dx * c + dy * s, -dx * s + dy * c  # the thing in the robot's frame
            r = margin + 0.07  # a thing is about 14 cm across
            if back - r < fx < front + r and abs(fy) < half + r:
                return False
        if self.bowl is not None and ignore != "bowl":
            dx, dy = self.bowl[0] - x, self.bowl[1] - y
            fx, fy = dx * c + dy * s, -dx * s + dy * c
            if back - margin - 0.13 < fx < front + margin + 0.13 and abs(fy) < half + margin + 0.13:
                return False
        fx, fy = np.meshgrid(np.arange(back - margin, front + margin + 0.01, 0.05), np.arange(-half - margin, half + margin + 0.01, 0.05))
        pts = np.stack([x + fx.ravel() * c - fy.ravel() * s, y + fx.ravel() * s + fy.ravel() * c], axis=1)
        return not self.in_map(pts).any()


def grow(mask, cells):
    """Grow the occupied cells of a grid by `cells` in every direction."""
    out = mask.copy()
    padded = np.pad(mask, cells)
    h, w = mask.shape
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            if dx * dx + dy * dy <= cells * cells:
                out |= padded[cells + dy:cells + dy + h, cells + dx:cells + dx + w]
    return out
# --8<-- [end:things]


# --8<-- [start:floor]
def floor_scan(lime, world, step=2):
    """One look with the depth camera: every thing on the floor that is not in the map.

    depth image -> 3D points (camera model + TF, like tutorial 13's locate())
      -> drop the floor (lower than 1.2 cm) and what the map has (walls, pillar, crate, divider)
      -> group the rest into things (neighbouring 5 cm cells belong together)
    """
    msg = lime.depth[max(lime.depth)]  # the newest depth image
    depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)[::step, ::step]
    v, u = np.mgrid[0:msg.height:step, 0:msg.width:step]
    ok = np.isfinite(depth) & (depth > 0.3) & (depth < 2.0)  # closer than 30 cm: maybe the robot's own arm
    fx, fy, px, py = lime.K[0, 0], lime.K[1, 1], lime.K[0, 2], lime.K[1, 2]
    z = depth[ok]
    cam = np.stack([(u[ok] - px) * z / fx, (v[ok] - py) * z / fy, z], axis=1)
    tf = lime.tf.lookup_transform("odom", msg.header.frame_id, msg.header.stamp, timeout=rclpy.duration.Duration(seconds=1.0))
    t, q = tf.transform.translation, tf.transform.rotation
    pts = cam @ quat_to_matrix(q.x, q.y, q.z, q.w).T + np.array([t.x, t.y, t.z])
    pts = pts[(pts[:, 2] > 0.012) & (pts[:, 2] < 0.30)]  # not the floor
    pts = pts[~world.in_map(pts[:, :2], layer="near")]  # not the walls, the pillar, ... (or within 15 cm of them)
    cells = {}
    for i, cell in enumerate(map(tuple, np.floor(pts[:, :2] / 0.05).astype(int))):
        cells.setdefault(cell, []).append(i)
    things, todo = [], set(cells)
    while todo:  # flood fill: one group of touching cells = one thing
        group, stack = [], [todo.pop()]
        while stack:
            cx, cy = stack.pop()
            group += cells[(cx, cy)]
            for n in ((cx + a, cy + b) for a in (-1, 0, 1) for b in (-1, 0, 1)):
                if n in todo:
                    todo.remove(n)
                    stack.append(n)
        if len(group) >= 40:  # a banana 1.4 m away gives about 120; fewer: noise at the edge of a wall or pillar
            things.append((pts[group, :2].mean(axis=0), len(group)))
    return things
# --8<-- [end:floor]


# --8<-- [start:look_around]
class LookAround(Action):
    """Turn round once, in six steps of 60 deg (the camera sees 69 deg), and scan the floor at each."""

    def __init__(self, lime, world):
        super().__init__("Look around")
        self.lime, self.world = lime, world

    def run(self):
        _, _, yaw = self.lime.pose()
        for k in range(6):
            if self.stop_requested:
                return False
            self.lime.turn_to(wrap(yaw + k * math.pi / 3))
            self.lime.spin_for(0.6)  # a fresh depth image
            for xy, points in floor_scan(self.lime, self.world):
                self.world.note(xy, points)
        new = [t for t in self.world.things if t.state == "new"]
        self.lime.log.info(f"things to check: {', '.join(t.describe() for t in new) or 'none'}")
        self.world.tell_nav2()
        return True
# --8<-- [end:look_around]


# --8<-- [start:check]
class NextThing(py_trees.behaviour.Behaviour):
    """Choose the nearest thing that has not been checked yet. FAILURE when there is none left."""

    def __init__(self, lime, world):
        super().__init__("Next thing to check")
        self.lime, self.world = lime, world
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="thing", access=Access.WRITE)

    def update(self):
        new = [t for t in self.world.things if t.state == "new"]
        if not new:
            return Status.FAILURE
        x, y, _ = self.lime.pose()
        self.blackboard.thing = min(new, key=lambda t: np.hypot(t.xy[0] - x, t.xy[1] - y))
        self.feedback_message = str(self.blackboard.thing)
        return Status.SUCCESS


class ChooseViewpoint(py_trees.behaviour.Behaviour):
    """A free spot VIEW metres from the thing, facing it, on the side the robot comes from -> blackboard 'goal'."""

    def __init__(self, lime, world):
        super().__init__("Choose where to look from")
        self.lime, self.world = lime, world
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="thing", access=Access.READ)
        self.blackboard.register_key(key="goal", access=Access.WRITE)

    def update(self):
        thing = self.blackboard.thing
        x, y, _ = self.lime.pose()
        here = math.atan2(y - thing.xy[1], x - thing.xy[0])  # the robot's direction, seen from the thing
        for a in sorted((k * math.pi / 12 for k in range(24)), key=lambda a: abs(wrap(a - here))):
            vx, vy = thing.xy + VIEW * np.array([math.cos(a), math.sin(a)])
            if self.world.free(vx, vy, a + math.pi, margin=NAV2_ROOM):
                self.blackboard.goal = self.world.to_map(vx, vy, a + math.pi)
                self.feedback_message = f"({vx:.2f}, {vy:.2f})"
                return Status.SUCCESS
        thing.state = "left"
        self.feedback_message = f"no free spot to look at the {thing} from"
        return Status.FAILURE


class WhatIsIt(Action):
    """Face the thing and ask YOLO: the most confident detection whose 3D points lie on the thing."""

    def __init__(self, lime, world):
        super().__init__("What is it?")
        self.lime, self.world = lime, world
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="thing", access=Access.READ)

    def run(self):
        thing, lime = self.blackboard.thing, self.lime
        x, y, _ = lime.pose()
        away = np.array([x, y]) - thing.xy
        if np.linalg.norm(away) > VIEW + 0.08:  # Nav2 stops within 25 cm of its goal: the last bit by odometry
            lime.drive_to(*(thing.xy + VIEW * away / np.linalg.norm(away)))
            x, y, _ = lime.pose()
        lime.turn_to(math.atan2(thing.xy[1] - y, thing.xy[0] - x))
        lime.detections.clear()
        lime.spin_for(3.0)
        best = None
        for msg, depth_msg in lime.detections:
            for d in msg.detections:
                h = d.results[0].hypothesis
                pts, _camera = lime.points(d, msg.header, depth_msg)
                if pts is not None and np.linalg.norm(np.median(pts[:, :2], axis=0) - thing.xy) < 0.15:
                    if best is None or h.score > best[0]:
                        best = (h.score, h.class_id)
        thing.what = best[1] if best else "unknown"
        lime.log.info(f"YOLO: {thing}" + (f" ({best[0]:.2f})" if best else ""))
        self.world.tell_nav2()
        return True


class IsBanana(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Is it a banana?")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="thing", access=Access.READ)

    def update(self):
        return Status.SUCCESS if self.blackboard.thing.what == "banana" else Status.FAILURE


class LeaveIt(Action):
    """The fallback: it is not a banana, or something went wrong - remember it as checked and carry on.
    (Still holding it? Then put it down first.)"""

    def __init__(self, lime, world):
        super().__init__("Leave it")
        self.lime, self.world = lime, world
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="thing", access=Access.READ)

    def run(self):
        thing = self.blackboard.thing
        if thing.state == "carried":
            self.lime.gripper(GRIPPER_OPEN)
            self.lime.move_arm(HOME, "arm folded")
        thing.state = "left"
        self.lime.log.info(f"leave the {thing}")
        self.world.tell_nav2()
        return True
# --8<-- [end:check]


# --8<-- [start:pick]
class PickUp(Action):
    """Tutorial 13's grasp, from wherever the robot stands: look from two sides, drive up blind, grasp, lift."""

    def __init__(self, lime, world):
        super().__init__("Pick it up")
        self.lime, self.world = lime, world
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="thing", access=Access.READ)

    def run(self):
        lime, world, thing = self.lime, self.world, self.blackboard.thing
        # 1. First look, from where Nav2 brought the robot.
        looks = [self.look()]
        if looks[0] is None:
            return self.fail("YOLO: lost the banana")
        centre, first_axis = long_object(looks[0])
        thing.xy = centre  # YOLO's points are better than the floor scan's
        first = math.atan2(*(looks[0]["camera"] - centre)[::-1])  # where the first look came from
        # 2. Second look, from 45 to 135 deg further round (90 is best). One look sees only the banana's near
        #    side, and seen nearly end-on, its direction comes out 25 deg wrong. Together they see all of it.
        def second_look_cost(a):  # how far from 90 deg round from the first look; None: not 45..135 deg
            round_by = abs(wrap(a - first))
            return abs(round_by - math.pi / 2) if math.pi / 4 < round_by < 3 * math.pi / 4 else None

        second = self.spot(centre, second_look_cost)
        if second is not None and self.go(centre, second):
            looks.append(self.look())
        centre, axis = long_object(merge(looks))
        lime.log.info(f"banana pointing {math.degrees(axis) % 180:.0f} deg from {len(looks)} looks "
                      f"(the first alone: {math.degrees(first_axis) % 180:.0f} deg)")
        # 3. The grasp side: as far off the banana's axis as possible, and at least 24 deg (closer to end-on, the
        #    banana's near end would be under the robot's front; tutorial 13 grasped from 26 deg). The robot must
        #    be able to drive straight up to the grasp position, and Nav2 takes over from there afterwards.
        x, y, _ = lime.pose()
        here = math.atan2(y - centre[1], x - centre[0])
        def grasp_cost(a):  # the more across the banana the better, and the nearer the robot's own side
            across = abs(math.sin(a - axis))  # 1 = from the side, 0 = end-on
            return -across + abs(wrap(a - here)) / math.pi if across > 0.4 else None

        side = self.spot(centre, grasp_cost, grasp=True)
        if side is None:
            return self.fail("no free side to grasp the banana from")
        if abs(wrap(side - here)) > math.radians(5):
            if not self.go(centre, side):
                return self.fail("could not get to the side of the banana")
            looks.append(self.look())
        centre, banana_yaw = long_object(merge(looks))
        lime.log.info(f"YOLO: banana at ({centre[0]:.3f}, {centre[1]:.3f}), pointing {math.degrees(banana_yaw) % 180:.0f} deg "
                      f"from {len(looks)} looks; grasp from {math.degrees(side):.0f} deg")
        # 4. Drive up blind until the banana is GRASP m in front, then tutorial 13's grasp.
        look = side + math.pi
        toward = np.array([math.cos(look), math.sin(look)])
        lime.drive_to(*(centre - GRASP * toward))
        lime.turn_to(look)
        lime.gripper(GRIPPER_OPEN)
        bx, by = lime.to_base(*centre)
        yaw = wrap(banana_yaw - lime.pose()[2])
        above, grasp, lifted = (top_down_ik(bx, by, z, yaw) for z in (0.10, 0.03, 0.15))
        if None in (above, grasp, lifted):
            return self.fail(f"the banana at ({bx:.3f}, {by:.3f}) in the robot frame is out of reach")
        if not (lime.move_arm(above, "above the banana") and lime.move_arm(grasp, "down to the banana")):
            return self.fail("MoveIt: could not reach down")
        lime.gripper(GRIPPER_CLOSED)
        if not (lime.move_arm(lifted, "lift") and lime.move_arm(CARRY, "carry")):
            return self.fail("MoveIt: could not lift")
        lime.spin_for(0.5)
        if world.gripper is None or world.gripper < GRIPPER_CLOSED + 0.004:  # the fingers closed on nothing
            return self.fail(f"missed the banana (gripper at {world.gripper})")
        thing.state = "carried"
        world.tell_nav2()  # it is not on the floor any more
        lime.log.info(f"holding the banana (gripper at {world.gripper * 1000:.1f} mm)")
        # 5. Leave the way you came: back out to BACK_OUT. Nav2 then starts in the open, not right next to
        #    whatever stands near the banana (started that close, it can sweep the robot's back into it).
        lime.drive_to(*(centre - BACK_OUT * toward), backwards=True)
        return True

    def look(self):
        """Face the banana and locate it (tutorial 13)."""
        x, y, _ = self.lime.pose()
        thing = self.blackboard.thing
        self.lime.turn_to(math.atan2(thing.xy[1] - y, thing.xy[0] - x))
        return self.lime.locate("banana", seconds=3.0)

    def spot(self, centre, cost, grasp=False):
        """The direction a (from the banana) of the best free spot SIDE m away; cost(a) = None: not allowed.
        For the grasp, the whole straight line from BACK_OUT to the grasp position must be free too, and Nav2
        needs more room at BACK_OUT (it takes over from there after the grasp)."""
        return best_spot(self.world, self.blackboard.thing, centre, cost, grasp)

    def go(self, centre, a):
        """To the spot SIDE m from the banana in direction a, facing it: a straight drive if the way is free
        (the banana counts as in the way, too), else Nav2."""
        lime, world = self.lime, self.world
        x, y, _ = lime.pose()
        spot = centre + SIDE * np.array([math.cos(a), math.sin(a)])
        heading = math.atan2(spot[1] - y, spot[0] - x)
        if not all(world.free(*p, heading) for p in np.linspace([x, y], spot, 10)):
            if not navigate(lime, world.to_map(*spot, a + math.pi), stop=lambda: self.stop_requested):
                return False
        lime.drive_to(*spot)
        lime.turn_to(a + math.pi)
        return True

    def fail(self, why):
        """Let go, fold the arm and back off, so that the next try (or the next thing) starts clean."""
        self.feedback_message = why
        self.lime.gripper(GRIPPER_OPEN)
        self.lime.move_arm(HOME, "arm folded")
        x, y, yaw = self.lime.pose()
        self.lime.drive_to(x - 0.15 * math.cos(yaw), y - 0.15 * math.sin(yaw), backwards=True)
        return False
# --8<-- [end:pick]


# --8<-- [start:bowl]
class FindBowl(Action):
    """At the start: where is the fruit bowl? (YOLO + depth, tutorial 13)"""

    def __init__(self, lime, world):
        super().__init__("Find the fruit bowl")
        self.lime, self.world = lime, world

    def run(self):
        found = self.lime.locate("bowl", seconds=3.0)
        if found is None:
            return False
        self.world.bowl, _radius = round_object(found)
        self.lime.log.info(f"YOLO: bowl ({found['score']:.2f}) at ({self.world.bowl[0]:.2f}, {self.world.bowl[1]:.2f})")
        self.world.tell_nav2()
        return True


class ChooseBowlSide(py_trees.behaviour.Behaviour):
    """The nearest free spot BOWL_APPROACH m from the bowl with room around it (30 cm to any thing), or else
    the one with the most room -> blackboard 'goal'."""

    def __init__(self, lime, world):
        super().__init__("Choose a side of the bowl")
        self.lime, self.world = lime, world
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="goal", access=Access.WRITE)
        self.blackboard.register_key(key="bowl_side", access=Access.WRITE)

    def update(self):
        bowl, sides = self.world.bowl, []
        x, y, _ = self.lime.pose()
        for a in (k * math.pi / 8 for k in range(16)):
            p = bowl + BOWL_APPROACH * np.array([math.cos(a), math.sin(a)])
            if self.world.free(*p, a + math.pi, ignore="bowl", margin=NAV2_ROOM):
                room = min([np.linalg.norm(t.xy - p) for t in self.world.things if t.state != "carried"] + [9.0])
                sides.append((room, np.hypot(p[0] - x, p[1] - y), a))
        if not sides:
            return Status.FAILURE
        roomy = [s for s in sides if s[0] > 0.30]
        a = min(roomy, key=lambda s: s[1])[2] if roomy else max(sides)[2]
        self.feedback_message = f"{math.degrees(a):.0f} deg"
        self.blackboard.bowl_side = a
        self.blackboard.goal = self.world.to_map(*(bowl + BOWL_APPROACH * np.array([math.cos(a), math.sin(a)])), a + math.pi)
        return Status.SUCCESS


class PutInBowl(Action):
    """Tutorial 13's release: line up with the bowl, banana across the robot, above the bowl's centre, open."""

    def __init__(self, lime, world):
        super().__init__("Put it in the bowl")
        self.lime, self.world = lime, world
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="thing", access=Access.READ)
        self.blackboard.register_key(key="bowl_side", access=Access.READ)

    def run(self):
        lime, world, a = self.lime, self.world, self.blackboard.bowl_side
        out = np.array([math.cos(a), math.sin(a)])  # from the bowl towards the robot
        lime.drive_to(*(world.bowl + BOWL_APPROACH * out))
        lime.turn_to(a + math.pi)
        found = lime.locate("bowl")
        bowl = round_object(found)[0] if found else world.bowl
        lime.drive_to(*(bowl + BOWL_RELEASE * out))
        lime.turn_to(a + math.pi)
        bx, by = lime.to_base(*bowl)
        release = next((q for x in np.arange(bx, 0.10, -0.005) if (q := top_down_ik(x, by, 0.14, math.pi / 2))), None)
        if release is None or not lime.move_arm(release, "above the bowl"):
            return False
        lime.gripper(GRIPPER_OPEN)
        lime.spin_for(1.0)
        lime.move_arm(HOME, "arm folded")
        lime.drive_to(*(bowl + BOWL_APPROACH * out), backwards=True)  # leave the way you came
        self.blackboard.thing.state = "tidied"
        lime.log.info(f"tidied: the {self.blackboard.thing}")
        return True
# --8<-- [end:bowl]


def best_spot(world, thing, centre, cost, grasp=False):
    """PickUp.spot() (a function, so that it can be tried without a robot)."""
    best = None
    for a in (k * math.pi / 12 for k in range(24)):
        c = cost(a)
        if c is None:
            continue
        d = np.array([math.cos(a), math.sin(a)])
        line = np.arange(BACK_OUT, GRASP - 0.001, -0.05) if grasp else [SIDE]
        if grasp and not world.free(*(centre + BACK_OUT * d), a + math.pi, ignore=thing, margin=NAV2_ROOM):
            continue
        # the last drive, from SIDE up to the banana, is slow and centimetre-accurate (odometry): 2 cm margin is enough
        if all(world.free(*(centre + r * d), a + math.pi, ignore=thing, margin=0.02 if grasp and r < SIDE + 0.01 else 0.05) for r in line) and (
            best is None or c < best[0]
        ):
            best = (c, a)
    return None if best is None else best[1]


def navigate(lime, goal, stop=lambda: False):
    """Nav2 to a map-frame pose, from inside an Action."""
    x, y, yaw = goal
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = lime.nav.get_clock().now().to_msg()
    pose.pose.position.x, pose.pose.position.y = x, y
    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    if not lime.nav.goToPose(pose):
        return False
    while not lime.nav.isTaskComplete():
        if stop():
            lime.nav.cancelTask()
            return False
    from nav2_simple_commander.robot_navigator import TaskResult

    return lime.nav.getResult() == TaskResult.SUCCEEDED


# --8<-- [start:tree]
def build_tree(lime, world):
    blackboard = py_trees.blackboard.Client(name="tidy")
    blackboard.register_key(key="lookouts", access=Access.WRITE)
    blackboard.lookouts = LOOKOUTS
    Seq, Sel = py_trees.composites.Sequence, py_trees.composites.Selector

    tidy_banana = Seq("Tidy the banana", memory=True, children=[
        IsBanana(),
        py_trees.decorators.Retry("Up to 3 tries", PickUp(lime, world), num_failures=3),
        ChooseBowlSide(lime, world),
        NavigateTo("Carry it to the bowl", lime, key="goal"),
        PutInBowl(lime, world),
    ])
    check_thing = Seq("Check the next thing", memory=True, children=[
        NextThing(lime, world),  # FAILURE when nothing is left: ends the loop below
        Sel("Tidy it or leave it", memory=True, children=[
            Seq("Go and look", memory=True, children=[
                ChooseViewpoint(lime, world),
                NavigateTo("Drive up to it", lime, key="goal"),
                WhatIsIt(lime, world),
                tidy_banana,
            ]),
            LeaveIt(lime, world),
        ]),
    ])
    at_lookout = Seq("At the lookout", memory=True, children=[
        Sel("Get there", memory=True, children=[
            py_trees.decorators.Retry("Up to 2 tries", NavigateTo("Go to the lookout", lime, key="lookout"), num_failures=2),
            py_trees.behaviours.Success("Skip it"),
        ]),
        LookAround(lime, world),
        py_trees.decorators.FailureIsSuccess("Until nothing is left", py_trees.decorators.Repeat("Repeat", check_thing, num_success=-1)),
    ])
    return Seq("Tidy the room", memory=True, children=[
        FindBowl(lime, world),
        py_trees.decorators.ForEach("Every lookout", at_lookout, source_key="lookouts", target_key="lookout"),
        NavigateTo("Go home", lime, pose=HOME_POSE),
    ])
# --8<-- [end:tree]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--render", metavar="DIR", help="only draw the tree into DIR (tidy.png / .svg / .dot)")
    args, _ros = parser.parse_known_args()
    if args.render:
        py_trees.display.render_dot_tree(build_tree(None, None), target_directory=args.render, name="tidy")
        return

    rclpy.init()
    lime = Lime()
    world = World(lime)
    start = PoseStamped()
    start.header.frame_id = "map"
    start.header.stamp = lime.nav.get_clock().now().to_msg()
    start.pose.orientation.w = 1.0
    lime.nav.setInitialPose(start)  # the robot starts at the map origin
    wait_until_nav2_active(lime.nav)
    if not (lime.arm.wait_for_server(timeout_sec=60.0) and lime.grip.wait_for_server(timeout_sec=60.0)):
        lime.log.error("MoveIt / gripper action servers not available")
        lime.close()
        sys.exit(1)
    lime.add_floor()
    while world.map is None or lime.K is None or not lime.depth:
        lime.spin_for(0.5)
    t0 = time.time()
    status = run_tree(build_tree(lime, world), lime.log)
    tidied = [t for t in world.things if t.state == "tidied"]
    lime.log.info(f"{'room tidy' if status == Status.SUCCESS else 'FAILED'}: {len(tidied)} bananas in the bowl, {time.time() - t0:.0f} s")
    lime.close()
    sys.exit(0 if status == Status.SUCCESS else 1)


from mission import CARRY, GRIPPER_CLOSED, GRIPPER_OPEN, HOME, Lime, long_object, quat_to_matrix, round_object, top_down_ik, wait_until_nav2_active  # noqa: E402

if __name__ == "__main__":
    main()
