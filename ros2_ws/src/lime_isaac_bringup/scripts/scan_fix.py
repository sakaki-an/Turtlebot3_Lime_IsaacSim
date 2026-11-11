#!/usr/bin/env python3
"""Republish Isaac Sim's /scan the way ROS expects it (tutorial 14).

    ros2 run lime_isaac_bringup scan_fix          # /scan -> /scan_fixed

Isaac Sim's RTX LiDAR writes -1.0 for a beam that hit nothing (nothing within 3.5 m). ROS (REP 117) says
"no return" is +inf. Tutorial 11's AMCL and costmaps work with the raw /scan, but slam_toolbox's scan matcher
skips only inf and NaN: it turns every -1 into a point 1 m away, on the opposite side of its own beam.
Those phantom points move with the robot and hold the scan matcher back, so the map drifts.
This node replaces every reading below range_min with +inf and keeps everything else (time stamp too).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


# --8<-- [start:scan_fix]
class ScanFix(Node):
    def __init__(self):
        super().__init__("scan_fix")
        self.pub = self.create_publisher(LaserScan, "/scan_fixed", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg: LaserScan):
        msg.ranges = [r if r >= msg.range_min else math.inf for r in msg.ranges]  # -1 -> inf
        self.pub.publish(msg)
# --8<-- [end:scan_fix]


def main():
    rclpy.init()
    node = ScanFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
